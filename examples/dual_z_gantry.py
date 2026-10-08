#!/usr/bin/env python3
"""Safe DualMotorAxis bench test using CANStepper Nodes 1 and 2.

This is configured for the two currently free motors:
  * Node 1 is the leader connected to USB.
  * Node 2 is the CAN follower.
  * Both motors rotate in the same direction.
  * Axis units are degrees; no mechanical homing is attempted.

For a real dual-Z gantry, change ROTATION_DISTANCE to the leadscrew travel per
revolution (for example 8.0 mm), set INVERT_SECONDARY for mirrored mounting,
install/tune the endstop or StallGuard, and only then add a homing operation.

Usage:
    python3 dual_z_gantry.py [PORT] [TARGET_DEGREES]
"""

from __future__ import annotations

import statistics
import sys
import time
from pathlib import Path

# Allow direct execution from examples/ without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import CANStepperBus, DualMotorAxis  # noqa: E402


PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
TARGET = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

PRIMARY_NODE = 1
SECONDARY_NODE = 2
ROTATION_DISTANCE = 360.0  # degree units for the current free-motor test
INVERT_SECONDARY = False   # current motors turn the same physical direction
FOLLOW_RATIO = 1.0

MAX_ENCODER_SPAN = 0.5
MAX_COUPLING_ERROR = 1.0


def encoder_span(node, samples: int = 20) -> float:
    values = []
    for _ in range(samples):
        values.append(node.get_position())
        time.sleep(0.03)
    span = max(values) - min(values)
    print(
        f"Node {node.node_id}: encoder mean={statistics.mean(values):.3f}°, "
        f"stationary span={span:.3f}°"
    )
    return span


def configure_motor(node) -> None:
    node.set_run_current(30)
    node.set_hold_current(15)
    node.set_steps_per_rev(200)
    node.set_microsteps(16)
    node.set_max_speed(30.0)
    node.set_acceleration(180.0)
    node.set_closed_loop(True, max_speed=30.0, max_accel=180.0)
    node.set_report_rates(fast_hz=30, slow_hz=2)


def wait_for_pair(primary, secondary, timeout: float = 10.0):
    """Wait for five consecutive encoder readings within the coupling limit."""
    deadline = time.monotonic() + timeout
    consecutive = 0
    p1 = p2 = float("nan")
    while time.monotonic() < deadline:
        p1 = primary.get_position()
        p2 = secondary.get_position()
        if abs(p2 - p1) <= MAX_COUPLING_ERROR:
            consecutive += 1
            if consecutive >= 5:
                return p1, p2
        else:
            consecutive = 0
        time.sleep(0.05)
    raise RuntimeError(
        f"dual-Z pair did not converge: Node 1={p1:.3f}°, "
        f"Node 2={p2:.3f}°, error={p2 - p1:+.3f}°"
    )


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        primary = bus.node(PRIMARY_NODE)
        secondary = bus.node(SECONDARY_NODE)
        z = DualMotorAxis(
            primary,
            secondary,
            rotation_distance=ROTATION_DISTANCE,
            invert_secondary=INVERT_SECONDARY,
            ratio=FOLLOW_RATIO,
            encoder_corrected=True,
            auto_couple=False,
            max_speed=30.0,
            name="z_bench_test",
        )

        try:
            found = bus.discover(timeout=2.0)
            print(f"Discovered nodes: {found}")
            if PRIMARY_NODE not in found or SECONDARY_NODE not in found:
                raise RuntimeError(
                    f"expected Nodes 1 and 2, discovered {found}; "
                    "check CAN power, wiring, termination, and node IDs"
                )

            # Start released and reject unstable feedback before applying torque.
            bus.estop_all()
            spans = (encoder_span(primary), encoder_span(secondary))
            if max(spans) > MAX_ENCODER_SPAN:
                raise RuntimeError(
                    "encoder unstable while stationary; check magnet alignment"
                )

            z.configure_both(configure_motor)
            primary.set_zero()
            secondary.set_zero()

            z.enable()
            time.sleep(0.6)
            primary.get_status()
            secondary.get_status()

            z.couple()
            time.sleep(0.5)
            follow = secondary.get_follow_status()
            print(
                f"Follower: enabled={follow.enabled}, synced={follow.synced}, "
                f"leader={follow.leader_id}, ratio={follow.ratio:.3f}"
            )
            if not follow.enabled or not follow.synced:
                raise RuntimeError(f"Node 2 failed to couple: {follow}")

            print(f"Moving dual-Z bench axis to {TARGET:.1f}°...")
            z.move_to(TARGET, speed=30.0, blocking=True, timeout=20.0)
            p1, p2 = wait_for_pair(primary, secondary)
            print(
                f"At target: Node 1={p1:.3f}°, Node 2={p2:.3f}°, "
                f"error={p2 - p1:+.3f}°"
            )

            print("Returning dual-Z bench axis to 0°...")
            z.move_to(0.0, speed=30.0, blocking=True, timeout=20.0)
            p1, p2 = wait_for_pair(primary, secondary)
            print(
                f"At zero: Node 1={p1:.3f}°, Node 2={p2:.3f}°, "
                f"error={p2 - p1:+.3f}°"
            )

            print("Resynchronizing the captured leader/follower offsets...")
            z.resync()
            time.sleep(0.3)
        finally:
            try:
                z.decouple()
            except Exception:
                pass
            bus.estop_all()
            print("Both motors emergency-stopped and disabled.")


if __name__ == "__main__":
    main()
