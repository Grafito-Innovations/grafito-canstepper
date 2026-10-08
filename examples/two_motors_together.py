#!/usr/bin/env python3
"""Move CANStepper nodes 1 and 2 together using on-bus following.

Node 1 is the leader and Node 2 follows its encoder position without the host
having to continually issue matching commands.  Start with a small move and no
mechanical load.  Ctrl-C or any exception sends a bus-wide emergency stop.
"""

from __future__ import annotations

import statistics
import sys
import time
from pathlib import Path

# Allow this repository example to run directly even before the package has
# been installed (``python3 can_stepper/examples/two_motors_together.py``).
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import CANStepperBus


PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
LEADER_ID = 1
FOLLOWER_ID = 2
MOVE_DEG = 90.0
STATIONARY_SAMPLES = 20
MAX_STATIONARY_SPAN_DEG = 0.5
FOLLOW_TOLERANCE_DEG = 1.0
FOLLOW_SETTLE_SAMPLES = 5


def stationary_span(node) -> float:
    values = []
    for _ in range(STATIONARY_SAMPLES):
        values.append(node.get_position())
        time.sleep(0.03)
    span = max(values) - min(values)
    print(
        f"node {node.node_id}: encoder={statistics.mean(values):.3f} deg, "
        f"stationary span={span:.3f} deg"
    )
    return span


def wait_follower_close(leader, follower, timeout: float = 10.0):
    """Wait for real encoder agreement, independent of cached PID telemetry."""
    deadline = time.monotonic() + timeout
    consecutive = 0
    last = (float("nan"), float("nan"))
    while time.monotonic() < deadline:
        last = (leader.get_position(), follower.get_position())
        if abs(last[0] - last[1]) <= FOLLOW_TOLERANCE_DEG:
            consecutive += 1
            if consecutive >= FOLLOW_SETTLE_SAMPLES:
                return last
        else:
            consecutive = 0
        time.sleep(0.05)
    raise RuntimeError(
        f"follower did not converge within {timeout:.1f}s: "
        f"node 1={last[0]:.3f} deg, node 2={last[1]:.3f} deg, "
        f"error={last[1] - last[0]:+.3f} deg"
    )


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        leader = bus.node(LEADER_ID)
        follower = bus.node(FOLLOWER_ID)
        try:
            found = bus.discover(timeout=2.0)
            if LEADER_ID not in found or FOLLOWER_ID not in found:
                raise RuntimeError(
                    f"expected nodes 1 and 2, discovered {found}; check unique IDs, "
                    "CAN wiring, termination, and that both boards run GCSP v1"
                )

            # Check feedback before energizing either motor. A stationary encoder
            # should not jump; closed-loop movement is unsafe until this passes.
            bus.estop_all()
            spans = (stationary_span(leader), stationary_span(follower))
            if max(spans) > MAX_STATIONARY_SPAN_DEG:
                raise RuntimeError(
                    "encoder is unstable while stationary; reflash the corrected "
                    "firmware, then check magnet centering/gap and SSI wiring"
                )

            for node in (leader, follower):
                node.set_run_current(30)
                node.set_hold_current(15)
                node.set_steps_per_rev(200)
                node.set_microsteps(16)
                node.set_max_speed(30.0)
                node.set_acceleration(180.0)
                node.set_closed_loop(True, max_speed=30.0, max_accel=180.0)
                node.set_report_rates(fast_hz=30, slow_hz=2)
                node.enable()

            leader.set_zero()
            follower.set_zero()
            follower.follow(leader, ratio=1.0, invert=False,
                            encoder_corrected=True)
            time.sleep(0.5)
            status = follower.get_follow_status()
            if not status.enabled or not status.synced:
                raise RuntimeError(f"follower did not synchronize: {status}")

            print(f"Moving both motors +{MOVE_DEG:.1f} degrees...")
            leader.move_to(MOVE_DEG, blocking=True, timeout=15.0)
            a, b = wait_follower_close(leader, follower)
            print(f"positions: node 1={a:.3f} deg, node 2={b:.3f} deg")

            print("Returning both motors to zero...")
            leader.move_to(0.0, blocking=True, timeout=15.0)
            a, b = wait_follower_close(leader, follower)
            print(
                f"positions: node 1={a:.3f} deg, node 2={b:.3f} deg"
            )
        finally:
            # Break the coupling before disabling, then stop both nodes even when
            # a telemetry request, move, or Ctrl-C interrupts the test.
            try:
                follower.unfollow()
            except Exception:
                pass
            bus.estop_all()


if __name__ == "__main__":
    main()
