#!/usr/bin/env python3
"""Sensorless (StallGuard) homing — no limit switch required.

Uses the TMC2209 StallGuard load detection on DIAG (GPIO 0). The axis
runs toward a hard mechanical stop until DIAG fires; firmware zeros and
backs off.

Tuning knobs:
  - stall_threshold (SGTHRS 0–255): HIGHER = more sensitive
  - homing current %: use 25–40 so the crash is gentle
  - speed: StallGuard needs motion; ~15–40 deg/s is a good start on NEMA17

Usage:
  PYTHONPATH=. python3 examples/home_sensorless.py [/dev/ttyACM0] [node_id] [dir] [threshold]

  dir: -1 (default) or +1 — travel direction toward the hard stop
  threshold: StallGuard SGTHRS (default 60)

Example:
  PYTHONPATH=. python3 examples/home_sensorless.py /dev/ttyACM0 1 -1 60
"""

from __future__ import annotations

import sys
import time

from canstepper import CANStepperBus, HomingFailed, Param

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
DIRECTION = int(sys.argv[3]) if len(sys.argv) > 3 else -1
THRESHOLD = int(sys.argv[4]) if len(sys.argv) > 4 else 60


def prep(node) -> None:
    node.set_run_current(40).set_hold_current(15).set_microsteps(8)
    node.set_closed_loop(True)
    node.set_param(Param.CL_MAX_SPEED, 1200)
    node.set_param(Param.MAX_SPEED, 1200)
    node.set_param(Param.CL_MAX_ACCEL, 4000)
    node.set_param(Param.ACCELERATION, 4000)
    node.enable()


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        node = bus.node(NODE_ID)
        prep(node)

        # method aliases: "stallguard" or "sensorless"
        node.set_stall_threshold(THRESHOLD)
        node.configure_homing(
            current_percent=30,
            backoff_deg=8.0,
            timeout_ms=30000,
        )

        try:
            sg = node.get_stallguard()
            print(f"stallguard live reading (idle): {sg}")
        except Exception as e:
            print(f"stallguard read skipped: {e}")

        print(
            f"sensorless home  dir={DIRECTION:+d}  "
            f"threshold={THRESHOLD}  speed=25 deg/s …"
        )
        try:
            node.home(
                method="stallguard",
                direction=DIRECTION,
                speed_deg_s=25.0,
                timeout=45.0,
            )
        except HomingFailed as e:
            print("!! homing failed:", e)
            print(
                "tips: raise threshold if never triggers; lower if false trips; "
                "increase speed slightly; ensure a hard stop in that direction"
            )
            sys.exit(1)

        st = node.get_status()
        print(
            f"after home: pos={node.get_position():.2f}°  "
            f"homed={st.homed}  stall_latched={st.stall_latched}"
        )

        away = 45.0 if DIRECTION < 0 else -45.0
        print(f"move_to {away}° (away from hard stop) …")
        node.move_to(away, blocking=True)
        print(f"position now {node.get_position():.2f}°")
        time.sleep(0.3)
        node.disable()
        print("done")


if __name__ == "__main__":
    main()
