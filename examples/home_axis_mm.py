#!/usr/bin/env python3
"""Axis-level homing in millimetres (endstop or sensorless).

Usage:
  PYTHONPATH=. python3 examples/home_axis_mm.py endstop [/dev/ttyACM0] [node] [dir]
  PYTHONPATH=. python3 examples/home_axis_mm.py stallguard [/dev/ttyACM0] [node] [dir]

  rotation_distance default 40.0 mm/rev (GT2-20T belt). Override with env or edit.
"""

from __future__ import annotations

import sys

from canstepper import Axis, CANStepperBus, EndstopAction, HomingFailed, Param

METHOD = (sys.argv[1] if len(sys.argv) > 1 else "endstop").lower()
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[3]) if len(sys.argv) > 3 else 1
DIRECTION = int(sys.argv[4]) if len(sys.argv) > 4 else -1
ROTATION_DISTANCE = 40.0  # mm per motor revolution


def main() -> None:
    if METHOD not in ("endstop", "stallguard", "sensorless", "set_zero"):
        print("method must be endstop | stallguard | sensorless | set_zero")
        sys.exit(2)

    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        node = bus.node(NODE_ID)
        node.set_run_current(40).set_hold_current(15).set_microsteps(8)
        node.set_closed_loop(True)
        node.set_param(Param.CL_MAX_SPEED, 1200)
        node.set_param(Param.ACCELERATION, 4000)
        node.enable()

        if METHOD == "endstop":
            node.configure_endstop(
                enabled=True, active_high=False, action=EndstopAction.STOP
            )
        if METHOD in ("stallguard", "sensorless"):
            node.set_stall_threshold(60)

        axis = Axis(
            node,
            rotation_distance=ROTATION_DISTANCE,
            require_homing=True,
            name="X",
        )
        print(f"homing {METHOD}  dir={DIRECTION:+d}  rd={ROTATION_DISTANCE} mm/rev")
        try:
            axis.home(
                method=METHOD,
                direction=DIRECTION,
                speed=8.0,  # mm/s
                current_percent=30,
                backoff=1.0,  # mm
                timeout=45.0,
            )
        except HomingFailed as e:
            print("!!", e)
            sys.exit(1)

        print(f"position mm: {axis.get_position():.3f}  homed={axis.homed}")
        # Move 5 mm away from home
        target = 5.0 if DIRECTION < 0 else -5.0
        axis.move_to(target, speed=15.0, blocking=True)
        print(f"after jog: {axis.get_position():.3f} mm")
        node.disable()


if __name__ == "__main__":
    main()
