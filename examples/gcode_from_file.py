#!/usr/bin/env python3
"""Run a .gcode / .nc file through GCodeController (CoreXY).

Usage:
  python3 examples/gcode_from_file.py path/to/file.gcode [/dev/ttyACM0] [a] [b]

Example:
  python3 examples/gcode_from_file.py examples/gcode/square.gcode
"""

from __future__ import annotations

import sys
from pathlib import Path

from canstepper import CANStepperBus, CoreXY, GCodeController, GCodeError, Param

GCODE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "gcode" / "square.gcode"
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"
A = int(sys.argv[3]) if len(sys.argv) > 3 else 1
B = int(sys.argv[4]) if len(sys.argv) > 4 else 2


def prep(node):
    node.enable()
    node.set_run_current(40).set_hold_current(15).set_microsteps(8)
    node.set_closed_loop(True)
    node.set_param(Param.CL_MAX_SPEED, 1500)
    node.set_param(Param.MAX_SPEED, 1500)
    node.set_param(Param.CL_MAX_ACCEL, 5000)
    node.set_param(Param.ACCELERATION, 5000)


def main() -> None:
    if not GCODE.is_file():
        print(f"!! file not found: {GCODE}")
        sys.exit(2)

    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        print("running", GCODE)
        na, nb = bus.node(A), bus.node(B)
        prep(na)
        prep(nb)
        xy = CoreXY(na, nb, rotation_distance=40.0, max_speed=40.0)
        g = GCodeController.from_corexy(xy, bus=bus)

        n = 0
        try:
            with GCODE.open("r", encoding="utf-8", errors="replace") as fh:
                for raw in fh:
                    line = raw.strip()
                    if not line:
                        continue
                    n += 1
                    print(f"{n:4d} >> {line}")
                    try:
                        print(f"      {g.run(line)}")
                    except GCodeError as e:
                        print(f"      !! {e}")
                        sys.exit(1)
        finally:
            try:
                g.run("M18")
            except Exception:
                pass


if __name__ == "__main__":
    main()
