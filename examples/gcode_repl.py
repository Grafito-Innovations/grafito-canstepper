#!/usr/bin/env python3
"""Interactive G-code REPL for canstepper (CoreXY on nodes A/B).

Usage:
  python3 examples/gcode_repl.py [/dev/ttyACM0] [node_a] [node_b]

Type G-code lines, or:
  help     — list commands
  quit     — exit (disables motors)
"""

from __future__ import annotations

import sys

from canstepper import CANStepperBus, CoreXY, GCodeController, GCodeError, Param

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
A = int(sys.argv[2]) if len(sys.argv) > 2 else 1
B = int(sys.argv[3]) if len(sys.argv) > 3 else 2

HELP = """
Common lines (mm, F in mm/min):
  G28              soft home (zero here)
  G90 / G91        absolute / relative
  G1 X10 Y5 F600   move
  G4 P500          dwell 500 ms
  M114             position
  M17 / M18        enable / disable
  M112             estop all
  M115             firmware info
  M220 S80         speed factor 80%
"""


def prep(node):
    node.enable()
    node.set_run_current(40).set_hold_current(15).set_microsteps(8)
    node.set_closed_loop(True)
    node.set_param(Param.CL_MAX_SPEED, 1500)
    node.set_param(Param.MAX_SPEED, 1500)
    node.set_param(Param.CL_MAX_ACCEL, 5000)
    node.set_param(Param.ACCELERATION, 5000)


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        na, nb = bus.node(A), bus.node(B)
        prep(na)
        prep(nb)
        xy = CoreXY(na, nb, rotation_distance=40.0, max_speed=50.0)
        g = GCodeController.from_corexy(xy, bus=bus)
        g.run("G21")
        g.run("G90")
        g.run("G28")
        print(HELP)
        print("G-code REPL — empty line or 'quit' to exit\n")
        try:
            while True:
                try:
                    line = input("gcode> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if not line or line.lower() in ("quit", "exit", "q"):
                    break
                if line.lower() in ("help", "?"):
                    print(HELP)
                    continue
                try:
                    print(g.run(line))
                except GCodeError as e:
                    print("!!", e)
                except Exception as e:
                    print("!!", type(e).__name__, e)
        finally:
            try:
                g.run("M18")
            except Exception:
                pass
            try:
                bus.estop_all()
            except Exception:
                pass


if __name__ == "__main__":
    main()
