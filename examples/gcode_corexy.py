#!/usr/bin/env python3
"""Run Klipper-style G-code against a two-node CoreXY (closed-loop).

Usage:
  python3 examples/gcode_corexy.py [/dev/ttyACM0] [node_a] [node_b]

Not a port of Klipper — same common G-code vocabulary mapped onto canstepper.
"""

from __future__ import annotations

import sys

from canstepper import CANStepperBus, CoreXY, GCodeController, Param

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
A = int(sys.argv[2]) if len(sys.argv) > 2 else 1
B = int(sys.argv[3]) if len(sys.argv) > 3 else 2


def prep(node):
    node.enable()
    node.set_run_current(45).set_hold_current(15).set_microsteps(8)
    node.set_closed_loop(True)
    node.set_param(Param.CL_MAX_SPEED, 1500)
    node.set_param(Param.MAX_SPEED, 1500)
    node.set_param(Param.CL_MAX_ACCEL, 5000)
    node.set_param(Param.ACCELERATION, 5000)


PROGRAM = """
G21 ; mm
G90 ; absolute
G28 ; soft home (zero here)
M114
G1 X15 Y0 F600
M114
G1 X15 Y12
G1 X0 Y12
G1 X0 Y0
G1 X10 Y6 F480
G1 X0 Y0
M114
M18 ; motors off
"""


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        na, nb = bus.node(A), bus.node(B)
        prep(na)
        prep(nb)
        xy = CoreXY(na, nb, rotation_distance=40.0, max_speed=40.0)
        g = GCodeController.from_corexy(xy, bus=bus, default_feedrate_mm_min=600)
        for line in PROGRAM.strip().splitlines():
            line = line.strip()
            if not line:
                continue
            print(">>", line)
            print("  ", g.run(line))


if __name__ == "__main__":
    main()
