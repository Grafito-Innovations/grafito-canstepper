#!/usr/bin/env python3
"""G-code on a Klipper-style cartesian mechanism (independent X/Y[/Z]).

Uses :class:`Cartesian` kinematics + :meth:`GCodeController.from_cartesian`
so workspace letters map 1:1 onto steppers (unlike CoreXY belt math).

Usage:
  python3 examples/gcode_cartesian.py [/dev/ttyACM0] [x_id] [y_id] [z_id]

Omit z_id (or pass 0) to run XY only. Closed-loop; soft G28 zeros at current pose.

Examples:
  PYTHONPATH=. python3 examples/gcode_cartesian.py /dev/ttyACM0 1 2
  PYTHONPATH=. python3 examples/gcode_cartesian.py /dev/ttyACM0 1 2 3
"""

from __future__ import annotations

import sys

from canstepper import CANStepperBus, Cartesian, GCodeController, Param

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
X_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
Y_ID = int(sys.argv[3]) if len(sys.argv) > 3 else 2
Z_ID = int(sys.argv[4]) if len(sys.argv) > 4 else 0


def prep(node):
    node.enable()
    node.set_run_current(40).set_hold_current(15).set_microsteps(8)
    node.set_closed_loop(True)
    node.set_param(Param.CL_MAX_SPEED, 1200)
    node.set_param(Param.MAX_SPEED, 1200)
    node.set_param(Param.CL_MAX_ACCEL, 4000)
    node.set_param(Param.ACCELERATION, 4000)


PROGRAM_XY = """
G21
G90
M17
G28
M114
G1 X10 F480
G1 Y8
G1 X0
G1 Y0
G1 X5 Y5 F360
G1 X0 Y0
M114
M18
"""

PROGRAM_XYZ = """
G21
G90
M17
G28
M114
G1 X10 F480
G1 Y8
G1 Z2 F300
G1 X0
G1 Y0
G1 Z0
G1 X5 Y5 Z1 F360
G1 X0 Y0 Z0
M114
M18
"""


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        nx, ny = bus.node(X_ID), bus.node(Y_ID)
        prep(nx)
        prep(ny)
        nz = None
        if Z_ID:
            nz = bus.node(Z_ID)
            prep(nz)

        cart = Cartesian.from_nodes(
            nx,
            ny,
            nz,
            rotation_distance=40.0,  # X/Y belt mm/rev
            rotation_distance_z=8.0,  # Z leadscrew mm/rev
            max_speed=40.0,
            name="machine",
        )
        g = GCodeController.from_cartesian(cart, bus=bus, default_feedrate_mm_min=480)
        program = PROGRAM_XYZ if Z_ID else PROGRAM_XY
        print("kinematics", cart)
        for line in program.strip().splitlines():
            line = line.strip()
            if not line:
                continue
            print(">>", line)
            print("  ", g.run(line))


if __name__ == "__main__":
    main()
