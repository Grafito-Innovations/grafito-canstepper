"""Minimal example: configure one node and move it.

Connect the USB port of any board on the chain and adjust PORT.
"""

import time

from canstepper import CANStepperBus

PORT = "/dev/ttyACM0"

with CANStepperBus.serial(PORT) as bus:
    print("Nodes on the bus:", bus.discover())

    node = bus.node(1)
    node.set_run_current(40).set_hold_current(15).set_microsteps(16)
    node.set_max_speed(1000.0).set_acceleration(720.0)
    node.enable()

    node.move_to(180.0, blocking=True)
    print("position:", node.get_position())

    node.run(360.0)          # continuous rotation, 90 deg/s
    time.sleep(2.0)
    node.stop()

    node.move_to(0.0, blocking=True)
    print("done at:", node.get_position())
