"""Groups and the three e-stop scopes.

- node.estop():      one node
- group.estop():     a set of nodes, one frame each
- bus.estop_all():   every node with a single broadcast frame (fastest)
"""

import time

from canstepper import CANStepperBus

PORT = "/dev/ttyACM0"

with CANStepperBus.serial(PORT) as bus:
    feeders = bus.group([6, 7, 8])
    feeders.enable()
    feeders.set_param("run_current", 35)
    feeders.run(720.0)            # all feeders spinning at 2 rev/s

    time.sleep(3.0)
    feeders.estop()               # stop just the feeder group

    # Something went badly wrong somewhere on the machine:
    bus.estop_all()               # one broadcast frame, every node halts

    # Recovery is explicit:
    feeders.enable()
