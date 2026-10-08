"""CoreXY: two motors, straight-line cartesian moves."""

from canstepper import CANStepperBus, CoreXY

PORT = "/dev/ttyACM0"

with CANStepperBus.serial(PORT) as bus:
    xy = CoreXY(
        bus.node(4),
        bus.node(5),
        rotation_distance=40.0,   # GT2 belt, 20-tooth pulley
        max_speed=80.0,           # mm/s along the path
    )
    xy.enable()
    xy.set_zero()                 # current position becomes (0, 0)

    for point in [(50, 0), (50, 30), (0, 30), (0, 0)]:
        duration = xy.move_to(*point, speed=60.0)
        print(f"-> {point}  ({duration:.2f}s)   now at {xy.get_position()}")
