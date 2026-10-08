import pytest

from canstepper import Axis, LimitViolation, NotHomed


def test_unit_conversion(bus):
    axis = Axis(bus.node(1), rotation_distance=8.0)
    assert axis.units_to_deg(8.0) == pytest.approx(360.0)
    assert axis.deg_to_units(360.0) == pytest.approx(8.0)


def test_gear_ratio(bus):
    axis = Axis(bus.node(1), rotation_distance=360.0, gear_ratio=5.0)
    # 5:1 gearbox: one output unit (deg here) needs 5 motor degrees
    assert axis.units_to_deg(360.0) == pytest.approx(1800.0)


def test_move_in_units(bus):
    axis = Axis(bus.node(1), rotation_distance=8.0)
    axis.move_to(4.0, blocking=True)          # half a revolution
    assert bus.node(1).get_position() == pytest.approx(180.0)
    assert axis.get_position() == pytest.approx(4.0)


def test_soft_limits(bus):
    axis = Axis(bus.node(1), rotation_distance=8.0, min_pos=0.0, max_pos=100.0)
    with pytest.raises(LimitViolation):
        axis.move_to(150.0)
    with pytest.raises(LimitViolation):
        axis.move_to(-1.0)
    axis.move_to(50.0, blocking=True)


def test_axis_speed_limit(bus):
    axis = Axis(bus.node(1), rotation_distance=8.0, max_speed=20.0)
    with pytest.raises(LimitViolation):
        axis.move_to(10.0, speed=25.0)
    with pytest.raises(LimitViolation):
        axis.run(-30.0)


def test_require_homing(bus):
    axis = Axis(bus.node(1), rotation_distance=8.0, require_homing=True)
    with pytest.raises(NotHomed):
        axis.move_to(5.0)
    axis.home(method="endstop", direction=-1, speed=10.0, backoff=1.0)
    # backoff of 1 unit on an 8 mm/rev axis = 45 deg, away from the switch
    assert bus.node(1).get_position() == pytest.approx(45.0)
    assert axis.get_position() == pytest.approx(1.0)
    axis.move_to(5.0, blocking=True)
    assert axis.get_position() == pytest.approx(5.0)
