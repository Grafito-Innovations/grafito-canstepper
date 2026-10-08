import math

import pytest

from canstepper import (
    Axis,
    Cartesian,
    CoreXY,
    DualMotorAxis,
    IndependentDualAxis,
    MotionGroup,
)
from canstepper.exceptions import EncoderGateTimeout
from canstepper.kinematics import (
    eval_trapezoid,
    plan_trapezoid,
    profile_for_time,
    trapezoid_time,
)


def test_trapezoid_time_cruise():
    # d=100, v=50, a=100: cruise reached -> t = d/v + v/a = 2 + 0.5
    assert trapezoid_time(100, 50, 100) == pytest.approx(2.5)


def test_trapezoid_time_triangular():
    # too short to cruise: t = 2*sqrt(d/a)
    assert trapezoid_time(10, 1000, 100) == pytest.approx(2 * math.sqrt(0.1))


def test_plan_trapezoid_matches_duration():
    v_peak, t_acc, t_cruise, t_dec, t_tot = plan_trapezoid(100, 50, 100)
    assert v_peak == pytest.approx(50)
    assert t_cruise > 0
    assert t_tot == pytest.approx(trapezoid_time(100, 50, 100))
    # short move → triangle
    v_peak, t_acc, t_cruise, t_dec, t_tot = plan_trapezoid(10, 1000, 100)
    assert t_cruise == pytest.approx(0.0)
    assert t_tot == pytest.approx(trapezoid_time(10, 1000, 100))


def test_eval_trapezoid_endpoints_and_cruise():
    d, v, a = 100.0, 50.0, 100.0
    s0, v0 = eval_trapezoid(d, v, a, 0.0)
    assert s0 == 0.0 and v0 == 0.0
    _, _, _, _, t_tot = plan_trapezoid(d, v, a)
    s1, v1 = eval_trapezoid(d, v, a, t_tot)
    assert s1 == pytest.approx(d)
    assert v1 == pytest.approx(0.0)
    # mid-cruise
    t_mid = plan_trapezoid(d, v, a)[1] + plan_trapezoid(d, v, a)[2] / 2
    s_m, v_m = eval_trapezoid(d, v, a, t_mid)
    assert v_m == pytest.approx(v)
    assert 0 < s_m < d
    # signed distance
    s_neg, v_neg = eval_trapezoid(-d, v, a, t_mid)
    assert s_neg == pytest.approx(-s_m)
    assert v_neg == pytest.approx(-v_m)


def test_profile_for_time_roundtrips():
    for d, v, a in [(100, 50, 100), (10, 1000, 100), (3.5, 7.0, 20.0)]:
        t = trapezoid_time(d, v, a)
        v2, a2 = profile_for_time(d, t, a)
        assert trapezoid_time(d, v2, a2) == pytest.approx(t, rel=1e-6)


def test_profile_slower_axis_gets_lower_speed():
    # Two distances, same accel, same deadline: shorter distance -> lower speed.
    t = trapezoid_time(100, 50, 100)
    v_short, _ = profile_for_time(40, t, 100)
    assert v_short < 50


def test_motion_group_coordinated_move(bus):
    ax1 = Axis(bus.node(1), rotation_distance=10.0, name="x")
    ax2 = Axis(bus.node(2), rotation_distance=10.0, name="y")
    group = MotionGroup([ax1, ax2])
    duration = group.move_to({"x": 20.0, "y": 5.0}, speeds={"x": 10.0, "y": 10.0})
    assert duration > 0
    assert ax1.get_position() == pytest.approx(20.0)
    assert ax2.get_position() == pytest.approx(5.0)


def test_dual_motor_axis_couples_secondary(bus):
    z = DualMotorAxis(
        bus.node(1), bus.node(2), rotation_distance=8.0, name="z"
    )
    bus.node(1).get_position()      # leader POSITION frame -> follower syncs
    z.move_to(4.0, blocking=True)   # half a motor revolution
    assert bus.node(1).get_position() == pytest.approx(180.0)
    assert bus.node(2).get_position() == pytest.approx(180.0)


def test_dual_motor_axis_inverted_secondary(bus):
    z = DualMotorAxis(
        bus.node(1), bus.node(2), rotation_distance=8.0,
        invert_secondary=True, name="z",
    )
    bus.node(1).get_position()
    z.move_to(2.0, blocking=True)
    assert bus.node(1).get_position() == pytest.approx(90.0)
    assert bus.node(2).get_position() == pytest.approx(-90.0)


def test_independent_dual_units(bus):
    # SFU1204: 4 mm/rev → 10 mm = 900°
    dual = IndependentDualAxis(
        bus.node(1), bus.node(2), rotation_distance=4.0, name="frame"
    )
    assert dual.units_to_deg(4.0) == pytest.approx(360.0)
    assert dual.units_to_deg(10.0) == pytest.approx(900.0)
    assert dual.units_to_deg(20.0) == pytest.approx(1800.0)
    assert dual.deg_to_units(900.0) == pytest.approx(10.0)


def test_independent_dual_move_and_gate(bus):
    dual = IndependentDualAxis(
        bus.node(1),
        bus.node(2),
        rotation_distance=4.0,
        invert_a=False,
        invert_b=False,
        name="frame",
        tol_mm=0.5,
        settle_s=0.05,
        gate_timeout_s=5.0,
    )
    dual.configure_open_loop(speed_mm_s=40.0)
    dual.enable().set_zero()
    pa, pb = dual.move_to(10.0, wait_encoders=True)
    assert pa == pytest.approx(10.0, abs=0.2)
    assert pb == pytest.approx(10.0, abs=0.2)
    pa, pb = dual.move_to(0.0, wait_encoders=True)
    assert pa == pytest.approx(0.0, abs=0.2)
    assert pb == pytest.approx(0.0, abs=0.2)


def test_independent_dual_oscillate(bus):
    dual = IndependentDualAxis(
        bus.node(1),
        bus.node(2),
        rotation_distance=4.0,
        invert_a=False,
        invert_b=False,
        tol_mm=0.5,
        settle_s=0.05,
        gate_timeout_s=5.0,
    )
    dual.configure_open_loop(speed_mm_s=50.0)
    dual.enable().set_zero()
    n = dual.oscillate(10.0, cycles=2, speed_mm_s=50.0)
    assert n == 2
    pa, pb = dual.get_positions()
    assert pa == pytest.approx(0.0, abs=0.3)
    assert pb == pytest.approx(0.0, abs=0.3)


def test_encoder_gate_timeout_attrs():
    err = EncoderGateTimeout(
        "test", target_mm=10.0, positions_mm=(9.0, 10.0), timeout=1.0
    )
    assert err.target_mm == 10.0
    assert err.positions_mm == (9.0, 10.0)
    assert err.timeout == 1.0


def test_cartesian_identity_math():
    assert Cartesian.cartesian_to_motors(10.0, 4.0, 2.0) == (10.0, 4.0, 2.0)
    assert Cartesian.motors_to_cartesian(10.0, 4.0, 2.0) == (10.0, 4.0, 2.0)


def test_cartesian_move_xy(bus):
    cart = Cartesian.from_nodes(
        bus.node(1), bus.node(2), rotation_distance=40.0, max_speed=50.0
    )
    cart.move_to(10.0, 5.0, speed=50.0)
    x, y, z = cart.get_position()
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(5.0)
    assert z == pytest.approx(0.0)
    # 1:1 mapping — motors match workspace
    assert cart.x.get_position() == pytest.approx(10.0)
    assert cart.y.get_position() == pytest.approx(5.0)


def test_cartesian_move_xyz(bus):
    cart = Cartesian.from_nodes(
        bus.node(1),
        bus.node(2),
        bus.node(3),
        rotation_distance=40.0,
        rotation_distance_z=8.0,
        max_speed=40.0,
    )
    cart.move_to(6.0, 3.0, 2.0, speed=40.0)
    x, y, z = cart.get_position()
    assert x == pytest.approx(6.0)
    assert y == pytest.approx(3.0)
    assert z == pytest.approx(2.0)


def test_cartesian_set_zero(bus):
    cart = Cartesian.from_nodes(bus.node(1), bus.node(2), rotation_distance=40.0)
    cart.move_to(5.0, 5.0, speed=50.0)
    cart.set_zero()
    assert cart.get_position()[:2] == (pytest.approx(0.0), pytest.approx(0.0))


def test_corexy_math():
    a, b = CoreXY.cartesian_to_motors(10.0, 4.0)
    assert (a, b) == (14.0, 6.0)
    assert CoreXY.motors_to_cartesian(a, b) == (10.0, 4.0)


def test_corexy_move(bus):
    xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
    xy.move_to(10.0, 5.0, speed=50.0)
    x, y = xy.get_position()
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(5.0)
    # motor A carried x+y, motor B x-y
    assert xy.axis_a.get_position() == pytest.approx(15.0)
    assert xy.axis_b.get_position() == pytest.approx(5.0)


def test_corexy_pure_y_move(bus):
    xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
    xy.move_to(0.0, 8.0, speed=50.0)
    assert xy.get_position() == (pytest.approx(0.0), pytest.approx(8.0))
    assert xy.axis_a.get_position() == pytest.approx(8.0)
    assert xy.axis_b.get_position() == pytest.approx(-8.0)
