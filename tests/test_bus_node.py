import pytest

from canstepper import (
    EStopActive,
    Event,
    LimitViolation,
    Param,
    ParamRejected,
    RequestTimeout,
)


def test_discover_finds_all_nodes(bus):
    found = bus.discover(timeout=0.1)
    assert sorted(found) == [1, 2, 3]
    assert all(v == "1.2" for v in found.values())


def test_ping_and_status(bus):
    status = bus.node(1).ping()
    assert status.enabled
    assert not status.estopped
    assert status.firmware == "1.2"
    assert status.protocol_version == 1


def test_param_set_get_roundtrip(bus):
    node = bus.node(1)
    node.set_param(Param.MAX_SPEED, 1234.5)
    assert node.get_param(Param.MAX_SPEED) == pytest.approx(1234.5)
    node.set_param("run_current", 55)
    assert node.get_param("run_current") == 55


def test_param_rejected(bus):
    node = bus.node(1)
    with pytest.raises(ParamRejected):
        node.set_param(Param.MICROSTEPS, 3)     # not a power of two
    with pytest.raises(ParamRejected):
        node.set_param(Param.RUN_CURRENT, 250)  # > 100 %


def test_request_timeout_for_missing_node(bus):
    node = bus.node(7)  # does not exist on the sim bus
    with pytest.raises(RequestTimeout):
        node.get_position(timeout=0.05)


def test_blocking_move_and_position(bus):
    node = bus.node(1)
    node.move_to(180.0, blocking=True, timeout=1.0)
    assert node.get_position() == pytest.approx(180.0)
    node.move_by(-45.0, blocking=True, timeout=1.0)
    assert node.get_position() == pytest.approx(135.0)


def test_set_logical_position_does_not_require_motion(bus):
    node = bus.node(1)
    node.set_logical_position(37.5)
    assert node.get_position() == pytest.approx(37.5)


def test_broadcast_move_uses_one_target_for_independent_nodes(bus):
    bus.broadcast_move_to(72.0, wait_for=[1, 2, 3], timeout=1.0)
    for node_id in (1, 2, 3):
        assert bus.node(node_id).get_position() == pytest.approx(72.0)


def test_estop_latches_and_enable_clears(bus):
    node = bus.node(2)
    node.estop()
    assert node.state.status.estopped
    with pytest.raises(EStopActive):
        node.move_to(10.0)
    node.enable()
    assert not node.state.status.estopped
    node.move_to(10.0, blocking=True)
    assert node.get_position() == pytest.approx(10.0)


def test_host_side_speed_limits(bus):
    node = bus.node(1)
    node.set_speed_limits(min_speed=1.0, max_speed=100.0)
    with pytest.raises(LimitViolation):
        node.run(150.0)
    with pytest.raises(LimitViolation):
        node.run(0.5)
    node.run(50.0)  # within limits


def test_velocity_mode_with_tick(bus, net):
    node = bus.node(1)
    node.run(90.0)
    net.tick(2.0)
    assert node.get_position() == pytest.approx(180.0)
    node.stop()
    net.tick(1.0)
    assert node.get_position() == pytest.approx(180.0)


def test_homing_endstop(bus):
    node = bus.node(1)
    node.configure_homing(backoff_deg=5.0)
    node.home(method="endstop", direction=-1, speed_deg_s=20.0, timeout=1.0)
    # homed away from the switch by the backoff, in the opposite direction
    assert node.get_position() == pytest.approx(5.0)
    assert node.ping().homed


def test_homing_set_zero(bus):
    node = bus.node(3)
    node.move_to(90.0, blocking=True)
    node.home(method="set_zero")
    assert node.get_position() == pytest.approx(0.0)


def test_event_subscription(bus):
    hits = []
    bus.on_event(lambda nid, evt, detail, data: hits.append((nid, evt)))
    bus.node(1).move_to(42.0, blocking=True)
    assert (1, Event.MOVE_DONE) in hits


def test_save_and_load_defaults(bus, net):
    node = bus.node(1)
    node.set_param(Param.MAX_SPEED, 999.0)
    node.save_config()
    assert net.node(1).nvs[int(Param.MAX_SPEED)] == pytest.approx(999.0)
    node.load_defaults()
    assert node.get_param(Param.MAX_SPEED) == pytest.approx(720.0)


def test_follower_tracks_leader(bus):
    leader, follower = bus.node(1), bus.node(2)
    follower.follow(leader, ratio=1.0)
    leader.get_position()               # emits POSITION -> captures references
    leader.move_to(90.0, blocking=True)
    assert follower.get_position() == pytest.approx(90.0)
    st = follower.get_follow_status()
    assert st.enabled and st.leader_id == 1 and st.synced


def test_follower_invert_and_ratio(bus):
    leader, follower = bus.node(1), bus.node(3)
    follower.follow(leader, ratio=0.5, invert=True)
    leader.get_position()
    leader.move_to(100.0, blocking=True)
    assert follower.get_position() == pytest.approx(-50.0)


def test_follow_self_rejected(bus):
    with pytest.raises(ValueError):
        bus.node(1).follow(bus.node(1))


def test_encoder_lut_calibrate_enable_clear(bus):
    node = bus.node(3)
    st = node.get_lut_status()
    assert not st.valid and not st.enabled
    node.calibrate_encoder_lut(timeout=1.0)
    st = node.get_lut_status()
    assert st.valid and st.enabled and st.points == 200
    assert node.get_param(Param.LUT_ENABLE) == 1
    node.set_lut_enabled(False)
    assert node.get_lut_status().enabled is False
    node.set_lut_enabled(True)
    assert node.get_lut_status().enabled is True
    node.clear_encoder_lut()
    st = node.get_lut_status()
    assert not st.valid and not st.enabled
