import pytest

from canstepper import Param


def test_group_fanout_move(bus):
    group = bus.group([1, 2])
    group.enable().move_to(30.0)
    assert bus.node(1).get_position() == pytest.approx(30.0)
    assert bus.node(2).get_position() == pytest.approx(30.0)
    assert bus.node(3).get_position() == pytest.approx(0.0)  # not in the group


def test_group_estop_scope(bus):
    bus.group([1, 3]).estop()
    assert bus.node(1).ping().estopped
    assert not bus.node(2).ping().estopped
    assert bus.node(3).ping().estopped


def test_bus_estop_all_broadcast(bus, net):
    bus.estop_all()
    for nid in (1, 2, 3):
        assert bus.node(nid).ping().estopped
        assert not net.node(nid).enabled


def test_group_param_fanout(bus):
    bus.group([1, 2, 3]).set_param(Param.RUN_CURRENT, 42)
    for nid in (1, 2, 3):
        assert bus.node(nid).get_param(Param.RUN_CURRENT) == 42


def test_group_each(bus):
    seen = []
    bus.group([1, 2]).each(lambda n: seen.append(n.node_id))
    assert seen == [1, 2]
