import pytest

from canstepper import ConfigError, Machine, Param
from canstepper.kinematics import CoreXY, DualMotorAxis, IndependentDualAxis


def machine_dict():
    return {
        "node": {
            "z_left": {"id": 1, "run_current": 45, "microsteps": 32},
            "z_right": {"id": 2, "run_current": 45},
            "feeder": {"id": 3, "max_speed": 900.0},
        },
        "axis": {
            "z": {
                "type": "dual_motor",
                "primary": "z_left",
                "secondary": "z_right",
                "invert_secondary": True,
                "rotation_distance": 8.0,
                "max_speed": 20.0,
                "homing": {"method": "stallguard", "direction": -1, "speed": 5.0},
            },
            "feed": {
                "type": "single",
                "node": "feeder",
                "rotation_distance": 30.0,
            },
        },
    }


def test_machine_from_dict_applies_params(bus):
    m = Machine.from_config(machine_dict(), bus=bus)
    assert bus.node(1).get_param(Param.RUN_CURRENT) == 45
    assert bus.node(1).get_param(Param.MICROSTEPS) == 32
    assert bus.node(3).get_param(Param.MAX_SPEED) == pytest.approx(900.0)
    assert set(m.nodes) == {"z_left", "z_right", "feeder"}
    assert isinstance(m.axes["z"], DualMotorAxis)
    assert m.axes["z"].invert_secondary
    assert m.axes["feed"].rotation_distance == pytest.approx(30.0)


def test_machine_homing_from_config(bus):
    m = Machine.from_config(machine_dict(), bus=bus)
    m.home("z")
    assert bus.node(1).ping().homed


def test_machine_corexy(bus):
    cfg = {
        "node": {"a": {"id": 1}, "b": {"id": 2}},
        "axis": {
            "xy": {
                "type": "corexy",
                "motor_a": "a",
                "motor_b": "b",
                "rotation_distance": 40.0,
                "max_speed": 100.0,
            }
        },
    }
    m = Machine.from_config(cfg, bus=bus)
    assert isinstance(m.axes["xy"], CoreXY)
    m.axes["xy"].move_to(5.0, 5.0)
    assert m.axes["xy"].get_position() == (pytest.approx(5.0), pytest.approx(5.0))


def test_machine_independent_dual(bus):
    cfg = {
        "node": {
            "left": {"id": 1, "run_current": 40},
            "right": {"id": 2, "run_current": 40},
        },
        "axis": {
            "z": {
                "type": "independent_dual",
                "primary": "left",
                "secondary": "right",
                "invert_a": True,
                "invert_b": False,
                "rotation_distance": 4.0,
                "tol_mm": 0.5,
                "settle_s": 0.1,
            }
        },
    }
    m = Machine.from_config(cfg, bus=bus)
    axis = m.axes["z"]
    assert isinstance(axis, IndependentDualAxis)
    assert axis.node_a.node_id == 1
    assert axis.node_b.node_id == 2
    assert axis.invert_a is True
    assert axis.rotation_distance == pytest.approx(4.0)
    assert axis.tol_mm == pytest.approx(0.5)
    m.home("z")
    assert axis.get_positions() == (pytest.approx(0.0), pytest.approx(0.0))


def test_machine_estop_all(bus):
    m = Machine.from_config(machine_dict(), bus=bus)
    m.estop_all()
    for nid in (1, 2, 3):
        assert bus.node(nid).ping().estopped


def test_config_errors(bus):
    with pytest.raises(ConfigError):
        Machine.from_config({"node": {"x": {"run_current": 10}}}, bus=bus)  # no id
    with pytest.raises(ConfigError):
        Machine.from_config(
            {"node": {"x": {"id": 1, "warp_factor": 9}}}, bus=bus
        )  # unknown param
    with pytest.raises(ConfigError):
        Machine.from_config(
            {
                "node": {"x": {"id": 1}},
                "axis": {"a": {"type": "single", "node": "x"}},
            },
            bus=bus,
        )  # missing rotation_distance
