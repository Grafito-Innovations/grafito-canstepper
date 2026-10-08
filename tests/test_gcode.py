"""G-code layer unit tests (simulator + pure parse)."""

from __future__ import annotations

import pytest

from canstepper import (
    Axis,
    Cartesian,
    CoreXY,
    GCodeController,
    GCodeError,
    parse_gcode_line,
)
from canstepper.sim import SimNetwork


def test_parse_basic():
    line = parse_gcode_line("G1 X10 Y20 F600 ; move")
    assert line is not None
    assert line.command == "G1"
    assert line.params["X"] == 10
    assert line.params["Y"] == 20
    assert line.params["F"] == 600
    assert "move" in line.comment


def test_parse_n_line_and_comment():
    line = parse_gcode_line("N12 G28 X (home x)")
    assert line is not None
    assert line.line_number == 12
    assert line.command == "G28"
    assert "X" in line.params


def test_parse_blank():
    assert parse_gcode_line("   ; only comment") is None
    assert parse_gcode_line("") is None


def test_gcode_corexy_square(bus):
    xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
    g = GCodeController.from_corexy(xy, bus=bus, default_feedrate_mm_min=3000)
    g.run("G21")
    g.run("G90")
    g.run("G28")
    g.run("G1 X10 Y0 F600")
    x, y = xy.get_position()
    assert x == pytest.approx(10.0, abs=0.05)
    assert y == pytest.approx(0.0, abs=0.05)
    assert "X:10" in g.run("M114") or "X:10.000" in g.run("M114")

    g.run("G1 Y5")
    x, y = xy.get_position()
    assert x == pytest.approx(10.0, abs=0.05)
    assert y == pytest.approx(5.0, abs=0.05)

    g.run("G91")
    g.run("G1 X-10 Y-5 F600")
    x, y = xy.get_position()
    assert x == pytest.approx(0.0, abs=0.1)
    assert y == pytest.approx(0.0, abs=0.1)


def test_gcode_cartesian_axes(bus):
    """Ad-hoc from_axes map (no named kinematics)."""
    ax = Axis(bus.node(1), rotation_distance=40.0, name="X")
    ay = Axis(bus.node(2), rotation_distance=40.0, name="Y")
    g = GCodeController.from_axes({"X": ax, "Y": ay}, bus=bus)
    g.run("G28")
    g.run("G1 X5 Y10 F1200")
    assert ax.get_position() == pytest.approx(5.0, abs=0.05)
    assert ay.get_position() == pytest.approx(10.0, abs=0.05)


def test_gcode_adhoc_letters_abdj(bus):
    """G1 must drive every letter in from_axes, not just XYZE."""
    axes = {
        letter: Axis(bus.node(i), rotation_distance=40.0, name=letter)
        for i, letter in enumerate(("A", "B", "D", "J"), start=1)
    }
    # SimNetwork fixture only has nodes 1–3; rebuild a 4-node net.
    from canstepper import CANStepperBus
    from canstepper.sim import SimNetwork

    net = SimNetwork([1, 2, 3, 4])
    bus4 = CANStepperBus(net)
    axes = {
        "A": Axis(bus4.node(1), rotation_distance=40.0, name="A"),
        "B": Axis(bus4.node(2), rotation_distance=40.0, name="B"),
        "D": Axis(bus4.node(3), rotation_distance=77.2, name="D"),
        "J": Axis(bus4.node(4), rotation_distance=40.0, name="J"),
    }
    g = GCodeController.from_axes(axes, bus=bus4)
    g.run("G90")
    g.run("G1 A10 B20 D4 J30 F6000")
    assert axes["A"].get_position() == pytest.approx(10.0, abs=0.05)
    assert axes["B"].get_position() == pytest.approx(20.0, abs=0.05)
    assert axes["D"].get_position() == pytest.approx(4.0, abs=0.05)
    assert axes["J"].get_position() == pytest.approx(30.0, abs=0.05)
    g.run("G92 A0")
    assert g.position["A"] == pytest.approx(0.0)
    bus4.close()


def test_gcode_g92_and_relative_adhoc(bus):
    ax = Axis(bus.node(1), rotation_distance=40.0, name="A")
    g = GCodeController.from_axes({"A": ax}, bus=bus)
    g.run("G90")
    g.run("G1 A15 F1200")
    assert ax.get_position() == pytest.approx(15.0, abs=0.05)
    g.run("G92 A0")
    assert g.position["A"] == pytest.approx(0.0)
    g.run("G91")
    g.run("G1 A5 F1200")
    assert ax.get_position() == pytest.approx(5.0, abs=0.15)


def test_gcode_unmapped_letter_errors(bus):
    ax = Axis(bus.node(1), rotation_distance=40.0, name="X")
    g = GCodeController.from_axes({"X": ax}, bus=bus)
    with pytest.raises(GCodeError, match="no axis mapped for A"):
        g.run("G1 A10 F600")


def test_gcode_independent_dual_letter(bus):
    from canstepper.kinematics import IndependentDualAxis

    dual = IndependentDualAxis(
        bus.node(1), bus.node(2), rotation_distance=4.0, name="H"
    )
    g = GCodeController.from_axes({"H": dual, "X": Axis(bus.node(3), 40.0, name="X")}, bus=bus)
    g.run("G90")
    g.run("G1 H10 X5 F600")
    assert dual.get_positions()[0] == pytest.approx(10.0, abs=0.2)
    g.run("M17")


def test_gcode_cartesian_mechanism_xyz(bus):
    """Klipper-style cartesian: independent X/Y/Z via Cartesian + from_cartesian."""
    cart = Cartesian.from_nodes(
        bus.node(1),
        bus.node(2),
        bus.node(3),
        rotation_distance=40.0,
        rotation_distance_z=8.0,
    )
    g = GCodeController.from_cartesian(cart, bus=bus, default_feedrate_mm_min=3000)
    g.run("G21")
    g.run("G90")
    g.run("G28")
    g.run("G1 X10 Y0 Z0 F1200")
    x, y, z = cart.get_position()
    assert x == pytest.approx(10.0, abs=0.05)
    assert y == pytest.approx(0.0, abs=0.05)
    assert z == pytest.approx(0.0, abs=0.05)

    g.run("G1 Y8 Z2")
    x, y, z = cart.get_position()
    assert x == pytest.approx(10.0, abs=0.05)
    assert y == pytest.approx(8.0, abs=0.05)
    assert z == pytest.approx(2.0, abs=0.05)

    # Diagonal path in XY (identity motors)
    g.run("G1 X0 Y0 Z0 F1800")
    x, y, z = cart.get_position()
    assert x == pytest.approx(0.0, abs=0.1)
    assert y == pytest.approx(0.0, abs=0.1)
    assert z == pytest.approx(0.0, abs=0.1)

    resp = g.run("M114")
    assert "X:0" in resp
    assert cart.x.get_position() == pytest.approx(0.0, abs=0.1)


def test_gcode_cartesian_g92_and_relative(bus):
    cart = Cartesian.from_nodes(bus.node(1), bus.node(2), rotation_distance=40.0)
    g = GCodeController.from_cartesian(cart, bus=bus, default_feedrate_mm_min=3000)
    g.run("G28")
    g.run("G1 X10 Y5 F3000")
    g.run("G92 X0 Y0")
    assert "X:0" in g.run("M114")
    g.run("G91")
    g.run("G1 X2 Y-1 F3000")
    x, y, _ = cart.get_position()
    assert x == pytest.approx(2.0, abs=0.15)
    assert y == pytest.approx(-1.0, abs=0.15)


def test_gcode_cartesian_rejects_z_without_axis(bus):
    cart = Cartesian.from_nodes(bus.node(1), bus.node(2), rotation_distance=40.0)
    g = GCodeController.from_cartesian(cart, bus=bus)
    g.run("G28")
    with pytest.raises(GCodeError):
        g.run("G1 Z1 F600")


def test_gcode_m112_estop(bus):
    xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
    g = GCodeController.from_corexy(xy, bus=bus)
    g.run("G28")
    g.run("M17")
    g.run("M112")
    assert bus.node(1).get_status().estopped
    g.run("M17")
    assert bus.node(1).get_status().enabled


def test_gcode_unsupported(bus):
    g = GCodeController.from_axes(
        {"X": Axis(bus.node(1), 40.0)}, bus=bus
    )
    with pytest.raises(GCodeError):
        g.run("G2 X1 Y1 I0 J1")
    with pytest.raises(GCodeError):
        g.run("G20")


def test_gcode_g92(bus):
    xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
    g = GCodeController.from_corexy(xy, bus=bus)
    g.run("G28")
    g.run("G1 X10 Y0 F3000")
    g.run("G92 X0 Y0")
    # logical zero at current physical pose
    resp = g.run("M114")
    assert "X:0" in resp
    g.run("G1 X5 F3000")
    # motor travel from new zero
    assert xy.get_position()[0] == pytest.approx(5.0, abs=0.15)


def test_gcode_m220_speed_factor(bus):
    xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
    g = GCodeController.from_corexy(xy, bus=bus)
    assert "S50" in g.run("M220 S50")
    assert g.speed_factor == pytest.approx(0.5)
