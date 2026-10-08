"""TMC2209 CS_ACTUAL → commanded I_RMS helpers."""

from __future__ import annotations

import math
import struct

import pytest

from canstepper import commanded_rms_amps, full_scale_rms_amps
from canstepper.telemetry import DriverStatus
from canstepper.tmc2209 import CANSTEPPER_RSENSE_OHM, CANSTEPPER_VFSENSE_V


def test_full_scale_canstepper_defaults():
    ifs = full_scale_rms_amps()
    assert ifs == pytest.approx(1.915, rel=0.002)


@pytest.mark.parametrize(
    "cs, expected",
    [
        (2, 0.180),
        (15, 0.958),
        (21, 1.317),
        (31, 1.915),
    ],
)
def test_commanded_rms_amps_bench_values(cs, expected):
    got = commanded_rms_amps(cs)
    assert got == pytest.approx(expected, abs=0.002)


def test_commanded_rms_amps_invalid_cs():
    with pytest.raises(ValueError):
        commanded_rms_amps(32)


def test_driver_status_property():
    payload = struct.pack("<HBBBBH", 40, 0x01, 0x40, 0x00, 15, 1000)
    d = DriverStatus.decode(payload)
    assert d.cs_actual == 15
    assert d.standstill is True
    assert d.commanded_rms_amps() == pytest.approx(0.958, abs=0.002)
    assert d.commanded_rms_amps_default == d.commanded_rms_amps()


def test_driver_status_custom_rsense():
    d = DriverStatus(cs_actual=15)
    custom = d.commanded_rms_amps(rsense_ohm=0.110)
    default = d.commanded_rms_amps()
    assert custom < default


def test_formula_matches_manual():
    cs = 15
    rs = CANSTEPPER_RSENSE_OHM
    vfs = CANSTEPPER_VFSENSE_V
    manual = ((cs + 1) / 32.0) * (vfs / ((rs + 0.020) * math.sqrt(2)))
    assert commanded_rms_amps(cs) == pytest.approx(manual)
