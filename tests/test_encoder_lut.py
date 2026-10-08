import math

from canstepper.encoder_lut import (
    ENC_CPR,
    LUT_N,
    STEP_COUNTS,
    apply_lut,
    average_bidir,
    eccentric_table,
    identity_table,
    peak_inl_deg,
    table_ok,
    unwrap_delta,
)


def test_unwrap_wraps_the_14bit_circle():
    assert unwrap_delta(16380, 10) == 14
    assert unwrap_delta(10, 16380) == -14
    assert unwrap_delta(100, 180) == 80


def test_identity_table_is_a_near_noop():
    table = identity_table()
    assert table_ok(table)
    assert peak_inl_deg(table) < 0.02
    for raw in (0, 40, 81, 82, 1000, 8192, 16300, 16383):
        cal = apply_lut(raw, table)
        err = min((cal - raw) % ENC_CPR, (raw - cal) % ENC_CPR)
        assert err <= 1


def test_constant_offset_is_removed():
    offset = 42
    table = [(v + offset) % ENC_CPR for v in identity_table()]
    # raw 42 is detent 0 → calibrated 0
    assert apply_lut(42, table) == 0
    # one full step later
    raw = (42 + int(round(STEP_COUNTS))) % ENC_CPR
    cal = apply_lut(raw, table)
    assert min(cal, ENC_CPR - cal) <= 2 or abs(cal - int(round(STEP_COUNTS))) <= 2


def test_eccentricity_is_flattened():
    # ~1.0° of 1st-harmonic magnet runout (typical cheap magnet mount)
    amp = 16384 * 1.0 / 360.0
    table = eccentric_table(amp, offset=30)
    assert table_ok(table)
    assert peak_inl_deg(table) > 0.7

    errors = []
    for k in range(0, LUT_N, 5):
        raw = table[k]
        cal = apply_lut(raw, table)
        expect = int(round(k * STEP_COUNTS)) % ENC_CPR
        err = min((cal - expect) % ENC_CPR, (expect - cal) % ENC_CPR)
        errors.append(err * 360.0 / ENC_CPR)
    assert max(errors) < 0.05


def test_interpolation_between_detents():
    table = identity_table()
    # Halfway between step 0 and step 1 → ~0.9°
    mid = int(round(STEP_COUNTS / 2))
    cal = apply_lut(mid, table)
    assert abs(cal - mid) <= 1


def test_bidir_average_cancels_hysteresis():
    base = identity_table()
    fwd = [(v + 4) % ENC_CPR for v in base]
    rev = [(v - 4) % ENC_CPR for v in base]
    avg = average_bidir(fwd, rev)
    for a, b in zip(avg, base):
        d = min((a - b) % ENC_CPR, (b - a) % ENC_CPR)
        assert d <= 1


def test_bad_table_rejected():
    bad = [0] * LUT_N
    assert not table_ok(bad)
    rev = list(reversed(identity_table()))
    assert not table_ok(rev)
    assert math.isfinite(peak_inl_deg(identity_table()))
