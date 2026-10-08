"""200-point encoder LUT: map MT6701 raw counts onto stepper detents.

A 1.8° hybrid has 200 full-step alignments per revolution. Open-loop
detents are the physical ruler; the table stores the MT6701 reading at
each detent so runtime interpolation can cancel magnet runout and sensor
INL. Firmware ``lutApply`` is the C equivalent of :func:`apply_lut`.
"""

from __future__ import annotations

from typing import Iterable, List, Sequence

ENC_CPR = 16384
LUT_N = 200
STEP_COUNTS = ENC_CPR / LUT_N  # 81.92 counts per full step


def unwrap_delta(a: int, b: int) -> int:
    """Signed shortest-arc delta ``b - a`` on a 14-bit circle."""
    d = (int(b) - int(a)) % ENC_CPR
    if d > ENC_CPR // 2:
        d -= ENC_CPR
    return d


def unwrap_table(measured: Sequence[int]) -> List[int]:
    """Monotonic unwrapped counts for the 200 measured detent readings."""
    if len(measured) != LUT_N:
        raise ValueError(f"LUT must have {LUT_N} points, got {len(measured)}")
    u = [0] * (LUT_N + 1)
    u[0] = int(measured[0])
    for k in range(1, LUT_N):
        u[k] = u[k - 1] + unwrap_delta(measured[k - 1], measured[k])
    u[LUT_N] = u[0] + ENC_CPR
    return u


def average_bidir(forward: Sequence[int], reverse: Sequence[int]) -> List[int]:
    """Average a forward and reverse pass, wrapping around 14 bits."""
    if len(forward) != LUT_N or len(reverse) != LUT_N:
        raise ValueError("forward and reverse must each have 200 points")
    out: List[int] = []
    for a, b in zip(forward, reverse):
        mid = int(a) + unwrap_delta(a, b) // 2
        out.append(mid % ENC_CPR)
    return out


def apply_lut(raw: int, measured: Sequence[int]) -> int:
    """Calibrated 14-bit angle for a raw MT6701 reading.

    Finds the two detents the raw count sits between and interpolates the
    theoretical step angle ``k * 16384 / 200``.
    """
    raw = int(raw) & (ENC_CPR - 1)
    u = unwrap_table(measured)
    q = u[0] + unwrap_delta(measured[0], raw)
    while q < u[0]:
        q += ENC_CPR
    while q >= u[0] + ENC_CPR:
        q -= ENC_CPR
    lo, hi = 0, LUT_N
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if u[mid] <= q:
            lo = mid
        else:
            hi = mid
    u0, u1 = u[lo], u[lo + 1]
    frac = 0.0 if u1 == u0 else (q - u0) / (u1 - u0)
    cal = (lo + frac) * STEP_COUNTS
    return int(round(cal)) % ENC_CPR


def peak_inl_deg(measured: Sequence[int]) -> float:
    """Peak |measured − theoretical| in degrees after removing the DC offset."""
    u = unwrap_table(measured)
    peak = 0.0
    for k in range(LUT_N):
        meas = u[k] - u[0]
        expect = k * STEP_COUNTS
        err = abs(meas - expect) * 360.0 / ENC_CPR
        if err > peak:
            peak = err
    return peak


def identity_table() -> List[int]:
    return [int(round(k * STEP_COUNTS)) % ENC_CPR for k in range(LUT_N)]


def eccentric_table(amplitude_counts: float, offset: int = 0) -> List[int]:
    """Synthetic magnet-runout table: linear detents plus a 1st-harmonic."""
    import math

    out: List[int] = []
    for k in range(LUT_N):
        ideal = k * STEP_COUNTS
        wobble = amplitude_counts * math.sin(2.0 * math.pi * k / LUT_N)
        out.append(int(round(ideal + wobble + offset)) % ENC_CPR)
    return out


def table_ok(measured: Iterable[int], min_step: int = 25, max_step: int = 160) -> bool:
    """True if every unwrapped detent step is in ``(min_step, max_step)``."""
    seq = list(measured)
    if len(seq) != LUT_N:
        return False
    u = unwrap_table(seq)
    for k in range(LUT_N):
        step = u[k + 1] - u[k]
        if step < min_step or step > max_step:
            return False
    return True
