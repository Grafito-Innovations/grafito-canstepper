"""TMC2209 current-scale helpers (host-side, GCSP ``TEL_DRIVER`` / ``cs_actual``).

The TMC2209 reports ``CS_ACTUAL`` (0–31) in ``DRV_STATUS`` — the driver's
live current-regulation scale, **not** an ADC measurement of instantaneous
phase current.  Use :func:`commanded_rms_amps` to convert that scale to
commanded/regulated RMS motor current per the TMC2209 datasheet (rev 1.09).

Default sense-network values match the Grafito CANStepper C3 schematic
(100 mΩ sense resistors, ``vsense=0``, ``V_FS = 0.325 V``).
"""

from __future__ import annotations

import math

# Grafito CANStepper C3 defaults (TMC2209 datasheet § sense resistor)
CANSTEPPER_RSENSE_OHM = 0.100
CANSTEPPER_VFSENSE_V = 0.325
TMC2209_R_INTERNAL_OHM = 0.020


def full_scale_rms_amps(
    rsense_ohm: float = CANSTEPPER_RSENSE_OHM,
    vfsense_v: float = CANSTEPPER_VFSENSE_V,
    r_internal_ohm: float = TMC2209_R_INTERNAL_OHM,
) -> float:
    """Peak RMS scale at ``CS=31`` for the given sense network."""
    denom = (rsense_ohm + r_internal_ohm) * math.sqrt(2.0)
    if denom <= 0:
        raise ValueError("rsense + r_internal must be positive")
    return vfsense_v / denom


def commanded_rms_amps(
    cs_actual: int,
    *,
    rsense_ohm: float = CANSTEPPER_RSENSE_OHM,
    vfsense_v: float = CANSTEPPER_VFSENSE_V,
    r_internal_ohm: float = TMC2209_R_INTERNAL_OHM,
) -> float:
    """Convert TMC ``CS_ACTUAL`` (0–31) to commanded/regulated I_RMS in amperes.

    Formula (TMC2209 datasheet, ``vsense=0``):

    ``I_RMS = (CS + 1) / 32 × V_FS / ((R_SENSE + 0.020) × √2)``

    This is the **regulation target scale**, not 24 V bus current and not
    instantaneous coil current.  Pair with SMPS input readings for total power.
    """
    cs = int(cs_actual)
    if not 0 <= cs <= 31:
        raise ValueError(f"cs_actual must be 0..31, got {cs}")
    ifull = full_scale_rms_amps(rsense_ohm, vfsense_v, r_internal_ohm)
    return ((cs + 1) / 32.0) * ifull
