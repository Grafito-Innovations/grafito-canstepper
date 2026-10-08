#!/usr/bin/env python3
"""Belt axis move in mm — Klipper-like speed profile on canstepper.

Klipper machine (reference)::

    step_pin: PB4
    dir_pin: PB3
    enable_pin: !PB6
    microsteps: 16
    rotation_distance: 77.2
    velocity: 50
    accel: 300
    homing_speed: 70

Mapped to GCSP / this board:
  - open-loop steps (like manual_stepper)
  - microsteps 16, RD 77.2, v=50 mm/s, a=300 mm/s²
  - StealthChop ON (SpreadCycle @ high current was tripping fault 5 on this TMC)
  - invert_dir default True so +mm moves away from the limit switch
  - run_current 55% (raise with ``current=70`` if needed; 85% often faults)

Usage:
  PYTHONPATH=. python3 examples/belt_move_mm.py
  PYTHONPATH=. python3 examples/belt_move_mm.py /dev/ttyACM0 1 10
  PYTHONPATH=. python3 examples/belt_move_mm.py /dev/ttyACM0 1 50 77.2 50
  PYTHONPATH=. python3 examples/belt_move_mm.py /dev/ttyACM0 1 10 77.2 70   # faster
  PYTHONPATH=. python3 examples/belt_move_mm.py /dev/ttyACM0 1 10 77.2 50 noinvert
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from canstepper import Axis, CANStepperBus, Param
from canstepper.exceptions import NodeFault

# --- positional args (flags stripped) ---
_raw = sys.argv[1:]
_flags = set()
_pos = []
for a in _raw:
    al = a.lower()
    if al in ("invert", "noinvert", "normal", "spread", "stealth") or al.startswith("current="):
        _flags.add(al)
    else:
        _pos.append(a)

PORT = _pos[0] if len(_pos) > 0 else "/dev/ttyACM0"
NODE_ID = int(_pos[1]) if len(_pos) > 1 else 1
DISTANCE_MM = float(_pos[2]) if len(_pos) > 2 else 10.0
ROTATION_DISTANCE = float(_pos[3]) if len(_pos) > 3 else 77.2
# Klipper velocity: 50 mm/s (homing_speed 70 available as higher arg)
SPEED_MM_S = float(_pos[4]) if len(_pos) > 4 else 50.0
ACCEL_MM_S2 = float(_pos[5]) if len(_pos) > 5 else 300.0

if "noinvert" in _flags or "normal" in _flags:
    INVERT = False
else:
    INVERT = True

STEALTH = "spread" not in _flags  # default StealthChop; pass ``spread`` for SpreadCycle
MICROSTEPS = 16
RUN_PCT = 55
HOLD_PCT = 25
for f in _flags:
    if f.startswith("current="):
        RUN_PCT = int(float(f.split("=", 1)[1]))
        HOLD_PCT = max(10, RUN_PCT // 2)


def mm_to_deg(mm: float) -> float:
    return mm / ROTATION_DISTANCE * 360.0


def prep(node) -> None:
    try:
        node.disable()
    except Exception:
        pass
    time.sleep(0.2)

    node.set_run_current(RUN_PCT).set_hold_current(HOLD_PCT)
    node.set_microsteps(MICROSTEPS)
    node.set_closed_loop(False)
    node.set_stealthchop(STEALTH)
    node.set_direction(INVERT)

    vmax = mm_to_deg(SPEED_MM_S)
    amax = mm_to_deg(ACCEL_MM_S2)
    # Host speed limits (deg/s) — firmware MAX_SPEED / ACCELERATION
    node.set_max_speed(max(vmax * 1.2, 400.0))
    node.set_acceleration(max(amax, 800.0))
    node.set_param(Param.MAX_SPEED, max(vmax * 1.2, 400.0))
    node.set_param(Param.ACCELERATION, max(amax, 800.0))
    node.set_param(Param.CL_MAX_SPEED, max(vmax * 1.5, 800.0))
    node.set_param(Param.CL_MAX_ACCEL, max(amax * 2, 2000.0))

    print(f"  invert={INVERT}  µsteps={MICROSTEPS}  stealthchop={STEALTH}")
    print(f"  I_run={RUN_PCT}%  I_hold={HOLD_PCT}%  (open-loop)")
    print(f"  velocity={SPEED_MM_S} mm/s → {vmax:.1f} deg/s")
    print(f"  accel={ACCEL_MM_S2} mm/s² → {amax:.1f} deg/s²")

    node.enable()
    time.sleep(0.2)
    st = node.get_status()
    if st.estopped or int(st.fault) != 0:
        print(f"  clear fault={st.fault} estop={st.estopped}")
        node.disable()
        time.sleep(0.35)
        node.enable()
        time.sleep(0.2)

    try:
        d = node.get_driver_status()
        print(
            f"  TMC cs={d.cs_actual} stealth={d.stealth_chop} "
            f"drv_err={d.drv_err} ot={d.over_temp_shutdown}"
        )
    except Exception as e:
        print(f"  TMC: {e}")


def main() -> None:
    print("=" * 60)
    print("Belt move @ Klipper-like speed")
    print(f"  {PORT} node={NODE_ID}  RD={ROTATION_DISTANCE} mm/rev")
    print(f"  distance={DISTANCE_MM:+.3f} mm  v={SPEED_MM_S} mm/s  a={ACCEL_MM_S2}")
    print("=" * 60)

    with CANStepperBus.serial(PORT) as bus:
        found = bus.discover()
        print("discover:", found)
        if NODE_ID not in found:
            sys.exit("node missing — check USB/power/fw")

        node = bus.node(NODE_ID)
        print(f"firmware: {node.get_status().firmware}")

        prep(node)
        axis = Axis(node, rotation_distance=ROTATION_DISTANCE, name="belt")
        axis.set_zero()
        t0 = time.monotonic()
        p0 = axis.get_position()
        print(f"start {p0:.3f} mm  ({mm_to_deg(DISTANCE_MM):.2f} deg)")

        try:
            axis.move_by(
                DISTANCE_MM,
                speed=SPEED_MM_S,
                blocking=True,
                timeout=120.0,
            )
        except NodeFault as e:
            print(f"!! NodeFault: {e}")
            try:
                d = node.get_driver_status()
                print(
                    f"   ls_short={d.low_side_short_a}/{d.low_side_short_b} "
                    f"gnd_short={d.short_to_gnd_a}/{d.short_to_gnd_b} "
                    f"drv_err={d.drv_err} cs={d.cs_actual}"
                )
            except Exception:
                pass
            print(f"   deg={node.get_position():.2f} fault={node.get_status().fault}")
            node.disable()
            print("hint: lower speed; try without 'spread'; current=40")
            sys.exit(1)

        dt = time.monotonic() - t0
        time.sleep(0.15)
        p1 = axis.get_position()
        print(f"end   {p1:.3f} mm  in {dt:.2f}s")
        print(f"delta {p1 - p0:+.3f} mm  (target {DISTANCE_MM:+.3f})")
        print(f"deg   {node.get_position():.2f}")
        if abs(DISTANCE_MM) > 0.1:
            print(f"avg speed ~{abs(p1 - p0) / max(dt, 1e-3):.1f} mm/s")
        err = abs((p1 - p0) - DISTANCE_MM)
        if err < abs(DISTANCE_MM) * 0.15 + 0.8:
            print("OK — near target")
        else:
            print("NOTE — distance short/long; tune current or load")

        node.disable()
        print("done")


if __name__ == "__main__":
    main()
