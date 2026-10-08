#!/usr/bin/env python3
"""Oscillate belt across a median using ABSOLUTE targets (no walk-down).

Problem with pure move_by(+50, -100, +100, …):
  if a leg is clipped by a hard stop, the soft coordinate frame drifts and
  the next “down 100” drives further into the lower obstruction.

Fix: zero once at the median, then always::

    move_to(+50)   # top
    move_to(-50)   # bottom
    move_to(+50)   # top
    …

So every half-cycle is an absolute pose about the original median.

Usage:
  PYTHONPATH=. python3 examples/belt_oscillate.py
  PYTHONPATH=. python3 examples/belt_oscillate.py /dev/ttyACM0 1 200 15
  PYTHONPATH=. python3 examples/belt_oscillate.py /dev/ttyACM0 1 400 5 77.2 65

Args:
  port  node_id  speed_mm_s  cycles  [rotation_distance] [current_pct]
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

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
SPEED = float(sys.argv[3]) if len(sys.argv) > 3 else 150.0
CYCLES = int(sys.argv[4]) if len(sys.argv) > 4 else 10
RD = float(sys.argv[5]) if len(sys.argv) > 5 else 77.2
RUN = int(sys.argv[6]) if len(sys.argv) > 6 else 70

HALF = 50.0          # ±50 mm about median  (100 mm full stroke)
TOP = +HALF
BOTTOM = -HALF
ACCEL = 1200.0
INVERT = True
MICRO = 16
TOL_MM = 6.0         # abort if we never get near target (hard stop / miss)


def mm_to_deg(mm: float) -> float:
    return mm / RD * 360.0


def prep(node, speed: float) -> None:
    try:
        node.disable()
    except Exception:
        pass
    time.sleep(0.12)
    node.set_run_current(RUN).set_hold_current(max(15, RUN // 2))
    node.set_microsteps(MICRO)
    node.set_closed_loop(False)
    node.set_stealthchop(True)
    node.set_direction(INVERT)
    vmax = mm_to_deg(speed)
    amax = mm_to_deg(ACCEL)
    node.set_max_speed(max(vmax * 1.5, 1000))
    node.set_acceleration(max(amax, 3000))
    node.set_param(Param.MAX_SPEED, max(vmax * 1.5, 1000))
    node.set_param(Param.ACCELERATION, max(amax, 3000))
    node.enable()
    time.sleep(0.1)
    st = node.get_status()
    if st.estopped or int(st.fault) != 0:
        node.disable()
        time.sleep(0.3)
        node.enable()
        time.sleep(0.1)


def go_to(axis: Axis, target: float, speed: float, label: str) -> bool:
    """Absolute move to target mm; return False if far from target (clip/miss)."""
    t0 = time.monotonic()
    p0 = axis.get_position()
    axis.move_to(target, speed=abs(speed), blocking=True, timeout=60.0)
    dt = time.monotonic() - t0
    time.sleep(0.04)
    p1 = axis.get_position()
    err = abs(p1 - target)
    avg = abs(p1 - p0) / max(dt, 1e-3)
    ok = err <= TOL_MM
    tag = "OK" if ok else "CLIP/MISS"
    print(
        f"  {label:12s}  target={target:+7.1f}  "
        f"pos={p1:+8.3f}  err={err:.3f}  "
        f"t={dt:.2f}s  avg={avg:.1f}  {tag}"
    )
    return ok


def main() -> None:
    print("=" * 64)
    print("Belt median oscillation (ABSOLUTE ±50 mm — no walk-down)")
    print(f"  {PORT}  node={NODE_ID}  RD={RD}")
    print(f"  pattern: go_to(+{HALF}) then loop go_to(-{HALF}) / go_to(+{HALF})")
    print(f"  cycles={CYCLES}  speed={SPEED} mm/s  accel={ACCEL}  I={RUN}%")
    print("=" * 64)
    print("Park at MEDIAN before start. Soft-zero = median.")
    print()

    with CANStepperBus.serial(PORT) as bus:
        found = bus.discover()
        print("discover:", found)
        if NODE_ID not in found:
            sys.exit("node missing")

        node = bus.node(NODE_ID)
        print(f"firmware: {node.get_status().firmware}")
        prep(node, SPEED)

        axis = Axis(node, rotation_distance=RD, name="belt")
        # ONE soft zero at median — never re-zero mid-test
        axis.set_zero()
        print(f"median = 0  envelope [{BOTTOM:.0f} .. {TOP:.0f}] mm")
        print()

        n_ok = 0
        n_fail = 0
        t_all = time.monotonic()

        try:
            # 1) first up to top
            if not go_to(axis, TOP, SPEED, "up to +50"):
                print("!! could not reach TOP — check clearance above median")
                n_fail += 1
            else:
                n_ok += 1

            for i in range(1, CYCLES + 1):
                print(f"\n-- cycle {i}/{CYCLES} --")
                try:
                    if not go_to(axis, BOTTOM, SPEED, "down to -50"):
                        n_fail += 1
                        print("!! clip/miss at BOTTOM — stop (would walk into stop)")
                        break
                    n_ok += 1
                    if not go_to(axis, TOP, SPEED, "up to +50"):
                        n_fail += 1
                        print("!! clip/miss at TOP — stop")
                        break
                    n_ok += 1
                except NodeFault as e:
                    n_fail += 1
                    print(f"!! NodeFault: {e}")
                    try:
                        d = node.get_driver_status()
                        print(
                            f"   ot={d.over_temp_shutdown} "
                            f"ls={d.low_side_short_a}/{d.low_side_short_b} "
                            f"cs={d.cs_actual}"
                        )
                    except Exception:
                        pass
                    break

        except KeyboardInterrupt:
            print("\n(interrupted)")

        print()
        print("=" * 64)
        print(
            f"done  ok_moves={n_ok}  fails={n_fail}  "
            f"t={time.monotonic() - t_all:.1f}s"
        )
        print(f"final pos (rel median) = {axis.get_position():+.3f} mm")
        print("=" * 64)
        node.disable()


if __name__ == "__main__":
    main()
