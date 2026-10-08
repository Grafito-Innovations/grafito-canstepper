#!/usr/bin/env python3
"""Visual max-speed ramp with ABSOLUTE ±50 mm oscillation (no walk-down).

Each stage:
  soft-zero at current pose as median (only once at start of stage)
  go_to(+50), then CYCLES × [go_to(-50), go_to(+50)]
  speed += STEP

Usage:
  PYTHONPATH=. python3 examples/belt_speed_ramp.py
  PYTHONPATH=. python3 examples/belt_speed_ramp.py /dev/ttyACM0 1 210 30 450
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
NODE = int(sys.argv[2]) if len(sys.argv) > 2 else 1
SPEED0 = float(sys.argv[3]) if len(sys.argv) > 3 else 150.0
STEP = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
SPEED_MAX = float(sys.argv[5]) if len(sys.argv) > 5 else 600.0

RD, INV, RUN = 77.2, True, 70
HALF, CYCLES = 50.0, 3
TOP, BOTTOM = HALF, -HALF
ACCEL = 1500.0
TOL = 6.0


def prep(n, speed):
    try:
        n.disable()
    except Exception:
        pass
    time.sleep(0.12)
    n.set_run_current(RUN).set_hold_current(35)
    n.set_microsteps(16).set_closed_loop(False).set_stealthchop(True)
    n.set_direction(INV)
    vmax = speed / RD * 360.0
    amax = ACCEL / RD * 360.0
    n.set_max_speed(max(vmax * 1.5, 1000))
    n.set_acceleration(max(amax, 3500))
    n.set_param(Param.MAX_SPEED, max(vmax * 1.5, 1000))
    n.set_param(Param.ACCELERATION, max(amax, 3500))
    n.enable()
    time.sleep(0.1)
    if int(n.get_status().fault) or n.get_status().estopped:
        n.disable()
        time.sleep(0.3)
        n.enable()
        time.sleep(0.1)


def go_to(axis, target, speed, label):
    t0 = time.monotonic()
    p0 = axis.get_position()
    axis.move_to(target, speed=abs(speed), blocking=True, timeout=60)
    dt = time.monotonic() - t0
    time.sleep(0.03)
    p1 = axis.get_position()
    err = abs(p1 - target)
    avg = abs(p1 - p0) / max(dt, 1e-3)
    ok = err <= TOL
    print(
        f"    {label:10s} target={target:+6.0f} pos={p1:+8.3f} "
        f"err={err:.2f} t={dt:.2f}s avg={avg:5.1f} {'OK' if ok else 'CLIP'}"
    )
    return ok, avg


print("=" * 66)
print("SPEED RAMP (absolute ±50 mm oscillation)")
print(f"  {SPEED0:.0f} .. {SPEED_MAX:.0f} step {STEP:.0f}  cycles/stage={CYCLES}")
print("=" * 66)

last_good = None
results = []

with CANStepperBus.serial(PORT) as bus:
    print("discover", bus.discover())
    n = bus.node(NODE)
    speed = SPEED0

    while speed <= SPEED_MAX + 0.1:
        print(f"\n##########  v_cmd = {speed:.0f} mm/s  ##########")
        try:
            prep(n, speed)
            ax = Axis(n, RD)
            # Zero ONCE per stage at current pose (= stage median)
            ax.set_zero()
            print("  median soft-zero here; targets fixed at ±50")
            avgs = []
            ok_all = True

            o, a = go_to(ax, TOP, speed, "to +50")
            ok_all &= o
            avgs.append(a)

            for c in range(1, CYCLES + 1):
                print(f"  -- cycle {c}/{CYCLES} --")
                o1, a1 = go_to(ax, BOTTOM, speed, "to -50")
                o2, a2 = go_to(ax, TOP, speed, "to +50")
                ok_all &= o1 and o2
                avgs.extend([a1, a2])
                if not (o1 and o2):
                    print("  stop stage: hard-stop or miss (frame not shifted)")
                    break

            mean = sum(avgs) / len(avgs)
            results.append((speed, ok_all, mean))
            if ok_all:
                last_good = speed
                print(f"  >>> {speed:.0f} PASS  mean avg ~{mean:.0f}")
            else:
                print(f"  >>> {speed:.0f} incomplete — continue next speed? stopping")
                n.disable()
                break

        except NodeFault as e:
            print(f"  >>> {speed:.0f} FAULT {e}")
            results.append((speed, False, None))
            n.disable()
            break
        except KeyboardInterrupt:
            print("\nstopped")
            n.disable()
            break

        n.disable()
        print(f"  pause 1.5s before {speed + STEP:.0f} …")
        time.sleep(1.5)
        speed += STEP

print("\n" + "=" * 66)
for s, ok, m in results:
    print(f"  {s:6.0f}  {'PASS' if ok else 'FAIL':4s}  avg={m}")
print(f"Highest PASS: {last_good}")
print("=" * 66)
