#!/usr/bin/env python3
"""Dual ball-screw frame: reverse / straight with encoder-gated closed-loop.

Two motors (e.g. SFU1204, 4 mm pitch) are commanded independently with the
same absolute mm target. One side typically uses ``invert_a=True`` so both
screws advance the frame in the same linear direction. The next pass starts
only after **both** encoders are within tolerance of the target (not merely
when step generators go idle).

Bring-up reference (Grafito dual frame):
  - closed-loop, SpreadCycle, run_current 50%
  - PID kp=12, ki=0.3, kd=0.1
  - proven continuous reverse/straight at 56–76 mm/s for 20 mm stroke

Usage:
  PYTHONPATH=. python3 examples/dual_screw_oscillate.py
  PYTHONPATH=. python3 examples/dual_screw_oscillate.py /dev/ttyACM0 1 2 20 5 56
  PYTHONPATH=. python3 examples/dual_screw_oscillate.py /dev/ttyACM0 1 2 20 5 36 4.0

Args:
  port  node_a  node_b  stroke_mm  cycles  speed_mm_s  [pitch_mm]
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import (  # noqa: E402
    CANStepperBus,
    EncoderGateTimeout,
    IndependentDualAxis,
    NodeFault,
)


PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_A = int(sys.argv[2]) if len(sys.argv) > 2 else 1
NODE_B = int(sys.argv[3]) if len(sys.argv) > 3 else 2
STROKE_MM = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
CYCLES = int(sys.argv[5]) if len(sys.argv) > 5 else 5
SPEED_MM_S = float(sys.argv[6]) if len(sys.argv) > 6 else 56.0
PITCH_MM = float(sys.argv[7]) if len(sys.argv) > 7 else 4.0  # SFU1204


def main() -> None:
    print("=" * 64)
    print("IndependentDualAxis — dual screw reverse/straight (encoder-gated CL)")
    print(f"  {PORT}  nodes {NODE_A}+{NODE_B}  pitch={PITCH_MM} mm/rev")
    print(f"  stroke=±{STROKE_MM} mm  cycles={CYCLES}  speed={SPEED_MM_S} mm/s")
    print(f"  invert_a=True  invert_b=False  (mirrored frame; edit if needed)")
    print("=" * 64)

    with CANStepperBus.serial(PORT) as bus:
        time.sleep(0.4)
        found = bus.discover(timeout=2.5)
        print("discover:", found)
        if NODE_A not in found or NODE_B not in found:
            sys.exit(f"need nodes {NODE_A} and {NODE_B}, found {found}")

        dual = IndependentDualAxis(
            bus.node(NODE_A),
            bus.node(NODE_B),
            rotation_distance=PITCH_MM,
            invert_a=True,
            invert_b=False,
            name="frame",
            tol_mm=0.8,
            settle_s=0.25,
            gate_timeout_s=25.0,
        )

        bus.estop_all()
        time.sleep(0.25)
        bus.enable_all(True)
        time.sleep(0.2)
        dual.enable()

        dual.configure_closed_loop(
            SPEED_MM_S,
            run_current=50,
            hold_current=15,
            microsteps=8,
            kp=12.0,
            ki=0.3,
            kd=0.10,
        )
        dual.set_zero()
        pa, pb = dual.get_positions()
        print(f"zero: a={pa:+.3f} mm  b={pb:+.3f} mm")
        print()

        t0 = time.monotonic()
        try:
            n = dual.oscillate(
                STROKE_MM,
                CYCLES,
                speed_mm_s=SPEED_MM_S,
                start_positive=True,
                wait_encoders=True,
            )
            print(f"\nPASS  completed {n}/{CYCLES} cycles in {time.monotonic() - t0:.1f}s")
            pa, pb = dual.get_positions()
            print(f"final a={pa:+.3f} mm  b={pb:+.3f} mm  Δ={pa - pb:+.3f} mm")
        except EncoderGateTimeout as e:
            print(f"\nGATE FAIL: {e}")
            if e.positions_mm:
                print(f"  positions: a={e.positions_mm[0]:+.3f} b={e.positions_mm[1]:+.3f} mm")
            sys.exit(2)
        except NodeFault as e:
            print(f"\nNODE FAULT: {e}")
            try:
                bus.estop_all()
            except Exception:
                pass
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n(interrupted)")
            bus.estop_all()
            sys.exit(130)

        dual.disable()
        print("done")


if __name__ == "__main__":
    main()
