#!/usr/bin/env python3
"""Phase 2 — dual-screw reverse/straight with StallGuard host estop.

Same dual-frame kinematics as ``dual_screw_oscillate.py``, plus
:class:`StallGuardEstop` (arm after free travel → STALL → ``estop_all()``).

Default motion mode is **open-loop + StealthChop** (same combination that
passed Phase 1 jam tests). Closed-loop + StealthChop often raises firmware
``FAULT_NO_PROGRESS`` (code 2) on this frame; use ``--closed-loop`` only if
you are retuning CL + SG together.

Usage
-----
  cd can_stepper

  # Free cycles (recommended first)
  PYTHONPATH=. python3 -u examples/dual_screw_protected.py /dev/ttyACM0 2 3 20 3 15

  # Then jam mid-stroke once to confirm protection
  PYTHONPATH=. python3 -u examples/dual_screw_protected.py /dev/ttyACM0 2 3 20 5 15 --threshold 12

Args:
  port  node_a  node_b  stroke_mm  cycles  speed_mm_s  [pitch_mm]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import (  # noqa: E402
    CANStepperBus,
    EncoderGateTimeout,
    Fault,
    IndependentDualAxis,
    NodeFault,
    StallGuardEstop,
    StallGuardTrip,
)

_FAULT_HINT = {
    int(Fault.NONE): "none",
    int(Fault.ENCODER): "encoder invalid",
    int(Fault.NO_PROGRESS): (
        "NO_PROGRESS (closed-loop stall watchdog ~3s) — encoder not tracking "
        "target. Prefer open-loop+StealthChop for SG, or lower speed / raise "
        "current / use SpreadCycle without relying on SG."
    ),
    int(Fault.HOMING_TIMEOUT): "homing timeout",
    int(Fault.DRIVER_OT): "TMC over-temp",
    int(Fault.DRIVER_SHORT): "TMC short",
}


def main() -> int:
    ap = argparse.ArgumentParser(description="Dual screw + StallGuard estop (Phase 2)")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("node_a", nargs="?", type=int, default=2)
    ap.add_argument("node_b", nargs="?", type=int, default=3)
    ap.add_argument("stroke_mm", nargs="?", type=float, default=20.0)
    ap.add_argument("cycles", nargs="?", type=int, default=3)
    ap.add_argument(
        "speed_mm_s",
        nargs="?",
        type=float,
        default=15.0,
        help="linear speed (default 15 mm/s — safer with StealthChop+SG)",
    )
    ap.add_argument("pitch_mm", nargs="?", type=float, default=4.0)
    ap.add_argument("--threshold", type=int, default=12)
    ap.add_argument("--current", type=int, default=40)
    ap.add_argument("--min-arm-mm", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=0.5)
    ap.add_argument("--no-sg", action="store_true")
    ap.add_argument(
        "--closed-loop",
        action="store_true",
        help="use encoder CL (may fault NO_PROGRESS with StealthChop; default is open-loop)",
    )
    ap.add_argument("--start-negative", action="store_true")
    args = ap.parse_args()

    mode = "closed-loop" if args.closed_loop else "open-loop"
    print("=" * 64, flush=True)
    print("Phase 2 — dual screw + StallGuard host estop", flush=True)
    print(
        f"  {args.port}  nodes {args.node_a}+{args.node_b}  "
        f"pitch={args.pitch_mm} mm  motion={mode}",
        flush=True,
    )
    print(
        f"  stroke=±{args.stroke_mm} mm  cycles={args.cycles}  "
        f"speed={args.speed_mm_s} mm/s  Irun={args.current}%",
        flush=True,
    )
    if args.no_sg:
        print("  StallGuard protection: OFF", flush=True)
    else:
        print(
            f"  StallGuard: thr={args.threshold}  min_arm={args.min_arm_mm} mm  "
            f"settle={args.settle}s  StealthChop=ON",
            flush=True,
        )
    print("=" * 64, flush=True)

    with CANStepperBus.serial(args.port) as bus:
        time.sleep(0.4)
        found = bus.discover(timeout=2.5)
        print("discover:", found, flush=True)
        if args.node_a not in found or args.node_b not in found:
            print(f"!! need nodes {args.node_a} and {args.node_b}", flush=True)
            return 1

        dual = IndependentDualAxis(
            bus.node(args.node_a),
            bus.node(args.node_b),
            rotation_distance=args.pitch_mm,
            invert_a=True,
            invert_b=False,
            name="frame",
            tol_mm=1.0,
            settle_s=0.3,
            gate_timeout_s=45.0,
        )

        protect: StallGuardEstop | None = None
        if not args.no_sg:
            protect = StallGuardEstop(
                bus,
                [args.node_a, args.node_b],
                threshold=args.threshold,
            )
            protect.attach()

        # Clear any previous estop / fault latch from Phase 1 tests
        print("clearing estop + enabling …", flush=True)
        bus.estop_all()
        time.sleep(0.2)
        bus.enable_all(True)
        time.sleep(0.25)
        dual.enable()
        time.sleep(0.15)

        if args.closed_loop:
            dual.configure_closed_loop(
                args.speed_mm_s,
                run_current=args.current,
                hold_current=15,
                microsteps=8,
                kp=12.0,
                ki=0.3,
                kd=0.10,
                tolerance_deg=12.0,
            )
        else:
            # Open-loop: matches Phase 1 jam path; encoder gate still checks both sides
            dual.configure_open_loop(
                args.speed_mm_s,
                run_current=args.current,
                hold_current=15,
                microsteps=8,
                accel_factor=2.5,
            )

        if protect is not None:
            # Must be after configure_* (those helpers force SpreadCycle off/on)
            protect.apply_to_nodes(
                [dual.node_a, dual.node_b],
                stealthchop=True,
                threshold=args.threshold,
            )
            print(
                f"  SG applied: thr={args.threshold} stealthchop=1 on both nodes",
                flush=True,
            )
        elif args.closed_loop:
            print("  CL SpreadCycle (no SG)", flush=True)

        # Status snapshot
        for n in (dual.node_a, dual.node_b):
            st = n.get_status()
            print(
                f"  node {n.node_id}: enabled={st.enabled} estopped={st.estopped} "
                f"fault={st.fault} moving={st.moving}",
                flush=True,
            )
            if st.estopped or int(st.fault) != 0:
                print(
                    f"  !! node {n.node_id} not clean — enable() again …",
                    flush=True,
                )
                n.enable()
                time.sleep(0.1)

        dual.set_zero()
        pa, pb = dual.get_positions()
        print(f"zero: a={pa:+.3f} mm  b={pb:+.3f} mm", flush=True)
        if not args.start_negative:
            print(
                "  Free path: first leg toward commanded +stroke "
                "(encoder may read −stroke with open-loop+invert — that is OK).",
                flush=True,
            )
        else:
            print(
                "  Free path: first leg toward commanded −stroke "
                "(--start-negative).",
                flush=True,
            )
        print(
            "  A real jam mid-leg should estop both.",
            flush=True,
        )
        print(flush=True)

        t0 = time.monotonic()
        try:
            n = dual.oscillate(
                args.stroke_mm,
                args.cycles,
                speed_mm_s=args.speed_mm_s,
                start_positive=not args.start_negative,
                protect=protect,
                settle_s=args.settle,
                min_arm_mm=args.min_arm_mm,
            )
            print(
                f"\nPASS  completed {n}/{args.cycles} cycles in "
                f"{time.monotonic() - t0:.1f}s",
                flush=True,
            )
            pa, pb = dual.get_positions()
            print(
                f"final a={pa:+.3f} mm  b={pb:+.3f} mm  Δ={pa - pb:+.3f} mm",
                flush=True,
            )
            if protect is not None:
                print(
                    f"  ignored (disarmed) STALLs: {protect.ignored_stalls}",
                    flush=True,
                )
            dual.disable()
            print("done", flush=True)
            return 0

        except StallGuardTrip as e:
            print(f"\nSTALL PROTECTION: {e}", flush=True)
            print(
                "  Both axes should be estopped. Clear jam, then re-run.",
                flush=True,
            )
            if protect is not None:
                print(
                    f"  trip node={protect.trip_node_id}  "
                    f"ignored_while_disarmed={protect.ignored_stalls}",
                    flush=True,
                )
            return 10

        except EncoderGateTimeout as e:
            print(f"\nGATE FAIL: {e}", flush=True)
            if e.positions_mm:
                print(
                    f"  positions: a={e.positions_mm[0]:+.3f} "
                    f"b={e.positions_mm[1]:+.3f} mm",
                    flush=True,
                )
            msg = str(e).lower()
            if "stopped short" in msg or "possible jam" in msg:
                print(
                    "  Physical jam likely; StallGuard DIAG did not fire "
                    "(raise --threshold, or lower --current).",
                    flush=True,
                )
            try:
                bus.estop_all()
            except Exception:
                pass
            return 2

        except NodeFault as e:
            code = int(getattr(e, "fault_code", -1))
            hint = _FAULT_HINT.get(code, f"unknown fault {code}")
            print(f"\nNODE FAULT: {e}", flush=True)
            print(f"  meaning: {hint}", flush=True)
            if code == int(Fault.NO_PROGRESS) and args.closed_loop:
                print(
                    "  tip: re-run without --closed-loop (open-loop default) "
                    "for StallGuard Phase 2.",
                    flush=True,
                )
            try:
                bus.estop_all()
            except Exception:
                pass
            return 1

        except KeyboardInterrupt:
            print("\n(interrupted)", flush=True)
            if protect is not None:
                protect.disarm()
            try:
                bus.estop_all()
            except Exception:
                pass
            return 130


if __name__ == "__main__":
    sys.exit(main())
