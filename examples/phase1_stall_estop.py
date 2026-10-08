#!/usr/bin/env python3
"""Phase 1 — host crash protection via StallGuard (EVT_STALL → estop both).

Modes
-----
**stroke** (default)
  From the *current* pose, drive both screws a fixed distance (e.g. +25 mm)
  toward a hard stop. No hand jam. When DIAG fires, host ``estop_all()``
  stops **both** motors.

**hand**
  Open-loop cruise; you manually lock one shaft after ARM.

Start-up DIAG glitches are ignored (arm only after ``--settle``).

Phase 0 reference: SGTHRS **30**, ~20% current, StealthChop ON.

Usage
-----
  cd can_stepper

  # Auto-jam: move +25 mm from here into hard stop
  PYTHONPATH=. python3 examples/phase1_stall_estop.py /dev/ttyACM0 2 3
  PYTHONPATH=. python3 examples/phase1_stall_estop.py /dev/ttyACM0 2 3 --stroke-mm 25
  PYTHONPATH=. python3 examples/phase1_stall_estop.py /dev/ttyACM0 2 3 --stroke-mm -25

  # Manual shaft jam
  PYTHONPATH=. python3 examples/phase1_stall_estop.py /dev/ttyACM0 2 3 --mode hand

Ctrl+C → estop. Keep limit switches as backup.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from canstepper import CANStepperBus, Event, IndependentDualAxis, Param  # noqa: E402
import canstepper as _cs  # noqa: E402


def _log(msg: str = "") -> None:
    """Print immediately (no buffered 'blank terminal')."""
    print(msg, flush=True)


def _prompt(msg: str, auto: bool) -> None:
    if auto:
        _log(f"  [auto] {msg} — starting in 2s (Ctrl+C to abort)")
        time.sleep(2.0)
        return
    _log("")
    _log("  ************************************************")
    _log(f"  >> {msg}")
    _log("  >> Press ENTER in this terminal to continue")
    _log("  ************************************************")
    try:
        input()
    except EOFError:
        pass
    _log("  (Enter received — continuing)")


def _prep_sg(node, current: int, threshold: int, cruise_deg_s: float) -> None:
    """Open-loop + StealthChop — needed for TMC2209 StallGuard4."""
    node.set_run_current(current)
    node.set_hold_current(max(10, current // 3))
    node.set_microsteps(8)
    node.set_closed_loop(False)
    node.set_stealthchop(True)
    node.set_stall_threshold(threshold)
    node.set_param(Param.MAX_SPEED, max(720.0, cruise_deg_s * 2.0))
    node.set_param(Param.ACCELERATION, max(800.0, cruise_deg_s * 4.0))
    node.enable()
    time.sleep(0.08)


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 1 StallGuard host estop")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("node_a", nargs="?", type=int, default=2)
    ap.add_argument("node_b", nargs="?", type=int, default=3)
    ap.add_argument("--mode", choices=("stroke", "hand"), default="stroke")
    ap.add_argument(
        "--stroke-mm",
        type=float,
        default=25.0,
        help="signed mm from current pose toward the jam (stroke mode)",
    )
    ap.add_argument("--pitch-mm", type=float, default=4.0, help="SFU1204 = 4")
    ap.add_argument(
        "--speed-mm",
        type=float,
        default=0.5,
        help="linear speed mm/s (stroke mode). Default 0.5 ≈ 45 deg/s on 4 mm "
        "pitch — closer to Phase 0 (40 deg/s). Faster free motion often false-trips.",
    )
    ap.add_argument(
        "--threshold",
        type=int,
        default=15,
        help="SGTHRS. Dual-frame free motion false-trips at 24–30; "
        "Phase 0 free was clean near 5–15. Raise only if jam never trips.",
    )
    ap.add_argument("--current", type=int, default=20)
    ap.add_argument("--speed", type=float, default=40.0, help="hand mode deg/s")
    ap.add_argument(
        "--settle",
        type=float,
        default=1.5,
        help="extra wait after start before min-arm distance check",
    )
    ap.add_argument(
        "--min-arm-mm",
        type=float,
        default=5.0,
        help="stroke mode: do NOT arm until frame has moved this far (filters "
        "start/accel false STALLs). Jam must be farther than this.",
    )
    ap.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="armed wait seconds (0 = auto from stroke, or 25s hand)",
    )
    ap.add_argument("--dir-b", type=int, default=1, choices=(-1, 1))
    ap.add_argument("--invert-a", type=int, default=1, choices=(0, 1))
    ap.add_argument("--invert-b", type=int, default=0, choices=(0, 1))
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    nodes_ids = [args.node_a, args.node_b]
    if args.mode == "stroke":
        cruise_deg = abs(args.speed_mm) / max(args.pitch_mm, 1e-6) * 360.0
        t_auto = abs(args.stroke_mm) / max(args.speed_mm, 0.05) + 15.0
    else:
        cruise_deg = abs(args.speed)
        t_auto = 25.0
    t_lim = args.timeout if args.timeout > 0 else t_auto

    # Line-buffer stdout so progress always appears in the terminal
    try:
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
    except Exception:
        pass

    _log("=" * 64)
    _log("Phase 1 — StallGuard host estop (dual axis)")
    _log(f"  port={args.port}  nodes={nodes_ids}  mode={args.mode}")
    _log(
        f"  SGTHRS={args.threshold}  Irun={args.current}%  "
        f"StealthChop=ON  settle={args.settle}s"
    )
    if args.mode == "stroke":
        _log(
            f"  From CURRENT pose → {args.stroke_mm:+.1f} mm @ "
            f"{args.speed_mm} mm/s (~{cruise_deg:.0f} deg/s, wait≤{t_lim:.0f}s)"
        )
        _log(
            f"  Arm only after ~{args.min_arm_mm:.1f} mm free travel "
            "(ignores early false STALLs). Jam must be beyond that."
        )
        _log(
            f"  invert_a={bool(args.invert_a)} invert_b={bool(args.invert_b)} "
            f"pitch={args.pitch_mm} mm"
        )
        _log(
            f"  At 0.5 mm/s, 50 mm takes ~100 s — heartbeats print every 2 s."
        )
    else:
        _log(f"  hand cruise {args.speed} deg/s — jam one shaft after ARMED")
    _log(f"  canstepper: {_cs.__file__}")
    _log("=" * 64)
    _log("Ctrl+C → estop all")
    _log("")

    lock = threading.Lock()
    state = {
        "armed": False,
        "tripped": False,
        "trip": None,  # type: Optional[Tuple[int, float, float]]
    }
    stall_log: List[str] = []

    with CANStepperBus.serial(args.port) as bus:
        _log("opening bus / discover …")
        time.sleep(0.3)
        found = bus.discover(timeout=2.5)
        _log(f"discover: {found}")
        for nid in nodes_ids:
            if nid not in found:
                _log(f"!! node {nid} missing — found {list(found)}")
                return 1

        na = bus.node(args.node_a)
        nb = bus.node(args.node_b)

        def on_evt(nid, event, detail, data):
            if event != Event.STALL or nid not in nodes_ids:
                return
            pos = float(data or 0.0)
            with lock:
                if not state["armed"]:
                    stall_log.append(f"node={nid} pos={pos:.2f}")
                    _log(
                        f"  [STALL ignored — not armed] node={nid} pos={pos:.2f}°"
                    )
                    return
                if state["tripped"]:
                    return
                state["tripped"] = True
                state["armed"] = False
                state["trip"] = (nid, pos, time.monotonic())
            _log(
                f"\n  *** STALL ARMED → ESTOP ALL  "
                f"(from node {nid} @ {pos:.2f}°) ***\n"
            )
            try:
                bus.estop_all()
            except Exception as exc:
                _log(f"  !! estop_all: {exc}")
                try:
                    na.estop()
                    nb.estop()
                except Exception:
                    pass

        bus.on_event(on_evt)

        _log("preparing nodes …")
        for n in (na, nb):
            _prep_sg(n, args.current, args.threshold, cruise_deg)
            _log(
                f"  node {n.node_id}: thr={n.get_param(Param.STALL_THRESHOLD)} "
                f"stealth={n.get_param(Param.STEALTHCHOP)}"
            )

        try:
            state["armed"] = False
            state["tripped"] = False
            state["trip"] = None

            if args.mode == "stroke":
                dual = IndependentDualAxis(
                    na,
                    nb,
                    rotation_distance=args.pitch_mm,
                    invert_a=bool(args.invert_a),
                    invert_b=bool(args.invert_b),
                    name="frame",
                )
                dual.apply_direction()
                for n in (na, nb):
                    n.set_stealthchop(True)
                    n.set_stall_threshold(args.threshold)
                    n.set_closed_loop(False)
                    n.set_run_current(args.current)
                    n.set_max_speed(cruise_deg)
                    # Gentle accel — hard snap false-trips StallGuard
                    n.set_acceleration(max(400.0, cruise_deg * 2.0))

                _log("\n--- Stroke into jam (automatic) ---")
                _log(
                    "  Sets zero at CURRENT position, then moves the stroke "
                    "toward the hard stop."
                )
                _log(
                    "  Pick --stroke-mm sign so travel is TOWARD the jam "
                    f"(now {args.stroke_mm:+.1f} mm)."
                )
                if abs(args.stroke_mm) <= args.min_arm_mm + 2.0:
                    _log(
                        f"  !! stroke |{args.stroke_mm}| mm is close to "
                        f"min-arm {args.min_arm_mm} mm — use a longer stroke "
                        "or lower --min-arm-mm"
                    )
                _prompt("Ready to move into jam?", args.yes)

                dual.set_zero()
                time.sleep(0.15)
                pa0, pb0 = dual.get_positions()
                _log(f"  start mm (after zero): a={pa0:+.2f} b={pb0:+.2f}")

                target_deg = dual.units_to_deg(args.stroke_mm)
                _log(f"  commanding move_to {target_deg:.1f} deg both nodes …")
                na.move_to(target_deg, blocking=False)
                nb.move_to(target_deg, blocking=False)
                _log(
                    f"  moving to {args.stroke_mm:+.1f} mm … "
                    f"settle {args.settle:.1f}s, arm after ≥{args.min_arm_mm:.1f} mm"
                )
                time.sleep(args.settle)
                _log("  settle done — watching position …")

                t0 = time.monotonic()
                last_hb = 0.0
                armed_announced = False
                while not state["tripped"]:
                    now = time.monotonic()
                    if now - t0 >= t_lim:
                        _log("\n  timeout — no armed STALL")
                        break
                    try:
                        pa, pb = dual.get_positions()
                        # Progress from start (0) toward target — use mean abs
                        progress = 0.5 * (abs(pa) + abs(pb))
                        if now - last_hb >= 2.0:
                            last_hb = now
                            sa, sb = na.ping(), nb.ping()
                            _log(
                                f"  … t={now - t0:5.1f}s  mm a={pa:+.2f} b={pb:+.2f}  "
                                f"progress~{progress:.2f}  "
                                f"moving={sa.moving}/{sb.moving}  "
                                f"armed={state['armed']}"
                            )

                        if (
                            not state["armed"]
                            and progress >= args.min_arm_mm
                        ):
                            state["armed"] = True
                            armed_announced = True
                            _log(
                                f"  PROTECTION ARMED at ~{progress:.2f} mm "
                                f"(a={pa:+.2f} b={pb:+.2f}) — waiting for jam STALL"
                            )

                        sa, sb = na.ping(), nb.ping()
                        if (
                            not sa.moving
                            and not sb.moving
                            and (now - t0) > args.settle + 1.0
                        ):
                            if (
                                abs(pa - args.stroke_mm) < 2.5
                                and abs(pb - args.stroke_mm) < 2.5
                            ):
                                _log(
                                    f"\n  reached ~target without STALL "
                                    f"(a={pa:+.2f} b={pb:+.2f} mm). "
                                    "Wrong direction, thr too low, or no jam in path."
                                )
                                break
                            if not state["armed"] and progress < args.min_arm_mm:
                                _log(
                                    f"\n  stopped early with little travel "
                                    f"(~{progress:.2f} mm) — check enable/current "
                                    "or mechanical bind / false STALL thr too high"
                                )
                                break
                    except Exception as exc:
                        if now - last_hb >= 2.0:
                            _log(f"  !! position poll: {exc}")
                            last_hb = now
                    time.sleep(0.05)

                if not armed_announced and not state["tripped"]:
                    _log(
                        "  note: never reached min-arm distance — protection "
                        "stayed disarmed (frame may not have moved enough)"
                    )

            else:
                print("\n--- Hand jam cruise ---")
                _prompt("Ready to start both motors?", args.yes)
                na.run(args.speed)
                nb.run(args.dir_b * args.speed)
                print(f"  settle {args.settle:.1f}s …")
                time.sleep(args.settle)
                if not state["tripped"]:
                    state["armed"] = True
                    print("  PROTECTION ARMED — jam ONE shaft by hand")
                t0 = time.monotonic()
                while not state["tripped"]:
                    if time.monotonic() - t0 >= t_lim:
                        print("\n  timeout — no jam")
                        break
                    time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n  interrupted")
        finally:
            state["armed"] = False
            if not state["tripped"]:
                try:
                    na.stop()
                    nb.stop()
                except Exception:
                    pass
                time.sleep(0.15)
                try:
                    bus.estop_all()
                except Exception:
                    pass

        time.sleep(0.3)
        print("\n--- After trip / stop ---")
        if stall_log:
            print(f"  ignored (disarmed) STALLs: {len(stall_log)}")
            for line in stall_log[-6:]:
                print(f"    {line}")

        ok_both = True
        for n in (na, nb):
            try:
                st = n.get_status()
                print(
                    f"  node {n.node_id}: enabled={st.enabled} "
                    f"estopped={st.estopped} moving={st.moving} fault={st.fault}"
                )
                if st.moving:
                    ok_both = False
            except Exception as exc:
                print(f"  node {n.node_id}: {exc}")
                ok_both = False

        print("\n" + "=" * 64)
        print("Phase 1 summary")
        trip = state["trip"]
        if trip:
            nid, pos, _ = trip
            # pos is encoder deg at event; rough mm from zero for open-loop test
            trip_mm = abs(pos) / 360.0 * args.pitch_mm
            print(f"  armed STALL from node {nid} @ {pos:.2f}° (~{trip_mm:.2f} mm)")
            print("  host response: estop_all()")
            print(f"  both axes not moving: {ok_both}")
            print("=" * 64)
            if trip_mm < max(3.0, args.min_arm_mm * 0.6):
                print(
                    "RESULT: FALSE_TRIP — stopped almost immediately; frame barely moved.\n"
                    "  Dual free motion is too “loud” for this SGTHRS.\n"
                    "  Retry:\n"
                    "    --threshold 10   (or 8)\n"
                    "    --min-arm-mm 5 --speed-mm 0.5\n"
                    "  Only trust a trip after the frame has clearly traveled."
                )
                return 3
            if ok_both:
                print("RESULT: PASS (estop chain) — both motors stopped on STALL.")
                print(
                    "  Confirm a REAL hard-stop/jam at that position.\n"
                    "  If free (no jam): lower --threshold or flip --stroke-mm sign."
                )
                return 0
            print("RESULT: PARTIAL — STALL seen; check moving flags")
            return 2

        print("  no armed STALL")
        print("=" * 64)
        print(
            "RESULT: NO_TRIP — try opposite sign / longer stroke / slightly higher thr:\n"
            "  --stroke-mm +25  or  -25   |  --threshold 18"
        )
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(130)
