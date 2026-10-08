#!/usr/bin/env python3
"""Phase 0 — StallGuard path check (single axis, no firmware change).

Verifies that the TMC2209 DIAG / StallGuard chain works on *your* board:

  1) Idle SG reading (UART telemetry)
  2) Free-running SG baseline (light load)
  3) Blocked / hard-stop SG response + EVT_STALL
  4) Optional sensorless home (HOME method 2)

Does **not** implement mid-move crash cut (that is Phase 1/3). Limit switches
are left alone.

Safety
------
Use low run current and moderate speed. For stage 3, either:
  - hold the *motor shaft* briefly by hand (not the screw if you can avoid it), or
  - drive slowly into a *known* hard stop you are willing to bump at low current.

If the axis is on an SFU with load, prefer current ≤ 35 % and speed ≤ 40 deg/s
for the block test.

Usage
-----
  cd can_stepper
  PYTHONPATH=. python3 examples/phase0_stallguard.py
  PYTHONPATH=. python3 examples/phase0_stallguard.py /dev/ttyACM0 1
  PYTHONPATH=. python3 examples/phase0_stallguard.py /dev/ttyACM0 1 --threshold 60
  PYTHONPATH=. python3 examples/phase0_stallguard.py /dev/ttyACM0 1 --home -1
  PYTHONPATH=. python3 examples/phase0_stallguard.py /dev/ttyACM0 1 --yes   # skip prompts

Args (positional):
  port      serial port (default /dev/ttyACM0)
  node_id   CAN node id (default 1)

Options:
  --threshold N   SGTHRS 0–255, higher = more sensitive (default 60)
  --speed DEG_S   free/block jog speed (default 30)
  --current PCT   run current % (default 30)
  --samples N     samples per stage (default 20)
  --home DIR      also run sensorless home (DIR = -1 or +1 toward hard stop)
  --yes           non-interactive (auto-continue; still stops between stages)
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from canstepper import (  # noqa: E402
    CANStepperBus,
    Event,
    HomingFailed,
    Param,
)
import canstepper as _cs  # noqa: E402


def _prompt(msg: str, auto: bool) -> None:
    if auto:
        print(f"  [auto] {msg}")
        time.sleep(0.4)
        return
    try:
        input(f"  >> {msg}  [Enter]")
    except EOFError:
        pass


def _sg(node) -> Tuple[int, object]:
    d = node.get_driver_status()
    return int(d.stallguard), d


def _sample_sg(node, n: int, period: float = 0.08) -> List[int]:
    vals: List[int] = []
    for _ in range(n):
        try:
            vals.append(_sg(node)[0])
        except Exception as exc:
            print(f"    !! SG read failed: {exc}")
            break
        time.sleep(period)
    return vals


def _stats(vals: List[int]) -> str:
    if not vals:
        return "no samples"
    return (
        f"n={len(vals)}  min={min(vals)}  max={max(vals)}  "
        f"mean={statistics.mean(vals):.1f}  median={statistics.median(vals):.1f}"
    )


def _prep(node, current: int, threshold: int, speed: float) -> None:
    # Open-loop velocity is fine for SG load sense; keep CL off so a jam does
    # not fight with PID while we only want DIAG / SG values.
    node.set_run_current(current)
    node.set_hold_current(max(10, current // 3))
    node.set_microsteps(8)
    node.set_closed_loop(False)
    node.set_stealthchop(True)  # StallGuard4 needs StealthChop on TMC2209
    node.set_stall_threshold(threshold)
    node.set_param(Param.MAX_SPEED, max(720.0, speed * 4.0))
    node.set_param(Param.ACCELERATION, 1500.0)
    node.enable()
    time.sleep(0.15)


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 0 StallGuard verification")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("node_id", nargs="?", type=int, default=1)
    ap.add_argument("--threshold", type=int, default=60, help="SGTHRS 0-255")
    ap.add_argument("--speed", type=float, default=30.0, help="jog deg/s")
    ap.add_argument("--current", type=int, default=30, help="run current %%")
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument(
        "--home",
        type=int,
        choices=(-1, 1),
        default=None,
        metavar="DIR",
        help="also run sensorless home toward hard stop (dir -1 or +1)",
    )
    ap.add_argument("--yes", action="store_true", help="skip interactive prompts")
    args = ap.parse_args()

    print("=" * 64)
    print("Phase 0 — StallGuard path check (single axis)")
    print(f"  port={args.port}  node={args.node_id}")
    print(f"  SGTHRS={args.threshold}  speed={args.speed} deg/s  Irun={args.current}%")
    print(f"  canstepper: {_cs.__file__}")
    print("=" * 64)
    print("Ctrl+C anytime → stop + disable.")
    print()

    stall_events: List[Tuple[float, float]] = []
    free_stop_stalls = 0
    results = {
        "idle_ok": False,
        "free_ok": False,
        "block_sg_shift": False,
        "block_event": False,
        "home_ok": None if args.home is None else False,
    }

    with CANStepperBus.serial(args.port) as bus:
        time.sleep(0.3)
        found = bus.discover(timeout=2.5)
        print("discover:", found)
        if args.node_id not in found:
            print(f"!! node {args.node_id} not found — pick an id from discover")
            if found:
                print(f"   example: PYTHONPATH=. python3 examples/phase0_stallguard.py "
                      f"{args.port} {next(iter(sorted(found)))}")
            return 1

        node = bus.node(args.node_id)

        def on_evt(nid, event, detail, data):
            if nid != args.node_id:
                return
            name = event.name if hasattr(event, "name") else str(event)
            print(f"  [EVENT] node={nid} {name} detail={detail} data={data}")
            if event == Event.STALL:
                stall_events.append((time.monotonic(), float(data or 0.0)))

        bus.on_event(on_evt)

        # --- board identity -------------------------------------------------
        st = node.get_status()
        print(
            f"status: fw={st.fw_major}.{st.fw_minor}  mode={st.mode}  "
            f"enabled={st.enabled}  homed={st.homed}  fault={st.fault}"
        )
        if st.fw_major < 1 or (st.fw_major == 1 and st.fw_minor < 4):
            print("!! fw < 1.4 — DRIVER/SG telemetry may be incomplete; upgrade recommended")

        _prep(node, args.current, args.threshold, args.speed)

        try:
            thr = node.get_param(Param.STALL_THRESHOLD)
            sc = node.get_param(Param.STEALTHCHOP)
            print(f"params: stall_threshold={thr}  stealthchop={sc}")
            if int(sc) == 0:
                print("  note: StealthChop was off in NVS; this script forced it ON for SG")
        except Exception as exc:
            print(f"param read skipped: {exc}")

        # --- Stage 1: idle --------------------------------------------------
        print("\n--- Stage 1: IDLE (motor enabled, not moving) ---")
        idle = _sample_sg(node, args.samples)
        print(f"  SG idle: {_stats(idle)}")
        d = node.get_driver_status()
        print(
            f"  driver: uart_ok={d.uart_ok} stealth={d.stealth_chop} "
            f"standstill={d.standstill} cs_actual={d.cs_actual} otpw={d.otpw}"
        )
        results["idle_ok"] = bool(idle) and d.uart_ok
        if not d.uart_ok:
            print("  FAIL: TMC UART not OK — fix wiring/driver before StallGuard")
        if idle and all(v == 0 for v in idle) and not d.standstill:
            print("  note: all-zero SG while moving is suspicious; at idle it can be low")

        # --- Stage 2: free run ----------------------------------------------
        # Only count EVT_STALL *during steady cruise*. Accel start and stop()
        # often glitch DIAG even when free SG is well above SGTHRS.
        print("\n--- Stage 2: FREE RUN (clear path, no jam) ---")
        print("  Ensure the screw/axis can turn freely for a few seconds.")
        print("  (stalls only counted in cruise window; start/stop glitches ignored)")
        _prompt("Ready for free jog?", args.yes)
        free: List[int] = []
        free_stalls = 0
        free_stop_stalls = 0
        try:
            node.run(args.speed)
            time.sleep(0.6)  # leave standstill + accel
            stall_events.clear()  # ignore start-up DIAG edges
            free = _sample_sg(node, args.samples)
            free_stalls = len(stall_events)
            print(f"  SG free (cruise): {_stats(free)}")
            print(f"  EVT_STALL during cruise: {free_stalls}")
        finally:
            stall_events.clear()
            node.stop()
            time.sleep(0.35)
            free_stop_stalls = len(stall_events)
            if free_stop_stalls:
                print(
                    f"  note: {free_stop_stalls} EVT_STALL at stop/decel "
                    "(ignored for free_ok — common DIAG glitch)"
                )
            stall_events.clear()

        results["free_ok"] = bool(free) and free_stalls == 0
        if free_stalls:
            print(
                "  WARN: false stall while cruising — lower SGTHRS "
                f"(try threshold {max(1, args.threshold - 5)})"
            )
        if free:
            free_med = float(statistics.median(free))
            free_min = float(min(free))
            print(
                f"  hint: free SG min={free_min:.0f} median={free_med:.0f} → "
                f"useful SGTHRS is usually a few counts below free min "
                f"(try ~{max(1, int(free_min) - 5)} … {max(1, int(free_min) - 2)})"
            )
            if args.threshold >= free_min:
                print(
                    f"  !! SGTHRS={args.threshold} ≥ free min={free_min:.0f} → "
                    "DIAG will fire continuously while moving; lower threshold"
                )
        if free and statistics.mean(free) < 1:
            print("  WARN: free SG ~0 — StealthChop off? speed too low?")

        # --- Stage 3: block / jam -------------------------------------------
        print("\n--- Stage 3: BLOCKED (jam or hard stop) ---")
        print("  Firmly lock the motor shaft so it STOPS (or hit a hard stop).")
        print("  Soft finger drag that still turns the screw will not drop SG.")
        _prompt("Ready for blocked jog? (hands clear of pinch points)", args.yes)
        block: List[int] = []
        block_stalls = 0
        try:
            node.run(args.speed)
            time.sleep(0.5)
            stall_events.clear()  # ignore start-up
            print("  >>> BLOCK THE SHAFT NOW (hold until samples finish) <<<")
            t_end = time.monotonic() + max(2.5, args.samples * 0.08)
            while time.monotonic() < t_end:
                try:
                    block.append(_sg(node)[0])
                except Exception as exc:
                    print(f"    !! SG read failed: {exc}")
                    break
                time.sleep(0.08)
            block_stalls = len(stall_events)
            print(f"  SG blocked (cruise): {_stats(block)}")
            print(f"  EVT_STALL during cruise: {block_stalls}")
        finally:
            stall_events.clear()
            node.stop()
            time.sleep(0.35)
            stop_n = len(stall_events)
            if stop_n:
                print(f"  note: {stop_n} EVT_STALL at stop (ignored)")
            stall_events.clear()

        results["block_event"] = block_stalls > 0

        free_med = float(statistics.median(free)) if free else 0.0
        block_med = float(statistics.median(block)) if block else 0.0
        drop = free_med - block_med
        if free and block:
            print(
                f"  median free={free_med:.1f}  blocked={block_med:.1f}  "
                f"drop={drop:.1f}  free_min={min(free)}  block_min={min(block)}"
            )
            results["block_sg_shift"] = drop >= 5.0 or (
                min(block) <= max(2.0, min(free) - 5.0)
            )

        # Judgment: free cruise clean + (block event OR SG drop) = usable SG
        if free_stalls > 0:
            print("  Stage 3 not decisive until free cruise is clean.")
        elif results["block_event"] and results["block_sg_shift"]:
            print("  PASS: free clean + jam drops SG and trips DIAG")
        elif results["block_event"]:
            print(
                "  PARTIAL: DIAG trip on block but SG median barely moved — "
                "OK for crude detect if free stays clean; re-test with harder lock"
            )
        elif results["block_sg_shift"]:
            print(
                "  PARTIAL: SG dropped under jam but no DIAG — raise SGTHRS a few "
                f"counts (try {min(255, args.threshold + 3)}…{min(255, args.threshold + 8)})"
            )
        else:
            print(
                "  FAIL: no cruise DIAG and no SG drop under jam. "
                "Harder lock, lower current, or SG may not separate free/jam on this axis."
            )

        # --- Stage 4: optional sensorless home ------------------------------
        if args.home is not None:
            print("\n--- Stage 4: SENSORLESS HOME (optional) ---")
            print(f"  Direction {args.home:+d} must point toward a HARD STOP.")
            print("  Homing uses ~30% current, force-stop + zero + backoff.")
            _prompt("Hard stop clear and direction correct?", args.yes)
            node.set_stall_threshold(args.threshold)
            node.configure_homing(
                current_percent=min(35, args.current),
                backoff_deg=8.0,
                timeout_ms=25000,
            )
            # home runs its own open-loop seek in firmware
            try:
                node.home(
                    method="stallguard",
                    direction=args.home,
                    speed_deg_s=max(15.0, min(args.speed, 35.0)),
                    timeout=40.0,
                )
                st2 = node.get_status()
                print(
                    f"  HOMING_DONE  pos={node.get_position():.2f}°  "
                    f"homed={st2.homed}"
                )
                results["home_ok"] = True
            except HomingFailed as exc:
                print(f"  HOMING_FAILED: {exc}")
                print(
                    "  tips: raise --threshold if never hits; lower if false trip; "
                    "check direction toward stop; speed ~20–35 deg/s"
                )
                results["home_ok"] = False
            except Exception as exc:
                print(f"  home error: {exc}")
                results["home_ok"] = False

        # --- cleanup --------------------------------------------------------
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.disable()
        except Exception:
            pass

    # --- summary ------------------------------------------------------------
    print("\n" + "=" * 64)
    print("Phase 0 summary")
    print(f"  idle telemetry OK : {results['idle_ok']}")
    print(f"  free cruise OK    : {results['free_ok']}  (no EVT_STALL in cruise)")
    print(f"  SG load shift     : {results['block_sg_shift']}")
    print(f"  EVT_STALL on jam  : {results['block_event']}  (cruise window only)")
    if results["home_ok"] is not None:
        print(f"  sensorless home   : {results['home_ok']}")
    print("=" * 64)

    # Usable jam detect: free quiet + (DIAG or clear SG drop under jam)
    usable = (
        results["idle_ok"]
        and results["free_ok"]
        and (results["block_event"] or results["block_sg_shift"])
    )
    diag_only = results["idle_ok"] and (
        results["block_event"] or free_stop_stalls > 0 or not results["free_ok"]
    )

    if usable and results["block_event"]:
        print("RESULT: PASS — free clean + jam trips DIAG. Phase 1 is reasonable.")
        print("Next: host Event.STALL → stop/estop (ignore start/stop glitches).")
        return 0
    if usable:
        print(
            "RESULT: PARTIAL — free clean + SG shifts under jam, no DIAG yet. "
            "Nudge --threshold up a few counts and re-test."
        )
        return 2
    if results["idle_ok"] and results["free_ok"] and not results["block_event"]:
        print(
            "RESULT: FREE_OK_ONLY — cruise is quiet but jam not detected. "
            "Harder shaft lock / lower --current / slightly higher --threshold."
        )
        return 2
    if diag_only:
        print(
            "RESULT: DIAG_LIVE — wiring/events work, but free/jam not separated yet "
            "(or only start/stop glitches). Re-run this updated script; do not start Phase 1."
        )
        return 2
    print("RESULT: FAIL — fix UART/StealthChop before more SG tests.")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(130)
