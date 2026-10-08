#!/usr/bin/env python3
"""Live IO8 endstop monitor (no motor motion).

Prints a heartbeat every 0.5s so you can see the script is alive, and prints
immediately on every change / ENDSTOP_HIT / ENDSTOP_RELEASED event.

Polarity (firmware ENDSTOP_ACTIVE_HIGH):

  active-low  (default, NO / NPN sink when hit)::
    idle HIGH → endstop_active=False
    hit  LOW  → endstop_active=True

  active-high (NC optical, or inverted sensors)::
    idle LOW  → endstop_active=False
    hit  HIGH → endstop_active=True

Usage:
  python3 examples/endstop_monitor.py [/dev/ttyACM0] [node_id] [seconds|0|inf] [low|high]

  # Continuous until Ctrl+C (recommended)
  python3 examples/endstop_monitor.py /dev/ttyACM0 1 0 high

  # NPN NO / switch to GND when hit, 60s then exit
  python3 examples/endstop_monitor.py /dev/ttyACM0 1 60 low

  # NC / inverted optical (HIGH = hit), continuous
  python3 examples/endstop_monitor.py /dev/ttyACM0 1 0 high
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Prefer this repo's canstepper over an older site-packages install
_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from canstepper import CANStepperBus, EndstopAction, Event, Param
import canstepper as _cs

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
_sec_raw = (sys.argv[3] if len(sys.argv) > 3 else "0").strip().lower()
# 0 / inf / forever / -1 → run until Ctrl+C
if _sec_raw in ("0", "inf", "infinite", "forever", "-1", "cont", "continuous"):
    SECONDS = 0.0  # 0 = continuous
else:
    SECONDS = float(_sec_raw)
# 4th arg: "low" | "high" | "0" | "1"  — active-low vs active-high
_pol = (sys.argv[4] if len(sys.argv) > 4 else "high").strip().lower()
ACTIVE_HIGH = _pol in ("high", "1", "true", "ah", "active-high", "active_high")


def _raw_high(st) -> str:
    """fw ≥1.5 exposes home_raw_high; older packages omit it."""
    raw = getattr(st, "home_raw_high", None)
    if raw is None:
        return "?"
    return "HIGH" if raw else "LOW"


def main() -> None:
    dur = "continuous (Ctrl+C to stop)" if SECONDS <= 0 else f"{SECONDS:.0f}s"
    print(f"port={PORT}  node={NODE_ID}  duration={dur}")
    print(f"canstepper: {_cs.__file__}")
    if ACTIVE_HIGH:
        print("polarity: ACTIVE-HIGH  (HIGH→triggered, LOW→idle)  [NC / inverted]")
    else:
        print("polarity: ACTIVE-LOW   (LOW→triggered, HIGH→idle)  [NO / NPN]")
    print("tip: if sense is inverted, re-run with arg 'high' or 'low'")
    print("-" * 56)

    with CANStepperBus.serial(PORT) as bus:
        nodes = bus.discover()
        print("discover", nodes)
        if not nodes:
            print("!! no nodes found — check USB + 24V power")
            sys.exit(1)
        nid = NODE_ID if NODE_ID in nodes else next(iter(nodes))
        if nid != NODE_ID:
            print(f"!! node {NODE_ID} missing, using {nid}")
        node = bus.node(nid)

        node.configure_endstop(
            enabled=True,
            active_high=ACTIVE_HIGH,
            action=EndstopAction.REPORT,
        )
        print(
            "params:",
            f"ENABLE={node.get_param(Param.ENDSTOP_ENABLE)}",
            f"ACTIVE_HIGH={node.get_param(Param.ENDSTOP_ACTIVE_HIGH)}",
            f"ACTION={node.get_param(Param.ENDSTOP_ACTION)}",
        )

        hits = releases = 0

        def on_evt(node_id: int, event: Event, detail: int, data: float) -> None:
            nonlocal hits, releases
            if node_id != nid:
                return
            if event == Event.ENDSTOP_HIT:
                hits += 1
                print(f"  ** EVENT ENDSTOP_HIT      pos={data:.2f}°")
            elif event == Event.ENDSTOP_RELEASED:
                releases += 1
                print(f"  ** EVENT ENDSTOP_RELEASED pos={data:.2f}°")
            sys.stdout.flush()

        bus.on_event(on_evt)

        last = None
        changes = 0
        true_seen = False
        false_seen = False
        t0 = time.monotonic()
        next_hb = t0
        try:
            while True:
                if SECONDS > 0 and (time.monotonic() - t0) >= SECONDS:
                    break
                st = node.get_status()
                active = bool(st.endstop_active)
                now = time.monotonic()
                raw_s = _raw_high(st)
                if active:
                    true_seen = True
                else:
                    false_seen = True
                if active != last:
                    changes += 1
                    print(
                        f"[{time.strftime('%H:%M:%S')}] CHANGE  "
                        f"endstop_active={active}  gpio8_raw={raw_s}  "
                        f"pos={node.get_position():.2f}°"
                    )
                    last = active
                    sys.stdout.flush()
                elif now >= next_hb:
                    # Heartbeat even when stuck — proves the script is running
                    print(
                        f"[{time.strftime('%H:%M:%S')}]        "
                        f"endstop_active={active}  gpio8_raw={raw_s}  "
                        f"pos={node.get_position():.2f}°"
                    )
                    sys.stdout.flush()
                    next_hb = now + 0.5
                time.sleep(0.04)
        except KeyboardInterrupt:
            print("\n(interrupted — continuous monitor stopped)")

        print("-" * 56)
        print(
            f"summary: changes={changes}  hits={hits}  releases={releases}  "
            f"saw_True={true_seen}  saw_False={false_seen}"
        )
        if true_seen and false_seen and changes >= 2:
            print("OK — switch is toggling.")
        elif true_seen or false_seen:
            print("Only one level seen; toggle the sensor if you expected both.")
        else:
            print("NO samples — check connection.")


if __name__ == "__main__":
    main()
