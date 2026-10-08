#!/usr/bin/env python3
"""Test whether GPIO8 level at reset breaks ESP32-C3 SPI boot.

ESP32-C3 samples GPIO8 only at reset. SPI boot wants GPIO8 HIGH.
This script:

  1. Reads current HOME level (gpio8_raw)
  2. Hard-resets the chip (USB serial RTS / esptool-style)
  3. Waits for the port and GCSP discover
  4. Repeats N times

Run once with the switch in the **idle/off** state (your gpio8_raw=LOW),
then optionally again with the switch **hit** (gpio8_raw=HIGH) and compare.

Usage:
  python3 examples/endstop_boot_test.py [/dev/ttyACM0] [node_id] [trials] [high|low]

  # Idle/off state (active-high polarity so LOW = idle) — the risky case
  python3 examples/endstop_boot_test.py /dev/ttyACM0 1 5 high

  # Leave sensor triggered (HIGH) for comparison
  # (trigger sensor first, then run)
  python3 examples/endstop_boot_test.py /dev/ttyACM0 1 5 high
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import serial
from serial.tools import list_ports

from canstepper import CANStepperBus, EndstopAction, Param
import canstepper as _cs

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
TRIALS = int(sys.argv[3]) if len(sys.argv) > 3 else 5
_pol = (sys.argv[4] if len(sys.argv) > 4 else "high").strip().lower()
ACTIVE_HIGH = _pol in ("high", "1", "true", "ah", "active-high", "active_high")


def _port_exists(port: str) -> bool:
    return any(p.device == port for p in list_ports.comports())


def _hard_reset(port: str) -> None:
    """Pulse RTS to reset ESP32-C3 USB-JTAG/serial (same idea as esptool)."""
    # Close any lingering handles first by opening exclusively
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    try:
        # Classic ESP auto-reset: RTS low holds chip in reset on many boards
        ser.setDTR(False)
        ser.setRTS(True)   # assert reset
        time.sleep(0.1)
        ser.setRTS(False)  # release reset → strapping latched
        time.sleep(0.05)
    finally:
        ser.close()
    # USB re-enumeration can take a moment
    time.sleep(0.3)


def _wait_port(port: str, timeout: float = 8.0) -> bool:
    t0 = time.monotonic()
    # First wait for possible disappear
    while time.monotonic() - t0 < 1.5:
        if not _port_exists(port):
            break
        time.sleep(0.05)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if _port_exists(port):
            time.sleep(0.4)  # settle after enum
            return True
        time.sleep(0.1)
    return False


def _snapshot(port: str, node_id: int) -> dict:
    with CANStepperBus.serial(port) as bus:
        nodes = bus.discover(timeout=1.5)
        if not nodes:
            return {"ok": False, "error": "discover empty"}
        nid = node_id if node_id in nodes else next(iter(nodes))
        n = bus.node(nid)
        n.configure_endstop(
            enabled=True,
            active_high=ACTIVE_HIGH,
            action=EndstopAction.REPORT,
        )
        st = n.get_status()
        raw = getattr(st, "home_raw_high", None)
        return {
            "ok": True,
            "fw": st.firmware,
            "nodes": nodes,
            "endstop_active": st.endstop_active,
            "home_raw_high": raw,
            "raw_s": "?" if raw is None else ("HIGH" if raw else "LOW"),
        }


def main() -> None:
    print(f"port={PORT}  node={NODE_ID}  trials={TRIALS}")
    print(f"canstepper: {_cs.__file__}")
    print(
        f"polarity: active_high={ACTIVE_HIGH}  "
        f"({'HIGH=hit LOW=idle' if ACTIVE_HIGH else 'LOW=hit HIGH=idle'})"
    )
    print()
    print("ESP32-C3: GPIO8 must be HIGH at reset for reliable SPI boot.")
    print("This test resets the chip while the HOME pin is at its CURRENT level.")
    print("-" * 60)

    if not _port_exists(PORT):
        print(f"!! {PORT} not present — plug in the board first")
        sys.exit(1)

    try:
        before = _snapshot(PORT, NODE_ID)
    except Exception as e:
        print(f"!! cannot talk to board before reset: {e}")
        sys.exit(1)

    if not before["ok"]:
        print(f"!! before reset: {before}")
        sys.exit(1)

    print(
        f"BEFORE reset: fw={before['fw']}  gpio8_raw={before['raw_s']}  "
        f"endstop_active={before['endstop_active']}"
    )
    if before["home_raw_high"] is False:
        print("  → Pin is LOW now (your switch-off idle). THIS is the risky case.")
    elif before["home_raw_high"] is True:
        print("  → Pin is HIGH now. Boot should usually succeed.")
    else:
        print("  → raw level unknown (old library); still testing reset recovery.")
    print()
    print(f"Running {TRIALS} reset + rediscover cycles …")
    print("-" * 60)

    ok = 0
    fail = 0
    results = []

    for i in range(1, TRIALS + 1):
        print(f"\n[trial {i}/{TRIALS}] hard-reset …")
        try:
            _hard_reset(PORT)
        except Exception as e:
            print(f"  reset pulse failed: {e}")
            fail += 1
            results.append(("reset_fail", str(e)))
            continue

        if not _wait_port(PORT, timeout=10.0):
            print(f"  FAIL: {PORT} did not reappear within 10s (boot/USB enum)")
            fail += 1
            results.append(("no_port", None))
            # wait a bit longer in case of slow recovery
            if _wait_port(PORT, timeout=5.0):
                print("  (port appeared late — counting as recovery)")
            else:
                continue

        # small delay so firmware finishes init
        time.sleep(0.6)
        try:
            after = _snapshot(PORT, NODE_ID)
        except Exception as e:
            print(f"  FAIL: port up but protocol error: {e}")
            fail += 1
            results.append(("proto_fail", str(e)))
            time.sleep(1.0)
            continue

        if after.get("ok"):
            ok += 1
            print(
                f"  OK  fw={after['fw']}  gpio8_raw={after['raw_s']}  "
                f"endstop_active={after['endstop_active']}"
            )
            results.append(("ok", after["raw_s"]))
        else:
            fail += 1
            print(f"  FAIL: {after}")
            results.append(("discover_fail", after))

        time.sleep(0.4)

    print()
    print("=" * 60)
    print(f"RESULT: {ok}/{TRIALS} boots recovered after reset")
    print(f"        {fail} failures")
    pin = before.get("raw_s", "?")
    if fail == 0:
        print(
            f"\nNo failures in {TRIALS} trials with gpio8={pin} at reset.\n"
            "  → This level looks OK on THIS board/setup (still not a guarantee\n"
            "    for every cold power-on, but a good sign)."
        )
    else:
        print(
            f"\nFailures while gpio8 was {pin} at reset.\n"
            "  → Treat idle/off (if that is LOW) as a real boot risk.\n"
            "  → Prefer rewiring so REST = HIGH, or power the sensor after MCU boot."
        )
    if pin == "LOW" and fail == 0:
        print(
            "\nNote: chip still booted with GPIO8 LOW on these USB resets.\n"
            "Some boards tolerate it; cold 24V+USB power-on can still differ.\n"
            "Optional: unplug USB+power fully, leave switch OFF, plug in again 5×."
        )
    print()
    print("Compare: trigger the sensor (gpio8 HIGH), re-run the same command.")


if __name__ == "__main__":
    main()
