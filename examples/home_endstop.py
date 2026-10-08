#!/usr/bin/env python3
"""Physical endstop homing on IO8 (HOME pin).

Wiring (recommended — active-low, normally-open to GND):

  3.3V -- 10k --+-- IO8 (GPIO 8)
                |
             100–330 Ω series (optional; keep small)
                |
             [NO switch]
                |
               GND

IO8 is an ESP32-C3 strapping pin: the switch must leave IO8 HIGH at power-on
(open at rest). A closed switch holding IO8 LOW can prevent boot.

Avoid a large series resistor (e.g. 4.7k) with a strong external 10k pull-up —
that forms a voltage divider and the pin may never read a solid LOW.

Usage:
  PYTHONPATH=. python3 examples/home_endstop.py [/dev/ttyACM0] [node_id] [dir]

  dir: -1 (default) or +1 — travel direction toward the switch

Example:
  PYTHONPATH=. python3 examples/home_endstop.py /dev/ttyACM0 1 -1
"""

from __future__ import annotations

import sys
import time

from canstepper import CANStepperBus, EndstopAction, HomingFailed, Param

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
DIRECTION = int(sys.argv[3]) if len(sys.argv) > 3 else -1
# 4th arg: polarity low|high (default low = NO/NPN; high = NC optical inverted)
_pol = (sys.argv[4] if len(sys.argv) > 4 else "low").strip().lower()
ACTIVE_HIGH = _pol in ("high", "1", "true", "ah", "active-high", "active_high")


def prep(node) -> None:
    node.set_run_current(40).set_hold_current(15).set_microsteps(8)
    node.set_closed_loop(True)
    node.set_param(Param.CL_MAX_SPEED, 1200)
    node.set_param(Param.MAX_SPEED, 1200)
    node.set_param(Param.CL_MAX_ACCEL, 4000)
    node.set_param(Param.ACCELERATION, 4000)
    node.enable()


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        node = bus.node(NODE_ID)
        prep(node)

        # Endstop on IO8 — polarity: low=NO/NPN sink, high=NC / inverted optical
        print(f"endstop polarity: active_high={ACTIVE_HIGH}")
        node.configure_endstop(
            enabled=True,
            active_high=ACTIVE_HIGH,
            action=EndstopAction.STOP,  # REPORT=0, STOP=1, STOP_AND_ZERO=2
        )
        node.configure_homing(
            current_percent=30,  # softer seek; 0 = keep run current
            backoff_deg=5.0,  # retreat after hit (degrees)
            timeout_ms=30000,
        )

        st = node.get_status()
        print(
            f"before home: pos={node.get_position():.2f}°  "
            f"endstop_active={st.endstop_active}  homed={st.homed}"
        )
        if st.endstop_active:
            print(
                "!! endstop already active — back away from the switch first, "
                "or check polarity / wiring"
            )

        print(f"homing endstop  direction={DIRECTION:+d}  speed=20 deg/s …")
        try:
            node.home(
                method="endstop",
                direction=DIRECTION,
                speed_deg_s=20.0,
                timeout=45.0,
            )
        except HomingFailed as e:
            print("!! homing failed:", e)
            sys.exit(1)

        st = node.get_status()
        print(
            f"after home:  pos={node.get_position():.2f}°  "
            f"endstop_active={st.endstop_active}  homed={st.homed}"
        )

        # Short move away from the switch to prove the new zero
        away = 30.0 if DIRECTION < 0 else -30.0
        print(f"move_to {away}° (away from home) …")
        node.move_to(away, blocking=True)
        print(f"position now {node.get_position():.2f}°")
        time.sleep(0.3)
        node.disable()
        print("done")


if __name__ == "__main__":
    main()
