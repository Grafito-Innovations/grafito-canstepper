"""Configure and verify high-speed closed-loop trapezoid moves.

Firmware ≥1.2 plans a rest-to-rest trapezoid (cl_max_speed / cl_max_accel)
with velocity feedforward and a light tracking PID, so the motor can cruise
near open-loop physical limits and still settle. This script applies a
high-speed tune via ``configure_closed_loop_speed()`` and verifies with
encoder-measured position moves.

Two modes:

  1. Configure ONE target speed (apply + persist so it sticks across reboots):

       python3 examples/closed_loop_speed.py /dev/ttyACM0 1 1440

  2. Sweep a ladder of speeds to find your motor's reliable closed-loop ceiling
     (non-destructive — restores your original config afterwards):

       python3 examples/closed_loop_speed.py

Also see ``tools/cl_speed_validate.py`` for an open-loop vs closed-loop comparison.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import CANStepperBus, NodeFault, Param, RequestTimeout  # noqa: E402
from canstepper.protocol import PARAMS  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE = int(sys.argv[2]) if len(sys.argv) > 2 else 1
TARGET = int(sys.argv[3]) if len(sys.argv) > 3 else None  # deg/s, or None=sweep

# High-speed driver prep (NEMA17 @ ~24 V — see docs/closed_loop_tuning.md)
# Measured: 70% / 8µs / SpreadCycle settles through 6000 deg/s; 100% is worse.
RUN_CURRENT = 70
MICROSTEPS = 8
STEALTHCHOP = False
MOVE_DEG = 1440.0  # four revolutions
SWEEP_SPEEDS = [360, 720, 1440, 2160, 3600, 4800, 6000]


def snapshot(node):
    return {p: node.get_param(p) for p in PARAMS}


def restore(node, snap):
    for p, v in snap.items():
        try:
            node.set_param(p, v)
        except Exception:
            pass


def apply_tune(node, cruise):
    node.configure_closed_loop_speed(
        float(cruise),
        run_current=RUN_CURRENT,
        microsteps=MICROSTEPS,
        stealthchop=STEALTHCHOP,
    )
    node.enable()
    node.ping()
    time.sleep(0.25)
    node.set_zero()
    try:
        node.move_to(90.0, blocking=True, timeout=5.0)
    except Exception:
        pass


def verify_move(node, cruise):
    node.set_zero()
    try:
        t0 = time.monotonic()
        node.move_to(MOVE_DEG, blocking=True,
                     timeout=max(12.0, MOVE_DEG / max(cruise, 1) * 4))
        dt = time.monotonic() - t0
        pos = node.get_position()
        pid = node.get_pid_status()
        ok = abs(pos - MOVE_DEG) < 8.0 and pid.state == 2 and pid.fault == 0
        return ok, f"reached {pos:.1f}° in {dt:.2f}s (pid={pid.state})"
    except RequestTimeout:
        node.enable()
        return False, "no settle (PID)"
    except NodeFault as e:
        node.enable()
        return False, f"FAULT {e}"


def main():
    print(f"== GrafitoCANStepper closed-loop speed config :: {PORT} node {NODE} ==")
    bus = CANStepperBus.serial(PORT)
    snap = {}
    try:
        time.sleep(0.4)
        node = bus.node(NODE)
        try:
            fw = node.ping(timeout=2.0).firmware
        except Exception:
            fw = None
        if not fw:
            print(f"node {NODE} not reachable on {PORT}; abort.")
            return
        print(f"node {NODE}: firmware {fw}")
        major_minor = tuple(int(x) for x in fw.split(".")[:2]) if fw else (0, 0)
        if major_minor < (1, 2):
            print(f"NOTE: fw {fw} lacks trap+v_ff closed-loop (need ≥1.2). "
                  "Expect a lower ceiling until you reflash.")
        snap = snapshot(node)

        if TARGET is not None:
            print(f"\nconfiguring closed loop for {TARGET} deg/s "
                  f"({TARGET/6:.0f} RPM)...")
            apply_tune(node, TARGET)
            ok, detail = verify_move(node, TARGET)
            print(f"  verify: {detail}")
            if ok:
                node.save_config()
                print(f"  PERSISTED — cruise={TARGET} deg/s via "
                      f"configure_closed_loop_speed()")
            else:
                restore(node, snap)
                node.save_config()
                print(f"  verify FAILED at {TARGET} deg/s — original config "
                      "restored. Try lower target, more current, or fewer µsteps.")
        else:
            print("\nsweeping closed-loop cruise speeds (non-destructive)...")
            print(f" {'cruise':>8} {'RPM':>7} | {'result':<34} verdict")
            print(" " + "-" * 60)
            last_good = 0
            for cruise in SWEEP_SPEEDS:
                apply_tune(node, cruise)
                ok, detail = verify_move(node, cruise)
                verdict = "OK" if ok else ("SETTLE" if "settle" in detail else "FAIL")
                if ok:
                    last_good = cruise
                print(f" {cruise:8d} {cruise/6:7.0f} | {detail:<34} {verdict}")
                time.sleep(0.2)
            print(f"\n reliable closed-loop ceiling: ~{last_good} deg/s "
                  f"({last_good/6:.0f} RPM)")
            print(" apply permanently with: "
                  f"python3 examples/closed_loop_speed.py {PORT} {NODE} {last_good or 1440}")
            restore(node, snap)
            node.save_config()
            print("\n(restored original config)")

        bus.estop_all()
        node.disable()
    except KeyboardInterrupt:
        print("\n!! interrupted")
    finally:
        try:
            bus.node(NODE).disable()
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    main()
