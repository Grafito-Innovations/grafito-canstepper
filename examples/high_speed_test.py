"""High-speed motor test — find your motor's practical speed ceiling.

Runs a velocity ladder (open-loop) ramping through increasing speeds, then a
closed-loop position-move ladder, measuring the *actual* shaft speed with the
encoder at each rung so you can see exactly where the motor stops tracking.

The script is NON-DESTRUCTIVE: it snapshots the node's parameters at start and
restores + persists them at the end, so running it never drifts your config.

What to expect:
  - At low/mid speeds the "measured" column tracks "commanded" closely.
  - As speed rises, inductance + supply voltage limit torque; "measured" starts
    to lag "commanded" and eventually the motor stalls (drops to ~0). The last
    rung that still tracks is your reliable ceiling for this voltage/current.
  - Raise run_current, lower microsteps, or disable StealthChop to push higher.

Connect the USB port of any board on the chain, then:

  python3 examples/high_speed_test.py
  python3 examples/high_speed_test.py /dev/ttyACM0 1          # port, node
  python3 examples/high_speed_test.py /dev/ttyACM0 1 80       # + run_current %
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import CANStepperBus, NodeFault, Param, RequestTimeout  # noqa: E402
from canstepper.protocol import PARAMS  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE = int(sys.argv[2]) if len(sys.argv) > 2 else 1
RUN_CURRENT = int(sys.argv[3]) if len(sys.argv) > 3 else 70

# deg/s targets for the open-loop velocity ladder (deg/s / 6 = RPM).
SPEEDS = [180, 360, 720, 1080, 1440, 1800, 2400, 3000, 3600, 4800, 6000]

# Closed-loop cruise-speed ladder (deg/s). Firmware ≥1.2 trap+v_ff can reach
# near the open-loop ceiling; raise cl_max_speed per rung.
CL_SPEEDS = [180, 360, 720, 1080, 1440, 1800, 2160, 2700, 3600]

STALL_RATIO = 0.5      # measured < 50% of commanded => considered a stall
TRACK_RATIO = 0.85     # >= 85% => "tracking" (good)
SETTLE = 0.6           # seconds to let each rung reach cruise before sampling


def snapshot(node):
    return {p: node.get_param(p) for p in PARAMS}


def restore(node, snap):
    for p, v in snap.items():
        try:
            node.set_param(p, v)
        except Exception:
            pass


def measured_speed(node, settle=0.6, sample=0.5):
    """Encoder-derived shaft speed (deg/s) while running."""
    p0 = node.get_position()
    t0 = time.monotonic()
    time.sleep(sample)
    p1 = node.get_position()
    return (p1 - p0) / (time.monotonic() - t0), p1


def banner(title):
    print("\n" + "=" * 64 + f"\n {title}\n" + "=" * 64)


def velocity_ladder(node):
    banner(f"OPEN-LOOP velocity ladder  (run_current={RUN_CURRENT}%, "
           f"microsteps={node.get_param(Param.MICROSTEPS)}, SpreadCycle)")
    print(f" {'cmd deg/s':>10} {'cmd RPM':>9} | {'meas deg/s':>11} {'meas RPM':>9} "
          f"{'ratio':>6}  verdict\n " + "-" * 70)

    last_good = 0
    for target in SPEEDS:
        node.set_zero()
        node.run(float(target))
        time.sleep(SETTLE)
        try:
            meas, _pos = measured_speed(node)
        except Exception:
            meas = 0.0
        node.stop()
        time.sleep(0.4)
        ratio = meas / target if target else 0
        if ratio >= TRACK_RATIO:
            verdict, last_good = "tracking", target
        elif ratio >= STALL_RATIO:
            verdict = "lagging"
        else:
            verdict = "STALL"
        print(f" {target:10d} {target/6:9.0f} | {meas:11.1f} {meas/6:9.0f} "
              f"{ratio:6.2f}  {verdict}")
        if ratio < STALL_RATIO:
            print("   -> motor stalled; stopping the open-loop ladder here.")
            break
    print(f"\n reliable open-loop ceiling: ~{last_good} deg/s ({last_good/6:.0f} RPM)")
    return last_good


def closed_loop_ladder(node):
    banner("CLOSED-LOOP position-move ladder  (raises cl_max_speed per rung)")
    print(f" {'cruise deg/s':>13} {'cruise RPM':>11} | {'result':>22}  "
          f"{'reached':>10}\n " + "-" * 64)

    last_good = 0
    target = 720.0  # two revolutions
    for cruise in CL_SPEEDS:
        node.configure_closed_loop_speed(float(cruise), accel_factor=4.0)
        node.set_zero()
        reached = ""
        try:
            t0 = time.monotonic()
            node.move_to(target, blocking=True,
                         timeout=max(12.0, target / max(cruise, 1) * 4))
            dt = time.monotonic() - t0
            pos = node.get_position()
            ok = abs(pos - target) < 10.0
            verdict = f"reached in {dt:.2f}s" if ok else f"missed ({pos:.0f}°)"
            reached = f"{pos:.0f}°"
            if ok:
                last_good = cruise
        except RequestTimeout:
            verdict = "no settle (PID)"  # didn't reach MOVE_DONE in time
            node.enable()
        except NodeFault as e:
            verdict = f"FAULT {e}"
            node.enable()  # clear latch / fault to continue
        except Exception as e:
            verdict = f"error {type(e).__name__}"
            node.enable()
        node.stop()
        print(f" {cruise:13d} {cruise/6:11.0f} | {verdict:>22}  {reached:>10}")
        time.sleep(0.3)
    print(f"\n reliable closed-loop cruise: ~{last_good} deg/s ({last_good/6:.0f} RPM)")
    return last_good


def main():
    print(f"== GrafitoCANStepper high-speed test :: {PORT} node {NODE} ==")
    bus = CANStepperBus.serial(PORT)
    snap = {}
    try:
        time.sleep(0.4)
        node = bus.node(NODE)
        found = bus.discover(timeout=2.0)
        # discover() can occasionally miss on a fresh open; ping is authoritative.
        try:
            fw = node.ping(timeout=2.0).firmware
        except Exception:
            fw = None
        if NODE not in found and not fw:
            print(f"node {NODE} not found on {PORT} (discovered {found}); abort.")
            return
        print(f"node {NODE}: firmware {fw or found[NODE]}  "
              f"(dir invert already persisted = {node.get_param(Param.INVERT_DIR)})")

        snap = snapshot(node)
        print(f"snapshot taken ({len(snap)} params) — will be restored at exit\n")

        # High-speed friendly baseline: more current, fewer microsteps, high
        # accel to punch through the low-speed resonance band, SpreadCycle.
        (node.set_run_current(RUN_CURRENT).set_hold_current(20)
         .set_microsteps(8).set_acceleration(20000.0)
         .set_stealthchop(False).set_closed_loop(False))
        node.enable()
        node.ping()
        time.sleep(0.3)

        ol_ceiling = velocity_ladder(node)

        # Closed-loop section: trap + velocity FF (fw ≥1.2) with per-rung
        # configure_closed_loop_speed(). Driver prep (current/µsteps/SpreadCycle)
        # carries over from the open-loop ladder above.
        node.set_closed_loop(True)
        node.enable()
        node.ping()
        time.sleep(0.2)
        cl_ceiling = closed_loop_ladder(node)

        banner("SUMMARY")
        print(f"  open-loop  reliable ceiling: ~{ol_ceiling} deg/s "
              f"({ol_ceiling/6:.0f} RPM)")
        print(f"  closed-loop reliable cruise: ~{cl_ceiling} deg/s "
              f"({cl_ceiling/6:.0f} RPM)")
        print("  (raise run_current / lower microsteps / 24V supply to go faster)")

    except KeyboardInterrupt:
        print("\n!! interrupted")
    finally:
        try:
            bus.estop_all()
            if snap:
                restore(bus.node(NODE), snap)
                bus.node(NODE).save_config()
                print(f"\n(restored + persisted {len(snap)} params to pre-test state)")
            bus.node(NODE).disable()
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    main()
