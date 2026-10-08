# Closed-loop speed tuning (firmware ≥1.2)

This guide explains how closed-loop position moves work on Grafito CANStepper,
why open-loop `run()` can still look “faster,” how to push closed-loop cruise
toward the motor’s physical ceiling, and **all observations from hardware
characterization** on a real NEMA 17 motor (model below).

Related docs: [quickstart](quickstart.md) · [protocol](protocol.md) ·
[Klipper kinematics (background)](https://www.klipper3d.org/Kinematics.html)

---

## 0. Device under test (characterization motor)

All tables and soak numbers in this document were measured on the following
stepper (unless a section explicitly says otherwise). Results will scale with
supply voltage, mechanical load, and reflected inertia — re-validate on your
mechanism.

### Motor

| Item | Spec |
|------|------|
| **Type** | NEMA 17 stepper motor |
| **Model No.** | **PR42HS40-1204AF-02** |
| **Step angle** | 1.8° (200 full steps/rev) |
| **Shaft** | D-type |
| **Shaft diameter** | 4.5 mm |
| **Shaft length** | 19 mm |
| **Frame size** | 42 × 42 mm |
| **Motor length** | 40 mm |
| **Phases** | 2 |
| **Lead wires** | 4 |
| **Rated current** | **1.2 A** |
| **Holding torque** | 4.2 kg·cm |
| **Detent torque** | 0.22 kg·cm |
| **Inductance** | **3.2 mH** (low-inductance → good high-speed headroom on 24 V) |
| **Rotor inertia** | 54 g·cm² |
| **Net weight** | 250 g |
| **Shipping weight** | 0.283 kg |
| **Shipping dimensions** | 7 × 6 × 6 cm |

### Controller / drive stack

| Item | Spec |
|------|------|
| Board | Grafito CANStepper (ESP32-C3 + TMC2209 + MT6701 + CAN) |
| Firmware | **GCSP v1, fw 1.2** (trap trajectory + velocity feedforward) |
| Encoder | MT6701 magnetic, 14-bit SSI |
| Driver | TMC2209 UART, SpreadCycle for high-speed tests |
| Bus supply | **24.0 V** measured under load |
| USB | CDC bridge on the same node under test |
| Node | id **1**, `invert_dir=1` (required so +command increases encoder angle) |

### What was tested

1. Current × cruise matrix (40–100%, up to 6000 deg/s CL)
2. Microsteps 4 / 8 / 16
3. StealthChop vs SpreadCycle
4. Open-loop velocity ladder to 7200 deg/s
5. Focused **100% current** OL + CL ladders + 5× burst
6. **10-minute continuous closed-loop soak** at production tune (70% / 4800)

---

## 1. Motion profiles: what the firmware actually does

| Mode | Profile | Encoder? | Settles on target? |
|------|---------|----------|--------------------|
| `run(v)` open-loop velocity | Accel ramp → constant speed | No | No — spins forever |
| `move_to` open-loop | **Trapezoid** (FastAccelStepper) | No (step count) | Step target only |
| `move_to` closed-loop **≥1.2** | **Planned trapezoid + velocity FF + tracking PID** | Yes | Yes → `MOVE_DONE` |
| FOLLOW (encoder-corrected) | Continuous braking-law chase | Yes | Holds leader |

There is **no S-curve (jerk-limited)** profile yet. Both open-loop position and
closed-loop 1.2 use **constant acceleration** trapezoids (or a triangle when
the move is too short to reach cruise).

### Closed-loop 1.2 control law

On every `MOVE_ABS` / `MOVE_REL` with `closed_loop=1`:

```
1. Plan rest-to-rest trap/triangle:
      vmax = min(max_speed, cl_max_speed)
      amax = cl_max_accel
      from encoder angle → target

2. Each 5 ms (200 Hz):
      r(t), v_ff(t)  = evaluate trapezoid
      e_track       = r(t) − encoder
      v_cmd         = v_ff + Kp·e_track + Ki·∫e_track − Kd·v_meas

3. Settle when:
      plan finished AND |target − encoder| ≤ pid_tolerance
      AND |v_meas| is low  →  emit MOVE_DONE
```

Velocity diagram of a long move:

```
  v
  ^
  |     /----------\        ← cruise at cl_max_speed
  |    /            \
  |   /              \
  +-----------------------> t
    accel   cruise   decel
```

Short moves never reach cruise (triangle): accel immediately becomes decel.

This matches the classic “trapezoid generator” described in Klipper’s
kinematics docs, plus encoder tracking so the shaft actually arrives.

### Why pure PID (pre-1.1) failed at high speed

With `v = Kp · error` only, stopping from cruise needs roughly `v/Kp`
degrees of residual error. At 2000 deg/s and Kp=6 that is ~300° of overshoot
→ reverse → hunt → `NO_PROGRESS` or never settle. Raising `cl_max_speed`
alone made it worse.

Firmware 1.2 **plans the deceleration**, so the feedforward already slows
before the target; the PID only trims lag/load.

### Why open-loop `run()` can still look faster

| Mode | Must decelerate? | Must settle on encoder? |
|------|------------------|-------------------------|
| Open-loop continuous velocity | No | No |
| Closed-loop position | Yes | Yes |

On long moves, closed-loop 1.2 still reaches ~0.8× the continuous open-loop
ceiling (e.g. CL 4800–6000 vs OL ~7200 deg/s on this motor) — expected.

---

## 2. Parameters that matter

| Param | Role in CL 1.2 | Production default (this motor) |
|-------|----------------|----------------------------------|
| `cl_max_speed` | Trap **cruise** vmax (deg/s) | **4800** (800 RPM) |
| `cl_max_accel` | Trap accel/decel (deg/s²) | **19200** (≈ 4× cruise) |
| `max_speed` / `acceleration` | Also used as plan caps / open-loop | Match CL values |
| `pid_kp/ki/kd` | Tracking trim around `v_ff` | **12 / 0.3 / 0.10** |
| `pid_tolerance` | Settle window (deg) | **0.35** |
| `run_current` | % of driver max toward motor rating | **70%** continuous |
| `microsteps` | Smoothness vs torque at speed | **8** |
| `stealthchop` | 0 = SpreadCycle | **0** (required for high speed) |
| `invert_dir` | +command must increase encoder | **1** on this wiring |

One-shot setup:

```python
node.set_direction(True)   # if closed loop previously ran away
node.configure_closed_loop_speed(
    4800.0,
    run_current=70,
    microsteps=8,
    stealthchop=False,
    persist=True,
)
```

---

## 3. Physics limits (what actually caps RPM)

Independent of firmware (see also [JLCMC stepper speed notes](https://jlcmc.com/blog/stepper-motor-speed)):

1. **Supply voltage** — higher V forces current into inductive windings against back-EMF. We test at **24 V**.
2. **Current** — more torque; on this 1.2 A motor, TMC % is relative to driver max. Continuous 70% was more reliable than hammering 100% without cool-down.
3. **Inductance & back-EMF** — this motor’s **3.2 mH** inductance is friendly to high speed; higher-L motors sag earlier.
4. **Rotor inertia (54 g·cm²)** — sets acceleration authority with available torque.
5. **Microstepping** — 8× is the usual high-speed sweet spot on this frame.
6. **Chopper mode** — **SpreadCycle** holds torque at speed; **StealthChop** is quiet but sags early.
7. **Resonance** — mid-band can stall open-loop; closed-loop recovers better.
8. **USB / CDC link** — aggressive 100% continuous motion once dropped CDC (`/dev/ttyACM0` vanished); treat as a system limit when USB data and high motor current share a node (logic is from Vin, not USB power).

Practical ceiling observed on this motor @ 24 V: open-loop ~**1200 RPM**, closed-loop position cruise ~**1000 RPM** with settle.

---

## 4. Hardware matrix results

**Conditions (default):** firmware 1.2, 24 V, PR42HS40-1204AF-02, node 1,
`invert_dir=1`, SpreadCycle, tracking PID 12 / 0.3 / 0.10, tol=0.35°,
accel = 4× cruise. Move length noted per subsection.

### 4.1 Current × cruise (microsteps = 8, move = 2160°)

| I% | 720 | 1440 | 2160 | 3600 | 4800 | 6000 deg/s |
|----|-----|------|------|------|------|------------|
| 40 | OK  | OK   | OK   | OK   | OK   | OK |
| 55 | OK  | OK   | OK   | OK   | OK   | OK |
| 70 | OK  | OK   | OK   | OK   | OK   | OK |
| 85 | OK  | OK   | OK   | OK\* | OK   | OK\* |
| 100 (early matrix) | timeout | timeout | OK | OK | OK\* | OK\* |

\* Sometimes longer settle or lower peak (inconsistent).

**Peak encoder speeds while settling (typical 70%):**

| Cruise (deg/s) | Peak (deg/s) | Peak RPM | Settle time (2160°) |
|----------------|--------------|----------|---------------------|
| 720 | ~1150 | ~190 | ~4–5 s |
| 1440 | ~2250 | ~375 | ~2.5 s |
| 2160 | ~3300 | ~550 | ~1.9 s |
| 3600 | ~4600–5000 | ~800 | ~1.5 s |
| 4800 | ~6000–6600 | ~1000–1100 | ~1.3–1.4 s |
| 6000 | ~7200–7500 | ~1200–1250 | ~1.3 s |

**Confirmatory re-run (70%, 8 µstep):** 3600 / 4800 / 6000 all OK; peaks
4638 / 6594 / 7174 deg/s.

### 4.2 Focused 100% current retest (after USB recovery)

A dedicated 100% session (with cool-downs between rungs, move = **1440°**)
showed **better** behavior than the first matrix pass:

**Open-loop @ 100%**

| cmd | result |
|-----|--------|
| 360 → **7200** deg/s | all **tracking** (ratio ≈ 1.0) |

Reliable OL ceiling @ 100%: **~7200 deg/s (1200 RPM)**.

**Closed-loop @ 100%**

| cruise | result | notes |
|--------|--------|--------|
| 360 … **6000** | all **OK** | peaks up to ~6260 deg/s |
| Burst 5× @ 6000 | **5/5 OK** | peaks ~6100–6160 |

**Caveats observed at 100%:**

- An earlier 100% session **dropped USB** mid-ladder (serial I/O error, then
  `/dev/ttyACM0` gone) — brownout / CDC reset under high motor current.
- First matrix pass had low-speed CL timeouts at 100%; second pass did not.
  Treat 100% as **possible for short bursts**, not as the default continuous
  duty without thermal and USB margin.

MCU rose ~33 → 44 °C during the successful 100% ladder; Vbus held 24.0 V.

### 4.3 Microsteps @ 85% current

| µsteps | 1440 | 3600 | 4800 | 6000 |
|--------|------|------|------|------|
| 4 | OK (slow) | timeout | OK | timeout |
| **8** | OK | **OK** | **OK** | **OK** |
| 16 | OK (slow) | OK | OK | OK |

**8 microsteps** is the most consistent high-speed choice on this motor.
4× is resonance-sensitive. 16× works but often slower settle at mid cruise.

### 4.4 StealthChop vs SpreadCycle (70%, 8 µstep)

| Mode | 720 | 1440 | 3600 | 4800 | Notes |
|------|-----|------|------|------|-------|
| StealthChop | timeout | OK | OK (peak~3k) | OK (peak~1.8k) | Quiet; weak at speed |
| **SpreadCycle** | **OK** | **OK** | **OK peak~5k** | **OK peak~6.2k** | Required for high CL cruise |

### 4.5 Open-loop velocity ceiling (summary)

| Current | Reliable tracked `run()` ceiling | Notes |
|---------|----------------------------------|--------|
| 70% | **~7200 deg/s** (1200 RPM) | Clean tracking |
| 100% (good run) | **~7200 deg/s** | With cool-downs |
| 100% (bad run) | stall by ~2160 | Thermal / supply / state dependent |

Closed-loop position with planned decel: **4800–6000 deg/s** cruise is proven
on this motor — about **0.7–0.8×** continuous open-loop ceiling.

---

## 5. Long-duration closed-loop soak (10 minutes)

Production-style reliability check on **PR42HS40-1204AF-02**.

| Item | Value |
|------|--------|
| Duration | **10.00 minutes** continuous |
| Firmware | 1.2 |
| Tune | **70%** current, **4800** deg/s cruise, 8 µstep, SpreadCycle, accel=19200 |
| Pattern | `0° → 1440° → 0°` (two encoder settles per full cycle) |
| Tracking PID | kp=12, ki=0.3, kd=0.10, tol=0.35° |
| `invert_dir` | 1 |

### Results

| Metric | Value |
|--------|--------|
| Full cycles (A→B) | **239** |
| Total settles | **478** |
| **Success rate** | **100.00%** (478/478) |
| Faults (`NO_PROGRESS` / encoder) | **0** |
| Timeouts | **0** |
| USB / transport errors | **0** (0 reconnects) |
| Settle time min / median / max | **1.09 / 1.15 / 3.91 s** |
| Peak speed min / median / max | **1589 / 5658 / 6014 deg/s** |
| \|final − target\| median / max | **0.110° / 0.461°** |
| MCU temp start → end | **41.4 → 61.4 °C** (steady climb, no runaway) |
| Vbus start → end | **24.0 → 24.0 V** |

Progress samples stayed at **100% success** from cycle 1 through 230+. Near
the end a few moves ran slower (peak ~1600 deg/s, dt ~3.7 s) but still
settled inside tolerance — no faults.

### Observation

On this NEMA 17 (1.2 A, 3.2 mH, 4.2 kg·cm), **closed-loop trap + v_ff at
70% / 4800 deg/s is stable for continuous multi-minute duty**: no missed
settles, no USB loss, bus voltage flat, MCU thermal rise ~20 °C over 10 min
of continuous high-speed reversing moves.

---

## 6. Recommended production settings

Validated on: **PR42HS40-1204AF-02** + Grafito CANStepper C3, TMC2209,
MT6701, **24 V**, firmware **1.2**, `invert_dir=1`.

```python
from canstepper import CANStepperBus

bus = CANStepperBus.serial("/dev/ttyACM0")
node = bus.node(1)

node.set_direction(True)   # required on the characterized wiring
node.configure_closed_loop_speed(
    cruise_deg_s=4800.0,   # 800 RPM — margin under 6000 max proven
    run_current=70,        # continuous; 100% OK for short bursts only
    microsteps=8,
    stealthchop=False,     # SpreadCycle
    accel_factor=4.0,       # cl_max_accel = 19200
    kp=12.0, ki=0.3, kd=0.10,
    tolerance_deg=0.35,
    persist=True,
)
node.enable()
node.set_zero()
node.move_to(720.0, blocking=True)
```

| Goal | Suggested cruise | Current | Notes |
|------|------------------|---------|-------|
| Quiet / light duty | 720–1440 | 40–55% | StealthChop OK only at low speed |
| **Balanced production** | **4800** | **70%** | **Persisted default; 10 min soak PASS** |
| Absolute max (validated) | 6000 | 70–100% short | Re-check warm; cool-down between bursts @ 100% |
| Continuous 100% | not recommended as default | 100% | USB drop risk; prefer 70% for long duty |

To ship max cruise after soak:

```python
node.configure_closed_loop_speed(6000, run_current=70, microsteps=8,
                                 stealthchop=False, persist=True)
```

---

## 7. Key observations (checklist)

1. **Firmware 1.2 trap + v_ff** is required for high closed-loop cruise with
   reliable settle; pure PID (≤1.0) and braking-only (1.1) are inferior.
2. **This motor (PR42HS40-1204AF-02)** with 3.2 mH / 1.2 A on **24 V** can
   open-loop track ~**1200 RPM** and closed-loop cruise ~**800–1000 RPM**.
3. **Polarity matters:** `invert_dir=1` on the test wiring so encoder angle
   increases with positive commands; wrong polarity → `NO_PROGRESS`.
4. **SpreadCycle** beats StealthChop above a few hundred deg/s.
5. **8 microsteps** is the sweet spot; 4× resonance; 16× slower mid-band settle.
6. **70% continuous** is the production sweet spot (soak 100% success).
7. **100% current** can match high speed in short tests but has shown **USB
   disconnects** and earlier inconsistent low-speed CL timeouts — use for
   bursts with cool-down, not as silent always-on default.
8. **10 min soak** at 70%/4800: **478/478** settles, peak med ~5658 deg/s,
   final error med **0.11°**, MCU +20 °C, Vbus flat.
9. Closed-loop position is **not** expected to match continuous open-loop
   `run()` ceiling; plan for accel + decel + settle overhead.
10. Host helpers `plan_trapezoid` / `eval_trapezoid` mirror firmware planning
    for multi-axis timing (`MotionGroup`).

---

## 8. Tuning procedure (your machine)

1. **Polarity** — slow closed-loop move. If it runs away →  
   `node.set_direction(True); node.save_config()`.
2. **Driver baseline** — SpreadCycle, 8 µstep, 70% current, 24 V solid supply.
3. **Ladder**  
   ```bash
   python3 tools/cl_speed_validate.py /dev/ttyACM0 1 70 8
   python3 examples/closed_loop_speed.py /dev/ttyACM0 1   # sweep
   python3 tools/cl_matrix.py /dev/ttyACM0 1 --quick
   ```
4. **Pick last OK cruise** with margin (e.g. if 6000 works, ship 4800).
5. **Persist** with `configure_closed_loop_speed(..., persist=True)`.
6. **Thermal** — multi-minute continuous moves; if motor case is very hot or
   MCU climbs hard without plateau, drop current 10%.
7. **Load** — retune with real mechanism attached; reflected inertia lowers
   usable accel first, then cruise.
8. **Long soak** — run continuous A↔B moves for ≥10 min at the chosen cruise
   before locking production config.

### If settle fails

| Symptom | Try |
|---------|-----|
| `NO_PROGRESS` immediately | `invert_dir`; check encoder OK bit |
| Timeout near target | Raise `pid_tolerance` slightly (0.35→0.5); raise `kd` a little |
| Stall mid-move | Raise current toward 85%; lower microsteps 16→8; ensure SpreadCycle |
| Works OL, fails CL | Confirm fw ≥1.2; raise `cl_max_accel` |
| Loud mid-band | Change µsteps; raise accel to punch through resonance |
| USB disconnect under load | Drop continuous current (e.g. 100%→70%); check 24 V bulk capacitance |

### Tracking PID notes (fw ≥1.2)

Feedforward carries the move. PID only trims lag:

- **Raise Kp** if the shaft lags the plan (tracking error large in MOTION tel).
- **Raise Kd** if it rings at the end of decel.
- **Ki** small; too much causes slow oscillation around target.
- Do **not** treat Kp as the primary speed knob — that is `cl_max_speed`.

---

## 9. Library helpers

```python
# Host-side trap math (same equations as firmware trajPlan / trajAdvance)
from canstepper.kinematics import plan_trapezoid, eval_trapezoid, trapezoid_time

v_peak, t_acc, t_cruise, t_dec, t_tot = plan_trapezoid(
    distance=720.0, speed=4800.0, accel=19200.0
)
s, v_ff = eval_trapezoid(720.0, 4800.0, 19200.0, t=t_acc)  # end of accel
```

`MotionGroup` uses the same trapezoid **duration** math so multi-axis
point-to-point moves finish together (not continuous look-ahead/contouring).

---

## 10. Validation commands

```bash
cd ~/Grafito-Edge-Services/can_stepper

python3 -m pytest                              # sim unit tests
python3 tools/cl_speed_validate.py /dev/ttyACM0 1 70 8
python3 tools/cl_matrix.py /dev/ttyACM0 1 --quick
python3 examples/closed_loop_speed.py /dev/ttyACM0 1 4800
python3 examples/high_speed_test.py /dev/ttyACM0 1 70
python3 tools/hw_validate.py /dev/ttyACM0 1
```

Flash firmware 1.2+:

```bash
export PATH="$HOME/.local/bin:$PATH"
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3
```

---

## 11. Version history (closed-loop)

| FW | Closed-loop behaviour |
|----|------------------------|
| ≤1.0 | Pure PI/D velocity chase; default cl_max_speed 30 deg/s |
| 1.1 | Braking-envelope cap on PI/D (no planned cruise) |
| **1.2** | **Trapezoidal trajectory + velocity feedforward + tracking PID** |

---

## 12. What we did *not* implement (future)

| Idea | Source | Status |
|------|--------|--------|
| S-curve / jerk limit | JLCMC, machine tools | Not yet — next planner upgrade |
| Look-ahead / junction speed | Klipper | Host multi-move paths only (future) |
| Minimum cruise ratio | Klipper short zigzags | Not yet |
| Pressure advance | Klipper extruder | N/A for pure axes |

For most single-axis and coordinated point-to-point machines, trap + v_ff is
the right first industrial profile; add S-curve if frame resonance dominates.
