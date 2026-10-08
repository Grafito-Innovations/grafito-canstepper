# Grafito CAN Stepper Protocol v1 (GCSP v1)

Wire protocol between a host and Grafito CANStepper boards. This document is
sufficient to implement an independent host client. The Python reference
implementation is `canstepper/protocol.py`; the firmware is
`firmware/GrafitoCANStepper_C3/`.

## Physical layer

- CAN 2.0A, **standard 11-bit identifiers**, **1 Mbps**, TCAN3413 transceiver.
- Daisy-chained nodes; up to **31 nodes** per bus.
- On-board **120 Ω termination is enabled by default** via a **0 Ω jumper**.
  Keep termination only on the **two physical ends** of the bus; **remove the
  0 Ω resistor** on intermediate (mid-chain) boards. End boards can short the
  termination pads if the jumper was removed and end termination is needed again.

## Addressing

```
CAN ID (11 bit) = (node_id << 6) | msg_id
```

| Field | Bits | Range | Meaning |
|---|---|---|---|
| `node_id` | 10..6 | 0–31 | 0 = broadcast (every node executes), 1–31 = one board |
| `msg_id` | 5..0 | 0–63 | 0–31 command (host→node), 32–63 telemetry (node→host) |

- All multi-byte payload values are **little-endian**.
- A **remote frame (RTR)** carrying a telemetry `msg_id` requests that node to
  transmit the telemetry immediately (poll). Data frames with telemetry IDs
  are never executed as commands.
- Nodes also broadcast telemetry periodically: the *fast* group
  (POSITION, MOTION, PID_STATUS) at `fast_rate_hz` (default 10 Hz) and the
  *slow* group (STATUS, DRIVER, ENV, FOLLOW_STATUS, CAN_HEALTH) at
  `slow_rate_hz` (default 1 Hz). Either rate can be 0 (off).

## Serial bridge

Any node bridges its USB CDC port (115200 baud) to the bus. One text line per
frame, in both directions:

```
<CAN_ID hex> <RTR 0|1> <payload hex>\n
```

Example — command node 1 to 180°: `44 0 0000000000C06640`
(`0x44 = (1<<6)|4`, MOVE_ABS, f64 little-endian 180.0).

- Payload length = hex-digit count / 2 (0–8 bytes). RTR lines carry no payload.
- Lines beginning with `#` are human-readable debug output — ignore them.
- The bridging node executes bridged frames addressed to it (or broadcast)
  *and* re-transmits them on CAN; every frame it sees on the bus, plus its own
  telemetry, is printed to the port.

## Commands (msg_id 0–31)

| # | Name | Payload | Behavior |
|---|------|---------|----------|
| 0 | `PING` | — | Node answers with STATUS immediately. |
| 1 | `ESTOP` | — | Instant stop + driver hardware-disable. **Latched**: motion commands are ignored until `ENABLE 1`. |
| 2 | `STOP` | — | Decelerating stop; driver stays energized; cancels homing/velocity/position. |
| 3 | `ENABLE` | `u8` 0/1 | Driver off/on. Enabling clears the e-stop latch. Disabling force-stops. |
| 4 | `MOVE_ABS` | `f64` deg | Absolute position move. Open-loop: FAS trapezoid to step target. Closed-loop (fw ≥1.2): rest-to-rest trap plan + velocity FF + tracking PID to encoder. |
| 5 | `MOVE_REL` | `f64` deg | Relative to the active target (or current position if idle). Same open/closed profiles as `MOVE_ABS`. |
| 6 | `MOVE_VEL` | `f32` deg/s | Signed continuous velocity; 0 = ramped stop. |
| 7 | `SET_ZERO` | — | Stop; current shaft angle becomes 0°. |
| 8 | `HOME` | `u8` method, `i8` dir, `f32` speed deg/s | Homing routine, see below. |
| 9 | `SET_PARAM` | `u8` id, 4-byte value | Set a parameter; node replies with PARAM (status + readback). |
| 10 | `GET_PARAM` | `u8` id | Node replies with PARAM. |
| 11 | `SAVE_CONFIG` | — | Persist all parameters to flash (applied at every boot). |
| 12 | `LOAD_DEFAULTS` | — | Factory defaults + persist; the node **keeps its ID**. |
| 13 | `FOLLOW` | `u8` en, `u8` leader, `u8` flags, `u8` rsvd, `f32` ratio | flags bit0 = invert, bit1 = encoder-corrected. |
| 14 | `FOLLOW_SYNC` | — | Recapture leader/local zero offset on the next leader frame. |
| 15 | `SET_POSITION` | `f64` deg | Stop and label the current physical shaft position with this logical angle; no motion. |
| 16–31 | reserved | | Ignored. |

### Homing (`HOME`)

`method`: 0 = **set-zero** (define zero here), 1 = **endstop** (IO8),
2 = **StallGuard** (sensorless). `dir`: +1 / −1. Sequence for methods 1–2:

1. Optionally switch to `homing_current` (param 25; 0 = keep run current).
2. Run at the commanded speed and direction until the trigger
   (endstop active / TMC DIAG rising edge) or `homing_timeout_ms`.
3. Force-stop; the trigger point becomes 0°.
4. Back off `homing_backoff` degrees in the opposite direction.
5. Restore current; emit `HOMING_DONE` (or `HOMING_FAILED` on timeout) and
   set the `homed` status flag.

### Endstop (IO8)

Independent of homing, when `endstop_enable` = 1 every edge produces an
`ENDSTOP_HIT` / `ENDSTOP_RELEASED` event, and `endstop_action` selects what a
hit does to running motion: 0 report-only, 1 stop, 2 stop + zero. Polarity via
`endstop_active_high` (default active-low with internal pull-up).

> **Boot caution:** IO8 is an ESP32-C3 strapping pin. It must be HIGH at
> power-on or the chip may not boot — use normally-open switches to GND, or
> wire NC switches through a series diode.

## Parameters

4-byte values, type `u32` or `f32`. Ranges are hardware-sanity checks
(mirrored by host and firmware); **motion limits are configurable defaults,
never firmware clamps**. Persisted only on `SAVE_CONFIG`; loaded at power-on.

| ID | Name | Type | Default | Range | Description |
|----|------|------|---------|-------|-------------|
| 1 | `node_id` | u32 | 1 | 1–31 | CAN address (save + reboot recommended after change) |
| 2 | `steps_per_rev` | u32 | 200 | 1–100000 | Full steps per motor revolution |
| 3 | `microsteps` | u32 | 16 | 1–256, power of 2 | TMC2209 microstepping |
| 4 | `run_current` | u32 | 30 | 1–100 | Run current, % of driver max |
| 5 | `hold_current` | u32 | 15 | 0–100 | Standstill current, % |
| 6 | `stall_threshold` | u32 | 10 | 0–255 | StallGuard SGTHRS (higher = more sensitive) |
| 7 | `invert_dir` | u32 | 0 | 0/1 | Invert physical rotation (logical frame preserved) |
| 8 | `closed_loop` | u32 | 1 | 0/1 | MT6701 position loop for MOVE_ABS/REL |
| 9 | `max_speed` | f32 | 720 | >0 | Cruise speed for open-loop position / host planning, deg/s |
| 10 | `acceleration` | f32 | 2880 | >0 | Open-loop position ramp, deg/s² |
| 11 | `cl_max_speed` | f32 | 720 | >0 | Closed-loop **trapezoid cruise** vmax, deg/s (fw ≥1.2) |
| 12 | `cl_max_accel` | f32 | 2880 | >0 | Closed-loop **trapezoid accel/decel** amax, deg/s² |
| 13 | `pid_kp` | f32 | 12.0 | ≥0 | Tracking Kp on (r − encoder); v_ff carries the move |
| 14 | `pid_ki` | f32 | 0.3 | ≥0 | Tracking Ki |
| 15 | `pid_kd` | f32 | 0.10 | ≥0 | D-on-measured-velocity damping |
| 16 | `pid_tolerance` | f32 | 0.35 | 0.001–360 | Settle window, deg |
| 17 | `fast_rate_hz` | u32 | 10 | 0–500 | POSITION/MOTION/PID_STATUS rate |
| 18 | `slow_rate_hz` | u32 | 1 | 0–500 | STATUS/DRIVER/ENV/… rate |
| 19 | `enable_on_boot` | u32 | 1 | 0/1 | |
| 20 | `zero_on_boot` | u32 | 0 | 0/1 | |
| 21 | `standstill_mode` | u32 | 0 | 0–3 | 0 normal, 1 freewheel, 2 brake, 3 strong brake |
| 22 | `endstop_enable` | u32 | 0 | 0/1 | |
| 23 | `endstop_active_high` | u32 | 0 | 0/1 | |
| 24 | `endstop_action` | u32 | 1 | 0–2 | 0 report, 1 stop, 2 stop+zero |
| 25 | `homing_current` | u32 | 0 | 0–100 | 0 = keep run current |
| 26 | `homing_backoff` | f32 | 2.0 | ≥0 | deg |
| 27 | `homing_timeout_ms` | u32 | 30000 | 100–600000 | |
| 28 | `stealthchop` | u32 | 1 | 0/1 | Quiet mode (off = SpreadCycle) |

## Telemetry (msg_id 32–63)

| # | Name | Payload | When |
|---|------|---------|------|
| 32 | `STATUS` | `u8` flags, `u8` mode, `u8` fault, `u8` fw_major, `u8` fw_minor, `u8` proto, `u8` node_id | slow rate, PING, RTR |
| 33 | `POSITION` | `f64` encoder angle deg (multi-turn) | fast rate, RTR — **drives followers** |
| 34 | `MOTION` | `f32` measured velocity deg/s, `f32` position error deg | fast rate, RTR |
| 35 | `TARGET` | `f64` active target deg | RTR |
| 36 | `EVENT` | `u8` event, `u8` detail, `f32` data | on occurrence |
| 37 | `DRIVER` | TMC diagnostics, **8 bytes** (fw ≥1.4; see below) | slow rate, RTR |
| 38 | `ENV` | `f32` MCU temp °C, `f32` bus volts | slow rate, RTR |
| 39 | `PARAM` | `u8` param id, `u8` status (0 ok, 1 unknown, 2 rejected), 4-byte value | reply to SET/GET_PARAM |
| 40 | `FOLLOW_STATUS` | `u8` en, `u8` leader, `u8` flags, `u8` synced, `f32` ratio | slow rate (when following), RTR |
| 41 | `CAN_HEALTH` | `u8` TWAI state, `u8` tx_err, `u8` rx_err, `u8` recoveries, `u16` tx_failed, `u16` bus_errors | slow rate, RTR |
| 42 | `ENC_COUNTS` | `i64` multi-turn counts (16384/rev) | RTR |
| 43 | `PID_STATUS` | `u8` state (0 idle, 1 running, 2 settled, 3 fault), `u8` fault, `f32` output deg/s | fast rate (closed loop), RTR |
| 44–63 | reserved | | |

### STATUS flag bits (byte 0)

| Bit | Meaning |
|-----|---------|
| 0x01 | driver enabled |
| 0x02 | moving |
| 0x04 | homed |
| 0x08 | e-stop latched |
| 0x10 | endstop active |
| 0x20 | stall latched |
| 0x40 | encoder frame OK |

### Modes / faults / events

- **mode**: 0 idle, 1 position, 2 velocity, 3 homing, 4 following.
- **fault**: 0 none, 1 encoder invalid, 2 no progress (mechanical stall in
  closed loop), 3 homing timeout, 4 TMC over-temperature shutdown (OT),
  5 TMC short (S2G / low-side).
- **event**: 1 boot, 2 endstop hit, 3 endstop released, 4 stall, 5 homing
  done, 6 homing failed, 7 move done, 8 e-stop, 9 fault. `data` is usually
  the position (deg) at the event (OT fault may carry MCU temp °C).

### DRIVER payload (msg_id 37, firmware ≥1.4)

| Bytes | Type | Meaning |
|-------|------|---------|
| 0–1 | u16 | StallGuard result |
| 2 | u8 flags_a | bit0 UART ok, bit1 **OTPW**, bit2 **OT**, bit3 S2GA, bit4 S2GB, bit5 S2VSA, bit6 S2VSB, bit7 OLA |
| 3 | u8 flags_b | bit0 OLB, bit1 t120, bit2 t143, bit3 t150, bit4 t157, bit5 stealth, bit6 standstill |
| 4 | u8 gstat | bit0 reset, bit1 drv_err, bit2 uv_cp |
| 5 | u8 | `cs_actual` (0–31) TMC current scale from `DRV_STATUS` |
| 6–7 | u16 | interstep duration (TSTEP-style, low 16 bits) |

Host conversion (library ≥0.2.1, TMC2209 datasheet, 100 mΩ sense):  
`I_RMS = (CS+1)/32 × 0.325 / ((0.100+0.020)×√2)` → `DriverStatus.commanded_rms_amps()`.  
Regulation scale only — not 24 V bus current.

Python: `node.get_driver_status()` → `DriverStatus` (includes `otpw`, `over_temp_shutdown`, …).  
OT shutdown stops motion and latches `fault=4` until `enable()`; OTPW is warning-only (flag + serial log).

### Firmware implementation (GrafitoCANStepper_C3, fw ≥1.4)

- `tmcPollStatus()` — read `getStatus()` / `getGlobalStatus()`, set OTPW/OT/short bits  
- `sendDriver()` — 8-byte `TEL_DRIVER` payload  
- `tmcDriverService()` — ~100 ms poll; OT → `FAULT_DRIVER_OT`, shorts → `FAULT_DRIVER_SHORT`  
- `setDriverEnabled(true)` — `clearDriveError()` / `clearReset()`, clear OT/short latches  

Public download + full-source preview (no private monorepo required):  
https://docs.grafito.in/docs/firmware  
and raw file https://docs.grafito.in/firmware/GrafitoCANStepper_C3.ino

## Closed-loop motion profile (firmware ≥1.2)

When `closed_loop = 1` and the node receives `MOVE_ABS` / `MOVE_REL`:

1. **Plan** a rest-to-rest trapezoid (or triangle if the distance is short)
   from the current encoder angle to the target, using
   `vmax = min(max_speed, cl_max_speed)` and `amax = cl_max_accel`.
2. **Generate** each 5 ms control tick a reference position `r(t)` and
   feedforward velocity `v_ff(t)` along that plan.
3. **Track** with a light PID on the residual:  
   `v_cmd = v_ff + Kp·(r − encoder) + Ki·∫ − Kd·v_meas`.
4. **Settle** when the plan is finished, `|target − encoder| ≤ pid_tolerance`,
   and measured speed is low → emit `MOVE_DONE` (`EVT` 7).

Open-loop position moves still use FastAccelStepper’s internal trapezoid
via step counts (no encoder in the loop). Continuous `MOVE_VEL` is open-loop
velocity with an accel ramp only.

FOLLOW mode with encoder correction uses a continuous braking-law chase
toward the live leader target (not a re-planned trap each frame).

Tuning guide, hardware matrix (current / microsteps / chopper mode), and
production recommendations: **[closed_loop_tuning.md](closed_loop_tuning.md)**.

## Leader / follower

`FOLLOW` makes a node track another node's `POSITION` broadcasts entirely
on-bus. The first leader frame after enable (or after `FOLLOW_SYNC`)
captures both zero references, so the follower never jumps on engage.
`follower = local_ref + (leader − leader_ref) × ratio × (invert ? −1 : 1)`.
With flags bit1 set the follower closes its own MT6701 loop on the mapped
target (recommended); otherwise it runs open-loop steps. A 0.15° hysteresis
deadband rejects stationary leader-encoder noise without adding lag.

## Bus load

A 1 Mbps standard frame with 8-byte payload is ≈111–135 bits with stuffing.
At defaults (fast 10 Hz × 3 frames + slow 1 Hz × 5 frames ≈ 35 frames/s/node),
31 nodes ≈ 1085 frames/s ≈ **15 % bus load**. Scale `fast_rate_hz` accordingly
if you raise rates or add nodes.
