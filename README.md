# Grafito CANStepper

![Grafito CANStepper](https://raw.githubusercontent.com/Grafito-Innovations/grafito-canstepper/main/CANStepper.png)

Closed-loop stepper motor control over CAN. Each board carries an ESP32-C3,
a TMC2209 driver, an MT6701 14-bit magnetic encoder and a CAN transceiver;
up to 31 boards daisy-chain on one 1 Mbps bus and are driven from Python
through the USB port of any board on the chain.

```
├── canstepper/                              # Python package (grafito-canstepper)
├── firmware/GrafitoCANStepper_C3/           # node firmware (GCSP v1, fw 1.12)
├── firmware/GrafitoCANStepper_C3_CANopen/   # optional CiA 301/402 SKU (fw 2.1)
├── examples/                                # runnable examples + machine.toml
├── tests/                                   # pytest suite (runs on a software sim)
├── model_assets/                            # assembly render, PCB drawing, STEP CAD
└── docs/                                    # flashing, protocol, quickstart, CAD
```

## Product links

- **Shop / brochure:** [CANStepper Adapter Board](https://grafito.in/shop/products/canstepper-adapter-board/)
- **Amazon (India):** https://www.amazon.in/dp/B0H8XZ6V99
- **Docs:** https://docs.grafito.in
- **Board test station:** https://docs.grafito.in/dashboard
- **Mechanical CAD & drawings:** https://docs.grafito.in/docs/mechanical-cad
- **PyPI:** [grafito-canstepper](https://pypi.org/project/grafito-canstepper/)

CAD and brochure files in this repo (`model_assets/`):

| File | Description |
| --- | --- |
| `CANStepper.png` | Product photo (also at repo root) |
| `CANStepper_pinout.png` | Connector / pinout diagram |
| `CANStepper_multiaxis_can.png` | Multi-axis CAN bus demonstration |
| `CANStepper_V1_assembly.mp4` | V1 board assembly video (historical) |
| `CANStepper_V1_assembly_preview.jpg` | Preview frame for the V1 video |
| `CANStepper_test_station.png` | Board Test Station (after USB connect) |
| `CANStepper_block_diagram.png` | System block diagram |
| `CANStepper_V2_Assembly.webp` | V2 assembly render |
| `Drawing_CANStepper_V2.pdf` | V2 PCB mechanical drawing |
| `CANStepperV2_3D_PCB.step` | V2 PCB STEP |
| `CASING_MOUNT.step` | Casing / mount STEP |
| `Heat_Spreader.step` | Heat spreader STEP |
| `TMC2209 UART Stepper Driver - Heat_Sink.step` | TMC2209 heatsink STEP |
| `canstepper-brochure-v2.pdf` | Product brochure |

<img src="https://raw.githubusercontent.com/Grafito-Innovations/grafito-canstepper/main/model_assets/CANStepper_pinout.png" alt="CANStepper pinout" width="720" />

<img src="https://raw.githubusercontent.com/Grafito-Innovations/grafito-canstepper/main/model_assets/CANStepper_multiaxis_can.png" alt="CAN bus multi-axis demonstration" width="900" />

**V1 board assembly** (click the preview to play the video):

[![V1 board assembly](https://raw.githubusercontent.com/Grafito-Innovations/grafito-canstepper/main/model_assets/CANStepper_V1_assembly_preview.jpg)](https://github.com/Grafito-Innovations/grafito-canstepper/blob/main/model_assets/CANStepper_V1_assembly.mp4)

This clip is **version 1** of the same board. V1 used **three different
connector types**, which made cabling and crimping difficult. The current
production board is **V2**, with **JST 2.0** connectors throughout for easy
crimping.

> **Important — encoder magnet:** Fit a **diametrical** magnet on the motor
> shaft. The magnet polarity must be **radial**, not axial. An axial magnet
> will not produce a valid MT6701 angle and closed-loop control will fail.

After USB and Vin are connected, test the board in the
**[CANStepper Test Station](https://docs.grafito.in/dashboard)** — set the
node ID, enable the driver, jog, and check encoder / TMC / CAN telemetry.

<a href="https://docs.grafito.in/dashboard">
  <img src="https://raw.githubusercontent.com/Grafito-Innovations/grafito-canstepper/main/model_assets/CANStepper_test_station.png" alt="CANStepper Test Station" width="900" />
</a>

## Highlights

- **Layered Python API** — raw frames → `StepperNode` → `NodeGroup` →
  `Axis` (mm units via `rotation_distance`) → `DualMotorAxis` /
  `IndependentDualAxis` / `Cartesian` / `CoreXY` / `MotionGroup` →
  declarative `machine.toml`.
- **Safety scopes** — `node.estop()`, `group.estop()`, `bus.estop_all()`
  (single broadcast frame); e-stop is latched until re-enabled.
- **Homing** — physical endstop on IO8, sensorless (StallGuard), or
  set-zero; configurable current, backoff and timeout.
- **Closed loop** — firmware ≥1.12 plans a rest-to-rest **S-curve**
  trajectory with **velocity + acceleration feedforward** and a light
  tracking PID on the MT6701 encoder (optional encoder LUT). All motion
  limits are *configurable defaults*, never hard clamps.
- **On-bus leader/follower** — dual-motor gantries stay coupled with no
  host in the loop (`DualMotorAxis`).
- **Encoder-gated dual screws** — `IndependentDualAxis` commands both
  motors independently and **blocks the next pass until both encoders**
  are within mm tolerance of the target.
- **Simulator included** — `canstepper.sim.SimNetwork` implements the whole
  protocol in software; the test suite and the examples run without hardware.

## Install & first spin

```bash
pip install grafito-canstepper

# Development (from this directory):
# pip install -U pip setuptools && pip install -e ".[dev]"
```

```python
from canstepper import CANStepperBus

with CANStepperBus.serial("/dev/ttyACM0") as bus:
    print(bus.discover())
    node = bus.node(1)
    node.set_run_current(40).enable()
    node.move_to(360.0, blocking=True)
```

Continue with:

- [docs/flashing.md](docs/flashing.md) — Arduino IDE + arduino-cli (GCSP and CANopen)
- [docs/quickstart.md](docs/quickstart.md) — first motion
- [docs/closed_loop_tuning.md](docs/closed_loop_tuning.md) — trapezoid + v_ff
- [docs/protocol.md](docs/protocol.md) — GCSP v1 wire protocol
- [docs/mechanical_cad.md](docs/mechanical_cad.md) — CAD, drawings, assembly video

G-code examples (from this directory):

```bash
PYTHONPATH=. python3 examples/gcode_cartesian.py /dev/ttyACM0 1 2
PYTHONPATH=. python3 examples/gcode_corexy.py /dev/ttyACM0 1 2
PYTHONPATH=. python3 examples/gcode_from_file.py examples/gcode/square.gcode
```

Homing examples (IO8 endstop, StallGuard sensorless, set-zero):

```bash
PYTHONPATH=. python3 examples/home_endstop.py /dev/ttyACM0 1 -1
PYTHONPATH=. python3 examples/home_sensorless.py /dev/ttyACM0 1 -1 60
PYTHONPATH=. python3 examples/home_set_zero.py /dev/ttyACM0 1
```

Belt axis (mm):

```bash
PYTHONPATH=. python3 examples/belt_move_mm.py /dev/ttyACM0 1 50 77.2 150
```

## Development

```bash
python3 -m pytest
```

Current GCSP firmware is **1.12** (S-curve + Ka + LUT; motors off at boot
until `enable()`). Optional **CANopen 2.1** lives in
`firmware/GrafitoCANStepper_C3_CANopen/` with `GrafitoCANStepper.eds` and
`GrafitoCANStepper_Node1.dcf` / `GrafitoCANStepper_Node2.dcf`
(FileRevision 2, identity `0x00020100`). Discard any older 2.0 EDS — it
will not import this board. Node 2 must be flashed with
`-DCO_FACTORY_NODE_ID=2`. See **https://docs.grafito.in/docs/canopen**.
Do not mix GCSP and CANopen on one bus.

**How to flash (Arduino IDE + arduino-cli, GCSP and CANopen):**
[docs/flashing.md](docs/flashing.md) · https://docs.grafito.in/docs/flashing

Firmware builds with Arduino IDE or `arduino-cli` (ESP32C3 Dev Module, USB
CDC On Boot = Enabled; libraries: FastAccelStepper, TMC2209 janelia-arduino).
**Apply Vin (5–24 V, typically 24 V) before upload** — USB-C is data only and
does not power the ESP32 for programming. CAN termination is **on by default**
(0 Ω jumper); remove that 0 Ω on mid-chain daisy-chain nodes.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3
```
