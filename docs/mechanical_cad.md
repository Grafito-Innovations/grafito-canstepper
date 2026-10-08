# Mechanical CAD & drawings

Published with in-page previews at **https://docs.grafito.in/docs/mechanical-cad**.

Source files live in `model_assets/` at the repo root. These are the **V2**
board assets used on the shop and docs sites.

| File (repo) | Published path | Description |
| --- | --- | --- |
| `CANStepper.png` | `/model-assets/canstepper.png` | Product photo |
| `CANStepper_pinout.png` | `/model-assets/canstepper-pinout.png` | Connector / pinout diagram |
| `CANStepper_multiaxis_can.png` | `/model-assets/canstepper-multiaxis-can.png` | Multi-axis CAN bus demonstration |
| `CANStepper_V1_assembly.mp4` | — | V1 board assembly video (historical) |
| `CANStepper_test_station.png` | https://docs.grafito.in/dashboard | Board Test Station after USB connect |
| `CANStepper_block_diagram.png` | `/model-assets/canstepper-block-diagram.webp` | System block diagram |
| `CANStepper_V2_Assembly.webp` | `/model-assets/canstepper-v2-assembly.webp` | V2 assembly render |
| `Drawing_CANStepper_V2.pdf` | `/model-assets/canstepper-v2-drawing.pdf` | V2 PCB mechanical drawing |
| `CANStepperV2_3D_PCB.step` | `/model-assets/canstepper-v2-3d-pcb.step` | V2 PCB STEP (~24 MB) |
| `CASING_MOUNT.step` | `/model-assets/canstepper-casing-mount.step` | Casing / mount STEP |
| `Heat_Spreader.step` | `/model-assets/canstepper-heat-spreader.step` | Heat spreader STEP |
| `TMC2209 UART Stepper Driver - Heat_Sink.step` | `/model-assets/tmc2209-heat-sink.step` | TMC2209 heatsink STEP |
| `canstepper-brochure-v2.pdf` | `/brochures/canstepper-brochure-v2.pdf` | Product brochure |

Shop product page CAD section:
https://grafito.in/shop/products/canstepper-adapter-board/#mechanical-cad

Brochure:
https://grafito.in/brochures/canstepper-brochure-v2.pdf

## Videos

| Topic | URL | Docs embed |
| --- | --- | --- |
| V1 assembly (this repo) | [model_assets/CANStepper_V1_assembly.mp4](../model_assets/CANStepper_V1_assembly.mp4) | Historical V1 board — mixed connectors |
| Assembly walkthrough | https://www.youtube.com/watch?v=ULpthBzb50s | https://docs.grafito.in/docs/mechanical-cad#assembly-video |
| Leader / follower (encoder mirror) | https://www.youtube.com/shorts/0dZ-bfxO9gM | https://docs.grafito.in/docs/motion#dual-motor-axes-eg-dual-z-gantries |

The V1 clip shows an earlier revision of the same board. V1 used **three
different connector types**, which made cabling and crimping difficult. The
current production board is **V2**, with **JST 2.0** connectors throughout.

> **Important — encoder magnet:** Use a **diametrical** magnet on the motor
> shaft. Polarity must be **radial**, not axial. An axial magnet will not
> give a valid MT6701 reading.
