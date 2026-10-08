# CANopen / CiA 402 (PLC)

Optional firmware **2.1** for a PLC that imports an EDS. Production Python
machines still use **GCSP 1.12**. **Do not mix GCSP and CANopen on the same
CAN bus.**

- Firmware + EDS/DCF: `firmware/GrafitoCANStepper_C3_CANopen/`
- Flash (including factory node 2): [flashing.md](flashing.md)
- Human docs: https://docs.grafito.in/docs/canopen

Import **firmware 2.1** `GrafitoCANStepper.eds` (FileRevision 2, identity
`0x00020100`). Delete any older 2.0 EDS. For node 2 import
`GrafitoCANStepper_Node2.dcf` and flash with `-DCO_FACTORY_NODE_ID=2`.

Enable: controlword `0x6040` = `6 → 7 → 15`. Units are encoder counts
(16384 / rev).

## Tune gains from the PLC

Closed-loop command is:

```
v = v_ff + Ka · a_ff + PID(r − encoder)
```

PID only **trims lag**. Cruise speed is **not** a PID gain — set that with
`0x6081`. Write the gain objects as **REAL32** (IEEE-754). In TwinCAT /
Codesys map them as **REAL**. Do not write integer `10` into `0x200F`; it
must be float `10.0`.

### Objects to change

| Index | Name | Type | Factory | What it does |
| --- | --- | --- | --- | --- |
| **0x200F** | Kp | REAL32 | **10.0** | Stiffness. Raise if it lags the plan; lower if it buzzes. |
| **0x2011** | Kd | REAL32 | **0.35** | Damping. Raise if it rings at the end of a move. |
| **0x2010** | Ki | REAL32 | **0.3** | Steady-state. Keep small; too much causes overshoot / following error `0x7122`. |
| **0x201A** | Ka | REAL32 | **0.04** | Accel feedforward. Raise if it sags during accel/decel. |
| **0x2012** | PID tolerance | REAL32 | **0.35°** | Target-reached band. Too tight at high RPM → `0x7122`. |

### Set these before touching PID

| Index | Name | Notes |
| --- | --- | --- |
| **0x2006** | Invert | `1` if a positive target runs the wrong way (PID will fight the encoder). |
| **0x2003** | Run current % | Start **30–40**. Gains cannot fix a starved motor. |
| **0x2007** | Closed loop | Must be **1** for encoder PID (`0` = open-loop step pulses). |
| **0x6081** | Profile velocity | counts/s. Default `32768` ≈ 720 °/s. |
| **0x6083** | Profile accel | counts/s². Default `65536` ≈ 1440 °/s². |
| **0x201B** | Jerk | deg/s³. **0 = auto** (amax / 0.05 s). |

Save so it survives power-cycle: write **0x2008 = 1**.

### Order on the PLC

1. Enable `6 → 7 → 15`, confirm `0x2007 = 1`, invert with `0x2006` if needed.
2. Set a modest `0x6081` / `0x6083` and current **30–40**.
3. Leave **Ki = 0.3** and **Ka = 0.04**.
4. Change **Kp** in steps of ~2 (try 8 → 10 → 12).
5. If it oscillates at settle, raise **Kd** (0.35 → 0.5) or drop Kp.
6. Only then nudge **Ka** or **Ki**.
7. Write `0x2008 = 1` when it is good.

Measured precision defaults (fw 2.1 / GCSP 1.12): Kp **10**, Ki **0.3**,
Kd **0.35**, Ka **0.04**, tolerance **0.35°**.
