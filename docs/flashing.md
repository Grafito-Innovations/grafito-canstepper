# Flashing firmware

Step-by-step upload for a Grafito CANStepper board (ESP32-C3). Pick **one**
sketch per board. **Do not mix GCSP and CANopen on the same CAN bus** — the
ID maps collide.

| Sketch | Version | When to use |
| --- | --- | --- |
| `firmware/GrafitoCANStepper_C3` | **GCSP 1.12** | Python (`grafito-canstepper`), G-code, Test Station |
| `firmware/GrafitoCANStepper_C3_CANopen` | **CANopen 2.1** | PLC that imports an EDS / DCF |
| `firmware/CANStepper_WiFiPortal` | demo | Phone browser bench UI only |

Public docs: **https://docs.grafito.in/docs/flashing**  
Firmware downloads: **https://docs.grafito.in/docs/firmware**  
Source: **https://github.com/Grafito-Innovations/grafito-canstepper**

## 1. Power and USB (do this first)

USB-C on this board is **data only**. It does **not** power the ESP32 for
programming.

1. Apply **Vin 5–24 V** (typically **24 V**) to the power / CAN connector.
2. Then plug USB-C into the host.
3. Keep Vin connected for the whole compile + upload. If Vin drops, the port
   disappears and the flash fails.

Do **not** hold a HOME switch closed at power-on. GPIO8 is a strapping pin:
if it is held LOW at reset, the C3 can enter download boot and look “dead”.
Leave HOME open (pin HIGH) until the board has booted.

## 2. Host tools (once per computer)

### Option A — Arduino IDE 2 (GUI)

1. Install [Arduino IDE 2](https://www.arduino.cc/en/software).
2. **File → Preferences → Additional boards manager URLs**, add:

   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

3. **Boards Manager:** install **esp32** by Espressif Systems.
4. **Library Manager:** install
   - **FastAccelStepper** (gin66)
   - **TMC2209** (janelia-arduino / Peter Polidoro)

### Option B — arduino-cli

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
export PATH="$HOME/bin:$PATH"

arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install FastAccelStepper
arduino-cli lib install TMC2209
```

On Debian/Ubuntu, add your user to `dialout` (log out and back in once):

```bash
sudo usermod -aG dialout "$USER"
```

## 3. Board settings (both IDE and CLI)

| Setting | Value |
| --- | --- |
| Board | **ESP32C3 Dev Module** |
| USB CDC On Boot | **Enabled** |
| Upload Speed | 921600 (drop to 115200 if uploads flake) |
| Flash Size | 4 MB (default) |
| Port | Linux `/dev/ttyACM0`, macOS `/dev/cu.usbmodem*`, Windows `COMx` |

**Arduino IDE:** **Tools → Board → esp32 → ESP32C3 Dev Module**, then
**Tools → USB CDC On Boot → Enabled**, then **Tools → Port**.

**FQBN** used by `arduino-cli`:

```
esp32:esp32:esp32c3:CDCOnBoot=cdc
```

Find the port after Vin + USB are connected:

```bash
# Linux
ls /dev/ttyACM* /dev/ttyUSB*

# macOS
ls /dev/cu.usbmodem*

# Windows (PowerShell)
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name
```

If two ACM devices appear, pick the CDC serial port (the one that shows the
boot banner at 115200). Opening that port **resets** the ESP32-C3 — wait for
the banner before talking to the board.

## 4. Flash GCSP 1.12 (Python / Test Station)

Arduino requires the folder name to match the `.ino` name. In this repo that
is already `firmware/GrafitoCANStepper_C3/GrafitoCANStepper_C3.ino`.

### Arduino IDE

1. **File → Open** `firmware/GrafitoCANStepper_C3/GrafitoCANStepper_C3.ino`.
2. Confirm board / CDC / port from the table above.
3. Click **Upload**. Wait until you see “Hard resetting via RTS pin…”.
4. **Tools → Serial Monitor**, 115200 baud.

### arduino-cli

From the repository root:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3
arduino-cli compile --upload -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3
```

Windows: replace `-p /dev/ttyACM0` with `-p COM5` (your port).

### Healthy GCSP banner

```
# GrafitoCANStepper fw 1.12 proto 1
# HOME pin GPIO8 INPUT_PULLUP raw=HIGH (strapping: keep HIGH at power-on)
# HOME: INPUT_PULLUP + endstop enable=1 active_high=1 action=stop (fw 1.12)
# POWER: enable_on_boot=0 hold<=5 run<=40 standstill=freewheel (fw 1.9)
# MT6701 SSI mode=3 encoder-loss-filter=25ms
# CAN started (1 Mbps)
```

Motors stay **off** until the host calls `enable()` (fw ≥1.9). Continue with
[quickstart.md](quickstart.md) or the
[Test Station](https://docs.grafito.in/dashboard).

GCSP node ID is stored in NVS and **survives a reflash**. Default is **1**.
Change it from Python (`node.set_node_id(n)` then `save_config()`), then
power-cycle. Do this **before** putting two boards on one CAN bus.

## 5. Flash CANopen 2.1 (PLC)

Flash the **folder**, not a lone `.ino`. The sketch includes `canopen_stack.h`.

```
firmware/GrafitoCANStepper_C3_CANopen/
  GrafitoCANStepper_C3_CANopen.ino
  canopen_stack.h
  GrafitoCANStepper.eds
  GrafitoCANStepper_Node1.dcf
  GrafitoCANStepper_Node2.dcf
```

Same board settings as GCSP (ESP32C3 Dev Module, USB CDC On Boot Enabled,
FastAccelStepper + TMC2209).

### Factory node 1 (default)

```bash
arduino-cli compile --upload -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  firmware/GrafitoCANStepper_C3_CANopen
```

Heartbeat COB-ID is `0x701`. Import `GrafitoCANStepper.eds`, then
`GrafitoCANStepper_Node1.dcf` if the PLC wants a commissioned instance.

### Factory node 2 (customer PLC pack)

If the PLC imports **`GrafitoCANStepper_Node2.dcf`**, the board **must** be
node 2. Heartbeat `0x702`, SDO `0x602` / `0x582`, RPDO1 `0x202`, TPDO1 `0x182`.

**arduino-cli:**

```bash
arduino-cli compile --upload -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  --build-property "compiler.cpp.extra_flags=-DCO_FACTORY_NODE_ID=2" \
  firmware/GrafitoCANStepper_C3_CANopen
```

**Arduino IDE:** at the **top** of `GrafitoCANStepper_C3_CANopen.ino` (before
the `#ifndef CO_FACTORY_NODE_ID` block), add:

```cpp
#define CO_FACTORY_NODE_ID 2
```

Then Upload. Revert that line before flashing a node-1 board.

### Healthy CANopen banner (node 2)

```
# GrafitoCANStepper CANopen fw 2.1 CiA402
702 0 00
# CAN started (1000000 bit/s) node 2
702 0 7F
```

Opening the USB serial monitor **resets** the C3. Wait for `7F` (pre-operational
heartbeat) before the PLC starts SDOs.

CANopen NVS namespace is `co402`. It does **not** inherit a GCSP node ID.
After a first flash, if NVS already stored a different ID, write object
`0x2000` = desired ID, `0x2008` = 1 (save), then reset.

PLC import: delete any older **2.0** EDS (revision `0x00020000`), then import
the **2.1** `GrafitoCANStepper.eds` (FileRevision 2, revision `0x00020100`)
and the matching Node DCF.

## 6. Confirm the flash

| You flashed | Serial (115200) | Host check |
| --- | --- | --- |
| GCSP 1.12 | `fw 1.12 proto 1` | `pip install -U grafito-canstepper` then `bus.discover()` → `{1: '1.12'}` |
| CANopen 2.1 node 1 | `CANopen fw 2.1` and `node 1` | Heartbeat `701 0 7F`. Python `discover()` is **empty** (wrong protocol) |
| CANopen 2.1 node 2 | `node 2` | Heartbeat `702 0 7F` |

Empty `discover()` after a CANopen flash is expected. Re-flash GCSP if you
need Python again.

## 7. If upload fails

| Symptom | Fix |
| --- | --- |
| Port missing / board not found | Vin first, then USB. Close Serial Monitor, Test Station, and other scripts. |
| `Failed to connect to ESP32: No serial data` | CDC On Boot was **Disabled** on the last flash, or GPIO8 held LOW at reset. Release HOME, hold **BOOT**, tap **RESET**, retry upload, then release BOOT. |
| Upload starts then stalls | Drop upload speed to 115200. Use a short USB data cable (not charge-only). |
| Linux `Permission denied` | `sudo usermod -aG dialout $USER` and re-login. |
| Banner is CANopen but you wanted Python | You flashed the PLC sketch. Flash `GrafitoCANStepper_C3`. |
| Two boards, CAN silent / collisions | Both still node 1. Set unique IDs **before** joining the bus. |
| PLC talks, motor never enables | Enable `6 → 7 → 15`. If HOME is already active, set `0x200C = 0`. |

## Related

- [Firmware downloads](https://docs.grafito.in/docs/firmware)
- [CANopen / CiA 402](canopen.md) — EDS, DCF, [tune gains from the PLC](canopen.md#tune-gains-from-the-plc)
- [Quickstart](quickstart.md)
- [Hardware (docs site)](https://docs.grafito.in/docs/hardware)
