/*
 * GrafitoCANStepper_C3 — Grafito Innovations CAN stepper node firmware
 *
 * Original firmware for the Grafito CANStepper board:
 *   ESP32-C3 + TMC2209 (UART) + MT6701 magnetic encoder (SSI) + TCAN3413 (CAN).
 *
 * Implements the Grafito CAN Stepper Protocol v1 (GCSP v1).
 * Protocol reference: can_stepper/docs/protocol.md
 *
 * Design notes:
 *  - All tunables live in a single parameter table (SET_PARAM / GET_PARAM),
 *    persisted to NVS with SAVE_CONFIG and loaded at power-on. Motion limits
 *    (max speed, closed-loop speed/accel) are DEFAULTS ONLY — the firmware
 *    never hard-clamps them; change them at runtime and persist as you wish.
 *  - HOME/endstop switch on IO8: configurable enable, polarity and action
 *    (report / stop / stop+zero), plus endstop-based homing (HOME command).
 *  - Homing methods: 0 = zero here, 1 = physical endstop, 2 = StallGuard.
 *  - Closed-loop position control (fw ≥1.2): rest-to-rest trapezoidal
 *    trajectory (accel → cruise → decel) generates a position reference
 *    r(t) and velocity feedforward v_ff(t). A light PI + D-on-velocity
 *    tracks the encoder to r(t). Command: v = v_ff + PID(r − encoder).
 *    FOLLOW mode uses a continuous braking-law chase (no replan thrash).
 *  - Leader/follower: a node can track another node's broadcast POSITION
 *    telemetry (gear ratio, invert, optional encoder correction).
 *  - CAN bus-off self-recovery (a node powered alone must not go mute).
 *  - USB serial <-> CAN bridge so a host can drive the whole bus from any node.
 *
 * Build (Arduino IDE / arduino-cli, esp32 core, board: ESP32C3 Dev Module):
 *  - Tools > USB CDC On Boot: Enabled
 *  - Libraries: FastAccelStepper (gin66), TMC2209 (janelia-arduino)
 *
 * Boot caution — ESP32-C3 strapping pins (datasheet Table 3-3):
 *   IO2 (encoder CS), IO8 (HOME), IO9 (BOOT) sample *only at reset release*.
 *   SPI flash boot wants GPIO8 = 1 (HIGH). After boot, GPIO8 is a normal GPIO.
 *   A CLOSED switch (or optical output) holding IO8 LOW at power-on can force
 *   Joint Download Boot instead of SPI boot — board appears "dead" until released.
 *   Prefer: NO mechanical switch to GND (open at rest = HIGH), or an optical
 *   open-collector that is *not* sinking at power-on.
 *
 * HOME wiring (default in this build: active-high, endstop enabled):
 *   Pressed/active ≈ GPIO8 HIGH; released ≈ LOW (or open if external drive).
 *   ESP32-C3 strapping: keep GPIO8 HIGH at power-on for normal SPI boot —
 *   do not leave a closed active-low switch holding IO8 LOW across reset.
 *   Runtime override still possible via ENDSTOP_* params + SAVE_CONFIG unless
 *   the boot hard-defaults below re-apply them each power-on.
 */

#include <FastAccelStepper.h>
#include <TMC2209.h>
#include <SPI.h>
#include <Preferences.h>
#include "driver/twai.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// ============================================================================
// Firmware / protocol identity
// ============================================================================
static const uint8_t FW_MAJOR  = 1;
static const uint8_t FW_MINOR  = 9;  // 1.9: low-power boot — motors off, low hold, freewheel
static const uint8_t PROTO_VER = 1;

// ============================================================================
// Board pin map (Grafito CANStepper C3 schematic)
// ============================================================================
static const int PIN_CAN_TX   = 21;  // -> TCAN3413 TXD
static const int PIN_CAN_RX   = 20;  // -> TCAN3413 RXD
static const int PIN_TMC_EN   = 3;   // TMC2209 EN (LOW = driver on)
static const int PIN_STEP     = 10;
static const int PIN_DIR      = 1;
static const int PIN_DIAG     = 0;   // TMC2209 DIAG (StallGuard), via 1k
static const int PIN_TMC_TX   = 7;   // PDN_UART via 1k series resistor
static const int PIN_TMC_RX   = 6;   // PDN_UART direct
static const int PIN_ENC_CLK  = 4;   // MT6701 SSI clock
static const int PIN_ENC_DO   = 5;   // MT6701 SSI data
static const int PIN_ENC_CS   = 2;   // MT6701 chip select (strapping pin)
static const int PIN_HOME     = 8;   // endstop input (strapping pin — see note)

// ============================================================================
// GCSP v1 message IDs.  CAN ID = (node_id << 6) | msg_id.  node 0 = broadcast.
// ============================================================================
enum Cmd : uint8_t {
  CMD_PING          = 0,
  CMD_ESTOP         = 1,
  CMD_STOP          = 2,
  CMD_ENABLE        = 3,
  CMD_MOVE_ABS      = 4,   // f64 degrees
  CMD_MOVE_REL      = 5,   // f64 degrees
  CMD_MOVE_VEL      = 6,   // f32 deg/s signed
  CMD_SET_ZERO      = 7,
  CMD_HOME          = 8,   // u8 method, i8 dir, f32 speed deg/s
  CMD_SET_PARAM     = 9,   // u8 param_id, 4-byte value
  CMD_GET_PARAM     = 10,  // u8 param_id
  CMD_SAVE_CONFIG   = 11,
  CMD_LOAD_DEFAULTS = 12,
  CMD_FOLLOW        = 13,  // u8 en, u8 leader, u8 flags, u8 rsvd, f32 ratio
  CMD_FOLLOW_SYNC   = 14,
};

enum Tel : uint8_t {
  TEL_STATUS        = 32,  // flags, mode, fault, fw_major, fw_minor, proto, node
  TEL_POSITION      = 33,  // f64 encoder angle deg (drives followers)
  TEL_MOTION        = 34,  // f32 velocity deg/s, f32 position error deg
  TEL_TARGET        = 35,  // f64 target deg (RTR)
  TEL_EVENT         = 36,  // u8 event, u8 detail, f32 data
  TEL_DRIVER        = 37,  // TMC diagnostics (see sendDriver); 8 bytes
  TEL_ENV           = 38,  // f32 mcu temp C, f32 vbus V
  TEL_PARAM         = 39,  // u8 param_id, u8 status, 4-byte value
  TEL_FOLLOW_STATUS = 40,  // u8 en, u8 leader, u8 flags, u8 synced, f32 ratio
  TEL_CAN_HEALTH    = 41,  // u8 state, u8 tx_err, u8 rx_err, u8 recov, u16, u16
  TEL_ENC_COUNTS    = 42,  // i64 multi-turn encoder counts (RTR)
  TEL_PID_STATUS    = 43,  // u8 state, u8 fault, f32 output deg/s
};

// TEL_DRIVER layout (little-endian, 8 bytes) — GCSP fw ≥1.4
//   [0..1] u16 stallguard result
//   [2]    flags_a:
//            bit0 uart_ok
//            bit1 otpw   (over-temperature pre-warning)
//            bit2 ot     (over-temperature shutdown)
//            bit3 s2ga   (short to GND A)
//            bit4 s2gb
//            bit5 s2vsa  (low-side short A)
//            bit6 s2vsb
//            bit7 ola    (open load A)
//   [3]    flags_b:
//            bit0 olb
//            bit1 t120 / bit2 t143 / bit3 t150 / bit4 t157 (temp thresholds)
//            bit5 stealth_chop_mode
//            bit6 standstill
//            bit7 reserved
//   [4]    gstat: bit0 reset, bit1 drv_err, bit2 uv_cp
//   [5]    cs_actual (0..31) — actual current scale from DRV_STATUS
//   [6..7] u16 interstep duration (TSTEP-style, low 16 bits of getInterstepDuration)

enum Event : uint8_t {
  EVT_BOOT = 1, EVT_ENDSTOP_HIT = 2, EVT_ENDSTOP_RELEASED = 3, EVT_STALL = 4,
  EVT_HOMING_DONE = 5, EVT_HOMING_FAILED = 6, EVT_MOVE_DONE = 7,
  EVT_ESTOP = 8, EVT_FAULT = 9,
};

enum Mode : uint8_t {
  MODE_IDLE = 0, MODE_POSITION = 1, MODE_VELOCITY = 2,
  MODE_HOMING = 3, MODE_FOLLOW = 4,
};

enum Fault : uint8_t {
  FAULT_NONE = 0, FAULT_ENCODER = 1, FAULT_NO_PROGRESS = 2,
  FAULT_HOMING_TIMEOUT = 3,
  FAULT_DRIVER_OT = 4,       // TMC over-temperature shutdown (OT)
  FAULT_DRIVER_SHORT = 5,    // TMC short-to-GND or low-side short
};

enum ParamStatus : uint8_t { PS_OK = 0, PS_UNKNOWN = 1, PS_REJECTED = 2 };

// ============================================================================
// Parameter table.  Each parameter is a 4-byte value (u32 or f32).
// Sanity limits protect hardware addressing/registers only — motion limits
// are user policy: defaults below, fully changeable at runtime, no clamps.
// ============================================================================
enum ParamId : uint8_t {
  P_NODE_ID             = 1,   // u32 1..31
  P_STEPS_PER_REV       = 2,   // u32 full steps per motor revolution
  P_MICROSTEPS          = 3,   // u32 power of two 1..256 (TMC register constraint)
  P_RUN_CURRENT         = 4,   // u32 percent 1..100
  P_HOLD_CURRENT        = 5,   // u32 percent 0..100
  P_STALL_THRESHOLD     = 6,   // u32 SGTHRS 0..255
  P_INVERT_DIR          = 7,   // u32 bool
  P_CLOSED_LOOP         = 8,   // u32 bool
  P_MAX_SPEED           = 9,   // f32 deg/s used for position moves
  P_ACCELERATION        = 10,  // f32 deg/s^2
  P_CL_MAX_SPEED        = 11,  // f32 deg/s closed-loop cruise ceiling (default 720)
  P_CL_MAX_ACCEL        = 12,  // f32 deg/s^2 accel while closed loop (default 2880)
  P_PID_KP              = 13,  // f32
  P_PID_KI              = 14,  // f32
  P_PID_KD              = 15,  // f32
  P_PID_TOLERANCE       = 16,  // f32 deg
  P_FAST_RATE_HZ        = 17,  // u32 0..500  (POSITION/MOTION/PID_STATUS)
  P_SLOW_RATE_HZ        = 18,  // u32 0..500  (STATUS/DRIVER/ENV/FOLLOW/CAN health)
  P_ENABLE_ON_BOOT      = 19,  // u32 bool
  P_ZERO_ON_BOOT        = 20,  // u32 bool
  P_STANDSTILL_MODE     = 21,  // u32 0 normal,1 freewheel,2 brake,3 strong brake
  P_ENDSTOP_ENABLE      = 22,  // u32 bool
  P_ENDSTOP_ACTIVE_HIGH = 23,  // u32 bool (0 = active low + pull-up, default)
  P_ENDSTOP_ACTION      = 24,  // u32 0 report,1 stop,2 stop+zero
  P_HOMING_CURRENT      = 25,  // u32 percent, 0 = keep run current
  P_HOMING_BACKOFF      = 26,  // f32 deg retreat after trigger
  P_HOMING_TIMEOUT_MS   = 27,  // u32
  P_STEALTHCHOP         = 28,  // u32 bool
  PARAM_MAX             = 28,
};

union ParamValue { uint32_t u; float f; };

struct ParamDef {
  uint8_t  id;
  bool     isFloat;
  float    def;      // default (cast for u32 params)
  float    minv;     // sanity range (hardware validity, not policy)
  float    maxv;
};

static const ParamDef PARAM_DEFS[] = {
  { P_NODE_ID,             false, 1,       1,      31       },
  { P_STEPS_PER_REV,       false, 200,     1,      100000   },
  { P_MICROSTEPS,          false, 16,      1,      256      },
  // Currents are % of TMC peak (hardware-dependent). Low defaults for multi-node
  // mesh power: host must raise RUN/HOLD for loaded axes and call enable().
  { P_RUN_CURRENT,         false, 20,      1,      100      },  // was 30
  { P_HOLD_CURRENT,        false, 5,       0,      100      },  // was 15 — idle torque
  { P_STALL_THRESHOLD,     false, 10,      0,      255      },
  { P_INVERT_DIR,          false, 0,       0,      1        },
  { P_CLOSED_LOOP,         false, 1,       0,      1        },
  { P_MAX_SPEED,           true,  720.0f,  0.001f, 1e9f     },
  { P_ACCELERATION,        true,  2880.0f, 0.001f, 1e9f     },
  { P_CL_MAX_SPEED,        true,  720.0f,  0.001f, 1e9f     },
  { P_CL_MAX_ACCEL,        true,  2880.0f, 0.001f, 1e9f     },
  // Tracking gains (velocity FF carries the move; PID trims residual).
  { P_PID_KP,              true,  12.0f,   0.0f,   1e6f     },
  { P_PID_KI,              true,  0.3f,    0.0f,   1e6f     },
  { P_PID_KD,              true,  0.10f,   0.0f,   1e6f     },
  { P_PID_TOLERANCE,       true,  0.35f,   0.001f, 360.0f   },
  { P_FAST_RATE_HZ,        false, 10,      0,      500      },
  { P_SLOW_RATE_HZ,        false, 1,       0,      500      },
  { P_ENABLE_ON_BOOT,      false, 0,       0,      1        },  // was 1 — mesh idle off
  { P_ZERO_ON_BOOT,        false, 0,       0,      1        },
  { P_STANDSTILL_MODE,     false, 1,       0,      3        },  // freewheel when standing
  { P_ENDSTOP_ENABLE,      false, 1,       0,      1        },  // on: stop on HOME
  { P_ENDSTOP_ACTIVE_HIGH, false, 1,       0,      1        },  // pressed = GPIO8 HIGH
  { P_ENDSTOP_ACTION,      false, 1,       0,      2        },  // 1 = stop
  { P_HOMING_CURRENT,      false, 0,       0,      100      },
  { P_HOMING_BACKOFF,      true,  2.0f,    0.0f,   36000.0f },
  { P_HOMING_TIMEOUT_MS,   false, 30000,   100,    600000   },
  { P_STEALTHCHOP,         false, 1,       0,      1        },
};
static const size_t PARAM_COUNT = sizeof(PARAM_DEFS) / sizeof(PARAM_DEFS[0]);

static ParamValue params[PARAM_MAX + 1];   // indexed by ParamId

static const ParamDef *paramDef(uint8_t id) {
  for (size_t i = 0; i < PARAM_COUNT; i++)
    if (PARAM_DEFS[i].id == id) return &PARAM_DEFS[i];
  return nullptr;
}
static inline uint32_t pu(uint8_t id) { return params[id].u; }
static inline float    pf(uint8_t id) { return params[id].f; }

// ============================================================================
// Globals
// ============================================================================
Preferences prefs;
TMC2209 tmc;
FastAccelStepperEngine stepperEngine;
FastAccelStepper *stepper = nullptr;
static SPIClass encSpi(FSPI);

static uint8_t  nodeId = 1;
static uint8_t  mode = MODE_IDLE;
static uint8_t  fault = FAULT_NONE;
static bool     driverEnabled = false;
static bool     estopLatched = false;
static bool     homed = false;

// encoder (MT6701, 14-bit angle -> 16384 counts/rev, multi-turn accumulated)
static const int32_t ENC_CPR = 16384;
static int64_t  encCounts = 0;         // multi-turn counts minus zero offset
static int64_t  encZeroOffset = 0;
static bool     encFrameOk = true;
static uint32_t encBadFrames = 0;
static bool     encHasGoodFrame = false;
static uint32_t encLastGoodUs = 0;
// A single SSI CRC error must not abort a move. Hold the last valid sample and
// fault only if feedback is continuously unavailable for this long.
static const uint32_t ENC_LOSS_TIMEOUT_US = 25000;  // five 200 Hz PID periods

// motion bookkeeping
static double   targetDeg = 0.0;       // active position/homing/follow target
static float    velTargetDegS = 0.0f;  // active velocity command
static int      cmdDirSign = 0;        // sign of last commanded motion
static bool     moveDonePending = false;

// Closed-loop: trapezoidal trajectory + velocity feedforward + tracking PID
static const uint32_t PID_PERIOD_US = 5000;     // 200 Hz
static const uint32_t PID_SETTLE_MS = 80;
static const uint32_t PID_STALL_TIMEOUT_MS = 3000;
static const float    PID_SETTLE_VEL_FRAC = 0.06f;
static const float    PID_SETTLE_VEL_MIN  = 12.0f;
static uint8_t  pidState = 0;          // 0 idle, 1 running, 2 settled, 3 fault
static float    pidOutDegS = 0.0f;
static float    pidErrDeg = 0.0f;      // target − measured (telemetry)
static float    pidIntegral = 0.0f;
static float    pidVelFilt = 0.0f;
static double   pidPrevDeg = 0.0;
static uint32_t pidPrevUs = 0;
static uint32_t pidSettleStartMs = 0;
static float    pidBestErr = 1e30f;
static uint32_t pidProgressMs = 0;
static bool     pidStopping = false;   // waiting for stop before reversing

// Rest-to-rest trapezoid: accel → cruise → decel (Klipper-style generator).
// POSITION moves plan once at beginPositionMove; FOLLOW uses online braking.
static bool   trajActive = false;
static bool   trajDone = true;
static double trajS0 = 0.0;
static double trajS1 = 0.0;
static float  trajSign = 1.0f;
static float  trajDist = 0.0f;
static float  trajVpeak = 0.0f;
static float  trajA = 1.0f;
static float  trajTacc = 0.0f;
static float  trajTcruise = 0.0f;
static float  trajTdec = 0.0f;
static float  trajTtot = 0.0f;
static float  trajT = 0.0f;
static double trajRef = 0.0;           // position reference r(t)
static float  trajVff = 0.0f;          // velocity feedforward v_ff(t)

// endstop (GPIO8 / PIN_HOME — strapping pin, see file header)
static bool     endstopActive = false;   // logical active (after polarity)
static bool     endstopRawHigh = true;   // last digitalRead (true = pin HIGH)
static uint32_t endstopChangeMs = 0;
static const uint32_t ENDSTOP_DEBOUNCE_MS = 5;

// homing
static uint8_t  homingMethod = 0;
static int8_t   homingDir = 1;
static float    homingSpeed = 0.0f;
static uint8_t  homingPhase = 0;       // 0 off, 1 seeking, 2 backing off
static uint32_t homingStartMs = 0;
static uint32_t savedRunCurrent = 0;

// StallGuard
static volatile bool stallFlag = false;

// follower
static bool    followEnabled = false;
static uint8_t followLeader = 0;
static bool    followInvert = false;
static bool    followEncCorrected = true;
static float   followRatio = 1.0f;
static bool    followSynced = false;
static double  followLeaderRefDeg = 0.0;
static double  followAcceptedDeg = 0.0;
static double  followLocalRefDeg = 0.0;
static int32_t followLocalRefSteps = 0;
static int32_t followLastTarget = 0;
static bool    followTargetValid = false;
static const double FOLLOW_DEADBAND_DEG = 0.15;

// CAN health
static uint8_t  canRecoveries = 0;
static bool     canRestartPending = false;
static uint32_t canHealthMs = 0;

// telemetry scheduling
static uint32_t fastTelMs = 0;
static uint32_t slowTelMs = 0;

// serial bridge line buffer
static char   lineBuf[80];
static size_t lineLen = 0;

// ============================================================================
// Small helpers
// ============================================================================
static inline double stepsPerDeg() {
  return (double)pu(P_STEPS_PER_REV) * (double)pu(P_MICROSTEPS) / 360.0;
}
static inline double countsToDeg(int64_t c) { return (double)c * 360.0 / ENC_CPR; }
static inline double encoderDeg() { return countsToDeg(encCounts); }

static void IRAM_ATTR onDiagRise() { stallFlag = true; }

// ============================================================================
// MT6701 encoder over SSI (24-bit frame: 14-bit angle, 4-bit status, 6-bit CRC)
// ============================================================================
static uint8_t crc6_itu(uint32_t data18) {
  // CRC-6/ITU, poly x^6 + x + 1, over the 18 data bits, MSB first.
  uint8_t crc = 0;
  for (int i = 17; i >= 0; i--) {
    uint8_t bit = (data18 >> i) & 1;
    uint8_t msb = (crc >> 5) & 1;
    crc = (uint8_t)((crc << 1) & 0x3F);
    if (bit ^ msb) crc ^= 0x03;
  }
  return crc;
}

static uint32_t encoderReadFrame() {
  // MODE3 is hardware-verified on the CANStepper ESP32-C3 board. Keep this in
  // sync with mt6701_ssi_test.ino; other modes return invalid/zero feedback on
  // this SPI peripheral despite the edge terminology used in the datasheet.
  encSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ENC_CS, LOW);
  delayMicroseconds(1);
  uint8_t a = encSpi.transfer(0x00);
  uint8_t b = encSpi.transfer(0x00);
  uint8_t c = encSpi.transfer(0x00);
  delayMicroseconds(1);
  digitalWrite(PIN_ENC_CS, HIGH);
  encSpi.endTransaction();
  return ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;
}

static bool encoderUpdate() {
  static uint16_t prevRaw = 0;
  static int32_t  turns = 0;
  static bool     first = true;

  uint32_t frame = encoderReadFrame();
  uint16_t raw   = (frame >> 10) & 0x3FFF;
  uint8_t  rxCrc = frame & 0x3F;

  // A stuck-low or stuck-high data line is a wiring fault, not an angle.
  bool stuck = (frame == 0x000000) || ((frame & 0xFFFFC0) == 0xFFFFC0);
  bool crcOk = (crc6_itu((frame >> 6) & 0x3FFFF) == rxCrc);
  encFrameOk = crcOk && !stuck;
  if (!encFrameOk) { encBadFrames++; return false; }  // keep last good sample

  encHasGoodFrame = true;
  encLastGoodUs = micros();

  if (first) { prevRaw = raw; first = false; }
  int32_t quarter = ENC_CPR / 4;
  if ((int32_t)prevRaw > 3 * quarter && (int32_t)raw < quarter) turns++;
  else if ((int32_t)prevRaw < quarter && (int32_t)raw > 3 * quarter) turns--;
  prevRaw = raw;

  encCounts = (int64_t)raw + (int64_t)turns * ENC_CPR - encZeroOffset;
  return true;
}

static void encoderZeroHere() {
  encoderUpdate();
  encZeroOffset += encCounts;
  encCounts = 0;
}

// ============================================================================
// CAN (raw ESP-IDF TWAI) + serial bridge
// ============================================================================
static bool canStart() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16;
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

static void serialPrintFrame(uint32_t id, bool rtr, const uint8_t *data, uint8_t len) {
  if (!Serial) return;
  char out[48];
  int n = snprintf(out, sizeof(out), "%lX %d ", (unsigned long)id, rtr ? 1 : 0);
  for (uint8_t i = 0; i < len && n < (int)sizeof(out) - 3; i++)
    n += snprintf(out + n, sizeof(out) - n, "%02X", data[i]);
  out[n++] = '\n'; out[n] = 0;
  Serial.print(out);
}

static void canSend(uint8_t msgId, const void *data, uint8_t len) {
  twai_message_t m;
  memset(&m, 0, sizeof(m));
  m.identifier = ((uint32_t)nodeId << 6) | msgId;
  m.data_length_code = len > 8 ? 8 : len;
  if (data && len) memcpy(m.data, data, m.data_length_code);
  twai_transmit(&m, 0);
  serialPrintFrame(m.identifier, false, m.data, m.data_length_code);
}

// ============================================================================
// Telemetry payload builders
// ============================================================================
static void sendStatus() {
  uint8_t flags = 0;
  if (driverEnabled)                    flags |= 0x01;
  if (stepper && stepper->isRunning())  flags |= 0x02;
  if (homed)                            flags |= 0x04;
  if (estopLatched)                     flags |= 0x08;
  if (endstopActive)                    flags |= 0x10;
  if (stallFlag)                        flags |= 0x20;
  if (encFrameOk)                       flags |= 0x40;
  // Byte 7 (fw ≥1.5): GPIO8 / HOME diagnostics (strapping pin — level after boot)
  //   bit0: raw pin HIGH (1) / LOW (0) from digitalRead
  //   bit1: logical endstop active (same as flags bit4)
  //   bit2: ENDSTOP_ENABLE param
  //   bit3: ENDSTOP_ACTIVE_HIGH param
  uint8_t homeDiag = 0;
  if (endstopRawHigh)             homeDiag |= 0x01;
  if (endstopActive)              homeDiag |= 0x02;
  if (pu(P_ENDSTOP_ENABLE))      homeDiag |= 0x04;
  if (pu(P_ENDSTOP_ACTIVE_HIGH)) homeDiag |= 0x08;
  uint8_t p[8] = { flags, mode, fault, FW_MAJOR, FW_MINOR, PROTO_VER, nodeId, homeDiag };
  canSend(TEL_STATUS, p, 8);
}

static void sendPosition() {
  double d = encoderDeg();
  canSend(TEL_POSITION, &d, 8);
}

static void sendMotion() {
  float v = pidVelFilt;   // encoder-derived, filtered (updated by control loop)
  float e = (mode == MODE_POSITION || mode == MODE_HOMING || mode == MODE_FOLLOW)
              ? (float)(targetDeg - encoderDeg()) : 0.0f;
  uint8_t p[8];
  memcpy(p, &v, 4); memcpy(p + 4, &e, 4);
  canSend(TEL_MOTION, p, 8);
}

static void sendEvent(uint8_t evt, uint8_t detail, float data) {
  uint8_t p[8] = {0};
  p[0] = evt; p[1] = detail;
  memcpy(p + 2, &data, 4);
  canSend(TEL_EVENT, p, 6);
}

// Cached last TMC DRV_STATUS / GSTAT snapshot for diagnostics + fault latch.
static uint16_t tmcStallGuard = 0;
static uint8_t  tmcFlagsA = 0;
static uint8_t  tmcFlagsB = 0;
static uint8_t  tmcGstat = 0;
static uint8_t  tmcCsActual = 0;
static uint16_t tmcInterstep = 0;
static bool     tmcOtLatched = false;
static bool     tmcShortLatched = false;
static uint32_t tmcPollMs = 0;

// Refresh TMC UART registers. Safe to call from telemetry / loop (not ISR).
static void tmcPollStatus() {
  bool uartOk = tmc.isSetupAndCommunicating();
  tmcFlagsA = uartOk ? 0x01 : 0x00;
  tmcFlagsB = 0;
  tmcGstat = 0;
  tmcCsActual = 0;
  tmcInterstep = 0;
  tmcStallGuard = 0;
  if (!uartOk) return;

  tmcStallGuard = (uint16_t)tmc.getStallGuardResult();
  TMC2209::Status st = tmc.getStatus();
  TMC2209::GlobalStatus gs = tmc.getGlobalStatus();
  uint32_t tstep = tmc.getInterstepDuration();
  tmcInterstep = (uint16_t)(tstep > 0xFFFFu ? 0xFFFFu : tstep);
  tmcCsActual = (uint8_t)(st.current_scaling & 0x1F);

  if (st.over_temperature_warning)  tmcFlagsA |= (1u << 1);
  if (st.over_temperature_shutdown) tmcFlagsA |= (1u << 2);
  if (st.short_to_ground_a)         tmcFlagsA |= (1u << 3);
  if (st.short_to_ground_b)         tmcFlagsA |= (1u << 4);
  if (st.low_side_short_a)          tmcFlagsA |= (1u << 5);
  if (st.low_side_short_b)          tmcFlagsA |= (1u << 6);
  if (st.open_load_a)               tmcFlagsA |= (1u << 7);
  if (st.open_load_b)               tmcFlagsB |= (1u << 0);
  if (st.over_temperature_120c)     tmcFlagsB |= (1u << 1);
  if (st.over_temperature_143c)     tmcFlagsB |= (1u << 2);
  if (st.over_temperature_150c)     tmcFlagsB |= (1u << 3);
  if (st.over_temperature_157c)     tmcFlagsB |= (1u << 4);
  if (st.stealth_chop_mode)         tmcFlagsB |= (1u << 5);
  if (st.standstill)                tmcFlagsB |= (1u << 6);

  if (gs.reset)  tmcGstat |= (1u << 0);
  if (gs.drv_err) tmcGstat |= (1u << 1);
  if (gs.uv_cp)  tmcGstat |= (1u << 2);

  // Latch serious driver conditions once so hosts can see them in STATUS.fault
  // even if the TMC clears the bit after cooling / re-enable.
  if (st.over_temperature_shutdown) tmcOtLatched = true;
  if (st.short_to_ground_a || st.short_to_ground_b ||
      st.low_side_short_a || st.low_side_short_b) {
    tmcShortLatched = true;
  }
}

static void sendDriver() {
  tmcPollStatus();
  uint8_t p[8] = {0};
  memcpy(p, &tmcStallGuard, 2);
  p[2] = tmcFlagsA;
  p[3] = tmcFlagsB;
  p[4] = tmcGstat;
  p[5] = tmcCsActual;
  memcpy(p + 6, &tmcInterstep, 2);
  canSend(TEL_DRIVER, p, 8);
}

// Periodic TMC health: raise FAULT_DRIVER_OT / FAULT_DRIVER_SHORT and stop motion.
static void tmcDriverService() {
  uint32_t now = millis();
  if ((uint32_t)(now - tmcPollMs) < 100) return;
  tmcPollMs = now;
  if (!driverEnabled) return;

  tmcPollStatus();
  bool ot = (tmcFlagsA & (1u << 2)) != 0;
  bool shorted = (tmcFlagsA & ((1u << 3) | (1u << 4) | (1u << 5) | (1u << 6))) != 0;

  if (ot && fault != FAULT_DRIVER_OT) {
    stopImmediate();
    pidState = 3;
    fault = FAULT_DRIVER_OT;
    sendEvent(EVT_FAULT, FAULT_DRIVER_OT, (float)temperatureRead());
    Serial.println("# FAULT: TMC over-temperature shutdown (OT)");
  } else if (shorted && fault != FAULT_DRIVER_SHORT && fault != FAULT_DRIVER_OT) {
    stopImmediate();
    pidState = 3;
    fault = FAULT_DRIVER_SHORT;
    sendEvent(EVT_FAULT, FAULT_DRIVER_SHORT, (float)tmcFlagsA);
    Serial.println("# FAULT: TMC short detected (S2G/S2VS)");
  }

  // OTPW alone: log once per edge via serial (do not stop — warning only).
  static bool otpwPrev = false;
  bool otpw = (tmcFlagsA & (1u << 1)) != 0;
  if (otpw && !otpwPrev) {
    Serial.println("# WARN: TMC over-temperature pre-warning (OTPW)");
  }
  otpwPrev = otpw;
}

static void sendEnv() {
  float t = temperatureRead();
  float v = 24.0f;   // fixed motor rail; the C3 board has no VBUS sense pin
  uint8_t p[8];
  memcpy(p, &t, 4); memcpy(p + 4, &v, 4);
  canSend(TEL_ENV, p, 8);
}

static void sendParam(uint8_t id, uint8_t status) {
  uint8_t p[8] = {0};
  p[0] = id; p[1] = status;
  if (id <= PARAM_MAX) memcpy(p + 2, &params[id], 4);
  canSend(TEL_PARAM, p, 6);
}

static void sendFollowStatus() {
  uint8_t p[8] = {0};
  p[0] = followEnabled ? 1 : 0;
  p[1] = followLeader;
  p[2] = (followInvert ? 1 : 0) | (followEncCorrected ? 2 : 0);
  p[3] = followSynced ? 1 : 0;
  memcpy(p + 4, &followRatio, 4);
  canSend(TEL_FOLLOW_STATUS, p, 8);
}

static void sendCanHealth() {
  uint8_t p[8] = {0};
  twai_status_info_t s;
  if (twai_get_status_info(&s) == ESP_OK) {
    p[0] = (uint8_t)s.state;
    p[1] = (uint8_t)(s.tx_error_counter > 255 ? 255 : s.tx_error_counter);
    p[2] = (uint8_t)(s.rx_error_counter > 255 ? 255 : s.rx_error_counter);
    p[3] = canRecoveries;
    uint16_t txf = s.tx_failed_count > 65535 ? 65535 : s.tx_failed_count;
    uint16_t bus = s.bus_error_count > 65535 ? 65535 : s.bus_error_count;
    memcpy(p + 4, &txf, 2); memcpy(p + 6, &bus, 2);
  } else {
    p[0] = 0xFF;
  }
  canSend(TEL_CAN_HEALTH, p, 8);
}

static void sendPidStatus() {
  uint8_t p[8] = {0};
  p[0] = pidState; p[1] = fault;
  memcpy(p + 2, &pidOutDegS, 4);
  canSend(TEL_PID_STATUS, p, 6);
}

static void sendTarget()    { canSend(TEL_TARGET, &targetDeg, 8); }
static void sendEncCounts() { int64_t c = encCounts; canSend(TEL_ENC_COUNTS, &c, 8); }

// ============================================================================
// Motion primitives
// ============================================================================
static void applySpeedAccel() {
  if (!stepper) return;
  bool closed = pu(P_CLOSED_LOOP) && (mode == MODE_POSITION || mode == MODE_FOLLOW);
  float spd = closed ? fminf(pf(P_MAX_SPEED), pf(P_CL_MAX_SPEED)) : pf(P_MAX_SPEED);
  float acc = closed ? pf(P_CL_MAX_ACCEL) : pf(P_ACCELERATION);
  uint32_t hz = (uint32_t)llround(fabsf(spd) * stepsPerDeg());
  int32_t  a  = (int32_t)llround(fabsf(acc) * stepsPerDeg());
  stepper->setSpeedInHz(hz < 1 ? 1 : hz);
  stepper->setAcceleration(a < 1 ? 1 : a);
}

static void stopImmediate() {
  if (stepper) stepper->forceStop();
  cmdDirSign = 0;
  velTargetDegS = 0.0f;
  pidState = 0; pidIntegral = 0; pidOutDegS = 0; pidStopping = false;
  trajClear();
}

static void stopRamped() {
  if (stepper) stepper->stopMove();
  cmdDirSign = 0;
  velTargetDegS = 0.0f;
  pidState = 0; pidIntegral = 0; pidOutDegS = 0; pidStopping = false;
  trajClear();
  mode = MODE_IDLE;
}

static void setDriverEnabled(bool en) {
  driverEnabled = en;
  if (en) {
    tmc.enable();
    digitalWrite(PIN_TMC_EN, LOW);
    estopLatched = false;
    // Clear sticky TMC GSTAT / thermal latches when the host re-enables.
    tmc.clearDriveError();
    tmc.clearReset();
    tmcOtLatched = false;
    tmcShortLatched = false;
    if (fault == FAULT_DRIVER_OT || fault == FAULT_DRIVER_SHORT) {
      fault = FAULT_NONE;
      pidState = 0;
    }
  } else {
    stopImmediate();
    followSynced = false;
    tmc.disable();
    digitalWrite(PIN_TMC_EN, HIGH);
    mode = MODE_IDLE;
  }
}

static void emergencyStop() {
  stopImmediate();
  driverEnabled = false;
  tmc.disable();
  digitalWrite(PIN_TMC_EN, HIGH);
  estopLatched = true;
  followEnabled = false;
  followSynced = false;
  homingPhase = 0;
  mode = MODE_IDLE;
  sendEvent(EVT_ESTOP, 0, (float)encoderDeg());
}

static void trajClear() {
  trajActive = false;
  trajDone = true;
  trajT = 0.0f;
  trajVff = 0.0f;
  trajVpeak = 0.0f;
  trajTtot = 0.0f;
}

// Plan a rest-to-rest trapezoid (or triangle if distance is short) from s0→s1.
static void trajPlan(double s0, double s1, float vmax, float amax) {
  trajS0 = s0;
  trajS1 = s1;
  float d = (float)(s1 - s0);
  trajDist = fabsf(d);
  trajSign = (d >= 0.0f) ? 1.0f : -1.0f;
  trajA = fmaxf(amax, 1.0f);
  vmax = fmaxf(vmax, 0.001f);
  trajT = 0.0f;
  trajRef = s0;
  trajVff = 0.0f;
  trajActive = true;

  if (trajDist < 1e-4f) {
    trajVpeak = 0.0f;
    trajTacc = trajTcruise = trajTdec = trajTtot = 0.0f;
    trajDone = true;
    trajRef = s1;
    return;
  }

  // Full accel distance to vmax: v²/(2a). If 2× that exceeds the move, triangle.
  float dAccFull = (vmax * vmax) / (2.0f * trajA);
  if (2.0f * dAccFull >= trajDist) {
    trajVpeak = sqrtf(trajA * trajDist);   // v²/a = dist  ⇒  v = √(a·dist)
    trajTacc = trajVpeak / trajA;
    trajTdec = trajTacc;
    trajTcruise = 0.0f;
  } else {
    trajVpeak = vmax;
    trajTacc = vmax / trajA;
    trajTdec = trajTacc;
    trajTcruise = (trajDist - 2.0f * dAccFull) / vmax;
  }
  trajTtot = trajTacc + trajTcruise + trajTdec;
  trajDone = false;
}

// Advance the trapezoid by dt; updates trajRef and trajVff.
static void trajAdvance(float dt) {
  if (!trajActive) return;
  if (trajDone) {
    trajRef = trajS1;
    trajVff = 0.0f;
    return;
  }
  trajT += dt;
  if (trajT >= trajTtot - 1e-7f) {
    trajT = trajTtot;
    trajDone = true;
    trajRef = trajS1;
    trajVff = 0.0f;
    return;
  }

  float t = trajT;
  float sLocal, vLocal;
  float dAcc = 0.5f * trajA * trajTacc * trajTacc;

  if (t <= trajTacc + 1e-9f) {
    vLocal = trajA * t;
    sLocal = 0.5f * trajA * t * t;
  } else if (t <= trajTacc + trajTcruise + 1e-9f) {
    float tc = t - trajTacc;
    vLocal = trajVpeak;
    sLocal = dAcc + trajVpeak * tc;
  } else {
    float td = t - trajTacc - trajTcruise;
    vLocal = trajVpeak - trajA * td;
    if (vLocal < 0.0f) vLocal = 0.0f;
    float dCruise = trajVpeak * trajTcruise;
    sLocal = dAcc + dCruise + trajVpeak * td - 0.5f * trajA * td * td;
  }
  if (sLocal > trajDist) sLocal = trajDist;
  if (sLocal < 0.0f) sLocal = 0.0f;
  trajRef = trajS0 + (double)(trajSign * sLocal);
  trajVff = trajSign * vLocal;
}

static void pidReset() {
  pidIntegral = 0.0f;
  pidOutDegS = 0.0f;
  pidErrDeg = 0.0f;
  pidPrevUs = 0;
  pidSettleStartMs = 0;
  pidBestErr = 1e30f;
  pidProgressMs = millis();
  pidStopping = false;
  trajClear();
  fault = FAULT_NONE;
}

// Begin a position move to `deg` (mode must already be set by the caller).
static void beginPositionMove(double deg) {
  if (!stepper || !driverEnabled || estopLatched) return;
  targetDeg = deg;
  moveDonePending = true;
  applySpeedAccel();
  if (pu(P_CLOSED_LOOP)) {
    bool wasRunning = stepper->isRunning();
    if (wasRunning) stepper->forceStop();
    cmdDirSign = 0;
    encoderUpdate();
    double here = encoderDeg();
    pidPrevDeg = here;
    pidReset();
    pidStopping = wasRunning;
    float vmax = fminf(pf(P_MAX_SPEED), pf(P_CL_MAX_SPEED));
    float amax = pf(P_CL_MAX_ACCEL);
    trajPlan(here, deg, vmax, amax);
    pidState = 1;
  } else {
    pidState = 0;
    trajClear();
    int64_t steps = (int64_t)llround(deg * stepsPerDeg());
    if (steps > INT32_MAX) steps = INT32_MAX;
    if (steps < INT32_MIN) steps = INT32_MIN;
    cmdDirSign = (steps >= stepper->getCurrentPosition()) ? 1 : -1;
    stepper->moveTo((int32_t)steps);
  }
}

// Start (or re-assert) a continuous run. Hardware-verified quirk: a single
// runForward()/runBackward() issued after a forceStop() is swallowed — the
// step queue keeps ignoring commands until the engine task observes an
// active ramp. Position moves recover via moveTo()/the PID retry loop, but
// continuous runs must be re-asserted until stepping actually starts; see
// velocityService()/homingService().
static void applyContinuousRun(float degS) {
  uint32_t hz = (uint32_t)llround(fabs((double)degS) * stepsPerDeg());
  if (hz < 1) hz = 1;
  stepper->setSpeedInHz(hz);
  if (degS > 0) { stepper->runForward(); cmdDirSign = 1; }
  else          { stepper->runBackward(); cmdDirSign = -1; }
}

static void beginVelocity(float degS) {
  if (!stepper || !driverEnabled || estopLatched) return;
  mode = MODE_VELOCITY;
  velTargetDegS = degS;
  pidState = 0;
  moveDonePending = false;
  uint32_t hz = (uint32_t)llround(fabs((double)degS) * stepsPerDeg());
  int32_t acc = (int32_t)llround(fabsf(pf(P_ACCELERATION)) * stepsPerDeg());
  stepper->setAcceleration(acc < 1 ? 1 : acc);
  if (hz < 1) { stepper->stopMove(); cmdDirSign = 0; velTargetDegS = 0.0f; mode = MODE_IDLE; return; }
  applyContinuousRun(degS);
}

// Re-assert a commanded velocity until the step generator is actually
// running (recovers the swallowed first command after a forceStop()).
static void velocityService() {
  if (mode != MODE_VELOCITY || !stepper || !driverEnabled || estopLatched) return;
  if (velTargetDegS == 0.0f || stepper->isRunning()) return;
  applyContinuousRun(velTargetDegS);
}

// ============================================================================
// Closed-loop position controller (200 Hz)
//
// POSITION (fw ≥1.2): trapezoidal trajectory generator + velocity feedforward
//   plan once at beginPositionMove (accel → cruise → decel, or triangle)
//   each tick: r(t), v_ff(t) from the plan
//   v_cmd = v_ff + Kp·(r − encoder) + Ki·∫ + (−Kd·v_meas)
//   Settle when plan is done, |target−enc| ≤ tol, and |vel| is low.
//
// FOLLOW (encoder-corrected): continuous braking-law chase toward the
//   moving leader target (re-planning a rest-to-rest trap every tick would
//   thrash). Same settle / reverse / rate-limit machinery.
//
// Open-loop position still uses FastAccelStepper's own trapezoid via moveTo.
// ============================================================================
static void pidService() {
  uint32_t nowUs = micros();
  if (pidPrevUs != 0 && (uint32_t)(nowUs - pidPrevUs) < PID_PERIOD_US) return;
  float dt = (pidPrevUs == 0) ? (PID_PERIOD_US * 1e-6f)
                              : ((uint32_t)(nowUs - pidPrevUs)) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = PID_PERIOD_US * 1e-6f;
  pidPrevUs = nowUs;

  bool sampleOk = encoderUpdate();
  double measured = encoderDeg();
  float velRaw = (float)((measured - pidPrevDeg) / dt);
  pidPrevDeg = measured;
  pidVelFilt += 0.30f * (velRaw - pidVelFilt);

  bool isFollow = (mode == MODE_FOLLOW && followEncCorrected);
  bool isPos = (mode == MODE_POSITION);
  bool active = pu(P_CLOSED_LOOP) && pidState == 1 && driverEnabled &&
                (isPos || isFollow);
  if (!active || !stepper) return;

  bool encoderLost = !encHasGoodFrame ||
      (!sampleOk && (uint32_t)(micros() - encLastGoodUs) >= ENC_LOSS_TIMEOUT_US);
  if (encoderLost) {
    stopImmediate();
    pidState = 3; fault = FAULT_ENCODER;
    sendEvent(EVT_FAULT, FAULT_ENCODER, (float)measured);
    return;
  }

  float vmax = fminf(pf(P_MAX_SPEED), pf(P_CL_MAX_SPEED));
  if (vmax < 0.001f) vmax = 0.001f;
  float amax = pf(P_CL_MAX_ACCEL);
  if (amax < 1.0f) amax = 1.0f;
  float tol = pf(P_PID_TOLERANCE);
  float kp = pf(P_PID_KP);
  float ki = pf(P_PID_KI);
  float kd = pf(P_PID_KD);
  uint32_t nowMs = millis();
  float settleVel = fmaxf(PID_SETTLE_VEL_MIN, PID_SETTLE_VEL_FRAC * vmax);
  float absVel = fabsf(pidVelFilt);

  // --- Reference + feedforward ----------------------------------------------
  float vFf = 0.0f;
  float trackErr;   // reference − measured (PID input)
  float targetErr = (float)(targetDeg - measured);
  pidErrDeg = targetErr;
  float absTargetErr = fabsf(targetErr);

  if (isPos && trajActive) {
    trajAdvance(dt);
    vFf = trajVff;
    trackErr = (float)(trajRef - measured);
    // After the plan finishes, hold the final target (same as trajS1).
    if (trajDone) {
      vFf = 0.0f;
      trackErr = targetErr;
    }
  } else {
    // FOLLOW (or no plan): time-optimal braking envelope toward live target.
    // v_ff = sign(e)·min(vmax, √(a·|e|))  — conservative (margin vs √(2as)).
    float vBrake = sqrtf(amax * absTargetErr + 1e-6f);
    float vCap = fminf(vmax, vBrake);
    if ((pidVelFilt * targetErr) > 0.0f) {
      float stopDist = (pidVelFilt * pidVelFilt) / (amax + 1e-6f);
      if (stopDist > absTargetErr) vCap = 0.0f;
    }
    vFf = copysignf(vCap, targetErr);
    trackErr = targetErr;  // chase the live target
  }

  // --- Settle ---------------------------------------------------------------
  bool planComplete = !isPos || !trajActive || trajDone;
  if (planComplete && absTargetErr <= tol && absVel <= settleVel) {
    if (stepper->isRunning()) { stepper->stopMove(); pidStopping = true; }
    else pidStopping = false;
    cmdDirSign = 0;
    pidOutDegS = 0.0f;
    pidIntegral *= 0.85f;
    pidBestErr = absTargetErr;
    pidProgressMs = nowMs;
    if (pidSettleStartMs == 0) pidSettleStartMs = nowMs;
    if ((uint32_t)(nowMs - pidSettleStartMs) >= PID_SETTLE_MS) {
      pidState = 2;
      if (moveDonePending && mode == MODE_POSITION) {
        moveDonePending = false;
        sendEvent(EVT_MOVE_DONE, 0, (float)measured);
      }
    }
    return;
  }
  pidSettleStartMs = 0;
  if (pidState == 2) pidState = 1;

  // --- No-progress watchdog -------------------------------------------------
  if (absTargetErr < pidBestErr - 0.011f) {
    pidBestErr = absTargetErr;
    pidProgressMs = nowMs;
  } else if (absTargetErr > 1.0f && (pidVelFilt * targetErr) > 0.0f && absVel > 5.0f) {
    pidProgressMs = nowMs;
  } else if (isPos && trajActive && !trajDone) {
    // Still executing the planned profile — not stuck.
    pidProgressMs = nowMs;
  }
  if (absTargetErr > fmaxf(1.0f, tol * 4.0f) &&
      (uint32_t)(nowMs - pidProgressMs) > PID_STALL_TIMEOUT_MS) {
    stopImmediate();
    pidState = 3; fault = FAULT_NO_PROGRESS;
    sendEvent(EVT_FAULT, FAULT_NO_PROGRESS, targetErr);
    return;
  }

  // --- Tracking PID on (r − encoder), added to v_ff -------------------------
  float integ = pidIntegral + trackErr * dt;
  if (ki > 1e-6f) {
    float ilim = (0.20f * vmax) / ki;
    integ = constrain(integ, -ilim, ilim);
  } else {
    integ = 0.0f;
  }
  // D on measured velocity only (no derivative kick from reference steps).
  float trim = kp * trackErr + ki * integ - kd * pidVelFilt;
  float trimCap = fmaxf(0.25f * vmax, 40.0f);
  trim = constrain(trim, -trimCap, trimCap);

  float out = vFf + trim;
  // Keep |out| within a soft headroom of vmax (FF already ≤ vmax).
  float outLim = vmax * 1.15f;
  out = constrain(out, -outLim, outLim);

  // Anti-windup: only grow integral when trim is not saturated.
  if (fabsf(trim) < trimCap * 0.98f || trim * trackErr < 0.0f) {
    pidIntegral = integ;
  }

  // Rate-limit command (match amax; stronger decel authority).
  float maxAcc = amax * dt;
  float maxDec = amax * 2.0f * dt;
  float prev = pidOutDegS;
  float delta = out - prev;
  bool isDecel = (fabsf(out) < fabsf(prev)) || (out * prev < 0.0f);
  float maxDelta = isDecel ? maxDec : maxAcc;
  if (delta >  maxDelta) out = prev + maxDelta;
  if (delta < -maxDelta) out = prev - maxDelta;

  // No instantaneous reverse while the step engine is still moving.
  if (cmdDirSign != 0 && out * (float)cmdDirSign < 0.0f && fabsf(prev) > 1.0f) {
    out = 0.0f;
    delta = out - prev;
    if (delta >  maxDec) out = prev + maxDec;
    if (delta < -maxDec) out = prev - maxDec;
  }

  float floorDegS = 360.0f / ((float)pu(P_STEPS_PER_REV) * (float)pu(P_MICROSTEPS));
  if (absTargetErr > fmaxf(tol * 4.0f, 1.0f) && fabsf(out) < floorDegS) {
    // Only force a microstep floor while the plan is still cruising/far.
    if (!(isPos && trajDone)) {
      out = copysignf(fmaxf(floorDegS, 0.5f), trackErr != 0.0f ? trackErr : targetErr);
    }
  }
  pidOutDegS = out;

  int dir = out > 0.5f * floorDegS ? 1 : (out < -0.5f * floorDegS ? -1 : 0);

  if (pidStopping) {
    if (stepper->isRunning()) { pidOutDegS = 0.0f; return; }
    pidStopping = false;
    cmdDirSign = 0;
  }

  if (dir == 0) {
    if (stepper->isRunning()) {
      stepper->stopMove();
      pidStopping = true;
    } else {
      cmdDirSign = 0;
    }
    pidOutDegS = 0.0f;
    return;
  }

  if (cmdDirSign != 0 && dir != cmdDirSign) {
    if (stepper->isRunning()) {
      stepper->stopMove();
      pidStopping = true;
      pidOutDegS = 0.0f;
      return;
    }
    cmdDirSign = 0;
  }

  int32_t aSteps = (int32_t)llround(fabsf(amax) * stepsPerDeg());
  stepper->setAcceleration(aSteps < 1 ? 1 : aSteps);
  uint32_t hz = (uint32_t)llround(fabsf(out) * stepsPerDeg());
  stepper->setSpeedInHz(hz < 1 ? 1 : hz);
  if (dir > 0) { stepper->runForward(); cmdDirSign = 1; }
  else         { stepper->runBackward(); cmdDirSign = -1; }
}

// ============================================================================
// Open-loop move-done detection
// ============================================================================
static void moveDoneService() {
  if (!moveDonePending || pu(P_CLOSED_LOOP) || mode != MODE_POSITION) return;
  if (stepper && !stepper->isRunning()) {
    moveDonePending = false;
    cmdDirSign = 0;
    sendEvent(EVT_MOVE_DONE, 0, (float)encoderDeg());
  }
}

// ============================================================================
// Endstop (IO8 / GPIO8 — strapping pin; level only matters for *boot* mode)
//
// After SPI boot, digitalRead is a normal CMOS threshold (~0.25/0.75 VDD).
// Mid-rail voltages (e.g. ~2.3 V from a bad divider) are undefined and will
// not toggle reliably. Fix the external circuit for ~0 V / ~3.3 V rails.
// ============================================================================
static void endstopService() {
  // HIGH = true when pin voltage is above the chip's VIH (not a voltmeter)
  bool rawHigh = digitalRead(PIN_HOME) == HIGH;
  endstopRawHigh = rawHigh;
  bool active = pu(P_ENDSTOP_ACTIVE_HIGH) ? rawHigh : !rawHigh;
  uint32_t now = millis();
  if (active == endstopActive) { endstopChangeMs = now; return; }
  if ((uint32_t)(now - endstopChangeMs) < ENDSTOP_DEBOUNCE_MS) return;
  endstopChangeMs = now;
  endstopActive = active;

  // USB debug: raw vs logical (helps optical sensors / divider issues)
  Serial.printf("# HOME gpio8_raw=%s logical_active=%s enable=%u active_high=%u\n",
                rawHigh ? "HIGH" : "LOW",
                active ? "yes" : "no",
                (unsigned)pu(P_ENDSTOP_ENABLE),
                (unsigned)pu(P_ENDSTOP_ACTIVE_HIGH));

  if (!pu(P_ENDSTOP_ENABLE)) return;

  if (active) {
    sendEvent(EVT_ENDSTOP_HIT, 0, (float)encoderDeg());
    if (mode == MODE_HOMING && homingMethod == 1 && homingPhase == 1) {
      return;   // homingService() consumes the trigger
    }
    uint32_t action = pu(P_ENDSTOP_ACTION);
    if (action >= 1) {
      stopImmediate();
      moveDonePending = false;
      mode = MODE_IDLE;
      if (action == 2) {
        encoderZeroHere();
        if (stepper) stepper->setCurrentPosition(0);
        targetDeg = 0.0;
      }
    }
  } else {
    sendEvent(EVT_ENDSTOP_RELEASED, 0, (float)encoderDeg());
  }
}

// ============================================================================
// Homing state machine
// ============================================================================
static void homingFinish(bool ok) {
  if (savedRunCurrent) {
    tmc.setRunCurrent(savedRunCurrent);
    savedRunCurrent = 0;
  }
  homingPhase = 0;
  mode = MODE_IDLE;
  if (ok) {
    homed = true;
    sendEvent(EVT_HOMING_DONE, homingMethod, (float)encoderDeg());
  } else {
    fault = FAULT_HOMING_TIMEOUT;
    stopImmediate();
    sendEvent(EVT_HOMING_FAILED, homingMethod, (float)encoderDeg());
  }
}

static void homingBegin(uint8_t method, int8_t dir, float speedDegS) {
  if (!driverEnabled || estopLatched || !stepper) return;
  homingMethod = method;
  homingDir = (dir >= 0) ? 1 : -1;
  homingSpeed = fabsf(speedDegS);
  fault = FAULT_NONE;
  stallFlag = false;

  if (method == 0) {   // software home: define zero right here
    stopImmediate();
    encoderZeroHere();
    stepper->setCurrentPosition(0);
    targetDeg = 0.0;
    mode = MODE_IDLE;
    homed = true;
    sendEvent(EVT_HOMING_DONE, 0, 0.0f);
    return;
  }
  if (homingSpeed < 0.001f) homingSpeed = 5.0f;

  uint32_t hc = pu(P_HOMING_CURRENT);
  if (hc > 0) {
    savedRunCurrent = pu(P_RUN_CURRENT);
    tmc.setRunCurrent(hc);
  }

  mode = MODE_HOMING;
  homingPhase = 1;
  homingStartMs = millis();
  pidState = 0;   // homing runs open-loop velocity
  velTargetDegS = homingSpeed * homingDir;
  uint32_t hz = (uint32_t)llround(homingSpeed * stepsPerDeg());
  int32_t acc = (int32_t)llround(fabsf(pf(P_ACCELERATION)) * stepsPerDeg());
  stepper->setAcceleration(acc < 1 ? 1 : acc);
  stepper->setSpeedInHz(hz < 1 ? 1 : hz);
  if (homingDir > 0) { stepper->runForward(); cmdDirSign = 1; }
  else               { stepper->runBackward(); cmdDirSign = -1; }
}

static void homingService() {
  if (mode != MODE_HOMING || homingPhase == 0 || !stepper) return;
  uint32_t now = millis();

  if ((uint32_t)(now - homingStartMs) > pu(P_HOMING_TIMEOUT_MS)) {
    homingFinish(false);
    return;
  }

  if (homingPhase == 1) {
    bool triggered = false;
    if (homingMethod == 1 && endstopActive) triggered = true;
    if (homingMethod == 2 && stallFlag) { stallFlag = false; triggered = true; }
    if (!triggered) {
      // Keep the seek run asserted (first run command after a forceStop()
      // can be swallowed — see applyContinuousRun()).
      if (!stepper->isRunning()) applyContinuousRun(homingSpeed * homingDir);
      return;
    }

    stepper->forceStop();
    cmdDirSign = 0;
    encoderZeroHere();
    stepper->setCurrentPosition(0);
    targetDeg = 0.0;

    float backoff = pf(P_HOMING_BACKOFF);
    if (backoff > 0.001f) {
      homingPhase = 2;
      int64_t steps = (int64_t)llround((double)backoff * -homingDir * stepsPerDeg());
      if (steps > INT32_MAX) steps = INT32_MAX;
      if (steps < INT32_MIN) steps = INT32_MIN;
      cmdDirSign = steps >= 0 ? 1 : -1;
      stepper->moveTo((int32_t)steps);
    } else {
      homingFinish(true);
    }
    return;
  }

  if (homingPhase == 2 && !stepper->isRunning()) {
    cmdDirSign = 0;
    homingFinish(true);
  }
}

// ============================================================================
// Leader/follower
// ============================================================================
static void followReset(bool stopMotor) {
  followSynced = false;
  followTargetValid = false;
  if (stopMotor) stopImmediate();
}

static void followHandleLeaderAngle(double leaderDeg) {
  if (!followEnabled || !stepper || !driverEnabled || estopLatched) return;
  if (!isfinite(leaderDeg)) return;
  mode = MODE_FOLLOW;

  if (!followSynced) {
    if (stepper->isRunning()) stepper->forceStop();
    followLeaderRefDeg = leaderDeg;
    followAcceptedDeg = leaderDeg;
    followLocalRefSteps = stepper->getCurrentPosition();
    followLastTarget = followLocalRefSteps;
    followTargetValid = true;
    encoderUpdate();
    followLocalRefDeg = encoderDeg();
    if (followEncCorrected) {
      targetDeg = followLocalRefDeg;
      pidPrevDeg = followLocalRefDeg;
      pidReset();
      pidState = 1;
      applySpeedAccel();
    }
    followSynced = true;
    return;
  }

  // Schmitt-style deadband: slow intentional motion accumulates, stationary
  // encoder noise from the leader cannot toggle the target.
  double delta = leaderDeg - followAcceptedDeg;
  if (fabs(delta) > FOLLOW_DEADBAND_DEG)
    followAcceptedDeg = leaderDeg - copysign(FOLLOW_DEADBAND_DEG, delta);

  double mapped = (followAcceptedDeg - followLeaderRefDeg) * followRatio *
                  (followInvert ? -1.0 : 1.0);

  if (followEncCorrected) {
    double newTarget = followLocalRefDeg + mapped;
    if (fabs(newTarget - targetDeg) > 0.001) {
      targetDeg = newTarget;
      pidBestErr = 1e30f;
      pidProgressMs = millis();
      pidSettleStartMs = 0;
      // Wake a settled controller only when the leader target really moved.
      // Reasserting state=running for every periodic leader frame prevents the
      // follower from ever holding PID_SETTLE_MS and makes wait_settled time out.
      if (pidState != 3) pidState = 1;
    }
    return;
  }

  int64_t steps = (int64_t)followLocalRefSteps +
                  (int64_t)llround(mapped * stepsPerDeg());
  if (steps > INT32_MAX) steps = INT32_MAX;
  if (steps < INT32_MIN) steps = INT32_MIN;
  int32_t tgt = (int32_t)steps;
  if (followTargetValid && tgt == followLastTarget) {
    if (!stepper->isRunning()) cmdDirSign = 0;
    return;
  }
  followLastTarget = tgt;
  followTargetValid = true;
  int32_t cur = stepper->getCurrentPosition();
  if (tgt == cur) {
    if (stepper->isRunning()) stepper->stopMove();
    cmdDirSign = 0;
    return;
  }
  applySpeedAccel();
  cmdDirSign = tgt > cur ? 1 : -1;
  stepper->moveTo(tgt);
}

// ============================================================================
// Parameter set/apply
// ============================================================================
static bool isPowerOfTwo(uint32_t v) { return v && (v & (v - 1)) == 0; }

static uint8_t paramApply(uint8_t id, ParamValue v) {
  const ParamDef *d = paramDef(id);
  if (!d) return PS_UNKNOWN;

  float asF = d->isFloat ? v.f : (float)v.u;
  if (d->isFloat && !isfinite(v.f)) return PS_REJECTED;
  if (asF < d->minv || asF > d->maxv) return PS_REJECTED;
  if (id == P_MICROSTEPS && !isPowerOfTwo(v.u)) return PS_REJECTED;

  params[id] = v;

  switch (id) {
    case P_NODE_ID:
      nodeId = (uint8_t)v.u;
      if (followEnabled && followLeader == nodeId) { followEnabled = false; followReset(true); }
      break;
    case P_MICROSTEPS:      tmc.setMicrostepsPerStep(v.u); applySpeedAccel(); break;
    case P_STEPS_PER_REV:   applySpeedAccel(); break;
    case P_RUN_CURRENT:     tmc.setRunCurrent(v.u); break;
    case P_HOLD_CURRENT:    tmc.setHoldCurrent(v.u); break;
    case P_STALL_THRESHOLD: tmc.setStallGuardThreshold(v.u); break;
    case P_INVERT_DIR:
      if (stepper) {
        stopImmediate();
        stepper->setDirectionPin(PIN_DIR, v.u == 0);
        // Re-anchor the step counter to the encoder so logical position holds.
        encoderUpdate();
        int64_t s = (int64_t)llround(encoderDeg() * stepsPerDeg());
        if (s > INT32_MAX) s = INT32_MAX;
        if (s < INT32_MIN) s = INT32_MIN;
        stepper->setCurrentPosition((int32_t)s);
      }
      break;
    case P_CLOSED_LOOP:
      if (!v.u) { pidState = 0; pidReset(); }
      applySpeedAccel();
      break;
    case P_MAX_SPEED: case P_ACCELERATION:
    case P_CL_MAX_SPEED: case P_CL_MAX_ACCEL:
      applySpeedAccel();
      break;
    case P_STANDSTILL_MODE:
      switch (v.u) {
        case 1: tmc.setStandstillMode(tmc.FREEWHEELING); break;
        case 2: tmc.setStandstillMode(tmc.BRAKING); break;
        case 3: tmc.setStandstillMode(tmc.STRONG_BRAKING); break;
        default: tmc.setStandstillMode(tmc.NORMAL); break;
      }
      break;
    case P_STEALTHCHOP:
      if (v.u) tmc.enableStealthChop(); else tmc.disableStealthChop();
      break;
    default: break;
  }
  return PS_OK;
}

static void paramsLoadDefaults() {
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    const ParamDef &d = PARAM_DEFS[i];
    ParamValue v;
    if (d.isFloat) v.f = d.def; else v.u = (uint32_t)d.def;
    params[d.id] = v;
  }
  nodeId = (uint8_t)pu(P_NODE_ID);
}

static void paramsLoadNvs() {
  prefs.begin("gcsp", true);
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    const ParamDef &d = PARAM_DEFS[i];
    char key[8];
    snprintf(key, sizeof(key), "p%u", d.id);
    uint32_t raw = prefs.getUInt(key, params[d.id].u);
    ParamValue v; v.u = raw;
    float asF = d.isFloat ? v.f : (float)v.u;
    bool ok = (!d.isFloat || isfinite(v.f)) && asF >= d.minv && asF <= d.maxv &&
              (d.id != P_MICROSTEPS || isPowerOfTwo(v.u));
    if (ok) params[d.id] = v;
  }
  prefs.end();
  nodeId = (uint8_t)pu(P_NODE_ID);
}

static void paramsSaveNvs() {
  prefs.begin("gcsp", false);
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    const ParamDef &d = PARAM_DEFS[i];
    char key[8];
    snprintf(key, sizeof(key), "p%u", d.id);
    prefs.putUInt(key, params[d.id].u);
  }
  prefs.end();
}

// ============================================================================
// Command dispatch
// ============================================================================
static void handleRtr(uint8_t msgId) {
  switch (msgId) {
    case TEL_STATUS:        sendStatus(); break;
    case TEL_POSITION:      encoderUpdate(); sendPosition(); break;
    case TEL_MOTION:        sendMotion(); break;
    case TEL_TARGET:        sendTarget(); break;
    case TEL_DRIVER:        sendDriver(); break;
    case TEL_ENV:           sendEnv(); break;
    case TEL_FOLLOW_STATUS: sendFollowStatus(); break;
    case TEL_CAN_HEALTH:    sendCanHealth(); break;
    case TEL_ENC_COUNTS:    encoderUpdate(); sendEncCounts(); break;
    case TEL_PID_STATUS:    sendPidStatus(); break;
    default: break;
  }
}

static void handleCommand(uint8_t msgId, const uint8_t *d, uint8_t len) {
  switch (msgId) {
    case CMD_PING: sendStatus(); break;
    case CMD_ESTOP: emergencyStop(); break;
    case CMD_STOP:
      homingPhase = 0;
      moveDonePending = false;
      stopRamped();
      break;
    case CMD_ENABLE:
      if (len >= 1) setDriverEnabled(d[0] != 0);
      break;
    case CMD_MOVE_ABS:
      if (len >= 8) {
        double deg; memcpy(&deg, d, 8);
        if (!isfinite(deg)) break;
        if (followEnabled) { followEnabled = false; followReset(true); }
        homingPhase = 0;
        mode = MODE_POSITION;
        beginPositionMove(deg);
      }
      break;
    case CMD_MOVE_REL:
      if (len >= 8) {
        double delta; memcpy(&delta, d, 8);
        if (!isfinite(delta)) break;
        if (followEnabled) { followEnabled = false; followReset(true); }
        homingPhase = 0;
        double base = (mode == MODE_POSITION) ? targetDeg : encoderDeg();
        mode = MODE_POSITION;
        beginPositionMove(base + delta);
      }
      break;
    case CMD_MOVE_VEL:
      if (len >= 4) {
        float v; memcpy(&v, d, 4);
        if (!isfinite(v)) break;
        if (followEnabled) { followEnabled = false; followReset(true); }
        homingPhase = 0;
        beginVelocity(v);
      }
      break;
    case CMD_SET_ZERO:
      stopImmediate();
      encoderZeroHere();
      if (stepper) stepper->setCurrentPosition(0);
      targetDeg = 0.0;
      mode = MODE_IDLE;
      break;
    case CMD_HOME:
      if (len >= 6) {
        uint8_t method = d[0];
        int8_t dir = (int8_t)d[1];
        float speed; memcpy(&speed, d + 2, 4);
        if (method <= 2 && isfinite(speed)) {
          if (followEnabled) { followEnabled = false; followReset(true); }
          homingBegin(method, dir, speed);
        }
      }
      break;
    case CMD_SET_PARAM:
      if (len >= 5) {
        ParamValue v; memcpy(&v, d + 1, 4);
        uint8_t st = paramApply(d[0], v);
        sendParam(d[0], st);
      }
      break;
    case CMD_GET_PARAM:
      if (len >= 1) sendParam(d[0], paramDef(d[0]) ? PS_OK : PS_UNKNOWN);
      break;
    case CMD_SAVE_CONFIG: paramsSaveNvs(); break;
    case CMD_LOAD_DEFAULTS: {
      stopImmediate();
      uint8_t keepId = nodeId;   // identity survives a factory reset
      paramsLoadDefaults();
      ParamValue v; v.u = keepId;
      params[P_NODE_ID] = v; nodeId = keepId;
      for (size_t i = 0; i < PARAM_COUNT; i++)
        paramApply(PARAM_DEFS[i].id, params[PARAM_DEFS[i].id]);
      paramsSaveNvs();
      break;
    }
    case CMD_FOLLOW:
      if (len >= 8) {
        bool en = d[0] != 0;
        uint8_t leader = d[1];
        float ratio; memcpy(&ratio, d + 4, 4);
        if (leader >= 1 && leader <= 31 && (!en || leader != nodeId) &&
            isfinite(ratio) && ratio > 0.0f) {
          followEnabled = en;
          followLeader = leader;
          followInvert = (d[2] & 1) != 0;
          followEncCorrected = (d[2] & 2) != 0;
          followRatio = ratio;
          followReset(true);
          if (!en) mode = MODE_IDLE;
        }
      }
      break;
    case CMD_FOLLOW_SYNC:
      if (followEnabled) followReset(true);
      break;
    default: break;
  }
}

static void dispatchFrame(uint32_t id, bool rtr, const uint8_t *data, uint8_t len) {
  uint8_t target = (id >> 6) & 0x1F;
  uint8_t msgId  = id & 0x3F;

  // Follower input: the leader's broadcast POSITION telemetry.
  if (!rtr && msgId == TEL_POSITION && followEnabled &&
      target == followLeader && len >= 8) {
    double deg; memcpy(&deg, data, 8);
    followHandleLeaderAngle(deg);
  }

  if (target != nodeId && target != 0) return;
  if (rtr) handleRtr(msgId);
  else if (msgId < 32) handleCommand(msgId, data, len);
}

// ============================================================================
// CAN health / bus-off recovery
// ============================================================================
static void canHealthService() {
  uint32_t now = millis();
  if ((uint32_t)(now - canHealthMs) < 100) return;
  canHealthMs = now;

  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return;

  if (s.state == TWAI_STATE_BUS_OFF) {
    if (twai_initiate_recovery() == ESP_OK) {
      canRestartPending = true;
      if (canRecoveries < 255) canRecoveries++;
      Serial.printf("# CAN bus-off, recovery started (count=%u)\n", canRecoveries);
    }
    return;
  }
  // Recovery parks the controller in STOPPED; it must be restarted explicitly.
  if (s.state == TWAI_STATE_STOPPED && canRestartPending) {
    if (twai_start() == ESP_OK) {
      canRestartPending = false;
      Serial.printf("# CAN recovered and restarted (count=%u)\n", canRecoveries);
    }
  }
}

// ============================================================================
// Serial bridge input:  "<ID hex> <rtr 0|1> <hex payload>"  per line.
// Lines beginning with '#' are ignored (debug output uses that prefix).
// ============================================================================
static void serialLineExecute(char *line) {
  if (line[0] == '#' || line[0] == 0) return;
  char *save = nullptr;
  char *tokId  = strtok_r(line, " \t", &save);
  char *tokRtr = strtok_r(nullptr, " \t", &save);
  char *tokHex = strtok_r(nullptr, " \t", &save);
  if (!tokId || !tokRtr) return;

  uint32_t id = strtoul(tokId, nullptr, 16);
  bool rtr = (tokRtr[0] == '1');
  uint8_t data[8] = {0};
  uint8_t len = 0;
  if (tokHex) {
    size_t hn = strlen(tokHex);
    len = (uint8_t)(hn / 2);
    if (len > 8) len = 8;
    for (uint8_t i = 0; i < len; i++) {
      char b[3] = { tokHex[2 * i], tokHex[2 * i + 1], 0 };
      data[i] = (uint8_t)strtoul(b, nullptr, 16);
    }
  }

  // Execute locally (if addressed to us / broadcast) and put it on the bus.
  dispatchFrame(id, rtr, data, len);
  twai_message_t m;
  memset(&m, 0, sizeof(m));
  m.identifier = id & 0x7FF;
  m.rtr = rtr ? 1 : 0;
  m.data_length_code = rtr ? 8 : len;
  if (!rtr) memcpy(m.data, data, len);
  twai_transmit(&m, 0);
}

static void serialService() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        serialLineExecute(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;   // overlong line: discard
    }
  }
}

// ============================================================================
// Telemetry scheduler
// ============================================================================
static void telemetryService() {
  uint32_t now = millis();
  uint32_t fastHz = pu(P_FAST_RATE_HZ);
  uint32_t slowHz = pu(P_SLOW_RATE_HZ);

  if (fastHz > 0 && (uint32_t)(now - fastTelMs) >= 1000 / fastHz) {
    fastTelMs = now;
    encoderUpdate();
    sendPosition();
    sendMotion();
    if (pu(P_CLOSED_LOOP)) sendPidStatus();
  }
  if (slowHz > 0 && (uint32_t)(now - slowTelMs) >= 1000 / slowHz) {
    slowTelMs = now;
    sendStatus();
    sendDriver();
    sendEnv();
    if (followEnabled) sendFollowStatus();
    sendCanHealth();
  }
}

// ============================================================================
// Setup / loop
// ============================================================================
void setup() {
  delay(400);
  Serial.begin(115200);
  delay(200);
  Serial.printf("# GrafitoCANStepper fw %u.%u proto %u\n", FW_MAJOR, FW_MINOR, PROTO_VER);

  pinMode(PIN_TMC_EN, OUTPUT);
  digitalWrite(PIN_TMC_EN, HIGH);      // start with the driver off
  pinMode(PIN_DIAG, INPUT);
  // GPIO8 is a strapping pin (must be HIGH at reset for SPI boot). After boot
  // it is a normal input. INPUT_PULLUP keeps the line high if the endstop is
  // open or unplugged; an external 10k to 3.3V is fine in parallel.
  // Active-low: open ≈ HIGH (inactive), closed to GND ≈ LOW (active).
  pinMode(PIN_HOME, INPUT_PULLUP);
  endstopRawHigh = (digitalRead(PIN_HOME) == HIGH);
  Serial.printf("# HOME pin GPIO8 INPUT_PULLUP raw=%s (strapping: keep HIGH at power-on)\n",
                endstopRawHigh ? "HIGH" : "LOW");
  pinMode(PIN_ENC_CS, OUTPUT);
  digitalWrite(PIN_ENC_CS, HIGH);

  attachInterrupt(digitalPinToInterrupt(PIN_DIAG), onDiagRise, RISING);

  encSpi.begin(PIN_ENC_CLK, PIN_ENC_DO, -1, -1);

  paramsLoadDefaults();
  paramsLoadNvs();
  // Hard defaults for this product wiring (override stale NVS from older fw).
  params[P_ENDSTOP_ENABLE].u = 1;
  params[P_ENDSTOP_ACTIVE_HIGH].u = 1;
  params[P_ENDSTOP_ACTION].u = 1;   // stop on hit
  // Multi-node mesh power policy (override bench NVS that left motors hot):
  //  - never auto-enable coils at boot (host must enable() for motion)
  //  - keep hold current low if NVS had high test values
  //  - freewheel at standstill so enabled-but-idle uses less current
  params[P_ENABLE_ON_BOOT].u = 0;
  if (pu(P_HOLD_CURRENT) > 5) params[P_HOLD_CURRENT].u = 5;
  if (pu(P_RUN_CURRENT) > 40) params[P_RUN_CURRENT].u = 40;  // cap absurd bench peaks
  params[P_STANDSTILL_MODE].u = 1;  // FREEWHEELING
  Serial.println("# HOME: INPUT_PULLUP + endstop enable=1 active_high=1 action=stop (fw 1.9)");
  Serial.printf("# POWER: enable_on_boot=0 hold<=%u run<=%u standstill=freewheel (fw 1.9)\n",
                (unsigned)pu(P_HOLD_CURRENT), (unsigned)pu(P_RUN_CURRENT));

  // Encoder self-check
  uint32_t f = encoderReadFrame();
  uint16_t a = (f >> 10) & 0x3FFF;
  Serial.println("# MT6701 SSI mode=3 encoder-loss-filter=25ms");
  Serial.printf("# MT6701 raw=0x%06lX angle=%.2f deg\n",
                (unsigned long)f, a * 360.0 / ENC_CPR);
  if (f == 0x000000 || (f & 0xFFFFC0) == 0xFFFFC0)
    Serial.println("# WARNING: encoder frame stuck (check CLK/DO/CS wiring)");

  // Step generator
  stepperEngine.init();
  stepper = stepperEngine.stepperConnectToPin(PIN_STEP);
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR, pu(P_INVERT_DIR) == 0);
    stepper->setAutoEnable(false);
    stepper->setCurrentPosition(0);
  } else {
    Serial.println("# ERROR: no step generator channel available");
  }
  applySpeedAccel();

  // TMC2209 over UART (Serial1)
  tmc.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0, PIN_TMC_RX, PIN_TMC_TX);
  tmc.setRunCurrent(pu(P_RUN_CURRENT));
  tmc.setHoldCurrent(pu(P_HOLD_CURRENT));
  tmc.setMicrostepsPerStep(pu(P_MICROSTEPS));
  tmc.setStallGuardThreshold(pu(P_STALL_THRESHOLD));
  tmc.enableAutomaticCurrentScaling();
  tmc.setCoolStepDurationThreshold(5000);
  if (pu(P_STEALTHCHOP)) tmc.enableStealthChop(); else tmc.disableStealthChop();
  switch (pu(P_STANDSTILL_MODE)) {
    case 1: tmc.setStandstillMode(tmc.FREEWHEELING); break;
    case 2: tmc.setStandstillMode(tmc.BRAKING); break;
    case 3: tmc.setStandstillMode(tmc.STRONG_BRAKING); break;
    default: tmc.setStandstillMode(tmc.NORMAL); break;
  }

  if (pu(P_ZERO_ON_BOOT)) encoderZeroHere();
  else encoderUpdate();
  pidPrevDeg = encoderDeg();

  setDriverEnabled(pu(P_ENABLE_ON_BOOT) != 0);

  // CAN last, after every other peripheral has claimed its pins.
  if (canStart()) Serial.println("# CAN started (1 Mbps)");
  else            Serial.println("# ERROR: CAN start failed");

  sendEvent(EVT_BOOT, nodeId, (float)FW_MAJOR + FW_MINOR / 100.0f);
}

void loop() {
  canHealthService();
  tmcDriverService();

  // Drain the CAN RX queue.
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    if (rx.extd) continue;
    if (!rx.rtr) serialPrintFrame(rx.identifier, false, rx.data, rx.data_length_code);
    dispatchFrame(rx.identifier, rx.rtr, rx.data, rx.data_length_code);
  }

  serialService();
  endstopService();
  homingService();
  velocityService();

  if (stallFlag && mode != MODE_HOMING) {
    stallFlag = false;
    sendEvent(EVT_STALL, 0, (float)encoderDeg());
  }

  pidService();
  moveDoneService();
  telemetryService();

  delay(1);
}
