/*
 * GrafitoCANStepper_C3_CANopen — CiA 301 / CiA 402 firmware (fw 2.0)
 *
 * Flash this sketch instead of GrafitoCANStepper_C3.ino when the node must
 * speak CANopen to a PLC. The original GCSP firmware is unchanged.
 *
 * Identity / EDS: GrafitoCANStepper.eds  (same folder)
 * Host reference + tests: canstepper/canopen402.py
 *
 * Physical layer: CAN 2.0A, 11-bit, default 1 Mbps (0x200E).
 * Units: encoder counts, 16384 / revolution.
 * Modes: pp=1, pv=3, hm=6.
 *
 * Build (Arduino IDE, ESP32C3 Dev Module, USB CDC On Boot: Enabled):
 *   Libraries: FastAccelStepper (gin66), TMC2209 (janelia-arduino)
 */

#include <FastAccelStepper.h>
#include <TMC2209.h>
#include <SPI.h>
#include <Preferences.h>
#include "driver/twai.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "canopen_stack.h"

static const uint8_t FW_MAJOR = 2;
static const uint8_t FW_MINOR = 0;

#ifndef CO_FACTORY_NODE_ID
#define CO_FACTORY_NODE_ID 1
#endif

static const int PIN_CAN_TX  = 21;
static const int PIN_CAN_RX  = 20;
static const int PIN_TMC_EN  = 3;
static const int PIN_STEP    = 10;
static const int PIN_DIR     = 1;
static const int PIN_DIAG    = 0;
static const int PIN_TMC_TX  = 7;
static const int PIN_TMC_RX  = 6;
static const int PIN_ENC_CLK = 4;
static const int PIN_ENC_DO  = 5;
static const int PIN_ENC_CS  = 2;
static const int PIN_HOME    = 8;

static const int32_t ENC_CPR = 16384;
static const uint32_t PID_PERIOD_US = 5000;
static const uint32_t PID_SETTLE_MS = 80;
static const uint32_t ENC_LOSS_TIMEOUT_US = 25000;

enum Mode : uint8_t { MODE_IDLE = 0, MODE_POSITION = 1, MODE_VELOCITY = 2, MODE_HOMING = 3 };

Preferences prefs;
TMC2209 tmc;
FastAccelStepperEngine stepperEngine;
FastAccelStepper *stepper = nullptr;
static SPIClass encSpi(FSPI);

static uint8_t  mode = MODE_IDLE;
static bool     driverEnabled = false;
static bool     homed = false;
static int64_t  encCounts = 0;
static int64_t  encZeroOffset = 0;
static bool     encFrameOk = true;
static bool     encHasGoodFrame = false;
static uint32_t encLastGoodUs = 0;
static double   targetDeg = 0.0;
static float    velTargetDegS = 0.0f;
static int      cmdDirSign = 0;
static bool     moveDonePending = false;

static uint8_t  pidState = 0;
static float    pidOutDegS = 0.0f;
static float    pidIntegral = 0.0f;
static float    pidVelFilt = 0.0f;
static double   pidPrevDeg = 0.0;
static uint32_t pidPrevUs = 0;
static uint32_t pidSettleStartMs = 0;
static float    pidBestErr = 1e30f;
static uint32_t pidProgressMs = 0;
static bool     pidStopping = false;

static bool   trajActive = false;
static bool   trajDone = true;
static double trajS0 = 0.0, trajS1 = 0.0, trajRef = 0.0;
static float  trajSign = 1.0f, trajDist = 0.0f, trajVpeak = 0.0f, trajA = 1.0f;
static float  trajTacc = 0.0f, trajTcruise = 0.0f, trajTdec = 0.0f, trajTtot = 0.0f;
static float  trajT = 0.0f, trajVff = 0.0f;

static bool     endstopActive = false;
static bool     endstopRawHigh = true;
static uint32_t endstopChangeMs = 0;
static uint8_t  homingMethod = 0;
static int8_t   homingDir = 1;
static float    homingSpeed = 0.0f;
static uint8_t  homingPhase = 0;
static uint32_t homingStartMs = 0;
static volatile bool stallFlag = false;
static uint8_t  canRecoveries = 0;
static bool     canRestartPending = false;
static uint32_t canHealthMs = 0;
static char     lineBuf[80];
static size_t   lineLen = 0;

static uint16_t tmcStallGuard = 0;
static uint8_t  tmcFlagsA = 0, tmcFlagsB = 0, tmcGstat = 0, tmcCsActual = 0;
static uint16_t tmcInterstep = 0;
static uint32_t tmcPollMs = 0;

static inline double stepsPerDeg() {
  return (double)coOd()->steps_per_rev * (double)coOd()->microsteps / 360.0;
}
static inline double countsToDeg(int64_t c) { return (double)c * 360.0 / ENC_CPR; }
static inline int32_t degToCounts(double d) { return (int32_t)lround(d * ENC_CPR / 360.0); }
static inline float countsSToDegS(int32_t v) { return (float)v * 360.0f / (float)ENC_CPR; }
static inline double encoderDeg() { return countsToDeg(encCounts); }
static void IRAM_ATTR onDiagRise() { stallFlag = true; }

static uint8_t crc6_itu(uint32_t data18) {
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
  static int32_t turns = 0;
  static bool first = true;
  uint32_t frame = encoderReadFrame();
  uint16_t raw = (frame >> 10) & 0x3FFF;
  uint8_t rxCrc = frame & 0x3F;
  bool stuck = (frame == 0) || ((frame & 0xFFFFC0) == 0xFFFFC0);
  encFrameOk = (crc6_itu((frame >> 6) & 0x3FFFF) == rxCrc) && !stuck;
  if (!encFrameOk) return false;
  encHasGoodFrame = true;
  encLastGoodUs = micros();
  if (first) { prevRaw = raw; first = false; }
  int32_t q = ENC_CPR / 4;
  if ((int32_t)prevRaw > 3 * q && (int32_t)raw < q) turns++;
  else if ((int32_t)prevRaw < q && (int32_t)raw > 3 * q) turns--;
  prevRaw = raw;
  encCounts = (int64_t)raw + (int64_t)turns * ENC_CPR - encZeroOffset;
  return true;
}

static void encoderZeroHere() {
  encoderUpdate();
  encZeroOffset += encCounts;
  encCounts = 0;
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

static void canTx(uint32_t id, const uint8_t *data, uint8_t len) {
  twai_message_t m;
  memset(&m, 0, sizeof(m));
  m.identifier = id;
  m.data_length_code = len > 8 ? 8 : len;
  if (data && len) memcpy(m.data, data, m.data_length_code);
  twai_transmit(&m, 0);
  serialPrintFrame(m.identifier, false, m.data, m.data_length_code);
}

static twai_timing_config_t bitrateConfig(uint32_t br) {
  if (br == 125000) return TWAI_TIMING_CONFIG_125KBITS();
  if (br == 250000) return TWAI_TIMING_CONFIG_250KBITS();
  if (br == 500000) return TWAI_TIMING_CONFIG_500KBITS();
  return TWAI_TIMING_CONFIG_1MBITS();
}

static bool canStart(uint32_t bitrate) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16;
  g.rx_queue_len = 32;
  twai_timing_config_t t = bitrateConfig(bitrate);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

static void trajClear() {
  trajActive = false;
  trajDone = true;
  trajT = 0;
  trajVff = 0;
}

static void stopImmediate() {
  if (stepper) stepper->forceStop();
  cmdDirSign = 0;
  velTargetDegS = 0;
  pidState = 0;
  pidIntegral = 0;
  pidOutDegS = 0;
  pidStopping = false;
  trajClear();
}

static void applySpeedAccel() {
  if (!stepper) return;
  float spd = countsSToDegS((int32_t)coOd()->profile_vel);
  float acc = countsSToDegS((int32_t)coOd()->profile_acc);
  if (spd < 0.001f) spd = 0.001f;
  if (acc < 1.0f) acc = 1.0f;
  uint32_t hz = (uint32_t)llround(fabsf(spd) * stepsPerDeg());
  int32_t a = (int32_t)llround(fabsf(acc) * stepsPerDeg());
  stepper->setSpeedInHz(hz < 1 ? 1 : hz);
  stepper->setAcceleration(a < 1 ? 1 : a);
}

static void applyTmcFromOd() {
  tmc.setRunCurrent(coOd()->run_current);
  tmc.setHoldCurrent(coOd()->hold_current);
  tmc.setMicrostepsPerStep(coOd()->microsteps);
  tmc.setStallGuardThreshold(coOd()->stall_threshold);
  if (coOd()->stealthchop) tmc.enableStealthChop(); else tmc.disableStealthChop();
  switch (coOd()->standstill) {
    case 1: tmc.setStandstillMode(tmc.FREEWHEELING); break;
    case 2: tmc.setStandstillMode(tmc.BRAKING); break;
    case 3: tmc.setStandstillMode(tmc.STRONG_BRAKING); break;
    default: tmc.setStandstillMode(tmc.NORMAL); break;
  }
  if (stepper) stepper->setDirectionPin(PIN_DIR, coOd()->invert_dir == 0);
  applySpeedAccel();
}

static void setDriverEnabled(bool en) {
  driverEnabled = en;
  if (en) {
    tmc.enable();
    digitalWrite(PIN_TMC_EN, LOW);
    tmc.clearDriveError();
    tmc.clearReset();
  } else {
    stopImmediate();
    tmc.disable();
    digitalWrite(PIN_TMC_EN, HIGH);
    mode = MODE_IDLE;
  }
}

static void trajPlan(double s0, double s1, float vmax, float amax) {
  trajS0 = s0; trajS1 = s1;
  float d = (float)(s1 - s0);
  trajDist = fabsf(d);
  trajSign = (d >= 0) ? 1.0f : -1.0f;
  trajA = fmaxf(amax, 1.0f);
  vmax = fmaxf(vmax, 0.001f);
  trajT = 0; trajRef = s0; trajVff = 0; trajActive = true;
  if (trajDist < 1e-4f) {
    trajDone = true; trajRef = s1; trajTtot = 0; return;
  }
  float dAccFull = (vmax * vmax) / (2.0f * trajA);
  if (2.0f * dAccFull >= trajDist) {
    trajVpeak = sqrtf(trajA * trajDist);
    trajTacc = trajTdec = trajVpeak / trajA;
    trajTcruise = 0;
  } else {
    trajVpeak = vmax;
    trajTacc = trajTdec = vmax / trajA;
    trajTcruise = (trajDist - 2.0f * dAccFull) / vmax;
  }
  trajTtot = trajTacc + trajTcruise + trajTdec;
  trajDone = false;
}

static void trajAdvance(float dt) {
  if (!trajActive) return;
  if (trajDone) { trajRef = trajS1; trajVff = 0; return; }
  trajT += dt;
  if (trajT >= trajTtot - 1e-7f) {
    trajDone = true; trajRef = trajS1; trajVff = 0; return;
  }
  float t = trajT, sLocal, vLocal;
  float dAcc = 0.5f * trajA * trajTacc * trajTacc;
  if (t <= trajTacc) {
    vLocal = trajA * t; sLocal = 0.5f * trajA * t * t;
  } else if (t <= trajTacc + trajTcruise) {
    vLocal = trajVpeak; sLocal = dAcc + trajVpeak * (t - trajTacc);
  } else {
    float td = t - trajTacc - trajTcruise;
    vLocal = fmaxf(0.0f, trajVpeak - trajA * td);
    sLocal = dAcc + trajVpeak * trajTcruise + trajVpeak * td - 0.5f * trajA * td * td;
  }
  trajRef = trajS0 + (double)(trajSign * sLocal);
  trajVff = trajSign * vLocal;
}

static void beginPositionMove(double deg) {
  if (!stepper || !driverEnabled) return;
  targetDeg = deg;
  moveDonePending = true;
  applySpeedAccel();
  if (coOd()->closed_loop) {
    bool wasRunning = stepper->isRunning();
    if (wasRunning) stepper->forceStop();
    encoderUpdate();
    double here = encoderDeg();
    pidPrevDeg = here;
    pidIntegral = 0; pidStopping = wasRunning; trajClear();
    trajPlan(here, deg, countsSToDegS((int32_t)coOd()->profile_vel),
             countsSToDegS((int32_t)coOd()->profile_acc));
    pidState = 1;
    pidBestErr = 1e30f;
    pidProgressMs = millis();
    pidSettleStartMs = 0;
  } else {
    pidState = 0;
    trajClear();
    int64_t steps = (int64_t)llround(deg * stepsPerDeg());
    if (steps > INT32_MAX) steps = INT32_MAX;
    if (steps < INT32_MIN) steps = INT32_MIN;
    cmdDirSign = (steps >= stepper->getCurrentPosition()) ? 1 : -1;
    stepper->moveTo((int32_t)steps);
  }
  mode = MODE_POSITION;
}

static void applyContinuousRun(float degS) {
  uint32_t hz = (uint32_t)llround(fabs((double)degS) * stepsPerDeg());
  if (hz < 1) hz = 1;
  stepper->setSpeedInHz(hz);
  if (degS > 0) { stepper->runForward(); cmdDirSign = 1; }
  else { stepper->runBackward(); cmdDirSign = -1; }
}

static void beginVelocity(float degS) {
  if (!stepper || !driverEnabled) return;
  mode = MODE_VELOCITY;
  velTargetDegS = degS;
  pidState = 0;
  moveDonePending = false;
  int32_t acc = (int32_t)llround(fabsf(countsSToDegS((int32_t)coOd()->profile_acc)) * stepsPerDeg());
  stepper->setAcceleration(acc < 1 ? 1 : acc);
  if (fabsf(degS) < 1e-3f) {
    stepper->stopMove();
    cmdDirSign = 0;
    velTargetDegS = 0;
    mode = MODE_IDLE;
    return;
  }
  applyContinuousRun(degS);
}

static void velocityService() {
  if (mode != MODE_VELOCITY || !stepper || !driverEnabled) return;
  if (velTargetDegS == 0 || stepper->isRunning()) return;
  applyContinuousRun(velTargetDegS);
}

static void pidService() {
  uint32_t nowUs = micros();
  if (pidPrevUs != 0 && (uint32_t)(nowUs - pidPrevUs) < PID_PERIOD_US) return;
  float dt = (pidPrevUs == 0) ? (PID_PERIOD_US * 1e-6f)
                              : ((uint32_t)(nowUs - pidPrevUs)) * 1e-6f;
  if (dt <= 0 || dt > 0.1f) dt = PID_PERIOD_US * 1e-6f;
  pidPrevUs = nowUs;

  bool sampleOk = encoderUpdate();
  double measured = encoderDeg();
  pidVelFilt += 0.30f * ((float)((measured - pidPrevDeg) / dt) - pidVelFilt);
  pidPrevDeg = measured;
  coSetPositionActual(degToCounts(measured));
  coSetVelocityActual((int32_t)lround(pidVelFilt * ENC_CPR / 360.0f));
  coSetEncoderOk(encFrameOk);

  if (!(coOd()->closed_loop && pidState == 1 && driverEnabled && mode == MODE_POSITION) || !stepper)
    return;

  bool lost = !encHasGoodFrame ||
      (!sampleOk && (uint32_t)(micros() - encLastGoodUs) >= ENC_LOSS_TIMEOUT_US);
  if (lost) {
    stopImmediate();
    pidState = 3;
    coRaiseFault(ERR_ENCODER, 0x21);
    return;
  }

  float vmax = countsSToDegS((int32_t)coOd()->profile_vel);
  float amax = countsSToDegS((int32_t)coOd()->profile_acc);
  if (vmax < 0.001f) vmax = 0.001f;
  if (amax < 1) amax = 1;
  float tol = coOd()->pid_tol;
  if (trajActive) trajAdvance(dt);
  float trackErr = trajActive ? (float)(trajRef - measured) : (float)(targetDeg - measured);
  float vFf = trajActive && !trajDone ? trajVff : 0.0f;
  float targetErr = (float)(targetDeg - measured);
  float absTargetErr = fabsf(targetErr);
  float absVel = fabsf(pidVelFilt);
  uint32_t nowMs = millis();

  if ((!trajActive || trajDone) && absTargetErr <= tol && absVel <= fmaxf(12.0f, 0.06f * vmax)) {
    if (stepper->isRunning()) { stepper->stopMove(); pidStopping = true; }
    else pidStopping = false;
    cmdDirSign = 0;
    pidOutDegS = 0;
    if (pidSettleStartMs == 0) pidSettleStartMs = nowMs;
    if ((uint32_t)(nowMs - pidSettleStartMs) >= PID_SETTLE_MS) {
      pidState = 2;
      if (moveDonePending) {
        moveDonePending = false;
        coSetTargetReached(true);
      }
    }
    return;
  }
  pidSettleStartMs = 0;

  if (absTargetErr < pidBestErr - 0.011f) { pidBestErr = absTargetErr; pidProgressMs = nowMs; }
  else if (trajActive && !trajDone) pidProgressMs = nowMs;
  if (absTargetErr > fmaxf(1.0f, tol * 4) && (uint32_t)(nowMs - pidProgressMs) > 3000) {
    stopImmediate();
    pidState = 3;
    coRaiseFault(ERR_BLOCKED, 0x21);
    return;
  }

  float integ = pidIntegral + trackErr * dt;
  float ki = coOd()->pid_ki, kp = coOd()->pid_kp, kd = coOd()->pid_kd;
  if (ki > 1e-6f) integ = constrain(integ, -(0.20f * vmax) / ki, (0.20f * vmax) / ki);
  else integ = 0;
  float trim = constrain(kp * trackErr + ki * integ - kd * pidVelFilt,
                         -fmaxf(0.25f * vmax, 40.0f), fmaxf(0.25f * vmax, 40.0f));
  float out = constrain(vFf + trim, -vmax * 1.15f, vmax * 1.15f);
  pidIntegral = integ;
  float maxD = amax * dt;
  float delta = out - pidOutDegS;
  if (delta > maxD) out = pidOutDegS + maxD;
  if (delta < -maxD) out = pidOutDegS - maxD;
  pidOutDegS = out;

  if (pidStopping) {
    if (stepper->isRunning()) return;
    pidStopping = false; cmdDirSign = 0;
  }
  int dir = out > 0.05f ? 1 : (out < -0.05f ? -1 : 0);
  if (dir == 0) {
    if (stepper->isRunning()) { stepper->stopMove(); pidStopping = true; }
    return;
  }
  if (cmdDirSign && dir != cmdDirSign && stepper->isRunning()) {
    stepper->stopMove(); pidStopping = true; return;
  }
  int32_t aSteps = (int32_t)llround(fabsf(amax) * stepsPerDeg());
  stepper->setAcceleration(aSteps < 1 ? 1 : aSteps);
  uint32_t hz = (uint32_t)llround(fabsf(out) * stepsPerDeg());
  stepper->setSpeedInHz(hz < 1 ? 1 : hz);
  if (dir > 0) { stepper->runForward(); cmdDirSign = 1; }
  else { stepper->runBackward(); cmdDirSign = -1; }
}

static void moveDoneService() {
  if (!moveDonePending || coOd()->closed_loop || mode != MODE_POSITION) return;
  if (stepper && !stepper->isRunning()) {
    moveDonePending = false;
    cmdDirSign = 0;
    coSetTargetReached(true);
  }
}

static void endstopService() {
  bool rawHigh = digitalRead(PIN_HOME) == HIGH;
  endstopRawHigh = rawHigh;
  bool active = coOd()->endstop_active_high ? rawHigh : !rawHigh;
  uint32_t now = millis();
  if (active == endstopActive) { endstopChangeMs = now; return; }
  if ((uint32_t)(now - endstopChangeMs) < 5) return;
  endstopChangeMs = now;
  endstopActive = active;
  coSetEndstopActive(active);
  if (!coOd()->endstop_enable) return;
  if (active && !(mode == MODE_HOMING && homingPhase == 1)) {
    stopImmediate();
    moveDonePending = false;
    mode = MODE_IDLE;
  }
}

static void homingFinish(bool ok) {
  homingPhase = 0;
  mode = MODE_IDLE;
  if (ok) {
    homed = true;
    coSetHomingAttained(true);
  } else {
    coRaiseFault(ERR_HOMING, 0x21);
  }
}

static void homingBeginInternal(uint8_t method, int8_t dir, float speedDegS) {
  if (!driverEnabled || !stepper) return;
  homingMethod = method;
  homingDir = (dir >= 0) ? 1 : -1;
  homingSpeed = fabsf(speedDegS);
  stallFlag = false;
  if (method == 0) {
    stopImmediate();
    encoderZeroHere();
    stepper->setCurrentPosition(0);
    targetDeg = 0;
    mode = MODE_IDLE;
    homed = true;
    coSetPositionActual(0);
    coSetHomingAttained(true);
    return;
  }
  if (homingSpeed < 0.001f) homingSpeed = 5.0f;
  mode = MODE_HOMING;
  homingPhase = 1;
  homingStartMs = millis();
  pidState = 0;
  velTargetDegS = homingSpeed * homingDir;
  applyContinuousRun(velTargetDegS);
}

static void homingService() {
  if (mode != MODE_HOMING || homingPhase == 0 || !stepper) return;
  if ((uint32_t)(millis() - homingStartMs) > coOd()->homing_timeout_ms) {
    homingFinish(false);
    return;
  }
  if (homingPhase == 1) {
    bool trig = (homingMethod == 1 && endstopActive) || (homingMethod == 2 && stallFlag);
    if (!trig) {
      if (!stepper->isRunning()) applyContinuousRun(homingSpeed * homingDir);
      return;
    }
    stallFlag = false;
    stepper->forceStop();
    encoderZeroHere();
    stepper->setCurrentPosition(0);
    targetDeg = 0;
    homingFinish(true);
  }
}

static void tmcPollStatus() {
  bool uartOk = tmc.isSetupAndCommunicating();
  tmcFlagsA = uartOk ? 0x01 : 0x00;
  if (!uartOk) return;
  tmcStallGuard = (uint16_t)tmc.getStallGuardResult();
  TMC2209::Status st = tmc.getStatus();
  TMC2209::GlobalStatus gs = tmc.getGlobalStatus();
  if (st.over_temperature_warning) tmcFlagsA |= (1u << 1);
  if (st.over_temperature_shutdown) tmcFlagsA |= (1u << 2);
  if (st.short_to_ground_a) tmcFlagsA |= (1u << 3);
  if (st.short_to_ground_b) tmcFlagsA |= (1u << 4);
  if (st.low_side_short_a) tmcFlagsA |= (1u << 5);
  if (st.low_side_short_b) tmcFlagsA |= (1u << 6);
  tmcGstat = (gs.reset ? 1 : 0) | (gs.drv_err ? 2 : 0) | (gs.uv_cp ? 4 : 0);
  tmcCsActual = (uint8_t)(st.current_scaling & 0x1F);
  (void)tmcFlagsB; (void)tmcInterstep; (void)tmcStallGuard; (void)tmcCsActual;
}

static void tmcDriverService() {
  uint32_t now = millis();
  if ((uint32_t)(now - tmcPollMs) < 100) return;
  tmcPollMs = now;
  if (!driverEnabled) return;
  tmcPollStatus();
  if (tmcFlagsA & (1u << 2)) {
    stopImmediate();
    coRaiseFault(ERR_TEMP, 0x09);
    Serial.println("# FAULT: TMC OT");
  } else if (tmcFlagsA & ((1u << 3) | (1u << 4) | (1u << 5) | (1u << 6))) {
    stopImmediate();
    coRaiseFault(ERR_SHORT, 0x03);
    Serial.println("# FAULT: TMC short");
  }
}

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
    }
    return;
  }
  if (s.state == TWAI_STATE_STOPPED && canRestartPending) {
    if (twai_start() == ESP_OK) canRestartPending = false;
  }
}

static void paramsSaveNvs() {
  prefs.begin("co402", false);
  prefs.putUChar("id", coOd()->node_id);
  prefs.putUInt("br", coOd()->bitrate);
  prefs.putUShort("hb", coOd()->heartbeat_ms);
  prefs.putUInt("spr", coOd()->steps_per_rev);
  prefs.putUInt("ms", coOd()->microsteps);
  prefs.putUChar("run", coOd()->run_current);
  prefs.putUChar("hold", coOd()->hold_current);
  prefs.putUChar("inv", coOd()->invert_dir);
  prefs.putUChar("cl", coOd()->closed_loop);
  prefs.end();
}

static void paramsLoadNvs() {
  prefs.begin("co402", true);
  uint8_t id = prefs.getUChar("id", CO_FACTORY_NODE_ID);
  if (id >= 1 && id <= 127) coOd()->node_id = id;
  uint32_t br = prefs.getUInt("br", 1000000);
  if (br == 125000 || br == 250000 || br == 500000 || br == 1000000) coOd()->bitrate = br;
  coOd()->heartbeat_ms = prefs.getUShort("hb", 1000);
  coOd()->steps_per_rev = prefs.getUInt("spr", 200);
  coOd()->microsteps = prefs.getUInt("ms", 16);
  coOd()->run_current = prefs.getUChar("run", 20);
  coOd()->hold_current = prefs.getUChar("hold", 5);
  coOd()->invert_dir = prefs.getUChar("inv", 0);
  coOd()->closed_loop = prefs.getUChar("cl", 1);
  prefs.end();
}

static void appEnable(bool on) { setDriverEnabled(on); }
static void appDisable() { setDriverEnabled(false); }
static void appQuickStop() {
  stopImmediate();
  setDriverEnabled(false);
  mode = MODE_IDLE;
}
static void appHalt() {
  if (stepper) stepper->stopMove();
  velTargetDegS = 0;
  pidState = 0;
  mode = MODE_IDLE;
}
static void appNewSetpoint(int32_t pos, bool relative) {
  encoderUpdate();
  double deg = countsToDeg(pos);
  if (relative) deg += encoderDeg();
  beginPositionMove(deg);
}
static void appVelocity(int32_t vel) { beginVelocity(countsSToDegS(vel)); }
static void appHome(int8_t method) {
  float spd = countsSToDegS((int32_t)coOd()->homing_speed1);
  if (method == 35 || method == 0) homingBeginInternal(0, 1, spd);
  else if (method == 17) homingBeginInternal(1, -1, spd);
  else if (method == 18) homingBeginInternal(1, 1, spd);
  else if (method == -1) homingBeginInternal(2, -1, spd);
  else if (method == -2) homingBeginInternal(2, 1, spd);
  else coRaiseFault(ERR_HOMING, 0x21);
}
static void appFaultReset() { /* coils stay off until enable sequence */ }
static void appSave() { paramsSaveNvs(); Serial.println("# NVS saved"); }
static void appLoadDefaults() { applyTmcFromOd(); }
static void appParam(uint16_t index) {
  if (index == 0x2000) Serial.printf("# node id now %u (save + reset to apply COB-IDs)\n",
                                     (unsigned)coOd()->node_id);
  else applyTmcFromOd();
}

static void serialLineExecute(char *line) {
  if (line[0] == '#' || line[0] == 0) return;
  char *save = nullptr;
  char *tokId = strtok_r(line, " \t", &save);
  char *tokRtr = strtok_r(nullptr, " \t", &save);
  char *tokHex = strtok_r(nullptr, " \t", &save);
  if (!tokId || !tokRtr) return;
  uint32_t id = strtoul(tokId, nullptr, 16);
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
  if (tokRtr[0] == '1') return;
  canTx(id, data, len);
  coOnRx(id, data, len);
}

static void serialService() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { lineBuf[lineLen] = 0; serialLineExecute(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }
}

void setup() {
  delay(400);
  Serial.begin(115200);
  delay(200);
  Serial.printf("# GrafitoCANStepper CANopen fw %u.%u CiA402\n", FW_MAJOR, FW_MINOR);

  pinMode(PIN_TMC_EN, OUTPUT);
  digitalWrite(PIN_TMC_EN, HIGH);
  pinMode(PIN_DIAG, INPUT);
  pinMode(PIN_HOME, INPUT_PULLUP);
  pinMode(PIN_ENC_CS, OUTPUT);
  digitalWrite(PIN_ENC_CS, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_DIAG), onDiagRise, RISING);
  encSpi.begin(PIN_ENC_CLK, PIN_ENC_DO, -1, -1);

  CoApp app = {};
  app.tx = canTx;
  app.on_enable = appEnable;
  app.on_disable = appDisable;
  app.on_quick_stop = appQuickStop;
  app.on_halt = appHalt;
  app.on_new_setpoint = appNewSetpoint;
  app.on_velocity = appVelocity;
  app.on_home = appHome;
  app.on_fault_reset = appFaultReset;
  app.on_save = appSave;
  app.on_load_defaults = appLoadDefaults;
  app.on_param = appParam;
  prefs.begin("co402", true);
  uint8_t bootId = prefs.getUChar("id", CO_FACTORY_NODE_ID);
  prefs.end();
  if (bootId < 1 || bootId > 127) bootId = CO_FACTORY_NODE_ID;
  coInit(bootId, &app);
  paramsLoadNvs();

  stepperEngine.init();
  stepper = stepperEngine.stepperConnectToPin(PIN_STEP);
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR, coOd()->invert_dir == 0);
    stepper->setAutoEnable(false);
    stepper->setCurrentPosition(0);
  }
  tmc.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0, PIN_TMC_RX, PIN_TMC_TX);
  tmc.enableAutomaticCurrentScaling();
  tmc.setCoolStepDurationThreshold(5000);
  applyTmcFromOd();
  encoderUpdate();
  pidPrevDeg = encoderDeg();
  setDriverEnabled(false);

  if (canStart(coOd()->bitrate))
    Serial.printf("# CAN started (%lu bit/s) node %u\n",
                  (unsigned long)coOd()->bitrate, (unsigned)coOd()->node_id);
  else
    Serial.println("# ERROR: CAN start failed");
}

void loop() {
  canHealthService();
  tmcDriverService();
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    if (rx.extd) continue;
    if (!rx.rtr) serialPrintFrame(rx.identifier, false, rx.data, rx.data_length_code);
    coOnRx(rx.identifier, rx.data, rx.data_length_code);
  }
  serialService();
  endstopService();
  homingService();
  velocityService();
  pidService();
  moveDoneService();
  encoderUpdate();
  coSetPositionActual(degToCounts(encoderDeg()));
  coSetEndstopActive(endstopActive);
  coTick(millis());
  delay(1);
}
