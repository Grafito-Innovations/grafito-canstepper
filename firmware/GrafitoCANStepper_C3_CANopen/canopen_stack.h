/*
 * Minimal CiA 301 + CiA 402 slave matching canstepper/canopen402.py
 * and GrafitoCANStepper.eds. Include this from the Arduino sketch.
 */
#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>

static const uint32_t CO_VENDOR_ID     = 0x000005A3UL;
static const uint32_t CO_PRODUCT_CODE  = 0x00004333UL;
static const uint32_t CO_REVISION      = 0x00020100UL;
static const uint32_t CO_SERIAL        = 0x00000001UL;
static const uint32_t CO_DEVICE_TYPE   = 0x00040192UL;
static const uint32_t CO_SUPPORTED_MODES = 0x00000025UL;
static const char     CO_DEVICE_NAME[] = "CANStepper";
static const char     CO_HW_VERSION[]  = "C3";
static const char     CO_SW_VERSION[]  = "2.1.0";

static const uint8_t  NMT_BOOTUP = 0x00;
static const uint8_t  NMT_STOPPED = 0x04;
static const uint8_t  NMT_OPERATIONAL = 0x05;
static const uint8_t  NMT_PREOP = 0x7F;

static const uint8_t  NMT_CMD_START = 0x01;
static const uint8_t  NMT_CMD_STOP = 0x02;
static const uint8_t  NMT_CMD_PREOP = 0x80;
static const uint8_t  NMT_CMD_RESET = 0x81;
static const uint8_t  NMT_CMD_RESET_COMM = 0x82;

static const uint8_t  DS_SOD = 1;
static const uint8_t  DS_RTSO = 2;
static const uint8_t  DS_SO = 3;
static const uint8_t  DS_OE = 4;
static const uint8_t  DS_QSA = 5;
static const uint8_t  DS_FAULT = 7;

static const uint16_t SW_SOD = 0x0040;
static const uint16_t SW_RTSO = 0x0021;
static const uint16_t SW_SO = 0x0023;
static const uint16_t SW_OE = 0x0027;
static const uint16_t SW_QSA = 0x0007;
static const uint16_t SW_FAULT = 0x0008;
static const uint16_t SW_VOLTAGE = 0x0010;
static const uint16_t SW_REMOTE = 0x0200;
static const uint16_t SW_TARGET = 0x0400;
static const uint16_t SW_SPACK = 0x1000;

static const uint16_t CW_NEW_SP = 0x0010;
static const uint16_t CW_RELATIVE = 0x0040;
static const uint16_t CW_FAULT_RST = 0x0080;
static const uint16_t CW_HALT = 0x0100;

static const uint32_t SDO_ABORT_NOOBJ = 0x06020000UL;
static const uint32_t SDO_ABORT_NOSUB = 0x06090011UL;
static const uint32_t SDO_ABORT_RO = 0x06010002UL;
static const uint32_t SDO_ABORT_WO = 0x06010001UL;
static const uint32_t SDO_ABORT_LEN = 0x06070010UL;
static const uint32_t SDO_ABORT_VAL = 0x06090030UL;
static const uint32_t SDO_ABORT_TOGGLE = 0x05030000UL;

static const uint16_t ERR_SHORT = 0x2310;
static const uint16_t ERR_TEMP = 0x4310;
static const uint16_t ERR_ENCODER = 0x7305;
static const uint16_t ERR_BLOCKED = 0x7122;
static const uint16_t ERR_HOMING = 0xFF01;

struct CoApp {
  void (*tx)(uint32_t id, const uint8_t *d, uint8_t len);
  void (*on_enable)(bool on);
  void (*on_disable)();
  void (*on_quick_stop)();
  void (*on_halt)();
  void (*on_new_setpoint)(int32_t pos, bool relative);
  void (*on_velocity)(int32_t vel);
  void (*on_home)(int8_t method);
  void (*on_fault_reset)();
  void (*on_save)();
  void (*on_load_defaults)();
  void (*on_param)(uint16_t index);
  void (*on_lut_cmd)(uint8_t action);
};

struct CoOd {
  uint8_t  node_id;
  uint16_t heartbeat_ms;
  uint32_t sync_cob;
  uint8_t  error_reg;
  uint16_t error_code;
  uint16_t controlword;
  uint16_t statusword;
  int16_t  qs_option;
  int8_t   mode;
  int8_t   mode_display;
  int32_t  pos_actual;
  int32_t  vel_actual;
  int32_t  target_pos;
  uint32_t profile_vel;
  uint32_t profile_acc;
  uint32_t profile_dec;
  uint32_t qs_dec;
  int8_t   homing_method;
  uint32_t homing_speed1;
  uint32_t homing_speed2;
  uint32_t homing_acc;
  int32_t  target_vel;
  uint8_t  steps_per_rev_unused; // placeholder to keep packing obvious
  uint32_t steps_per_rev;
  uint32_t microsteps;
  uint8_t  run_current;
  uint8_t  hold_current;
  uint8_t  stall_threshold;
  uint8_t  invert_dir;
  uint8_t  closed_loop;
  uint8_t  stealthchop;
  uint8_t  standstill;
  uint8_t  endstop_enable;
  uint8_t  endstop_active_high;
  uint32_t bitrate;
  float    pid_kp;
  float    pid_ki;
  float    pid_kd;
  float    pid_tol;
  float    pid_ka;
  float    profile_jerk;
  uint8_t  lut_enable;
  uint8_t  lut_valid;
  float    lut_peak_inl;
  uint8_t  enable_on_boot;
  uint8_t  zero_on_boot;
  uint32_t homing_timeout_ms;
  uint8_t  encoder_ok;
  uint8_t  endstop_active;
  float    bus_voltage;
  uint8_t  rpdo1_type;
  uint8_t  rpdo2_type;
  uint8_t  tpdo1_type;
  uint8_t  tpdo2_type;
};

static CoApp gApp;
static CoOd  gOd;
static uint8_t gNmt = NMT_PREOP;
static uint8_t gDrive = DS_SOD;
static uint16_t gPrevCw = 0;
static bool gTargetReached = false;
static bool gSpAck = false;
static bool gHomed = false;
static uint32_t gHbLast = 0;
static uint32_t gTpdoLast = 0;
static const uint8_t *gSegBlob = nullptr;
static uint16_t gSegLen = 0;
static uint16_t gSegOff = 0;
static uint16_t gSegIndex = 0;
static uint8_t  gSegSub = 0;
static uint8_t  gSegToggle = 0;

static inline uint32_t cobEmcy(uint8_t n) { return 0x080u + n; }
static inline uint32_t cobTpdo1(uint8_t n) { return 0x180u + n; }
static inline uint32_t cobRpdo1(uint8_t n) { return 0x200u + n; }
static inline uint32_t cobTpdo2(uint8_t n) { return 0x280u + n; }
static inline uint32_t cobRpdo2(uint8_t n) { return 0x300u + n; }
static inline uint32_t cobTsdo(uint8_t n) { return 0x580u + n; }
static inline uint32_t cobRsdo(uint8_t n) { return 0x600u + n; }
static inline uint32_t cobHb(uint8_t n) { return 0x700u + n; }

static void coTx(uint32_t id, const uint8_t *d, uint8_t len) {
  if (gApp.tx) gApp.tx(id, d, len);
}

static void coDefaults(uint8_t nodeId) {
  memset(&gOd, 0, sizeof(gOd));
  gOd.node_id = nodeId;
  gOd.heartbeat_ms = 1000;
  gOd.sync_cob = 0x080;
  gOd.qs_option = 2;
  gOd.mode = 1;
  gOd.mode_display = 1;
  gOd.profile_vel = 32768;     // 720 deg/s
  gOd.profile_acc = 65536;     // 1440 deg/s² (fw 2.1 precision default)
  gOd.profile_dec = 65536;
  gOd.qs_dec = 262144;
  gOd.homing_method = 35;
  gOd.homing_speed1 = 2275;
  gOd.homing_speed2 = 455;
  gOd.homing_acc = 131072;
  gOd.steps_per_rev = 200;
  gOd.microsteps = 16;
  gOd.run_current = 20;
  gOd.hold_current = 5;
  gOd.stall_threshold = 10;
  gOd.closed_loop = 1;
  gOd.stealthchop = 1;
  gOd.standstill = 1;
  gOd.endstop_enable = 1;
  gOd.endstop_active_high = 1;
  gOd.bitrate = 1000000;
  gOd.pid_kp = 10.0f;
  gOd.pid_ki = 0.3f;
  gOd.pid_kd = 0.35f;
  gOd.pid_tol = 0.35f;
  gOd.pid_ka = 0.04f;
  gOd.profile_jerk = 0.0f;     // 0 = auto amax/0.05 s
  gOd.lut_enable = 0;
  gOd.lut_valid = 0;
  gOd.lut_peak_inl = 0.0f;
  gOd.homing_timeout_ms = 30000;
  gOd.encoder_ok = 1;
  gOd.bus_voltage = 24.0f;
  gOd.rpdo1_type = 255;
  gOd.rpdo2_type = 255;
  gOd.tpdo1_type = 255;
  gOd.tpdo2_type = 255;
}

static uint16_t coStatusLow() {
  switch (gDrive) {
    case DS_RTSO: return SW_RTSO;
    case DS_SO:   return SW_SO;
    case DS_OE:   return SW_OE;
    case DS_QSA:  return SW_QSA;
    case DS_FAULT: return SW_FAULT;
    default:      return SW_SOD;
  }
}

static uint16_t coBuildStatus() {
  uint16_t sw = coStatusLow() | SW_REMOTE;
  if (gDrive == DS_RTSO || gDrive == DS_SO || gDrive == DS_OE || gDrive == DS_QSA)
    sw |= SW_VOLTAGE;
  if (gTargetReached) sw |= SW_TARGET;
  if (gSpAck || gHomed) sw |= SW_SPACK;
  gOd.statusword = sw;
  return sw;
}

static const char *coClassifyCw(uint16_t cw) {
  if ((cw & CW_FAULT_RST) == 0) {
    if ((cw & 0x0087) == 0x0006) return "shutdown";
    if ((cw & 0x008F) == 0x000F) return "enable_operation";
    if ((cw & 0x008F) == 0x0007) return "switch_on";
    if ((cw & 0x0086) == 0x0002) return "quick_stop";
    if ((cw & 0x0082) == 0x0000) return "disable_voltage";
  }
  return "none";
}

static void coApplyControlword(uint16_t cw) {
  bool risingReset = (cw & CW_FAULT_RST) && !(gPrevCw & CW_FAULT_RST);
  const char *cmd = coClassifyCw(cw);
  uint8_t prev = gDrive;

  if (gDrive == DS_FAULT && risingReset) {
    gDrive = DS_SOD;
    gOd.error_code = 0;
    gOd.error_reg = 0;
    if (gApp.on_fault_reset) gApp.on_fault_reset();
  } else if (gDrive == DS_SOD) {
    if (strcmp(cmd, "shutdown") == 0) gDrive = DS_RTSO;
  } else if (gDrive == DS_RTSO) {
    if (strcmp(cmd, "switch_on") == 0) gDrive = DS_SO;
    else if (strcmp(cmd, "disable_voltage") == 0 || strcmp(cmd, "quick_stop") == 0)
      gDrive = DS_SOD;
  } else if (gDrive == DS_SO) {
    if (strcmp(cmd, "enable_operation") == 0) {
      gDrive = DS_OE;
      if (gApp.on_enable) gApp.on_enable(true);
    } else if (strcmp(cmd, "shutdown") == 0) {
      gDrive = DS_RTSO;
    } else if (strcmp(cmd, "disable_voltage") == 0 || strcmp(cmd, "quick_stop") == 0) {
      gDrive = DS_SOD;
      if (gApp.on_disable) gApp.on_disable();
    }
  } else if (gDrive == DS_OE) {
    if (strcmp(cmd, "switch_on") == 0) {
      gDrive = DS_SO;
      if (gApp.on_disable) gApp.on_disable();
    } else if (strcmp(cmd, "shutdown") == 0) {
      gDrive = DS_RTSO;
      if (gApp.on_disable) gApp.on_disable();
    } else if (strcmp(cmd, "disable_voltage") == 0) {
      gDrive = DS_SOD;
      if (gApp.on_disable) gApp.on_disable();
    } else if (strcmp(cmd, "quick_stop") == 0) {
      gDrive = DS_QSA;
      if (gApp.on_quick_stop) gApp.on_quick_stop();
      gTargetReached = true;
    }
  } else if (gDrive == DS_QSA) {
    if (strcmp(cmd, "disable_voltage") == 0 || strcmp(cmd, "shutdown") == 0) {
      gDrive = DS_SOD;
      if (gApp.on_disable) gApp.on_disable();
    } else if (strcmp(cmd, "enable_operation") == 0) {
      gDrive = DS_OE;
      if (gApp.on_enable) gApp.on_enable(true);
    }
  }

  bool risingSp = (cw & CW_NEW_SP) && !(gPrevCw & CW_NEW_SP);
  gPrevCw = cw;
  gOd.controlword = cw;
  gOd.mode_display = gOd.mode;
  coBuildStatus();

  if (gDrive != DS_OE) return;

  if (cw & CW_HALT) {
    if (gApp.on_halt) gApp.on_halt();
    gTargetReached = true;
    gOd.vel_actual = 0;
    return;
  }

  if (gOd.mode == 1 && risingSp) {
    gSpAck = true;
    gHomed = false;
    gTargetReached = false;
    if (gApp.on_new_setpoint)
      gApp.on_new_setpoint(gOd.target_pos, (cw & CW_RELATIVE) != 0);
  } else if (gOd.mode == 3) {
    gSpAck = false;
    if (gApp.on_velocity) gApp.on_velocity(gOd.target_vel);
    gTargetReached = (gOd.target_vel == 0);
  } else if (gOd.mode == 6 && risingSp) {
    gSpAck = false;
    gTargetReached = false;
    gHomed = false;
    if (gApp.on_home) gApp.on_home(gOd.homing_method);
  }

  (void)prev;
}

static void coSendAbort(uint16_t index, uint8_t sub, uint32_t code) {
  uint8_t p[8] = { 0x80, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0 };
  memcpy(p + 4, &code, 4);
  coTx(cobTsdo(gOd.node_id), p, 8);
}

static void coSendDownloadOk(uint16_t index, uint8_t sub) {
  uint8_t p[8] = { 0x60, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0 };
  coTx(cobTsdo(gOd.node_id), p, 8);
}

static bool coCopyU(void *dst, const uint8_t *raw, uint8_t n) {
  memcpy(dst, raw, n);
  return true;
}

static bool coOdRead(uint16_t index, uint8_t sub, uint8_t *out, uint8_t *len, const uint8_t **blob) {
  *blob = nullptr;
  *len = 0;
  #define U8(v)  do { out[0] = (uint8_t)(v); *len = 1; return true; } while (0)
  #define I8(v)  do { out[0] = (uint8_t)(int8_t)(v); *len = 1; return true; } while (0)
  #define U16(v) do { uint16_t _x = (uint16_t)(v); memcpy(out, &_x, 2); *len = 2; return true; } while (0)
  #define I16(v) do { int16_t _x = (int16_t)(v); memcpy(out, &_x, 2); *len = 2; return true; } while (0)
  #define U32(v) do { uint32_t _x = (uint32_t)(v); memcpy(out, &_x, 4); *len = 4; return true; } while (0)
  #define I32(v) do { int32_t _x = (int32_t)(v); memcpy(out, &_x, 4); *len = 4; return true; } while (0)
  #define R32(v) do { float _x = (float)(v); memcpy(out, &_x, 4); *len = 4; return true; } while (0)
  #define STR(s) do { *blob = (const uint8_t *)(s); *len = (uint8_t)strlen(s); return true; } while (0)

  switch (index) {
    case 0x1000: if (sub == 0) U32(CO_DEVICE_TYPE); break;
    case 0x1001: if (sub == 0) U8(gOd.error_reg); break;
    case 0x1005: if (sub == 0) U32(gOd.sync_cob); break;
    case 0x1008: if (sub == 0) STR(CO_DEVICE_NAME); break;
    case 0x1009: if (sub == 0) STR(CO_HW_VERSION); break;
    case 0x100A: if (sub == 0) STR(CO_SW_VERSION); break;
    case 0x1014: if (sub == 0) U32(cobEmcy(gOd.node_id)); break;
    case 0x1017: if (sub == 0) U16(gOd.heartbeat_ms); break;
    case 0x1018:
      if (sub == 0) U8(4);
      if (sub == 1) U32(CO_VENDOR_ID);
      if (sub == 2) U32(CO_PRODUCT_CODE);
      if (sub == 3) U32(CO_REVISION);
      if (sub == 4) U32(CO_SERIAL);
      return false;
    case 0x1200:
      if (sub == 0) U8(2);
      if (sub == 1) U32(cobRsdo(gOd.node_id));
      if (sub == 2) U32(cobTsdo(gOd.node_id));
      return false;
    case 0x1400:
      if (sub == 0) U8(2);
      if (sub == 1) U32(cobRpdo1(gOd.node_id));
      if (sub == 2) U8(gOd.rpdo1_type);
      return false;
    case 0x1401:
      if (sub == 0) U8(2);
      if (sub == 1) U32(cobRpdo2(gOd.node_id));
      if (sub == 2) U8(gOd.rpdo2_type);
      return false;
    case 0x1600:
      if (sub == 0) U8(2);
      if (sub == 1) U32(0x60400010UL);
      if (sub == 2) U32(0x607A0020UL);
      return false;
    case 0x1601:
      if (sub == 0) U8(2);
      if (sub == 1) U32(0x60400010UL);
      if (sub == 2) U32(0x60FF0020UL);
      return false;
    case 0x1800:
      if (sub == 0) U8(2);
      if (sub == 1) U32(cobTpdo1(gOd.node_id));
      if (sub == 2) U8(gOd.tpdo1_type);
      return false;
    case 0x1801:
      if (sub == 0) U8(2);
      if (sub == 1) U32(cobTpdo2(gOd.node_id));
      if (sub == 2) U8(gOd.tpdo2_type);
      return false;
    case 0x1A00:
      if (sub == 0) U8(2);
      if (sub == 1) U32(0x60410010UL);
      if (sub == 2) U32(0x60640020UL);
      return false;
    case 0x1A01:
      if (sub == 0) U8(2);
      if (sub == 1) U32(0x60410010UL);
      if (sub == 2) U32(0x606C0020UL);
      return false;
    case 0x603F: if (sub == 0) U16(gOd.error_code); break;
    case 0x6040: if (sub == 0) U16(gOd.controlword); break;
    case 0x6041: if (sub == 0) U16(coBuildStatus()); break;
    case 0x605A: if (sub == 0) I16(gOd.qs_option); break;
    case 0x6060: if (sub == 0) I8(gOd.mode); break;
    case 0x6061: if (sub == 0) I8(gOd.mode_display); break;
    case 0x6064: if (sub == 0) I32(gOd.pos_actual); break;
    case 0x606C: if (sub == 0) I32(gOd.vel_actual); break;
    case 0x607A: if (sub == 0) I32(gOd.target_pos); break;
    case 0x6081: if (sub == 0) U32(gOd.profile_vel); break;
    case 0x6083: if (sub == 0) U32(gOd.profile_acc); break;
    case 0x6084: if (sub == 0) U32(gOd.profile_dec); break;
    case 0x6085: if (sub == 0) U32(gOd.qs_dec); break;
    case 0x6098: if (sub == 0) I8(gOd.homing_method); break;
    case 0x6099:
      if (sub == 0) U8(2);
      if (sub == 1) U32(gOd.homing_speed1);
      if (sub == 2) U32(gOd.homing_speed2);
      return false;
    case 0x609A: if (sub == 0) U32(gOd.homing_acc); break;
    case 0x60FF: if (sub == 0) I32(gOd.target_vel); break;
    case 0x6502: if (sub == 0) U32(CO_SUPPORTED_MODES); break;
    case 0x2000: if (sub == 0) U8(gOd.node_id); break;
    case 0x2001: if (sub == 0) U32(gOd.steps_per_rev); break;
    case 0x2002: if (sub == 0) U32(gOd.microsteps); break;
    case 0x2003: if (sub == 0) U8(gOd.run_current); break;
    case 0x2004: if (sub == 0) U8(gOd.hold_current); break;
    case 0x2005: if (sub == 0) U8(gOd.stall_threshold); break;
    case 0x2006: if (sub == 0) U8(gOd.invert_dir); break;
    case 0x2007: if (sub == 0) U8(gOd.closed_loop); break;
    case 0x2008:
    case 0x2009:
      return false;
    case 0x200A: if (sub == 0) U8(gOd.stealthchop); break;
    case 0x200B: if (sub == 0) U8(gOd.standstill); break;
    case 0x200C: if (sub == 0) U8(gOd.endstop_enable); break;
    case 0x200D: if (sub == 0) U8(gOd.endstop_active_high); break;
    case 0x200E: if (sub == 0) U32(gOd.bitrate); break;
    case 0x200F: if (sub == 0) R32(gOd.pid_kp); break;
    case 0x2010: if (sub == 0) R32(gOd.pid_ki); break;
    case 0x2011: if (sub == 0) R32(gOd.pid_kd); break;
    case 0x2012: if (sub == 0) R32(gOd.pid_tol); break;
    case 0x2013: if (sub == 0) U8(gOd.enable_on_boot); break;
    case 0x2014: if (sub == 0) U8(gOd.zero_on_boot); break;
    case 0x2015: if (sub == 0) U32(gOd.homing_timeout_ms); break;
    case 0x2016: if (sub == 0) U16(0x0201); break;
    case 0x2017: if (sub == 0) U8(gOd.encoder_ok); break;
    case 0x2018: if (sub == 0) U8(gOd.endstop_active); break;
    case 0x2019: if (sub == 0) R32(gOd.bus_voltage); break;
    case 0x201A: if (sub == 0) R32(gOd.pid_ka); break;
    case 0x201B: if (sub == 0) R32(gOd.profile_jerk); break;
    case 0x201C: if (sub == 0) U8(gOd.lut_enable); break;
    case 0x201D: return false;
    case 0x201E: if (sub == 0) U8(gOd.lut_valid); break;
    case 0x201F: if (sub == 0) R32(gOd.lut_peak_inl); break;
    default: return false;
  }
  return false;
  #undef U8
  #undef I8
  #undef U16
  #undef I16
  #undef U32
  #undef I32
  #undef R32
  #undef STR
}

static bool coKnownIndex(uint16_t index) {
  switch (index) {
    case 0x1000: case 0x1001: case 0x1005: case 0x1008: case 0x1009: case 0x100A:
    case 0x1014: case 0x1017: case 0x1018: case 0x1200:
    case 0x1400: case 0x1401: case 0x1600: case 0x1601:
    case 0x1800: case 0x1801: case 0x1A00: case 0x1A01:
    case 0x603F: case 0x6040: case 0x6041: case 0x605A:
    case 0x6060: case 0x6061: case 0x6064: case 0x606C:
    case 0x607A: case 0x6081: case 0x6083: case 0x6084: case 0x6085:
    case 0x6098: case 0x6099: case 0x609A: case 0x60FF: case 0x6502:
    case 0x2000: case 0x2001: case 0x2002: case 0x2003: case 0x2004:
    case 0x2005: case 0x2006: case 0x2007: case 0x2008: case 0x2009:
    case 0x200A: case 0x200B: case 0x200C: case 0x200D: case 0x200E:
    case 0x200F: case 0x2010: case 0x2011: case 0x2012: case 0x2013:
    case 0x2014: case 0x2015: case 0x2016: case 0x2017: case 0x2018:
    case 0x2019: case 0x201A: case 0x201B: case 0x201C: case 0x201D:
    case 0x201E: case 0x201F:
      return true;
    default:
      return false;
  }
}

static bool coOdWrite(uint16_t index, uint8_t sub, const uint8_t *raw, uint8_t n) {
  if (index == 0x2008 && sub == 0) {
    if (n >= 1 && raw[0] == 1 && gApp.on_save) gApp.on_save();
    return true;
  }
  if (index == 0x2009 && sub == 0) {
    if (n >= 1 && raw[0] == 1) {
      uint8_t keep = gOd.node_id;
      coDefaults(keep);
      if (gApp.on_load_defaults) gApp.on_load_defaults();
    }
    return true;
  }

  auto need = [&](uint8_t k) { return n >= k; };

  switch (index) {
    case 0x1005: if (sub == 0 && need(4)) { coCopyU(&gOd.sync_cob, raw, 4); return true; } break;
    case 0x1017: if (sub == 0 && need(2)) { coCopyU(&gOd.heartbeat_ms, raw, 2); return true; } break;
    case 0x1400:
      if (sub == 1 && need(4)) return true;  // COB-ID accepted; runtime ID follows node_id
      if (sub == 2 && need(1)) { gOd.rpdo1_type = raw[0]; return true; } break;
    case 0x1401:
      if (sub == 1 && need(4)) return true;
      if (sub == 2 && need(1)) { gOd.rpdo2_type = raw[0]; return true; } break;
    case 0x1800:
      if (sub == 1 && need(4)) return true;
      if (sub == 2 && need(1)) { gOd.tpdo1_type = raw[0]; return true; } break;
    case 0x1801:
      if (sub == 1 && need(4)) return true;
      if (sub == 2 && need(1)) { gOd.tpdo2_type = raw[0]; return true; } break;
    case 0x6040: {
      if (sub != 0 || !need(2)) break;
      uint16_t cw; memcpy(&cw, raw, 2);
      coApplyControlword(cw);
      return true;
    }
    case 0x605A: if (sub == 0 && need(2)) { coCopyU(&gOd.qs_option, raw, 2); return true; } break;
    case 0x6060: {
      if (sub != 0 || !need(1)) break;
      int8_t m = (int8_t)raw[0];
      if (m != 0 && m != 1 && m != 3 && m != 6) return false;
      gOd.mode = m;
      gOd.mode_display = m;
      return true;
    }
    case 0x607A: if (sub == 0 && need(4)) { coCopyU(&gOd.target_pos, raw, 4); return true; } break;
    case 0x6081: if (sub == 0 && need(4)) { coCopyU(&gOd.profile_vel, raw, 4); return true; } break;
    case 0x6083: if (sub == 0 && need(4)) { coCopyU(&gOd.profile_acc, raw, 4); return true; } break;
    case 0x6084: if (sub == 0 && need(4)) { coCopyU(&gOd.profile_dec, raw, 4); return true; } break;
    case 0x6085: if (sub == 0 && need(4)) { coCopyU(&gOd.qs_dec, raw, 4); return true; } break;
    case 0x6098: if (sub == 0 && need(1)) { gOd.homing_method = (int8_t)raw[0]; return true; } break;
    case 0x6099:
      if (sub == 1 && need(4)) { coCopyU(&gOd.homing_speed1, raw, 4); return true; }
      if (sub == 2 && need(4)) { coCopyU(&gOd.homing_speed2, raw, 4); return true; }
      break;
    case 0x609A: if (sub == 0 && need(4)) { coCopyU(&gOd.homing_acc, raw, 4); return true; } break;
    case 0x60FF: if (sub == 0 && need(4)) { coCopyU(&gOd.target_vel, raw, 4); return true; } break;
    case 0x2000: {
      if (sub != 0 || !need(1) || raw[0] < 1 || raw[0] > 127) return false;
      gOd.node_id = raw[0];
      if (gApp.on_param) gApp.on_param(index);
      return true;
    }
    case 0x2001: if (sub == 0 && need(4)) { coCopyU(&gOd.steps_per_rev, raw, 4); if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x2002: {
      if (sub != 0 || !need(4)) break;
      uint32_t ms; memcpy(&ms, raw, 4);
      if (!ms || (ms & (ms - 1)) || ms > 256) return false;
      gOd.microsteps = ms;
      if (gApp.on_param) gApp.on_param(index);
      return true;
    }
    case 0x2003: if (sub == 0 && need(1) && raw[0] >= 1 && raw[0] <= 100) { gOd.run_current = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x2004: if (sub == 0 && need(1) && raw[0] <= 100) { gOd.hold_current = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x2005: if (sub == 0 && need(1)) { gOd.stall_threshold = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x2006: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.invert_dir = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x2007: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.closed_loop = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x200A: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.stealthchop = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x200B: if (sub == 0 && need(1) && raw[0] <= 3) { gOd.standstill = raw[0]; if (gApp.on_param) gApp.on_param(index); return true; } break;
    case 0x200C: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.endstop_enable = raw[0]; return true; } break;
    case 0x200D: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.endstop_active_high = raw[0]; return true; } break;
    case 0x200E: {
      if (sub != 0 || !need(4)) break;
      uint32_t br; memcpy(&br, raw, 4);
      if (br != 125000 && br != 250000 && br != 500000 && br != 1000000) return false;
      gOd.bitrate = br;
      return true;
    }
    case 0x200F: if (sub == 0 && need(4)) { coCopyU(&gOd.pid_kp, raw, 4); return true; } break;
    case 0x2010: if (sub == 0 && need(4)) { coCopyU(&gOd.pid_ki, raw, 4); return true; } break;
    case 0x2011: if (sub == 0 && need(4)) { coCopyU(&gOd.pid_kd, raw, 4); return true; } break;
    case 0x2012: if (sub == 0 && need(4)) { coCopyU(&gOd.pid_tol, raw, 4); return true; } break;
    case 0x2013: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.enable_on_boot = raw[0]; return true; } break;
    case 0x2014: if (sub == 0 && need(1) && raw[0] <= 1) { gOd.zero_on_boot = raw[0]; return true; } break;
    case 0x2015: if (sub == 0 && need(4)) { coCopyU(&gOd.homing_timeout_ms, raw, 4); return true; } break;
    case 0x201A: if (sub == 0 && need(4)) { coCopyU(&gOd.pid_ka, raw, 4); return true; } break;
    case 0x201B: if (sub == 0 && need(4)) { coCopyU(&gOd.profile_jerk, raw, 4); return true; } break;
    case 0x201C: {
      if (sub != 0 || !need(1) || raw[0] > 1) break;
      if (raw[0] && !gOd.lut_valid) return false;
      gOd.lut_enable = raw[0];
      return true;
    }
    case 0x201D:
      if (sub == 0 && need(1) && gApp.on_lut_cmd) { gApp.on_lut_cmd(raw[0]); return true; }
      break;
    default: break;
  }
  return false;
}

static void coSdoRead(uint16_t index, uint8_t sub) {
  uint8_t tmp[4] = {0};
  uint8_t len = 0;
  const uint8_t *blob = nullptr;
  if (index == 0x2008 || index == 0x2009 || index == 0x201D) {
    coSendAbort(index, sub, SDO_ABORT_WO);
    return;
  }
  if (!coOdRead(index, sub, tmp, &len, &blob)) {
    coSendAbort(index, sub, coKnownIndex(index) ? SDO_ABORT_NOSUB : SDO_ABORT_NOOBJ);
    return;
  }
  if (blob) {
    gSegBlob = blob;
    gSegLen = len;
    gSegOff = 0;
    gSegIndex = index;
    gSegSub = sub;
    gSegToggle = 0;
    uint8_t p[8] = { 0x41, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0 };
    uint32_t sz = len;
    memcpy(p + 4, &sz, 4);
    coTx(cobTsdo(gOd.node_id), p, 8);
    return;
  }
  uint8_t n = (uint8_t)(4 - len);
  uint8_t cmd = (uint8_t)(0x43 | (n << 2));
  uint8_t p[8] = { cmd, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0 };
  memcpy(p + 4, tmp, len);
  coTx(cobTsdo(gOd.node_id), p, 8);
}

static void coSdoSegment(uint8_t cmd) {
  if (!gSegBlob) {
    coSendAbort(0, 0, SDO_ABORT_TOGGLE);
    return;
  }
  uint8_t toggle = (cmd >> 4) & 1;
  if (toggle != gSegToggle) {
    coSendAbort(gSegIndex, gSegSub, SDO_ABORT_TOGGLE);
    return;
  }
  uint8_t remain = (uint8_t)(gSegLen - gSegOff);
  uint8_t take = remain > 7 ? 7 : remain;
  uint8_t n = (uint8_t)(7 - take);
  bool last = (gSegOff + take) >= gSegLen;
  uint8_t p[8] = {0};
  p[0] = (uint8_t)(0x00 | (toggle << 4) | (n << 1) | (last ? 1 : 0));
  memcpy(p + 1, gSegBlob + gSegOff, take);
  gSegOff = (uint16_t)(gSegOff + take);
  gSegToggle ^= 1;
  if (last) gSegBlob = nullptr;
  coTx(cobTsdo(gOd.node_id), p, 8);
}

static void coOnSdo(const uint8_t *d, uint8_t len) {
  if (len < 4) return;
  uint8_t cmd = d[0];
  uint16_t index = (uint16_t)d[1] | ((uint16_t)d[2] << 8);
  uint8_t sub = d[3];
  uint8_t ccs = (cmd >> 5) & 7;
  if (ccs == 1) {
    if (!(cmd & 0x02)) { coSendAbort(index, sub, SDO_ABORT_LEN); return; }
    uint8_t n = (cmd >> 2) & 3;
    uint8_t rawn = (uint8_t)((cmd & 0x01) ? (4 - n) : 4);
    const uint8_t *raw = d + 4;
    bool ro = (index <= 0x1018 && index != 0x1005 && index != 0x1017) ||
              index == 0x1000 || index == 0x1001 || index == 0x1008 ||
              index == 0x1009 || index == 0x100A || index == 0x1014 ||
              index == 0x1018 || index == 0x1200 ||
              index == 0x1600 || index == 0x1601 ||
              index == 0x1A00 || index == 0x1A01 ||
              index == 0x603F || index == 0x6041 || index == 0x6061 ||
              index == 0x6064 || index == 0x606C || index == 0x6502 ||
              index == 0x2016 || index == 0x2017 || index == 0x2018 ||
              index == 0x2019 || index == 0x201E || index == 0x201F;
    if (!coKnownIndex(index)) { coSendAbort(index, sub, SDO_ABORT_NOOBJ); return; }
    if (ro) { coSendAbort(index, sub, SDO_ABORT_RO); return; }
    if (!coOdWrite(index, sub, raw, rawn)) {
      coSendAbort(index, sub, SDO_ABORT_VAL);
      return;
    }
    coSendDownloadOk(index, sub);
    return;
  }
  if (ccs == 2) { coSdoRead(index, sub); return; }
  if (ccs == 3) { coSdoSegment(cmd); return; }
  coSendAbort(index, sub, SDO_ABORT_LEN);
}

static void coSendTpdo() {
  uint16_t sw = coBuildStatus();
  uint8_t p[8] = {0};
  memcpy(p, &sw, 2);
  memcpy(p + 2, &gOd.pos_actual, 4);
  coTx(cobTpdo1(gOd.node_id), p, 8);
  memcpy(p + 2, &gOd.vel_actual, 4);
  coTx(cobTpdo2(gOd.node_id), p, 8);
}

static void coOnRpdo1(const uint8_t *d, uint8_t len) {
  if (len < 6) return;
  memcpy(&gOd.target_pos, d + 2, 4);
  uint16_t cw; memcpy(&cw, d, 2);
  coApplyControlword(cw);
}

static void coOnRpdo2(const uint8_t *d, uint8_t len) {
  if (len < 6) return;
  memcpy(&gOd.target_vel, d + 2, 4);
  uint16_t cw; memcpy(&cw, d, 2);
  coApplyControlword(cw);
}

static void coBootup() {
  gNmt = NMT_PREOP;
  uint8_t st = NMT_BOOTUP;
  coTx(cobHb(gOd.node_id), &st, 1);
}

static void coOnNmt(const uint8_t *d, uint8_t len) {
  if (len < 2) return;
  uint8_t cmd = d[0], dest = d[1];
  if (dest != 0 && dest != gOd.node_id) return;
  if (cmd == NMT_CMD_START) gNmt = NMT_OPERATIONAL;
  else if (cmd == NMT_CMD_STOP) gNmt = NMT_STOPPED;
  else if (cmd == NMT_CMD_PREOP) gNmt = NMT_PREOP;
  else if (cmd == NMT_CMD_RESET || cmd == NMT_CMD_RESET_COMM) {
    gDrive = DS_SOD;
    gPrevCw = 0;
    gOd.controlword = 0;
    gTargetReached = false;
    gSpAck = false;
    gHomed = false;
    if (gApp.on_disable) gApp.on_disable();
    coBootup();
  }
}

static void coInit(uint8_t nodeId, const CoApp *app) {
  gApp = *app;
  coDefaults(nodeId);
  gDrive = DS_SOD;
  gPrevCw = 0;
  gTargetReached = false;
  gSpAck = false;
  gHomed = false;
  gHbLast = 0;
  gTpdoLast = 0;
  coBuildStatus();
  coBootup();
}

static void coOnRx(uint32_t id, const uint8_t *d, uint8_t len) {
  if (id == 0x000) { coOnNmt(d, len); return; }
  if (id == cobRsdo(gOd.node_id)) {
    if (gNmt == NMT_PREOP || gNmt == NMT_OPERATIONAL) coOnSdo(d, len);
    return;
  }
  if (gNmt != NMT_OPERATIONAL) return;
  if (id == cobRpdo1(gOd.node_id)) coOnRpdo1(d, len);
  else if (id == cobRpdo2(gOd.node_id)) coOnRpdo2(d, len);
}

static void coTick(uint32_t nowMs) {
  if (gOd.heartbeat_ms && (uint32_t)(nowMs - gHbLast) >= gOd.heartbeat_ms) {
    gHbLast = nowMs;
    uint8_t st = gNmt;
    coTx(cobHb(gOd.node_id), &st, 1);
  }
  if (gNmt == NMT_OPERATIONAL && (uint32_t)(nowMs - gTpdoLast) >= 20) {
    gTpdoLast = nowMs;
    coSendTpdo();
  }
}

static void coSetPositionActual(int32_t c) { gOd.pos_actual = c; }
static void coSetVelocityActual(int32_t v) { gOd.vel_actual = v; }
static void coSetEncoderOk(bool ok) { gOd.encoder_ok = ok ? 1 : 0; }
static void coSetEndstopActive(bool a) { gOd.endstop_active = a ? 1 : 0; }
static void coSetTargetReached(bool v) { gTargetReached = v; gSpAck = v ? gSpAck : false; }
static void coSetHomingAttained(bool v) { gHomed = v; if (v) gTargetReached = true; }

static void coRaiseFault(uint16_t code, uint8_t errReg) {
  gOd.error_code = code;
  gOd.error_reg = errReg;
  gDrive = DS_FAULT;
  gSpAck = false;
  uint8_t p[8] = {0};
  memcpy(p, &code, 2);
  p[2] = errReg;
  coTx(cobEmcy(gOd.node_id), p, 8);
  if (gApp.on_disable) gApp.on_disable();
}

static inline CoOd *coOd() { return &gOd; }
static inline uint8_t coNmtState() { return gNmt; }
static inline uint8_t coDriveState() { return gDrive; }
