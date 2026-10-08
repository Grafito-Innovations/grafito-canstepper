"""Grafito CAN Stepper Protocol v1 (GCSP v1) — frame codecs and registries.

This module is the single source of truth for the wire protocol shared with
the GrafitoCANStepper_C3 firmware:

* CAN ID layout: 11-bit standard ID = ``(node_id << 6) | msg_id``.
  ``node_id`` 1–31 addresses one board, 0 broadcasts to every board.
  ``msg_id`` 0–31 are commands (host → node), 32–63 telemetry (node → host).
  A remote (RTR) frame carrying a telemetry msg_id asks the node to transmit
  that telemetry immediately.
* All multi-byte values are little-endian.
* Serial bridge line format (USB CDC, 115200 baud):
  ``"<CAN_ID hex> <RTR 0|1> <payload hex>\\n"`` — payload length is implied
  by the hex digit count. Lines starting with ``#`` are debug output.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional, Union

PROTOCOL_VERSION = 1
BROADCAST = 0
MAX_NODE_ID = 31


class Cmd(IntEnum):
    """Command message IDs (host -> node, msg_id 0-31)."""

    PING = 0            # no payload; node answers with STATUS
    ESTOP = 1           # no payload; instant stop + driver disable (latched)
    STOP = 2            # no payload; ramped stop, driver stays enabled
    ENABLE = 3          # u8 (0/1); enabling clears an e-stop latch
    MOVE_ABS = 4        # f64 target degrees
    MOVE_REL = 5        # f64 delta degrees
    MOVE_VEL = 6        # f32 signed deg/s
    SET_ZERO = 7        # no payload; stop and define zero here
    HOME = 8            # u8 method, i8 direction, f32 speed deg/s
    SET_PARAM = 9       # u8 param_id, 4-byte value; node answers with PARAM
    GET_PARAM = 10      # u8 param_id; node answers with PARAM
    SAVE_CONFIG = 11    # persist the parameter table to NVS
    LOAD_DEFAULTS = 12  # factory defaults (node keeps its ID)
    FOLLOW = 13         # u8 en, u8 leader, u8 flags, u8 rsvd, f32 ratio
    FOLLOW_SYNC = 14    # recapture the leader/local zero offset
    SET_POSITION = 15   # f64 logical angle here; shaft does not move
    LUT = 16            # u8 action: 0 cal, 1 enable, 2 disable, 3 clear (fw ≥1.10)


class Tel(IntEnum):
    """Telemetry message IDs (node -> host, msg_id 32-63)."""

    STATUS = 32         # u8 flags, u8 mode, u8 fault, u8 fw_major, u8 fw_minor,
                        # u8 proto_version, u8 node_id
    POSITION = 33       # f64 encoder angle in degrees (drives followers)
    MOTION = 34         # f32 measured velocity deg/s, f32 position error deg
    TARGET = 35         # f64 active target degrees
    EVENT = 36          # u8 event, u8 detail, f32 data
    DRIVER = 37         # TMC diagnostics (8 bytes, fw ≥1.4):
                        #   u16 stallguard,
                        #   u8 flags_a (uart_ok|otpw|ot|s2ga|s2gb|s2vsa|s2vsb|ola),
                        #   u8 flags_b (olb|t120|t143|t150|t157|stealth|standstill),
                        #   u8 gstat (reset|drv_err|uv_cp),
                        #   u8 cs_actual (0..31),
                        #   u16 interstep (TSTEP-style)
    ENV = 38            # f32 mcu temperature C, f32 bus voltage V
    PARAM = 39          # u8 param_id, u8 status, 4-byte value
    FOLLOW_STATUS = 40  # u8 en, u8 leader, u8 flags, u8 synced, f32 ratio
    CAN_HEALTH = 41     # u8 state, u8 tx_err, u8 rx_err, u8 recoveries,
                        # u16 tx_failed, u16 bus_errors
    ENC_COUNTS = 42     # i64 multi-turn encoder counts
    PID_STATUS = 43     # u8 state, u8 fault, f32 output deg/s
    LUT_STATUS = 44     # u8 valid, u8 enabled, u16 n, f32 peak_inl_deg (fw ≥1.10)


class Event(IntEnum):
    BOOT = 1
    ENDSTOP_HIT = 2
    ENDSTOP_RELEASED = 3
    STALL = 4
    HOMING_DONE = 5
    HOMING_FAILED = 6
    MOVE_DONE = 7
    ESTOP = 8
    FAULT = 9
    LUT_DONE = 10       # encoder LUT calibration finished; data = peak INL deg
    LUT_FAILED = 11     # LUT calibration failed; detail = reason code


class Mode(IntEnum):
    IDLE = 0
    POSITION = 1
    VELOCITY = 2
    HOMING = 3
    FOLLOW = 4
    LUT = 5             # 200-step encoder LUT calibration in progress (fw ≥1.10)


class Fault(IntEnum):
    NONE = 0
    ENCODER = 1
    NO_PROGRESS = 2
    HOMING_TIMEOUT = 3
    DRIVER_OT = 4        # TMC over-temperature shutdown (OT)
    DRIVER_SHORT = 5     # TMC short-to-GND or low-side short


class HomeMethod(IntEnum):
    SET_ZERO = 0
    ENDSTOP = 1
    STALLGUARD = 2


class EndstopAction(IntEnum):
    REPORT = 0
    STOP = 1
    STOP_AND_ZERO = 2


class StandstillMode(IntEnum):
    NORMAL = 0
    FREEWHEELING = 1
    BRAKING = 2
    STRONG_BRAKING = 3


class ParamStatus(IntEnum):
    OK = 0
    UNKNOWN = 1
    REJECTED = 2


# STATUS flag bits (byte 0 of Tel.STATUS)
FLAG_ENABLED = 0x01
FLAG_MOVING = 0x02
FLAG_HOMED = 0x04
FLAG_ESTOPPED = 0x08
FLAG_ENDSTOP = 0x10
FLAG_STALLED = 0x20
FLAG_ENCODER_OK = 0x40


class Param(IntEnum):
    """Parameter IDs for SET_PARAM/GET_PARAM (see ``PARAMS`` for metadata)."""

    NODE_ID = 1
    STEPS_PER_REV = 2
    MICROSTEPS = 3
    RUN_CURRENT = 4
    HOLD_CURRENT = 5
    STALL_THRESHOLD = 6
    INVERT_DIR = 7
    CLOSED_LOOP = 8
    MAX_SPEED = 9
    ACCELERATION = 10
    CL_MAX_SPEED = 11
    CL_MAX_ACCEL = 12
    PID_KP = 13
    PID_KI = 14
    PID_KD = 15
    PID_TOLERANCE = 16
    FAST_RATE_HZ = 17
    SLOW_RATE_HZ = 18
    ENABLE_ON_BOOT = 19
    ZERO_ON_BOOT = 20
    STANDSTILL_MODE = 21
    ENDSTOP_ENABLE = 22
    ENDSTOP_ACTIVE_HIGH = 23
    ENDSTOP_ACTION = 24
    HOMING_CURRENT = 25
    HOMING_BACKOFF = 26
    HOMING_TIMEOUT_MS = 27
    STEALTHCHOP = 28
    LUT_ENABLE = 29     # 0/1 apply 200-step encoder LUT (fw ≥1.10; needs a table)
    CL_MAX_JERK = 30    # f32 deg/s^3; 0 = auto amax/0.05s (fw ≥1.11)
    PID_KA = 31         # f32 s; acceleration feedforward v += Ka·a (fw ≥1.11)


@dataclass(frozen=True)
class ParamDef:
    """Metadata for one firmware parameter.

    ``fmt`` is a struct format char: ``"I"`` (uint32) or ``"f"`` (float32).
    ``minv``/``maxv`` mirror the firmware's sanity range — hardware validity
    only, never a motion-policy clamp.
    """

    param: Param
    fmt: str
    default: Union[int, float]
    minv: float
    maxv: float
    doc: str

    @property
    def is_float(self) -> bool:
        return self.fmt == "f"


def _p(param: Param, fmt: str, default, minv, maxv, doc) -> ParamDef:
    return ParamDef(param, fmt, default, minv, maxv, doc)


PARAMS: Dict[Param, ParamDef] = {
    d.param: d
    for d in [
        _p(Param.NODE_ID, "I", 1, 1, 31, "CAN node address"),
        _p(Param.STEPS_PER_REV, "I", 200, 1, 100000, "full steps per motor rev"),
        _p(Param.MICROSTEPS, "I", 16, 1, 256, "TMC microsteps (power of two)"),
        _p(Param.RUN_CURRENT, "I", 30, 1, 100, "run current, percent"),
        _p(Param.HOLD_CURRENT, "I", 15, 0, 100, "standstill current, percent"),
        _p(Param.STALL_THRESHOLD, "I", 10, 0, 255, "StallGuard SGTHRS"),
        _p(Param.INVERT_DIR, "I", 0, 0, 1, "invert physical direction"),
        _p(Param.CLOSED_LOOP, "I", 1, 0, 1, "encoder position loop on/off"),
        _p(Param.MAX_SPEED, "f", 720.0, 0.001, 1e9, "position-move speed, deg/s"),
        _p(Param.ACCELERATION, "f", 1440.0, 0.001, 1e9, "acceleration, deg/s^2"),
        _p(Param.CL_MAX_SPEED, "f", 720.0, 0.001, 1e9, "CL S-curve cruise vmax, deg/s"),
        _p(Param.CL_MAX_ACCEL, "f", 1440.0, 0.001, 1e9, "CL S-curve amax, deg/s^2"),
        _p(Param.PID_KP, "f", 10.0, 0.0, 1e6, "CL tracking Kp (v_ff + Ka·a_ff + Kp·(r−enc))"),
        _p(Param.PID_KI, "f", 0.3, 0.0, 1e6, "CL tracking Ki"),
        _p(Param.PID_KD, "f", 0.35, 0.0, 1e6, "CL D-on-measured-velocity Kd"),
        _p(Param.PID_TOLERANCE, "f", 0.35, 0.001, 360.0, "settle tolerance, deg"),
        _p(Param.FAST_RATE_HZ, "I", 10, 0, 500, "fast telemetry rate (0 = off)"),
        _p(Param.SLOW_RATE_HZ, "I", 1, 0, 500, "slow telemetry rate (0 = off)"),
        _p(Param.ENABLE_ON_BOOT, "I", 1, 0, 1, "enable driver at power-on"),
        _p(Param.ZERO_ON_BOOT, "I", 0, 0, 1, "zero encoder at power-on"),
        _p(Param.STANDSTILL_MODE, "I", 0, 0, 3, "TMC standstill mode"),
        _p(Param.ENDSTOP_ENABLE, "I", 0, 0, 1, "IO8 endstop enabled"),
        _p(Param.ENDSTOP_ACTIVE_HIGH, "I", 0, 0, 1, "endstop polarity (0 = active low)"),
        _p(Param.ENDSTOP_ACTION, "I", 1, 0, 2, "0 report, 1 stop, 2 stop+zero"),
        _p(Param.HOMING_CURRENT, "I", 0, 0, 100, "current during homing (0 = keep)"),
        _p(Param.HOMING_BACKOFF, "f", 2.0, 0.0, 36000.0, "post-trigger backoff, deg"),
        _p(Param.HOMING_TIMEOUT_MS, "I", 30000, 100, 600000, "homing timeout, ms"),
        _p(Param.STEALTHCHOP, "I", 1, 0, 1, "StealthChop quiet mode"),
        _p(Param.LUT_ENABLE, "I", 0, 0, 1, "apply 200-step MT6701 LUT (fw ≥1.10)"),
        _p(Param.CL_MAX_JERK, "f", 0.0, 0.0, 1e9, "CL S-curve jerk; 0 = auto (fw ≥1.11)"),
        _p(Param.PID_KA, "f", 0.04, 0.0, 1.0, "CL acceleration feedforward gain, seconds"),
    ]
}


class LutAction(IntEnum):
    """Payload byte 0 of ``Cmd.LUT``."""

    CALIBRATE = 0
    ENABLE = 1
    DISABLE = 2
    CLEAR = 3

PARAMS_BY_NAME: Dict[str, ParamDef] = {d.param.name.lower(): d for d in PARAMS.values()}


def resolve_param(param: Union[Param, int, str]) -> ParamDef:
    """Accept a Param, a numeric ID, or a case-insensitive name."""
    if isinstance(param, str):
        try:
            return PARAMS_BY_NAME[param.lower()]
        except KeyError:
            raise KeyError(f"unknown parameter name: {param!r}") from None
    p = Param(param)
    return PARAMS[p]


def encode_param_value(d: ParamDef, value: Union[int, float, bool]) -> bytes:
    if d.is_float:
        return struct.pack("<f", float(value))
    return struct.pack("<I", int(value))


def decode_param_value(d: ParamDef, raw: bytes) -> Union[int, float]:
    if d.is_float:
        return struct.unpack("<f", raw[:4])[0]
    return struct.unpack("<I", raw[:4])[0]


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------

def make_can_id(node_id: int, msg_id: int) -> int:
    if not 0 <= node_id <= MAX_NODE_ID:
        raise ValueError(f"node_id must be 0-{MAX_NODE_ID}, got {node_id}")
    if not 0 <= msg_id <= 63:
        raise ValueError(f"msg_id must be 0-63, got {msg_id}")
    return (node_id << 6) | msg_id


def split_can_id(can_id: int) -> tuple:
    """Return ``(node_id, msg_id)``."""
    return (can_id >> 6) & 0x1F, can_id & 0x3F


@dataclass(frozen=True)
class Frame:
    """One CAN frame as seen on the bus / serial bridge."""

    can_id: int
    rtr: bool = False
    data: bytes = b""

    @property
    def node_id(self) -> int:
        return (self.can_id >> 6) & 0x1F

    @property
    def msg_id(self) -> int:
        return self.can_id & 0x3F

    @staticmethod
    def command(node_id: int, cmd: int, data: bytes = b"") -> "Frame":
        return Frame(make_can_id(node_id, int(cmd)), False, data)

    @staticmethod
    def rtr_request(node_id: int, tel: int) -> "Frame":
        return Frame(make_can_id(node_id, int(tel)), True, b"")

    def __repr__(self) -> str:
        kind = "RTR" if self.rtr else self.data.hex().upper() or "-"
        return f"<Frame node={self.node_id} msg={self.msg_id} {kind}>"


def encode_line(frame: Frame) -> bytes:
    """Serialize a frame to one serial-bridge text line."""
    payload = "" if frame.rtr else frame.data[:8].hex().upper()
    return f"{frame.can_id:X} {1 if frame.rtr else 0} {payload}\n".encode("ascii")


def decode_line(line: Union[str, bytes]) -> Optional[Frame]:
    """Parse one serial-bridge line; returns None for debug/garbled lines.

    USB-JTAG often drops characters, producing mashed tokens that used to
    parse as ghost node IDs (``100 5100000``). Require an exact ``0|1``
    RTR field and an even-length hex payload of at most 8 bytes.
    """
    if isinstance(line, bytes):
        line = line.decode("ascii", errors="replace")
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split()
    if len(parts) < 2:
        return None
    if parts[1] not in ("0", "1"):
        return None
    payload = parts[2] if len(parts) >= 3 else ""
    if payload and (len(payload) % 2 or len(payload) > 16):
        return None
    try:
        can_id = int(parts[0], 16)
        if not 0 <= can_id <= 0x7FF:
            return None
        data = bytes.fromhex(payload) if payload else b""
    except ValueError:
        return None
    return Frame(can_id, parts[1] == "1", data)
