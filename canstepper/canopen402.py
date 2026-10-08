"""Grafito CANStepper CANopen / CiA 402 reference stack.

This module is the host-side source of truth for the CANopen firmware in
``firmware/GrafitoCANStepper_C3_CANopen/``. It provides:

* Object-dictionary constants matching the EDS / DCF
* NMT / SDO / PDO / heartbeat / EMCY codecs
* A CiA 402 drive state machine
* An in-process slave used by the end-to-end tests
* EDS, DCF, and object-dictionary (JSON/CSV) renderers for PLC import

The on-wire protocol is CiA 301 + a CiA 402 subset (pp / pv / hm). It is
**not** GCSP and must not share a bus with GCSP nodes.
"""

from __future__ import annotations

import csv
import io
import json
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Identity (keep in sync with firmware + EDS)
# ---------------------------------------------------------------------------

VENDOR_ID = 0x000005A3  # placeholder until a CiA vendor ID is registered
PRODUCT_CODE = 0x00004333
REVISION = 0x00020000  # firmware 2.0.0
SERIAL_NUMBER = 0x00000001
DEVICE_TYPE = 0x00040192  # stepper + CiA 402
SUPPORTED_MODES = 0x00000025  # bit0 pp, bit2 pv, bit5 hm
DEVICE_NAME = "CANStepper"
HW_VERSION = "C3"
SW_VERSION = "2.0.0"
PRODUCT_NAME = "Grafito CANStepper C3"
VENDOR_NAME = "Grafito Innovations"
ENC_CPR = 16384
DEFAULT_NODE_ID = 1
DEFAULT_HEARTBEAT_MS = 1000
DEFAULT_BITRATE = 1_000_000

EDS_FILENAME = "GrafitoCANStepper.eds"
DCF_FILENAME = "GrafitoCANStepper_Node1.dcf"
OD_JSON_FILENAME = "object_dictionary.json"
OD_CSV_FILENAME = "object_dictionary.csv"


def _canopen_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "firmware" / "GrafitoCANStepper_C3_CANopen"


def eds_path() -> Path:
    return _canopen_dir() / EDS_FILENAME


def dcf_path() -> Path:
    return _canopen_dir() / DCF_FILENAME


def od_json_path() -> Path:
    return _canopen_dir() / OD_JSON_FILENAME


def od_csv_path() -> Path:
    return _canopen_dir() / OD_CSV_FILENAME


# ---------------------------------------------------------------------------
# CAN identifiers
# ---------------------------------------------------------------------------

COB_NMT = 0x000
COB_SYNC = 0x080


def cob_emcy(node_id: int) -> int:
    return 0x080 + node_id


def cob_tpdo1(node_id: int) -> int:
    return 0x180 + node_id


def cob_rpdo1(node_id: int) -> int:
    return 0x200 + node_id


def cob_tpdo2(node_id: int) -> int:
    return 0x280 + node_id


def cob_rpdo2(node_id: int) -> int:
    return 0x300 + node_id


def cob_tsdo(node_id: int) -> int:
    return 0x580 + node_id


def cob_rsdo(node_id: int) -> int:
    return 0x600 + node_id


def cob_heartbeat(node_id: int) -> int:
    return 0x700 + node_id


# ---------------------------------------------------------------------------
# NMT / SDO / 402
# ---------------------------------------------------------------------------

class NmtCommand(IntEnum):
    START = 0x01
    STOP = 0x02
    ENTER_PREOP = 0x80
    RESET_NODE = 0x81
    RESET_COMM = 0x82


class NmtState(IntEnum):
    BOOTUP = 0x00
    STOPPED = 0x04
    OPERATIONAL = 0x05
    PRE_OPERATIONAL = 0x7F


class DriveState(IntEnum):
    NOT_READY = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION = 6
    FAULT = 7


class OpMode(IntEnum):
    NO_MODE = 0
    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 3
    HOMING = 6


class HomingMethod(IntEnum):
    NONE = 0
    ENDSTOP_NEG = 17
    ENDSTOP_POS = 18
    CURRENT_POSITION = 35
    STALL_NEG = -1
    STALL_POS = -2


# Statusword low 7 bits (CiA 402 Table 19)
SW_NOT_READY = 0x0000
SW_SWITCH_ON_DISABLED = 0x0040
SW_READY_TO_SWITCH_ON = 0x0021
SW_SWITCHED_ON = 0x0023
SW_OPERATION_ENABLED = 0x0027
SW_QUICK_STOP_ACTIVE = 0x0007
SW_FAULT_REACTION = 0x000F
SW_FAULT = 0x0008
SW_VOLTAGE_ENABLED = 0x0010
SW_REMOTE = 0x0200
SW_TARGET_REACHED = 0x0400
SW_INTERNAL_LIMIT = 0x0800
SW_SP_ACK_HOMED = 0x1000  # pp set-point ack / hm homing attained
SW_FOLLOWING_ERR = 0x2000

# Controlword bits
CW_SWITCH_ON = 0x0001
CW_ENABLE_VOLTAGE = 0x0002
CW_QUICK_STOP = 0x0004
CW_ENABLE_OP = 0x0008
CW_NEW_SETPOINT = 0x0010
CW_CHANGE_IMMEDIATE = 0x0020
CW_RELATIVE = 0x0040
CW_FAULT_RESET = 0x0080
CW_HALT = 0x0100

SDO_ABORT_NO_OBJECT = 0x06020000
SDO_ABORT_NO_SUB = 0x06090011
SDO_ABORT_RO = 0x06010002
SDO_ABORT_WO = 0x06010001
SDO_ABORT_LENGTH = 0x06070010
SDO_ABORT_VALUE = 0x06090030
SDO_ABORT_LOCAL = 0x08000021
SDO_ABORT_TOGGLE = 0x05030000

ERR_NONE = 0x0000
ERR_SHORT = 0x2310
ERR_TEMP = 0x4310
ERR_ENCODER = 0x7305
ERR_BLOCKED = 0x7122
ERR_FOLLOWING = 0x8611
ERR_HOMING = 0xFF01

EREG_GENERIC = 0x01
EREG_CURRENT = 0x02
EREG_TEMP = 0x08
EREG_PROFILE = 0x20
EREG_MANUFACTURER = 0x80


_STATE_STATUS = {
    DriveState.NOT_READY: SW_NOT_READY,
    DriveState.SWITCH_ON_DISABLED: SW_SWITCH_ON_DISABLED,
    DriveState.READY_TO_SWITCH_ON: SW_READY_TO_SWITCH_ON,
    DriveState.SWITCHED_ON: SW_SWITCHED_ON,
    DriveState.OPERATION_ENABLED: SW_OPERATION_ENABLED,
    DriveState.QUICK_STOP_ACTIVE: SW_QUICK_STOP_ACTIVE,
    DriveState.FAULT_REACTION: SW_FAULT_REACTION,
    DriveState.FAULT: SW_FAULT,
}


def statusword_for(state: DriveState, extra: int = 0) -> int:
    sw = _STATE_STATUS[state] | SW_REMOTE
    if state in (
        DriveState.READY_TO_SWITCH_ON,
        DriveState.SWITCHED_ON,
        DriveState.OPERATION_ENABLED,
        DriveState.QUICK_STOP_ACTIVE,
    ):
        sw |= SW_VOLTAGE_ENABLED
    return sw | extra


def classify_controlword(cw: int) -> str:
    """Return the CiA 402 command name encoded in ``cw`` (bits 0-3, 7)."""
    if (cw & CW_FAULT_RESET) == 0:
        if (cw & 0x0087) == 0x0006:
            return "shutdown"
        if (cw & 0x008F) == 0x000F:
            return "enable_operation"
        if (cw & 0x008F) == 0x0007:
            return "switch_on"  # also "disable_operation" depending on state
        if (cw & 0x0086) == 0x0002:
            return "quick_stop"
        if (cw & 0x0082) == 0x0000:
            return "disable_voltage"
    return "none"


# ---------------------------------------------------------------------------
# Object dictionary description
# ---------------------------------------------------------------------------

class DType(IntEnum):
    I8 = 0x0002
    I16 = 0x0003
    I32 = 0x0004
    U8 = 0x0005
    U16 = 0x0006
    U32 = 0x0007
    R32 = 0x0008
    VIS = 0x0009


_DTYPE_SIZE = {
    DType.I8: 1,
    DType.I16: 2,
    DType.I32: 4,
    DType.U8: 1,
    DType.U16: 2,
    DType.U32: 4,
    DType.R32: 4,
}


@dataclass(frozen=True)
class OdEntry:
    index: int
    sub: int
    name: str
    dtype: DType
    access: str  # ro / rw / wo
    default: object
    pdo: bool = False
    minimum: Optional[object] = None
    maximum: Optional[object] = None


def _od_table() -> List[OdEntry]:
    e = OdEntry
    return [
        e(0x1000, 0, "Device Type", DType.U32, "ro", DEVICE_TYPE),
        e(0x1001, 0, "Error Register", DType.U8, "ro", 0),
        e(0x1005, 0, "COB-ID SYNC", DType.U32, "rw", COB_SYNC),
        e(0x1008, 0, "Manufacturer Device Name", DType.VIS, "ro", DEVICE_NAME),
        e(0x1009, 0, "Manufacturer Hardware Version", DType.VIS, "ro", HW_VERSION),
        e(0x100A, 0, "Manufacturer Software Version", DType.VIS, "ro", SW_VERSION),
        e(0x1014, 0, "COB-ID EMCY", DType.U32, "ro", 0x80 + DEFAULT_NODE_ID),
        e(0x1017, 0, "Producer Heartbeat Time", DType.U16, "rw", DEFAULT_HEARTBEAT_MS,
          minimum=0, maximum=65535),
        e(0x1018, 0, "Identity Number of Entries", DType.U8, "ro", 4),
        e(0x1018, 1, "Vendor ID", DType.U32, "ro", VENDOR_ID),
        e(0x1018, 2, "Product Code", DType.U32, "ro", PRODUCT_CODE),
        e(0x1018, 3, "Revision Number", DType.U32, "ro", REVISION),
        e(0x1018, 4, "Serial Number", DType.U32, "ro", SERIAL_NUMBER),
        e(0x1200, 0, "SDO Server Number of Entries", DType.U8, "ro", 2),
        e(0x1200, 1, "COB-ID Client to Server", DType.U32, "ro", cob_rsdo(DEFAULT_NODE_ID)),
        e(0x1200, 2, "COB-ID Server to Client", DType.U32, "ro", cob_tsdo(DEFAULT_NODE_ID)),
        e(0x1400, 0, "RPDO1 Comm Number of Entries", DType.U8, "ro", 2),
        e(0x1400, 1, "RPDO1 COB-ID", DType.U32, "rw", cob_rpdo1(DEFAULT_NODE_ID)),
        e(0x1400, 2, "RPDO1 Transmission Type", DType.U8, "rw", 255),
        e(0x1401, 0, "RPDO2 Comm Number of Entries", DType.U8, "ro", 2),
        e(0x1401, 1, "RPDO2 COB-ID", DType.U32, "rw", cob_rpdo2(DEFAULT_NODE_ID)),
        e(0x1401, 2, "RPDO2 Transmission Type", DType.U8, "rw", 255),
        e(0x1600, 0, "RPDO1 Map Number of Entries", DType.U8, "ro", 2),
        e(0x1600, 1, "RPDO1 Mapping 1", DType.U32, "ro", 0x60400010),
        e(0x1600, 2, "RPDO1 Mapping 2", DType.U32, "ro", 0x607A0020),
        e(0x1601, 0, "RPDO2 Map Number of Entries", DType.U8, "ro", 2),
        e(0x1601, 1, "RPDO2 Mapping 1", DType.U32, "ro", 0x60400010),
        e(0x1601, 2, "RPDO2 Mapping 2", DType.U32, "ro", 0x60FF0020),
        e(0x1800, 0, "TPDO1 Comm Number of Entries", DType.U8, "ro", 2),
        e(0x1800, 1, "TPDO1 COB-ID", DType.U32, "rw", cob_tpdo1(DEFAULT_NODE_ID)),
        e(0x1800, 2, "TPDO1 Transmission Type", DType.U8, "rw", 255),
        e(0x1801, 0, "TPDO2 Comm Number of Entries", DType.U8, "ro", 2),
        e(0x1801, 1, "TPDO2 COB-ID", DType.U32, "rw", cob_tpdo2(DEFAULT_NODE_ID)),
        e(0x1801, 2, "TPDO2 Transmission Type", DType.U8, "rw", 255),
        e(0x1A00, 0, "TPDO1 Map Number of Entries", DType.U8, "ro", 2),
        e(0x1A00, 1, "TPDO1 Mapping 1", DType.U32, "ro", 0x60410010),
        e(0x1A00, 2, "TPDO1 Mapping 2", DType.U32, "ro", 0x60640020),
        e(0x1A01, 0, "TPDO2 Map Number of Entries", DType.U8, "ro", 2),
        e(0x1A01, 1, "TPDO2 Mapping 1", DType.U32, "ro", 0x60410010),
        e(0x1A01, 2, "TPDO2 Mapping 2", DType.U32, "ro", 0x606C0020),
        e(0x603F, 0, "Error Code", DType.U16, "ro", 0),
        e(0x6040, 0, "Controlword", DType.U16, "rw", 0, pdo=True),
        e(0x6041, 0, "Statusword", DType.U16, "ro", SW_SWITCH_ON_DISABLED | SW_REMOTE, pdo=True),
        e(0x605A, 0, "Quick Stop Option Code", DType.I16, "rw", 2),
        e(0x6060, 0, "Modes of Operation", DType.I8, "rw", OpMode.PROFILE_POSITION, pdo=True),
        e(0x6061, 0, "Modes of Operation Display", DType.I8, "ro", OpMode.PROFILE_POSITION, pdo=True),
        e(0x6064, 0, "Position Actual Value", DType.I32, "ro", 0, pdo=True),
        e(0x606C, 0, "Velocity Actual Value", DType.I32, "ro", 0, pdo=True),
        e(0x607A, 0, "Target Position", DType.I32, "rw", 0, pdo=True),
        e(0x6081, 0, "Profile Velocity", DType.U32, "rw", 32768, minimum=1),  # counts/s (~720 deg/s)
        e(0x6083, 0, "Profile Acceleration", DType.U32, "rw", 131072, minimum=1),
        e(0x6084, 0, "Profile Deceleration", DType.U32, "rw", 131072, minimum=1),
        e(0x6085, 0, "Quick Stop Deceleration", DType.U32, "rw", 262144, minimum=1),
        e(0x6098, 0, "Homing Method", DType.I8, "rw", HomingMethod.CURRENT_POSITION),
        e(0x6099, 0, "Homing Speeds Number of Entries", DType.U8, "ro", 2),
        e(0x6099, 1, "Speed During Search for Switch", DType.U32, "rw", 2275),  # ~50 deg/s
        e(0x6099, 2, "Speed During Search for Zero", DType.U32, "rw", 455),
        e(0x609A, 0, "Homing Acceleration", DType.U32, "rw", 131072, minimum=1),
        e(0x60FF, 0, "Target Velocity", DType.I32, "rw", 0, pdo=True),
        e(0x6502, 0, "Supported Drive Modes", DType.U32, "ro", SUPPORTED_MODES),
        e(0x2000, 0, "Node ID", DType.U8, "rw", DEFAULT_NODE_ID, minimum=1, maximum=127),
        e(0x2001, 0, "Steps Per Revolution", DType.U32, "rw", 200, minimum=1, maximum=100000),
        e(0x2002, 0, "Microsteps", DType.U32, "rw", 16, minimum=1, maximum=256),
        e(0x2003, 0, "Run Current Percent", DType.U8, "rw", 20, minimum=1, maximum=100),
        e(0x2004, 0, "Hold Current Percent", DType.U8, "rw", 5, minimum=0, maximum=100),
        e(0x2005, 0, "Stall Threshold", DType.U8, "rw", 10, maximum=255),
        e(0x2006, 0, "Invert Direction", DType.U8, "rw", 0, maximum=1),
        e(0x2007, 0, "Closed Loop Enable", DType.U8, "rw", 1, maximum=1),
        e(0x2008, 0, "Save Config", DType.U8, "wo", 0),
        e(0x2009, 0, "Load Defaults", DType.U8, "wo", 0),
        e(0x200A, 0, "StealthChop Enable", DType.U8, "rw", 1, maximum=1),
        e(0x200B, 0, "Standstill Mode", DType.U8, "rw", 1, maximum=3),
        e(0x200C, 0, "Endstop Enable", DType.U8, "rw", 1, maximum=1),
        e(0x200D, 0, "Endstop Active High", DType.U8, "rw", 1, maximum=1),
        e(0x200E, 0, "CAN Bitrate", DType.U32, "rw", DEFAULT_BITRATE),
        e(0x200F, 0, "PID Kp", DType.R32, "rw", 12.0),
        e(0x2010, 0, "PID Ki", DType.R32, "rw", 0.3),
        e(0x2011, 0, "PID Kd", DType.R32, "rw", 0.10),
        e(0x2012, 0, "PID Tolerance Deg", DType.R32, "rw", 0.35),
        e(0x2013, 0, "Enable On Boot", DType.U8, "rw", 0, maximum=1),
        e(0x2014, 0, "Zero On Boot", DType.U8, "rw", 0, maximum=1),
        e(0x2015, 0, "Homing Timeout ms", DType.U32, "rw", 30000, minimum=100, maximum=600000),
        e(0x2016, 0, "Firmware Version", DType.U16, "ro", 0x0200),
        e(0x2017, 0, "Encoder OK", DType.U8, "ro", 1),
        e(0x2018, 0, "Endstop Active", DType.U8, "ro", 0),
        e(0x2019, 0, "Bus Voltage", DType.R32, "ro", 24.0),
    ]


OD_ENTRIES: Dict[Tuple[int, int], OdEntry] = {(x.index, x.sub): x for x in _od_table()}


def od_entry(index: int, sub: int = 0) -> OdEntry:
    try:
        return OD_ENTRIES[(index, sub)]
    except KeyError as exc:
        raise KeyError(f"0x{index:04X}:{sub:02X} is not in the object dictionary") from exc


# ---------------------------------------------------------------------------
# Pack / unpack
# ---------------------------------------------------------------------------

def pack_value(dtype: DType, value: object) -> bytes:
    if dtype == DType.VIS:
        return str(value).encode("ascii")
    if dtype == DType.U8:
        return struct.pack("<B", int(value) & 0xFF)
    if dtype == DType.I8:
        return struct.pack("<b", int(value))
    if dtype == DType.U16:
        return struct.pack("<H", int(value) & 0xFFFF)
    if dtype == DType.I16:
        return struct.pack("<h", int(value))
    if dtype == DType.U32:
        return struct.pack("<I", int(value) & 0xFFFFFFFF)
    if dtype == DType.I32:
        return struct.pack("<i", int(value))
    if dtype == DType.R32:
        return struct.pack("<f", float(value))
    raise TypeError(dtype)


def unpack_value(dtype: DType, raw: bytes) -> object:
    if dtype == DType.VIS:
        return raw.split(b"\x00", 1)[0].decode("ascii")
    need = _DTYPE_SIZE[dtype]
    if len(raw) < need:
        raw = raw + bytes(need - len(raw))
    if dtype == DType.U8:
        return struct.unpack("<B", raw[:1])[0]
    if dtype == DType.I8:
        return struct.unpack("<b", raw[:1])[0]
    if dtype == DType.U16:
        return struct.unpack("<H", raw[:2])[0]
    if dtype == DType.I16:
        return struct.unpack("<h", raw[:2])[0]
    if dtype == DType.U32:
        return struct.unpack("<I", raw[:4])[0]
    if dtype == DType.I32:
        return struct.unpack("<i", raw[:4])[0]
    if dtype == DType.R32:
        return struct.unpack("<f", raw[:4])[0]
    raise TypeError(dtype)


def encode_sdo_expedited_download(index: int, sub: int, data: bytes) -> bytes:
    n = 4 - len(data)
    if n < 0 or n > 3:
        raise ValueError("expedited SDO download supports 1-4 bytes")
    cmd = 0x23 | (n << 2)
    return bytes((cmd, index & 0xFF, (index >> 8) & 0xFF, sub)) + data.ljust(4, b"\x00")


def encode_sdo_upload_request(index: int, sub: int) -> bytes:
    return bytes((0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0))


def encode_sdo_segment_request(toggle: int) -> bytes:
    return bytes((0x60 | ((toggle & 1) << 4), 0, 0, 0, 0, 0, 0, 0))


def parse_sdo(payload: bytes) -> Tuple[int, int, int, bytes]:
    if len(payload) < 4:
        raise ValueError("SDO payload too short")
    pad = payload + bytes(8 - len(payload)) if len(payload) < 8 else payload
    cmd, idx_lo, idx_hi, sub = pad[0], pad[1], pad[2], pad[3]
    return cmd, idx_lo | (idx_hi << 8), sub, pad[4:8]


def sdo_abort_payload(index: int, sub: int, code: int) -> bytes:
    return bytes((0x80, index & 0xFF, (index >> 8) & 0xFF, sub)) + struct.pack("<I", code)


def encode_nmt(command: int, node_id: int) -> Tuple[int, bytes]:
    return COB_NMT, bytes((command & 0xFF, node_id & 0xFF))


def encode_rpdo1(controlword: int, target_position: int) -> bytes:
    return struct.pack("<Hi", controlword & 0xFFFF, int(target_position)) + b"\x00\x00"


def encode_rpdo2(controlword: int, target_velocity: int) -> bytes:
    return struct.pack("<Hi", controlword & 0xFFFF, int(target_velocity)) + b"\x00\x00"


def decode_tpdo1(data: bytes) -> Tuple[int, int]:
    raw = data[:6].ljust(6, b"\x00")
    sw, pos = struct.unpack("<Hi", raw)
    return sw, pos


def decode_tpdo2(data: bytes) -> Tuple[int, int]:
    raw = data[:6].ljust(6, b"\x00")
    sw, vel = struct.unpack("<Hi", raw)
    return sw, vel


def counts_from_deg(deg: float) -> int:
    return int(round(float(deg) * ENC_CPR / 360.0))


def deg_from_counts(counts: int) -> float:
    return float(counts) * 360.0 / ENC_CPR


# ---------------------------------------------------------------------------
# CiA 402 state machine
# ---------------------------------------------------------------------------

class Cia402StateMachine:
    """CiA 402 drive FSA used by both the Python slave and the firmware spec."""

    def __init__(self) -> None:
        self.state = DriveState.SWITCH_ON_DISABLED
        self.controlword = 0
        self._prev_cw = 0
        self.target_reached = False
        self.setpoint_ack = False
        self.homing_attained = False
        self.following_error = False
        self.warning = False

    def reset(self) -> None:
        self.state = DriveState.SWITCH_ON_DISABLED
        self.controlword = 0
        self._prev_cw = 0
        self.target_reached = False
        self.setpoint_ack = False
        self.homing_attained = False
        self.following_error = False

    def statusword(self) -> int:
        extra = 0
        if self.target_reached:
            extra |= SW_TARGET_REACHED
        if self.setpoint_ack or self.homing_attained:
            extra |= SW_SP_ACK_HOMED
        if self.following_error:
            extra |= SW_FOLLOWING_ERR
        if self.warning:
            extra |= 0x0080
        return statusword_for(self.state, extra)

    def raise_fault(self) -> None:
        self.state = DriveState.FAULT
        self.setpoint_ack = False

    def apply_controlword(self, cw: int) -> List[str]:
        """Apply a controlword. Returns a list of side-effect names for the app."""
        actions: List[str] = []
        rising_fault_reset = (cw & CW_FAULT_RESET) and not (self._prev_cw & CW_FAULT_RESET)
        cmd = classify_controlword(cw)
        prev = self.state

        if self.state == DriveState.FAULT and rising_fault_reset:
            self.state = DriveState.SWITCH_ON_DISABLED
            actions.append("fault_reset")
        elif self.state == DriveState.SWITCH_ON_DISABLED:
            if cmd == "shutdown":
                self.state = DriveState.READY_TO_SWITCH_ON
        elif self.state == DriveState.READY_TO_SWITCH_ON:
            if cmd == "switch_on":
                self.state = DriveState.SWITCHED_ON
            elif cmd == "disable_voltage":
                self.state = DriveState.SWITCH_ON_DISABLED
            elif cmd == "quick_stop":
                self.state = DriveState.SWITCH_ON_DISABLED
        elif self.state == DriveState.SWITCHED_ON:
            if cmd == "enable_operation":
                self.state = DriveState.OPERATION_ENABLED
                actions.append("enable")
            elif cmd == "shutdown":
                self.state = DriveState.READY_TO_SWITCH_ON
            elif cmd in ("disable_voltage", "quick_stop"):
                self.state = DriveState.SWITCH_ON_DISABLED
                actions.append("disable")
        elif self.state == DriveState.OPERATION_ENABLED:
            if cmd == "switch_on":  # disable operation
                self.state = DriveState.SWITCHED_ON
                actions.append("disable_operation")
            elif cmd == "shutdown":
                self.state = DriveState.READY_TO_SWITCH_ON
                actions.append("disable")
            elif cmd == "disable_voltage":
                self.state = DriveState.SWITCH_ON_DISABLED
                actions.append("disable")
            elif cmd == "quick_stop":
                self.state = DriveState.QUICK_STOP_ACTIVE
                actions.append("quick_stop")
        elif self.state == DriveState.QUICK_STOP_ACTIVE:
            if cmd in ("disable_voltage", "shutdown"):
                self.state = DriveState.SWITCH_ON_DISABLED
                actions.append("disable")
            elif cmd == "enable_operation":
                self.state = DriveState.OPERATION_ENABLED
                actions.append("enable")

        if prev != DriveState.OPERATION_ENABLED and self.state == DriveState.OPERATION_ENABLED:
            if "enable" not in actions:
                actions.append("enable")
        if prev == DriveState.OPERATION_ENABLED and self.state != DriveState.OPERATION_ENABLED:
            if not any(a in actions for a in ("disable", "disable_operation", "quick_stop")):
                actions.append("disable_operation")

        self._prev_cw = self.controlword
        self.controlword = cw
        return actions

    def new_setpoint_rising(self) -> bool:
        return bool(self.controlword & CW_NEW_SETPOINT) and not (self._prev_cw & CW_NEW_SETPOINT)

    def halt_active(self) -> bool:
        return bool(self.controlword & CW_HALT)

    def relative_setpoint(self) -> bool:
        return bool(self.controlword & CW_RELATIVE)


# ---------------------------------------------------------------------------
# CAN frame + virtual bus
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes = b""
    rtr: bool = False


class VirtualCanBus:
    """In-process 11-bit CAN bus used by the end-to-end tests."""

    def __init__(self) -> None:
        self._rx: List[CanFrame] = []
        self.slaves: List["CanOpenSlave"] = []

    def attach(self, slave: "CanOpenSlave") -> None:
        slave.attach_bus(self)
        self.slaves.append(slave)

    def send(self, frame: CanFrame) -> None:
        for slave in self.slaves:
            slave.handle(frame)

    def emit(self, frame: CanFrame) -> None:
        self._rx.append(frame)

    def drain(self) -> List[CanFrame]:
        out = list(self._rx)
        self._rx.clear()
        return out

    def take(self, can_id: int) -> List[CanFrame]:
        keep: List[CanFrame] = []
        hit: List[CanFrame] = []
        for fr in self._rx:
            if fr.can_id == can_id:
                hit.append(fr)
            else:
                keep.append(fr)
        self._rx = keep
        return hit


# ---------------------------------------------------------------------------
# Slave
# ---------------------------------------------------------------------------

class CanOpenSlave:
    """Software CiA 402 node matching the Grafito CANopen firmware."""

    def __init__(self, node_id: int = DEFAULT_NODE_ID, bus: Optional[VirtualCanBus] = None):
        if not 1 <= node_id <= 127:
            raise ValueError("CANopen node id must be 1-127")
        self.node_id = node_id
        self.bus: Optional[VirtualCanBus] = None
        self.nmt = NmtState.PRE_OPERATIONAL
        self.drive = Cia402StateMachine()
        self.values: Dict[Tuple[int, int], object] = {
            (e.index, e.sub): e.default for e in OD_ENTRIES.values()
        }
        self.values[(0x2000, 0)] = node_id
        self.values[(0x1014, 0)] = cob_emcy(node_id)
        self.values[(0x1200, 1)] = cob_rsdo(node_id)
        self.values[(0x1200, 2)] = cob_tsdo(node_id)
        self.values[(0x1400, 1)] = cob_rpdo1(node_id)
        self.values[(0x1401, 1)] = cob_rpdo2(node_id)
        self.values[(0x1800, 1)] = cob_tpdo1(node_id)
        self.values[(0x1801, 1)] = cob_tpdo2(node_id)
        self._nvs = dict(self.values)
        self._hb_last_ms = 0
        self._now_ms = 0
        self._upload: Optional[Tuple[int, int, bytes, int]] = None  # idx, sub, blob, offset
        self._upload_toggle = 0
        self.saved = False
        if bus is not None:
            bus.attach(self)
            self._bootup()

    def attach_bus(self, bus: VirtualCanBus) -> None:
        self.bus = bus

    def _tx(self, can_id: int, data: bytes) -> None:
        if self.bus is not None:
            self.bus.emit(CanFrame(can_id, bytes(data)))

    def _bootup(self) -> None:
        self.nmt = NmtState.PRE_OPERATIONAL
        self._tx(cob_heartbeat(self.node_id), bytes((NmtState.BOOTUP,)))

    def reset_node(self) -> None:
        nid = int(self.values[(0x2000, 0)])
        saved = dict(self._nvs)
        self.__init__(nid, None)  # type: ignore[misc]
        self.values.update(saved)
        self.node_id = int(self.values[(0x2000, 0)])
        self._refresh_cob_ids()
        if self.bus is not None:
            self._bootup()

    def _refresh_cob_ids(self) -> None:
        n = self.node_id
        self.values[(0x1014, 0)] = cob_emcy(n)
        self.values[(0x1200, 1)] = cob_rsdo(n)
        self.values[(0x1200, 2)] = cob_tsdo(n)
        self.values[(0x1400, 1)] = cob_rpdo1(n)
        self.values[(0x1401, 1)] = cob_rpdo2(n)
        self.values[(0x1800, 1)] = cob_tpdo1(n)
        self.values[(0x1801, 1)] = cob_tpdo2(n)

    # -- time ------------------------------------------------------------------

    def tick(self, now_ms: Optional[int] = None) -> None:
        if now_ms is None:
            self._now_ms += 20
        else:
            self._now_ms = now_ms
        hb = int(self.values[(0x1017, 0)])
        if hb > 0 and self.nmt != NmtState.BOOTUP:
            if self._now_ms - self._hb_last_ms >= hb:
                self._hb_last_ms = self._now_ms
                self._tx(cob_heartbeat(self.node_id), bytes((int(self.nmt),)))
        if self.nmt == NmtState.OPERATIONAL:
            self._send_tpdos()
        self.values[(0x6041, 0)] = self.drive.statusword()
        self.values[(0x6061, 0)] = int(self.values[(0x6060, 0)])

    def _send_tpdos(self) -> None:
        sw = self.drive.statusword()
        pos = int(self.values[(0x6064, 0)])
        vel = int(self.values[(0x606C, 0)])
        self._tx(cob_tpdo1(self.node_id), encode_rpdo1(sw, pos))
        self._tx(cob_tpdo2(self.node_id), encode_rpdo2(sw, vel))

    # -- RX --------------------------------------------------------------------

    def handle(self, frame: CanFrame) -> None:
        if frame.rtr:
            return
        cid, data = frame.can_id, frame.data
        if cid == COB_NMT:
            self._on_nmt(data)
            return
        if cid == cob_rsdo(self.node_id):
            if self.nmt in (NmtState.PRE_OPERATIONAL, NmtState.OPERATIONAL):
                self._on_sdo(data)
            return
        if self.nmt != NmtState.OPERATIONAL:
            return
        if cid == cob_rpdo1(self.node_id):
            self._on_rpdo1(data)
        elif cid == cob_rpdo2(self.node_id):
            self._on_rpdo2(data)

    def _on_nmt(self, data: bytes) -> None:
        if len(data) < 2:
            return
        cmd, dest = data[0], data[1]
        if dest not in (0, self.node_id):
            return
        if cmd == NmtCommand.START:
            self.nmt = NmtState.OPERATIONAL
        elif cmd == NmtCommand.STOP:
            self.nmt = NmtState.STOPPED
        elif cmd == NmtCommand.ENTER_PREOP:
            self.nmt = NmtState.PRE_OPERATIONAL
        elif cmd in (NmtCommand.RESET_NODE, NmtCommand.RESET_COMM):
            bus = self.bus
            self.reset_node()
            self.bus = bus
            self._bootup()

    def _on_rpdo1(self, data: bytes) -> None:
        if len(data) < 6:
            return
        cw, pos = decode_tpdo1(data)
        self._write_od(0x607A, 0, pos)
        self._apply_controlword(cw)

    def _on_rpdo2(self, data: bytes) -> None:
        if len(data) < 6:
            return
        cw, vel = decode_tpdo2(data)
        self._write_od(0x60FF, 0, vel)
        self._apply_controlword(cw)

    # -- SDO -------------------------------------------------------------------

    def _on_sdo(self, data: bytes) -> None:
        cmd, index, sub, payload = parse_sdo(data)
        ccs = (cmd >> 5) & 0x7
        if ccs == 1:  # download initiate
            if cmd & 0x02:  # expedited
                n = (cmd >> 2) & 0x3
                raw = payload[: 4 - n] if (cmd & 0x01) else payload
                self._sdo_write(index, sub, raw)
            else:
                self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_LENGTH))
            return
        if ccs == 2:  # upload initiate
            self._sdo_read(index, sub)
            return
        if ccs == 3:  # upload segment
            self._sdo_segment(cmd)
            return
        self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_LOCAL))

    def _sdo_read(self, index: int, sub: int) -> None:
        key = (index, sub)
        if key not in OD_ENTRIES:
            code = SDO_ABORT_NO_SUB if any(i == index for i, _s in OD_ENTRIES) else SDO_ABORT_NO_OBJECT
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, code))
            return
        entry = OD_ENTRIES[key]
        if entry.access == "wo":
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_WO))
            return
        if key == (0x6041, 0):
            self.values[key] = self.drive.statusword()
        raw = pack_value(entry.dtype, self.values[key])
        if entry.dtype == DType.VIS or len(raw) > 4:
            self._upload = (index, sub, raw, 0)
            self._upload_toggle = 0
            hdr = bytes((0x41, index & 0xFF, (index >> 8) & 0xFF, sub))
            self._tx(cob_tsdo(self.node_id), hdr + struct.pack("<I", len(raw)))
            return
        n = 4 - len(raw)
        cmd = 0x43 | (n << 2)
        self._tx(
            cob_tsdo(self.node_id),
            bytes((cmd, index & 0xFF, (index >> 8) & 0xFF, sub)) + raw.ljust(4, b"\x00"),
        )

    def _sdo_segment(self, cmd: int) -> None:
        if self._upload is None:
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(0, 0, SDO_ABORT_TOGGLE))
            return
        toggle = (cmd >> 4) & 1
        if toggle != self._upload_toggle:
            index, sub, _blob, _off = self._upload
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_TOGGLE))
            return
        index, sub, blob, offset = self._upload
        chunk = blob[offset : offset + 7]
        offset += len(chunk)
        last = offset >= len(blob)
        n = 7 - len(chunk)
        resp = (0x00 | (toggle << 4) | (n << 1) | (1 if last else 0))
        self._tx(cob_tsdo(self.node_id), bytes((resp,)) + chunk.ljust(7, b"\x00"))
        self._upload_toggle ^= 1
        if last:
            self._upload = None
        else:
            self._upload = (index, sub, blob, offset)

    def _sdo_write(self, index: int, sub: int, raw: bytes) -> None:
        key = (index, sub)
        if key not in OD_ENTRIES:
            code = SDO_ABORT_NO_SUB if any(i == index for i, _s in OD_ENTRIES) else SDO_ABORT_NO_OBJECT
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, code))
            return
        entry = OD_ENTRIES[key]
        if entry.access == "ro":
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_RO))
            return
        try:
            value = unpack_value(entry.dtype, raw)
        except Exception:
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_LENGTH))
            return
        if not self._validate(entry, value):
            self._tx(cob_tsdo(self.node_id), sdo_abort_payload(index, sub, SDO_ABORT_VALUE))
            return
        self._write_od(index, sub, value)
        self._tx(
            cob_tsdo(self.node_id),
            bytes((0x60, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0)),
        )

    def _validate(self, entry: OdEntry, value: object) -> bool:
        if entry.index == 0x2002 and entry.sub == 0:
            v = int(value)
            return v > 0 and (v & (v - 1)) == 0 and v <= 256
        if entry.index == 0x200E:
            return int(value) in (125_000, 250_000, 500_000, 1_000_000)
        if entry.index == 0x6060:
            return int(value) in (0, 1, 3, 6)
        if entry.minimum is not None and value < entry.minimum:  # type: ignore[operator]
            return False
        if entry.maximum is not None and value > entry.maximum:  # type: ignore[operator]
            return False
        return True

    def _write_od(self, index: int, sub: int, value: object) -> None:
        key = (index, sub)
        if index == 0x2008 and int(value) == 1:
            self._nvs = dict(self.values)
            self.saved = True
            self.values[key] = 0
            return
        if index == 0x2009 and int(value) == 1:
            for e in OD_ENTRIES.values():
                if e.access != "wo":
                    self.values[(e.index, e.sub)] = e.default
            self.values[(0x2000, 0)] = self.node_id
            self._refresh_cob_ids()
            return
        self.values[key] = value
        if index == 0x6040:
            self._apply_controlword(int(value))
        elif index == 0x6060:
            self.values[(0x6061, 0)] = int(value)
        elif index == 0x2000:
            # Applied after save + reset; keep runtime id until then.
            pass

    def _apply_controlword(self, cw: int) -> None:
        actions = self.drive.apply_controlword(cw)
        self.values[(0x6040, 0)] = cw
        self.values[(0x6041, 0)] = self.drive.statusword()
        if "fault_reset" in actions:
            self.values[(0x603F, 0)] = 0
            self.values[(0x1001, 0)] = 0
            self.drive.following_error = False
        if "quick_stop" in actions:
            self.values[(0x606C, 0)] = 0
            self.drive.target_reached = True
        if "disable" in actions or "disable_operation" in actions:
            self.values[(0x606C, 0)] = 0
        if self.drive.state != DriveState.OPERATION_ENABLED:
            return
        mode = int(self.values[(0x6060, 0)])
        if self.drive.halt_active():
            self.values[(0x606C, 0)] = 0
            self.drive.target_reached = True
            return
        if mode == OpMode.PROFILE_POSITION and self.drive.new_setpoint_rising():
            target = int(self.values[(0x607A, 0)])
            if self.drive.relative_setpoint():
                target = int(self.values[(0x6064, 0)]) + target
            self.values[(0x6064, 0)] = target
            self.values[(0x606C, 0)] = 0
            self.drive.target_reached = True
            self.drive.setpoint_ack = True
            self.drive.homing_attained = False
        elif mode == OpMode.PROFILE_VELOCITY:
            vel = int(self.values[(0x60FF, 0)])
            self.values[(0x606C, 0)] = vel
            self.drive.target_reached = vel == 0
            self.drive.setpoint_ack = False
        elif mode == OpMode.HOMING and self.drive.new_setpoint_rising():
            method = int(self.values[(0x6098, 0)])
            if method in (
                int(HomingMethod.CURRENT_POSITION),
                int(HomingMethod.NONE),
                int(HomingMethod.ENDSTOP_NEG),
                int(HomingMethod.ENDSTOP_POS),
                int(HomingMethod.STALL_NEG),
                int(HomingMethod.STALL_POS),
            ):
                self.values[(0x6064, 0)] = 0
                self.values[(0x606C, 0)] = 0
                self.drive.homing_attained = True
                self.drive.target_reached = True
            else:
                self.raise_fault(ERR_HOMING, EREG_PROFILE | EREG_GENERIC)

    def raise_fault(self, code: int, err_reg: int) -> None:
        self.values[(0x603F, 0)] = code
        self.values[(0x1001, 0)] = err_reg
        self.drive.raise_fault()
        self.values[(0x6041, 0)] = self.drive.statusword()
        emcy = struct.pack("<HB", code & 0xFFFF, err_reg & 0xFF) + bytes(5)
        self._tx(cob_emcy(self.node_id), emcy)


# ---------------------------------------------------------------------------
# Master helper (test / PLC-like sequence)
# ---------------------------------------------------------------------------

class CanOpenMaster:
    """Minimal CANopen master for the in-process bus."""

    def __init__(self, bus: VirtualCanBus, node_id: int = DEFAULT_NODE_ID):
        self.bus = bus
        self.node_id = node_id

    def nmt(self, command: int, dest: Optional[int] = None) -> None:
        cid, data = encode_nmt(command, self.node_id if dest is None else dest)
        self.bus.send(CanFrame(cid, data))

    def start(self) -> None:
        self.nmt(NmtCommand.START)

    def preop(self) -> None:
        self.nmt(NmtCommand.ENTER_PREOP)

    def sdo_write(self, index: int, sub: int, value: object, dtype: Optional[DType] = None) -> None:
        entry = od_entry(index, sub)
        raw = pack_value(dtype or entry.dtype, value)
        if entry.dtype == DType.VIS or len(raw) > 4:
            raise ValueError("master helper writes expedited SDOs only")
        self.bus.send(CanFrame(cob_rsdo(self.node_id), encode_sdo_expedited_download(index, sub, raw)))
        replies = self.bus.take(cob_tsdo(self.node_id))
        if not replies:
            raise TimeoutError(f"no SDO write confirm for 0x{index:04X}:{sub:02X}")
        cmd, idx, sb, _payload = parse_sdo(replies[-1].data)
        if cmd == 0x80:
            code = struct.unpack("<I", replies[-1].data[4:8])[0]
            raise RuntimeError(f"SDO abort 0x{code:08X} writing 0x{idx:04X}:{sb:02X}")
        if cmd != 0x60:
            raise RuntimeError(f"unexpected SDO write reply 0x{cmd:02X}")

    def sdo_read(self, index: int, sub: int = 0) -> object:
        entry = od_entry(index, sub)
        self.bus.send(CanFrame(cob_rsdo(self.node_id), encode_sdo_upload_request(index, sub)))
        replies = self.bus.take(cob_tsdo(self.node_id))
        if not replies:
            raise TimeoutError(f"no SDO reply for 0x{index:04X}:{sub:02X}")
        cmd, idx, sb, payload = parse_sdo(replies[-1].data)
        if cmd == 0x80:
            code = struct.unpack("<I", replies[-1].data[4:8])[0]
            raise RuntimeError(f"SDO abort 0x{code:08X} on 0x{idx:04X}:{sb:02X}")
        if (cmd & 0xE0) == 0x40 and (cmd & 0x02):  # expedited upload
            n = (cmd >> 2) & 0x3
            raw = payload[: 4 - n]
            return unpack_value(entry.dtype, raw)
        if cmd == 0x41:
            size = struct.unpack("<I", payload)[0]
            blob = b""
            toggle = 0
            while len(blob) < size:
                self.bus.send(CanFrame(cob_rsdo(self.node_id), encode_sdo_segment_request(toggle)))
                segs = self.bus.take(cob_tsdo(self.node_id))
                if not segs:
                    raise TimeoutError("segmented SDO timed out")
                scmd = segs[-1].data[0]
                n = (scmd >> 1) & 0x7
                last = scmd & 0x01
                chunk = segs[-1].data[1 : 8 - n]
                blob += chunk
                toggle ^= 1
                if last:
                    break
            return unpack_value(entry.dtype, blob)
        raise RuntimeError(f"unexpected SDO cmd 0x{cmd:02X}")

    def enable_operation(self) -> None:
        """Standard CiA 402 enable sequence: shutdown → switch on → enable."""
        self.sdo_write(0x6040, 0, 0x0006)
        self.sdo_write(0x6040, 0, 0x0007)
        self.sdo_write(0x6040, 0, 0x000F)

    def move_absolute(self, counts: int) -> None:
        self.sdo_write(0x6060, 0, int(OpMode.PROFILE_POSITION))
        self.sdo_write(0x607A, 0, counts)
        cw = 0x000F | CW_NEW_SETPOINT
        self.sdo_write(0x6040, 0, cw)
        self.sdo_write(0x6040, 0, 0x000F)  # drop new-setpoint, keep enabled

    def move_relative(self, counts: int) -> None:
        self.sdo_write(0x6060, 0, int(OpMode.PROFILE_POSITION))
        self.sdo_write(0x607A, 0, counts)
        self.sdo_write(0x6040, 0, 0x000F | CW_NEW_SETPOINT | CW_RELATIVE)
        self.sdo_write(0x6040, 0, 0x000F)

    def set_velocity(self, counts_s: int) -> None:
        self.sdo_write(0x6060, 0, int(OpMode.PROFILE_VELOCITY))
        self.sdo_write(0x60FF, 0, counts_s)
        self.sdo_write(0x6040, 0, 0x000F)

    def home(self, method: int = int(HomingMethod.CURRENT_POSITION)) -> None:
        self.sdo_write(0x6060, 0, int(OpMode.HOMING))
        self.sdo_write(0x6098, 0, method)
        self.sdo_write(0x6040, 0, 0x000F | CW_NEW_SETPOINT)
        self.sdo_write(0x6040, 0, 0x000F)

    def send_rpdo1(self, controlword: int, target_position: int) -> None:
        self.bus.send(CanFrame(cob_rpdo1(self.node_id), encode_rpdo1(controlword, target_position)))

    def send_rpdo2(self, controlword: int, target_velocity: int) -> None:
        self.bus.send(CanFrame(cob_rpdo2(self.node_id), encode_rpdo2(controlword, target_velocity)))

    def last_tpdo1(self) -> Optional[Tuple[int, int]]:
        frames = self.bus.take(cob_tpdo1(self.node_id))
        if not frames:
            return None
        return decode_tpdo1(frames[-1].data)

    def last_tpdo2(self) -> Optional[Tuple[int, int]]:
        frames = self.bus.take(cob_tpdo2(self.node_id))
        if not frames:
            return None
        return decode_tpdo2(frames[-1].data)


class SerialBridgeBus:
    """Talk CANopen through a flashed node's USB hex bridge (same line format as GCSP)."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required: pip install pyserial") from exc
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        try:
            self._ser.dtr = False
            self._ser.rts = False
        except Exception:
            pass
        self._rx: List[CanFrame] = []

    def send(self, frame: CanFrame) -> None:
        from .protocol import Frame, encode_line

        self._ser.write(encode_line(Frame(frame.can_id, frame.rtr, frame.data)))
        self._pump()

    def emit(self, frame: CanFrame) -> None:
        self._rx.append(frame)

    def take(self, can_id: int) -> List[CanFrame]:
        self._pump()
        hit = [fr for fr in self._rx if fr.can_id == can_id]
        self._rx = [fr for fr in self._rx if fr.can_id != can_id]
        return hit

    def drain(self) -> List[CanFrame]:
        self._pump()
        out = list(self._rx)
        self._rx.clear()
        return out

    def _pump(self) -> None:
        from .protocol import decode_line

        deadline = time.time() + 0.05
        while time.time() < deadline:
            raw = self._ser.readline()
            if not raw:
                break
            parsed = decode_line(raw)
            if parsed is not None:
                self._rx.append(CanFrame(parsed.can_id, parsed.data, parsed.rtr))

    def close(self) -> None:
        self._ser.close()


# ---------------------------------------------------------------------------
# EDS
# ---------------------------------------------------------------------------

_EDS_HEADER = """\
; Grafito CANStepper C3 - CANopen Electronic Data Sheet (CiA 306)
; Matches firmware/GrafitoCANStepper_C3_CANopen (fw 2.0, CiA 402 subset).
; Vendor ID 0x000005A3 is a development placeholder. Register a CiA ID before
; shipping a production EDS.

[FileInfo]
FileName=GrafitoCANStepper.eds
FileVersion=1
FileRevision=0
EDSVersion=4.0
Description=Grafito CANStepper C3 CiA 402
CreationTime=12:00PM
CreationDate=09-13-2026
CreatedBy=Grafito Innovations
ModificationTime=12:00PM
ModificationDate=09-13-2026
ModifiedBy=Grafito Innovations

[DeviceInfo]
VendorName={vendor}
VendorNumber=0x{vendor_id:08X}
ProductName={product_name}
ProductNumber=0x{product_code:08X}
RevisionNumber=0x{revision:08X}
OrderCode=GRAFITO-CANSTEPPER-C3
BaudRate_10=0
BaudRate_20=0
BaudRate_50=0
BaudRate_125=1
BaudRate_250=1
BaudRate_500=1
BaudRate_800=0
BaudRate_1000=1
SimpleBootUpMaster=0
SimpleBootUpSlave=1
Granularity=8
DynamicChannelsSupported=0
CompactPDO=0
GroupMessaging=0
NrOfRXPDO=2
NrOfTXPDO=2
LSS_Supported=0

[DummyUsage]
Dummy0001=0
Dummy0002=1
Dummy0003=1
Dummy0004=1
Dummy0005=1
Dummy0006=1
Dummy0007=1

[Comments]
Lines=2
Line1=Units: position = MT6701 encoder counts (16384 / rev).
Line2=Supported 402 modes: pp (1), pv (3), hm (6).

"""


def _eds_object_type(index: int, subs: List[int]) -> str:
    if len(subs) == 1 and subs[0] == 0:
        return "0x7"
    # identity / comm / mapping records
    if index in (0x1018, 0x1200) or 0x1400 <= index <= 0x1BFF or index == 0x6099:
        return "0x9"
    return "0x8"


def render_eds() -> str:
    by_index: Dict[int, List[OdEntry]] = {}
    for entry in OD_ENTRIES.values():
        by_index.setdefault(entry.index, []).append(entry)
    for entries in by_index.values():
        entries.sort(key=lambda x: x.sub)

    mandatory = [0x1000, 0x1001, 0x1018]
    manufacturer = [i for i in by_index if 0x2000 <= i <= 0x5FFF]
    optional = [i for i in by_index if i not in mandatory and i not in manufacturer]
    mandatory.sort()
    optional.sort()
    manufacturer.sort()

    lines = [
        _EDS_HEADER.format(
            vendor=VENDOR_NAME,
            vendor_id=VENDOR_ID,
            product_name=PRODUCT_NAME,
            product_code=PRODUCT_CODE,
            revision=REVISION,
        )
    ]

    def emit_list(title: str, indexes: List[int]) -> None:
        lines.append(f"[{title}]")
        lines.append(f"SupportedObjects={len(indexes)}")
        for n, idx in enumerate(indexes, start=1):
            lines.append(f"{n}=0x{idx:04X}")
        lines.append("")

    emit_list("MandatoryObjects", mandatory)
    emit_list("OptionalObjects", optional)
    emit_list("ManufacturerObjects", manufacturer)

    for idx in mandatory + optional + manufacturer:
        entries = by_index[idx]
        subs = [e.sub for e in entries]
        if len(entries) == 1 and entries[0].sub == 0:
            lines.append(_render_var(entries[0], f"{idx:04X}"))
        else:
            lines.append(f"[{idx:04X}]")
            lines.append(f"SubNumber={len(entries)}")
            lines.append(f"ParameterName={entries[0].name.split(' Number')[0]}")
            lines.append(f"ObjectType={_eds_object_type(idx, subs)}")
            lines.append("")
            for entry in entries:
                lines.append(_render_var(entry, f"{idx:04X}sub{entry.sub}"))
    return "\n".join(lines).rstrip() + "\n"


def _format_od_value(entry: OdEntry, value: Optional[object] = None) -> str:
    v = entry.default if value is None else value
    if entry.dtype == DType.VIS:
        return str(v)
    if entry.dtype == DType.R32:
        return str(float(v))
    ival = int(v)
    return f"0x{ival & 0xFFFFFFFF:X}" if ival > 9 else str(ival)


def _dtype_name(dtype: DType) -> str:
    return {
        DType.I8: "INTEGER8",
        DType.I16: "INTEGER16",
        DType.I32: "INTEGER32",
        DType.U8: "UNSIGNED8",
        DType.U16: "UNSIGNED16",
        DType.U32: "UNSIGNED32",
        DType.R32: "REAL32",
        DType.VIS: "VISIBLE_STRING",
    }[dtype]


def _od_group(index: int) -> str:
    if index in (0x1000, 0x1001, 0x1018):
        return "mandatory"
    if 0x2000 <= index <= 0x5FFF:
        return "manufacturer"
    return "optional"


def instance_values(node_id: int = DEFAULT_NODE_ID) -> Dict[Tuple[int, int], object]:
    """Factory defaults with COB-IDs and node ID filled for one DCF instance."""
    if not 1 <= node_id <= 127:
        raise ValueError("node_id must be 1-127")
    values = {(e.index, e.sub): e.default for e in OD_ENTRIES.values()}
    values[(0x2000, 0)] = node_id
    values[(0x1014, 0)] = cob_emcy(node_id)
    values[(0x1200, 1)] = cob_rsdo(node_id)
    values[(0x1200, 2)] = cob_tsdo(node_id)
    values[(0x1400, 1)] = cob_rpdo1(node_id)
    values[(0x1401, 1)] = cob_rpdo2(node_id)
    values[(0x1800, 1)] = cob_tpdo1(node_id)
    values[(0x1801, 1)] = cob_tpdo2(node_id)
    return values


def od_catalog(node_id: int = DEFAULT_NODE_ID) -> List[dict]:
    """Flat object dictionary used by JSON/CSV exports and PLC tools."""
    values = instance_values(node_id)
    rows = []
    for entry in sorted(OD_ENTRIES.values(), key=lambda e: (e.index, e.sub)):
        value = values[(entry.index, entry.sub)]
        rows.append(
            {
                "index": f"0x{entry.index:04X}",
                "index_int": entry.index,
                "subindex": entry.sub,
                "name": entry.name,
                "data_type": _dtype_name(entry.dtype),
                "data_type_code": f"0x{int(entry.dtype):04X}",
                "access": entry.access,
                "pdo_mapping": bool(entry.pdo),
                "default": value if entry.dtype != DType.VIS else str(value),
                "default_text": _format_od_value(entry, value),
                "minimum": entry.minimum,
                "maximum": entry.maximum,
                "group": _od_group(entry.index),
            }
        )
    return rows


def render_od_json(node_id: int = DEFAULT_NODE_ID) -> str:
    payload = {
        "device": {
            "vendor_name": VENDOR_NAME,
            "vendor_id": f"0x{VENDOR_ID:08X}",
            "product_name": PRODUCT_NAME,
            "product_code": f"0x{PRODUCT_CODE:08X}",
            "revision": f"0x{REVISION:08X}",
            "device_type": f"0x{DEVICE_TYPE:08X}",
            "firmware": SW_VERSION,
            "node_id": node_id,
            "bitrate_bps": DEFAULT_BITRATE,
        },
        "units": {
            "position": "MT6701 encoder counts",
            "counts_per_revolution": ENC_CPR,
            "degrees": "counts * 360 / 16384",
        },
        "objects": od_catalog(node_id),
    }
    return json.dumps(payload, indent=2) + "\n"


def render_od_csv(node_id: int = DEFAULT_NODE_ID) -> str:
    buf = io.StringIO()
    fields = [
        "index",
        "subindex",
        "name",
        "data_type",
        "access",
        "pdo_mapping",
        "default_text",
        "minimum",
        "maximum",
        "group",
    ]
    writer = csv.DictWriter(buf, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
    writer.writeheader()
    for row in od_catalog(node_id):
        writer.writerow(row)
    return buf.getvalue()


def _render_var(entry: OdEntry, section: str, parameter_value: Optional[object] = None) -> str:
    default_s = _format_od_value(entry)
    rows = [
        f"[{section}]",
        f"ParameterName={entry.name}",
        "ObjectType=0x7",
        f"DataType=0x{int(entry.dtype):04X}",
        f"AccessType={entry.access}",
        f"DefaultValue={default_s}",
        f"PDOMapping={1 if entry.pdo else 0}",
    ]
    if parameter_value is not None:
        rows.append(f"ParameterValue={_format_od_value(entry, parameter_value)}")
    if entry.minimum is not None:
        rows.append(f"LowLimit={entry.minimum}")
    if entry.maximum is not None:
        rows.append(f"HighLimit={entry.maximum}")
    rows.append("")
    return "\n".join(rows)


_DCF_HEADER = """\
; Grafito CANStepper C3 - CANopen Device Configuration File (CiA 306 DCF)
; Instance of GrafitoCANStepper.eds for a commissioned node.
; Import this in the PLC after the EDS (or instead, if the tool accepts DCF).

[FileInfo]
FileName={dcf_name}
FileVersion=1
FileRevision=0
EDSVersion=4.0
Description=Grafito CANStepper C3 node {node_id} instance
CreationTime=12:00PM
CreationDate=09-13-2026
CreatedBy=Grafito Innovations
ModificationTime=12:00PM
ModificationDate=09-13-2026
ModifiedBy=Grafito Innovations
LastEDS=GrafitoCANStepper.eds

[DeviceInfo]
VendorName={vendor}
VendorNumber=0x{vendor_id:08X}
ProductName={product_name}
ProductNumber=0x{product_code:08X}
RevisionNumber=0x{revision:08X}
OrderCode=GRAFITO-CANSTEPPER-C3
BaudRate_10=0
BaudRate_20=0
BaudRate_50=0
BaudRate_125=1
BaudRate_250=1
BaudRate_500=1
BaudRate_800=0
BaudRate_1000=1
SimpleBootUpMaster=0
SimpleBootUpSlave=1
Granularity=8
DynamicChannelsSupported=0
CompactPDO=0
GroupMessaging=0
NrOfRXPDO=2
NrOfTXPDO=2
LSS_Supported=0

[DeviceComissioning]
NodeID={node_id}
NodeName=CANStepper_{node_id}
Baudrate={baud_k}
NetNumber=1
NetworkName=GrafitoCAN
LSS_SerialNumber=1

[DummyUsage]
Dummy0001=0
Dummy0002=1
Dummy0003=1
Dummy0004=1
Dummy0005=1
Dummy0006=1
Dummy0007=1

[Comments]
Lines=3
Line1=Units: position = MT6701 encoder counts (16384 / rev).
Line2=Supported 402 modes: pp (1), pv (3), hm (6).
Line3=ParameterValue is the commissioned instance; DefaultValue is the factory EDS default.

"""


def render_dcf(node_id: int = DEFAULT_NODE_ID, bitrate_bps: int = DEFAULT_BITRATE) -> str:
    if bitrate_bps not in (125_000, 250_000, 500_000, 1_000_000):
        raise ValueError("unsupported DCF bitrate")
    values = instance_values(node_id)
    by_index: Dict[int, List[OdEntry]] = {}
    for entry in OD_ENTRIES.values():
        by_index.setdefault(entry.index, []).append(entry)
    for entries in by_index.values():
        entries.sort(key=lambda x: x.sub)

    mandatory = [0x1000, 0x1001, 0x1018]
    manufacturer = sorted(i for i in by_index if 0x2000 <= i <= 0x5FFF)
    optional = sorted(i for i in by_index if i not in mandatory and i not in manufacturer)

    lines = [
        _DCF_HEADER.format(
            vendor=VENDOR_NAME,
            vendor_id=VENDOR_ID,
            product_name=PRODUCT_NAME,
            product_code=PRODUCT_CODE,
            revision=REVISION,
            node_id=node_id,
            dcf_name=f"GrafitoCANStepper_Node{node_id}.dcf",
            baud_k=bitrate_bps // 1000,
        )
    ]

    def emit_list(title: str, indexes: List[int]) -> None:
        lines.append(f"[{title}]")
        lines.append(f"SupportedObjects={len(indexes)}")
        for n, idx in enumerate(indexes, start=1):
            lines.append(f"{n}=0x{idx:04X}")
        lines.append("")

    emit_list("MandatoryObjects", mandatory)
    emit_list("OptionalObjects", optional)
    emit_list("ManufacturerObjects", manufacturer)

    for idx in mandatory + optional + manufacturer:
        entries = by_index[idx]
        subs = [e.sub for e in entries]
        if len(entries) == 1 and entries[0].sub == 0:
            lines.append(
                _render_var(entries[0], f"{idx:04X}", values[(idx, 0)])
            )
        else:
            lines.append(f"[{idx:04X}]")
            lines.append(f"SubNumber={len(entries)}")
            lines.append(f"ParameterName={entries[0].name.split(' Number')[0]}")
            lines.append(f"ObjectType={_eds_object_type(idx, subs)}")
            lines.append("")
            for entry in entries:
                lines.append(
                    _render_var(
                        entry,
                        f"{idx:04X}sub{entry.sub}",
                        values[(entry.index, entry.sub)],
                    )
                )
    return "\n".join(lines).rstrip() + "\n"


def write_eds(path: Optional[Path] = None) -> Path:
    dest = path or eds_path()
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(render_eds(), encoding="ascii")
    return dest


def write_dcf(path: Optional[Path] = None, node_id: int = DEFAULT_NODE_ID) -> Path:
    dest = path or dcf_path()
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(render_dcf(node_id=node_id), encoding="ascii")
    return dest


def write_object_dictionary(node_id: int = DEFAULT_NODE_ID) -> Tuple[Path, Path]:
    json_dest = od_json_path()
    csv_dest = od_csv_path()
    json_dest.parent.mkdir(parents=True, exist_ok=True)
    json_dest.write_text(render_od_json(node_id), encoding="utf-8")
    csv_dest.write_text(render_od_csv(node_id), encoding="utf-8")
    return json_dest, csv_dest


def write_canopen_descriptions(node_id: int = DEFAULT_NODE_ID) -> Dict[str, Path]:
    """Write EDS + DCF + object-dictionary JSON/CSV from the same OD table."""
    od_json, od_csv = write_object_dictionary(node_id)
    return {
        "eds": write_eds(),
        "dcf": write_dcf(node_id=node_id),
        "od_json": od_json,
        "od_csv": od_csv,
    }


def parse_eds(text: str) -> Dict[str, Dict[str, str]]:
    """Minimal INI-style EDS parser (enough to validate PLC import contents)."""
    sections: Dict[str, Dict[str, str]] = {}
    current = ""
    for raw in text.splitlines():
        line = raw.split(";", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            current = line[1:-1]
            sections[current] = {}
            continue
        if "=" in line and current:
            key, val = line.split("=", 1)
            sections[current][key.strip()] = val.strip()
    return sections


def eds_object_indexes(sections: Dict[str, Dict[str, str]]) -> List[int]:
    out: List[int] = []
    for group in ("MandatoryObjects", "OptionalObjects", "ManufacturerObjects"):
        block = sections.get(group, {})
        count = int(block.get("SupportedObjects", "0"))
        for i in range(1, count + 1):
            out.append(int(block[str(i)], 16))
    return out
