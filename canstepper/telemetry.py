"""Cached, decoded telemetry state per node."""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Optional

from .protocol import (
    FLAG_ENABLED,
    FLAG_ENCODER_OK,
    FLAG_ENDSTOP,
    FLAG_ESTOPPED,
    FLAG_HOMED,
    FLAG_MOVING,
    FLAG_STALLED,
    Fault,
    Frame,
    Mode,
    Tel,
)


@dataclass
class NodeStatus:
    """Decoded Tel.STATUS."""

    enabled: bool = False
    moving: bool = False
    homed: bool = False
    estopped: bool = False
    endstop_active: bool = False
    stall_latched: bool = False
    encoder_ok: bool = True
    mode: Mode = Mode.IDLE
    fault: Fault = Fault.NONE
    fw_major: int = 0
    fw_minor: int = 0
    protocol_version: int = 0
    # fw ≥1.5 STATUS byte 7 — GPIO8 / HOME pin diagnostics
    home_raw_high: Optional[bool] = None  # True if digitalRead is HIGH
    endstop_enabled: Optional[bool] = None
    endstop_active_high: Optional[bool] = None

    @property
    def firmware(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}"

    @staticmethod
    def decode(data: bytes) -> "NodeStatus":
        if len(data) < 6:
            raise ValueError(f"STATUS payload too short ({len(data)} bytes)")
        flags = data[0]
        home_raw_high = None
        endstop_enabled = None
        endstop_active_high = None
        # Byte 7 present on fw ≥1.5 (and always sent as 8-byte STATUS)
        if len(data) >= 8 and (data[3] > 1 or (data[3] == 1 and data[4] >= 5)):
            diag = data[7]
            home_raw_high = bool(diag & 0x01)
            endstop_enabled = bool(diag & 0x04)
            endstop_active_high = bool(diag & 0x08)
        return NodeStatus(
            enabled=bool(flags & FLAG_ENABLED),
            moving=bool(flags & FLAG_MOVING),
            homed=bool(flags & FLAG_HOMED),
            estopped=bool(flags & FLAG_ESTOPPED),
            endstop_active=bool(flags & FLAG_ENDSTOP),
            stall_latched=bool(flags & FLAG_STALLED),
            encoder_ok=bool(flags & FLAG_ENCODER_OK),
            mode=Mode(data[1]) if data[1] in Mode._value2member_map_ else Mode.IDLE,
            fault=Fault(data[2]) if data[2] in Fault._value2member_map_ else Fault.NONE,
            fw_major=data[3],
            fw_minor=data[4],
            protocol_version=data[5],
            home_raw_high=home_raw_high,
            endstop_enabled=endstop_enabled,
            endstop_active_high=endstop_active_high,
        )


@dataclass
class PidStatus:
    """Decoded Tel.PID_STATUS."""

    state: int = 0          # 0 idle, 1 running, 2 settled, 3 fault
    fault: Fault = Fault.NONE
    output_deg_s: float = 0.0

    @staticmethod
    def decode(data: bytes) -> "PidStatus":
        out = struct.unpack("<f", data[2:6])[0]
        return PidStatus(
            data[0],
            Fault(data[1]) if data[1] in Fault._value2member_map_ else Fault.NONE,
            out,
        )


@dataclass
class DriverStatus:
    """Decoded Tel.DRIVER — TMC2209 DRV_STATUS + GSTAT (+ StallGuard).

    Layout matches firmware ≥1.4 (8-byte payload). Older 3-byte frames still
    decode: only ``stallguard`` and ``uart_ok`` are populated.
    """

    stallguard: int = 0
    uart_ok: bool = False
    # Thermal
    otpw: bool = False                 # over-temperature pre-warning
    over_temp_shutdown: bool = False   # OT — driver disabled itself
    temp_120c: bool = False
    temp_143c: bool = False
    temp_150c: bool = False
    temp_157c: bool = False
    # Faults / shorts / open load
    short_to_gnd_a: bool = False
    short_to_gnd_b: bool = False
    low_side_short_a: bool = False
    low_side_short_b: bool = False
    open_load_a: bool = False
    open_load_b: bool = False
    # Mode
    stealth_chop: bool = False
    standstill: bool = False
    # GSTAT
    reset_flag: bool = False
    drv_err: bool = False
    uv_cp: bool = False                # charge-pump under-voltage
    # Live scale / timing
    cs_actual: int = 0                 # 0..31 actual current scale
    interstep: int = 0                 # TSTEP-style interstep duration

    def commanded_rms_amps(
        self,
        rsense_ohm: Optional[float] = None,
        vfsense_v: Optional[float] = None,
    ) -> float:
        """Commanded/regulated motor I_RMS from ``cs_actual`` (TMC2209 datasheet).

        Uses Grafito CANStepper defaults (100 mΩ, ``V_FS=0.325 V``) unless
        ``rsense_ohm`` / ``vfsense_v`` are overridden.  This is **not** 24 V
        bus current — see :mod:`canstepper.tmc2209`.
        """
        from .tmc2209 import CANSTEPPER_RSENSE_OHM, CANSTEPPER_VFSENSE_V, commanded_rms_amps

        kwargs = {}
        if rsense_ohm is not None:
            kwargs["rsense_ohm"] = rsense_ohm
        if vfsense_v is not None:
            kwargs["vfsense_v"] = vfsense_v
        return commanded_rms_amps(self.cs_actual, **kwargs)

    @property
    def commanded_rms_amps_default(self) -> float:
        """Same as ``commanded_rms_amps()`` with board-default sense values."""
        return self.commanded_rms_amps()

    @property
    def any_short(self) -> bool:
        return (
            self.short_to_gnd_a
            or self.short_to_gnd_b
            or self.low_side_short_a
            or self.low_side_short_b
        )

    @property
    def thermal_warning(self) -> bool:
        return self.otpw or self.temp_120c or self.temp_143c

    @property
    def thermal_fault(self) -> bool:
        return self.over_temp_shutdown or self.temp_150c or self.temp_157c

    @staticmethod
    def decode(data: bytes) -> "DriverStatus":
        if len(data) < 2:
            return DriverStatus()
        sg = struct.unpack_from("<H", data, 0)[0]
        if len(data) < 3:
            return DriverStatus(stallguard=sg)
        fa = data[2]
        # Legacy 3-byte frame: byte2 was 0/1 uart_ok only
        if len(data) < 8:
            return DriverStatus(stallguard=sg, uart_ok=bool(fa & 0x01) or fa == 1)
        fb = data[3]
        gstat = data[4]
        return DriverStatus(
            stallguard=sg,
            uart_ok=bool(fa & 0x01),
            otpw=bool(fa & 0x02),
            over_temp_shutdown=bool(fa & 0x04),
            short_to_gnd_a=bool(fa & 0x08),
            short_to_gnd_b=bool(fa & 0x10),
            low_side_short_a=bool(fa & 0x20),
            low_side_short_b=bool(fa & 0x40),
            open_load_a=bool(fa & 0x80),
            open_load_b=bool(fb & 0x01),
            temp_120c=bool(fb & 0x02),
            temp_143c=bool(fb & 0x04),
            temp_150c=bool(fb & 0x08),
            temp_157c=bool(fb & 0x10),
            stealth_chop=bool(fb & 0x20),
            standstill=bool(fb & 0x40),
            reset_flag=bool(gstat & 0x01),
            drv_err=bool(gstat & 0x02),
            uv_cp=bool(gstat & 0x04),
            cs_actual=int(data[5] & 0x1F),
            interstep=struct.unpack_from("<H", data, 6)[0],
        )


@dataclass
class FollowStatus:
    """Decoded Tel.FOLLOW_STATUS."""

    enabled: bool = False
    leader_id: int = 0
    invert: bool = False
    encoder_corrected: bool = False
    synced: bool = False
    ratio: float = 1.0

    @staticmethod
    def decode(data: bytes) -> "FollowStatus":
        return FollowStatus(
            enabled=bool(data[0]),
            leader_id=data[1],
            invert=bool(data[2] & 1),
            encoder_corrected=bool(data[2] & 2),
            synced=bool(data[3]),
            ratio=struct.unpack("<f", data[4:8])[0],
        )


@dataclass
class LutStatus:
    """Decoded Tel.LUT_STATUS (fw ≥1.10)."""

    valid: bool = False
    enabled: bool = False
    points: int = 0
    peak_inl_deg: float = 0.0

    @staticmethod
    def decode(data: bytes) -> "LutStatus":
        if len(data) < 8:
            raise ValueError(f"LUT_STATUS payload too short ({len(data)} bytes)")
        return LutStatus(
            valid=bool(data[0]),
            enabled=bool(data[1]),
            points=struct.unpack_from("<H", data, 2)[0],
            peak_inl_deg=struct.unpack_from("<f", data, 4)[0],
        )


@dataclass
class CanHealth:
    """Decoded Tel.CAN_HEALTH."""

    state: int = 0          # TWAI state: 0 stopped, 1 running, 2 bus-off, 3 recovering
    tx_error_count: int = 0
    rx_error_count: int = 0
    recovery_count: int = 0
    tx_failed_count: int = 0
    bus_error_count: int = 0

    STATE_NAMES = {0: "stopped", 1: "running", 2: "bus_off", 3: "recovering",
                   0xFF: "unavailable"}

    @property
    def state_name(self) -> str:
        return self.STATE_NAMES.get(self.state, "unknown")

    @staticmethod
    def decode(data: bytes) -> "CanHealth":
        txf, bus = struct.unpack("<HH", data[4:8])
        return CanHealth(data[0], data[1], data[2], data[3], txf, bus)


@dataclass
class NodeState:
    """Last-known telemetry for one node, updated from the RX stream.

    Reading from this object never touches the bus; check ``age()`` if
    freshness matters, or use the node's blocking getters instead.
    """

    node_id: int
    position_deg: Optional[float] = None
    velocity_deg_s: Optional[float] = None
    position_error_deg: Optional[float] = None
    target_deg: Optional[float] = None
    encoder_counts: Optional[int] = None
    status: Optional[NodeStatus] = None
    pid: Optional[PidStatus] = None
    follow: Optional[FollowStatus] = None
    can_health: Optional[CanHealth] = None
    mcu_temp_c: Optional[float] = None
    bus_voltage: Optional[float] = None
    stallguard: Optional[int] = None
    driver: Optional["DriverStatus"] = None
    lut: Optional["LutStatus"] = None
    last_seen: float = 0.0
    _stamps: dict = field(default_factory=dict, repr=False)

    def age(self, what: Optional[str] = None) -> float:
        """Seconds since the last frame (optionally for one field)."""
        stamp = self._stamps.get(what, self.last_seen) if what else self.last_seen
        if not stamp:
            return float("inf")
        return time.monotonic() - stamp

    def update(self, frame: Frame) -> None:
        now = time.monotonic()
        self.last_seen = now
        msg = frame.msg_id
        data = frame.data
        try:
            if msg == Tel.POSITION and len(data) >= 8:
                self.position_deg = struct.unpack("<d", data[:8])[0]
                self._stamps["position"] = now
            elif msg == Tel.MOTION and len(data) >= 8:
                self.velocity_deg_s, self.position_error_deg = struct.unpack(
                    "<ff", data[:8]
                )
                self._stamps["motion"] = now
            elif msg == Tel.STATUS and len(data) >= 7:
                self.status = NodeStatus.decode(data)
                self._stamps["status"] = now
            elif msg == Tel.TARGET and len(data) >= 8:
                self.target_deg = struct.unpack("<d", data[:8])[0]
            elif msg == Tel.ENC_COUNTS and len(data) >= 8:
                self.encoder_counts = struct.unpack("<q", data[:8])[0]
            elif msg == Tel.PID_STATUS and len(data) >= 6:
                self.pid = PidStatus.decode(data)
                self._stamps["pid"] = now
            elif msg == Tel.FOLLOW_STATUS and len(data) >= 8:
                self.follow = FollowStatus.decode(data)
            elif msg == Tel.CAN_HEALTH and len(data) >= 8:
                self.can_health = CanHealth.decode(data)
            elif msg == Tel.ENV and len(data) >= 8:
                self.mcu_temp_c, self.bus_voltage = struct.unpack("<ff", data[:8])
            elif msg == Tel.DRIVER and len(data) >= 2:
                self.driver = DriverStatus.decode(data)
                self.stallguard = self.driver.stallguard
                self._stamps["driver"] = now
            elif msg == Tel.LUT_STATUS and len(data) >= 8:
                self.lut = LutStatus.decode(data)
                self._stamps["lut"] = now
        except (struct.error, ValueError):
            pass
