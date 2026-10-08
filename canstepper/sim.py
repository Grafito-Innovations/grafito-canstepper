"""Software simulation of GrafitoCANStepper nodes (GCSP v1).

``SimNetwork`` is a :class:`~canstepper.transport.base.Transport` whose far
end is a set of :class:`SimNode` objects implementing the firmware's command
table. It powers the test suite and lets every example in the documentation
run without hardware::

    from canstepper import CANStepperBus
    from canstepper.sim import SimNetwork

    bus = CANStepperBus(SimNetwork([1, 2, 3]))
    print(bus.discover(timeout=0.1))

Simplifications versus real firmware: moves complete instantly (the shaft
teleports to the target and MOVE_DONE fires), velocity mode integrates only
when ``tick()`` is called, and there is no CAN arbitration or loss.
"""

from __future__ import annotations

import struct
from typing import Dict, Iterable, List, Optional

from .protocol import (
    FLAG_ENABLED,
    FLAG_ENCODER_OK,
    FLAG_ENDSTOP,
    FLAG_ESTOPPED,
    FLAG_HOMED,
    FLAG_MOVING,
    Cmd,
    Event,
    Fault,
    Frame,
    HomeMethod,
    Mode,
    PARAMS,
    Param,
    ParamStatus,
    Tel,
    make_can_id,
    split_can_id,
)
from .transport.base import Transport

SIM_FW_MAJOR = 1
SIM_FW_MINOR = 2
SIM_PROTO = 1


def _default_params() -> Dict[int, float]:
    return {int(d.param): d.default for d in PARAMS.values()}


class SimNode:
    """One simulated CANStepper board."""

    def __init__(self, node_id: int, network: "SimNetwork"):
        self._net = network
        self.params: Dict[int, float] = _default_params()
        self.params[int(Param.NODE_ID)] = node_id
        self.nvs: Dict[int, float] = dict(self.params)
        self.position_deg = 0.0
        self.velocity_deg_s = 0.0
        self.target_deg = 0.0
        self.mode = Mode.IDLE
        self.fault = Fault.NONE
        self.enabled = bool(self.params[int(Param.ENABLE_ON_BOOT)])
        self.estopped = False
        self.homed = False
        self.endstop_active = False
        # follower state
        self.follow_enabled = False
        self.follow_leader = 0
        self.follow_invert = False
        self.follow_enc_corrected = True
        self.follow_ratio = 1.0
        self.follow_synced = False
        self._follow_leader_ref = 0.0
        self._follow_local_ref = 0.0
        self.lut_valid = False
        self.lut_enabled = False
        self.lut_peak_inl_deg = 0.0

    # -- identity ---------------------------------------------------------------

    @property
    def node_id(self) -> int:
        return int(self.params[int(Param.NODE_ID)])

    # -- frame I/O -----------------------------------------------------------------

    def handle(self, frame: Frame) -> None:
        target, msg_id = split_can_id(frame.can_id)

        if (
            not frame.rtr
            and msg_id == Tel.POSITION
            and self.follow_enabled
            and target == self.follow_leader
            and len(frame.data) >= 8
        ):
            self._on_leader_position(struct.unpack("<d", frame.data[:8])[0])

        if target not in (self.node_id, 0):
            return
        if frame.rtr:
            self._handle_rtr(msg_id)
        elif msg_id < 32:
            self._handle_cmd(msg_id, frame.data)

    def _emit(self, msg_id: int, data: bytes) -> None:
        self._net._emit_from_node(self.node_id, msg_id, data)

    # -- telemetry ---------------------------------------------------------------

    def _status_payload(self) -> bytes:
        flags = FLAG_ENCODER_OK
        if self.enabled:
            flags |= FLAG_ENABLED
        if self.mode == Mode.VELOCITY and self.velocity_deg_s != 0.0:
            flags |= FLAG_MOVING
        if self.homed:
            flags |= FLAG_HOMED
        if self.estopped:
            flags |= FLAG_ESTOPPED
        if self.endstop_active:
            flags |= FLAG_ENDSTOP
        return bytes(
            [flags, int(self.mode), int(self.fault),
             SIM_FW_MAJOR, SIM_FW_MINOR, SIM_PROTO, self.node_id, 0]
        )

    def _handle_rtr(self, msg_id: int) -> None:
        if msg_id == Tel.STATUS:
            self._emit(Tel.STATUS, self._status_payload())
        elif msg_id == Tel.POSITION:
            self._emit(Tel.POSITION, struct.pack("<d", self.position_deg))
        elif msg_id == Tel.MOTION:
            err = self.target_deg - self.position_deg if self.mode == Mode.POSITION else 0.0
            self._emit(Tel.MOTION, struct.pack("<ff", self.velocity_deg_s, err))
        elif msg_id == Tel.TARGET:
            self._emit(Tel.TARGET, struct.pack("<d", self.target_deg))
        elif msg_id == Tel.ENC_COUNTS:
            counts = int(round(self.position_deg / 360.0 * 16384))
            self._emit(Tel.ENC_COUNTS, struct.pack("<q", counts))
        elif msg_id == Tel.DRIVER:
            # fw ≥1.4 layout: sg, flags_a (uart_ok), flags_b, gstat, cs, interstep
            self._emit(
                Tel.DRIVER,
                struct.pack("<HBBBBH", 250, 0x01, 0x40, 0x00, 16, 100),
            )
        elif msg_id == Tel.ENV:
            self._emit(Tel.ENV, struct.pack("<ff", 42.0, 24.0))
        elif msg_id == Tel.FOLLOW_STATUS:
            self._emit(Tel.FOLLOW_STATUS, self._follow_payload())
        elif msg_id == Tel.CAN_HEALTH:
            self._emit(Tel.CAN_HEALTH, struct.pack("<BBBBHH", 1, 0, 0, 0, 0, 0))
        elif msg_id == Tel.PID_STATUS:
            state = 2 if self.mode == Mode.POSITION else 0
            self._emit(Tel.PID_STATUS, struct.pack("<BBf", state, int(self.fault), 0.0))
        elif msg_id == Tel.LUT_STATUS:
            self._emit(
                Tel.LUT_STATUS,
                struct.pack(
                    "<BBHf",
                    1 if self.lut_valid else 0,
                    1 if self.lut_enabled else 0,
                    200 if self.lut_valid else 0,
                    float(self.lut_peak_inl_deg),
                ),
            )

    def _follow_payload(self) -> bytes:
        flags = (1 if self.follow_invert else 0) | (2 if self.follow_enc_corrected else 0)
        return struct.pack(
            "<BBBBf",
            1 if self.follow_enabled else 0,
            self.follow_leader or 1,
            flags,
            1 if self.follow_synced else 0,
            self.follow_ratio,
        )

    def _event(self, event: Event, detail: int = 0, data: float = 0.0) -> None:
        self._emit(Tel.EVENT, struct.pack("<BBf", int(event), detail, data))

    # -- commands -----------------------------------------------------------------

    def _handle_cmd(self, msg_id: int, d: bytes) -> None:
        if msg_id == Cmd.PING:
            self._emit(Tel.STATUS, self._status_payload())
        elif msg_id == Cmd.ESTOP:
            self.enabled = False
            self.estopped = True
            self.velocity_deg_s = 0.0
            self.follow_enabled = False
            self.mode = Mode.IDLE
            self._event(Event.ESTOP, 0, self.position_deg)
            self._emit(Tel.STATUS, self._status_payload())
        elif msg_id == Cmd.STOP:
            self.velocity_deg_s = 0.0
            self.mode = Mode.IDLE
        elif msg_id == Cmd.ENABLE and len(d) >= 1:
            self.enabled = d[0] != 0
            if self.enabled:
                self.estopped = False
            else:
                self.velocity_deg_s = 0.0
                self.mode = Mode.IDLE
            self._emit(Tel.STATUS, self._status_payload())
        elif msg_id == Cmd.MOVE_ABS and len(d) >= 8:
            self._move(struct.unpack("<d", d[:8])[0])
        elif msg_id == Cmd.MOVE_REL and len(d) >= 8:
            base = self.target_deg if self.mode == Mode.POSITION else self.position_deg
            self._move(base + struct.unpack("<d", d[:8])[0])
        elif msg_id == Cmd.MOVE_VEL and len(d) >= 4:
            if self.enabled and not self.estopped:
                self.follow_enabled = False
                self.velocity_deg_s = struct.unpack("<f", d[:4])[0]
                self.mode = Mode.VELOCITY if self.velocity_deg_s else Mode.IDLE
        elif msg_id == Cmd.SET_ZERO:
            self.position_deg = 0.0
            self.target_deg = 0.0
            self.velocity_deg_s = 0.0
            self.mode = Mode.IDLE
        elif msg_id == Cmd.SET_POSITION and len(d) >= 8:
            self.position_deg = struct.unpack("<d", d[:8])[0]
            self.target_deg = self.position_deg
            self.velocity_deg_s = 0.0
            self.mode = Mode.IDLE
        elif msg_id == Cmd.HOME and len(d) >= 6:
            self._home(d[0], 1 if d[1] < 128 else -1, struct.unpack("<f", d[2:6])[0])
        elif msg_id == Cmd.SET_PARAM and len(d) >= 5:
            self._set_param(d[0], d[1:5])
        elif msg_id == Cmd.GET_PARAM and len(d) >= 1:
            self._get_param(d[0])
        elif msg_id == Cmd.SAVE_CONFIG:
            self.nvs = dict(self.params)
        elif msg_id == Cmd.LOAD_DEFAULTS:
            keep = self.node_id
            self.params = _default_params()
            self.params[int(Param.NODE_ID)] = keep
            self.nvs = dict(self.params)
            self.lut_valid = False
            self.lut_enabled = False
            self.lut_peak_inl_deg = 0.0
        elif msg_id == Cmd.FOLLOW and len(d) >= 8:
            self._follow_config(d)
        elif msg_id == Cmd.FOLLOW_SYNC:
            self.follow_synced = False
        elif msg_id == Cmd.LUT and len(d) >= 1:
            self._lut(d[0])

    def _move(self, target: float) -> None:
        if not self.enabled or self.estopped:
            return
        self.follow_enabled = False
        self.mode = Mode.POSITION
        self.target_deg = target
        self.position_deg = target        # instantaneous in simulation
        self.velocity_deg_s = 0.0
        self._emit(Tel.POSITION, struct.pack("<d", self.position_deg))
        self._event(Event.MOVE_DONE, 0, self.position_deg)

    def _home(self, method: int, direction: int, speed: float) -> None:
        if not self.enabled or self.estopped or method > 2:
            return
        self.follow_enabled = False
        backoff = float(self.params[int(Param.HOMING_BACKOFF)])
        if method == HomeMethod.SET_ZERO:
            self.position_deg = 0.0
        else:
            self.position_deg = backoff * -direction
        self.target_deg = self.position_deg
        self.homed = True
        self.mode = Mode.IDLE
        self._event(Event.HOMING_DONE, method, self.position_deg)

    def _set_param(self, pid: int, raw: bytes) -> None:
        pdef = PARAMS.get(Param(pid)) if pid in set(int(p) for p in Param) else None
        if pdef is None:
            self._emit(Tel.PARAM, struct.pack("<BB4s", pid, int(ParamStatus.UNKNOWN), b"\0" * 4))
            return
        value = struct.unpack("<f" if pdef.is_float else "<I", raw)[0]
        ok = pdef.minv <= float(value) <= pdef.maxv
        if pdef.param == Param.MICROSTEPS:
            v = int(value)
            ok = ok and v > 0 and (v & (v - 1)) == 0
        if not ok:
            self._emit(
                Tel.PARAM,
                bytes([pid, int(ParamStatus.REJECTED)]) + raw[:4],
            )
            return
        self.params[pid] = value
        self._emit(Tel.PARAM, bytes([pid, int(ParamStatus.OK)]) + raw[:4])

    def _get_param(self, pid: int) -> None:
        try:
            pdef = PARAMS[Param(pid)]
        except (ValueError, KeyError):
            self._emit(Tel.PARAM, bytes([pid, int(ParamStatus.UNKNOWN)]) + b"\0" * 4)
            return
        value = self.params[pid]
        raw = struct.pack("<f", float(value)) if pdef.is_float else struct.pack("<I", int(value))
        self._emit(Tel.PARAM, bytes([pid, int(ParamStatus.OK)]) + raw)

    def _lut(self, action: int) -> None:
        if not self.enabled or self.estopped:
            if action == 0:
                self._event(Event.LUT_FAILED, 1, 0.0)
            return
        if action == 0:
            self.lut_valid = True
            self.lut_enabled = True
            self.lut_peak_inl_deg = 0.0
            self.params[int(Param.LUT_ENABLE)] = 1
            self._event(Event.LUT_DONE, 0, 0.0)
        elif action == 1:
            if self.lut_valid:
                self.lut_enabled = True
                self.params[int(Param.LUT_ENABLE)] = 1
        elif action == 2:
            self.lut_enabled = False
            self.params[int(Param.LUT_ENABLE)] = 0
        elif action == 3:
            self.lut_valid = False
            self.lut_enabled = False
            self.lut_peak_inl_deg = 0.0
            self.params[int(Param.LUT_ENABLE)] = 0

    def _follow_config(self, d: bytes) -> None:
        enable = d[0] != 0
        leader = d[1]
        ratio = struct.unpack("<f", d[4:8])[0]
        if enable and (leader == self.node_id or not 1 <= leader <= 31 or ratio <= 0):
            return
        self.follow_enabled = enable
        self.follow_leader = leader
        self.follow_invert = bool(d[2] & 1)
        self.follow_enc_corrected = bool(d[2] & 2)
        self.follow_ratio = ratio
        self.follow_synced = False
        if enable:
            self.mode = Mode.FOLLOW

    def _on_leader_position(self, leader_deg: float) -> None:
        if not self.enabled or self.estopped:
            return
        if not self.follow_synced:
            self._follow_leader_ref = leader_deg
            self._follow_local_ref = self.position_deg
            self.follow_synced = True
            return
        delta = (leader_deg - self._follow_leader_ref) * self.follow_ratio
        if self.follow_invert:
            delta = -delta
        self.position_deg = self._follow_local_ref + delta
        self.target_deg = self.position_deg

    # -- time ------------------------------------------------------------------------

    def tick(self, dt: float) -> None:
        """Advance velocity-mode motion by ``dt`` seconds."""
        if self.mode == Mode.VELOCITY and self.enabled and not self.estopped:
            self.position_deg += self.velocity_deg_s * dt
            self._emit(Tel.POSITION, struct.pack("<d", self.position_deg))


class SimNetwork(Transport):
    """Transport whose bus is populated by :class:`SimNode` objects."""

    def __init__(self, node_ids: Iterable[int] = (1,)):
        super().__init__()
        self.nodes: Dict[int, SimNode] = {}
        for nid in node_ids:
            self.add_node(nid)

    def add_node(self, node_id: int) -> SimNode:
        node = SimNode(node_id, self)
        self.nodes[node_id] = node
        return node

    def node(self, node_id: int) -> SimNode:
        return self.nodes[node_id]

    def tick(self, dt: float) -> None:
        """Advance all velocity-mode nodes by ``dt`` seconds."""
        for node in list(self.nodes.values()):
            node.tick(dt)

    # -- Transport interface -----------------------------------------------------

    def send(self, frame: Frame) -> None:
        for node in list(self.nodes.values()):
            node.handle(frame)

    def close(self) -> None:
        pass

    # -- called by SimNode ----------------------------------------------------------

    def _emit_from_node(self, node_id: int, msg_id: int, data: bytes) -> None:
        frame = Frame(make_can_id(node_id, msg_id), False, data)
        # Other nodes see the frame too (leader/follower runs on-bus).
        for node in list(self.nodes.values()):
            if node.node_id != node_id:
                node.handle(frame)
        self._deliver(frame)
