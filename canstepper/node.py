"""StepperNode — the full command/telemetry surface of one board."""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Union

from .exceptions import (
    EStopActive,
    HomingFailed,
    LimitViolation,
    NodeFault,
    ParamRejected,
    RequestTimeout,
    UnknownParam,
)
from .protocol import (
    Cmd,
    EndstopAction,
    Event,
    HomeMethod,
    Param,
    ParamStatus,
    StandstillMode,
    Tel,
    decode_param_value,
    encode_param_value,
    resolve_param,
)
from .telemetry import (
    CanHealth,
    DriverStatus,
    FollowStatus,
    NodeState,
    NodeStatus,
    PidStatus,
)

_HOME_METHODS = {
    "set_zero": HomeMethod.SET_ZERO,
    "endstop": HomeMethod.ENDSTOP,
    "stallguard": HomeMethod.STALLGUARD,
    "sensorless": HomeMethod.STALLGUARD,
}


@dataclass
class SpeedLimits:
    """Host-side speed policy for one node (degrees/second).

    These are *your* limits enforced by the library before a command is
    sent; the firmware itself applies no motion clamps.
    """

    min_speed: Optional[float] = None
    max_speed: Optional[float] = None

    def check(self, speed_deg_s: float) -> None:
        mag = abs(speed_deg_s)
        if mag == 0.0:
            return
        if self.max_speed is not None and mag > self.max_speed:
            raise LimitViolation(
                f"requested {mag:.3f} deg/s exceeds max_speed {self.max_speed:.3f}"
            )
        if self.min_speed is not None and mag < self.min_speed:
            raise LimitViolation(
                f"requested {mag:.3f} deg/s is below min_speed {self.min_speed:.3f}"
            )


class StepperNode:
    """High-level interface to a single CANStepper board.

    Configuration setters return ``self`` so calls can be chained::

        node.set_run_current(40).set_microsteps(16).enable()
    """

    def __init__(self, bus, node_id: int):
        self._bus = bus
        self.node_id = node_id
        self.limits = SpeedLimits()
        #: default timeout for request/verify round-trips, seconds
        self.timeout = 1.0

    # -- cached state ----------------------------------------------------------

    @property
    def state(self) -> NodeState:
        """Last-known telemetry (never touches the bus; check ``.age()``)."""
        with self._bus._lock:
            st = self._bus.states.get(self.node_id)
            if st is None:
                st = self._bus.states[self.node_id] = NodeState(self.node_id)
            return st

    # -- power / safety ----------------------------------------------------------

    def enable(self) -> "StepperNode":
        """Energize the driver (also clears an e-stop latch)."""
        self._cmd(Cmd.ENABLE, struct.pack("<B", 1))
        return self

    def disable(self) -> "StepperNode":
        """De-energize the driver (the shaft can move freely)."""
        self._cmd(Cmd.ENABLE, struct.pack("<B", 0))
        return self

    def estop(self) -> "StepperNode":
        """EMERGENCY STOP: instant halt + driver disable, latched until
        ``enable()`` is called again."""
        self._cmd(Cmd.ESTOP)
        return self

    def stop(self) -> "StepperNode":
        """Controlled stop using the configured deceleration; stays enabled."""
        self._cmd(Cmd.STOP)
        return self

    def ping(self, timeout: Optional[float] = None) -> NodeStatus:
        """Round-trip liveness check; returns the node's status."""
        payload = self._bus.request(
            self.node_id, Tel.STATUS, timeout or self.timeout
        )
        return NodeStatus.decode(payload)

    # -- motion --------------------------------------------------------------------

    def move_to(
        self,
        degrees: float,
        blocking: bool = False,
        timeout: float = 30.0,
    ) -> "StepperNode":
        """Absolute move in degrees. With ``blocking=True`` waits for the
        firmware's move-done event and raises :class:`NodeFault` on fault."""
        self._guard_estop()
        if blocking:
            with self._bus.arm_events(
                self.node_id, [Event.MOVE_DONE, Event.FAULT, Event.ESTOP]
            ) as waiter:
                self._cmd(Cmd.MOVE_ABS, struct.pack("<d", float(degrees)))
                self._wait_motion_event(waiter, timeout, "move")
        else:
            self._cmd(Cmd.MOVE_ABS, struct.pack("<d", float(degrees)))
        return self

    def move_by(
        self,
        degrees: float,
        blocking: bool = False,
        timeout: float = 30.0,
    ) -> "StepperNode":
        """Relative move in degrees (relative to the active target if any)."""
        self._guard_estop()
        if blocking:
            with self._bus.arm_events(
                self.node_id, [Event.MOVE_DONE, Event.FAULT, Event.ESTOP]
            ) as waiter:
                self._cmd(Cmd.MOVE_REL, struct.pack("<d", float(degrees)))
                self._wait_motion_event(waiter, timeout, "move")
        else:
            self._cmd(Cmd.MOVE_REL, struct.pack("<d", float(degrees)))
        return self

    def run(self, deg_per_sec: float) -> "StepperNode":
        """Continuous rotation at the given signed velocity (deg/s)."""
        self._guard_estop()
        self.limits.check(deg_per_sec)
        self._cmd(Cmd.MOVE_VEL, struct.pack("<f", float(deg_per_sec)))
        return self

    def set_zero(self) -> "StepperNode":
        """Stop and define the current shaft position as 0 degrees."""
        self._cmd(Cmd.SET_ZERO)
        return self

    def set_logical_position(self, degrees: float) -> "StepperNode":
        """Relabel the current shaft position without physically moving it.

        This is useful for aligning coordinate systems on two motors that are
        already mechanically coupled. The encoder offset and internal step
        counter are re-anchored to ``degrees``.
        """
        self._cmd(Cmd.SET_POSITION, struct.pack("<d", float(degrees)))
        return self

    def home(
        self,
        method: Union[str, HomeMethod] = HomeMethod.ENDSTOP,
        direction: int = -1,
        speed_deg_s: float = 30.0,
        blocking: bool = True,
        timeout: float = 60.0,
    ) -> "StepperNode":
        """Run the firmware homing routine.

        ``method``: ``"set_zero"`` | ``"endstop"`` | ``"stallguard"`` (or a
        :class:`HomeMethod`). Homing current/backoff/timeout come from the
        node parameters (``homing_current``, ``homing_backoff``,
        ``homing_timeout_ms``).
        """
        self._guard_estop()
        if isinstance(method, str):
            try:
                method = _HOME_METHODS[method.lower()]
            except KeyError:
                raise ValueError(
                    f"unknown homing method {method!r}; "
                    f"use one of {sorted(_HOME_METHODS)}"
                ) from None
        payload = struct.pack(
            "<Bbf", int(method), 1 if direction >= 0 else -1, float(speed_deg_s)
        )
        if not blocking:
            self._cmd(Cmd.HOME, payload)
            return self
        with self._bus.arm_events(
            self.node_id,
            [Event.HOMING_DONE, Event.HOMING_FAILED, Event.FAULT, Event.ESTOP],
        ) as waiter:
            self._cmd(Cmd.HOME, payload)
            hit = waiter.wait(timeout)
        if hit is None:
            raise RequestTimeout(self.node_id, "homing completion", timeout)
        event, detail, _data = hit
        if event != Event.HOMING_DONE:
            raise HomingFailed(
                self.node_id, detail, f"node {self.node_id}: homing failed ({event.name})"
            )
        return self

    def wait_settled(self, timeout: float = 30.0, poll: float = 0.02) -> "StepperNode":
        """Poll cached PID/status telemetry until the axis reports settled."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pid = self.state.pid
            if pid is not None:
                if pid.state == 3:
                    raise NodeFault(self.node_id, int(pid.fault))
                if pid.state == 2:
                    return self
            status = self.state.status
            if status is not None and not status.moving and pid is None:
                return self
            time.sleep(poll)
        raise RequestTimeout(self.node_id, "settle", timeout)

    # -- parameters ----------------------------------------------------------------

    def set_param(
        self,
        param: Union[Param, int, str],
        value: Union[int, float, bool],
        timeout: Optional[float] = None,
    ) -> "StepperNode":
        """Set one firmware parameter and verify the node's acknowledgement.

        Raises :class:`ParamRejected` / :class:`UnknownParam` if the node
        refuses the value (its sanity range mirrors ``protocol.PARAMS``).
        """
        d = resolve_param(param)
        payload = bytes([int(d.param)]) + encode_param_value(d, value)
        reply = self._bus.request_param_frame(
            self.node_id, Cmd.SET_PARAM, payload, int(d.param),
            timeout or self.timeout,
        )
        status = reply[1]
        if status == ParamStatus.UNKNOWN:
            raise UnknownParam(f"node {self.node_id}: unknown param {d.param.name}")
        if status == ParamStatus.REJECTED:
            raise ParamRejected(
                f"node {self.node_id}: rejected {d.param.name}={value!r} "
                f"(valid {d.minv}..{d.maxv})"
            )
        return self

    def get_param(
        self,
        param: Union[Param, int, str],
        timeout: Optional[float] = None,
    ) -> Union[int, float]:
        """Read one firmware parameter."""
        d = resolve_param(param)
        reply = self._bus.request_param_frame(
            self.node_id, Cmd.GET_PARAM, bytes([int(d.param)]), int(d.param),
            timeout or self.timeout,
        )
        if reply[1] == ParamStatus.UNKNOWN:
            raise UnknownParam(f"node {self.node_id}: unknown param {d.param.name}")
        return decode_param_value(d, reply[2:6])

    def save_config(self) -> "StepperNode":
        """Persist the whole parameter table to the node's flash (applied at
        every power-on)."""
        self._cmd(Cmd.SAVE_CONFIG)
        return self

    def load_defaults(self) -> "StepperNode":
        """Factory-reset all parameters (the node keeps its ID)."""
        self._cmd(Cmd.LOAD_DEFAULTS)
        return self

    # -- named convenience setters ---------------------------------------------------

    def set_node_id(self, new_id: int, save: bool = True) -> "StepperNode":
        """Re-address the node (optionally persisting immediately).

        Firmware applies ``NODE_ID`` *before* sending the PARAM acknowledgement,
        so the ACK is emitted under the **new** CAN id. Waiting on the old id
        (as a normal ``set_param`` does) therefore times out. We fire SET_PARAM
        without that wait, then SAVE_CONFIG addressed to the new id.
        """
        nid = int(new_id)
        if not 1 <= nid <= 31:
            raise ValueError(f"node_id must be 1-31, got {nid}")
        d = resolve_param(Param.NODE_ID)
        payload = bytes([int(d.param)]) + encode_param_value(d, nid)
        old_id = self.node_id
        self._bus.send_command(old_id, Cmd.SET_PARAM, payload)
        time.sleep(0.05)
        self.node_id = nid
        if save:
            self._cmd(Cmd.SAVE_CONFIG)
        with self._bus._lock:
            self._bus._nodes.pop(old_id, None)
            self._bus._nodes[nid] = self
        return self

    def set_microsteps(self, microsteps: int) -> "StepperNode":
        """Microstepping (power of two, 1-256)."""
        return self.set_param(Param.MICROSTEPS, microsteps)

    def set_steps_per_rev(self, steps: int) -> "StepperNode":
        """Full steps per motor revolution (200 for a 1.8 deg motor)."""
        return self.set_param(Param.STEPS_PER_REV, steps)

    def set_run_current(self, percent: int) -> "StepperNode":
        """Motor run current as percent of the driver maximum."""
        return self.set_param(Param.RUN_CURRENT, percent)

    def set_hold_current(self, percent: int) -> "StepperNode":
        """Standstill (hold) current as percent of the driver maximum."""
        return self.set_param(Param.HOLD_CURRENT, percent)

    def set_direction(self, inverted: bool) -> "StepperNode":
        """Invert the motor's physical rotation direction."""
        return self.set_param(Param.INVERT_DIR, 1 if inverted else 0)

    def set_max_speed(self, deg_per_sec: float) -> "StepperNode":
        """Cruise speed used by position moves (``MAX_SPEED``)."""
        return self.set_param(Param.MAX_SPEED, deg_per_sec)

    def set_acceleration(self, deg_per_sec2: float) -> "StepperNode":
        """Acceleration/deceleration ramp for position moves (``ACCELERATION``)."""
        return self.set_param(Param.ACCELERATION, deg_per_sec2)

    def set_cl_max_speed(self, deg_per_sec: float) -> "StepperNode":
        """Closed-loop trapezoid cruise speed for MOVE_ABS/REL (``CL_MAX_SPEED``).

        Firmware ≥1.2 plans a rest-to-rest trapezoidal trajectory
        (accel → cruise at this speed → decel) and tracks it with velocity
        feedforward + a light PID. Keep :meth:`set_max_speed` ≥ this value.
        See :meth:`configure_closed_loop_speed` for a one-shot setup.
        """
        return self.set_param(Param.CL_MAX_SPEED, deg_per_sec)

    def set_cl_max_accel(self, deg_per_sec2: float) -> "StepperNode":
        """Trapezoid accel/decel for closed-loop position moves
        (``CL_MAX_ACCEL``, default 2880 deg/s²)."""
        return self.set_param(Param.CL_MAX_ACCEL, deg_per_sec2)

    def configure_closed_loop_speed(
        self,
        cruise_deg_s: float,
        *,
        accel_deg_s2: Optional[float] = None,
        accel_factor: float = 4.0,
        run_current: Optional[int] = None,
        microsteps: Optional[int] = None,
        stealthchop: bool = False,
        kp: float = 12.0,
        ki: float = 0.3,
        kd: float = 0.10,
        tolerance_deg: float = 0.35,
        persist: bool = False,
    ) -> "StepperNode":
        """One-shot setup for high-speed closed-loop trapezoid moves.

        Firmware ≥1.2 uses a planned trapezoidal profile with velocity
        feedforward (``v = v_ff + PID(r − encoder)``). This helper sets the
        trap ceilings (``cl_max_speed`` / ``cl_max_accel`` and matching
        ``max_speed`` / ``acceleration``), enables the encoder loop, loads
        tracking PID gains, and optionally raises run current / drops
        microsteps / forces SpreadCycle for more torque at speed.

        Parameters
        ----------
        cruise_deg_s:
            Trap cruise speed in deg/s (÷6 for RPM). Physical ceiling is set
            by supply voltage, current, and microstepping. Use
            ``examples/closed_loop_speed.py`` or ``tools/cl_speed_validate.py``
            to find your motor's limit.
        accel_deg_s2:
            Trap accel/decel; default is ``cruise_deg_s * accel_factor``.
        run_current, microsteps, stealthchop:
            Optional driver prep. ``stealthchop=False`` (SpreadCycle) is
            recommended above ~180 deg/s.
        persist:
            If True, ``save_config()`` so the tune survives reboot.
        """
        cruise = float(cruise_deg_s)
        if cruise <= 0:
            raise ValueError("cruise_deg_s must be > 0")
        accel = float(accel_deg_s2) if accel_deg_s2 is not None else cruise * float(accel_factor)
        if run_current is not None:
            self.set_run_current(int(run_current))
        if microsteps is not None:
            self.set_microsteps(int(microsteps))
        self.set_stealthchop(bool(stealthchop))
        (self.set_closed_loop(True)
         .set_cl_max_speed(cruise)
         .set_cl_max_accel(accel)
         .set_max_speed(cruise)
         .set_acceleration(accel)
         .set_pid(kp, ki, kd, tolerance_deg=tolerance_deg))
        if persist:
            self.save_config()
        return self

    def set_speed_limits(
        self,
        min_speed: Optional[float] = None,
        max_speed: Optional[float] = None,
    ) -> "StepperNode":
        """Host-side velocity policy for ``run()`` (see :class:`SpeedLimits`)."""
        self.limits = SpeedLimits(min_speed, max_speed)
        return self

    def set_closed_loop(
        self,
        enabled: bool = True,
        max_speed: Optional[float] = None,
        max_accel: Optional[float] = None,
    ) -> "StepperNode":
        """Toggle the encoder position loop; optionally set its trap
        cruise/accel (see :meth:`set_cl_max_speed` / :meth:`set_cl_max_accel`,
        or :meth:`configure_closed_loop_speed`). Defaults are 720 deg/s /
        2880 deg/s² on firmware ≥1.2 — configurable, not hard clamps."""
        self.set_param(Param.CLOSED_LOOP, 1 if enabled else 0)
        if max_speed is not None:
            self.set_param(Param.CL_MAX_SPEED, max_speed)
        if max_accel is not None:
            self.set_param(Param.CL_MAX_ACCEL, max_accel)
        return self

    def set_pid(
        self,
        kp: float,
        ki: float,
        kd: float,
        tolerance_deg: Optional[float] = None,
    ) -> "StepperNode":
        """Closed-loop tracking PID gains (trim around velocity feedforward).

        On firmware ≥1.2 the move is driven by a trapezoidal ``v_ff``; these
        gains only correct residual tracking error (units: deg/s per deg).
        """
        self.set_param(Param.PID_KP, kp)
        self.set_param(Param.PID_KI, ki)
        self.set_param(Param.PID_KD, kd)
        if tolerance_deg is not None:
            self.set_param(Param.PID_TOLERANCE, tolerance_deg)
        return self

    def set_stall_threshold(self, threshold: int) -> "StepperNode":
        """StallGuard sensitivity (SGTHRS, 0-255; higher = more sensitive)."""
        return self.set_param(Param.STALL_THRESHOLD, threshold)

    def set_standstill_mode(self, mode: Union[StandstillMode, int]) -> "StepperNode":
        return self.set_param(Param.STANDSTILL_MODE, int(mode))

    def set_stealthchop(self, enabled: bool = True) -> "StepperNode":
        return self.set_param(Param.STEALTHCHOP, 1 if enabled else 0)

    def set_report_rates(self, fast_hz: int = 10, slow_hz: int = 1) -> "StepperNode":
        """Telemetry rates: fast = position/motion/PID, slow = status/env."""
        self.set_param(Param.FAST_RATE_HZ, fast_hz)
        self.set_param(Param.SLOW_RATE_HZ, slow_hz)
        return self

    def set_enable_on_boot(self, enabled: bool = True) -> "StepperNode":
        return self.set_param(Param.ENABLE_ON_BOOT, 1 if enabled else 0)

    def set_zero_on_boot(self, enabled: bool = True) -> "StepperNode":
        return self.set_param(Param.ZERO_ON_BOOT, 1 if enabled else 0)

    def configure_endstop(
        self,
        enabled: bool = True,
        active_high: bool = False,
        action: Union[EndstopAction, int] = EndstopAction.STOP,
    ) -> "StepperNode":
        """Configure the IO8 endstop input.

        ``action``: REPORT (telemetry only), STOP, or STOP_AND_ZERO.
        Note the boot caveat: IO8 is an ESP32-C3 strapping pin — the switch
        must leave it HIGH at power-on (use a normally-open switch to GND,
        or wire NC switches through a series diode).
        """
        self.set_param(Param.ENDSTOP_ENABLE, 1 if enabled else 0)
        self.set_param(Param.ENDSTOP_ACTIVE_HIGH, 1 if active_high else 0)
        self.set_param(Param.ENDSTOP_ACTION, int(action))
        return self

    def configure_homing(
        self,
        current_percent: Optional[int] = None,
        backoff_deg: Optional[float] = None,
        timeout_ms: Optional[int] = None,
    ) -> "StepperNode":
        """Homing behavior: reduced current, post-trigger backoff, timeout."""
        if current_percent is not None:
            self.set_param(Param.HOMING_CURRENT, current_percent)
        if backoff_deg is not None:
            self.set_param(Param.HOMING_BACKOFF, backoff_deg)
        if timeout_ms is not None:
            self.set_param(Param.HOMING_TIMEOUT_MS, timeout_ms)
        return self

    # -- leader/follower ---------------------------------------------------------------

    def follow(
        self,
        leader: Union["StepperNode", int],
        ratio: float = 1.0,
        invert: bool = False,
        encoder_corrected: bool = True,
    ) -> "StepperNode":
        """Track another node's shaft on-bus (no host in the loop).

        The first leader position frame after enabling captures both zero
        references, so the follower never jumps on engage.
        """
        leader_id = leader.node_id if isinstance(leader, StepperNode) else int(leader)
        if leader_id == self.node_id:
            raise ValueError("a node cannot follow itself")
        if not 1 <= leader_id <= 31:
            raise ValueError("leader id must be 1-31")
        if ratio <= 0:
            raise ValueError("ratio must be positive")
        flags = (1 if invert else 0) | (2 if encoder_corrected else 0)
        self._cmd(
            Cmd.FOLLOW,
            struct.pack("<BBBBf", 1, leader_id, flags, 0, float(ratio)),
        )
        return self

    def unfollow(self) -> "StepperNode":
        self._cmd(Cmd.FOLLOW, struct.pack("<BBBBf", 0, 1, 0, 0, 1.0))
        return self

    def follow_sync(self) -> "StepperNode":
        """Recapture the leader/local zero offset on the next leader frame."""
        self._cmd(Cmd.FOLLOW_SYNC)
        return self

    # -- blocking telemetry getters -------------------------------------------------------

    def get_status(self, timeout: Optional[float] = None) -> NodeStatus:
        return NodeStatus.decode(self._req(Tel.STATUS, timeout))

    def get_position(self, timeout: Optional[float] = None) -> float:
        """Encoder angle in degrees (multi-turn)."""
        return struct.unpack("<d", self._req(Tel.POSITION, timeout)[:8])[0]

    def get_target(self, timeout: Optional[float] = None) -> float:
        return struct.unpack("<d", self._req(Tel.TARGET, timeout)[:8])[0]

    def get_motion(self, timeout: Optional[float] = None) -> Tuple[float, float]:
        """``(velocity deg/s, position error deg)``."""
        return struct.unpack("<ff", self._req(Tel.MOTION, timeout)[:8])

    def get_velocity(self, timeout: Optional[float] = None) -> float:
        return self.get_motion(timeout)[0]

    def get_encoder_counts(self, timeout: Optional[float] = None) -> int:
        return struct.unpack("<q", self._req(Tel.ENC_COUNTS, timeout)[:8])[0]

    def get_stallguard(self, timeout: Optional[float] = None) -> int:
        """StallGuard result (also available via :meth:`get_driver_status`)."""
        return self.get_driver_status(timeout).stallguard

    def get_driver_status(self, timeout: Optional[float] = None) -> DriverStatus:
        """Full TMC2209 diagnostics from ``TEL_DRIVER`` (OTPW/OT, shorts, CS, …).

        Requires firmware ≥1.4 for the complete 8-byte payload. Older boards
        still return ``stallguard`` + ``uart_ok``.
        """
        raw = self._req(Tel.DRIVER, timeout)
        status = DriverStatus.decode(raw)
        try:
            self.state.driver = status
            self.state.stallguard = status.stallguard
        except Exception:
            pass
        return status

    def get_env(self, timeout: Optional[float] = None) -> Tuple[float, float]:
        """``(mcu temperature C, bus voltage V)``."""
        return struct.unpack("<ff", self._req(Tel.ENV, timeout)[:8])

    def get_pid_status(self, timeout: Optional[float] = None) -> PidStatus:
        return PidStatus.decode(self._req(Tel.PID_STATUS, timeout))

    def get_follow_status(self, timeout: Optional[float] = None) -> FollowStatus:
        return FollowStatus.decode(self._req(Tel.FOLLOW_STATUS, timeout))

    def get_can_health(self, timeout: Optional[float] = None) -> CanHealth:
        return CanHealth.decode(self._req(Tel.CAN_HEALTH, timeout))

    def get_firmware_version(self, timeout: Optional[float] = None) -> str:
        return self.get_status(timeout).firmware

    # -- internals ---------------------------------------------------------------------

    def _cmd(self, cmd: Cmd, data: bytes = b"") -> None:
        self._bus.send_command(self.node_id, cmd, data)

    def _req(self, tel: Tel, timeout: Optional[float]) -> bytes:
        return self._bus.request(self.node_id, tel, timeout or self.timeout)

    def _guard_estop(self) -> None:
        status = self.state.status
        if status is not None and status.estopped:
            raise EStopActive(
                f"node {self.node_id} is e-stop latched; call enable() first"
            )

    def _wait_motion_event(self, waiter, timeout: float, what: str) -> None:
        hit = waiter.wait(timeout)
        if hit is None:
            raise RequestTimeout(self.node_id, f"{what} completion", timeout)
        event, detail, _data = hit
        if event == Event.FAULT:
            raise NodeFault(self.node_id, detail)
        if event == Event.ESTOP:
            raise EStopActive(f"node {self.node_id} e-stopped during {what}")

    def __repr__(self) -> str:
        return f"<StepperNode {self.node_id}>"
