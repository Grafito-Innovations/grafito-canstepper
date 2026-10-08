"""Coordinated multi-motor motion: MotionGroup, DualMotorAxis,
IndependentDualAxis, Cartesian, CoreXY.

Coordination model (v1): point-to-point *time-synchronized* moves. The host
computes one common move duration and scales each motor's cruise speed and
acceleration so all motors start together and finish together. Start skew is
one CAN frame per motor (~0.15 ms at 1 Mbps) plus host scheduling; this is
excellent for gantries, pick-and-place and feeders, but it is not a
continuous-path (contouring) controller.

Kinematics (Klipper-inspired naming):

* :class:`Cartesian` — independent X/Y/(Z) steppers; motor travel = workspace
  travel on each axis (``kinematics: cartesian`` style).
* :class:`CoreXY` — coupled belts ``A=X+Y``, ``B=X-Y``.
* :class:`DualMotorAxis` — two motors on one linear axis (e.g. dual-Z gantry)
  via firmware leader/follower (no host in the coupling loop).
* :class:`IndependentDualAxis` — two motors each close their own loop (or
  open-loop steps); host **encoder-gates** the next pass until *both*
  encoders are within tolerance of the commanded mm target. Proven path for
  dual SFU1204 ball-screw frames.
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

from .axis import Axis
from .exceptions import EncoderGateTimeout, NodeFault, RequestTimeout
from .node import StepperNode
from .protocol import Event, Param

# StallGuardEstop is typed as object in method signatures to avoid a circular
# import; runtime accepts canstepper.stall_protect.StallGuardEstop.


# ---------------------------------------------------------------------------
# Trapezoidal profile math
#
# Mirrors the firmware ≥1.2 closed-loop planner (rest-to-rest trap / triangle)
# and the host MotionGroup time-sync planner. Velocity diagram::
#
#     v
#     ^
#     |    /--------\        trapezoid (reaches cruise)
#     |   /          \
#     |  /            \
#     +------------------> t
#       accel cruise decel
#
#     v
#     ^
#     |    /\                triangle (short move)
#     |   /  \
#     +----------> t
# ---------------------------------------------------------------------------

def trapezoid_time(distance: float, speed: float, accel: float) -> float:
    """Duration of a trapezoidal (or triangular) move."""
    d = abs(distance)
    if d == 0.0 or speed <= 0.0 or accel <= 0.0:
        return 0.0
    if d >= speed * speed / accel:          # reaches cruise speed
        return d / speed + speed / accel
    return 2.0 * math.sqrt(d / accel)       # triangular profile


def plan_trapezoid(
    distance: float, speed: float, accel: float
) -> Tuple[float, float, float, float, float]:
    """Plan a rest-to-rest trap/triangle.

    Returns ``(v_peak, t_acc, t_cruise, t_dec, t_total)``.
    Matches firmware ``trajPlan()`` (fw ≥1.2).
    """
    d = abs(distance)
    if d <= 0.0 or speed <= 0.0 or accel <= 0.0:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    d_acc_full = (speed * speed) / (2.0 * accel)
    if 2.0 * d_acc_full >= d:
        v_peak = math.sqrt(accel * d)
        t_acc = v_peak / accel
        return v_peak, t_acc, 0.0, t_acc, 2.0 * t_acc
    t_acc = speed / accel
    t_cruise = (d - 2.0 * d_acc_full) / speed
    return speed, t_acc, t_cruise, t_acc, 2.0 * t_acc + t_cruise


def eval_trapezoid(
    distance: float,
    speed: float,
    accel: float,
    t: float,
) -> Tuple[float, float]:
    """Evaluate position along the move and feedforward velocity at time ``t``.

    ``distance`` is signed. Returns ``(s_along_signed, v_ff_signed)`` where
    ``s`` runs from 0 to ``distance``. Used by tests and host-side preview;
    firmware evaluates the same equations in ``trajAdvance()``.
    """
    sign = 1.0 if distance >= 0.0 else -1.0
    d = abs(distance)
    v_peak, t_acc, t_cruise, t_dec, t_tot = plan_trapezoid(d, speed, accel)
    if d <= 0.0 or t_tot <= 0.0:
        return 0.0, 0.0
    if t <= 0.0:
        return 0.0, 0.0
    if t >= t_tot:
        return distance, 0.0

    d_acc = 0.5 * accel * t_acc * t_acc
    if t <= t_acc:
        v = accel * t
        s = 0.5 * accel * t * t
    elif t <= t_acc + t_cruise:
        tc = t - t_acc
        v = v_peak
        s = d_acc + v_peak * tc
    else:
        td = t - t_acc - t_cruise
        v = max(0.0, v_peak - accel * td)
        s = d_acc + v_peak * t_cruise + v_peak * td - 0.5 * accel * td * td
    s = min(d, max(0.0, s))
    return sign * s, sign * v


def profile_for_time(
    distance: float, duration: float, accel: float
) -> Tuple[float, float]:
    """Pick ``(speed, accel)`` that covers ``distance`` in ``duration``.

    Keeps the given acceleration when possible (trapezoid); raises it only
    when the move is too short to reach any cruise speed in time.
    """
    d = abs(distance)
    if d == 0.0 or duration <= 0.0:
        return 0.0, accel
    disc = accel * accel * duration * duration - 4.0 * accel * d
    if disc >= 0.0:
        speed = (accel * duration - math.sqrt(disc)) / 2.0
        if speed > 0.0:
            return speed, accel
    # Triangular fallback: v_peak = 2d/T with a = 4d/T².
    return 2.0 * d / duration, 4.0 * d / (duration * duration)


# ---------------------------------------------------------------------------
# MotionGroup
# ---------------------------------------------------------------------------

class MotionGroup:
    """Time-synchronized point-to-point moves across several axes."""

    def __init__(self, axes: Sequence[Axis]):
        if not axes:
            raise ValueError("MotionGroup needs at least one axis")
        self.axes: List[Axis] = list(axes)
        self._by_name = {a.name: a for a in self.axes}

    def axis(self, key: Union[str, Axis]) -> Axis:
        if isinstance(key, Axis):
            return key
        return self._by_name[key]

    def move_to(
        self,
        targets: Dict[Union[str, Axis], float],
        speeds: Optional[Dict[Union[str, Axis], float]] = None,
        blocking: bool = True,
        timeout: float = 120.0,
    ) -> float:
        """Move each axis to its target (axis units), all finishing together.

        ``speeds`` optionally caps per-axis speed (units/s); otherwise the
        axis's ``max_speed`` or the node's current MAX_SPEED parameter is
        used. Returns the planned move duration in seconds.
        """
        plan = []  # (axis, target, distance, cap_speed, accel) in axis units
        for key, target in targets.items():
            ax = self.axis(key)
            current = ax.get_position()
            distance = target - current
            cap = None
            if speeds and key in speeds:
                cap = speeds[key]
            elif speeds and ax in speeds:  # type: ignore[operator]
                cap = speeds[ax]           # type: ignore[index]
            if cap is None:
                cap = ax.max_speed
            if cap is None:
                cap = ax.deg_to_units(ax.node.get_param(Param.MAX_SPEED))
            accel = ax.deg_to_units(ax.node.get_param(Param.ACCELERATION))
            ax._check_motion_allowed(target, cap)
            plan.append((ax, target, distance, float(cap), float(accel)))

        duration = max(
            (trapezoid_time(d, v, a) for _, _, d, v, a in plan), default=0.0
        )
        if duration == 0.0:
            return 0.0

        moving = []
        for ax, target, distance, cap, accel in plan:
            if distance == 0.0:
                continue
            v, a = profile_for_time(distance, duration, accel)
            ax.node.set_param(Param.MAX_SPEED, ax.units_to_deg(v))
            ax.node.set_param(Param.ACCELERATION, ax.units_to_deg(a))
            moving.append((ax, target))

        # Arm completion waiters BEFORE dispatch so no event can be missed,
        # then send all move commands back-to-back.
        waiters = []
        if blocking:
            for ax, _t in moving:
                waiters.append(
                    ax.node._bus.arm_events(
                        ax.node.node_id,
                        [Event.MOVE_DONE, Event.FAULT, Event.ESTOP],
                    )
                )
        try:
            for ax, target in moving:
                ax.node.move_to(ax.units_to_deg(target))
            if blocking:
                for (ax, _t), waiter in zip(moving, waiters):
                    ax.node._wait_motion_event(waiter, timeout, "coordinated move")
        finally:
            for waiter in waiters:
                waiter.cancel()
        return duration

    def stop(self) -> None:
        for ax in self.axes:
            ax.stop()

    def estop(self) -> None:
        for ax in self.axes:
            ax.estop()


# ---------------------------------------------------------------------------
# DualMotorAxis — rigidly coupled pair (e.g. dual-Z gantry)
# ---------------------------------------------------------------------------

class DualMotorAxis(Axis):
    """Two motors driving one axis, coupled by firmware leader/follower.

    Commands go to the primary node; the secondary tracks the primary's
    encoder angle on the bus itself (optionally encoder-corrected). The
    host is not in the coupling loop.
    """

    def __init__(
        self,
        primary: StepperNode,
        secondary: StepperNode,
        rotation_distance: float,
        invert_secondary: bool = False,
        ratio: float = 1.0,
        encoder_corrected: bool = True,
        auto_couple: bool = True,
        **axis_kwargs,
    ):
        super().__init__(primary, rotation_distance, **axis_kwargs)
        self.secondary = secondary
        self.invert_secondary = invert_secondary
        self.ratio = ratio
        self.encoder_corrected = encoder_corrected
        if auto_couple:
            self.couple()

    def couple(self) -> "DualMotorAxis":
        """(Re)engage the follower coupling; zero offsets are captured from
        the next leader position frame, so the pair never jumps."""
        self.secondary.enable()
        self.secondary.follow(
            self.node,
            ratio=self.ratio,
            invert=self.invert_secondary,
            encoder_corrected=self.encoder_corrected,
        )
        return self

    def decouple(self) -> "DualMotorAxis":
        self.secondary.unfollow()
        return self

    def resync(self) -> "DualMotorAxis":
        """Recapture the leader/follower zero offset (e.g. after re-leveling)."""
        self.secondary.follow_sync()
        return self

    def enable(self) -> "DualMotorAxis":
        self.node.enable()
        self.secondary.enable()
        return self

    def disable(self) -> "DualMotorAxis":
        self.node.disable()
        self.secondary.disable()
        return self

    def estop(self) -> "DualMotorAxis":
        # Per-node frames; for whole-bus e-stop use bus.estop_all().
        self.node.estop()
        self.secondary.estop()
        return self

    def configure_both(self, fn) -> "DualMotorAxis":
        """Apply ``fn(node)`` to primary and secondary (e.g. currents)."""
        fn(self.node)
        fn(self.secondary)
        return self

    def __repr__(self) -> str:
        return (
            f"<DualMotorAxis {self.name!r} primary={self.node.node_id} "
            f"secondary={self.secondary.node_id}>"
        )


# ---------------------------------------------------------------------------
# IndependentDualAxis — dual screws, host encoder gate (no firmware follow)
# ---------------------------------------------------------------------------

class IndependentDualAxis:
    """Two motors commanded independently in linear units, gated on *both*
    encoders before the next pass.

    Unlike :class:`DualMotorAxis` (firmware leader/follower), each motor runs
    its own closed-loop (or open-loop) move. The host issues the same
    absolute mm target to both, optionally with one side's ``INVERT_DIR`` set
    for mirrored ball-screw mounting, then **blocks until both encoders**
    are within ``tol_mm`` of that target for ``settle_s`` seconds.

    This prevents the next reverse/straight leg from starting when one side
    finished its step train but has not physically reached the mm pose
    (the desync mode seen with pure ``moving=False`` gating).

    Typical dual SFU1204 frame (4 mm pitch)::

        dual = IndependentDualAxis(
            bus.node(1), bus.node(2),
            rotation_distance=4.0,
            invert_a=True,   # mirrored mounting
            invert_b=False,
            name="frame",
        )
        dual.configure_closed_loop(speed_mm_s=56.0)
        dual.enable().set_zero()
        dual.oscillate(stroke_mm=20.0, cycles=5, speed_mm_s=56.0)
    """

    def __init__(
        self,
        node_a: StepperNode,
        node_b: StepperNode,
        rotation_distance: float,
        invert_a: bool = True,
        invert_b: bool = False,
        name: str = "dual",
        tol_mm: float = 0.8,
        settle_s: float = 0.25,
        gate_timeout_s: float = 30.0,
    ):
        if rotation_distance <= 0:
            raise ValueError("rotation_distance must be positive")
        self.node_a = node_a
        self.node_b = node_b
        self.rotation_distance = float(rotation_distance)
        self.invert_a = bool(invert_a)
        self.invert_b = bool(invert_b)
        self.name = name
        self.tol_mm = float(tol_mm)
        self.settle_s = float(settle_s)
        self.gate_timeout_s = float(gate_timeout_s)
        self._closed_loop = False
        self._speed_mm_s = 36.0

    # -- units -----------------------------------------------------------------

    def units_to_deg(self, mm: float) -> float:
        return float(mm) / self.rotation_distance * 360.0

    def deg_to_units(self, deg: float) -> float:
        return float(deg) / 360.0 * self.rotation_distance

    def get_positions(self) -> Tuple[float, float]:
        """Round-trip encoder positions in mm for (a, b)."""
        return (
            self.deg_to_units(self.node_a.get_position()),
            self.deg_to_units(self.node_b.get_position()),
        )

    # -- lifecycle -------------------------------------------------------------

    def enable(self) -> "IndependentDualAxis":
        self.node_a.enable()
        self.node_b.enable()
        return self

    def disable(self) -> "IndependentDualAxis":
        self.node_a.disable()
        self.node_b.disable()
        return self

    def estop(self) -> "IndependentDualAxis":
        self.node_a.estop()
        self.node_b.estop()
        return self

    def set_zero(self) -> "IndependentDualAxis":
        """Define the current pose as 0 mm on both encoders."""
        self.node_a.set_zero()
        self.node_b.set_zero()
        return self

    def apply_direction(self) -> "IndependentDualAxis":
        """Write invert flags to both boards (call after enable)."""
        self.node_a.set_direction(inverted=self.invert_a)
        self.node_b.set_direction(inverted=self.invert_b)
        return self

    def configure_both(self, fn: Callable[[StepperNode], None]) -> "IndependentDualAxis":
        fn(self.node_a)
        fn(self.node_b)
        return self

    # -- driver setup ----------------------------------------------------------

    def configure_closed_loop(
        self,
        speed_mm_s: float = 36.0,
        *,
        run_current: int = 50,
        hold_current: int = 15,
        microsteps: int = 8,
        kp: float = 12.0,
        ki: float = 0.3,
        kd: float = 0.10,
        tolerance_deg: float = 12.0,
        accel_factor: float = 4.0,
        persist: bool = False,
    ) -> "IndependentDualAxis":
        """Enable encoder closed-loop on both motors for linear mm moves.

        ``speed_mm_s`` is converted via ``rotation_distance`` to trap cruise
        deg/s. Defaults match the dual SFU1204 bring-up that landed ±20 mm
        at 56–76 mm/s with both encoders in band.
        """
        cruise = self.units_to_deg(abs(speed_mm_s))
        if cruise <= 0:
            raise ValueError("speed_mm_s must be > 0")
        self._speed_mm_s = abs(float(speed_mm_s))
        self._closed_loop = True
        self.apply_direction()

        def _one(n: StepperNode) -> None:
            n.set_steps_per_rev(200)
            n.set_hold_current(int(hold_current))
            n.configure_closed_loop_speed(
                cruise,
                run_current=int(run_current),
                microsteps=int(microsteps),
                stealthchop=False,
                kp=float(kp),
                ki=float(ki),
                kd=float(kd),
                tolerance_deg=float(tolerance_deg),
                accel_factor=float(accel_factor),
                persist=bool(persist),
            )

        self.configure_both(_one)
        return self

    def configure_open_loop(
        self,
        speed_mm_s: float = 36.0,
        *,
        run_current: int = 55,
        hold_current: int = 15,
        microsteps: int = 16,
        accel_factor: float = 3.0,
    ) -> "IndependentDualAxis":
        """Open-loop dual setup (step generators only; still encoder-gated)."""
        cruise = self.units_to_deg(abs(speed_mm_s))
        accel = cruise * float(accel_factor)
        self._speed_mm_s = abs(float(speed_mm_s))
        self._closed_loop = False
        self.apply_direction()

        def _one(n: StepperNode) -> None:
            n.set_param(Param.CLOSED_LOOP, 0)
            n.set_run_current(int(run_current))
            n.set_hold_current(int(hold_current))
            n.set_steps_per_rev(200)
            n.set_microsteps(int(microsteps))
            n.set_stealthchop(False)
            n.set_max_speed(cruise)
            n.set_acceleration(accel)

        self.configure_both(_one)
        return self

    # -- encoder gate ----------------------------------------------------------

    def wait_both_at(
        self,
        target_mm: float,
        *,
        tol_mm: Optional[float] = None,
        settle_s: Optional[float] = None,
        timeout: Optional[float] = None,
        poll: Optional[Callable[[], None]] = None,
    ) -> Tuple[float, float]:
        """Block until both encoders are within ``tol_mm`` of ``target_mm``.

        Raises :class:`EncoderGateTimeout` if the window expires, or
        :class:`NodeFault` if either node reports a fault.
        Returns ``(pos_a_mm, pos_b_mm)``.

        ``poll`` (optional) is called each loop iteration — use
        :meth:`~canstepper.stall_protect.StallGuardEstop.raise_if_tripped`
        so a StallGuard estop aborts the wait promptly.
        """
        tol = self.tol_mm if tol_mm is None else float(tol_mm)
        settle = self.settle_s if settle_s is None else float(settle_s)
        limit = self.gate_timeout_s if timeout is None else float(timeout)
        target = float(target_mm)
        deadline = time.monotonic() + limit
        in_band_since: Optional[float] = None

        # Closed-loop: firmware tracks encoder to the commanded angle.
        # Open-loop: step targets and encoder can disagree in *sign* when
        # invert_dir is used (DIR flip). Dual SFU with invert_a often ends at
        # encoder ≈ −command_mm when host commanded +command_mm (both sides
        # still agree with each other). Accept either band so the gate matches
        # physical arrival.
        goals = [target]
        if not self._closed_loop and abs(target) > 1e-6:
            goals.append(-target)

        stuck_since: Optional[float] = None
        # If both step gens stop well short of target, treat as jam/stall even
        # when StallGuard DIAG did not fire (common at high current / low thr).
        stuck_limit_s = 2.0

        while time.monotonic() < deadline:
            if poll is not None:
                poll()
            pa = self.deg_to_units(self.node_a.get_position())
            pb = self.deg_to_units(self.node_b.get_position())
            sa, sb = self.node_a.ping(), self.node_b.ping()
            if int(sa.fault) != 0:
                raise NodeFault(self.node_a.node_id, int(sa.fault))
            if int(sb.fault) != 0:
                raise NodeFault(self.node_b.node_id, int(sb.fault))
            in_band = False
            for g in goals:
                if abs(pa - g) <= tol and abs(pb - g) <= tol:
                    in_band = True
                    break
            if in_band:
                now = time.monotonic()
                if in_band_since is None:
                    in_band_since = now
                elif now - in_band_since >= settle:
                    return pa, pb
                stuck_since = None
            else:
                in_band_since = None
                now = time.monotonic()
                if (not sa.moving) and (not sb.moving):
                    if stuck_since is None:
                        stuck_since = now
                    elif (now - stuck_since) >= stuck_limit_s:
                        raise EncoderGateTimeout(
                            f"{self.name}: both motors stopped short of target "
                            f"(possible jam without StallGuard event) "
                            f"a={pa:+.3f} b={pb:+.3f} mm want [{', '.join(f'{g:+.1f}' for g in goals)}]",
                            target_mm=target,
                            positions_mm=(pa, pb),
                            timeout=now - (deadline - limit),
                        )
                else:
                    stuck_since = None
            time.sleep(0.04)

        pa, pb = self.get_positions()
        goal_txt = " or ".join(f"{g:+.3f}" for g in goals)
        raise EncoderGateTimeout(
            f"{self.name}: encoders not both within {tol} mm of [{goal_txt}] "
            f"within {limit:.1f}s (a={pa:+.3f} b={pb:+.3f} mm)",
            target_mm=target,
            positions_mm=(pa, pb),
            timeout=limit,
        )

    # -- motion ----------------------------------------------------------------

    def move_to(
        self,
        target_mm: float,
        *,
        speed_mm_s: Optional[float] = None,
        wait_encoders: bool = True,
        tol_mm: Optional[float] = None,
        timeout: Optional[float] = None,
        poll: Optional[Callable[[], None]] = None,
    ) -> Tuple[float, float]:
        """Command both motors to the same absolute mm target.

        If ``wait_encoders`` is True (default), blocks until both encoders
        are in band (or raises :class:`EncoderGateTimeout`). If False,
        returns immediately after issuing the moves (fire-and-forget).

        ``poll`` is forwarded to :meth:`wait_both_at` when waiting.
        """
        if speed_mm_s is not None and speed_mm_s > 0:
            deg_s = self.units_to_deg(speed_mm_s)
            if self._closed_loop:
                self.node_a.set_cl_max_speed(deg_s).set_max_speed(deg_s)
                self.node_b.set_cl_max_speed(deg_s).set_max_speed(deg_s)
            else:
                self.node_a.set_max_speed(deg_s)
                self.node_b.set_max_speed(deg_s)
            self._speed_mm_s = float(speed_mm_s)

        deg = self.units_to_deg(target_mm)
        self.node_a.move_to(deg, blocking=False)
        self.node_b.move_to(deg, blocking=False)
        if not wait_encoders:
            return self.get_positions()
        return self.wait_both_at(
            target_mm, tol_mm=tol_mm, timeout=timeout, poll=poll
        )

    def move_to_protected(
        self,
        target_mm: float,
        protect: "object",
        *,
        speed_mm_s: Optional[float] = None,
        tol_mm: Optional[float] = None,
        timeout: Optional[float] = None,
        settle_s: float = 0.4,
        min_arm_mm: float = 3.0,
    ) -> Tuple[float, float]:
        """``move_to`` with StallGuard host estop armed after free travel.

        ``protect`` is a :class:`~canstepper.stall_protect.StallGuardEstop`
        already :meth:`~canstepper.stall_protect.StallGuardEstop.attach`ed.
        Disarms around start, arms after ``settle_s`` and ``min_arm_mm`` of
        progress toward the target, then waits with trip polling.
        """
        start_a, start_b = self.get_positions()
        start_mean = 0.5 * (start_a + start_b)
        target = float(target_mm)

        protect.disarm()
        self.move_to(
            target,
            speed_mm_s=speed_mm_s,
            wait_encoders=False,
            tol_mm=tol_mm,
            timeout=timeout,
        )

        def _progress() -> float:
            pa, pb = self.get_positions()
            # Distance traveled from leg start toward target (mean of both).
            cur = 0.5 * (pa + pb)
            return abs(cur - start_mean)

        protect.arm_after(
            settle_s,
            min_travel_mm=float(min_arm_mm),
            position_fn=_progress,
            timeout_s=max(5.0, abs(target - start_mean) / max(self._speed_mm_s, 0.1) + 5.0),
        )
        try:
            return self.wait_both_at(
                target,
                tol_mm=tol_mm,
                timeout=timeout,
                poll=protect.raise_if_tripped,
            )
        finally:
            protect.disarm()

    def reverse(
        self,
        stroke_mm: float,
        *,
        speed_mm_s: Optional[float] = None,
        wait_encoders: bool = True,
        **gate_kw,
    ) -> Tuple[float, float]:
        """Move both to ``+stroke_mm`` (absolute, after :meth:`set_zero`)."""
        return self.move_to(
            abs(float(stroke_mm)),
            speed_mm_s=speed_mm_s,
            wait_encoders=wait_encoders,
            **gate_kw,
        )

    def straight(
        self,
        *,
        speed_mm_s: Optional[float] = None,
        wait_encoders: bool = True,
        **gate_kw,
    ) -> Tuple[float, float]:
        """Return both to 0 mm (absolute)."""
        return self.move_to(
            0.0, speed_mm_s=speed_mm_s, wait_encoders=wait_encoders, **gate_kw
        )

    def oscillate(
        self,
        stroke_mm: float,
        cycles: int = 1,
        *,
        speed_mm_s: Optional[float] = None,
        start_positive: bool = True,
        wait_encoders: bool = True,
        tol_mm: Optional[float] = None,
        timeout: Optional[float] = None,
        poll: Optional[Callable[[], None]] = None,
        protect: Optional[object] = None,
        settle_s: float = 0.4,
        min_arm_mm: float = 3.0,
    ) -> int:
        """Run reverse/straight (or straight/reverse) continuously.

        Each cycle: move to ``±stroke_mm`` then back to ``0``, waiting for
        both encoders before the next leg when ``wait_encoders`` is True.

        If ``protect`` is a :class:`~canstepper.stall_protect.StallGuardEstop`,
        each leg uses :meth:`move_to_protected` (ignores ``poll`` /
        ``wait_encoders`` for the gate path).

        Returns the number of full cycles completed. Raises
        :class:`EncoderGateTimeout` or :class:`NodeFault` on failure (partial
        cycle count is not returned — call from a try/except if needed).
        """
        stroke = abs(float(stroke_mm))
        n = max(0, int(cycles))
        far = stroke if start_positive else -stroke
        for _ in range(n):
            if protect is not None:
                self.move_to_protected(
                    far,
                    protect,
                    speed_mm_s=speed_mm_s,
                    tol_mm=tol_mm,
                    timeout=timeout,
                    settle_s=settle_s,
                    min_arm_mm=min_arm_mm,
                )
                self.move_to_protected(
                    0.0,
                    protect,
                    speed_mm_s=speed_mm_s,
                    tol_mm=tol_mm,
                    timeout=timeout,
                    settle_s=settle_s,
                    min_arm_mm=min_arm_mm,
                )
            else:
                self.move_to(
                    far,
                    speed_mm_s=speed_mm_s,
                    wait_encoders=wait_encoders,
                    tol_mm=tol_mm,
                    timeout=timeout,
                    poll=poll,
                )
                self.move_to(
                    0.0,
                    speed_mm_s=speed_mm_s,
                    wait_encoders=wait_encoders,
                    tol_mm=tol_mm,
                    timeout=timeout,
                    poll=poll,
                )
        return n

    def __repr__(self) -> str:
        return (
            f"<IndependentDualAxis {self.name!r} "
            f"a={self.node_a.node_id} b={self.node_b.node_id} "
            f"rd={self.rotation_distance} cl={self._closed_loop}>"
        )


# ---------------------------------------------------------------------------
# Cartesian (Klipper-style independent axes)
# ---------------------------------------------------------------------------

class Cartesian:
    """Cartesian kinematics: one (or dual) stepper per workspace axis.

    Matches Klipper's ``kinematics: cartesian`` model — each motor drives a
    single cartesian axis with a 1:1 mapping::

        motor_x = x
        motor_y = y
        motor_z = z   # optional

    This is the usual bed/gantry mill, router, or cartesian 3D printer layout
    (as opposed to :class:`CoreXY`, where both motors contribute to X and Y).

    Build from :class:`Axis` objects (recommended when X/Y/Z have different
    ``rotation_distance``), or use :meth:`from_nodes` for a quick bench setup.

    Dual-Z gantries: pass a :class:`DualMotorAxis` as ``z``.
    """

    def __init__(
        self,
        x: Axis,
        y: Axis,
        z: Optional[Axis] = None,
        *,
        max_speed: Optional[float] = None,
        name: str = "cartesian",
    ):
        self.name = name
        self.max_speed = max_speed
        self.x = x
        self.y = y
        self.z = z
        axes: List[Axis] = [x, y]
        if z is not None:
            axes.append(z)
        self._group = MotionGroup(axes)

    @classmethod
    def from_nodes(
        cls,
        motor_x: StepperNode,
        motor_y: StepperNode,
        motor_z: Optional[StepperNode] = None,
        *,
        rotation_distance: float = 40.0,
        rotation_distance_z: float = 8.0,
        max_speed: Optional[float] = None,
        name: str = "cartesian",
    ) -> "Cartesian":
        """Convenience: wrap nodes in :class:`Axis` with Klipper-style units.

        ``rotation_distance`` applies to X and Y (typical belt). Z defaults to
        ``rotation_distance_z`` (typical leadscrew lead, e.g. 8 mm).
        """
        x = Axis(motor_x, rotation_distance, name=f"{name}_x")
        y = Axis(motor_y, rotation_distance, name=f"{name}_y")
        z = (
            Axis(motor_z, rotation_distance_z, name=f"{name}_z")
            if motor_z is not None
            else None
        )
        return cls(x, y, z, max_speed=max_speed, name=name)

    # -- kinematics (identity) -------------------------------------------------

    @staticmethod
    def cartesian_to_motors(
        x: float, y: float, z: float = 0.0
    ) -> Tuple[float, float, float]:
        """Identity map — motor travel equals workspace travel."""
        return x, y, z

    @staticmethod
    def motors_to_cartesian(
        mx: float, my: float, mz: float = 0.0
    ) -> Tuple[float, float, float]:
        return mx, my, mz

    def get_position(self) -> Tuple[float, float, float]:
        """Workspace ``(x, y, z)``; ``z`` is 0.0 when no Z axis is configured."""
        zx = self.z.get_position() if self.z is not None else 0.0
        return self.x.get_position(), self.y.get_position(), zx

    # -- motion ----------------------------------------------------------------

    def move_to(
        self,
        x: float,
        y: float,
        z: Optional[float] = None,
        speed: Optional[float] = None,
        blocking: bool = True,
        timeout: float = 120.0,
    ) -> float:
        """Straight-line move in workspace; ``speed`` is path speed (units/s).

        If ``z`` is omitted, the current Z is held (or ignored with no Z axis).
        Returns planned duration in seconds (from :class:`MotionGroup`).
        """
        cur_x, cur_y, cur_z = self.get_position()
        if z is None:
            z = cur_z
        elif self.z is None and abs(z - cur_z) > 1e-12:
            raise ValueError(f"{self.name}: Z target given but no Z axis configured")

        dx, dy = x - cur_x, y - cur_y
        dz = (z - cur_z) if self.z is not None else 0.0
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist == 0.0:
            return 0.0

        speed = speed if speed is not None else self.max_speed
        targets: Dict[Union[str, Axis], float] = {self.x: x, self.y: y}
        speeds: Optional[Dict[Union[str, Axis], float]] = None
        if speed is not None:
            speeds = {}
            if dx != 0.0:
                speeds[self.x] = abs(dx) / dist * speed
            if dy != 0.0:
                speeds[self.y] = abs(dy) / dist * speed
        if self.z is not None:
            targets[self.z] = z
            if speeds is not None and dz != 0.0:
                speeds[self.z] = abs(dz) / dist * speed

        return self._group.move_to(
            targets, speeds=speeds, blocking=blocking, timeout=timeout
        )

    def set_zero(self) -> "Cartesian":
        """Define the current pose as workspace (0, 0[, 0])."""
        self.x.set_zero()
        self.y.set_zero()
        if self.z is not None:
            self.z.set_zero()
        return self

    def enable(self) -> "Cartesian":
        for ax in self._iter_axes():
            if isinstance(ax, DualMotorAxis):
                ax.enable()
            else:
                ax.node.enable()
        return self

    def disable(self) -> "Cartesian":
        for ax in self._iter_axes():
            if isinstance(ax, DualMotorAxis):
                ax.disable()
            else:
                ax.node.disable()
        return self

    def stop(self) -> "Cartesian":
        self._group.stop()
        return self

    def estop(self) -> "Cartesian":
        self._group.estop()
        return self

    def _iter_axes(self) -> List[Axis]:
        axes = [self.x, self.y]
        if self.z is not None:
            axes.append(self.z)
        return axes

    def axis_map(self) -> Dict[str, Axis]:
        """``{\"X\": ..., \"Y\": ..., \"Z\": ...}`` for G-code / config."""
        m: Dict[str, Axis] = {"X": self.x, "Y": self.y}
        if self.z is not None:
            m["Z"] = self.z
        return m

    def __repr__(self) -> str:
        z = f" z={self.z.node.node_id}" if self.z is not None else ""
        return (
            f"<Cartesian {self.name!r} x={self.x.node.node_id} "
            f"y={self.y.node.node_id}{z}>"
        )


# ---------------------------------------------------------------------------
# CoreXY
# ---------------------------------------------------------------------------

class CoreXY:
    """CoreXY kinematics on two motors.

    Belt equations (standard CoreXY):
        a = x + y        (motor A belt travel)
        b = x - y        (motor B belt travel)
        x = (a + b) / 2
        y = (a - b) / 2

    ``rotation_distance`` is belt travel per motor revolution
    (pulley teeth × belt pitch, e.g. 20 T × 2 mm = 40.0).
    """

    def __init__(
        self,
        motor_a: StepperNode,
        motor_b: StepperNode,
        rotation_distance: float,
        max_speed: Optional[float] = None,
        name: str = "corexy",
    ):
        self.name = name
        self.max_speed = max_speed
        self.axis_a = Axis(motor_a, rotation_distance, name=f"{name}_a")
        self.axis_b = Axis(motor_b, rotation_distance, name=f"{name}_b")
        self._group = MotionGroup([self.axis_a, self.axis_b])

    # -- kinematics ------------------------------------------------------------

    @staticmethod
    def cartesian_to_motors(x: float, y: float) -> Tuple[float, float]:
        return x + y, x - y

    @staticmethod
    def motors_to_cartesian(a: float, b: float) -> Tuple[float, float]:
        return (a + b) / 2.0, (a - b) / 2.0

    def get_position(self) -> Tuple[float, float]:
        """Cartesian ``(x, y)`` from the two motor encoders."""
        return self.motors_to_cartesian(
            self.axis_a.get_position(), self.axis_b.get_position()
        )

    # -- motion -----------------------------------------------------------------

    def move_to(
        self,
        x: float,
        y: float,
        speed: Optional[float] = None,
        blocking: bool = True,
        timeout: float = 120.0,
    ) -> float:
        """Straight-line move to ``(x, y)``; ``speed`` is the cartesian
        path speed (units/s). Returns the planned duration in seconds."""
        cur_x, cur_y = self.get_position()
        dx, dy = x - cur_x, y - cur_y
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            return 0.0
        speed = speed if speed is not None else self.max_speed
        a_target, b_target = self.cartesian_to_motors(x, y)
        da, db = self.cartesian_to_motors(dx, dy)

        speeds = None
        if speed is not None:
            # Split the cartesian path speed onto the belts so both motors
            # finish together and the path stays straight.
            speeds = {}
            if da != 0.0:
                speeds[self.axis_a] = abs(da) / dist * speed
            if db != 0.0:
                speeds[self.axis_b] = abs(db) / dist * speed

        targets: Dict[Union[str, Axis], float] = {}
        if da != 0.0:
            targets[self.axis_a] = a_target
        if db != 0.0:
            targets[self.axis_b] = b_target
        return self._group.move_to(
            targets, speeds=speeds, blocking=blocking, timeout=timeout
        )

    def set_zero(self) -> "CoreXY":
        """Define the current position as cartesian (0, 0)."""
        self.axis_a.set_zero()
        self.axis_b.set_zero()
        return self

    def enable(self) -> "CoreXY":
        self.axis_a.node.enable()
        self.axis_b.node.enable()
        return self

    def disable(self) -> "CoreXY":
        self.axis_a.node.disable()
        self.axis_b.node.disable()
        return self

    def stop(self) -> "CoreXY":
        self._group.stop()
        return self

    def estop(self) -> "CoreXY":
        self._group.estop()
        return self

    def __repr__(self) -> str:
        return (
            f"<CoreXY {self.name!r} a={self.axis_a.node.node_id} "
            f"b={self.axis_b.node.node_id}>"
        )
