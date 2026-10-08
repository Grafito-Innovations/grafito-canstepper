"""Axis — a stepper node dressed in real-world linear units.

Klipper-style ``rotation_distance``: the distance the axis travels for one
full motor revolution (e.g. an 8 mm-lead leadscrew -> 8.0, a GT2-20T belt
-> 40.0). All Axis positions/speeds are in those units ("mm" below).
"""

from __future__ import annotations

from typing import Optional, Union

from .exceptions import LimitViolation, NotHomed
from .node import StepperNode
from .protocol import HomeMethod, Param


class Axis:
    """Linear (or geared rotary) axis on one node.

    Args:
        node:               the StepperNode driving the axis
        rotation_distance:  units of travel per motor revolution
        gear_ratio:         motor revs per output rev (e.g. 5.0 for a 5:1 box)
        min_pos / max_pos:  optional host-side soft travel limits (units)
        max_speed:          optional host-side speed limit (units/s)
        require_homing:     refuse moves until the axis was homed
    """

    def __init__(
        self,
        node: StepperNode,
        rotation_distance: float,
        gear_ratio: float = 1.0,
        min_pos: Optional[float] = None,
        max_pos: Optional[float] = None,
        max_speed: Optional[float] = None,
        require_homing: bool = False,
        name: str = "",
    ):
        if rotation_distance <= 0:
            raise ValueError("rotation_distance must be positive")
        if gear_ratio <= 0:
            raise ValueError("gear_ratio must be positive")
        self.node = node
        self.rotation_distance = float(rotation_distance)
        self.gear_ratio = float(gear_ratio)
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.max_speed = max_speed
        self.require_homing = require_homing
        self.name = name or f"axis{node.node_id}"
        self._homed = False

    # -- unit conversion ---------------------------------------------------------

    def units_to_deg(self, units: float) -> float:
        return units / self.rotation_distance * 360.0 * self.gear_ratio

    def deg_to_units(self, deg: float) -> float:
        return deg / 360.0 / self.gear_ratio * self.rotation_distance

    # -- state ---------------------------------------------------------------------

    @property
    def position(self) -> Optional[float]:
        """Last-known position in axis units (cached telemetry)."""
        deg = self.node.state.position_deg
        return None if deg is None else self.deg_to_units(deg)

    def get_position(self) -> float:
        """Round-trip read of the position in axis units."""
        return self.deg_to_units(self.node.get_position())

    @property
    def homed(self) -> bool:
        status = self.node.state.status
        if status is not None:
            return status.homed or self._homed
        return self._homed

    # -- motion ---------------------------------------------------------------------

    def move_to(
        self,
        pos: float,
        speed: Optional[float] = None,
        blocking: bool = False,
        timeout: float = 60.0,
    ) -> "Axis":
        """Absolute move in axis units; ``speed`` (units/s) sets the node's
        move speed for this and subsequent moves."""
        self._check_motion_allowed(pos, speed)
        if speed is not None:
            self.node.set_max_speed(self.units_to_deg(speed))
        self.node.move_to(self.units_to_deg(pos), blocking=blocking, timeout=timeout)
        return self

    def move_by(
        self,
        delta: float,
        speed: Optional[float] = None,
        blocking: bool = False,
        timeout: float = 60.0,
    ) -> "Axis":
        target = None
        if self.min_pos is not None or self.max_pos is not None:
            current = self.position
            if current is not None:
                target = current + delta
        self._check_motion_allowed(target, speed)
        if speed is not None:
            self.node.set_max_speed(self.units_to_deg(speed))
        self.node.move_by(self.units_to_deg(delta), blocking=blocking, timeout=timeout)
        return self

    def run(self, speed: float) -> "Axis":
        """Continuous motion at signed ``speed`` (units/s)."""
        if self.max_speed is not None and abs(speed) > self.max_speed:
            raise LimitViolation(
                f"{self.name}: {abs(speed):.3f} exceeds max_speed {self.max_speed:.3f}"
            )
        self.node.run(self.units_to_deg(speed))
        return self

    def stop(self) -> "Axis":
        self.node.stop()
        return self

    def estop(self) -> "Axis":
        self.node.estop()
        return self

    # -- homing ---------------------------------------------------------------------

    def home(
        self,
        method: Union[str, HomeMethod] = HomeMethod.ENDSTOP,
        direction: int = -1,
        speed: float = 10.0,
        current_percent: Optional[int] = None,
        backoff: Optional[float] = None,
        timeout: float = 60.0,
    ) -> "Axis":
        """Home the axis. ``speed`` and ``backoff`` are in axis units.

        For ``"stallguard"`` homing, tune ``node.set_stall_threshold()`` and
        use a reduced ``current_percent`` so the crash into the hard stop
        stays gentle.
        """
        if current_percent is not None:
            self.node.set_param(Param.HOMING_CURRENT, current_percent)
        if backoff is not None:
            self.node.set_param(Param.HOMING_BACKOFF, self.units_to_deg(backoff))
        self.node.home(
            method=method,
            direction=direction,
            speed_deg_s=self.units_to_deg(speed),
            blocking=True,
            timeout=timeout,
        )
        self._homed = True
        return self

    def set_zero(self) -> "Axis":
        self.node.set_zero()
        self._homed = True
        return self

    def wait_settled(self, timeout: float = 30.0) -> "Axis":
        self.node.wait_settled(timeout=timeout)
        return self

    # -- internals ---------------------------------------------------------------------

    def _check_motion_allowed(
        self, target: Optional[float], speed: Optional[float]
    ) -> None:
        if self.require_homing and not self.homed:
            raise NotHomed(f"{self.name}: home the axis before moving it")
        if speed is not None and self.max_speed is not None and abs(speed) > self.max_speed:
            raise LimitViolation(
                f"{self.name}: {abs(speed):.3f} exceeds max_speed {self.max_speed:.3f}"
            )
        if target is not None:
            if self.min_pos is not None and target < self.min_pos:
                raise LimitViolation(
                    f"{self.name}: target {target:.3f} below min_pos {self.min_pos:.3f}"
                )
            if self.max_pos is not None and target > self.max_pos:
                raise LimitViolation(
                    f"{self.name}: target {target:.3f} above max_pos {self.max_pos:.3f}"
                )

    def __repr__(self) -> str:
        return (
            f"<Axis {self.name!r} node={self.node.node_id} "
            f"rotation_distance={self.rotation_distance}>"
        )
