"""Typed errors raised by the canstepper package."""

from __future__ import annotations

from typing import Optional, Tuple


class CANStepperError(Exception):
    """Base class for every canstepper error."""


class TransportError(CANStepperError):
    """The underlying serial/CAN transport failed."""


class RequestTimeout(CANStepperError, TimeoutError):
    """A node did not answer a request in time."""

    def __init__(self, node_id: int, what: str, timeout: float):
        self.node_id = node_id
        self.what = what
        self.timeout = timeout
        super().__init__(
            f"node {node_id}: no {what} response within {timeout:.2f}s"
        )


class UnknownParam(CANStepperError):
    """The node does not know the requested parameter ID."""


class ParamRejected(CANStepperError):
    """The node rejected a parameter value (outside its sanity range)."""


class NodeFault(CANStepperError):
    """The node reported a fault (encoder, no-progress, homing timeout)."""

    def __init__(self, node_id: int, fault_code: int, message: str = ""):
        self.node_id = node_id
        self.fault_code = fault_code
        super().__init__(message or f"node {node_id}: fault code {fault_code}")


class GCodeError(CANStepperError):
    """Invalid or unsupported G-code line / state."""

    def __init__(self, message: str, line: str = ""):
        self.line = line
        super().__init__(message if not line else f"{message}  (line: {line!r})")


class HomingFailed(NodeFault):
    """A homing run timed out or faulted."""


class EStopActive(CANStepperError):
    """Motion was requested while the node is e-stop latched."""


class LimitViolation(CANStepperError):
    """A host-side axis/speed limit would be exceeded."""


class NotHomed(CANStepperError):
    """Motion was requested on an axis that requires homing first."""


class ConfigError(CANStepperError):
    """A machine configuration file is invalid or inconsistent."""


class EncoderGateTimeout(CANStepperError, TimeoutError):
    """Both motors did not reach the encoder target within the gate window.

    Used by :class:`~canstepper.kinematics.IndependentDualAxis` when the next
    pass is blocked until *both* encoders are within tolerance of the
    commanded linear position (not merely when step generators go idle).
    """

    def __init__(
        self,
        message: str,
        *,
        target_mm: float = 0.0,
        positions_mm: Optional[Tuple[float, float]] = None,
        timeout: float = 0.0,
    ):
        self.target_mm = target_mm
        self.positions_mm = positions_mm
        self.timeout = timeout
        super().__init__(message)


class StallGuardTrip(CANStepperError):
    """Armed StallGuard event stopped the machine (host estop).

    Raised by :class:`~canstepper.stall_protect.StallGuardEstop` when a
    DIAG/STALL fires while protection is armed. Motors are already
    emergency-stopped when this is raised.
    """

    def __init__(
        self,
        node_id: int,
        position_deg: float = 0.0,
        message: str = "",
    ):
        self.node_id = int(node_id)
        self.position_deg = float(position_deg)
        super().__init__(
            message
            or (
                f"StallGuard trip on node {self.node_id} "
                f"@ {self.position_deg:.2f}° — estop issued"
            )
        )
