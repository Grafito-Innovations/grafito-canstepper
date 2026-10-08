"""grafito-canstepper — host library for Grafito CANStepper boards (GCSP v1).

Quick start::

    from canstepper import CANStepperBus

    bus = CANStepperBus.serial("/dev/ttyACM0")
    print(bus.discover())

    node = bus.node(1)
    node.set_run_current(40).set_microsteps(16).enable()
    node.move_to(180.0, blocking=True)

    bus.estop_all()   # single broadcast frame stops every node
"""

from .axis import Axis
from .bus import CANStepperBus
from .config import Machine
from .exceptions import (
    CANStepperError,
    ConfigError,
    EncoderGateTimeout,
    EStopActive,
    GCodeError,
    HomingFailed,
    LimitViolation,
    NodeFault,
    NotHomed,
    ParamRejected,
    RequestTimeout,
    StallGuardTrip,
    TransportError,
    UnknownParam,
)
from .stall_protect import StallGuardEstop
from .gcode import GCodeController, GCodeLine, parse_gcode_line
from .group import NodeGroup
from .kinematics import (
    Cartesian,
    CoreXY,
    DualMotorAxis,
    IndependentDualAxis,
    MotionGroup,
)
from .node import SpeedLimits, StepperNode
from .protocol import (
    BROADCAST,
    PROTOCOL_VERSION,
    Cmd,
    EndstopAction,
    Event,
    Fault,
    Frame,
    HomeMethod,
    LutAction,
    Mode,
    Param,
    StandstillMode,
    Tel,
)
from .telemetry import (
    CanHealth,
    DriverStatus,
    FollowStatus,
    LutStatus,
    NodeState,
    NodeStatus,
    PidStatus,
)
from .tmc2209 import (
    CANSTEPPER_RSENSE_OHM,
    CANSTEPPER_VFSENSE_V,
    TMC2209_R_INTERNAL_OHM,
    commanded_rms_amps,
    full_scale_rms_amps,
)
from .transport import SerialBridgeTransport, Transport

__version__ = "0.2.3"

__all__ = [
    "Axis",
    "BROADCAST",
    "CANSTEPPER_RSENSE_OHM",
    "CANSTEPPER_VFSENSE_V",
    "CANStepperBus",
    "CANStepperError",
    "CanHealth",
    "commanded_rms_amps",
    "Cartesian",
    "Cmd",
    "ConfigError",
    "CoreXY",
    "DriverStatus",
    "DualMotorAxis",
    "EncoderGateTimeout",
    "EndstopAction",
    "EStopActive",
    "Event",
    "Fault",
    "FollowStatus",
    "Frame",
    "full_scale_rms_amps",
    "GCodeController",
    "GCodeError",
    "GCodeLine",
    "HomeMethod",
    "HomingFailed",
    "IndependentDualAxis",
    "LimitViolation",
    "LutAction",
    "LutStatus",
    "Machine",
    "Mode",
    "MotionGroup",
    "NodeFault",
    "NodeGroup",
    "NodeState",
    "NodeStatus",
    "NotHomed",
    "Param",
    "ParamRejected",
    "parse_gcode_line",
    "PidStatus",
    "PROTOCOL_VERSION",
    "RequestTimeout",
    "SerialBridgeTransport",
    "SpeedLimits",
    "StandstillMode",
    "StallGuardEstop",
    "StallGuardTrip",
    "StepperNode",
    "Tel",
    "TMC2209_R_INTERNAL_OHM",
    "Transport",
    "TransportError",
    "UnknownParam",
]
