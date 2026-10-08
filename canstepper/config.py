"""Declarative machine configuration (TOML or dict).

Example ``machine.toml``::

    [bus]
    transport = "serial"
    port = "/dev/ttyACM0"

    [node.z_left]
    id = 2
    run_current = 40
    microsteps = 16
    closed_loop = 1

    [node.z_right]
    id = 3
    run_current = 40

    [axis.z]
    type = "dual_motor"
    primary = "z_left"
    secondary = "z_right"
    invert_secondary = true
    rotation_distance = 8.0
    max_speed = 20.0

    [axis.z.homing]
    method = "stallguard"
    direction = -1
    speed = 5.0

Any key in a ``[node.*]`` table other than ``id`` must be a firmware
parameter name from :data:`canstepper.protocol.PARAMS` and is applied (and
verified) on load.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional, Union

from .axis import Axis
from .bus import CANStepperBus
from .exceptions import ConfigError
from .kinematics import CoreXY, DualMotorAxis, IndependentDualAxis
from .node import StepperNode
from .protocol import PARAMS_BY_NAME, resolve_param

_AXIS_KEYS = {
    "type", "node", "primary", "secondary", "motor_a", "motor_b",
    "node_a", "node_b",
    "rotation_distance", "gear_ratio", "min_pos", "max_pos", "max_speed",
    "require_homing", "invert_secondary", "invert_a", "invert_b",
    "invert_primary", "ratio", "encoder_corrected",
    "tol_mm", "settle_s", "gate_timeout_s",
    "homing",
}


def _load_toml(path: Union[str, Path]) -> Dict[str, Any]:
    try:
        import tomllib  # Python 3.11+
    except ImportError:
        try:
            import tomli as tomllib  # type: ignore[no-redef]
        except ImportError as exc:
            raise ConfigError(
                "reading TOML on Python < 3.11 requires the 'tomli' package "
                "(pip install tomli), or pass a dict instead of a path"
            ) from exc
    with open(path, "rb") as fh:
        return tomllib.load(fh)


class Machine:
    """A whole machine: bus + named nodes + named axes, built from config."""

    def __init__(self, bus: CANStepperBus, owns_bus: bool = False):
        self.bus = bus
        self._owns_bus = owns_bus
        self.nodes: Dict[str, StepperNode] = {}
        self.axes: Dict[str, Union[Axis, DualMotorAxis, CoreXY, IndependentDualAxis]] = {}
        self.homing: Dict[str, Dict[str, Any]] = {}

    # -- construction ---------------------------------------------------------

    @classmethod
    def from_config(
        cls,
        source: Union[str, Path, Dict[str, Any]],
        bus: Optional[CANStepperBus] = None,
        apply: bool = True,
    ) -> "Machine":
        """Build a Machine from a TOML file path or an equivalent dict.

        With ``apply=True`` every node parameter in the config is written to
        (and acknowledged by) the hardware. Values are volatile until you
        call :meth:`save_all`.
        """
        cfg = _load_toml(source) if not isinstance(source, dict) else source

        owns_bus = bus is None
        if bus is None:
            bus_cfg = cfg.get("bus", {})
            transport = bus_cfg.get("transport", "serial")
            if transport != "serial":
                raise ConfigError(f"unsupported transport {transport!r}")
            port = bus_cfg.get("port")
            if not port:
                raise ConfigError("[bus] needs a 'port' for the serial transport")
            bus = CANStepperBus.serial(port, baudrate=bus_cfg.get("baudrate", 115200))

        machine = cls(bus, owns_bus=owns_bus)
        try:
            machine._build_nodes(cfg.get("node", {}), apply=apply)
            machine._build_axes(cfg.get("axis", {}), apply=apply)
        except Exception:
            if owns_bus:
                bus.close()
            raise
        return machine

    def _build_nodes(self, node_cfg: Dict[str, Any], apply: bool) -> None:
        for name, table in node_cfg.items():
            if "id" not in table:
                raise ConfigError(f"[node.{name}] is missing 'id'")
            node = self.bus.node(int(table["id"]))
            self.nodes[name] = node
            for key, value in table.items():
                if key == "id":
                    continue
                if key.lower() not in PARAMS_BY_NAME:
                    raise ConfigError(
                        f"[node.{name}] unknown parameter {key!r} "
                        f"(see canstepper.protocol.PARAMS)"
                    )
                if apply:
                    node.set_param(key, value)

    def _node_ref(self, axis_name: str, table: Dict[str, Any], key: str) -> StepperNode:
        ref = table.get(key)
        if ref is None:
            raise ConfigError(f"[axis.{axis_name}] is missing {key!r}")
        if ref not in self.nodes:
            raise ConfigError(
                f"[axis.{axis_name}] references unknown node {ref!r}"
            )
        return self.nodes[ref]

    def _build_axes(self, axis_cfg: Dict[str, Any], apply: bool = True) -> None:
        for name, table in axis_cfg.items():
            unknown = set(table) - _AXIS_KEYS
            if unknown:
                raise ConfigError(f"[axis.{name}] unknown keys: {sorted(unknown)}")
            kind = table.get("type", "single")
            common = dict(
                gear_ratio=table.get("gear_ratio", 1.0),
                min_pos=table.get("min_pos"),
                max_pos=table.get("max_pos"),
                max_speed=table.get("max_speed"),
                require_homing=table.get("require_homing", False),
                name=name,
            )
            rotation = table.get("rotation_distance")
            if kind in ("single", "dual_motor", "independent_dual") and rotation is None:
                raise ConfigError(f"[axis.{name}] needs 'rotation_distance'")

            if kind == "single":
                axis = Axis(self._node_ref(name, table, "node"), rotation, **common)
            elif kind == "dual_motor":
                axis = DualMotorAxis(
                    self._node_ref(name, table, "primary"),
                    self._node_ref(name, table, "secondary"),
                    rotation,
                    invert_secondary=table.get("invert_secondary", False),
                    ratio=table.get("ratio", 1.0),
                    encoder_corrected=table.get("encoder_corrected", True),
                    **common,
                )
            elif kind == "independent_dual":
                a_key = "node_a" if "node_a" in table else "primary"
                b_key = "node_b" if "node_b" in table else "secondary"
                axis = IndependentDualAxis(
                    self._node_ref(name, table, a_key),
                    self._node_ref(name, table, b_key),
                    rotation,
                    invert_a=bool(table.get("invert_a", table.get("invert_primary", False))),
                    invert_b=bool(table.get("invert_b", table.get("invert_secondary", False))),
                    name=name,
                    tol_mm=float(table.get("tol_mm", 0.8)),
                    settle_s=float(table.get("settle_s", 0.25)),
                    gate_timeout_s=float(table.get("gate_timeout_s", 30.0)),
                )
                if apply:
                    axis.apply_direction()
            elif kind == "corexy":
                if rotation is None:
                    raise ConfigError(f"[axis.{name}] needs 'rotation_distance'")
                axis = CoreXY(
                    self._node_ref(name, table, "motor_a"),
                    self._node_ref(name, table, "motor_b"),
                    rotation,
                    max_speed=table.get("max_speed"),
                    name=name,
                )
            else:
                raise ConfigError(f"[axis.{name}] unknown type {kind!r}")

            self.axes[name] = axis
            if "homing" in table:
                self.homing[name] = dict(table["homing"])

    # -- operations ---------------------------------------------------------------

    def home(self, axis_name: str, **overrides) -> None:
        """Home one axis using its ``[axis.*.homing]`` config (plus overrides)."""
        axis = self.axes[axis_name]
        if isinstance(axis, CoreXY):
            raise ConfigError("CoreXY homing is not automated in v1; home the "
                              "motors individually or use set_zero()")
        if isinstance(axis, IndependentDualAxis):
            # Dual-screw frames: encoder-gate set_zero until a dedicated home
            # method is wired (switch lives on one of the two nodes).
            axis.set_zero()
            return
        kwargs = dict(self.homing.get(axis_name, {}))
        kwargs.update(overrides)
        axis.home(**kwargs)

    def check(self) -> Dict[str, Dict[str, Any]]:
        """Compare live node parameters against nothing-changed expectations.

        Returns ``{node_name: {param_name: live_value}}`` for every node —
        a quick way to snapshot/diff what the hardware actually runs.
        """
        report: Dict[str, Dict[str, Any]] = {}
        for name, node in self.nodes.items():
            values: Dict[str, Any] = {}
            for pname, pdef in PARAMS_BY_NAME.items():
                values[pname] = node.get_param(pdef.param)
            report[name] = values
        return report

    def enable_all(self) -> None:
        self.bus.enable_all(True)

    def estop_all(self) -> None:
        """Single broadcast frame: every node stops and disables instantly."""
        self.bus.estop_all()

    def save_all(self) -> None:
        """Persist every configured node's parameters to its flash."""
        for node in self.nodes.values():
            node.save_config()

    def close(self) -> None:
        if self._owns_bus:
            self.bus.close()

    def __enter__(self) -> "Machine":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
