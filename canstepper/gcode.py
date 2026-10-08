"""G-code translation layer for canstepper (Klipper/Marlin-inspired).

This is an **independent** host-side interpreter. Command names and semantics
follow common 3D-printer / CNC G-code practice as used by projects such as
`Klipper <https://www.klipper3d.org/G-Codes.html>`_ (G0/G1, G28, G90/G91,
G92, M18/M84, M112, M114, M400, …). It is **not** a port of Klipper source
code and does not implement Klipper's full toolhead / MCU protocol.

Supported motion targets (Klipper-inspired):

* **Cartesian** — :class:`Cartesian` (independent X/Y/Z steppers, 1:1 map)
* **CoreXY** — :class:`CoreXY` for the XY plane, optional separate ``Z``/``E``
* **Ad-hoc axes** — letter map to :class:`Axis` via :meth:`from_axes`

Units are **millimetres** and feedrate **F** is **mm/min** (converted to mm/s
internally), matching typical slicer G-code.
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    Iterable,
    List,
    Mapping,
    Optional,
    Sequence,
    TextIO,
    Tuple,
    Union,
)

from .axis import Axis
from .bus import CANStepperBus
from .exceptions import GCodeError
from .kinematics import (
    Cartesian,
    CoreXY,
    DualMotorAxis,
    IndependentDualAxis,
    MotionGroup,
)
from .node import StepperNode

# Word token: letter + optional number (bare ``G28 X`` allowed)
_WORD_RE = re.compile(
    r"([A-Za-z])\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)?"
)

# G-code words that are never linear axis letters. Do NOT put A/B/C/D/H/I/J
# here — those are first-class grafting axes on from_axes machines.
_NON_AXIS_WORDS = frozenset({"F", "G", "M", "P", "S", "N", "T"})


def _axis_nodes(axis) -> List[StepperNode]:
    if isinstance(axis, IndependentDualAxis):
        return [axis.node_a, axis.node_b]
    nodes = [axis.node]
    if isinstance(axis, DualMotorAxis):
        nodes.append(axis.secondary)
    return nodes


def _axis_position(axis) -> float:
    if isinstance(axis, IndependentDualAxis):
        a, b = axis.get_positions()
        return (a + b) / 2.0
    return axis.get_position()


def _axis_set_logical(axis, units: float) -> None:
    if isinstance(axis, IndependentDualAxis):
        deg = axis.units_to_deg(units)
        axis.node_a.set_logical_position(deg)
        axis.node_b.set_logical_position(deg)
        return
    axis.node.set_logical_position(axis.units_to_deg(units))


@dataclass
class GCodeLine:
    """One parsed G/M command."""

    command: str  # e.g. "G1", "M112"
    params: Dict[str, float] = field(default_factory=dict)
    comment: str = ""
    raw: str = ""
    line_number: Optional[int] = None


def strip_comments(line: str) -> Tuple[str, str]:
    """Remove ``;`` and ``(...)`` comments. Returns ``(code, comment)``."""
    comment_parts: List[str] = []
    # Parenthetical comments
    def _paren(m: re.Match[str]) -> str:
        comment_parts.append(m.group(1).strip())
        return " "

    s = re.sub(r"\((.*?)\)", _paren, line)
    if ";" in s:
        code, _, rest = s.partition(";")
        comment_parts.append(rest.strip())
        s = code
    return s.strip(), " ".join(p for p in comment_parts if p)


def parse_gcode_line(line: str) -> Optional[GCodeLine]:
    """Parse a single G-code line. Returns ``None`` for blank/comment-only."""
    raw = line.rstrip("\n\r")
    code, comment = strip_comments(raw)
    if not code:
        return None

    line_number: Optional[int] = None
    m_n = re.match(r"N(\d+)\b", code, re.IGNORECASE)
    if m_n:
        line_number = int(m_n.group(1))
        code = code[m_n.end() :].strip()

    words = _WORD_RE.findall(code)
    if not words:
        if code:
            raise GCodeError(f"unparseable G-code", raw)
        return None

    # First G or M word is the command; subsequent G/M become separate only
    # if we split — Klipper/Marlin often put one command per line. We take
    # the first G/M as command and remaining letter words as params.
    command: Optional[str] = None
    params: Dict[str, float] = {}
    for letter, num_s in words:
        L = letter.upper()
        if num_s is None or num_s == "":
            # Bare flag word (e.g. G28 X) → present with value 0
            if L in ("G", "M") and command is None:
                raise GCodeError(f"command {L} requires a number", raw)
            params[L] = 0.0
            continue
        try:
            val = float(num_s)
        except ValueError as exc:
            raise GCodeError(f"bad number {num_s!r}", raw) from exc
        if L in ("G", "M") and command is None:
            # G1 / M112 style: integer command id when whole number
            if abs(val - round(val)) < 1e-9:
                command = f"{L}{int(round(val))}"
            else:
                command = f"{L}{val}"
        else:
            params[L] = val

    if command is None:
        # Bare parameters without G/M — treat as modal G1 (common in some streams)
        command = "G1"

    return GCodeLine(
        command=command,
        params=params,
        comment=comment,
        raw=raw,
        line_number=line_number,
    )


class GCodeController:
    """Execute G-code against a canstepper machine.

    Example (CoreXY + optional Z)::

        from canstepper import CANStepperBus, CoreXY, Axis, GCodeController

        with CANStepperBus.serial("/dev/ttyACM0") as bus:
            xy = CoreXY(bus.node(1), bus.node(2), rotation_distance=40.0)
            z = Axis(bus.node(3), rotation_distance=8.0, name="Z")
            g = GCodeController.from_corexy(xy, z=z, bus=bus)
            g.run("G28")
            g.run("G1 X10 Y5 F600")
            g.run("M114")

    Example (cartesian mechanism — Klipper-style independent X/Y/Z)::

        from canstepper import Cartesian, GCodeController

        cart = Cartesian.from_nodes(
            bus.node(1), bus.node(2), bus.node(3),
            rotation_distance=40.0,   # X/Y belts
            rotation_distance_z=8.0,  # Z leadscrew
        )
        g = GCodeController.from_cartesian(cart, bus=bus)
        g.run("G28")
        g.run("G1 X10 Y5 Z2 F600")
    """

    def __init__(
        self,
        bus: Optional[CANStepperBus] = None,
        *,
        axes: Optional[Mapping[str, Axis]] = None,
        corexy: Optional[CoreXY] = None,
        cartesian: Optional[Cartesian] = None,
        default_feedrate_mm_min: float = 600.0,
        move_timeout: float = 120.0,
    ):
        if corexy is not None and cartesian is not None:
            raise ValueError("use either corexy or cartesian kinematics, not both")
        self.bus = bus
        self.corexy = corexy
        self.cartesian = cartesian
        self.axes: Dict[str, Axis] = {}
        if axes:
            for k, ax in axes.items():
                self.axes[k.upper()] = ax
        if cartesian is not None:
            for letter, ax in cartesian.axis_map().items():
                self.axes.setdefault(letter, ax)

        self.absolute = True
        self.absolute_e = True
        self.feedrate_mm_min = float(default_feedrate_mm_min)
        self.speed_factor = 1.0  # M220 S%
        self.move_timeout = float(move_timeout)
        self.position: Dict[str, float] = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}
        self._handlers: Dict[str, Callable[[GCodeLine], str]] = {}
        self._register_defaults()
        self._sync_position_from_hardware()

    # -- construction helpers -------------------------------------------------

    @classmethod
    def from_corexy(
        cls,
        corexy: CoreXY,
        *,
        z: Optional[Axis] = None,
        e: Optional[Axis] = None,
        bus: Optional[CANStepperBus] = None,
        **kwargs: Any,
    ) -> "GCodeController":
        axes: Dict[str, Axis] = {}
        if z is not None:
            axes["Z"] = z
        if e is not None:
            axes["E"] = e
        if bus is None:
            bus = corexy.axis_a.node._bus  # type: ignore[attr-defined]
        return cls(bus, axes=axes, corexy=corexy, **kwargs)

    @classmethod
    def from_cartesian(
        cls,
        cartesian: Cartesian,
        *,
        e: Optional[Axis] = None,
        bus: Optional[CANStepperBus] = None,
        **kwargs: Any,
    ) -> "GCodeController":
        """Build a controller for a Klipper-style cartesian machine.

        ``cartesian`` supplies X/Y and optional Z. Pass ``e`` for an extruder
        or other independent feed axis.
        """
        axes = dict(cartesian.axis_map())
        if e is not None:
            axes["E"] = e
        if bus is None:
            bus = cartesian.x.node._bus  # type: ignore[attr-defined]
        return cls(bus, axes=axes, cartesian=cartesian, **kwargs)

    @classmethod
    def from_axes(
        cls,
        axes: Mapping[str, Axis],
        bus: Optional[CANStepperBus] = None,
        **kwargs: Any,
    ) -> "GCodeController":
        """Ad-hoc letter→:class:`Axis` map (no named kinematics object).

        Prefer :meth:`from_cartesian` when the machine is a standard
        independent-axis cartesian layout.
        """
        if bus is None and axes:
            first = next(iter(axes.values()))
            bus = first.node._bus  # type: ignore[attr-defined]
        return cls(bus, axes=axes, **kwargs)

    def _register_defaults(self) -> None:
        for code, fn in (
            ("G0", self._cmd_move),
            ("G1", self._cmd_move),
            ("G4", self._cmd_dwell),
            ("G20", self._cmd_inches),
            ("G21", self._cmd_mm),
            ("G28", self._cmd_home),
            ("G90", self._cmd_abs),
            ("G91", self._cmd_rel),
            ("G92", self._cmd_set_position),
            ("M17", self._cmd_enable),
            ("M18", self._cmd_disable),
            ("M84", self._cmd_disable),
            ("M82", self._cmd_e_abs),
            ("M83", self._cmd_e_rel),
            ("M105", self._cmd_temps),
            ("M112", self._cmd_estop),
            ("M114", self._cmd_get_position),
            ("M115", self._cmd_firmware_info),
            ("M220", self._cmd_speed_factor),
            ("M400", self._cmd_finish_moves),
        ):
            self._handlers[code] = fn

    def register(self, command: str, handler: Callable[[GCodeLine], str]) -> None:
        """Register or override a command handler (e.g. ``G2``, custom ``M``)."""
        self._handlers[command.upper()] = handler

    # -- public API -----------------------------------------------------------

    def run(self, line: str) -> str:
        """Parse and execute one line. Returns a Klipper-like response string."""
        parsed = parse_gcode_line(line)
        if parsed is None:
            return "ok"
        handler = self._handlers.get(parsed.command)
        if handler is None:
            raise GCodeError(f"unsupported command {parsed.command}", parsed.raw)
        return handler(parsed)

    def run_many(self, lines: Iterable[str]) -> List[str]:
        return [self.run(line) for line in lines]

    def run_file(self, path: Union[str, Path]) -> List[str]:
        path = Path(path)
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            return self.run_stream(fh)

    def run_stream(self, stream: TextIO) -> List[str]:
        out: List[str] = []
        for line in stream:
            out.append(self.run(line))
        return out

    def enable(self) -> "GCodeController":
        self._cmd_enable(GCodeLine("M17"))
        return self

    def disable(self) -> "GCodeController":
        self._cmd_disable(GCodeLine("M18"))
        return self

    # -- hardware sync --------------------------------------------------------

    def _sync_position_from_hardware(self) -> None:
        if self.corexy is not None:
            try:
                x, y = self.corexy.get_position()
                self.position["X"] = x
                self.position["Y"] = y
            except Exception:
                pass
        if self.cartesian is not None:
            try:
                x, y, z = self.cartesian.get_position()
                self.position["X"] = x
                self.position["Y"] = y
                self.position["Z"] = z
            except Exception:
                pass
        for letter, axis in self.axes.items():
            if letter in ("X", "Y") and self.corexy is not None:
                continue
            if letter in ("X", "Y", "Z") and self.cartesian is not None:
                continue
            try:
                self.position[letter] = _axis_position(axis)
            except Exception:
                self.position.setdefault(letter, 0.0)

    def _all_nodes(self) -> List[StepperNode]:
        nodes: List[StepperNode] = []
        seen = set()

        def _add(n: StepperNode) -> None:
            if n.node_id not in seen:
                nodes.append(n)
                seen.add(n.node_id)

        if self.corexy is not None:
            _add(self.corexy.axis_a.node)
            _add(self.corexy.axis_b.node)
        if self.cartesian is not None:
            for ax in self.cartesian._iter_axes():
                _add(ax.node)
                if isinstance(ax, DualMotorAxis):
                    _add(ax.secondary)
        for ax in self.axes.values():
            for node in _axis_nodes(ax):
                _add(node)
        return nodes

    # -- command handlers -----------------------------------------------------

    def _cmd_move(self, line: GCodeLine) -> str:
        # Ensure drivers are live; host status cache can still say estopped
        # briefly after M17 if telemetry has not refreshed.
        self._ensure_enabled()
        p = line.params
        if "F" in p:
            self.feedrate_mm_min = max(1.0, p["F"])

        targets: Dict[str, float] = {}
        for ax, raw in p.items():
            if ax in _NON_AXIS_WORDS:
                continue
            mapped = (
                ax in self.axes
                or (self.corexy is not None and ax in ("X", "Y"))
                or (self.cartesian is not None and ax in ("X", "Y", "Z"))
            )
            if not mapped:
                raise GCodeError(f"no axis mapped for {ax}", line.raw)
            abs_mode = self.absolute_e if ax == "E" else self.absolute
            if abs_mode:
                targets[ax] = raw
            else:
                targets[ax] = self.position.get(ax, 0.0) + raw

        if not targets:
            return "ok"

        speed_mm_s = (self.feedrate_mm_min / 60.0) * self.speed_factor
        speed_mm_s = max(0.1, speed_mm_s)

        # CoreXY plane (coupled belts)
        xy_keys = {k for k in targets if k in ("X", "Y")}
        if self.corexy is not None and xy_keys:
            x = targets.get("X", self.position["X"])
            y = targets.get("Y", self.position["Y"])
            self.corexy.move_to(
                x, y, speed=speed_mm_s, blocking=True, timeout=self.move_timeout
            )
            self.position["X"], self.position["Y"] = self.corexy.get_position()

        # Named cartesian mechanism (independent X/Y/Z, path-coordinated)
        cart_keys = {k for k in targets if k in ("X", "Y", "Z")}
        if self.cartesian is not None and cart_keys:
            if "Z" in targets and self.cartesian.z is None:
                raise GCodeError("no Z axis on cartesian mechanism", line.raw)
            x = targets.get("X", self.position["X"])
            y = targets.get("Y", self.position["Y"])
            z_arg: Optional[float] = None
            if self.cartesian.z is not None:
                z_arg = targets.get("Z", self.position.get("Z", 0.0))
            try:
                self.cartesian.move_to(
                    x,
                    y,
                    z_arg,
                    speed=speed_mm_s,
                    blocking=True,
                    timeout=self.move_timeout,
                )
            except ValueError as exc:
                raise GCodeError(str(exc), line.raw) from exc
            cx, cy, cz = self.cartesian.get_position()
            self.position["X"], self.position["Y"], self.position["Z"] = cx, cy, cz

        # Remaining linear axes (E, or ad-hoc from_axes without named kinematics)
        lin: Dict[str, float] = {}
        for ax, val in targets.items():
            if self.corexy is not None and ax in ("X", "Y"):
                continue
            if self.cartesian is not None and ax in ("X", "Y", "Z"):
                continue
            if ax not in self.axes:
                raise GCodeError(f"no axis mapped for {ax}", line.raw)
            lin[ax] = val

        if lin:
            self._move_cartesian(lin, speed_mm_s)

        # refresh any axis positions we own
        for ax in targets:
            owned_by_corexy = self.corexy is not None and ax in ("X", "Y")
            owned_by_cart = self.cartesian is not None and ax in ("X", "Y", "Z")
            if ax in self.axes and not owned_by_corexy and not owned_by_cart:
                try:
                    self.position[ax] = _axis_position(self.axes[ax])
                except Exception:
                    self.position[ax] = targets[ax]
            elif ax in targets and not owned_by_corexy and not owned_by_cart:
                self.position[ax] = targets[ax]

        return "ok"

    def _move_cartesian(self, targets: Dict[str, float], speed_mm_s: float) -> None:
        """Coordinated move for independent :class:`Axis` map."""
        # Path length in workspace for multi-axis sync
        cur = {k: self.position.get(k, _axis_position(self.axes[k])) for k in targets}
        dist = math.sqrt(sum((targets[k] - cur[k]) ** 2 for k in targets))
        if dist < 1e-9:
            return

        duals = {
            k: ax for k, ax in ((k, self.axes[k]) for k in targets)
            if isinstance(ax, IndependentDualAxis)
        }
        singles = [k for k in targets if k not in duals]
        if singles:
            mg = MotionGroup([self.axes[k] for k in singles])
            speeds = {}
            for k in singles:
                d = abs(targets[k] - cur[k])
                if d > 0:
                    speeds[self.axes[k]] = (d / dist) * speed_mm_s
            mg.move_to(
                targets={self.axes[k]: targets[k] for k in singles},
                speeds=speeds,
                blocking=True,
                timeout=self.move_timeout,
            )
        for k, ax in duals.items():
            d = abs(targets[k] - cur[k])
            spd = (d / dist) * speed_mm_s if dist > 0 else speed_mm_s
            ax.move_to(targets[k], speed_mm_s=spd)

    def _cmd_dwell(self, line: GCodeLine) -> str:
        # G4 P<ms> or S<seconds>
        p = line.params
        if "P" in p:
            time.sleep(max(0.0, p["P"] / 1000.0))
        elif "S" in p:
            time.sleep(max(0.0, p["S"]))
        return "ok"

    def _cmd_inches(self, line: GCodeLine) -> str:
        raise GCodeError("G20 inches not supported; use G21 millimetres", line.raw)

    def _cmd_mm(self, line: GCodeLine) -> str:
        return "ok"

    def _cmd_home(self, line: GCodeLine) -> str:
        """G28 — soft home: enable, set logical zero at current pose.

        Optional axes: ``G28 X Y``. Without letters, all known axes.
        Physical endstop/StallGuard homing can be wired via :meth:`register`.
        """
        letters = [L for L in line.params if L not in _NON_AXIS_WORDS]
        if not letters:
            letters = []
            if self.corexy is not None:
                letters.extend(["X", "Y"])
            if self.cartesian is not None:
                letters.extend(L for L in self.cartesian.axis_map() if L not in letters)
            letters.extend(L for L in self.axes if L not in letters)

        self._cmd_enable(line)

        home_xy = self.corexy is not None and (
            not letters or "X" in letters or "Y" in letters
        )
        if home_xy and self.corexy is not None:
            self.corexy.set_zero()
            self.position["X"] = 0.0
            self.position["Y"] = 0.0

        if self.cartesian is not None:
            home_all_cart = not any(L in line.params for L in ("X", "Y", "Z", "E"))
            cart_letters = [L for L in ("X", "Y", "Z") if L in letters]
            if home_all_cart or (
                cart_letters
                and set(cart_letters) >= set(self.cartesian.axis_map())
            ):
                # Full soft home of the mechanism
                self.cartesian.set_zero()
                self.position["X"] = 0.0
                self.position["Y"] = 0.0
                self.position["Z"] = 0.0
            else:
                for L in cart_letters:
                    if L in self.cartesian.axis_map():
                        self.cartesian.axis_map()[L].set_zero()
                        self.position[L] = 0.0

        for L in letters:
            if L in ("X", "Y") and self.corexy is not None:
                continue
            if L in ("X", "Y", "Z") and self.cartesian is not None:
                continue
            if L in self.axes:
                self.axes[L].set_zero()
                self.position[L] = 0.0

        return "ok"

    def _cmd_abs(self, line: GCodeLine) -> str:
        self.absolute = True
        return "ok"

    def _cmd_rel(self, line: GCodeLine) -> str:
        self.absolute = False
        return "ok"

    def _cmd_e_abs(self, line: GCodeLine) -> str:
        self.absolute_e = True
        return "ok"

    def _cmd_e_rel(self, line: GCodeLine) -> str:
        self.absolute_e = False
        return "ok"

    def _cmd_set_position(self, line: GCodeLine) -> str:
        """G92 — set logical position without moving (where supported)."""
        p = line.params
        requested = [L for L in p if L not in _NON_AXIS_WORDS]
        if not requested:
            # bare G92 → zero all
            return self._cmd_home(GCodeLine("G28", raw=line.raw))

        # CoreXY: set both motors so cartesian matches
        if self.corexy is not None and ("X" in p or "Y" in p):
            x = p.get("X", self.position["X"])
            y = p.get("Y", self.position["Y"])
            a, b = CoreXY.cartesian_to_motors(x, y)
            self.corexy.axis_a.node.set_logical_position(
                self.corexy.axis_a.units_to_deg(a)
            )
            self.corexy.axis_b.node.set_logical_position(
                self.corexy.axis_b.units_to_deg(b)
            )
            self.position["X"], self.position["Y"] = x, y

        # Named cartesian: identity map per axis
        if self.cartesian is not None:
            for L, ax in self.cartesian.axis_map().items():
                if L in p:
                    ax.node.set_logical_position(ax.units_to_deg(p[L]))
                    self.position[L] = p[L]

        for L in requested:
            if self.corexy is not None and L in ("X", "Y"):
                continue
            if self.cartesian is not None and L in ("X", "Y", "Z"):
                continue
            if L not in self.axes:
                raise GCodeError(f"no axis mapped for {L}", line.raw)
            _axis_set_logical(self.axes[L], p[L])
            self.position[L] = p[L]

        return "ok"

    def _ensure_enabled(self) -> None:
        for n in self._all_nodes():
            try:
                st = n.get_status()
                if st.estopped or not st.enabled:
                    n.enable()
            except Exception:
                try:
                    n.enable()
                except Exception:
                    pass

    def _cmd_enable(self, line: GCodeLine) -> str:
        for n in self._all_nodes():
            n.enable()
            # Force host cache clear by reading status after enable
            try:
                n.get_status()
            except Exception:
                pass
        return "ok"

    def _cmd_disable(self, line: GCodeLine) -> str:
        for n in self._all_nodes():
            n.disable()
        return "ok"

    def _cmd_temps(self, line: GCodeLine) -> str:
        # Optional: report MCU temps from first node
        nodes = self._all_nodes()
        if not nodes:
            return "ok T:0 /0 B:0 /0"
        try:
            t, _ = nodes[0].get_env()
            return f"ok T:{t:.1f} /0 B:0 /0"
        except Exception:
            return "ok T:0 /0 B:0 /0"

    def _cmd_estop(self, line: GCodeLine) -> str:
        if self.bus is not None:
            self.bus.estop_all()
        else:
            for n in self._all_nodes():
                n.estop()
        return "ok"

    def _cmd_get_position(self, line: GCodeLine) -> str:
        self._sync_position_from_hardware()
        parts = [
            f"X:{self.position.get('X', 0.0):.3f}",
            f"Y:{self.position.get('Y', 0.0):.3f}",
            f"Z:{self.position.get('Z', 0.0):.3f}",
            f"E:{self.position.get('E', 0.0):.3f}",
        ]
        return "ok " + " ".join(parts)

    def _cmd_firmware_info(self, line: GCodeLine) -> str:
        nodes = self._all_nodes()
        vers = []
        for n in nodes:
            try:
                vers.append(f"n{n.node_id}:{n.get_status().firmware}")
            except Exception:
                vers.append(f"n{n.node_id}:?")
        return "ok PROTOCOL:GCSP FIRMWARE:canstepper " + " ".join(vers)

    def _cmd_speed_factor(self, line: GCodeLine) -> str:
        if "S" in line.params:
            self.speed_factor = max(0.01, line.params["S"] / 100.0)
        return f"ok S{self.speed_factor * 100:.0f}"

    def _cmd_finish_moves(self, line: GCodeLine) -> str:
        # Moves are already blocking; nothing to drain.
        return "ok"
