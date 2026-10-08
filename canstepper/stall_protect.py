"""Host-side StallGuard crash protection for multi-axis machines.

Firmware only *reports* mid-move DIAG (EVT_STALL). This helper arms after
motion is underway, listens for ``Event.STALL``, and broadcasts
``bus.estop_all()`` so a jam on one screw does not rack a dual frame.

Typical dual-frame use (Phase 2)::

    protect = StallGuardEstop(bus, [2, 3], threshold=12)
    protect.attach()
    protect.apply_to_nodes(stealthchop=True)  # StallGuard4 needs StealthChop

    protect.disarm()
    dual.move_to(20.0, wait_encoders=False)
    time.sleep(0.4)
    protect.arm()
    dual.wait_both_at(20.0, poll=protect.raise_if_tripped)
    protect.disarm()

Or use :meth:`IndependentDualAxis.move_to` with ``poll=protect.raise_if_tripped``
and arm after a short settle / min travel yourself.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Iterable, List, Optional, Sequence, Set, Union

from .exceptions import StallGuardTrip
from .node import StepperNode
from .protocol import Event, Param


class StallGuardEstop:
    """Armable host reaction to StallGuard DIAG events.

    Parameters
    ----------
    bus:
        :class:`~canstepper.bus.CANStepperBus` instance.
    node_ids:
        Nodes that may raise STALL and that share the estop (e.g. both screws).
    threshold:
        TMC SGTHRS (0–255). Higher = more sensitive. Dual-frame Phase 1 used ~12.
    on_trip:
        Optional callback ``(node_id, position_deg)`` after estop (logging/UI).
    quiet:
        If True, do not print trip lines to stdout.
    """

    def __init__(
        self,
        bus,
        node_ids: Sequence[int],
        *,
        threshold: int = 12,
        on_trip: Optional[Callable[[int, float], None]] = None,
        quiet: bool = False,
    ):
        ids = [int(n) for n in node_ids]
        if not ids:
            raise ValueError("StallGuardEstop needs at least one node id")
        self._bus = bus
        self.node_ids: List[int] = ids
        self._id_set: Set[int] = set(ids)
        self.threshold = int(threshold)
        self._on_trip = on_trip
        self.quiet = bool(quiet)

        self._lock = threading.Lock()
        self._armed = False
        self._tripped = False
        self._trip_node: Optional[int] = None
        self._trip_pos: float = 0.0
        self._attached = False
        self._ignored = 0

    # -- state -----------------------------------------------------------------

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def tripped(self) -> bool:
        with self._lock:
            return self._tripped

    @property
    def trip_node_id(self) -> Optional[int]:
        with self._lock:
            return self._trip_node

    @property
    def trip_position_deg(self) -> float:
        with self._lock:
            return self._trip_pos

    @property
    def ignored_stalls(self) -> int:
        with self._lock:
            return self._ignored

    # -- lifecycle -------------------------------------------------------------

    def attach(self) -> "StallGuardEstop":
        """Subscribe to bus EVENT frames (idempotent)."""
        if self._attached:
            return self
        self._bus.on_event(self._on_event)
        self._attached = True
        return self

    def apply_to_nodes(
        self,
        nodes: Optional[Iterable[StepperNode]] = None,
        *,
        stealthchop: bool = True,
        threshold: Optional[int] = None,
    ) -> "StallGuardEstop":
        """Write SGTHRS (+ optional StealthChop) on the protected nodes.

        Call **after** closed-loop / motion config that may reset driver mode.
        TMC2209 StallGuard4 is reliable only with StealthChop enabled.
        """
        thr = self.threshold if threshold is None else int(threshold)
        self.threshold = thr
        if nodes is None:
            nodes = [self._bus.node(i) for i in self.node_ids]
        for n in nodes:
            if n.node_id not in self._id_set:
                continue
            n.set_stall_threshold(thr)
            if stealthchop:
                n.set_stealthchop(True)
        return self

    def reset(self) -> "StallGuardEstop":
        """Clear trip latch (does not clear firmware e-stop — call enable)."""
        with self._lock:
            self._tripped = False
            self._trip_node = None
            self._trip_pos = 0.0
            self._ignored = 0
            self._armed = False
        return self

    def arm(self) -> "StallGuardEstop":
        """Start reacting to STALL (estop on next armed event)."""
        with self._lock:
            if self._tripped:
                return self
            self._armed = True
        return self

    def disarm(self) -> "StallGuardEstop":
        """Ignore STALL (start/stop glitches, intentional halt)."""
        with self._lock:
            self._armed = False
        return self

    def raise_if_tripped(self) -> None:
        """No-op or raise :class:`StallGuardTrip` — use as ``poll=`` callback."""
        with self._lock:
            if not self._tripped:
                return
            nid = self._trip_node if self._trip_node is not None else self.node_ids[0]
            pos = self._trip_pos
        raise StallGuardTrip(nid, pos)

    def arm_after(
        self,
        settle_s: float = 0.4,
        *,
        min_travel_mm: float = 0.0,
        position_fn: Optional[Callable[[], float]] = None,
        poll_s: float = 0.05,
        timeout_s: float = 30.0,
    ) -> "StallGuardEstop":
        """Wait then arm.

        If ``min_travel_mm > 0``, ``position_fn`` must return progress in mm
        (e.g. mean abs encoder mm from leg start). Arms when progress reaches
        the threshold or ``timeout_s`` elapses (then arms anyway if not tripped).
        """
        t0 = time.monotonic()
        if settle_s > 0:
            time.sleep(settle_s)
        if min_travel_mm > 0 and position_fn is not None:
            while time.monotonic() - t0 < timeout_s:
                self.raise_if_tripped()
                try:
                    if float(position_fn()) >= min_travel_mm:
                        break
                except StallGuardTrip:
                    raise
                except Exception:
                    pass
                time.sleep(poll_s)
        if not self.tripped:
            self.arm()
        return self

    # -- internal --------------------------------------------------------------

    def _on_event(self, node_id, event, detail, data) -> None:
        if event != Event.STALL:
            return
        if int(node_id) not in self._id_set:
            return
        pos = float(data or 0.0)
        with self._lock:
            if not self._armed:
                self._ignored += 1
                return
            if self._tripped:
                return
            self._tripped = True
            self._armed = False
            self._trip_node = int(node_id)
            self._trip_pos = pos
        # Outside lock: stop the whole bus ASAP
        try:
            self._bus.estop_all()
        except Exception:
            for nid in self.node_ids:
                try:
                    self._bus.node(nid).estop()
                except Exception:
                    pass
        if not self.quiet:
            print(
                f"\n  *** StallGuardEstop: STALL node={node_id} "
                f"@ {pos:.2f}° → estop_all() ***\n",
                flush=True,
            )
        if self._on_trip is not None:
            try:
                self._on_trip(int(node_id), pos)
            except Exception:
                pass
