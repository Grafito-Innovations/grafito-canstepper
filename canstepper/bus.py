"""CANStepperBus — connection, frame routing, discovery, request/response."""

from __future__ import annotations

import struct
import threading
import time
import math
from collections import defaultdict
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

from .exceptions import RequestTimeout
from .protocol import (
    BROADCAST,
    MAX_NODE_ID,
    Cmd,
    Event,
    Frame,
    Tel,
)
from .telemetry import NodeState
from .transport.base import Transport

TelemetryCallback = Callable[[Frame], None]
EventCallback = Callable[[int, Event, int, float], None]  # node, event, detail, data


class _Waiter:
    """One blocked caller waiting for a matching frame."""

    __slots__ = ("event", "frame")

    def __init__(self) -> None:
        self.event = threading.Event()
        self.frame: Optional[Frame] = None

    def resolve(self, frame: Frame) -> None:
        self.frame = frame
        self.event.set()


class EventWaiter:
    """Armed *before* a command is sent so a synchronous or fast reply
    cannot be missed; then blocks until one of the accepted events arrives."""

    def __init__(self, bus: "CANStepperBus", node_id: int, accepted: Sequence[Event]):
        self._bus = bus
        self.node_id = node_id
        self.accepted = {Event(e) for e in accepted}
        self._cond = threading.Condition()
        self._hits: List[Tuple[Event, int, float]] = []

    def _offer(self, event: Event, detail: int, data: float) -> None:
        if event not in self.accepted:
            return
        with self._cond:
            self._hits.append((event, detail, data))
            self._cond.notify_all()

    def wait(self, timeout: float) -> Optional[Tuple[Event, int, float]]:
        """Return ``(event, detail, data)`` or None on timeout."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while not self._hits:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)
            return self._hits.pop(0)

    def cancel(self) -> None:
        self._bus._remove_event_waiter(self)

    def __enter__(self) -> "EventWaiter":
        return self

    def __exit__(self, *_exc) -> None:
        self.cancel()


class CANStepperBus:
    """A bus of 1-31 GrafitoCANStepper nodes behind one transport.

    Typical entry point::

        bus = CANStepperBus.serial("/dev/ttyACM0")
        print(bus.discover())
        node = bus.node(1)
    """

    def __init__(self, transport: Transport):
        self._transport = transport
        self._lock = threading.RLock()
        self.states: Dict[int, NodeState] = {}
        self._pending: Dict[tuple, List[_Waiter]] = defaultdict(list)
        self._event_waiters: List[EventWaiter] = []
        self._subscribers: Dict[tuple, List[TelemetryCallback]] = defaultdict(list)
        self._event_subscribers: List[EventCallback] = []
        self._nodes: Dict[int, "StepperNode"] = {}
        self._closed = False
        transport.set_receiver(self._on_frame)

    # -- construction --------------------------------------------------------

    @classmethod
    def serial(cls, port: str, baudrate: int = 115200) -> "CANStepperBus":
        """Connect through a node's USB serial bridge."""
        from .transport.serial_bridge import SerialBridgeTransport

        return cls(SerialBridgeTransport(port, baudrate=baudrate))

    # -- node handles ---------------------------------------------------------

    def node(self, node_id: int) -> "StepperNode":
        """A (cached) handle for one node, 1-31."""
        from .node import StepperNode

        if not 1 <= node_id <= MAX_NODE_ID:
            raise ValueError(f"node_id must be 1-{MAX_NODE_ID}, got {node_id}")
        with self._lock:
            if node_id not in self._nodes:
                self._nodes[node_id] = StepperNode(self, node_id)
            return self._nodes[node_id]

    def group(self, node_ids: Sequence[int]) -> "NodeGroup":
        """A group handle fanning commands out to several nodes."""
        from .group import NodeGroup

        return NodeGroup(self, node_ids)

    # -- bus-wide safety ------------------------------------------------------

    def estop_all(self) -> None:
        """Emergency-stop every node with a single broadcast frame."""
        self.send_command(BROADCAST, Cmd.ESTOP)

    def stop_all(self) -> None:
        """Ramped stop on every node (drivers stay enabled)."""
        self.send_command(BROADCAST, Cmd.STOP)

    def enable_all(self, enabled: bool = True) -> None:
        self.send_command(BROADCAST, Cmd.ENABLE, struct.pack("<B", 1 if enabled else 0))

    def broadcast_move_to(
        self,
        degrees: float,
        wait_for: Sequence[int] = (),
        timeout: float = 60.0,
    ) -> None:
        """Send one absolute-position frame to every node on the CAN bus.

        Every enabled node receives the same target from the same broadcast
        frame and closes its own encoder PID loop. ``wait_for`` optionally
        lists the node IDs whose MOVE_DONE/fault events must be collected.

        This command addresses the whole bus, not a subset. Callers should
        verify that no unrelated motion nodes are connected before using it.
        """
        target = float(degrees)
        if not math.isfinite(target):
            raise ValueError("broadcast target must be finite")

        node_ids = list(dict.fromkeys(int(node_id) for node_id in wait_for))
        nodes = [self.node(node_id) for node_id in node_ids]
        waiters = [
            self.arm_events(
                node.node_id,
                [Event.MOVE_DONE, Event.FAULT, Event.ESTOP],
            )
            for node in nodes
        ]
        try:
            self.send_command(BROADCAST, Cmd.MOVE_ABS, struct.pack("<d", target))
            deadline = time.monotonic() + timeout
            for node, waiter in zip(nodes, waiters):
                remaining = max(0.0, deadline - time.monotonic())
                node._wait_motion_event(waiter, remaining, "broadcast move")
        finally:
            for waiter in waiters:
                waiter.cancel()

    # -- discovery ------------------------------------------------------------

    def discover(self, timeout: float = 1.0) -> Dict[int, str]:
        """Broadcast a PING and return ``{node_id: firmware_version}``.

        Nodes answering within ``timeout`` (or already known from periodic
        STATUS telemetry received during the window) are included.
        """
        seen: Dict[int, str] = {}
        self.send_command(BROADCAST, Cmd.PING)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.02)
            with self._lock:
                for nid, state in self.states.items():
                    if state.status is not None and state.age("status") <= timeout:
                        seen[nid] = state.status.firmware
        return dict(sorted(seen.items()))

    # -- raw send / request ---------------------------------------------------

    def send_command(self, node_id: int, cmd: Union[Cmd, int], data: bytes = b"") -> None:
        self._transport.send(Frame.command(node_id, int(cmd), data))

    def send_frame(self, frame: Frame) -> None:
        self._transport.send(frame)

    def request(
        self,
        node_id: int,
        tel: Union[Tel, int],
        timeout: float = 1.0,
        retries: int = 1,
    ) -> bytes:
        """Send an RTR request and return the response payload."""
        key = ("tel", node_id, int(tel))
        attempts = max(1, retries + 1)
        for attempt in range(attempts):
            waiter = _Waiter()
            with self._lock:
                self._pending[key].append(waiter)
            self._transport.send(Frame.rtr_request(node_id, int(tel)))
            if waiter.event.wait(timeout):
                return waiter.frame.data  # type: ignore[union-attr]
            with self._lock:
                if waiter in self._pending.get(key, []):
                    self._pending[key].remove(waiter)
            if attempt == attempts - 1:
                raise RequestTimeout(node_id, Tel(int(tel)).name, timeout)
        raise AssertionError("unreachable")

    def request_param_frame(
        self,
        node_id: int,
        cmd: Cmd,
        payload: bytes,
        param_id: int,
        timeout: float = 1.0,
        retries: int = 1,
    ) -> bytes:
        """Send SET_PARAM/GET_PARAM and wait for the matching PARAM reply."""
        key = ("param", node_id, param_id)
        attempts = max(1, retries + 1)
        for attempt in range(attempts):
            waiter = _Waiter()
            with self._lock:
                self._pending[key].append(waiter)
            self.send_command(node_id, cmd, payload)
            if waiter.event.wait(timeout):
                return waiter.frame.data  # type: ignore[union-attr]
            with self._lock:
                if waiter in self._pending.get(key, []):
                    self._pending[key].remove(waiter)
            if attempt == attempts - 1:
                raise RequestTimeout(node_id, f"PARAM({param_id})", timeout)
        raise AssertionError("unreachable")

    # -- subscriptions ---------------------------------------------------------

    def subscribe(
        self,
        node_id: Optional[int],
        msg_id: Optional[Union[Tel, int]],
        callback: TelemetryCallback,
    ) -> None:
        """Call ``callback(frame)`` for matching telemetry.

        ``node_id=None`` matches every node; ``msg_id=None`` every message.
        Callbacks run on the RX thread — keep them fast, hand off real work.
        """
        key = (node_id, None if msg_id is None else int(msg_id))
        with self._lock:
            self._subscribers[key].append(callback)

    def on_event(self, callback: EventCallback) -> None:
        """Call ``callback(node_id, event, detail, data)`` for every EVENT."""
        with self._lock:
            self._event_subscribers.append(callback)

    def arm_events(self, node_id: int, accepted: Sequence[Event]) -> EventWaiter:
        """Arm an event waiter *before* sending the command that triggers it."""
        waiter = EventWaiter(self, node_id, accepted)
        with self._lock:
            self._event_waiters.append(waiter)
        return waiter

    def _remove_event_waiter(self, waiter: EventWaiter) -> None:
        with self._lock:
            if waiter in self._event_waiters:
                self._event_waiters.remove(waiter)

    # -- RX path ---------------------------------------------------------------

    def _on_frame(self, frame: Frame) -> None:
        if frame.rtr:
            return
        node_id = frame.node_id
        msg_id = frame.msg_id
        if msg_id < 32:
            return  # a command echo from another host; not telemetry

        with self._lock:
            state = self.states.get(node_id)
            if state is None:
                state = self.states[node_id] = NodeState(node_id)
            state.update(frame)

            # Resolve blocked requesters.
            for key in (("tel", node_id, msg_id),):
                waiters = self._pending.pop(key, None)
                if waiters:
                    for w in waiters:
                        w.resolve(frame)
            if msg_id == Tel.PARAM and len(frame.data) >= 2:
                key = ("param", node_id, frame.data[0])
                waiters = self._pending.pop(key, None)
                if waiters:
                    for w in waiters:
                        w.resolve(frame)

            event_waiters = list(self._event_waiters)
            subscriber_lists = (
                self._subscribers.get((node_id, msg_id), [])
                + self._subscribers.get((node_id, None), [])
                + self._subscribers.get((None, msg_id), [])
                + self._subscribers.get((None, None), [])
            )
            event_subscribers = list(self._event_subscribers)

        # Event decode + fan-out outside the lock.
        if msg_id == Tel.EVENT and len(frame.data) >= 6:
            try:
                evt = Event(frame.data[0])
            except ValueError:
                evt = None
            if evt is not None:
                detail = frame.data[1]
                data = struct.unpack("<f", frame.data[2:6])[0]
                for waiter in event_waiters:
                    if waiter.node_id in (node_id, BROADCAST):
                        waiter._offer(evt, detail, data)
                for cb in event_subscribers:
                    try:
                        cb(node_id, evt, detail, data)
                    except Exception:
                        pass

        for cb in subscriber_lists:
            try:
                cb(frame)
            except Exception:
                pass

    # -- lifecycle ---------------------------------------------------------------

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._transport.close()

    def __enter__(self) -> "CANStepperBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
