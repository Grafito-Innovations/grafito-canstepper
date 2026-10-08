"""NodeGroup — fan a command out to a set of node IDs."""

from __future__ import annotations

import time
from typing import Dict, List, Sequence, Union

from .exceptions import RequestTimeout
from .node import StepperNode
from .protocol import Param
from .telemetry import NodeState


class NodeGroup:
    """Operate on several nodes as one unit (e.g. all feeders, both Z motors).

    Commands are sent per node back-to-back (~0.15 ms apart on a 1 Mbps
    bus). For a whole-bus atomic emergency stop use ``bus.estop_all()``,
    which is a single broadcast frame.
    """

    def __init__(self, bus, node_ids: Sequence[int]):
        if not node_ids:
            raise ValueError("a NodeGroup needs at least one node id")
        self._bus = bus
        self.nodes: List[StepperNode] = [bus.node(n) for n in node_ids]

    @property
    def node_ids(self) -> List[int]:
        return [n.node_id for n in self.nodes]

    # -- safety -----------------------------------------------------------------

    def estop(self) -> "NodeGroup":
        """Emergency-stop every node in this group."""
        for n in self.nodes:
            n.estop()
        return self

    def stop(self) -> "NodeGroup":
        for n in self.nodes:
            n.stop()
        return self

    def enable(self) -> "NodeGroup":
        for n in self.nodes:
            n.enable()
        return self

    def disable(self) -> "NodeGroup":
        for n in self.nodes:
            n.disable()
        return self

    # -- motion ------------------------------------------------------------------

    def move_to(self, degrees: float) -> "NodeGroup":
        """Send the same absolute target to every node."""
        for n in self.nodes:
            n.move_to(degrees)
        return self

    def move_by(self, degrees: float) -> "NodeGroup":
        for n in self.nodes:
            n.move_by(degrees)
        return self

    def run(self, deg_per_sec: float) -> "NodeGroup":
        for n in self.nodes:
            n.run(deg_per_sec)
        return self

    def set_zero(self) -> "NodeGroup":
        for n in self.nodes:
            n.set_zero()
        return self

    # -- configuration ---------------------------------------------------------------

    def set_param(self, param: Union[Param, int, str], value) -> "NodeGroup":
        """Set (and verify) a parameter on every node in the group."""
        for n in self.nodes:
            n.set_param(param, value)
        return self

    def save_config(self) -> "NodeGroup":
        for n in self.nodes:
            n.save_config()
        return self

    def each(self, fn) -> "NodeGroup":
        """Run ``fn(node)`` on every node — escape hatch for anything else."""
        for n in self.nodes:
            fn(n)
        return self

    # -- state ------------------------------------------------------------------------

    def states(self) -> Dict[int, NodeState]:
        return {n.node_id: n.state for n in self.nodes}

    def wait_all_settled(self, timeout: float = 30.0) -> "NodeGroup":
        """Block until every node reports settled (closed loop) or stopped."""
        deadline = time.monotonic() + timeout
        for n in self.nodes:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RequestTimeout(n.node_id, "group settle", timeout)
            n.wait_settled(timeout=remaining)
        return self

    def __repr__(self) -> str:
        return f"<NodeGroup {self.node_ids}>"
