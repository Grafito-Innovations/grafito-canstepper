"""Transport abstraction: anything that can move GCSP frames."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Optional

from ..protocol import Frame

FrameReceiver = Callable[[Frame], None]


class Transport(ABC):
    """Moves frames between the host and the CAN bus.

    Implementations must be thread-safe for ``send`` and must deliver every
    received frame to the receiver callback installed with ``set_receiver``
    (from any thread; the bus layer serializes its own state).
    """

    def __init__(self) -> None:
        self._receiver: Optional[FrameReceiver] = None

    def set_receiver(self, receiver: FrameReceiver) -> None:
        self._receiver = receiver

    def _deliver(self, frame: Frame) -> None:
        if self._receiver is not None:
            self._receiver(frame)

    @abstractmethod
    def send(self, frame: Frame) -> None:
        """Transmit one frame toward the bus."""

    @abstractmethod
    def close(self) -> None:
        """Release the underlying resource. Idempotent."""

    def __enter__(self) -> "Transport":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
