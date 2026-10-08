"""USB CDC serial bridge transport.

Any GrafitoCANStepper node bridges its USB serial port to the CAN bus:
lines written to the port become CAN frames (and are executed by the
bridging node when addressed to it), and every frame the node sees on the
bus — plus its own telemetry — is printed as a line.
"""

from __future__ import annotations

import threading
import time

from ..exceptions import TransportError
from ..protocol import Frame, decode_line, encode_line
from .base import Transport


class SerialBridgeTransport(Transport):
    """Speak GCSP over a node's USB serial port.

    Requires ``pyserial`` (``pip install pyserial``); imported lazily so the
    rest of the package (and the simulator) works without it.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        super().__init__()
        try:
            import serial  # pyserial, imported lazily
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise TransportError(
                "pyserial is required for the serial bridge transport: "
                "pip install pyserial"
            ) from exc

        try:
            self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        except Exception as exc:
            raise TransportError(f"cannot open serial port {port!r}: {exc}") from exc

        # USB-JTAG on ESP32-C3 resets when DTR/RTS toggle. Hold them down.
        try:
            self._ser.dtr = False
            self._ser.rts = False
        except Exception:
            pass
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

        self._tx_lock = threading.Lock()
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name=f"canstepper-rx-{port}", daemon=True
        )
        self._rx_thread.start()

    def send(self, frame: Frame) -> None:
        line = encode_line(frame)
        with self._tx_lock:
            try:
                self._ser.write(line)
            except Exception as exc:
                raise TransportError(f"serial write failed: {exc}") from exc

    def _rx_loop(self) -> None:
        # Chunked reads + manual line split. readline() reads byte-at-a-time and
        # the ESP32 USB-JTAG gadget raises spurious select() wakeups (readable
        # but 0 bytes), so readline() busy-spins between select and read —
        # ~40-50% of a Pi core even while idle. Chunk reads amortize the syscall
        # and an explicit backoff on empty reads kills the spin.
        buf = b""
        while self._running:
            try:
                raw = self._ser.read(1024)
            except Exception:
                if self._running:
                    continue
                return
            if not raw:
                # Spurious wakeup or true silence: yield instead of spinning.
                time.sleep(0.002)
                continue
            buf += raw
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                frame = decode_line(line)
                if frame is not None:
                    try:
                        self._deliver(frame)
                    except Exception:
                        # A misbehaving consumer must not kill the RX loop.
                        pass

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            self._ser.close()
        except Exception:
            pass
        self._rx_thread.join(timeout=1.0)
