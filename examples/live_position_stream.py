"""Consume streamed telemetry without polling.

Every node broadcasts POSITION at the fast rate (param fast_rate_hz,
default 10 Hz). Subscribe once and the callback runs for every frame;
`node.state` always holds the latest decoded values.
"""

import struct
import time

from canstepper import CANStepperBus, Event, Tel

PORT = "/dev/ttyACM0"

with CANStepperBus.serial(PORT) as bus:
    def on_position(frame):
        deg = struct.unpack("<d", frame.data[:8])[0]
        print(f"node {frame.node_id}: {deg:8.2f} deg")

    bus.subscribe(node_id=1, msg_id=Tel.POSITION, callback=on_position)
    bus.on_event(lambda nid, evt, detail, data:
                 print(f"EVENT node {nid}: {evt.name} detail={detail} data={data:.2f}"))

    node = bus.node(1)
    node.enable()
    node.set_param("fast_rate_hz", 20)
    node.move_to(720.0)

    time.sleep(5.0)
    print("cached state:", node.state.position_deg, node.state.velocity_deg_s)
