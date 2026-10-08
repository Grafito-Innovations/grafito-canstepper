#!/usr/bin/env python3
"""Software home: declare the current pose as zero (no motion).

Same as firmware method 0 / G-code soft G28. Does not search for a switch
or hard stop.

Usage:
  PYTHONPATH=. python3 examples/home_set_zero.py [/dev/ttyACM0] [node_id]
"""

from __future__ import annotations

import sys

from canstepper import CANStepperBus

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NODE_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1


def main() -> None:
    with CANStepperBus.serial(PORT) as bus:
        print("discover", bus.discover())
        node = bus.node(NODE_ID)
        node.enable()

        before = node.get_position()
        print(f"position before: {before:.3f}°")

        node.home(method="set_zero")
        # equivalent: node.set_zero()

        after = node.get_position()
        st = node.get_status()
        print(f"position after:  {after:.3f}°  (expect ~0)")
        print(f"homed flag:      {st.homed}")


if __name__ == "__main__":
    main()
