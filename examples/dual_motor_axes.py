#!/usr/bin/env python3
"""Run the safe Node 1 + Node 2 DualMotorAxis test from examples/."""

from __future__ import annotations

import sys
from pathlib import Path

# The complete, easy-to-find test lives at can_stepper/dual_motor_axes.py.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dual_motor_axes import main  # noqa: E402


if __name__ == "__main__":
    main()
