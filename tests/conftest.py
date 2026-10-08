import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from canstepper import CANStepperBus  # noqa: E402
from canstepper.sim import SimNetwork  # noqa: E402


@pytest.fixture()
def net():
    return SimNetwork([1, 2, 3])


@pytest.fixture()
def bus(net):
    b = CANStepperBus(net)
    yield b
    b.close()
