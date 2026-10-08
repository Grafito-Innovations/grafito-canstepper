"""Compatibility shim for older pip/setuptools editable installs.

Project metadata lives in pyproject.toml.  Ubuntu 22.04's pip 22.0 may fall
back to the legacy editable-install path, which requires setup.py even when
the isolated build environment supports modern wheel builds.
"""

from setuptools import find_packages, setup


setup(
    name="grafito-canstepper",
    version="0.2.2",
    description="Host library for Grafito CANStepper boards (GCSP v1)",
    packages=find_packages(include=("canstepper", "canstepper.*")),
    package_data={"canstepper": ["py.typed"]},
    python_requires=">=3.9",
    install_requires=["pyserial>=3.4"],
)
