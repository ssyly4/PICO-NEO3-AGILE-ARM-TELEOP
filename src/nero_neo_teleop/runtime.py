"""Filesystem and external-workspace configuration shared by runtime modules."""

from __future__ import annotations

import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS_DIR = Path(
    os.environ.get("NERO_TELEOP_ARTIFACTS_DIR", PROJECT_ROOT / "artifacts")
).expanduser()
NERO_WS = Path(os.environ.get("NERO_WS", Path.home() / "nero_ws")).expanduser()
PYAGXARM_SOURCE = Path(
    os.environ.get("PYAGXARM_SOURCE", NERO_WS / "src" / "pyAgxArm")
).expanduser()
NERO_URDF = Path(
    os.environ.get(
        "NERO_URDF",
        NERO_WS
        / "src/piper_ros/src/robot_description/nero_description/urdf/nero_description.urdf",
    )
).expanduser()
