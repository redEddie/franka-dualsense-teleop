"""Shared datatypes for the teleop pipeline."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class RobotState:
    """Snapshot of the arm, identical layout for sim and real."""

    q: np.ndarray            # (7,) joint positions [rad]
    dq: np.ndarray           # (7,) joint velocities [rad/s]
    ee_pos: np.ndarray       # (3,) TCP position in base frame [m]
    ee_quat: np.ndarray      # (4,) TCP orientation, wxyz
    gripper_width: float     # [m], 0..0.08
    t: float                 # backend timestamp [s]


@dataclass
class EETarget:
    """Command produced by the teleop session each tick."""

    pos: np.ndarray                       # (3,)
    quat: np.ndarray                      # (4,) wxyz
    gripper: float = 1.0                  # 0 = closed, 1 = fully open
    q_ref: np.ndarray | None = field(default=None)  # optional IK solution (real robot path)
