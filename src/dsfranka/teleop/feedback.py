"""Haptic feedback policy: session state -> gamepad rumble/trigger/lightbar.

Channels
--------
high-freq motor   workspace boundary warning: ramps up as the target pose
                  approaches a face of the workspace box
low-freq motor    blocked/pressing feedback: the IK anti-windup gap
                  |q_ref - q| is a direct measure of how hard the position
                  servo is pushing (works identically in sim and on the real
                  robot; upgraded to O_F_ext_hat_K with protocol v2)
lightbar          red = recording, blue = idle

(The R2 trigger is driven by the session's gripper logic — a haptic detent at
the open/close hysteresis point — not here.)

All outputs go through the Gamepad feedback API, which is a no-op for
drivers without haptics (evdev, Mock records for tests).
"""
from __future__ import annotations

import numpy as np

from ..common.types import EETarget, RobotState
from ..input.gamepad import Gamepad


class FeedbackController:
    def __init__(self, cfg: dict):
        fb = cfg.get("feedback", {})
        self.enabled = bool(fb.get("enabled", True))
        self.boundary_margin = float(fb.get("boundary_margin", 0.05))
        self.boundary_max = float(fb.get("boundary_max", 0.6))
        self.blocked_deadband = float(fb.get("blocked_deadband", 0.25))
        self.blocked_max = float(fb.get("blocked_max", 0.9))
        self.cmd_lag_limit = float(cfg.get("ik", {}).get("cmd_lag_limit", 0.15))
        ws = cfg["workspace"]
        self.ws_lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
        self.ws_hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])

    # -- individual signals ------------------------------------------------
    def boundary_intensity(self, pos: np.ndarray) -> float:
        """0 away from the box faces -> boundary_max at/beyond a face."""
        d = float(min(np.min(pos - self.ws_lo), np.min(self.ws_hi - pos)))
        if d >= self.boundary_margin:
            return 0.0
        return (1.0 - max(d, 0.0) / self.boundary_margin) * self.boundary_max

    def blocked_intensity(self, state: RobotState, target: EETarget) -> float:
        """0 while tracking normally -> blocked_max when the anti-windup
        clamp is saturated (servo pushing as hard as we allow)."""
        if target.q_ref is None:
            return 0.0
        ratio = float(np.max(np.abs(target.q_ref - state.q))) / self.cmd_lag_limit
        if ratio <= self.blocked_deadband:
            return 0.0
        return min(1.0, (ratio - self.blocked_deadband) / (1.0 - self.blocked_deadband)) \
            * self.blocked_max

    # -- per-tick update -----------------------------------------------------
    def update(self, pad: Gamepad, pos: np.ndarray, state: RobotState,
               target: EETarget, recording: bool) -> None:
        if not self.enabled:
            return
        pad.set_rumble(self.blocked_intensity(state, target),
                       self.boundary_intensity(pos))
        pad.set_lightbar(*((255, 20, 20) if recording else (0, 0, 255)))
