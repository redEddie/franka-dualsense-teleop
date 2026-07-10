"""Teleop session: button mapping + target-pose state machine.

Backend-agnostic — drives anything exposing the ArmBackend protocol
(MujocoArm for sim, FrankaArm for the real robot).

DualSense mapping
-----------------
left stick      x-y translation (base frame)
right stick     orientation (rotation about world x/y)
L1 / R1         yaw - / +
d-pad up/down   z up / down (manual)
L2 / R2         gripper open / close (analog rate)
Cross           auto-descend to configured height
Triangle        home (EE pose of the home configuration)
Square          orientation reset (home orientation, position kept)
Circle          cycle tilt 0/30/60/90 deg about configured axis
Options         episode recording start / stop+save
Create          discard current recording
PS              quit session
"""
from __future__ import annotations

import mujoco
import numpy as np

from ..common.rate import Rate
from ..common.types import EETarget
from ..data.recorder import EpisodeRecorder
from ..input.gamepad import Gamepad, GamepadState

AUTO_CANCEL_STICK = 0.5   # stick deflection that cancels an auto-move
AUTO_DONE_POS = 2e-3      # [m]
AUTO_DONE_ROT = 1e-2      # [rad]


def _rotate_quat_world(quat: np.ndarray, rotvec: np.ndarray) -> np.ndarray:
    """Apply a world-frame rotation (axis*angle) to a wxyz quaternion."""
    dq = np.zeros(4)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return quat
    mujoco.mju_axisAngle2Quat(dq, rotvec / angle, angle)
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, dq, quat)
    mujoco.mju_normalize4(out)
    return out


def _quat_err_world(target: np.ndarray, cur: np.ndarray) -> np.ndarray:
    """World-frame rotation vector taking cur -> target."""
    conj = np.zeros(4)
    err_q = np.zeros(4)
    mujoco.mju_negQuat(conj, cur)
    mujoco.mju_mulQuat(err_q, target, conj)
    vec = np.zeros(3)
    mujoco.mju_quat2Vel(vec, err_q, 1.0)
    return vec


class TeleopSession:
    def __init__(self, cfg: dict, arm, gamepad: Gamepad, recorder: EpisodeRecorder | None = None):
        self.cfg = cfg
        self.arm = arm
        self.gamepad = gamepad
        self.recorder = recorder or EpisodeRecorder(cfg["recorder"]["out_dir"])
        self.dt = 1.0 / cfg["control"]["rate_hz"]
        self.speed = cfg["speed"]
        ws = cfg["workspace"]
        self.ws_lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
        self.ws_hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])
        self.tilt_angles = [np.deg2rad(a) for a in cfg["features"]["tilt_angles_deg"]]
        self.tilt_axis = np.asarray(cfg["features"]["tilt_axis"], dtype=float)
        self.tilt_axis /= np.linalg.norm(self.tilt_axis)
        self.tilt_idx = 0
        self.descend_z = float(cfg["features"]["descend_z"])

        # target state, initialized from the arm's home pose
        self.home_pos, self.home_quat = arm.home_pose()
        self.pos = self.home_pos.copy()
        self.quat = self.home_quat.copy()
        self.gripper = 1.0

        # auto-move goal: (pos|None, quat|None)
        self._auto: tuple[np.ndarray | None, np.ndarray | None] | None = None
        self.quit = False

    # ------------------------------------------------------------------
    def _on_button(self, name: str):
        if name == "options":
            if self.recorder.recording:
                path = self.recorder.stop(save=True)
                print(f"[rec] saved -> {path}")
            else:
                self.recorder.start()
                print("[rec] recording started")
        elif name == "create":
            if self.recorder.recording:
                self.recorder.stop(save=False)
                print("[rec] discarded")
        elif name == "triangle":
            self._auto = (self.home_pos.copy(), self.home_quat.copy())
            print("[auto] homing")
        elif name == "square":
            self._auto = (None, self.home_quat.copy())
            print("[auto] orientation reset")
        elif name == "circle":
            self.tilt_idx = (self.tilt_idx + 1) % len(self.tilt_angles)
            ang = self.tilt_angles[self.tilt_idx]
            goal = _rotate_quat_world(self.home_quat, self.tilt_axis * ang)
            self._auto = (None, goal)
            print(f"[auto] tilt {np.rad2deg(ang):.0f} deg")
        elif name == "cross":
            goal = self.pos.copy()
            goal[2] = self.descend_z
            self._auto = (goal, None)
            print(f"[auto] descend to z={self.descend_z:.3f}")
        elif name == "ps":
            self.quit = True

    # ------------------------------------------------------------------
    def _integrate_manual(self, gp: GamepadState):
        v = np.zeros(3)
        v[0] = gp.ly * self.speed["xy"]          # stick up -> +x (away from base)
        v[1] = -gp.lx * self.speed["xy"]         # stick left -> +y
        if gp.is_held("dpad_up"):
            v[2] += self.speed["z"]
        if gp.is_held("dpad_down"):
            v[2] -= self.speed["z"]
        self.pos = np.clip(self.pos + v * self.dt, self.ws_lo, self.ws_hi)

        w = np.zeros(3)
        w[0] = -gp.rx * self.speed["rot"]        # right stick x -> roll about world x
        w[1] = gp.ry * self.speed["rot"]         # right stick y -> pitch about world y
        if gp.is_held("l1"):
            w[2] += self.speed["yaw"]
        if gp.is_held("r1"):
            w[2] -= self.speed["yaw"]
        self.quat = _rotate_quat_world(self.quat, w * self.dt)

        g_rate = (gp.l2 - gp.r2) * self.speed["gripper"] / 0.08  # width-rate -> [0,1]-rate
        self.gripper = float(np.clip(self.gripper + g_rate * self.dt, 0.0, 1.0))

    def _integrate_auto(self):
        goal_pos, goal_quat = self._auto
        v_auto = self.speed.get("auto_xyz", self.speed["xy"])
        w_auto = self.speed.get("auto_rot", self.speed["rot"])
        done = True
        if goal_pos is not None:
            err = goal_pos - self.pos
            dist = np.linalg.norm(err)
            if dist > 1e-9:
                step = min(v_auto * self.dt, dist)
                self.pos = np.clip(self.pos + err / dist * step, self.ws_lo, self.ws_hi)
            done &= bool(np.linalg.norm(goal_pos - self.pos) < AUTO_DONE_POS)
        if goal_quat is not None:
            vec = _quat_err_world(goal_quat, self.quat)
            ang = np.linalg.norm(vec)
            max_step = w_auto * self.dt
            if ang > max_step:
                vec *= max_step / ang
            self.quat = _rotate_quat_world(self.quat, vec)
            done &= bool(np.linalg.norm(_quat_err_world(goal_quat, self.quat)) < AUTO_DONE_ROT)
        if done:
            self._auto = None
            print("[auto] done")

    # ------------------------------------------------------------------
    def tick(self, gp: GamepadState | None = None):
        gp = gp if gp is not None else self.gamepad.poll()
        for name in gp.pressed:
            self._on_button(name)

        if self._auto is not None:
            if max(abs(gp.lx), abs(gp.ly), abs(gp.rx), abs(gp.ry)) > AUTO_CANCEL_STICK:
                self._auto = None
                print("[auto] cancelled by stick input")
            else:
                self._integrate_auto()
        if self._auto is None:
            self._integrate_manual(gp)

        target = EETarget(pos=self.pos.copy(), quat=self.quat.copy(), gripper=self.gripper)
        state = self.arm.apply(target, self.dt)
        self.recorder.add(state, target)
        return state, target

    def run(self, max_ticks: int | None = None, on_tick=None):
        rate = Rate(1.0 / self.dt)
        n = 0
        try:
            while not self.quit:
                state, target = self.tick()
                if on_tick is not None and on_tick(state, target) is False:
                    break
                n += 1
                if max_ticks is not None and n >= max_ticks:
                    break
                rate.sleep()
        finally:
            if self.recorder.recording:
                self.recorder.stop(save=False)
                print("[rec] active recording discarded on exit")
            self.gamepad.close()
