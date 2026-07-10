"""Teleop session: button mapping + target-pose state machine.

Backend-agnostic — drives anything exposing the ArmBackend protocol
(MujocoArm for sim, FrankaArm for the real robot).

DualSense mapping
-----------------
left stick      x-y translation (base frame)
right stick     z height, continuous (up/down)
L1 / R1         yaw + / -
d-pad           tilt direction select (up/down/left/right, base frame)
Cross           tap: tilt +30 deg step (snaps up to the 30-deg grid)
                hold: slow continuous tilt increase
Circle          tap: tilt -30 deg step toward 0 (snaps down to the grid)
                hold: slow continuous return
Square          orientation reset (yaw & tilt -> 0, position kept)
Triangle        home (EE pose of the home configuration)
R3              auto-descend to configured height
L2 / R2         gripper open / close (analog rate)
Create          episode recording start / stop+save   (capture key)
Options         discard current recording             (menu key)
PS              quit session

Orientation model: quat = yaw(world z) o tilt(selected axis, angle) o home.
Yaw and tilt are tracked as scalars so taps can snap the tilt angle onto the
30-degree grid (e.g. 74 -> 90 on Cross, 74 -> 60 on Circle).
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

DEFAULT_TILT_AXES = {     # world-frame rotation axis per d-pad direction
    "dpad_up": [0.0, -1.0, 0.0],     # tip toward +x (away from base)
    "dpad_down": [0.0, 1.0, 0.0],    # tip toward -x
    "dpad_left": [1.0, 0.0, 0.0],    # tip toward +y
    "dpad_right": [-1.0, 0.0, 0.0],  # tip toward -y
}


def _rotate_quat_world(quat: np.ndarray, rotvec) -> np.ndarray:
    """Apply a world-frame rotation (axis*angle) to a wxyz quaternion."""
    rotvec = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return quat
    dq = np.zeros(4)
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
        accel = cfg.get("accel", {})
        self.acc_xyz = float(accel.get("xyz", 1.0))
        self.acc_yaw = float(accel.get("yaw", 4.0))
        # slew-rate-limited velocity state (smooth ramps instead of stick steps)
        self._v = np.zeros(3)
        self._yaw_v = 0.0
        ws = cfg["workspace"]
        self.ws_lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
        self.ws_hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])
        self.descend_z = float(cfg["features"]["descend_z"])

        tilt = cfg.get("tilt", {})
        self.tilt_step = float(tilt.get("step_deg", 30.0))
        self.tilt_max = float(tilt.get("max_deg", 90.0))
        self.tilt_hold_speed = float(tilt.get("hold_speed_deg", 40.0))
        self.hold_threshold = float(tilt.get("hold_threshold_s", 0.35))
        axes = {**DEFAULT_TILT_AXES, **tilt.get("axes", {})}
        self.tilt_axes = {k: np.asarray(v, dtype=float) / np.linalg.norm(v)
                          for k, v in axes.items()}
        if tilt.get("invert_ud", False):
            self.tilt_axes["dpad_up"], self.tilt_axes["dpad_down"] = (
                self.tilt_axes["dpad_down"], self.tilt_axes["dpad_up"])
        if tilt.get("invert_lr", False):
            self.tilt_axes["dpad_left"], self.tilt_axes["dpad_right"] = (
                self.tilt_axes["dpad_right"], self.tilt_axes["dpad_left"])

        # target state, initialized from the arm's home pose
        self.home_pos, self.home_quat = arm.home_pose()
        self.pos = self.home_pos.copy()
        self.quat = self.home_quat.copy()
        self.gripper = 1.0

        # orientation bookkeeping (see module docstring)
        self.yaw = 0.0                 # [rad]
        self.tilt_dir = "dpad_up"
        self.tilt_deg = 0.0

        # tap-vs-hold tracking for Cross/Circle (session time, not wall clock)
        self._t = 0.0
        self._btn_prev: dict[str, bool] = {}
        self._press_t: dict[str, float] = {}

        # auto-move goal: (pos|None, quat|None)
        self._auto: tuple[np.ndarray | None, np.ndarray | None] | None = None
        self.quit = False

    # -- orientation composition ---------------------------------------
    def _ori_goal(self) -> np.ndarray:
        q = _rotate_quat_world(self.home_quat,
                               self.tilt_axes[self.tilt_dir] * np.deg2rad(self.tilt_deg))
        return _rotate_quat_world(q, np.array([0.0, 0.0, 1.0]) * self.yaw)

    def _tilt_axis_world(self) -> np.ndarray:
        """Tilt axis rotated by the current yaw, keeping increments consistent
        with the yaw-after-tilt composition of _ori_goal()."""
        qz = np.zeros(4)
        mujoco.mju_axisAngle2Quat(qz, np.array([0.0, 0.0, 1.0]), self.yaw)
        out = np.zeros(3)
        mujoco.mju_rotVecQuat(out, self.tilt_axes[self.tilt_dir], qz)
        return out

    # -- discrete buttons ------------------------------------------------
    def _on_button(self, name: str):
        if name == "create":            # capture key: record start / stop+save
            if self.recorder.recording:
                path = self.recorder.stop(save=True)
                print(f"[rec] saved -> {path}")
            else:
                self.recorder.start()
                print("[rec] recording started")
        elif name == "options":         # menu key: discard
            if self.recorder.recording:
                self.recorder.stop(save=False)
                print("[rec] discarded")
        elif name == "triangle":
            self.yaw = 0.0
            self.tilt_deg = 0.0
            self._auto = (self.home_pos.copy(), self.home_quat.copy())
            print("[auto] homing")
        elif name == "square":
            self.yaw = 0.0
            self.tilt_deg = 0.0
            self._auto = (None, self.home_quat.copy())
            print("[auto] orientation reset")
        elif name == "r3":
            goal = self.pos.copy()
            goal[2] = self.descend_z
            self._auto = (goal, None)
            print(f"[auto] descend to z={self.descend_z:.3f}")
        elif name in self.tilt_axes:
            if name != self.tilt_dir:
                self.tilt_dir = name
                if abs(self.tilt_deg) > 1e-9:
                    # switching direction returns to untilted orientation first
                    self.tilt_deg = 0.0
                    self._auto = (None, self._ori_goal())
                print(f"[tilt] direction -> {name}")
        elif name == "ps":
            self.quit = True

    # -- Cross/Circle tap-vs-hold tilt control ----------------------------
    def _update_tilt(self, gp: GamepadState):
        for name, sign in (("cross", 1.0), ("circle", -1.0)):
            held = gp.is_held(name)
            was = self._btn_prev.get(name, False)
            if held and not was:
                self._press_t[name] = self._t
            if held and (self._t - self._press_t.get(name, self._t)) >= self.hold_threshold:
                # hold: slow continuous tilt
                new = float(np.clip(self.tilt_deg + sign * self.tilt_hold_speed * self.dt,
                                    0.0, self.tilt_max))
                delta = new - self.tilt_deg
                if abs(delta) > 1e-9:
                    self._auto = None
                    self.quat = _rotate_quat_world(
                        self.quat, self._tilt_axis_world() * np.deg2rad(delta))
                    self.tilt_deg = new
            if was and not held:
                if (self._t - self._press_t.get(name, self._t)) < self.hold_threshold:
                    # tap: snap onto the 30-degree grid
                    if sign > 0:
                        new = min(self.tilt_max,
                                  (np.floor(self.tilt_deg / self.tilt_step + 1e-6) + 1)
                                  * self.tilt_step)
                    else:
                        new = max(0.0,
                                  (np.ceil(self.tilt_deg / self.tilt_step - 1e-6) - 1)
                                  * self.tilt_step)
                    if abs(new - self.tilt_deg) > 1e-9:
                        self.tilt_deg = float(new)
                        self._auto = (None, self._ori_goal())
                        print(f"[tilt] {self.tilt_dir} -> {self.tilt_deg:.0f} deg")
            self._btn_prev[name] = held

    # -- continuous integration -------------------------------------------
    def _slew(self, current, desired, accel):
        """Move `current` toward `desired` at bounded acceleration."""
        return current + np.clip(desired - current, -accel * self.dt, accel * self.dt)

    def _integrate_manual(self, gp: GamepadState):
        v_des = np.array([
            gp.ly * self.speed["xy"],            # stick up -> +x (away from base)
            -gp.lx * self.speed["xy"],           # stick left -> +y
            gp.ry * self.speed["z"],             # right stick up -> +z
        ])
        self._v = self._slew(self._v, v_des, self.acc_xyz)
        self.pos = np.clip(self.pos + self._v * self.dt, self.ws_lo, self.ws_hi)

        yaw_des = 0.0
        if gp.is_held("l1"):
            yaw_des += self.speed["yaw"]
        if gp.is_held("r1"):
            yaw_des -= self.speed["yaw"]
        self._yaw_v = float(self._slew(self._yaw_v, yaw_des, self.acc_yaw))
        if abs(self._yaw_v) > 1e-9:
            self.yaw += self._yaw_v * self.dt
            self.quat = _rotate_quat_world(
                self.quat, np.array([0.0, 0.0, 1.0]) * self._yaw_v * self.dt)

        g_rate = (gp.l2 - gp.r2) * self.speed["gripper"] / 0.08  # width-rate -> [0,1]-rate
        self.gripper = float(np.clip(self.gripper + g_rate * self.dt, 0.0, 1.0))

    def _integrate_auto(self):
        goal_pos, goal_quat = self._auto
        v_auto = self.speed.get("auto_xyz", self.speed["xy"])
        w_auto = self.speed.get("auto_rot", self.speed["yaw"])
        # decay any leftover manual velocity so mode switches don't kick
        self._v = self._slew(self._v, np.zeros(3), self.acc_xyz)
        self._yaw_v = float(self._slew(self._yaw_v, 0.0, self.acc_yaw))
        done = True
        if goal_pos is not None:
            err = goal_pos - self.pos
            dist = np.linalg.norm(err)
            if dist > 1e-9:
                # trapezoidal arrival: cruise, then decelerate into the goal
                v = min(v_auto, np.sqrt(2.0 * self.acc_xyz * dist))
                step = min(v * self.dt, dist)
                self.pos = np.clip(self.pos + err / dist * step, self.ws_lo, self.ws_hi)
            done &= bool(np.linalg.norm(goal_pos - self.pos) < AUTO_DONE_POS)
        if goal_quat is not None:
            vec = _quat_err_world(goal_quat, self.quat)
            ang = np.linalg.norm(vec)
            if ang > 1e-9:
                w = min(w_auto, np.sqrt(2.0 * self.acc_yaw * ang))
                if ang > w * self.dt:
                    vec *= w * self.dt / ang
                self.quat = _rotate_quat_world(self.quat, vec)
            done &= bool(np.linalg.norm(_quat_err_world(goal_quat, self.quat)) < AUTO_DONE_ROT)
        if done:
            self._auto = None
            print("[auto] done")

    # ------------------------------------------------------------------
    def tick(self, gp: GamepadState | None = None):
        gp = gp if gp is not None else self.gamepad.poll()
        self._t += self.dt
        for name in gp.pressed:
            self._on_button(name)
        self._update_tilt(gp)

        if self._auto is not None:
            manual_input = (
                max(abs(gp.lx), abs(gp.ly), abs(gp.rx), abs(gp.ry)) > AUTO_CANCEL_STICK
                or gp.is_held("l1") or gp.is_held("r1"))
            if manual_input:
                self._auto = None
                print("[auto] cancelled by manual input")
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
