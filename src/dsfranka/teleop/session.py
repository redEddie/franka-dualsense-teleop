"""Teleop session: button mapping + target-pose state machine.

Backend-agnostic — drives anything exposing the ArmBackend protocol
(MujocoArm for sim, FrankaArm for the real robot).

DualSense mapping
-----------------
left stick      x-y translation (base frame)
right stick     z height, continuous (up/down)
L1 / R1         yaw + / -
d-pad           select which tilt component to edit, and in which direction
                (up/down -> ud component, left/right -> lr component);
                selection alone never moves the robot and never resets state
Cross           tap: step the selected component 30 deg in the selected
                direction FROM ITS CURRENT VALUE (snaps onto the grid);
                hold: slow continuous change in that direction
Circle          tap: step the selected component 30 deg toward 0 (snap);
                hold: slow continuous return to 0
Square          orientation reset (yaw & both tilt components -> 0)
Triangle        home (EE pose of the home configuration)
R3              auto-descend to configured height
L2 / R2         gripper open / close (analog rate)
Create          episode recording start / stop+save   (capture key)
Options         discard current recording             (menu key)
PS              quit session

Orientation model:
    quat = yaw(world z) o rot(lr_axis, lr_deg) o rot(ud_axis, ud_deg) o home
The two tilt components are signed scalars in [-max, max], so d-pad direction
changes are continuous (e.g. tilted +60 forward, select down, tap Cross ->
+30) and combined tilts (forward + sideways) are possible.
"""
from __future__ import annotations

import mujoco
import numpy as np

from ..common.rate import Rate
from ..common.types import EETarget
from ..data.recorder import EpisodeRecorder
from ..input.gamepad import Gamepad, GamepadState
from .feedback import FeedbackController

AUTO_CANCEL_STICK = 0.5   # stick deflection that cancels an auto-move
AUTO_DONE_POS = 2e-3      # [m]
AUTO_DONE_ROT = 1e-2      # [rad]

DEFAULT_TILT_AXES = {     # world-frame rotation axis per tilt component
    "ud": [0.0, -1.0, 0.0],   # up/down: + tips toward +x (before inversion)
    "lr": [1.0, 0.0, 0.0],    # left/right: + tips toward +y (before inversion)
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
        up_sign = -1.0 if tilt.get("invert_ud", False) else 1.0
        left_sign = -1.0 if tilt.get("invert_lr", False) else 1.0
        self.dpad_map = {          # d-pad button -> (component, step direction)
            "dpad_up": ("ud", up_sign), "dpad_down": ("ud", -up_sign),
            "dpad_left": ("lr", left_sign), "dpad_right": ("lr", -left_sign),
        }

        # target state, initialized from the arm's home pose
        self.home_pos, self.home_quat = arm.home_pose()
        self.pos = self.home_pos.copy()
        self.quat = self.home_quat.copy()
        self.gripper = 1.0

        # orientation bookkeeping (see module docstring)
        self.yaw = 0.0                       # [rad]
        self.tilt = {"ud": 0.0, "lr": 0.0}   # signed [deg]
        self.active_tilt = self.dpad_map["dpad_up"]

        # tap-vs-hold tracking for Cross/Circle (session time, not wall clock)
        self._t = 0.0
        self._btn_prev: dict[str, bool] = {}
        self._press_t: dict[str, float] = {}

        # auto-move goal: (pos|None, quat|None)
        self._auto: tuple[np.ndarray | None, np.ndarray | None] | None = None
        self.quit = False

        self.feedback = FeedbackController(cfg)

    # -- orientation composition ---------------------------------------
    def _ori_goal(self) -> np.ndarray:
        q = _rotate_quat_world(self.home_quat,
                               self.tilt_axes["ud"] * np.deg2rad(self.tilt["ud"]))
        q = _rotate_quat_world(q, self.tilt_axes["lr"] * np.deg2rad(self.tilt["lr"]))
        return _rotate_quat_world(q, np.array([0.0, 0.0, 1.0]) * self.yaw)

    def _snap(self, val: float, direction: float) -> float:
        """Next 30-deg grid value from `val` in `direction` (+1/-1), clamped."""
        if direction > 0:
            new = (np.floor(val / self.tilt_step + 1e-6) + 1) * self.tilt_step
        else:
            new = (np.ceil(val / self.tilt_step - 1e-6) - 1) * self.tilt_step
        return float(np.clip(new, -self.tilt_max, self.tilt_max))

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
            self.tilt = {"ud": 0.0, "lr": 0.0}
            self._auto = (self.home_pos.copy(), self.home_quat.copy())
            print("[auto] homing")
        elif name == "square":
            self.yaw = 0.0
            self.tilt = {"ud": 0.0, "lr": 0.0}
            self._auto = (None, self.home_quat.copy())
            print("[auto] orientation reset")
        elif name == "r3":
            goal = self.pos.copy()
            goal[2] = self.descend_z
            self._auto = (goal, None)
            print(f"[auto] descend to z={self.descend_z:.3f}")
        elif name in self.dpad_map:
            # selection only — nothing moves, current tilt values are kept
            self.active_tilt = self.dpad_map[name]
            comp, sgn = self.active_tilt
            print(f"[tilt] editing {comp} ({'+' if sgn > 0 else '-'}), "
                  f"now ud={self.tilt['ud']:.0f} lr={self.tilt['lr']:.0f} deg")
        elif name == "ps":
            self.quit = True

    # -- Cross/Circle tap-vs-hold tilt control ----------------------------
    def _set_tilt(self, comp: str, new: float, smooth: bool):
        """Update one tilt component and drive the orientation toward it.

        smooth=True (taps) animates via auto-move; smooth=False (hold creep)
        writes the target directly — unless the current orientation is far
        from the bookkeeping (e.g. a cancelled auto-move), in which case it
        falls back to auto-move to avoid a jump.
        """
        self.tilt[comp] = new
        goal = self._ori_goal()
        if not smooth and np.linalg.norm(_quat_err_world(goal, self.quat)) < 0.15:
            self._auto = None
            self.quat = goal
        else:
            self._auto = (None, goal)

    def _update_tilt(self, gp: GamepadState):
        comp, sgn = self.active_tilt
        for name in ("cross", "circle"):
            held = gp.is_held(name)
            was = self._btn_prev.get(name, False)
            if held and not was:
                self._press_t[name] = self._t
            cur = self.tilt[comp]
            # step direction: Cross follows the d-pad selection, Circle goes toward 0
            direction = sgn if name == "cross" else (-np.sign(cur) if cur else 0.0)
            if held and (self._t - self._press_t.get(name, self._t)) >= self.hold_threshold:
                # hold: slow continuous change from the current value
                step = direction * self.tilt_hold_speed * self.dt
                if name == "circle":
                    step = -np.sign(cur) * min(abs(cur), self.tilt_hold_speed * self.dt)
                new = float(np.clip(cur + step, -self.tilt_max, self.tilt_max))
                if abs(new - cur) > 1e-9:
                    self._set_tilt(comp, new, smooth=False)
            if was and not held:
                if (self._t - self._press_t.get(name, self._t)) < self.hold_threshold \
                        and direction != 0.0:
                    new = self._snap(cur, direction)
                    if name == "circle":
                        # never overshoot past 0 when cancelling
                        new = 0.0 if new * cur < 0 else new
                    if abs(new - cur) > 1e-9:
                        self._set_tilt(comp, new, smooth=True)
                        print(f"[tilt] {comp} {cur:.0f} -> {new:.0f} deg "
                              f"(ud={self.tilt['ud']:.0f} lr={self.tilt['lr']:.0f})")
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
        self.feedback.update(self.gamepad, self.pos, state, target,
                             recording=self.recorder.recording)
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
