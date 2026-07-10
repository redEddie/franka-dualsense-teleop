"""Damped-least-squares differential IK on a MuJoCo model.

Used by BOTH backends: the sim steps this model as physics, the real-robot
client uses the same model kinematics-only, so the sticks feel identical.
"""
from __future__ import annotations

import mujoco
import numpy as np


class DiffIK:
    def __init__(self, model: mujoco.MjModel, site: str, q_home: np.ndarray, cfg: dict):
        self.m = model
        self.site_id = model.site(site).id
        self.q_home = np.asarray(q_home, dtype=float)
        self.pos_gain = float(cfg.get("pos_gain", 5.0))
        self.rot_gain = float(cfg.get("rot_gain", 5.0))
        self.damping = float(cfg.get("damping", 1e-4))
        self.null_gain = float(cfg.get("nullspace_gain", 0.25))
        self.max_lin_vel = float(cfg.get("max_lin_vel", 0.6))
        self.max_ang_vel = float(cfg.get("max_ang_vel", 2.0))
        self.max_qvel = float(cfg.get("max_joint_vel", 2.0))
        # anti-windup: commanded q may not lead measured q by more than this
        self.cmd_lag_limit = float(cfg.get("cmd_lag_limit", 0.15))
        # arm dof indices: first 7 velocity dofs of the panda tree
        self.dof = np.arange(7)
        jnt_range = model.jnt_range[:7].copy()
        self.q_lo, self.q_hi = jnt_range[:, 0], jnt_range[:, 1]
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))
        self._q_cmd: np.ndarray | None = None  # integrated command state

    def ee_pose(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        pos = data.site_xpos[self.site_id].copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, data.site_xmat[self.site_id])
        return pos, quat

    def reset_cmd(self, q: np.ndarray | None = None):
        self._q_cmd = None if q is None else np.asarray(q, dtype=float).copy()

    def step(
        self,
        data: mujoco.MjData,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """One IK step. Assumes mj_forward/mj_step has been run on `data`.

        Returns a 7-dof joint position command (integrated, limit-clamped).
        The command is integrated internally — NOT re-derived from measured q
        each tick — otherwise servo tracking error ratchets the pose downward.
        """
        cur_pos, cur_quat = self.ee_pose(data)

        # world-frame twist that drives current pose toward target
        err = np.zeros(6)
        err[:3] = self.pos_gain * (target_pos - cur_pos)
        quat_conj = np.zeros(4)
        quat_err = np.zeros(4)
        mujoco.mju_negQuat(quat_conj, cur_quat)
        mujoco.mju_mulQuat(quat_err, target_quat, quat_conj)
        rot_vec = np.zeros(3)
        mujoco.mju_quat2Vel(rot_vec, quat_err, 1.0)
        err[3:] = self.rot_gain * rot_vec

        # task-space velocity clamp: large target jumps become bounded,
        # constant-speed approaches instead of proportional lunges
        lin = np.linalg.norm(err[:3])
        if lin > self.max_lin_vel:
            err[:3] *= self.max_lin_vel / lin
        ang = np.linalg.norm(err[3:])
        if ang > self.max_ang_vel:
            err[3:] *= self.max_ang_vel / ang

        mujoco.mj_jacSite(self.m, data, self._jacp, self._jacr, self.site_id)
        J = np.vstack([self._jacp[:, self.dof], self._jacr[:, self.dof]])  # (6,7)

        # damped least squares
        JJt = J @ J.T + self.damping * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, err)

        # nullspace bias toward home configuration
        q = data.qpos[self.dof].copy()
        J_pinv = J.T @ np.linalg.inv(JJt)
        N = np.eye(7) - J_pinv @ J
        dq += N @ (self.null_gain * (self.q_home - q))

        np.clip(dq, -self.max_qvel, self.max_qvel, out=dq)

        if self._q_cmd is None:
            self._q_cmd = q.copy()
        q_cmd = self._q_cmd + dq * dt
        # anti-windup: don't let the command run away from the measured state
        # (e.g. when the arm is blocked by contact)
        np.clip(q_cmd, q - self.cmd_lag_limit, q + self.cmd_lag_limit, out=q_cmd)
        np.clip(q_cmd, self.q_lo, self.q_hi, out=q_cmd)
        self._q_cmd = q_cmd
        return q_cmd.copy()
