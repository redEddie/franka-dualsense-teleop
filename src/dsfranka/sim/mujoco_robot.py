"""MuJoCo backend: position-servo panda driven by differential IK."""
from __future__ import annotations

import pathlib

import mujoco
import numpy as np

from ..common.ik import DiffIK
from ..common.types import EETarget, RobotState

ASSETS = pathlib.Path(__file__).resolve().parents[3] / "assets"


class MujocoArm:
    def __init__(self, cfg: dict, scene: str | pathlib.Path | None = None):
        scene = pathlib.Path(scene) if scene else ASSETS / "franka_emika_panda" / "scene_teleop.xml"
        self.m = mujoco.MjModel.from_xml_path(str(scene))
        self.d = mujoco.MjData(self.m)
        self.q_home = np.asarray(cfg["home"]["qpos"], dtype=float)
        self.ik = DiffIK(self.m, "tcp", self.q_home, cfg["ik"])
        self.dt = 1.0 / cfg["control"]["rate_hz"]
        self.substeps = max(1, round(self.dt / self.m.opt.timestep))
        self._marker_id = self.m.body("target_marker").mocapid[0] if "target_marker" in [
            self.m.body(i).name for i in range(self.m.nbody)] else -1
        self.reset()

    def reset(self):
        key = self.m.key("home").id
        mujoco.mj_resetDataKeyframe(self.m, self.d, key)
        mujoco.mj_forward(self.m, self.d)
        self.ik.reset_cmd(self.d.qpos[:7])

    def home_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """FK of the home configuration on a scratch MjData."""
        d = mujoco.MjData(self.m)
        d.qpos[:7] = self.q_home
        mujoco.mj_forward(self.m, d)
        return self.ik.ee_pose(d)

    def apply(self, target: EETarget, dt: float) -> RobotState:
        q_cmd = self.ik.step(self.d, target.pos, target.quat, dt)
        target.q_ref = q_cmd
        self.d.ctrl[:7] = q_cmd
        self.d.ctrl[7] = target.gripper * 255.0
        if self._marker_id >= 0:
            self.d.mocap_pos[self._marker_id] = target.pos
            self.d.mocap_quat[self._marker_id] = target.quat
        for _ in range(self.substeps):
            mujoco.mj_step(self.m, self.d)
        return self.state()

    def state(self) -> RobotState:
        ee_pos, ee_quat = self.ik.ee_pose(self.d)
        return RobotState(
            q=self.d.qpos[:7].copy(),
            dq=self.d.qvel[:7].copy(),
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            gripper_width=float(self.d.qpos[7] + self.d.qpos[8]),
            t=float(self.d.time),
        )
