"""MuJoCo backend: position-servo panda driven by differential IK."""
from __future__ import annotations

import pathlib

import mujoco
import numpy as np

from ..common.ik import DiffIK
from ..common.models import scene_path
from ..common.types import EETarget, RobotState


class MujocoArm:
    def __init__(self, cfg: dict, scene: str | pathlib.Path | None = None):
        scene = pathlib.Path(scene) if scene else scene_path(cfg)
        self.m = mujoco.MjModel.from_xml_path(str(scene))
        self.d = mujoco.MjData(self.m)
        self.q_home = np.asarray(cfg["home"]["qpos"], dtype=float)
        self.ik = DiffIK(self.m, "tcp", self.q_home, cfg["ik"])
        self.dt = 1.0 / cfg["control"]["rate_hz"]
        self.substeps = max(1, round(self.dt / self.m.opt.timestep))
        self._marker_id = self.m.body("target_marker").mocapid[0] if "target_marker" in [
            self.m.body(i).name for i in range(self.m.nbody)] else -1
        # gripper: last actuator (tendon servo, same layout for panda and fr3)
        self._grip_act = self.m.actuator("actuator8").id
        self._grip_lo, self._grip_hi = self.m.actuator_ctrlrange[self._grip_act]
        self._finger_adr = [self.m.joint(n).qposadr[0] for n in ("finger_joint1", "finger_joint2")]
        self.reset()

    def reset(self):
        key = self.m.key("home").id
        mujoco.mj_resetDataKeyframe(self.m, self.d, key)
        # honor the configured home preset, which may differ from the model's
        # "home" keyframe (franka_ready / libero); also point the position servo
        # there so it holds instead of driving back to the keyframe
        self.d.qpos[:7] = self.q_home
        if self.m.nu >= 7:
            self.d.ctrl[:7] = self.q_home
        # the robot-file keyframe doesn't cover scene objects: put free bodies
        # (cube etc.) back at their XML spawn pose instead of zero-padded origin
        for j in range(self.m.njnt):
            if self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                adr = self.m.jnt_qposadr[j]
                self.d.qpos[adr:adr + 7] = self.m.qpos0[adr:adr + 7]
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
        self.d.ctrl[self._grip_act] = self._grip_lo + target.gripper * (self._grip_hi - self._grip_lo)
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
            gripper_width=float(sum(self.d.qpos[a] for a in self._finger_adr)),
            t=float(self.d.time),
        )
