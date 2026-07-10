"""Real-robot backend: same DiffIK on a kinematics-only MuJoCo model,
joint targets streamed over UDP to the C++ libfranka bridge.

Wire format must stay in sync with cpp/bridge/udp_protocol.hpp.

NOTE: untested until the bridge runs against a real FCI connection.
"""
from __future__ import annotations

import pathlib
import socket
import struct
import time

import mujoco
import numpy as np

from ..common.ik import DiffIK
from ..common.types import EETarget, RobotState

ASSETS = pathlib.Path(__file__).resolve().parents[3] / "assets"

MAGIC = 0x44534652  # "DSFR"
# command: magic u32, seq u32, q[7] f64, gripper f64, flags u8  (little-endian, packed)
CMD_FMT = "<II8dB"
# state: magic u32, seq u32, q[7] dq[7] f64, ee_pos[3] ee_quat[4] f64, width f64, mode u8
STATE_FMT = "<II22dB"
FLAG_NONE = 0


class FrankaArm:
    def __init__(self, cfg: dict, bridge_host: str = "127.0.0.1",
                 cmd_port: int = 5555, state_port: int = 5556):
        # kinematics-only shadow model (no physics stepping)
        self.m = mujoco.MjModel.from_xml_path(str(ASSETS / "franka_emika_panda" / "scene_teleop.xml"))
        self.d = mujoco.MjData(self.m)
        self.q_home = np.asarray(cfg["home"]["qpos"], dtype=float)
        self.ik = DiffIK(self.m, "tcp", self.q_home, cfg["ik"])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", state_port))
        self.sock.settimeout(0.1)
        self.bridge = (bridge_host, cmd_port)
        self.seq = 0
        self._last_state: RobotState | None = None
        self._sync_from_robot()

    def _sync_from_robot(self):
        """Initialize the shadow model from the robot's actual configuration."""
        st = self._recv_state(timeout=2.0)
        if st is None:
            raise RuntimeError("No state packets from franka_bridge — is it running?")
        self.d.qpos[:7] = st.q
        mujoco.mj_forward(self.m, self.d)
        self.ik.reset_cmd(st.q)

    def home_pose(self) -> tuple[np.ndarray, np.ndarray]:
        d = mujoco.MjData(self.m)
        d.qpos[:7] = self.q_home
        mujoco.mj_forward(self.m, d)
        return self.ik.ee_pose(d)

    def apply(self, target: EETarget, dt: float) -> RobotState:
        # IK on the shadow model, integrate its configuration
        q_cmd = self.ik.step(self.d, target.pos, target.quat, dt)
        target.q_ref = q_cmd
        self.d.qpos[:7] = q_cmd
        mujoco.mj_forward(self.m, self.d)

        self.seq += 1
        pkt = struct.pack(CMD_FMT, MAGIC, self.seq, *q_cmd, target.gripper, FLAG_NONE)
        self.sock.sendto(pkt, self.bridge)

        st = self._recv_state()
        if st is not None:
            self._last_state = st
        if self._last_state is None:
            raise RuntimeError("Lost connection to franka_bridge")
        return self._last_state

    def _recv_state(self, timeout: float | None = None) -> RobotState | None:
        if timeout is not None:
            self.sock.settimeout(timeout)
        try:
            data, _ = self.sock.recvfrom(4096)
        except socket.timeout:
            return None
        finally:
            if timeout is not None:
                self.sock.settimeout(0.1)
        if len(data) != struct.calcsize(STATE_FMT):
            return None
        vals = struct.unpack(STATE_FMT, data)
        if vals[0] != MAGIC:
            return None
        v = np.asarray(vals[2:24], dtype=float)
        return RobotState(
            q=v[0:7], dq=v[7:14], ee_pos=v[14:17], ee_quat=v[17:21],
            gripper_width=float(v[21]), t=time.time(),
        )
