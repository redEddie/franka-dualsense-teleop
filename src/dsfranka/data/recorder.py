"""Episode recorder: buffers (state, action) pairs, saves to HDF5 or discards."""
from __future__ import annotations

import datetime
import pathlib

import h5py
import numpy as np

from ..common.types import EETarget, RobotState


class EpisodeRecorder:
    def __init__(self, out_dir: str | pathlib.Path):
        self.out_dir = pathlib.Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.recording = False
        self._buf: dict[str, list] = {}

    def start(self):
        self._buf = {k: [] for k in (
            "t", "q", "dq", "ee_pos", "ee_quat", "gripper_width",
            "action_ee_pos", "action_ee_quat", "action_gripper",
        )}
        self.recording = True

    def add(self, state: RobotState, target: EETarget):
        if not self.recording:
            return
        b = self._buf
        b["t"].append(state.t)
        b["q"].append(state.q.copy())
        b["dq"].append(state.dq.copy())
        b["ee_pos"].append(state.ee_pos.copy())
        b["ee_quat"].append(state.ee_quat.copy())
        b["gripper_width"].append(state.gripper_width)
        b["action_ee_pos"].append(target.pos.copy())
        b["action_ee_quat"].append(target.quat.copy())
        b["action_gripper"].append(target.gripper)

    def stop(self, save: bool) -> pathlib.Path | None:
        self.recording = False
        n = len(self._buf.get("t", []))
        if not save or n == 0:
            self._buf = {}
            return None
        path = self._next_path()
        with h5py.File(path, "w") as f:
            f.attrs["created"] = datetime.datetime.now().isoformat()
            f.attrs["num_steps"] = n
            obs = f.create_group("obs")
            act = f.create_group("action")
            f.create_dataset("t", data=np.asarray(self._buf["t"]))
            for k in ("q", "dq", "ee_pos", "ee_quat", "gripper_width"):
                obs.create_dataset(k, data=np.asarray(self._buf[k]))
            act.create_dataset("ee_pos", data=np.asarray(self._buf["action_ee_pos"]))
            act.create_dataset("ee_quat", data=np.asarray(self._buf["action_ee_quat"]))
            act.create_dataset("gripper", data=np.asarray(self._buf["action_gripper"]))
        self._buf = {}
        return path

    def _next_path(self) -> pathlib.Path:
        existing = sorted(self.out_dir.glob("episode_*.hdf5"))
        idx = 0
        if existing:
            idx = int(existing[-1].stem.split("_")[1]) + 1
        return self.out_dir / f"episode_{idx:04d}.hdf5"
