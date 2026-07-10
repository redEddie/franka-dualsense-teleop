#!/usr/bin/env python3
"""Headless integration test: scripted gamepad drives the full sim pipeline.

    python tests/test_pipeline.py
"""
import pathlib
import sys
import tempfile

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dsfranka.data.recorder import EpisodeRecorder
from dsfranka.input.gamepad import GamepadState, MockGamepad
from dsfranka.sim.mujoco_robot import MujocoArm
from dsfranka.teleop.session import TeleopSession


def run_ticks(session, states):
    for gp in states:
        state, target = session.tick(gp)
    return state, target


def main():
    cfg = yaml.safe_load(open(ROOT / "configs/teleop.yaml"))
    tmp = tempfile.mkdtemp()
    arm = MujocoArm(cfg)
    rec = EpisodeRecorder(tmp)
    s = TeleopSession(cfg, arm, MockGamepad(), rec)
    hz = cfg["control"]["rate_hz"]

    # --- settle at home, check IK tracks the initial target -------------
    st, tg = run_ticks(s, [GamepadState() for _ in range(hz)])
    err0 = np.linalg.norm(st.ee_pos - tg.pos)
    assert err0 < 0.01, f"home tracking error {err0:.4f} m"
    print(f"[ok] home tracking err={err0*1000:.1f} mm")

    # --- record while translating with left stick ------------------------
    press_rec = GamepadState(pressed=["options"])
    st, _ = run_ticks(s, [press_rec] + [GamepadState(ly=1.0) for _ in range(hz)])
    moved_x = st.ee_pos[0] - s.home_pos[0]
    assert moved_x > 0.1, f"stick +x motion too small: {moved_x:.3f} m"
    print(f"[ok] left stick moved EE +{moved_x:.3f} m in x while recording")

    # gripper close with R2
    st, tg = run_ticks(s, [GamepadState(r2=1.0) for _ in range(2 * hz)])
    assert st.gripper_width < 0.02, f"gripper did not close: {st.gripper_width:.3f}"
    print(f"[ok] R2 closed gripper to {st.gripper_width*1000:.1f} mm")

    # stop + save
    st, _ = run_ticks(s, [GamepadState(pressed=["options"])])
    files = list(pathlib.Path(tmp).glob("episode_*.hdf5"))
    assert len(files) == 1, f"expected 1 episode file, got {files}"
    import h5py
    with h5py.File(files[0]) as f:
        n = f.attrs["num_steps"]
        assert f["obs/q"].shape == (n, 7) and f["action/ee_pos"].shape == (n, 3)
    print(f"[ok] episode saved: {files[0].name} ({n} steps)")

    # --- discard path ----------------------------------------------------
    run_ticks(s, [GamepadState(pressed=["options"])])   # start rec
    run_ticks(s, [GamepadState(lx=1.0) for _ in range(10)])
    run_ticks(s, [GamepadState(pressed=["create"])])    # discard
    assert len(list(pathlib.Path(tmp).glob("episode_*.hdf5"))) == 1
    print("[ok] discard leaves no file")

    # --- auto descend (Cross) ---------------------------------------------
    st, _ = run_ticks(s, [GamepadState(pressed=["cross"])] +
                         [GamepadState() for _ in range(4 * hz)])
    assert abs(st.ee_pos[2] - cfg["features"]["descend_z"]) < 0.02, \
        f"descend failed, z={st.ee_pos[2]:.3f}"
    print(f"[ok] auto-descend reached z={st.ee_pos[2]:.3f}")

    # --- tilt cycle (Circle -> 30 deg) ------------------------------------
    st, tg = run_ticks(s, [GamepadState(pressed=["circle"])] +
                          [GamepadState() for _ in range(3 * hz)])
    import mujoco
    conj, dq = np.zeros(4), np.zeros(4)
    mujoco.mju_negQuat(conj, s.home_quat)
    mujoco.mju_mulQuat(dq, tg.quat, conj)
    ang = 2 * np.arccos(min(1.0, abs(dq[0])))
    assert abs(np.rad2deg(ang) - 30) < 3, f"tilt angle {np.rad2deg(ang):.1f} != 30"
    print(f"[ok] tilt target = {np.rad2deg(ang):.1f} deg")

    # --- homing (Triangle) --------------------------------------------------
    st, _ = run_ticks(s, [GamepadState(pressed=["triangle"])] +
                         [GamepadState() for _ in range(6 * hz)])
    err = np.linalg.norm(st.ee_pos - s.home_pos)
    assert err < 0.02, f"homing error {err:.3f} m"
    print(f"[ok] homing err={err*1000:.1f} mm")

    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
