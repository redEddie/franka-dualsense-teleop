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

import mujoco

from dsfranka.data.recorder import EpisodeRecorder
from dsfranka.input.gamepad import GamepadState, MockGamepad
from dsfranka.sim.mujoco_robot import MujocoArm
from dsfranka.teleop.session import TeleopSession


def run_ticks(session, states):
    for gp in states:
        state, target = session.tick(gp)
    return state, target


def hold(name, n):
    return [GamepadState(held={name: True}) for _ in range(n)]


def tap(name):
    """Short press: 3 ticks held (60 ms < hold threshold), then release."""
    return hold(name, 3) + [GamepadState()]


def tilt_of(session, quat):
    """Angle [deg] between quat and the session home orientation."""
    conj, dq = np.zeros(4), np.zeros(4)
    mujoco.mju_negQuat(conj, session.home_quat)
    mujoco.mju_mulQuat(dq, quat, conj)
    return np.rad2deg(2 * np.arccos(min(1.0, abs(dq[0]))))


def main():
    cfg = yaml.safe_load(open(ROOT / "configs/teleop.yaml"))
    tmp = tempfile.mkdtemp()
    arm = MujocoArm(cfg)
    rec = EpisodeRecorder(tmp)
    s = TeleopSession(cfg, arm, MockGamepad(), rec)
    hz = cfg["control"]["rate_hz"]
    idle = lambda n: [GamepadState() for _ in range(n)]

    # --- settle at home, check IK tracks the initial target -------------
    st, tg = run_ticks(s, idle(hz))
    err0 = np.linalg.norm(st.ee_pos - tg.pos)
    assert err0 < 0.01, f"home tracking error {err0:.4f} m"
    print(f"[ok] home tracking err={err0*1000:.1f} mm")

    # --- record (Create) while translating with left stick ---------------
    st, _ = run_ticks(s, [GamepadState(pressed=["create"])] +
                         [GamepadState(ly=1.0) for _ in range(hz)])
    # home x=0.554, workspace x max 0.70 -> at most ~0.145 m of travel
    moved_x = st.ee_pos[0] - s.home_pos[0]
    assert moved_x > 0.12, f"stick +x motion too small: {moved_x:.3f} m"
    print(f"[ok] left stick moved EE +{moved_x:.3f} m in x while recording")

    # right stick down -> z decreases (0.2 m/s with accel ramp + EE lag);
    # let the EE settle from the previous segment before measuring
    st, _ = run_ticks(s, idle(hz))
    z0 = st.ee_pos[2]
    st, _ = run_ticks(s, [GamepadState(ry=-1.0) for _ in range(hz)])
    assert st.ee_pos[2] < z0 - 0.08, f"right stick z motion failed: {st.ee_pos[2]:.3f} (z0={z0:.3f})"
    print(f"[ok] right stick lowered EE by {z0 - st.ee_pos[2]:.3f} m")

    # gripper close with R2
    st, tg = run_ticks(s, [GamepadState(r2=1.0) for _ in range(hz)])
    assert st.gripper_width < 0.02, f"gripper did not close: {st.gripper_width:.3f}"
    print(f"[ok] R2 closed gripper to {st.gripper_width*1000:.1f} mm")

    # stop + save (Create again)
    st, _ = run_ticks(s, [GamepadState(pressed=["create"])])
    files = list(pathlib.Path(tmp).glob("episode_*.hdf5"))
    assert len(files) == 1, f"expected 1 episode file, got {files}"
    import h5py
    with h5py.File(files[0]) as f:
        n = f.attrs["num_steps"]
        assert f["obs/q"].shape == (n, 7) and f["action/ee_pos"].shape == (n, 3)
    print(f"[ok] episode saved: {files[0].name} ({n} steps)")

    # --- discard path (Options) -------------------------------------------
    run_ticks(s, [GamepadState(pressed=["create"])])    # start rec
    run_ticks(s, [GamepadState(lx=1.0) for _ in range(10)])
    run_ticks(s, [GamepadState(pressed=["options"])])   # discard
    assert len(list(pathlib.Path(tmp).glob("episode_*.hdf5"))) == 1
    print("[ok] Options discards, leaves no file")

    # --- auto descend (R3) --------------------------------------------------
    st, _ = run_ticks(s, [GamepadState(pressed=["r3"])] + idle(3 * hz))
    assert abs(st.ee_pos[2] - cfg["features"]["descend_z"]) < 0.02, \
        f"descend failed, z={st.ee_pos[2]:.3f}"
    print(f"[ok] R3 auto-descend reached z={st.ee_pos[2]:.3f}")

    # --- homing (Triangle) ---------------------------------------------------
    st, _ = run_ticks(s, [GamepadState(pressed=["triangle"])] + idle(4 * hz))
    assert np.linalg.norm(st.ee_pos - s.home_pos) < 0.02
    print("[ok] homing")

    # --- tilt: tap Cross -> 30, tap again -> 60 (ud component) ----------------
    st, tg = run_ticks(s, tap("cross") + idle(2 * hz))
    assert abs(tilt_of(s, tg.quat) - 30) < 2, f"tilt {tilt_of(s, tg.quat):.1f} != 30"
    st, tg = run_ticks(s, tap("cross") + idle(2 * hz))
    assert abs(tilt_of(s, tg.quat) - 60) < 2, f"tilt {tilt_of(s, tg.quat):.1f} != 60"
    assert abs(abs(s.tilt["ud"]) - 60) < 1e-6
    print("[ok] Cross taps: 0 -> 30 -> 60 deg")

    # --- hold Cross: continuous creep past the grid ---------------------------
    st, tg = run_ticks(s, hold("cross", int(1.0 * hz)) + [GamepadState()])
    creep = tilt_of(s, tg.quat)
    # 1.0 s hold - 0.35 s threshold = 0.65 s at 40 deg/s ~= 26 deg on top of 60
    assert 70 < creep < 90, f"hold creep angle {creep:.1f} not in (70, 90)"
    assert abs(abs(s.tilt["ud"]) - creep) < 2, f"bookkeeping {s.tilt['ud']:.1f} vs {creep:.1f}"
    print(f"[ok] Cross hold creep -> {creep:.1f} deg (ambiguous, off-grid)")

    # --- continuity: opposite d-pad direction edits the SAME value ------------
    st, tg = run_ticks(s, [GamepadState(pressed=["dpad_down"])] + idle(hz))
    assert abs(tilt_of(s, tg.quat) - creep) < 2, "d-pad selection must not move the robot"
    st, tg = run_ticks(s, tap("cross") + idle(2 * hz))
    assert abs(tilt_of(s, tg.quat) - 60) < 2, \
        f"opposite-direction step from {creep:.1f} should snap to 60, got {tilt_of(s, tg.quat):.1f}"
    print(f"[ok] dpad_down keeps value; Cross steps {creep:.1f} -> 60 (continuous)")

    # --- Circle tap: toward 0 on the active component ---------------------------
    st, tg = run_ticks(s, tap("circle") + idle(2 * hz))
    assert abs(tilt_of(s, tg.quat) - 30) < 2, f"cancel step {tilt_of(s, tg.quat):.1f} != 30"
    print("[ok] Circle tap 60 -> 30 (toward zero)")

    # --- combined tilt: add lr on top of ud ------------------------------------
    st, tg = run_ticks(s, [GamepadState(pressed=["dpad_left"])] + tap("cross") + idle(2 * hz))
    assert abs(abs(s.tilt["lr"]) - 30) < 1e-6 and abs(abs(s.tilt["ud"]) - 30) < 1e-6
    assert tilt_of(s, tg.quat) > 35, f"combined tilt too small: {tilt_of(s, tg.quat):.1f}"
    print(f"[ok] combined tilt ud=30 + lr=30 -> total {tilt_of(s, tg.quat):.1f} deg")

    # --- max cap on the ud component -------------------------------------------
    st, tg = run_ticks(s, [GamepadState(pressed=["dpad_up"])] +
                          tap("cross") + tap("cross") + tap("cross") + idle(2 * hz))
    assert abs(abs(s.tilt["ud"]) - 90) < 1e-6, f"ud should cap at 90, got {s.tilt['ud']}"
    print("[ok] ud capped at 90 deg")

    # --- Square: full orientation reset ----------------------------------------
    st, tg = run_ticks(s, [GamepadState(pressed=["square"])] + idle(2 * hz))
    assert tilt_of(s, tg.quat) < 2 and s.tilt == {"ud": 0.0, "lr": 0.0} and s.yaw == 0.0
    print("[ok] Square resets orientation")

    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
