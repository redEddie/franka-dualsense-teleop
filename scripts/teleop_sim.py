#!/usr/bin/env python3
"""DualSense teleop of the Panda in MuJoCo.

    python scripts/teleop_sim.py                 # DualSense + viewer
    python scripts/teleop_sim.py --mock --headless --ticks 200   # smoke test
"""
import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dsfranka.common.config import load_config
from dsfranka.data.recorder import EpisodeRecorder
from dsfranka.input.gamepad import MockGamepad
from dsfranka.sim.mujoco_robot import MujocoArm
from dsfranka.teleop.session import TeleopSession


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs/teleop.yaml"))
    ap.add_argument("--mock", action="store_true", help="run without a controller")
    ap.add_argument("--headless", action="store_true", help="no viewer")
    ap.add_argument("--ticks", type=int, default=None, help="stop after N ticks")
    args = ap.parse_args()

    cfg = load_config(args.config)
    arm = MujocoArm(cfg)

    if args.mock:
        pad = MockGamepad()
    else:
        from dsfranka.input.factory import make_gamepad
        pad = make_gamepad(cfg)

    rec = EpisodeRecorder(ROOT / cfg["recorder"]["out_dir"])
    session = TeleopSession(cfg, arm, pad, rec)

    if args.headless:
        session.run(max_ticks=args.ticks)
    else:
        import mujoco
        import mujoco.viewer
        with mujoco.viewer.launch_passive(arm.m, arm.d) as viewer:
            # launch_passive starts with a bare MjvCamera and ignores the
            # model's visual/global azimuth/elevation — apply them explicitly
            mujoco.mjv_defaultFreeCamera(arm.m, viewer.cam)
            viewer.sync()

            def on_tick(state, target):
                if not viewer.is_running():
                    return False
                viewer.sync()
                return True
            session.run(max_ticks=args.ticks, on_tick=on_tick)

    print("bye")


if __name__ == "__main__":
    main()
