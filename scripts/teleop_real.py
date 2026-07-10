#!/usr/bin/env python3
"""DualSense teleop of the real Franka via the C++ UDP bridge.

Prerequisites:
    1. cpp/build/franka_bridge <robot-ip> is running (robot in FCI mode, user stop released)
    2. DualSense connected

    python scripts/teleop_real.py --bridge-host 127.0.0.1
"""
import argparse
import pathlib
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dsfranka.data.recorder import EpisodeRecorder
from dsfranka.input.dualsense_evdev import DualSenseEvdev
from dsfranka.real.franka_client import FrankaArm
from dsfranka.teleop.session import TeleopSession


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "configs/teleop.yaml"))
    ap.add_argument("--bridge-host", default="127.0.0.1")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    arm = FrankaArm(cfg, bridge_host=args.bridge_host)
    g = cfg["gamepad"]
    pad = DualSenseEvdev(deadzone=g["deadzone"],
                         invert_ly=g["invert_ly"], invert_ry=g["invert_ry"])
    rec = EpisodeRecorder(ROOT / cfg["recorder"]["out_dir"])

    print("Teleop started. PS button quits.")
    TeleopSession(cfg, arm, pad, rec).run()


if __name__ == "__main__":
    main()
