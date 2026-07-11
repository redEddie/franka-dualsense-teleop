"""Gamepad driver selection from config."""
from __future__ import annotations

from .gamepad import Gamepad


def make_gamepad(cfg: dict) -> Gamepad:
    g = cfg.get("gamepad", {})
    driver = g.get("driver", "pydualsense")
    kwargs = dict(deadzone=g.get("deadzone", 0.08),
                  invert_ly=g.get("invert_ly", True),
                  invert_ry=g.get("invert_ry", True))
    if driver == "pydualsense":
        try:
            from .dualsense_hid import DualSenseHID
            pad = DualSenseHID(**kwargs)
            print("[input] pydualsense driver (haptics enabled)")
            return pad
        except Exception as e:  # no lib / no permission / no device
            print(f"[input] pydualsense unavailable ({e}); falling back to evdev")
    from .dualsense_evdev import DualSenseEvdev
    pad = DualSenseEvdev(**kwargs)
    print("[input] evdev driver (no haptics)")
    return pad
