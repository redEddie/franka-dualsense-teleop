"""Config loading with workspace-calibration overlay.

If configs/workspace_calibrated.yaml exists (produced by
scripts/calibrate_workspace.py from a kinesthetic sweep), its workspace box
overrides the one in teleop.yaml — teleop.yaml itself is never rewritten.
"""
import pathlib

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = ROOT / "configs" / "teleop.yaml"
CALIBRATED_WS = ROOT / "configs" / "workspace_calibrated.yaml"


def _resolve_home(cfg: dict) -> None:
    """Pick the active home qpos from home.presets[home.select].

    Backends read cfg["home"]["qpos"]; this fills it in from the selected preset
    so multiple homing poses can be saved and switched from one place. A literal
    home.qpos (legacy) is left as-is when no presets/select are given."""
    home = cfg.get("home")
    if not isinstance(home, dict):
        return
    presets, select = home.get("presets"), home.get("select")
    if presets and select is not None:
        if select not in presets:
            raise ValueError(
                f"home.select '{select}' not in home.presets {list(presets)}")
        home["qpos"] = list(presets[select])
        print(f"[config] home preset '{select}': {home['qpos']}")


def load_config(path: str | pathlib.Path | None = None) -> dict:
    cfg = yaml.safe_load(open(path or DEFAULT_CONFIG))
    _resolve_home(cfg)
    if CALIBRATED_WS.exists():
        overlay = yaml.safe_load(open(CALIBRATED_WS))
        if overlay and "workspace" in overlay:
            cfg["workspace"] = overlay["workspace"]
            print(f"[config] workspace overridden by {CALIBRATED_WS.name}: {cfg['workspace']}")
    return cfg
