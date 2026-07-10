"""Robot-type -> MuJoCo scene resolution. Single source of truth: configs/teleop.yaml `robot`."""
import pathlib

ASSETS = pathlib.Path(__file__).resolve().parents[3] / "assets"

SCENES = {
    "fr3": ASSETS / "franka_fr3" / "scene_teleop.xml",
    "panda": ASSETS / "franka_emika_panda" / "scene_teleop.xml",
}


def scene_path(cfg: dict) -> pathlib.Path:
    robot = cfg.get("robot", "fr3")
    if robot not in SCENES:
        raise ValueError(f"unknown robot type {robot!r}; expected one of {sorted(SCENES)}")
    return SCENES[robot]
