"""Normalized gamepad state, independent of the physical driver."""
from __future__ import annotations

from dataclasses import dataclass, field


BUTTONS = (
    "cross", "circle", "triangle", "square",
    "l1", "r1", "l3", "r3",
    "create", "options", "ps",
    "dpad_up", "dpad_down", "dpad_left", "dpad_right",
)


@dataclass
class GamepadState:
    lx: float = 0.0   # [-1, 1] right positive
    ly: float = 0.0   # [-1, 1] up positive (driver applies inversion)
    rx: float = 0.0
    ry: float = 0.0
    l2: float = 0.0   # [0, 1]
    r2: float = 0.0   # [0, 1]
    held: dict = field(default_factory=dict)      # button -> bool
    pressed: list = field(default_factory=list)   # rising edges since last poll

    def is_held(self, name: str) -> bool:
        return bool(self.held.get(name, False))


class Gamepad:
    """Driver interface: poll() returns the latest state with edge events."""

    def poll(self) -> GamepadState:
        raise NotImplementedError

    def close(self) -> None:
        pass


class MockGamepad(Gamepad):
    """Scriptable pad for tests/headless smoke runs."""

    def __init__(self, script=None):
        # script: iterable of GamepadState, replayed once then zeros
        self._script = iter(script) if script is not None else iter(())

    def poll(self) -> GamepadState:
        return next(self._script, GamepadState())
