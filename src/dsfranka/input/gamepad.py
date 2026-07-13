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
    gyro: tuple = (0.0, 0.0, 0.0)                 # raw IMU (driver-dependent units)
    accel: tuple = (0.0, 0.0, 0.0)

    def is_held(self, name: str) -> bool:
        return bool(self.held.get(name, False))


class Gamepad:
    """Driver interface: poll() returns the latest state with edge events.

    Feedback methods are no-ops by default — only haptics-capable drivers
    (DualSenseHID) override them, so callers can invoke them unconditionally.
    """

    def poll(self) -> GamepadState:
        raise NotImplementedError

    # --- haptic feedback (optional capability) -------------------------
    def set_rumble(self, low: float, high: float) -> None:
        """low/high frequency motor intensity, 0..1 each."""

    def set_trigger(self, side: str, force: float, buzz_hz: float = 0.0) -> None:
        """Adaptive trigger resistance: side 'L2'|'R2', force 0..1 (0 = off).
        buzz_hz > 0 renders the force as a square wave at that frequency
        (driver-side oscillator) instead of steady resistance."""

    def set_lightbar(self, r: int, g: int, b: int) -> None:
        """Lightbar RGB, 0..255 each."""

    def close(self) -> None:
        pass


class MockGamepad(Gamepad):
    """Scriptable pad for tests/headless smoke runs. Records feedback calls."""

    def __init__(self, script=None):
        # script: iterable of GamepadState, replayed once then zeros
        self._script = iter(script) if script is not None else iter(())
        self.rumble = (0.0, 0.0)
        self.trigger = {"L2": (0.0, 0.0), "R2": (0.0, 0.0)}  # (force, buzz_hz)
        self.lightbar = (0, 0, 0)

    def poll(self) -> GamepadState:
        return next(self._script, GamepadState())

    def set_rumble(self, low: float, high: float) -> None:
        self.rumble = (low, high)

    def set_trigger(self, side: str, force: float, buzz_hz: float = 0.0) -> None:
        self.trigger[side] = (force, buzz_hz)

    def set_lightbar(self, r: int, g: int, b: int) -> None:
        self.lightbar = (r, g, b)
