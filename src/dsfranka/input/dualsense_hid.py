"""DualSense driver via pydualsense (hidraw): full input + haptic output.

Preferred driver — gives rumble, adaptive triggers, lightbar and IMU on top
of everything the evdev driver provides. Requires libhidapi-hidraw0 and a
udev rule for /dev/hidraw* access (see README).
"""
from __future__ import annotations

import threading
import time

from pydualsense import TriggerModes, pydualsense

from .gamepad import Gamepad, GamepadState

# DSState attribute -> our button name
BUTTON_ATTRS = {
    "cross": "cross", "circle": "circle", "triangle": "triangle", "square": "square",
    "L1": "l1", "R1": "r1", "L3": "l3", "R3": "r3",
    "share": "create", "options": "options", "ps": "ps",
    "DpadUp": "dpad_up", "DpadDown": "dpad_down",
    "DpadLeft": "dpad_left", "DpadRight": "dpad_right",
}


class DualSenseHID(Gamepad):
    def __init__(self, deadzone: float = 0.08, invert_ly: bool = True, invert_ry: bool = True):
        self.ds = pydualsense()
        self.ds.init()  # raises if no controller / no permission
        self.deadzone = deadzone
        self.invert_ly = invert_ly
        self.invert_ry = invert_ry
        self._prev_held: dict[str, bool] = {}
        # output caches: skip redundant HID property writes
        self._rumble = (-1.0, -1.0)
        self._lightbar = (-1, -1, -1)
        # trigger states rendered by a dedicated ~250 Hz writer thread, so a
        # buzz square wave isn't limited by the caller's tick rate
        self._trig_lock = threading.Lock()
        self._trig_state = {"L2": (0.0, 0.0), "R2": (0.0, 0.0)}  # (force, buzz_hz)
        self._trig_applied = {"L2": -1.0, "R2": -1.0}
        self._trig_run = True
        self._trig_thread = threading.Thread(target=self._trigger_writer, daemon=True)
        self._trig_thread.start()
        self.set_lightbar(0, 0, 255)

    # --- input ----------------------------------------------------------
    def _norm_stick(self, value: int) -> float:
        # pydualsense pre-centers sticks to -128..127 (0 = neutral)
        v = value / 127.0
        if abs(v) < self.deadzone:
            return 0.0
        s = (abs(v) - self.deadzone) / (1.0 - self.deadzone)
        return max(-1.0, min(1.0, s if v > 0 else -s))

    def poll(self) -> GamepadState:
        s = self.ds.state
        held = {ours: bool(getattr(s, attr)) for attr, ours in BUTTON_ATTRS.items()}
        pressed = [n for n, h in held.items() if h and not self._prev_held.get(n)]
        self._prev_held = held
        ly = self._norm_stick(s.LY)
        ry = self._norm_stick(s.RY)
        return GamepadState(
            lx=self._norm_stick(s.LX),
            ly=-ly if self.invert_ly else ly,
            rx=self._norm_stick(s.RX),
            ry=-ry if self.invert_ry else ry,
            l2=s.L2_value / 255.0,
            r2=s.R2_value / 255.0,
            held=held,
            pressed=pressed,
            gyro=(s.gyro.Pitch, s.gyro.Yaw, s.gyro.Roll),
            accel=(s.accelerometer.X, s.accelerometer.Y, s.accelerometer.Z),
        )

    # --- haptic output ----------------------------------------------------
    def set_rumble(self, low: float, high: float) -> None:
        low = max(0.0, min(1.0, low))
        high = max(0.0, min(1.0, high))
        if (low, high) != self._rumble:
            self.ds.setLeftMotor(int(low * 255))
            self.ds.setRightMotor(int(high * 255))
            self._rumble = (low, high)

    def set_trigger(self, side: str, force: float, buzz_hz: float = 0.0) -> None:
        with self._trig_lock:
            self._trig_state[side] = (max(0.0, min(1.0, force)), max(0.0, buzz_hz))

    def _trigger_writer(self):
        """Renders trigger states to hardware. buzz_hz > 0 becomes a square
        wave (force <-> 30% force) generated here at up to ~125 Hz."""
        t0 = time.monotonic()
        while self._trig_run:
            now = time.monotonic() - t0
            with self._trig_lock:
                states = dict(self._trig_state)
            for side, (force, hz) in states.items():
                out = force
                if force > 0.0 and hz > 0.0:
                    out = force if (now * hz) % 1.0 < 0.5 else 0.3 * force
                self._apply_trigger(side, out)
            time.sleep(0.004)

    def _apply_trigger(self, side: str, force: float) -> None:
        if abs(force - self._trig_applied[side]) < 1e-3:
            return
        trig = self.ds.triggerR if side == "R2" else self.ds.triggerL
        if force <= 0.0:
            trig.setMode(TriggerModes.Off)
        else:
            trig.setMode(TriggerModes.Rigid)
            trig.setForce(1, int(force * 255))
        self._trig_applied[side] = force

    def set_lightbar(self, r: int, g: int, b: int) -> None:
        if (r, g, b) != self._lightbar:
            self.ds.light.setColorI(r, g, b)
            self._lightbar = (r, g, b)

    def close(self) -> None:
        try:
            self._trig_run = False
            self._trig_thread.join(timeout=0.5)
            self.set_rumble(0.0, 0.0)
            self._apply_trigger("L2", 0.0)
            self._apply_trigger("R2", 0.0)
            self.set_lightbar(0, 0, 255)
            self.ds.close()
        except Exception:
            pass  # device may already be gone
