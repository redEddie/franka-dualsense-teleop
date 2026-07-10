"""DualSense driver via the Linux kernel hid-playstation driver (evdev).

Works over USB and Bluetooth with no extra userspace deps. A background
thread drains kernel events; poll() returns the latest normalized state
plus rising-edge button events accumulated since the previous poll.
"""
from __future__ import annotations

import threading

import evdev
from evdev import ecodes

from .gamepad import Gamepad, GamepadState

DEVICE_NAME_HINTS = ("dualsense", "wireless controller", "sony")

BUTTON_MAP = {
    ecodes.BTN_SOUTH: "cross",
    ecodes.BTN_EAST: "circle",
    ecodes.BTN_NORTH: "triangle",
    ecodes.BTN_WEST: "square",
    ecodes.BTN_TL: "l1",
    ecodes.BTN_TR: "r1",
    ecodes.BTN_THUMBL: "l3",
    ecodes.BTN_THUMBR: "r3",
    ecodes.BTN_SELECT: "create",
    ecodes.BTN_START: "options",
    ecodes.BTN_MODE: "ps",
}


def find_dualsense() -> evdev.InputDevice | None:
    for path in evdev.list_devices():
        dev = evdev.InputDevice(path)
        name = dev.name.lower()
        caps = dev.capabilities()
        # require a gamepad-looking device (sticks + face buttons), skips the touchpad node
        if any(h in name for h in DEVICE_NAME_HINTS) and ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
            keys = caps[ecodes.EV_KEY]
            if ecodes.BTN_SOUTH in keys:
                return dev
        dev.close()
    return None


class DualSenseEvdev(Gamepad):
    def __init__(self, deadzone: float = 0.08, invert_ly: bool = True, invert_ry: bool = True):
        dev = find_dualsense()
        if dev is None:
            raise RuntimeError(
                "No DualSense found. Connect via USB/Bluetooth and check permissions "
                "(user must be in the 'input' group, or add a udev rule)."
            )
        self.dev = dev
        self.deadzone = deadzone
        self.invert_ly = invert_ly
        self.invert_ry = invert_ry
        self._lock = threading.Lock()
        self._axes = {c: 0.0 for c in ("lx", "ly", "rx", "ry", "l2", "r2")}
        self._held: dict[str, bool] = {}
        self._edges: list[str] = []
        self._absinfo = {code: dev.absinfo(code) for code in
                         (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY,
                          ecodes.ABS_Z, ecodes.ABS_RZ)}
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    # --- normalization -------------------------------------------------
    def _norm_stick(self, code, value) -> float:
        info = self._absinfo[code]
        mid = (info.max + info.min) / 2.0
        half = (info.max - info.min) / 2.0
        v = (value - mid) / half
        if abs(v) < self.deadzone:
            return 0.0
        # rescale so output is continuous at the deadzone edge
        s = (abs(v) - self.deadzone) / (1.0 - self.deadzone)
        return max(-1.0, min(1.0, s if v > 0 else -s))

    def _norm_trigger(self, code, value) -> float:
        info = self._absinfo[code]
        return (value - info.min) / (info.max - info.min)

    # --- reader thread --------------------------------------------------
    def _reader(self):
        try:
            for ev in self.dev.read_loop():
                if not self._running:
                    break
                with self._lock:
                    if ev.type == ecodes.EV_ABS:
                        self._on_abs(ev)
                    elif ev.type == ecodes.EV_KEY and ev.code in BUTTON_MAP:
                        name = BUTTON_MAP[ev.code]
                        if ev.value == 1 and not self._held.get(name):
                            self._edges.append(name)
                        self._held[name] = ev.value != 0
        except OSError:
            pass  # device unplugged; poll() keeps returning last-known zeros

    def _on_abs(self, ev):
        c = ecodes
        if ev.code == c.ABS_X:
            self._axes["lx"] = self._norm_stick(ev.code, ev.value)
        elif ev.code == c.ABS_Y:
            v = self._norm_stick(ev.code, ev.value)
            self._axes["ly"] = -v if self.invert_ly else v
        elif ev.code == c.ABS_RX:
            self._axes["rx"] = self._norm_stick(ev.code, ev.value)
        elif ev.code == c.ABS_RY:
            v = self._norm_stick(ev.code, ev.value)
            self._axes["ry"] = -v if self.invert_ry else v
        elif ev.code == c.ABS_Z:
            self._axes["l2"] = self._norm_trigger(ev.code, ev.value)
        elif ev.code == c.ABS_RZ:
            self._axes["r2"] = self._norm_trigger(ev.code, ev.value)
        elif ev.code == c.ABS_HAT0X:
            self._dpad("dpad_left", ev.value == -1)
            self._dpad("dpad_right", ev.value == 1)
        elif ev.code == c.ABS_HAT0Y:
            self._dpad("dpad_up", ev.value == -1)
            self._dpad("dpad_down", ev.value == 1)

    def _dpad(self, name, active):
        if active and not self._held.get(name):
            self._edges.append(name)
        self._held[name] = active

    # --- public ----------------------------------------------------------
    def poll(self) -> GamepadState:
        with self._lock:
            st = GamepadState(
                lx=self._axes["lx"], ly=self._axes["ly"],
                rx=self._axes["rx"], ry=self._axes["ry"],
                l2=self._axes["l2"], r2=self._axes["r2"],
                held=dict(self._held), pressed=list(self._edges),
            )
            self._edges.clear()
        return st

    def close(self):
        self._running = False
        try:
            self.dev.close()
        except OSError:
            pass
