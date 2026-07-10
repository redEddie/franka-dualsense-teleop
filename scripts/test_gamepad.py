#!/usr/bin/env python3
"""Print live DualSense state — verify connection, axes and button mapping."""
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dsfranka.input.dualsense_evdev import DualSenseEvdev


def main():
    pad = DualSenseEvdev()
    print(f"device: {pad.dev.name} ({pad.dev.path})  — Ctrl+C to quit")
    try:
        while True:
            s = pad.poll()
            for b in s.pressed:
                print(f"\n[press] {b}")
            held = ",".join(k for k, v in s.held.items() if v)
            print(f"\rL({s.lx:+.2f},{s.ly:+.2f}) R({s.rx:+.2f},{s.ry:+.2f}) "
                  f"L2={s.l2:.2f} R2={s.r2:.2f} held=[{held}]      ", end="")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        pad.close()


if __name__ == "__main__":
    main()
