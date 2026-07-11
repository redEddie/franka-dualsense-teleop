#!/usr/bin/env python3
"""pydualsense hardware check: rumble, adaptive triggers, lightbar, IMU.

Runs a fixed sequence (~15 s) and prints live state. Hold the pad!
    python scripts/test_haptics.py
"""
import time

from pydualsense import pydualsense, TriggerModes


def main():
    ds = pydualsense()
    ds.init()
    print(f"connected. battery={ds.battery.Level}%  usb={ds.determineConnectionType() if hasattr(ds, 'determineConnectionType') else 'n/a'}")

    try:
        # 1. lightbar: red -> green -> blue
        print("[1/4] lightbar cycle")
        for rgb in ((255, 0, 0), (0, 255, 0), (0, 0, 255)):
            ds.light.setColorI(*rgb)
            time.sleep(0.6)

        # 2. rumble: low-freq (left) then high-freq (right) ramp
        print("[2/4] rumble ramp — left (low-freq) motor")
        for i in range(0, 256, 32):
            ds.setLeftMotor(i)
            time.sleep(0.15)
        ds.setLeftMotor(0)
        time.sleep(0.3)
        print("        rumble ramp — right (high-freq) motor")
        for i in range(0, 256, 32):
            ds.setRightMotor(i)
            time.sleep(0.15)
        ds.setRightMotor(0)

        # 3. adaptive trigger: R2 rigid resistance — squeeze it!
        print("[3/4] R2 adaptive trigger: RIGID resistance for 4 s — squeeze R2")
        ds.triggerR.setMode(TriggerModes.Rigid)
        ds.triggerR.setForce(1, 255)
        t0 = time.time()
        while time.time() - t0 < 4:
            print(f"\r        R2={ds.state.R2:3d}", end="", flush=True)
            time.sleep(0.05)
        ds.triggerR.setMode(TriggerModes.Off)
        print()

        # 4. IMU live read
        print("[4/4] IMU for 4 s — tilt/shake the pad")
        t0 = time.time()
        while time.time() - t0 < 4:
            g, a = ds.state.gyro, ds.state.accelerometer
            print(f"\r        gyro=({g.Pitch:+6d},{g.Yaw:+6d},{g.Roll:+6d}) "
                  f"accel=({a.X:+6d},{a.Y:+6d},{a.Z:+6d})", end="", flush=True)
            time.sleep(0.05)
        print("\nALL HAPTICS CHECKS DONE")
    finally:
        ds.setLeftMotor(0)
        ds.setRightMotor(0)
        ds.triggerR.setMode(TriggerModes.Off)
        ds.light.setColorI(0, 0, 255)
        ds.close()


if __name__ == "__main__":
    main()
