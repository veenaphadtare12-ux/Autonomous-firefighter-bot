# Robotex AI: Physical Hardware Testing Log

This document serves as a permanent record of all physical hardware bugs discovered during real-world testing on the Raspberry Pi 5, and the exact software engineering solutions used to fix them.

## 1. The "Runaway Motor" Bug (Software PWM Glitch)
**The Bug:** When running code in Thonny IDE, pressing the RED STOP button would instantly crash the Python script. Instead of the robot stopping, the wheels would randomly speed up to 100% maximum power and the robot would blast across the floor uncontrollably.
**The Cause:** Thonny performs a "hard kill" on background threads. When the script dies instantly, the Raspberry Pi's GPIO pins are left in a "floating" state in hardware memory. Because the `gpiozero` PWM thread died, the pin randomly defaulted to a logic HIGH signal, sending 100% voltage to the motor drivers.
**The Fix:** We implemented a `motors.shutdown()` method inside a `try...finally:` block. This guarantees that no matter how the script crashes (or if the user presses `Ctrl+C` in the terminal), the Python script intercepts the crash and explicitly forces all motor pins to `0V` and `.close()`s them in memory before dying.

## 2. The Ultrasonic Thread Crash (Pi 5 Incompatibility)
**The Bug:** The standard `DistanceSensor` class in `gpiozero` kept throwing a massive red error: `DistanceSensorNoEcho`.
**The Cause:** The Raspberry Pi 5 uses a completely new chip architecture (RP1) for GPIO. The background timing threads inside the `gpiozero` library are too slow for the Pi 5's new architecture, causing it to miss the ultrasonic echo returning at the speed of sound.
**The Fix:** We completely bypassed the `gpiozero` background threads. We built a custom `RawUltrasonic` class that manually toggles the Trigger pin and uses a raw `while` loop to count the microsecond timing of the Echo pin directly on the CPU.

## 3. The Laser "999.0 cm" Bug (2D ctypes Arrays)
**The Bug:** The VL53L5CX laser sensor successfully booted up, but kept returning `999.0` (or `100.0 cm`) forever. The robot thought the path was clear and refused to stop.
**The Cause:** The `vl53l5cx_ctypes` Python library is a C++ wrapper. When we asked it for the 64 pixels of data (`get_data().distance_mm`), we expected a simple Python List of 64 numbers. Instead, C++ returned a **2-Dimensional Array** (`c_short_Array_64_Array_1`). It gave us an array of length 1, and *hiding inside* that 1 slot was the array of 64 pixels! Because our Python code looked at the outer array, it thought it only received 1 pixel of data, threw a mathematical error, and the safety code defaulted the laser to 999.0 to prevent a crash.
**The Fix:** We updated `robot_hardware.py` to extract the inner array by appending `[0]` to the raw data: `self.vl53.get_data().distance_mm[0]`. This successfully extracted all 64 pixels!

## Project Files on the Raspberry Pi
As of this log, all experimental testing junk files have been deleted. The ONLY active files stored on the Raspberry Pi `/home/veena_pi/robotex_ai/` directory are the Master Architecture files:
1. `robot_hardware.py` *(The Master Brain that talks to all sensors and motors)*
2. `master_brain.py` *(The final architecture file for autonomous mode)*
3. `super_maze.py` *(The fully verified, working obstacle avoidance test script)*
4. `robot_env.py` *(RL training environment, currently inactive)*
5. `pid_kinematics.py` *(Currently inactive)*
6. `slam_mapping.py` *(Currently inactive)*
7. `HARDWARE_LOG.md` *(This file)*
