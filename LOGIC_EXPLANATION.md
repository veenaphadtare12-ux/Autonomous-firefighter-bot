# The Main Logic: How Robotex AI Actually Works

This document explains the core mathematical and logical algorithms that power the V2 Master Brain. If you ever need to explain to a judge or professor how your robot works, this is the exact logic.

---

## 1. The Core Loop (10Hz Cycle)
Robots cannot just "think" continuously like humans; they operate in cycles. The `autonomous_core.py` runs exactly **10 times per second (10Hz)**. 
Every 100 milliseconds, it does exactly three things in this order:
1. **Gather:** Read all sensors, odometry, and camera frames.
2. **Think:** Pass data into the State Machine to choose a Target Waypoint (X, Y).
3. **Act:** Pass the Waypoint into the APF and PID controllers to spin the wheels.

---

## 2. The Finite State Machine (The Brain)
A Finite State Machine (FSM) means the robot can only be in *one state at a time*. It uses strict rules to jump between them:

* **STATE_EXPLORE (The Default):** If the camera sees no fire, the robot feeds its ultrasonic and laser distances into the **Stable-Baselines3 Reinforcement Learning Neural Network**. The RL model outputs a prediction (Go Forward, Turn Left, or Turn Right) based on its training to map the maze.
* **STATE_TRACK:** If the YOLO background thread suddenly shouts *"I see a candle!"*, the FSM instantly abandons the RL model. It calculates the exact X/Y coordinate of the candle and makes that the new Target Waypoint.
* **STATE_EXTINGUISH:** If the robot is in `TRACK` mode and the YOLO box gets huge (or the laser reads < 15cm), the robot knows it has arrived. It stops the motors and triggers the fan.
* **STATE_EMERGENCY:** The highest priority. If the IR sensor detects the black line of the arena edge, it instantly overrides every other state, slams the wheels into reverse, and then returns to `EXPLORE`.

---

## 3. Artificial Potential Fields (APF)
This is the most advanced algorithm in the code. It solves the problem: *"What if YOLO tells the robot to drive straight toward the candle, but there is a wall in the way?"*

APF works exactly like magnets:
1. **The Attractive Force (The Candle):** The Target Waypoint exerts a gravitational "pull" on the robot. The robot calculates a mathematical vector pointing straight at it.
2. **The Repulsive Force (The Walls):** Every ultrasonic and laser reading acts like a magnetic "push". If the Front-Left ultrasonic reads 20cm, the APF generates a vector pushing the robot *away* to the Back-Right. The closer the wall, the stronger the push!
3. **The Sum:** The robot adds the Attractive vector and the Repulsive vectors together. The result is a new, curved vector that safely routes the robot *around* the obstacle while still moving toward the candle!

---

## 4. PID Kinematics (The Muscles)
Once the APF gives us a safe, final Waypoint (X, Y), we have to figure out exactly how much voltage to send to the Left and Right wheels. We use a **Proportional-Integral-Derivative (PID)** Controller.

1. **Calculate Linear Error:** How far away in meters is the waypoint?
2. **Calculate Angular Error:** How many degrees do I need to twist to face the waypoint?
3. **The PID Math:** 
   * *Proportional (P):* The bigger the error, the faster the motor spins. (If you are far away, drive fast. If you are close, drive slow).
   * *Derivative (D):* Looks into the future to prevent overshooting. If the robot is spinning too fast toward the target angle, the D-term slams on the brakes so the robot doesn't spin past it.
4. **Differential Drive Math:** It takes the desired Forward Speed ($V$) and the desired Twist Speed ($\omega$) and converts them to wheel speeds:
   * `Left Wheel = V - (\omega / 2)`
   * `Right Wheel = V + (\omega / 2)`

---

## 5. Vision Threading
If we ask YOLO to scan a picture inside the main 10Hz loop, the robot will freeze for 0.2 seconds while YOLO thinks, causing it to crash into walls.
Instead, `yolo_vision.py` runs on a completely separate **Background Thread** on the Raspberry Pi's CPU. The background thread constantly pulls pictures from the camera and stores them in memory. When the main robot loop asks for a picture, it doesn't wait for the camera; it just instantly grabs the newest picture from memory, allowing the robot to drive flawlessly without lag!
