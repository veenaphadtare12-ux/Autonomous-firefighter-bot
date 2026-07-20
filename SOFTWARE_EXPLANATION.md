# ROBOTEX ADVANCED SOFTWARE ARCHITECTURE - THE ULTIMATE CHEAT SHEET

This document explains the mathematical formulas and logic powering your custom 3-Tier Autonomous Robot. Keep this file handy! If the judges ask you how your robot works, explaining these three formulas will prove you are a true robotics engineer.

========================================================================
## TIER 1: SLAM (Simultaneous Localization and Mapping)
**Goal:** Figure out exactly where the robot is on the floor (X, Y coordinate) and map the walls.
**File:** `slam_mapping.py`
**The Problem:** We do not have hardware wheel encoders to measure distance. 
**The Solution:** Mathematical "Dead Reckoning" + Gyroscope Trigonometry.

### 1. The Heading (Theta / Angle) Math
We use the MPU6050 gyroscope to measure the exact twisting speed (Radians per Second).
Formula: `Heading = Old_Heading + (Gyro_Twist_Speed * Time_Passed)`
Code: `self.theta += gyro_z_velocity * dt`

### 2. The Forward Distance Math (Dead Reckoning)
We guess the forward speed based on how much power the battery is sending to the motors.
Formula: `Average_Speed = (Left_Power + Right_Power) / 2.0`
Code: `forward_velocity = ((left_motor_pwm + right_motor_pwm) / 2.0) * max_speed_mps`

### 3. The (X, Y) Coordinate Math (Trigonometry)
We use Sine and Cosine to project the distance traveled onto a 2D graph.
Code: 
`self.x += distance_traveled * math.cos(self.theta)`
`self.y += distance_traveled * math.sin(self.theta)`


========================================================================
## TIER 2: REINFORCEMENT LEARNING (The Brain)
**Goal:** Navigate a complex maze and generate a target Waypoint to drive towards.
**File:** `rl_brain_master.zip` (Executed inside `master_brain.py`)
**The Problem:** Hard-coding "If wall on left, turn right" is too slow and buggy for a maze.
**The Solution:** An Artificial Neural Network (PPO - Proximal Policy Optimization).

### 1. The Input (Observation Space)
We feed the Brain an array of exactly 9 numbers 10 times every second.
Code: `obs = np.array([yolo_offset, yolo_dist, tof_L, tof_C, tof_R, us_FL, us_FR, us_BL, us_BR])`
This array contains the camera data (YOLO) and the distances of all walls from the lasers and ultrasonics.

### 2. The Decision (Matrix Math)
The neural network multiplies those 9 numbers by thousands of "Weights" learned during simulation. 
Code: `action, _ = model.predict(obs)`
The output is simple: 0 (Go Forward), 1 (Turn Left), or 2 (Turn Right).

### 3. The Waypoint Generation
If the brain says "Go Forward", we project a virtual target 10 centimeters directly in front of the robot.
Code: 
`target_x = current_x + (0.1 * math.cos(current_theta))`
`target_y = current_y + (0.1 * math.sin(current_theta))`


========================================================================
## TIER 3: PID KINEMATICS (The Muscles)
**Goal:** Drive the left and right wheels at the exact perfect speeds to glide smoothly to the Target Waypoint in a curved arc.
**File:** `pid_kinematics.py`
**The Problem:** If we just turn motors on/off, the robot jerks violently and slides on the floor.
**The Solution:** Differential Drive Calculus (PID).

### 1. Finding the Target (Geometry)
We use the Pythagorean Theorem (a² + b² = c²) to find the straight-line distance to the target.
Code: `distance_error = math.hypot(target_x - current_x, target_y - current_y)`

We use the Arctangent function to find the exact angle of the target relative to the robot.
Code: `angle_to_target = math.atan2(target_y - current_y, target_x - current_x)`

### 2. The Calculus (Braking smoothly)
PID uses the "Derivative" (Rate of Change) to act as a mathematical brake. As the distance shrinks, the derivative is negative, slowing the robot down smoothly so it doesn't overshoot.
Code: `linear_derivative = distance_error - prev_error_linear`
Code: `forward_speed = (kp * distance_error) + (kd * linear_derivative)`

### 3. The Differential Drive Steering
To make a beautifully curved S-Turn, we take the Forward Speed (V) and the Spin Speed (Omega) and split them between the two wheels.
Code:
`left_pwm = v - (omega / 2.0)`
`right_pwm = v + (omega / 2.0)`
If we need to turn right (positive Omega), the math speeds up the left wheel and slows down the right wheel!
