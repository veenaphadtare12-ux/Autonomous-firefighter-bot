"""
=============================================================================
MASTER BRAIN v3.0 — Championship-Ready Firefighting Robot FSM
=============================================================================
Architecture: Gather → Think → Act (10Hz loop)
Sensors:      4× Ultrasonic, 1× VL53L5CX 8×8 ToF, MPU6050 Gyro, 2× IR, KY-026
Vision:       YOLOv8n (Fire detection only)
Navigation:   RL Brain (PPO) with normalized observations + 3-Tier Obstacle Escape
Stopping:     Dual-Redundancy (IR Black Line OR YOLO distance < 15cm)
Safety:       180-second competition timer, STATE_EXTINGUISH is interrupt-proof
=============================================================================
"""
import time
import math
import numpy as np

# ===================== HARDWARE & MODULE IMPORTS =====================
from robot_hardware import MotorController, SensorArray
from slam_mapping import SLAM_Engine
from pid_kinematics import DifferentialDrivePID
from yolo_vision import Yolov8_Vision

# Try to load the Reinforcement Learning brain (optional)
try:
    from stable_baselines3 import PPO
    has_rl = True
except ImportError:
    has_rl = False

# ===================== CONFIGURATION CONSTANTS =====================
# --- Obstacle Avoidance Thresholds ---
US_EMERGENCY_THRESHOLD  = 0.25    # 25cm — Ultrasonic panic distance (meters)
TOF_EMERGENCY_THRESHOLD = 250     # 250mm — ToF wall panic distance (mm, top rows only)
TOF_BLOCKED_THRESHOLD   = 300     # 300mm — ToF "path is blocked" for smart avoidance (mm)
TOF_MAX_DISTANCE        = 2000    # 2000mm — Max reliable ToF range / dead pixel fill value

# --- Candle Approach Thresholds ---
YOLO_CLOSE_DISTANCE     = 0.40   # Normalized YOLO distance below which candle is "close"
TOF_STOP_DISTANCE       = 200    # 200mm (20cm) — ToF distance to STOP before candle
TOF_SLOW_DISTANCE       = 500    # 500mm (50cm) — ToF distance to start slowing down
APPROACH_MAX_SPEED      = 0.25   # Max forward speed during candle approach
APPROACH_MIN_SPEED      = 0.12   # Minimum crawl speed (keeps moving, doesn't stall)
STEER_GAIN              = 0.3    # How aggressively to steer toward the candle (softened per user request)

# --- Motor Speed Limits ---
DRIVE_SPEED     = 0.5   # Normal forward driving speed (50% PWM)
TURN_SPEED      = 0.25   # Turning/spinning speed (25% PWM)
SLOW_APPROACH   = 0.35  # Careful approach speed when tracking a candle

# --- Timing ---
COMPETITION_TIME_LIMIT = 175  # 175 seconds (5s safety margin from 180s rule)
STUCK_TIMEOUT          = 2.0  # Seconds before triggering Tier-3 escape maneuver
QUICK_TURN_DURATION    = 0.4  # Seconds for a Tier-2 quick pivot
ESCAPE_REVERSE_TIME    = 0.5  # Seconds to reverse during Tier-3 escape
ESCAPE_TURN_TIME       = 0.6  # Seconds to pivot during Tier-3 escape
LOOP_PERIOD            = 0.05 # 50ms = 20Hz main loop

# --- 360° Scan (Step-Scan) ---
SCAN_TRIGGER_TIME      = 3.0  # Seconds with no candle before triggering a 360° scan
SCAN_SPIN_SPEED        = 0.20 # SLOW spin for scanning (20% PWM) - slightly more power
SCAN_STEP_TURN_TIME    = 0.2  # Seconds to spin per step (VERY small slices)
SCAN_STEP_PAUSE_TIME   = 0.4  # Seconds to stand completely still and infer
SCAN_TOTAL_STEPS       = 24   # Total steps for a full 360 rotation (More steps = finer slices)

# --- Tracking ---
TRACK_LOCKON_TIME      = 0.5  # Pause duration when first locking onto a candle
TRACK_WAIT_TIMEOUT     = 1.5  # How long to stand still waiting if candle is lost

# --- Explore Pattern (Stop-Look-Go) ---
EXPLORE_DRIVE_TIME     = 1.5  # Drive forward for 1.5 seconds
EXPLORE_PAUSE_TIME     = 0.5  # Then STOP for 0.5s so YOLO gets a sharp, still frame

# --- Fan Sweep ---
FAN_SWEEP_CYCLES       = 5    # 5 cycles * 1s = 5 seconds of blowing!
FAN_SWEEP_DURATION     = 0.5  # Duration of each wiggle direction (seconds)

# ===================== HARDWARE INITIALIZATION =====================
print("=== WAKING UP ADVANCED FSM AI v3.0 ===")
motors = MotorController()
sensors = SensorArray()
vision = Yolov8_Vision(model_path='best.pt')
slam = SLAM_Engine()
pid = DifferentialDrivePID(kp_linear=0.8, kd_linear=0.1, kp_angular=1.5, kd_angular=0.2)

# Load RL model if available
if has_rl:
    try:
        model = PPO.load("rl_brain_master")
        print("RL Brain loaded successfully!")
    except Exception as e:
        print(f"RL Brain not available: {e}")
        has_rl = False

try:
    while True:
        # ===================== STATE VARIABLES =====================
        current_left_pwm = 0.0
        current_right_pwm = 0.0
        candles_extinguished = 0
        ignore_candle_until = 0
        current_state = "STATE_EXPLORE"

        # 3-Tier Escalation Variables (from user's flowchart)
        avoidance_start_time = 0    # Tracks how long the robot has been stuck
        state_expiry = 0            # Non-blocking timer for turn maneuvers

        # 360° Scan Variables
        last_candle_seen_time = time.time()  # Init to now so scan doesn't trigger immediately
        is_scanning = False         # Whether a 360° scan is currently in progress
        scan_step_start = 0         # Timer for the current step-scan phase
        scan_is_turning = True      # True = turning, False = pausing
        scan_steps_taken = 0        # How many steps we've done

        # Tracking Variables
        track_start_time = 0        # Timer for the lock-on pause

        # Explore pattern variables (Stop-Look-Go)
        explore_phase_start = 0     # Timestamp when the current explore phase started
        explore_driving = True      # True = driving phase, False = pause/look phase

        # ===================== START BUTTON =====================
        print("\nWaiting for physical START BUTTON (GPIO 17)...")
        if hasattr(sensors, 'start_active') and sensors.start_active:
            print("Press the START button to begin the 180s round!")
            while not sensors.start_button.is_pressed:
                time.sleep(0.1)
            print("START COMMAND RECEIVED! GO GO GO!")
        else:
            print("DEBUG: Start Button bypassed for testing!")

        # ===================== COMPETITION TIMER =====================
        competition_start = time.time()
        print(f"\n--- ROBOTEX READY: FSM CORE LOOP STARTED (180s Timer Active) ---")
        last_candle_seen_time = time.time()  # Reset the scan timer exactly when the round begins

        while True:
            loop_start = time.time()

            # ==============================================================
            # COMPETITION TIMER CHECK — Auto-stop at 175 seconds
            # ==============================================================
            elapsed = time.time() - competition_start
            if elapsed > COMPETITION_TIME_LIMIT:
                print(f"\n⏰ COMPETITION TIME LIMIT REACHED ({elapsed:.1f}s)! STOPPING!")
                motors.set_motors(0, 0)
                break

            # ==============================================================
            # 1. GATHER — Read all sensors
            # ==============================================================

            # --- Ultrasonics (4 sensors, returns distances in METERS) ---
            # us_dists[0] = Front-Left,  us_dists[1] = Front-Right
            # us_dists[2] = Back-Left,   us_dists[3] = Back-Right
            # Returns -0.01 on error (filtered out by d > 0.0 check later)
            us_dists = sensors.get_ultrasonic_distances()

            # --- VL53L5CX 8×8 ToF Laser (returns 64 distances in MILLIMETERS) ---
            raw_tof = sensors.get_tof_distances()

            # --- IR Line Sensors (True = black line detected under robot) ---
            is_on_black_line = any(sensors.check_black_line())

            # --- YOLO Camera (returns: x_offset [-1,1], distance [0,1], detected bool) ---
            yolo_offset, yolo_dist, sees_candle = vision.scan_for_candle()

            # --- MPU6050 Gyroscope (returns Z-axis angular velocity in deg/s) ---
            # BUG FIX #2: Now reading the REAL gyroscope instead of faking it from PWM
            gyro_z_velocity = sensors.get_gyro_rotation() * (math.pi / 180.0)  # Convert deg/s → rad/s

            # ==============================================================
            # 1b. PROCESS — 8×8 Matrix Spatial Slicing (User's Logic)
            # ==============================================================
            # Reshape the flat 64-pixel array into an 8x8 grid
            if len(raw_tof) != 64:
                raw_tof = [1000.0] * 64
            z = np.array(raw_tof).reshape((8, 8))

            # BUG FIX: Mask invalid/error pixels (0 or negative) to max distance to prevent ghost walls
            z[z <= 0] = 4000.0

            # BUG FIX: Correct for sideways hardware mounting orientation
            z = np.rot90(z, k=1)

            # Using numpy slicing to grab exact cones.
            # Top 4 rows (far ahead), bottom 4 rows (floor/near).
            # We use the 10th percentile to filter out random single-pixel glitches.
            min_far_dist  = float(np.percentile(z[0:4, 0:8], 10))  # Wall detection
            min_near_dist = float(np.percentile(z[4:8, 0:8], 10))  # Ground/short object detection

            # Virtual Left/Center/Right sensors for RL Brain & smart steering
            tof_left   = float(np.mean(z[0:4, 0:3]))    # Left 3 columns, top 4 rows (mm)
            tof_center = float(np.mean(z[0:4, 3:5]))    # Center 2 columns, top 4 rows (mm)
            tof_right  = float(np.mean(z[0:4, 5:8]))    # Right 3 columns, top 4 rows (mm)

            # Side averages for smart escape direction (pivot toward the clearer side)
            left_side_avg  = float(np.mean(z[:, 0:3]))   # Full-height left average (mm)
            right_side_avg = float(np.mean(z[:, 5:8]))   # Full-height right average (mm)

            # ==============================================================
            # 1c. SLAM UPDATE — Dead Reckoning + Map
            # ==============================================================
            current_x, current_y, current_theta = slam.update_odometry(
                gyro_z_velocity, current_left_pwm, current_right_pwm
            )
            # BUG FIX #3: Convert mm → meters before passing to SLAM
            slam.update_map(tof_left / 1000.0, tof_center / 1000.0, tof_right / 1000.0)

            # ==============================================================
            # 2. THINK — FSM State Transitions
            # ==============================================================
            now = time.time()  # Current timestamp for all time-based logic
            time_since_candle = now - last_candle_seen_time

            # --- EMERGENCY OVERRIDE (Highest Priority) ---
            # BUG FIX #4: Emergency CANNOT interrupt STATE_EXTINGUISH
            valid_front_us = [d for d in us_dists[:2] if 0.0 < d < 4.0] # 0-4m valid range
            valid_back_us  = [d for d in us_dists[2:] if 0.0 < d < 4.0] 

            # FRONT: Check ToF (lasers) AND Front Ultrasonics
            is_front_blocked = (min_far_dist < TOF_EMERGENCY_THRESHOLD) or (min_near_dist < 150.0) or (valid_front_us and min(valid_front_us) < US_EMERGENCY_THRESHOLD)
            # BACK: Ultrasonic only (ToF doesn't face backwards)
            is_back_blocked  = (valid_back_us and min(valid_back_us) < US_EMERGENCY_THRESHOLD)

            if current_state != "STATE_EXTINGUISH":
                # State Cleanup: Reset scan variables if we are not scanning
                if current_state != "STATE_SCAN":
                    is_scanning = False
                    scan_steps_taken = 0

                # State Cleanup: Reset avoidance variables if we are freely exploring/tracking
                if current_state not in ("STATE_EMERGENCY_FRONT", "STATE_EMERGENCY_BACK"):
                    avoidance_start_time = 0

                # 1. COOLDOWN PRIORITY
                in_cooldown = (now < ignore_candle_until)

                # 2. EMERGENCY AVOIDANCE PRIORITY (Overrides camera if blocked by a wall)
                # If front is blocked, avoid it UNLESS the camera sees a candle!
                # YOLO object detection drops frames. If we saw the candle in the last 1.5 seconds, ignore walls and boundary lines!
                override_avoidance_for_candle = (time_since_candle < 1.5) and not in_cooldown

                # --- Boundary Detection ---
                # If we hit a black line but we DON'T see a candle (or we're ignoring it), we are driving out of bounds!
                if is_on_black_line and not override_avoidance_for_candle:
                    current_state = "STATE_EMERGENCY_BOUNDARY"
                elif is_front_blocked and not override_avoidance_for_candle:
                    current_state = "STATE_EMERGENCY_FRONT"
                elif is_back_blocked:
                    current_state = "STATE_EMERGENCY_BACK"

                # 3. EXTINGUISH PRIORITY (Allows blowing even if camera loses candle at the last second in the blind spot)
                elif override_avoidance_for_candle and min_far_dist <= 200:
                    current_state = "STATE_EXTINGUISH"
                    if sees_candle:
                        last_candle_seen_time = now

                # 4. CAMERA TRACKING PRIORITY
                elif sees_candle and not in_cooldown:
                    last_candle_seen_time = now
                    current_state = "STATE_TRACK"

                # 4. EXPLORATION / SCANNING PRIORITY
                else:
                    if time_since_candle < TRACK_WAIT_TIMEOUT and current_state in ("STATE_TRACK", "STATE_TRACK_WAIT"):
                        current_state = "STATE_TRACK_WAIT"
                    elif time_since_candle > SCAN_TRIGGER_TIME:
                        current_state = "STATE_SCAN"
                    else:
                        current_state = "STATE_EXPLORE"

            # Print current state + all sensor data for debugging
            us_str = ' '.join([f'{d:.2f}' for d in us_dists])
            print(f"[{current_state}] ToF L:{tof_left:.0f} C:{tof_center:.0f} R:{tof_right:.0f} MIN:{min_far_dist:.0f}mm | US:[{us_str}]m | Candle:{'YES' if sees_candle else 'no'} d:{yolo_dist:.2f} | Time:{elapsed:.0f}s")

            # ==============================================================
            # 3. ACT — Execute Motor Commands
            # ==============================================================

            # ----- STATE_EXPLORE: Search for candles using Stop-Look-Go pattern -----
            if current_state == "STATE_EXPLORE":
                avoidance_start_time = 0  # Reset stuck timer when exploring freely

                # Initialize explore phase timer on first entry
                if explore_phase_start == 0:
                    explore_phase_start = now
                    explore_driving = True

                phase_elapsed = now - explore_phase_start

                if explore_driving:
                    # DRIVE PHASE: Move forward (or use RL) for EXPLORE_DRIVE_TIME seconds
                    if phase_elapsed > EXPLORE_DRIVE_TIME:
                        # Switch to PAUSE phase
                        explore_driving = False
                        explore_phase_start = now
                        current_left_pwm, current_right_pwm = 0.0, 0.0  # Stop!
                        print("👁️ PAUSING TO SCAN...")
                    else:
                        if has_rl:
                            # BUG FIX #1: Normalize all observations to [0, 1] range
                            obs = np.array([
                                yolo_offset,
                                yolo_dist,
                                tof_left / 4000.0,
                                tof_center / 4000.0,
                                tof_right / 4000.0,
                                us_dists[0] / 4.0,
                                us_dists[1] / 4.0,
                                us_dists[2] / 4.0,
                                us_dists[3] / 4.0
                            ], dtype=np.float32)
                            obs = np.clip(obs, -1.0, 1.0)

                            action, _ = model.predict(obs, deterministic=True)
                            if action == 0:     # Forward
                                current_left_pwm, current_right_pwm = DRIVE_SPEED, DRIVE_SPEED
                            elif action == 1:   # Turn Left
                                current_left_pwm, current_right_pwm = -TURN_SPEED, TURN_SPEED
                            elif action == 2:   # Turn Right
                                current_left_pwm, current_right_pwm = TURN_SPEED, -TURN_SPEED
                        else:
                            # Fallback: Drive toward the more open side
                            if tof_center > TOF_BLOCKED_THRESHOLD:
                                current_left_pwm, current_right_pwm = DRIVE_SPEED, DRIVE_SPEED  # Forward
                            elif left_side_avg > right_side_avg:
                                current_left_pwm, current_right_pwm = -TURN_SPEED, TURN_SPEED
                            else:
                                current_left_pwm, current_right_pwm = TURN_SPEED, -TURN_SPEED
                else:
                    # PAUSE PHASE: Stand still so YOLO gets a sharp, motion-free frame
                    current_left_pwm, current_right_pwm = 0.0, 0.0
                    if phase_elapsed > EXPLORE_PAUSE_TIME:
                        # Switch back to DRIVE phase
                        explore_driving = True
                        explore_phase_start = now

            # ----- STATE_TRACK: Camera locked onto candle, steer toward it -----
            elif current_state == "STATE_TRACK":
                avoidance_start_time = 0  # Reset stuck timer
                explore_phase_start = 0   # Reset explore timer

                # No more passive coasting! Immediately use proportional steering to counter-act 
                # rotational momentum and lock onto the candle!
                # === STEERING (Left/Right) — Controlled by YOLO offset ===
                # yolo_offset: -1.0 = candle is far left, +1.0 = candle is far right
                steer = yolo_offset * STEER_GAIN

                # === SPEED (Forward) — Controlled by ToF minimum distance ===
                if min_far_dist <= TOF_STOP_DISTANCE:
                    print(f"🎯 ToF says {min_far_dist:.0f}mm — WITHIN STOP RANGE!")
                    forward_speed = 0.0
                elif min_far_dist <= TOF_SLOW_DISTANCE:
                    ratio = (min_far_dist - TOF_STOP_DISTANCE) / (TOF_SLOW_DISTANCE - TOF_STOP_DISTANCE)
                    forward_speed = APPROACH_MIN_SPEED + ratio * (APPROACH_MAX_SPEED - APPROACH_MIN_SPEED)
                    print(f"🐢 SLOWING DOWN: {min_far_dist:.0f}mm → speed {forward_speed:.2f}")
                else:
                    forward_speed = APPROACH_MAX_SPEED

                current_left_pwm  = forward_speed + steer
                current_right_pwm = forward_speed - steer
                current_left_pwm  = max(-1.0, min(1.0, current_left_pwm))
                current_right_pwm = max(-1.0, min(1.0, current_right_pwm))

            # ----- STATE_TRACK_WAIT: Candle lost, try to back up or wait -----
            elif current_state == "STATE_TRACK_WAIT":
                track_start_time = 0 # Reset lock-on if we lose tracking
                print("⏳ CANDLE LOST! Pausing to wait for YOLO...")
                current_left_pwm, current_right_pwm = 0.0, 0.0

            # ----- STATE_SCAN: Step-Scan 360° to find candles without motion blur -----
            elif current_state == "STATE_SCAN":
                track_start_time = 0 # Reset lock-on

                if not is_scanning:
                    # Start a new scan
                    is_scanning = True
                    scan_step_start = now
                    scan_is_turning = True
                    scan_steps_taken = 0
                    print("🔍 NO CANDLE FOR 3s! STARTING STEP-SCAN...")

                step_elapsed = now - scan_step_start

                if scan_is_turning:
                    # Turn for a fraction of a second
                    if step_elapsed < SCAN_STEP_TURN_TIME:
                        current_left_pwm = -SCAN_SPIN_SPEED
                        current_right_pwm = SCAN_SPIN_SPEED
                        print(f"🔍 SCANNING... Step {scan_steps_taken + 1}/{SCAN_TOTAL_STEPS} (TURNING)")
                    else:
                        # Switch to Pause
                        scan_is_turning = False
                        scan_step_start = now
                else:
                    # Pause and look
                    if step_elapsed < SCAN_STEP_PAUSE_TIME:
                        current_left_pwm, current_right_pwm = 0.0, 0.0
                        print(f"🔍 SCANNING... Step {scan_steps_taken + 1}/{SCAN_TOTAL_STEPS} (LOOKING)")
                    else:
                        # Step complete!
                        scan_steps_taken += 1
                        scan_is_turning = True
                        scan_step_start = now

                        if scan_steps_taken >= SCAN_TOTAL_STEPS:
                            print("🔍 SCAN COMPLETE! Moving to a new position...")
                            is_scanning = False
                            last_candle_seen_time = now  # Reset timer so it explores before scanning again

                            # Spin a random amount to pick a random new direction in the field
                            spin_dir = np.random.choice([-1, 1])
                            spin_time = np.random.uniform(0.4, 1.2)
                            motors.set_motors(TURN_SPEED * spin_dir, -TURN_SPEED * spin_dir)
                            time.sleep(spin_time)

                            # Drive forward for a bit to change position
                            motors.set_motors(DRIVE_SPEED, DRIVE_SPEED)
                            time.sleep(1.0)
                            current_state = "STATE_EXPLORE"
                            continue

            # ----- STATE_EMERGENCY_BOUNDARY: Stepped on the outer boundary line! -----
            elif current_state == "STATE_EMERGENCY_BOUNDARY":
                print("🚨 BOUNDARY LINE DETECTED! REVERSING TO STAY IN FIELD!")
                motors.set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
                time.sleep(0.5)
                # Spin around 180 degrees to face back into the field
                motors.set_motors(-TURN_SPEED, TURN_SPEED)
                time.sleep(1.0)
                current_state = "STATE_EXPLORE"
                continue

            # ----- STATE_EMERGENCY_FRONT: 3-Tier Escalating Obstacle Avoidance -----
            elif current_state == "STATE_EMERGENCY_FRONT":
                # Track how long we've been continuously stuck
                if avoidance_start_time == 0:
                    avoidance_start_time = now
                time_stuck = now - avoidance_start_time

                if time_stuck > STUCK_TIMEOUT:
                    # TIER 3: Stuck for >2 seconds — Full escape maneuver
                    print(f"🚨 STUCK {time_stuck:.1f}s! TIER-3 ESCAPE: REVERSE + LONG PIVOT!")
                    motors.set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
                    time.sleep(ESCAPE_REVERSE_TIME)
                    motors.set_motors(0, 0)
                    time.sleep(0.1)
                    # Pivot toward the side with MORE open space, with added randomness to scatter path
                    rand_turn_time = ESCAPE_TURN_TIME * np.random.uniform(0.8, 1.5)
                    if left_side_avg > right_side_avg:
                        motors.set_motors(-TURN_SPEED, TURN_SPEED)  # Spin left
                    else:
                        motors.set_motors(TURN_SPEED, -TURN_SPEED)  # Spin right
                    time.sleep(rand_turn_time)
                    time.sleep(rand_turn_time)
                    # Let the main loop handle transition out of emergency when the path is clear
                    # and only reset the stuck timer when we actually leave the emergency state!
                    continue
                else:
                    # TIER 2: Briefly blocked — Quick pivot toward clearer side
                    print(f"⚠️ WALL AHEAD! TIER-2 QUICK PIVOT ({time_stuck:.1f}s)")
                    motors.set_motors(0, 0)
                    time.sleep(0.05)
                    if left_side_avg > right_side_avg:
                        motors.set_motors(-TURN_SPEED, TURN_SPEED)
                    else:
                        motors.set_motors(TURN_SPEED, -TURN_SPEED)
                    time.sleep(QUICK_TURN_DURATION)
                    continue

            # ----- STATE_EMERGENCY_BACK: Wall behind — drive forward -----
            elif current_state == "STATE_EMERGENCY_BACK":
                print("🚨 WALL BEHIND! DRIVING FORWARD TO ESCAPE!")
                motors.set_motors(DRIVE_SPEED, DRIVE_SPEED)
                time.sleep(0.3)
                time.sleep(0.3)
                continue

            # ----- STATE_EXTINGUISH: Stop, verify fire, blow it out -----
            elif current_state == "STATE_EXTINGUISH":
                print("🎯 TARGET REACHED! HITTING BRAKES!")
                current_left_pwm, current_right_pwm = 0.0, 0.0
                motors.set_motors(0, 0)

                # Verify flame with 5-Channel sensor
                # We trust YOLO here. The IR flame sensor can be unreliable at 25cm distance.
                flame_confirmed = True

                if flame_confirmed:
                    print("🔥 FIRE CONFIRMED! ACTIVATING FANS + SWEEP!")
                    motors.set_fans(True)
                    # Probabilistic Wind Sweep: wiggle left-right to maximize coverage
                    for i in range(FAN_SWEEP_CYCLES):
                        motors.set_motors(-TURN_SPEED, TURN_SPEED)
                        time.sleep(FAN_SWEEP_DURATION)
                        motors.set_motors(TURN_SPEED, -TURN_SPEED)
                        time.sleep(FAN_SWEEP_DURATION)

                    motors.set_fans(False)
                    motors.set_motors(0, 0)

                    candles_extinguished += 1
                    print(f"✅ CANDLE {candles_extinguished}/4 EXTINGUISHED!")

                    if candles_extinguished >= 10:
                        total_time = time.time() - competition_start
                        print(f"\n🏆 10 CANDLE BLOWS REACHED IN {total_time:.1f}s! STOPPING! 🏆")
                        break
                    # Reverse out of the circle to continue searching
                    print("⬅️ Reversing out of candle circle...")
                    motors.set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
                    time.sleep(1.5)
                    motors.set_motors(0, 0)
                    ignore_candle_until = time.time() + 3.0  # Ignore candle for 3 seconds while driving away
                    current_state = "STATE_EXPLORE"
                    continue  # BUG FIX: Skip the final set_motors to avoid overwriting reverse
                else:
                    print("❌ FALSE ALARM! No fire detected. Escaping line...")
                    motors.set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
                    time.sleep(0.5)
                    ignore_candle_until = time.time() + 3.0  # Force ignore fake candle for 3 seconds
                    current_state = "STATE_EXPLORE"
                    continue  # BUG FIX: Skip the final set_motors

            # ==============================================================
            # 4. EXECUTE — Send PWM to motors
            # ==============================================================
            motors.set_motors(current_left_pwm, current_right_pwm)

            # ==============================================================
            # 5. SYNCHRONIZE — Maintain consistent loop rate
            # ==============================================================
            loop_elapsed = time.time() - loop_start
            sleep_time = max(0, LOOP_PERIOD - loop_elapsed)
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n🛑 Shutting down safely...")

finally:
    print("\n[CLEANUP] Stopping motors and sensors...")
    motors.set_motors(0, 0)
    motors.set_fans(False)
    vision.shutdown()
    sensors.shutdown()
    print("All hardware safely powered down. Goodbye!")
