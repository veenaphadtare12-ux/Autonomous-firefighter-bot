import time
import warnings
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice, DistanceSensor, Button

# Suppress annoying gpiozero warnings on Pi 5
warnings.filterwarnings("ignore")

# --- Hardware I2C Libraries ---
try:
    from mpu6050 import mpu6050
except ImportError:
    pass

try:
    import vl53l5cx_ctypes as vl53l5cx
except ImportError:
    pass

try:
    import board
    import busio
    import adafruit_vl53l0x
except ImportError:
    pass

# smbus2 / MLX90614 Removed (Swapped to KY-026 Flame Sensor)


class MotorController:
    """
    Controls the Bridged TB6612FNG Motor Drivers.
    Includes fail-safe shutdown to prevent Thonny runaway glitches.
    """
    def __init__(self):
        print("Initializing Motor Controller...")
        
        # --- CALIBRATION SETTINGS ---
        self.INVERT_LEFT = False
        self.INVERT_RIGHT = True
        self.LEFT_TRIM = 1.0
        self.RIGHT_TRIM = 1.0
        
        # Left Wheels (Bridged)
        self.left_pwm = PWMOutputDevice(12)
        self.left_in1 = DigitalOutputDevice(27)
        self.left_in2 = DigitalOutputDevice(22)
        
        # Right Wheels (Bridged)
        self.right_pwm = PWMOutputDevice(13)
        self.right_in1 = DigitalOutputDevice(23)
        self.right_in2 = DigitalOutputDevice(24)
        
        # Dual 4-inch Fans (Relay on GPIO 18)
        # BUGFIX: The relay module is active-low! active_high=False inverts the logic automatically.
        self.fans = DigitalOutputDevice(18, active_high=False, initial_value=False)
        
        # Ensure starting at 0
        self.set_motors(0, 0)
        self.fans.off()

    def set_motors(self, left_target, right_target):
        # Apply Inversions
        if self.INVERT_LEFT:
            left_target = -left_target
        if self.INVERT_RIGHT:
            right_target = -right_target
            
        # Apply Trims
        left_speed = left_target * self.LEFT_TRIM
        right_speed = right_target * self.RIGHT_TRIM

        # INVERT POLARITY: The robot physically drove backwards when commanded forward
        left_speed = -left_speed
        right_speed = -right_speed

        # TB6612FNG CURRENT LIMITER (Max 80% PWM to protect from 12V inrush current burnouts)
        MAX_SAFE_SPEED = 0.80
        left_speed = max(min(left_speed, MAX_SAFE_SPEED), -MAX_SAFE_SPEED)
        right_speed = max(min(right_speed, MAX_SAFE_SPEED), -MAX_SAFE_SPEED)

        # Left Direction
        if left_speed > 0:
            self.left_in1.on()
            self.left_in2.off()
        elif left_speed < 0:
            self.left_in1.off()
            self.left_in2.on()
        else:
            self.left_in1.off()
            self.left_in2.off()
            
        # Right Direction
        if right_speed > 0:
            self.right_in1.on()
            self.right_in2.off()
        elif right_speed < 0:
            self.right_in1.off()
            self.right_in2.on()
        else:
            self.right_in1.off()
            self.right_in2.off()
            
        self.left_pwm.value = abs(left_speed)
        self.right_pwm.value = abs(right_speed)

    def set_fans(self, on):
        if on:
            self.fans.on()
        else:
            self.fans.off()

    def shutdown(self):
        """
        FAIL-SAFE SHUTDOWN: Closes pins to prevent Thonny from locking them HIGH.
        Always call this in a finally block!
        """
        try:
            self.set_motors(0, 0)
            self.left_pwm.close()
            self.left_in1.close()
            self.left_in2.close()
            self.right_pwm.close()
            self.right_in1.close()
            self.right_in2.close()
            self.fans.close()
            print("Motors and Fans safely powered down.")
        except Exception as e:
            print(f"Error during shutdown: {e}")


class RawUltrasonic:
    """
    Custom Pi 5 Bypass for Ultrasonic Sensors to avoid gpiozero thread crashes.
    """
    def __init__(self, trig_pin, echo_pin):
        self.trig = DigitalOutputDevice(trig_pin)
        self.echo = DigitalInputDevice(echo_pin)
        
    def get_distance_cm(self):
        self.trig.on()
        time.sleep(0.00001)
        self.trig.off()
        
        start_t = time.time()
        timeout = start_t + 0.05
        
        pulse_start = start_t
        while self.echo.value == 0 and time.time() < timeout:
            pulse_start = time.time()
            
        pulse_end = pulse_start
        while self.echo.value == 1 and time.time() < timeout:
            pulse_end = time.time()
            
        if pulse_end > pulse_start:
            duration = pulse_end - pulse_start
            return round((duration * 34300) / 2, 1)
        return -1.0
        
    def close(self):
        self.trig.close()
        self.echo.close()


class SensorArray:
    """
    The Ultimate Hardware Interface.
    Communicates with I2C (Gyro, Laser, Temp) and GPIO (Ultrasonic, IR).
    """
    def __init__(self):
        print("Initializing All Sensors...")
        
        # (Removed duplicate Start Button on GPIO 25 - it is properly initialized on GPIO 17 later)
            
        # 2. Dual IR Line Sensors (Left=GPIO 4, Right=GPIO 19)
        try:
            self.ir_left = DigitalInputDevice(4)
            self.ir_right = DigitalInputDevice(19)
            self.ir_active = True
        except Exception as e:
            print(f"IR Sensor Error: {e}")
            self.ir_active = False

        # 2. Ultrasonic Sensors (Using Custom Pi 5 Bypass)
        try:
            self.us_fl = RawUltrasonic(5, 6)
            self.us_fr = RawUltrasonic(16, 26)
            self.us_bl = RawUltrasonic(20, 21)
            self.us_br = RawUltrasonic(14, 15)
            self.ultrasonics_active = True
        except Exception as e:
            print(f"Ultrasonic Error: {e}")
            self.ultrasonics_active = False

        # --- I2C SENSORS ---
        # 3. Initialize MPU6050 Gyroscope
        try:
            import mpu6050
            self.gyro = mpu6050.mpu6050(0x68)
            self.gyro_active = True
            print("  ? Gyroscope OK")
        except Exception as e:
            print(f"  ?? Gyroscope Error: {e}")
            self.gyro_active = False

        # 4. Initialize Front Laser (Single Laser Setup)
        try:
            self.vl53 = vl53l5cx.VL53L5CX()
            self.vl53.init()
            self.vl53.set_resolution(8 * 8)
            self.vl53.start_ranging()
            self.laser_active = True
            print("  ✅ Front Laser initialized at 0x29!")
        except Exception as e:
            print(f"  ⚠️ Front Laser Error: {e}")
            self.laser_active = False

        self.vl53_back_active = False

        # 5. Initialize Referee Start Button (Pin 17)
        try:
            self.start_button = Button(17, pull_up=True)
            self.start_active = True
        except Exception as e:
            print(f"⚠️ Start Button Error: {e}")
            self.start_active = False

        # 6. Initialize 5-Channel Flame Array
        try:
            self.flame_array = [
                DigitalInputDevice(7, pull_up=None, active_state=True),  # D1 Far Left
                DigitalInputDevice(8, pull_up=None, active_state=True),  # D2 Mid Left
                DigitalInputDevice(9, pull_up=None, active_state=True),  # D3 Center
                DigitalInputDevice(11, pull_up=None, active_state=True), # D4 Mid Right
                DigitalInputDevice(10, pull_up=None, active_state=True)  # D5 Far Right (SPI disabled, so it works!)
            ]
            self.flame_array_active = True
        except Exception as e:
            print(f"⚠️ 5-Channel Flame Error: {e}")
            self.flame_array_active = False

        # 8. Initialize UPS Battery Monitor (Waveshare INA219 at 0x43)
        try:
            import smbus2
            self.ups_bus = smbus2.SMBus(1)
            self.ups_active = True
        except Exception as e:
            self.ups_active = False

        # Cache for I2C sensors so we don't return junk when data isn't ready
        self.last_laser_data = [1000.0] * 64
        self.last_gyro_data = 0.0

    # --- GPIO SENSOR METHODS ---
    def check_black_line(self):
        if not self.ir_active:
            return False, False
        try:
            return (self.ir_left.value == 1, self.ir_right.value == 1)
        except Exception as e:
            print(f"IR Sensor Read Error: {e}")
            return False, False

    def get_ultrasonic_distances(self):
        if not self.ultrasonics_active:
            return [-1.0, -1.0, -1.0, -1.0]
        try:
            return [
                self.us_fl.get_distance_cm() / 100.0, 
                self.us_fr.get_distance_cm() / 100.0, 
                self.us_bl.get_distance_cm() / 100.0, 
                self.us_br.get_distance_cm() / 100.0
            ]
        except Exception as e:
            print(f"Ultrasonic Read Error: {e}")
            return [-1.0, -1.0, -1.0, -1.0]

    # --- I2C SENSOR METHODS ---
    def get_gyro_rotation(self):
        if self.gyro_active:
            try:
                # Apply the Z-axis drift offset (-0.58363 rad/s) discovered during testing
                self.last_gyro_data = self.gyro.get_gyro_data()['z'] - (-0.58363)
            except:
                pass
        return self.last_gyro_data

    def get_tof_distances(self):
        if self.laser_active:
            try:
                if self.vl53.data_ready():
                    # The library returns a 2D ctypes array like c_short_Array_64_Array_1
                    # We must grab the inner array at index [0] before converting to list!
                    raw_data = self.vl53.get_data().distance_mm[0]
                    self.last_laser_data = list(raw_data)
            except Exception as e:
                print(f"Laser Read Error: {e}")
        return self.last_laser_data

    def check_fire_wavelength(self):
        if hasattr(self, 'flame_active') and self.flame_active:
            return self.flame_sensor.value == 1 # 1 = Fire Detected
        return False

    def get_5ch_flame(self):
        if hasattr(self, 'flame_array_active') and self.flame_array_active:
            try:
                # According to hardware diagnostics, pins rest at 0 and flip to 1 when fire is present
                return [ch.value == 1 for ch in self.flame_array]
            except Exception as e:
                print(f"Flame Sensor Read Error: {e}")
                return [False, False, False, False, False]
        return [False, False, False, False, False]
        
    def get_rear_laser(self):
        if hasattr(self, 'vl53_back_active') and self.vl53_back_active:
            try:
                return self.vl53_back.range / 10.0 # Convert mm to cm
            except:
                pass
        return -1.0
        
    def get_ups_voltage(self):
        if hasattr(self, 'ups_active') and self.ups_active:
            try:
                word = self.ups_bus.read_word_data(0x43, 0x02)
                word = ((word & 0xFF) << 8) | (word >> 8)
                return (word >> 3) * 0.004
            except:
                pass
        return 0.0
        
    def shutdown(self):
        """ Closes GPIO sensors to prevent memory leaks on Thonny """
        if hasattr(self, 'laser_active') and self.laser_active:
            try:
                self.vl53.stop_ranging()
            except:
                pass
                
        if hasattr(self, 'flame_active') and self.flame_active:
            self.flame_sensor.close()
            
        if hasattr(self, 'flame_array_active') and self.flame_array_active:
            for ch in self.flame_array:
                ch.close()
                
        if hasattr(self, 'ir_active') and self.ir_active:
            self.ir_left.close()
            self.ir_right.close()
            
        if hasattr(self, 'start_active') and self.start_active:
            self.start_button.close()
            self.us_bl.close()
            self.us_br.close()
