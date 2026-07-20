import math
import time
import numpy as np

class SLAM_Engine:
    """
    Tier 1: Simultaneous Localization and Mapping (SLAM)
    The Memory and Senses of the Robot.
    
    Because we do not have wheel encoders, we use "Dead Reckoning" combined with the MPU6050 Gyroscope.
    This calculates exactly where the robot is on the competition floor (X, Y) and what angle it is facing (Theta).
    """
    
    def __init__(self):
        # 1. The Robot's exact physical location in the real world
        # We assume the robot starts at coordinate (0, 0) facing 0 degrees (North)
        self.x = 0.0      # X coordinate in meters
        self.y = 0.0      # Y coordinate in meters
        self.theta = 0.0  # Angle in radians (0 is North, PI/2 is East)
        
        self.last_time = time.time()
        
        # 2. The Occupancy Grid (The 2D Map)
        # 100x100 grid representing a 5x5 meter arena (5cm resolution per cell)
        self.map_resolution = 0.05 
        self.grid_size = 100
        self.map = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        
    def update_odometry(self, gyro_z_velocity, left_motor_pwm, right_motor_pwm):
        """
        Updates the robot's (X, Y, Theta) based on how fast the motors are spinning and the Gyroscope data.
        Call this function 10 times a second!
        """
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01
            
        # 1. Update Theta (Heading) using the highly accurate MPU6050 Gyroscope!
        # gyro_z_velocity is how fast the robot is twisting in Radians/Second
        self.theta += gyro_z_velocity * dt
        
        # Keep Theta between -PI and +PI (Standard robotic heading)
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # 2. Dead Reckoning: Guess how fast we are moving based on motor PWM
        # Assuming 1.0 PWM = 0.68 meters/second (Max speed of TT motors on 12V)
        max_speed_mps = 0.68
        
        # Average speed of the two wheels is our forward velocity
        forward_velocity = ((left_motor_pwm + right_motor_pwm) / 2.0) * max_speed_mps
        
        # 3. Calculate new X and Y using Trigonometry
        distance_traveled = forward_velocity * dt
        
        self.x += distance_traveled * math.cos(self.theta)
        self.y += distance_traveled * math.sin(self.theta)
        
        self.last_time = current_time
        
        return self.x, self.y, self.theta
        
    def update_map(self, tof_front_dist, tof_left_dist, tof_right_dist):
        """
        Takes the VL53L5CX Laser distances and draws virtual "walls" on our 2D grid map.
        This allows the RL Brain to 'see' the walls and plot a path around them.
        """
        # We calculate the absolute X,Y coordinate of the wall by projecting the laser distance 
        # outward from the robot's current X,Y,Theta position.
        
        # Front Laser
        if tof_front_dist < 2.0: # If wall is within 2 meters
            wall_x = self.x + (tof_front_dist * math.cos(self.theta))
            wall_y = self.y + (tof_front_dist * math.sin(self.theta))
            self._mark_wall_on_grid(wall_x, wall_y)
            
        # Left Laser (assuming mounted 30 degrees to the left)
        if tof_left_dist < 2.0:
            left_angle = self.theta + math.radians(30)
            wall_x = self.x + (tof_left_dist * math.cos(left_angle))
            wall_y = self.y + (tof_left_dist * math.sin(left_angle))
            self._mark_wall_on_grid(wall_x, wall_y)
            
        # Right Laser (assuming mounted 30 degrees to the right)
        if tof_right_dist < 2.0:
            right_angle = self.theta - math.radians(30)
            wall_x = self.x + (tof_right_dist * math.cos(right_angle))
            wall_y = self.y + (tof_right_dist * math.sin(right_angle))
            self._mark_wall_on_grid(wall_x, wall_y)
            
    def _mark_wall_on_grid(self, wall_x, wall_y):
        """ Translates real-world meters into a Grid Array Index """
        # Shift the map center so (0,0) is in the middle of the 100x100 grid (Index 50,50)
        grid_x = int((wall_x / self.map_resolution) + (self.grid_size / 2))
        grid_y = int((wall_y / self.map_resolution) + (self.grid_size / 2))
        
        # If it fits on the map, draw a wall! (100 = Obstacle)
        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            self.map[grid_y][grid_x] = 100 
            
    def get_pose(self):
        return self.x, self.y, self.theta
