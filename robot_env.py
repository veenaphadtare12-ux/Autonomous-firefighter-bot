import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random

class RobotexEnv(gym.Env):
    """
    ULTIMATE Robotex Girls Firefighting Simulator
    - Scoring rules perfectly matched (1000 total points + time bonus).
    - 180 Second max time (1800 steps).
    - Ultrasonics mounted on the 4 corners (±45 degrees).
    - Rubber tires on 2kg robot (0.68m/s).
    """
    metadata = {'render_modes': ['console']}

    def __init__(self):
        super(RobotexEnv, self).__init__()
        self.action_space = spaces.Discrete(3)
        
        # [YOLO_Offset, YOLO_Dist, ToF_L, ToF_C, ToF_R, US_FL, US_FR, US_BL, US_BR]
        self.observation_space = spaces.Box(
            low=np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), 
            dtype=np.float32
        )
        
        self.robot_radius = 0.14 # 20x20cm square means corner radius is ~14cm
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0 
        
        self.candles = []
        self.candle_scores = []
        self.walls = []
        
        self.steps = 0
        self.max_steps = 1800 # 180 seconds @ 10 frames per second
        self.candles_extinguished = [False, False, False, False]
        self.total_score = 0

    def raycast(self, origin, angle, max_range=4.0):
        min_dist = max_range
        ox, oy = origin
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        for (wx1, wy1), (wx2, wy2) in self.walls:
            x1, y1 = ox, oy
            x2, y2 = ox + dx * max_range, oy + dy * max_range
            x3, y3 = wx1, wy1
            x4, y4 = wx2, wy2
            
            den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
            if den == 0: continue
            
            t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
            u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / den
            
            if 0 <= t <= 1 and 0 <= u <= 1:
                dist = t * max_range
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def get_observation(self):
        # 1. VL53L5CX (Front)
        tof_left = self.raycast((self.robot_x, self.robot_y), self.robot_theta + math.radians(30), max_range=4.0)
        tof_center = self.raycast((self.robot_x, self.robot_y), self.robot_theta, max_range=4.0)
        tof_right = self.raycast((self.robot_x, self.robot_y), self.robot_theta - math.radians(30), max_range=4.0)
        
        # 2. Ultrasonics (Mounted on the 4 corners: ±45 deg front, ±135 deg back)
        us_fl = self.raycast((self.robot_x, self.robot_y), self.robot_theta + math.radians(45), max_range=2.0)
        us_fr = self.raycast((self.robot_x, self.robot_y), self.robot_theta - math.radians(45), max_range=2.0)
        us_bl = self.raycast((self.robot_x, self.robot_y), self.robot_theta + math.radians(135), max_range=2.0)
        us_br = self.raycast((self.robot_x, self.robot_y), self.robot_theta - math.radians(135), max_range=2.0)
        
        # 3. YOLO Camera (Center mounted, 60 deg FOV)
        yolo_offset = 0.0
        yolo_dist = 1.0
        
        for i, (cx, cy) in enumerate(self.candles):
            if self.candles_extinguished[i]: continue 
            
            dist = math.hypot(cx - self.robot_x, cy - self.robot_y)
            angle_to_candle = math.atan2(cy - self.robot_y, cx - self.robot_x)
            diff = (angle_to_candle - self.robot_theta + math.pi) % (2*math.pi) - math.pi
            
            if abs(diff) < math.radians(30):
                wall_dist = self.raycast((self.robot_x, self.robot_y), angle_to_candle, max_range=4.0)
                if wall_dist >= dist:
                    yolo_offset = diff / math.radians(30)
                    yolo_dist = dist / 4.0
                    break 
                    
        obs = [yolo_offset, yolo_dist, tof_left/4.0, tof_center/4.0, tof_right/4.0, 
               us_fl/2.0, us_fr/2.0, us_bl/2.0, us_br/2.0]
        return np.array(obs, dtype=np.float32)

    def generate_walls_for_candle(self, cx, cy, num_walls):
        radius = 0.2 
        base_angle = random.uniform(0, math.pi*2)
        for i in range(num_walls):
            angle = base_angle + (i * math.pi/2)
            wall_width = random.uniform(0.2, 0.35)
            wx = cx + math.cos(angle) * radius
            wy = cy + math.sin(angle) * radius
            perp_angle = angle + math.pi/2
            half_width = wall_width / 2.0
            x1 = wx + math.cos(perp_angle) * half_width
            y1 = wy + math.sin(perp_angle) * half_width
            x2 = wx - math.cos(perp_angle) * half_width
            y2 = wy - math.sin(perp_angle) * half_width
            self.walls.append(((x1,y1), (x2,y2)))

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        self.total_score = 0
        self.candles_extinguished = [False, False, False, False]
        self.walls = [
            ((0,0), (2.5,0)), ((2.5,0), (2.5,3.5)), 
            ((2.5,3.5), (0,3.5)), ((0,3.5), (0,0))
        ]
        
        quadrants = [(0.6, 0.8), (1.9, 0.8), (0.6, 2.7), (1.9, 2.7)]
        random.shuffle(quadrants)
        self.candles = []
        self.candle_scores = []
        
        wall_counts = [0, 1, 2, 3]
        scores = [100, 200, 300, 400] # Rule 10 Appendix 1
        
        for i in range(4):
            qx, qy = quadrants[i]
            cx = qx + random.uniform(-0.2, 0.2)
            cy = qy + random.uniform(-0.2, 0.2)
            self.candles.append((cx, cy))
            self.candle_scores.append(scores[i])
            self.generate_walls_for_candle(cx, cy, wall_counts[i])
            
        self.robot_x = 1.25
        self.robot_y = 1.75
        self.robot_theta = random.uniform(0, math.pi*2)
        
        return self.get_observation(), {}

    def step(self, action):
        self.steps += 1
        reward = 0
        
        speed = 0.068 # 0.68 m/s divided by 10 fps
        turn_speed = math.radians(36) # 360 deg/sec divided by 10 fps
        
        if action == 0: 
            self.robot_x += math.cos(self.robot_theta) * speed
            self.robot_y += math.sin(self.robot_theta) * speed
        elif action == 1: 
            self.robot_theta += turn_speed
        elif action == 2: 
            self.robot_theta -= turn_speed
            
        obs = self.get_observation()
        terminated = False
        truncated = self.steps >= self.max_steps
        
        # Boundary Collision (Radius 14cm to corners)
        if min(obs[5:9]) * 2.0 < self.robot_radius:
            reward = -50
            terminated = True
            
        # Extinguishing Logic (Must enter 400mm circle = 200mm radius = 0.2m)
        for i, (cx, cy) in enumerate(self.candles):
            if not self.candles_extinguished[i]:
                dist = math.hypot(cx - self.robot_x, cy - self.robot_y)
                if dist < 0.2:
                    points = self.candle_scores[i]
                    reward = points
                    self.total_score += points
                    self.candles_extinguished[i] = True
                    
                    # Rule: Extinguished candles become obstacles (Add a tiny digital wall)
                    self.walls.append(((cx-0.05, cy), (cx+0.05, cy)))
                    
                    if all(self.candles_extinguished):
                        # Rule 10 Appendix 1: Unused seconds added to score
                        unused_seconds = (self.max_steps - self.steps) / 10.0
                        reward += unused_seconds
                        terminated = True
                    break
            
        return obs, reward, terminated, truncated, {}
