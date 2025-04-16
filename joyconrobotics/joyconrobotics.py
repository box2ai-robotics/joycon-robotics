# code by boxjod 2025.1.13 copyright Box2AI Robotics 盒桥智能 版权所有

import math
import time
from glm import vec3, quat, angleAxis

from .joycon import JoyCon
from .gyro import GyroTrackingJoyCon
from .event import ButtonEventJoyCon
from .device import get_R_id, get_L_id

from scipy.spatial.transform import Rotation as R
import numpy as np
import threading
import logging

JOYCON_SERIAL_SUPPORT = '9c:54:'

class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev_value = 0.0

    def update(self, new_value):
        self.prev_value = self.alpha * new_value + (1 - self.alpha) * self.prev_value
        return self.prev_value
    
class AttitudeEstimator:
    def __init__(self, 
                pitch_Threhold = math.pi/2.0, 
                roll_Threhold = math.pi/2.02, 
                yaw_Threhold = -1, 
                common_rad = True,
                lerobot = False,
                pitch_down_double = False,
                lowpassfilter_alpha_rate = 0.05
                ):
        self.pitch = 0.0 
        self.roll = 0.0   
        self.yaw = 0.0   
        self.dt = 0.01  
        self.alpha = 0.55
        
        self.yaw_diff = 0.0
        self.pitch_rad_T = pitch_Threhold
        self.roll_rad_T = roll_Threhold
        self.yaw_rad_T = yaw_Threhold
        
        self.common_rad = common_rad
        self.lerobot = lerobot
        self.pitch_down_double = pitch_down_double
        
        self.direction_X = vec3(1, 0, 0)
        self.direction_Y = vec3(0, 1, 0)
        self.direction_Z = vec3(0, 0, 1)
        self.direction_Q = quat()
        
        self.lowpassfilter_alpha = 0.05 * lowpassfilter_alpha_rate# lerobot-plus 0.1
        if self.lerobot:
            self.lowpassfilter_alpha = 0.08 * lowpassfilter_alpha_rate
            
        self.lpf_roll = LowPassFilter(alpha=self.lowpassfilter_alpha)   # lerobot real 
        self.lpf_pitch = LowPassFilter(alpha=self.lowpassfilter_alpha)  # lerobot real 
    
    def reset_yaw(self):
        self.direction_X = vec3(1, 0, 0)
        self.direction_Y = vec3(0, 1, 0)
        self.direction_Z = vec3(0, 0, 1)
        self.direction_Q = quat()
    
    def set_yaw_diff(self,data):
        self.yaw_diff = data
        
    def update(self, gyro_in_rad, accel_in_g):
        self.pitch = 0.0 
        self.roll = 0.0   
        
        ax, ay, az = accel_in_g
        ax = ax * math.pi
        ay = ay * math.pi
        az = az * math.pi
        
        gx, gy, gz = gyro_in_rad

        # Calculate the pitch and roll angles provided by the accelerometers
        roll_acc = math.atan2(ay, -az)
        pitch_acc = math.atan2(ax, math.sqrt(ay**2 + az**2))
        
        # Updating angles with gyroscope data
        self.pitch += gy * self.dt
        self.roll -= gx * self.dt

        # Complementary filters: weighted fusion of accelerometer and gyroscope data
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc
        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
        
        # The final output roll and pitch angles are then low-pass filtered
        self.pitch = self.lpf_pitch.update(self.pitch)
        self.roll = self.lpf_roll.update(self.roll)
        
        # Yaw angle (updated by gyroscope)
        rotation = angleAxis(gx * (-1/86), self.direction_X) \
            * angleAxis(gy * (-1/86), self.direction_Y) \
            * angleAxis(gz * (-1/86), self.direction_Z)

        self.direction_X *= rotation
        self.direction_Y *= rotation
        self.direction_Z *= rotation
        self.direction_Q *= rotation        
        
        self.yaw = self.direction_X[1]
        
        if self.common_rad:
            self.roll = self.roll * math.pi/1.5
            self.pitch = self.pitch * math.pi/1.5
            self.yaw = -self.yaw * math.pi/1.5 # * 10.0
            
        else:
            self.yaw = -self.yaw * math.pi/2  
            
        if self.pitch_down_double:
            self.pitch = self.pitch * 3.0 if self.pitch < 0 else self.pitch
        if self.lerobot:
            self.roll = self.roll * math.pi/2
            # self.yaw = -self.yaw * math.pi/1.5 # * 10.0      
        self.yaw = self.yaw - self.yaw_diff    
        
        if self.pitch_rad_T != -1:
            self.pitch = self.pitch_rad_T if self.pitch > self.pitch_rad_T else (-self.pitch_rad_T if self.pitch < -self.pitch_rad_T else self.pitch) 
        
        if self.roll_rad_T != -1:
            self.roll = self.roll_rad_T if self.roll > self.roll_rad_T else (-self.roll_rad_T if self.roll < -self.roll_rad_T else self.roll) 
        
        if self.yaw_rad_T != -1:
            self.yaw = self.yaw_rad_T if self.yaw > self.yaw_rad_T else (-self.yaw_rad_T if self.yaw < -self.yaw_rad_T else self.yaw) 
        
        orientation = [self.roll, self.pitch, self.yaw]
        # Return roll angle, pitch angle, yaw angle (in radians)
        return orientation


class JoyconRobotics:
    def __init__(self, 
                 device: str = "right", 
                 gripper_open: float = 1.0, 
                 gripper_close: float = 0.0, 
                 gripper_state: float = 1.0,
                 horizontal_stick_mode: str = "y",
                 close_y: bool = False,
                 limit_dof: bool = False,
                 glimit: list = [[0.125, -0.4,  0.046, -3.1, -1.5, -1.57], 
                                 [0.380,  0.4,  0.23,  3.1,  1.5,  1.57]],
                 offset_position_m: list = [0.0, 0.0, 0.0], # just use the position and yaw
                 offset_euler_rad: list = [0.0, 0.0, 0.0], # adjust the orientation
                 euler_reverse: list = [1, 1, 1], # -1 reverse
                 direction_reverse: list = [1, 1, 1], # -1 reverse
                 dof_speed: list = [1,1,1,1,1,1],
                 rotation_filter_alpha_rate = 1,
                 common_rad: bool = True,
                 lerobot: bool = False,
                 pitch_down_double: bool = False,
                 without_rest_init: bool = False,
                 pure_xz: bool = True,
                 change_down_to_gripper: bool = False, # ZR to toggle gripper state is common for lerobot, ARX ARM and VixperX. But for UR, Sawyer and panda you could try this. ZR to go down and stick button to toggle gripper
                 lowpassfilter_alpha_rate = 0.05,
                 ):
        
        if device == "right":
            self.joycon_id = get_R_id()
        elif device == "left":
            self.joycon_id = get_L_id()
        else:
            print("get a wrong device name of joycon")
        device_serial = self.joycon_id[2][:6]
        
        # init joycon
        self.joycon = JoyCon(*self.joycon_id)
        
        self.gyro = GyroTrackingJoyCon(*self.joycon_id)
        self.lerobot = lerobot
        self.pitch_down_double = pitch_down_double
        self.rotation_filter_alpha_rate = rotation_filter_alpha_rate
        self.orientation_sensor = AttitudeEstimator(common_rad=common_rad, lerobot=self.lerobot, pitch_down_double = self.pitch_down_double, lowpassfilter_alpha_rate = self.rotation_filter_alpha_rate)
        self.button = ButtonEventJoyCon(*self.joycon_id, track_sticks=True)
        self.without_rest_init = without_rest_init
        # print(f"connect to {device} joycon successful.")
        
        print(f"\033[32mconnect to {device} joycon successful.\033[0m")
        if not self.without_rest_init:
            self.reset_joycon()
        
        print()
        # more information
        self.gripper_open = gripper_open
        self.gripper_close = gripper_close
        self.gripper_state = gripper_state # 1 for open, 0 for close
        
        self.position = offset_position_m.copy()
        self.orientation_rad = offset_euler_rad.copy()
        self.direction_vector = []
        self.direction_vector_right = []
        self.yaw_diff = 0.0
        
        self.offset_position_m = offset_position_m.copy()
        self.posture = offset_position_m.copy()
        
        self.horizontal_stick_mode = horizontal_stick_mode
        self.if_close_y = close_y
        self.if_limit_dof = limit_dof
        self.dof_speed = dof_speed.copy()
        self.glimit = glimit
        self.offset_euler_rad = offset_euler_rad
        self.euler_reverse = euler_reverse
        self.pure_xz = pure_xz
        self.direction_reverse = direction_reverse
        self.change_down_to_gripper = change_down_to_gripper
        self.gripper_toggle_button = 0
        # Start the thread to read inputs
        
        self.reset_button = 0
        self.next_episode_button = 0
        self.restart_episode_button = 0
        self.button_control = 0
        
        if device_serial != JOYCON_SERIAL_SUPPORT and self.joycon_id != None:
            raise IOError("There is no joycon for robotics")
        
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.solve_loop, daemon=True)
        self.thread.start()
        
    def disconnnect(self):
        self.joycon._close()
    
    def reset_joycon(self):
        
        print(f"\033[33mcalibrating(2 seconds)..., please place it horizontally on the desktop.\033[0m")

        self.gyro.calibrate()
        time.sleep(2)
        self.gyro.reset_orientation
        self.orientation_sensor.reset_yaw()

        print(f"\033[32mJoycon calibrations is complete.\033[0m")
    
    def check_limits_position(self):
        for i in range(3):
            self.position[i] = self.glimit[0][i] if self.position[i] < self.glimit[0][i] else (self.glimit[1][i] if self.position[i] > self.glimit[1][i] else self.position[i])
    
    def check_limits_orientation(self):
        for i in range(3):
            self.orientation_rad[i] = self.glimit[0][3+i] if self.orientation_rad[i] < self.glimit[0][3+i] else (self.glimit[1][3+i] if self.orientation_rad[i] > self.glimit[1][3+i] else self.orientation_rad[i])
    
    def common_update(self):
        # Forward and Backward movement
        joycon_stick_v = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        if joycon_stick_v > 4000:
            self.position[0] += 0.001 * self.direction_vector[0] * self.dof_speed[0] * self.direction_reverse[0]
            self.position[2] += 0.001 * self.direction_vector[2] * self.dof_speed[2] * self.direction_reverse[2]
            if not self.if_close_y: # recommend for lerobot SO100
                self.position[1] += 0.001 * self.direction_vector[1] * self.dof_speed[1] * self.direction_reverse[1]
        elif joycon_stick_v < 1000:
            self.position[0] -= 0.001 * self.direction_vector[0] * self.dof_speed[0] * self.direction_reverse[0]
            self.position[2] -= 0.001 * self.direction_vector[2] * self.dof_speed[2] * self.direction_reverse[2]
            if not self.if_close_y: # recommend for lerobot SO100
                self.position[1] -= 0.001 * self.direction_vector[1] * self.dof_speed[1] * self.direction_reverse[1]
        
        # Left and right movement
        joycon_stick_h = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_left_horizontal()
        if self.horizontal_stick_mode == "y":
            if joycon_stick_h > 4000:
                self.position[0] -= 0.001 * self.direction_vector_right[0] * self.dof_speed[0] * self.direction_reverse[0]
                self.position[1] -= 0.001 * self.direction_vector_right[1] * self.dof_speed[1] * self.direction_reverse[1]
                self.position[2] -= 0.001 * self.direction_vector_right[2] * self.dof_speed[2] * self.direction_reverse[2]
            elif joycon_stick_h < 1000:
                self.position[0] += 0.001 * self.direction_vector_right[0] * self.dof_speed[0] * self.direction_reverse[0]
                self.position[1] += 0.001 * self.direction_vector_right[1] * self.dof_speed[1] * self.direction_reverse[1]
                self.position[2] += 0.001 * self.direction_vector_right[2] * self.dof_speed[2] * self.direction_reverse[2]
        elif self.horizontal_stick_mode == "yaw_diff": # for lerobot SO100
            if joycon_stick_h > 4000:
                if self.yaw_diff < self.glimit[1][5] / 2.0:
                    self.yaw_diff +=0.02 * self.dof_speed[5] / 2.0
                    self.orientation_sensor.set_yaw_diff(self.yaw_diff)
            elif joycon_stick_h < 1000:
                if self.yaw_diff > self.glimit[0][5] / 2.0:
                    self.yaw_diff -=0.02 * self.dof_speed[5]  / 2.0
                    self.orientation_sensor.set_yaw_diff(self.yaw_diff)
        
        # Up and down movement
        joycon_button_up = self.joycon.get_button_r() if self.joycon.is_right() else self.joycon.get_button_l()
        if joycon_button_up == 1:
            if self.pure_xz:
                self.position[2] += 0.001 * self.dof_speed[2]
            else:
                self.position[0] += 0.001 * self.direction_vector_up[0] * self.dof_speed[0] * self.direction_reverse[0]
                self.position[1] += 0.001 * self.direction_vector_up[1] * self.dof_speed[1] * self.direction_reverse[1]
                self.position[2] += 0.001 * self.direction_vector_up[2] * self.dof_speed[2] * self.direction_reverse[2]
        
        if not self.change_down_to_gripper:
            joycon_button_down = self.joycon.get_button_r_stick() if self.joycon.is_right() else self.joycon.get_button_l_stick()
        else:
            joycon_button_down = self.joycon.get_button_zr() if self.joycon.is_right() else self.joycon.get_button_zl
                        
        if joycon_button_down == 1:
            if self.pure_xz:
                self.position[2] -= 0.001 * self.dof_speed[2]
            else:
                self.position[0] -= 0.001 * self.direction_vector_up[0] * self.dof_speed[0] * self.direction_reverse[0]
                self.position[1] -= 0.001 * self.direction_vector_up[1] * self.dof_speed[1] * self.direction_reverse[1]
                self.position[2] -= 0.001 * self.direction_vector_up[2] * self.dof_speed[2] * self.direction_reverse[2]

        # Common buttons
        joycon_button_xup = self.joycon.get_button_x() if self.joycon.is_right() else self.joycon.get_button_up()
        joycon_button_xback = self.joycon.get_button_b() if self.joycon.is_right() else self.joycon.get_button_down()
        if joycon_button_xup == 1:
            self.position[0] += 0.001 * self.dof_speed[0]
        elif joycon_button_xback == 1:
            self.position[0] -= 0.001 * self.dof_speed[0]
        
        joycon_button_home = self.joycon.get_button_home() if self.joycon.is_right() else self.joycon.get_button_capture()
        if joycon_button_home == 1:
            
            if self.position[0] > self.offset_position_m[0] + 0.002: 
                self.position[0] = self.position[0] - 0.001 * self.dof_speed[0] * 2.0
            elif self.position[0] < self.offset_position_m[0] - 0.002:
                self.position[0] = self.position[0] + 0.001 * self.dof_speed[0] * 2.0
            else:
                self.position[0] = self.position[0]
            
            if self.position[1] > self.offset_position_m[1] + 0.002: 
                self.position[1] = self.position[1] - 0.001 * self.dof_speed[1] * 2.0
            elif self.position[1] < self.offset_position_m[1] - 0.002:
                self.position[1] = self.position[1] + 0.001 * self.dof_speed[1] * 2.0
            else:
                self.position[1] = self.position[1]
            
            if self.position[2] > self.offset_position_m[2] + 0.002: 
                self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
            elif self.position[2] < self.offset_position_m[2] - 0.002:
                self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
            else:
                self.position[2] = self.position[2]
            
            if self.orientation_rad[2] > self.offset_euler_rad[2] + 0.02 : # * self.dof_speed[5]:
                self.yaw_diff = self.yaw_diff + (0.01 * self.dof_speed[5])  
            elif self.orientation_rad[2] < self.offset_euler_rad[2] - 0.02 : # * self.dof_speed[5]:
                self.yaw_diff = self.yaw_diff - (0.01 * self.dof_speed[5])  
            else:
                self.yaw_diff = self.yaw_diff
                
            # print(f'{self.yaw_diff=}')
            self.orientation_sensor.set_yaw_diff(self.yaw_diff)
            
            # print(f'{self.orientation_rad[2]=}')
            if self.orientation_rad[2] < (0.02 * self.dof_speed[5]) and self.orientation_rad[2] > (-0.02* self.dof_speed[5]):
                self.orientation_sensor.reset_yaw()# gyro.reset_orientation()
                self.yaw_diff = 0.0
                self.orientation_sensor.set_yaw_diff(self.yaw_diff)

        
        # gripper 
        for event_type, status in self.button.events():
            if (self.joycon.is_right() and event_type == 'plus' and status == 1) or (self.joycon.is_left() and event_type == 'minus' and status == 1):
                self.reset_button = 1
                self.reset_joycon()
            elif self.joycon.is_right() and event_type == 'a':
                self.next_episode_button = status
            elif self.joycon.is_right() and event_type == 'y':
                self.restart_episode_button = status
            elif ((self.joycon.is_right() and event_type == 'zr') or (self.joycon.is_left() and event_type == 'zl')) and not self.change_down_to_gripper:
                self.gripper_toggle_button = status
            elif ((self.joycon.is_right() and event_type == 'stick_r_btn') or (self.joycon.is_left() and event_type == 'stick_l_btn')) and self.change_down_to_gripper:
                self.gripper_toggle_button = status
            # print(f'{event_type=}, {status=}')
            else: 
                self.reset_button = 0
            
        if self.gripper_toggle_button == 1 :
            if self.gripper_state == self.gripper_open:
                self.gripper_state = self.gripper_close
            else:
                self.gripper_state = self.gripper_open
            self.gripper_toggle_button = 0

        # record
        if self.joycon.is_right():
            if self.next_episode_button == 1:
                self.button_control = 1
            elif self.restart_episode_button == 1:
                self.button_control = -1
            elif self.reset_button == 1:
                self.button_control = 8
            else:
                self.button_control = 0
        
        return self.position, self.gripper_state, self.button_control
                        
                        
    def get_orientation(self): # euler_rad, euler_deg, quaternion,
        self.orientation_rad = self.orientation_sensor.update(self.gyro.gyro_in_rad[0], self.gyro.accel_in_g[0])
        
        # roll, pitch, yaw = self.orientation_rad
        # self.direction_vector_right = vec3(math.cos(roll) * math.cos(yaw + math.pi/2 * self.euler_reverse[2]), math.cos(roll) * math.sin(yaw + math.pi/2 * self.euler_reverse[2]), math.sin(roll))
        # self.direction_vector_up = vec3(math.cos(-roll + math.pi/2 * self.euler_reverse[0]) * math.cos(pitch + math.pi/2 * self.euler_reverse[2]), 
        #                                 math.cos(-roll + math.pi/2 * self.euler_reverse[0]) * math.sin(pitch + math.pi/2 * self.euler_reverse[2]), 
        #                                 math.sin(-roll + math.pi/2 * self.euler_reverse[0]))
        
        for i in range(3): # deal with offset and reverse
            self.orientation_rad[i] = (self.orientation_rad[i] + self.offset_euler_rad[i]) * self.euler_reverse[i]
            
        roll, pitch, yaw = self.orientation_rad
        # self.direction_vector = vec3(math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
        self.direction_vector = vec3(math.cos(pitch) * math.cos(yaw), 
                                     math.cos(pitch) * math.sin(yaw), 
                                     math.sin(pitch))
                                     
        self.direction_vector_right = vec3(math.cos(roll) * math.sin(yaw), 
                                           math.cos(roll) * math.cos(yaw), 
                                           math.sin(roll))
        
        self.direction_vector_up = vec3(math.sin(-roll) * math.sin(-pitch), 
                                        math.sin(-roll) * math.cos(-pitch), 
                                        math.cos(-roll))
            
        if self.if_limit_dof:
            self.check_limits_orientation()

        return self.orientation_rad
    
    def update(self):
        roll, pitch, yaw = self.get_orientation()
        self.position, gripper, button_control = self.common_update()

        if self.if_limit_dof:
            self.check_limits_position()
            
        x,y,z = self.position
        self.posture = [x,y,z,roll, pitch, yaw]
        
        return self.posture, gripper, button_control
    
    def solve_loop(self):
        while self.running:
            try:
                self.update()
                # print("solve successful")
                time.sleep(0.01)
            except Exception as e:
                logging.error(f"Error solve_loop from device: {e}")
                time.sleep(1)  # Wait before retrying
                
    def get_control(self, out_format="euler_rad"):
        if out_format == "euler_deg":
            orientation_output = np.rad2deg(self.orientation_rad)
        elif out_format == "quaternion":
            r4 = R.from_euler('xyz', self.orientation_rad, degrees=False)
            orientation_output = r4.as_quat()
        else:
            orientation_output = self.orientation_rad
            
        roll, pitch, yaw = orientation_output
        x,y,z = self.position
            
        self.posture = [x,y,z,roll, pitch, yaw]
        # print(f'{self.posture=}')
        return self.posture, self.gripper_state, self.button_control
            
    
    # More information
    def get_stick(self):
        stick_vertical = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        stick_horizontal = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_right_horizontal()
        stick_button = self.joycon.get_button_r_stick() if self.joycon.is_right() else self.joycon.get_button_l_stick()
        
        return stick_vertical, stick_horizontal, stick_button
    
    def listen_button(self, button, show_all=False): 
        # the button names: 
        # right: r, zr, y, x, a, b, plus, r-stick, home, sr, sl
        # left: l, zl, left, up, right, down, minis, r-stick, capture, sr, sl
        
        for event_type, status in self.button.events():
            if show_all == True:
                print(event_type, status)
                
            if event_type == button:
                return status
                
        return None
    
    def set_position(self, set_position):
        # self.x, self.y, self.z = set_position
        self.position = set_position
        print('set position complect.')
        
    def close_horizontal_stick(self):
        self.close_horizontal_stick = 'close'
        return
    
    def close_y(self):
        self.close_y = True
        return
    
    def open_horizontal(self):
        self.close_horizontal_stick = True
        return
    
    def close_y(self):
        self.close_y = True
        return
    
    def set_gripper_close_value(self, gripper_close):
        self.gripper_close = gripper_close
        return
    
    def set_gripper_open_value(self, gripper_open):
        self.gripper_open = gripper_open
        return
    
    def open_gripper(self):
        self.gripper_state = self.gripper_open
        return
    
    def close_gripper(self):
        self.gripper_state = self.gripper_close
        return
    
    def set_posture_limits(self, glimit):
        # glimit = [[x_min, y_min, z_min, roll_min, pitch_min, yaw_min]
        #           [x_max, y_max, z_max, roll_max, pitch_max, yaw_max]]
        # such as glimit = [[0.000, -0.4,  0.046, -3.1, -1.5, -1.5], 
        #                   [0.430,  0.4,  0.23,  3.1,  1.5,  1.5]]
        self.glimit = glimit
        return
    
    def set_dof_speed(self, dof_speed):
        # glimit = [x_speed, y_speed, z_speed, _, _, yaw_speed]
        self.dof_speed = dof_speed
        return
    
    
    
    
    
    
    