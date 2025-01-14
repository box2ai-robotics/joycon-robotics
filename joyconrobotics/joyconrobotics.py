# code by boxjod 2025.1.13 copyright Box2AI Robotics 盒桥智能 版权所有

import math
import time
from glm import vec2, vec3, quat, angleAxis, eulerAngles

from .joycon import JoyCon
from .wrappers import PythonicJoyCon  # as JoyCon
from .gyro import GyroTrackingJoyCon
from .event import ButtonEventJoyCon
from .device import get_device_ids, get_ids_of_type
from .device import is_id_L
from .device import get_R_ids, get_L_ids
from .device import get_R_id, get_L_id

from scipy.spatial.transform import Rotation as R
import numpy as np

class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev_value = 0.0

    def update(self, new_value):
        self.prev_value = self.alpha * new_value + (1 - self.alpha) * self.prev_value
        return self.prev_value
    
class AttitudeEstimator:
    def __init__(self, 
                pitch_Threhold = math.pi/2, 
                yaw_Threhold = -1, 
                common_rad = True,
                lerobot = False
                ):
        self.pitch = 0.0 
        self.roll = 0.0   
        self.yaw = 0.0   
        self.dt = 0.001  
        self.alpha = 0.5 
        
        self.yaw_diff = 0.0
        self.pitch_rad_T = pitch_Threhold
        self.yaw_rad_T = yaw_Threhold
        self.common_rad = common_rad
        self.lerobot = lerobot
        
        self.direction_X = vec3(1, 0, 0)
        self.direction_Y = vec3(0, 1, 0)
        self.direction_Z = vec3(0, 0, 1)
        self.direction_Q = quat()
        
        self.lpf_roll = LowPassFilter(alpha=0.50)   
        self.lpf_pitch = LowPassFilter(alpha=0.50)  
    
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
        self.yaw = self.yaw + self.yaw_diff    
        
        if self.common_rad:
            self.roll = self.roll * math.pi/1.5
            self.pitch = self.pitch * math.pi/1.5
            self.yaw = -self.yaw * math.pi/1.8 * 10.0
            
        else:
            self.yaw = -self.yaw * math.pi/2  
            if self.lerobot:
                self.pitch = self.pitch * 3.0 if self.pitch < 0 else self.pitch
                self.roll = self.roll * math.pi
            
        if self.pitch_rad_T != -1:
            self.pitch = self.pitch_rad_T if self.pitch > self.pitch_rad_T else (-self.pitch_rad_T if self.pitch < -self.pitch_rad_T else self.pitch) 
        
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
                 with_calibrate: bool = True, 
                 horizontal_stick_mode: str = "y",
                 close_y: bool = False,
                 limit_dof: bool = False,
                 init_gpos: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 dof_speed: list = [1,1,1,1,1,1],
                 common_rad: bool = True,
                 lerobot: bool = False):
        if device == "right":
            joycon_id = get_R_id()
        elif device == "left":
            joycon_id = get_L_id()
        else:
            print("get a wrong device name of joycon")
        self.joycon = JoyCon(*joycon_id)
        print(f"detect {device} {joycon_id=}")
        self.gyro = GyroTrackingJoyCon(*joycon_id)
        self.lerobot = lerobot
        self.orientation_sensor = AttitudeEstimator(common_rad=common_rad, lerobot=self.lerobot)
        self.button = ButtonEventJoyCon(*joycon_id)
        print(f"connect to {device} complete.")
        print()
        if with_calibrate:
            self.reset_joycon()
        
        # information
        self.gripper_open = gripper_open
        self.gripper_close = gripper_close
        self.gripper_state = 1 # 1 for open, 0 for close
        
        self.position = init_gpos[0:3].copy()
        self.orientation_rad = init_gpos[3:6].copy()
        self.direction_vector = []
        self.yaw_diff = 0.0
        
        self.init_gpos = init_gpos.copy()
        self.posture = init_gpos.copy()
        
        self.horizontal_stick_mode = horizontal_stick_mode
        self.if_close_y = close_y
        self.if_limit_dof = limit_dof
        self.dof_speed = dof_speed.copy()
        self.glimit = [[0.000, -0.4,  0.046, -3.1, -1.5, -1.5], 
                  [0.430,  0.4,  0.25,  3.1,  1.5,  1.5]]
        
        
    def reset_joycon(self):
        print('waiting for calibrations, please place it horizontally on the desktop.')
        print('it will takes 4s...')
    
        self.gyro.calibrate()
        self.gyro.reset_orientation
        self.orientation_sensor.reset_yaw()
        time.sleep(2)
        self.gyro.calibrate()
        self.gyro.reset_orientation
        self.orientation_sensor.reset_yaw()
        time.sleep(2)
        
        print('Joycon calibrations is complete.')
    
    def check_limits_position(self):
        for i in range(3):
            self.position[i] = self.glimit[0][i] if self.position[i] < self.glimit[0][i] else (self.glimit[1][i] if self.position[i] > self.glimit[1][i] else self.position[i])
    
    def check_limits_orientation(self):
        for i in range(3):
            self.orientation_rad[i] = self.glimit[0][3+i] if self.orientation_rad[i] < self.glimit[0][3+i] else (self.glimit[1][3+i] if self.orientation_rad[i] > self.glimit[1][3+i] else self.orientation_rad[i])
    
    def common_update(self):
        joycon_stick_v = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        if joycon_stick_v > 4000:
            # Forward movement: 0.1 speed in the direction of the direction vector.
            self.position[0] += 0.002 * self.direction_vector[0] * self.dof_speed[0]
            if not self.if_close_y:
                self.position[1] += 0.002 * self.direction_vector[1] * self.dof_speed[1]
            self.position[2] += 0.002 * self.direction_vector[2] * self.dof_speed[2]     
        elif joycon_stick_v < 1000:
            # Backward movement: 0.1 speed in the opposite direction of the direction vector.
            self.position[0] -= 0.002 * self.direction_vector[0] * self.dof_speed[0]
            if not self.if_close_y:
                self.position[1] -= 0.002 * self.direction_vector[1] * self.dof_speed[1]
            self.position[2] -= 0.002 * self.direction_vector[2] * self.dof_speed[2]
        
        joycon_stick_h = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_right_horizontal()
        rotation_matrix_h = np.array([[0, -1, 0], [1, 0, 0],[0, 0, 1]])
        direction_vector_h = np.dot(rotation_matrix_h, self.direction_vector)
        
        if self.horizontal_stick_mode == "y":
            if joycon_stick_h > 4000:
                self.position[0] += 0.002 * direction_vector_h[0] * self.dof_speed[0]
                self.position[1] += 0.002 * direction_vector_h[1] * self.dof_speed[1]
                self.position[2] += 0.002 * direction_vector_h[2] * self.dof_speed[2]   
            elif joycon_stick_h < 1000:
                self.position[0] -= 0.002 * direction_vector_h[0] * self.dof_speed[1]
                self.position[1] -= 0.002 * direction_vector_h[1] * self.dof_speed[2]
                self.position[2] -= 0.002 * direction_vector_h[2] * self.dof_speed[3]
        elif self.horizontal_stick_mode == "yaw_diff":
            if joycon_stick_h > 4000:
                if self.yaw_diff < self.glimit[1][5] / 2.0:
                    self.yaw_diff +=0.02 * self.dof_speed[5] / 2.0
                    self.orientation_sensor.set_yaw_diff(self.yaw_diff)
            elif joycon_stick_h < 1000:
                if self.yaw_diff > self.glimit[0][5] / 2.0:
                    self.yaw_diff -=0.02 * self.dof_speed[5]  / 2.0
                    self.orientation_sensor.set_yaw_diff(self.yaw_diff)
        
        # Common buttons
        joycon_button_up = self.joycon.get_button_r() if self.joycon.is_right() else self.joycon.get_button_l()
        if joycon_button_up == 1:
            self.position[2] += 0.0005
            
        joycon_button_down = self.joycon.get_button_r_stick() if self.joycon.is_right() else self.joycon.get_button_l_stick()
        if joycon_button_down == 1:
            self.position[2] -= 0.0005
        
        joycon_button_xup = self.joycon.get_button_x() if self.joycon.is_right() else self.joycon.get_button_up()
        joycon_button_xback = self.joycon.get_button_b() if self.joycon.is_right() else self.joycon.get_button_down()
        if joycon_button_xup == 1:
            self.position[0] += 0.0005
        elif joycon_button_xback == 1:
            self.position[0] -= 0.0005 
        
        joycon_button_home = self.joycon.get_button_home() if self.joycon.is_right() else self.joycon.get_button_capture()
        if joycon_button_home == 1:
            
            # print(f'{self.position=}')
            # print("position:", [f"{x:.3f}" for x in self.position])
            # print("init_gpos:", [f"{x:.3f}" for x in self.init_gpos])
            
            if self.position[0] > self.init_gpos[0] + 0.002: 
                self.position[0] = self.position[0] - 0.002 * self.dof_speed[0] * 2.0
            elif self.position[0] < self.init_gpos[0] - 0.002:
                self.position[0] = self.position[0] + 0.002 * self.dof_speed[0] * 2.0
            else:
                self.position[0] = self.position[0]
            
            if self.position[1] > self.init_gpos[1] + 0.002: 
                self.position[1] = self.position[1] - 0.002 * self.dof_speed[1] * 2.0
            elif self.position[1] < self.init_gpos[1] - 0.002:
                self.position[1] = self.position[1] + 0.002 * self.dof_speed[1] * 2.0
            else:
                self.position[1] = self.position[1]
            
            if self.position[2] > self.init_gpos[2] + 0.002: 
                self.position[2] = self.position[2] - 0.002 * self.dof_speed[2] * 2.0
            elif self.position[2] < self.init_gpos[2] - 0.002:
                self.position[2] = self.position[2] + 0.002 * self.dof_speed[2] * 2.0
            else:
                self.position[2] = self.position[2]
            
            # self.position[1] = self.position[1] - 0.002 if self.position[1] > self.init_gpos[1] + 0.002 else (self.position[1] + 0.002 if self.position[1] < self.init_gpos[1] - 0.002 else self.position[1])
            # self.position[2] = self.position[2] - 0.002 if self.position[2] > self.init_gpos[2] + 0.002 else (self.position[2] + 0.002 if self.position[2] < self.init_gpos[2] - 0.002 else self.position[2])
            
            
            if self.orientation_rad[2] > self.init_gpos[5] + 0.04:
                self.yaw_diff = self.yaw_diff + (0.02 * self.dof_speed[5])  
            elif self.orientation_rad[2] < self.init_gpos[5] - 0.04:
                self.yaw_diff = self.yaw_diff - (0.02 * self.dof_speed[5])  
            else:
                self.yaw_diff = self.yaw_diff
                
            print(f'{self.yaw_diff=}')
            self.orientation_sensor.set_yaw_diff(self.yaw_diff)
            
            print(f'{self.orientation_rad[2]=}')
            if self.orientation_rad[2] <( 0.04* self.dof_speed[5]) and self.orientation_rad[2] > (-0.04* self.dof_speed[5]):
                self.gyro.reset_orientation()
                self.yaw_diff = 0.0

        
        # gripper 
        for event_type, status in self.button.events():
            if (self.joycon.is_right() and event_type == 'plus' and status == 1) or (self.joycon.is_left() and event_type == 'minus' and status == 1):
                self.reset_joycon()
            elif (self.joycon.is_right() and event_type == 'zr') or (self.joycon.is_left() and event_type == 'zl'):
                if status == 1:
                    if self.gripper_state == 1:
                        self.gripper_state = self.gripper_close
                    else:
                        self.gripper_state = self.gripper_open
        
        return self.position, self.gripper_state
                        
                        
    def get_orientation(self, out_format="euler_rad"): # euler_rad, euler_deg, quaternion,
        self.orientation_rad = self.orientation_sensor.update(self.gyro.gyro_in_rad[0], self.gyro.accel_in_g[0])
        
        if self.if_limit_dof:
            self.check_limits_orientation()
        
        roll, pitch, yaw = self.orientation_rad
        
        self.direction_vector = vec3(math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
        
        if out_format == "euler_deg":
            orientation_output = np.rad2deg(self.orientation_rad)
        elif out_format == "quaternion":
            r4 = R.from_euler('xyz', self.orientation_rad, degrees=False)
            orientation_output = r4.as_quat()
        else:
            orientation_output = self.orientation_rad
        
        return orientation_output

    def update(self, out_format="euler_rad"):
        roll, pitch, yaw = self.get_orientation(out_format=out_format)
        self.position, gripper = self.common_update()
        
        # self.orientation_rad = [roll, pitch, yaw]
        
        if self.if_limit_dof:
            self.check_limits_position()
            
        x,y,z = self.position
        self.posture = [x,y,z,roll, pitch, yaw]
        
        return self.posture, gripper
    
    
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
        set_position = set_position
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
    
    
    
    
    
    
    