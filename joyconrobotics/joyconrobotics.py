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
    def __init__(self):
        self.pitch = 0.0 
        self.roll = 0.0   
        self.yaw = 0.0   
        self.dt = 0.001  
        self.alpha = 0.5 
        
        self.yaw_diff = 0.0
        
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
        
        self.roll = self.roll * math.pi/1.5
        self.pitch = self.pitch * math.pi/1.5
        self.yaw = -self.direction_X[1] * math.pi/2 * 10 
        self.yaw = self.yaw + self.yaw_diff
        
        orientation = [self.roll, self.pitch, self.yaw]
        # Return roll angle, pitch angle, yaw angle (in radians)
        return orientation


class JoyconRobotics:
    def __init__(self, device: str = "right", gripper_open: float = 1.0, gripper_close: float = 0.0, with_calibrate: bool = True):
        if device == "right":
            joycon_id = get_R_id()
        elif device == "left":
            joycon_id = get_L_id()
        else:
            print("get a wrong device name of joycon")
        self.joycon = JoyCon(*joycon_id)
        print(f"detect {device} {joycon_id=}")
        self.gyro = GyroTrackingJoyCon(*joycon_id)
        self.orientation_sensor = AttitudeEstimator()
        self.button = ButtonEventJoyCon(*joycon_id)
        print(f"connect to {device} complete.")
        print()
        if with_calibrate:
            self.reset_joycon()
        
        # information
        self.gripper_open = gripper_open
        self.gripper_close = gripper_close
        self.gripper_state = 1 # 1 for open, 0 for close
        
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self.direction_vector = []
        self.yaw_diff = 0.0
        
        self.posture = [0,0,0,0,0,0]
        
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
        
    def set_position(self, set_position):
        self.x, self.y, self.z = set_position
        print('set position complect.')
        
    def back_home(self):
        
        print('back to home complect.')
        return
        
    def common_update(self):
        joycon_stick_v = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        if joycon_stick_v > 4000:
            # Forward movement: 0.1 speed in the direction of the direction vector.
            self.position[0] += 0.002 * self.direction_vector[0]
            self.position[1] += 0.002 * self.direction_vector[1]
            self.position[2] += 0.002 * self.direction_vector[2]    
        elif joycon_stick_v < 1000:
            # Backward movement: 0.1 speed in the opposite direction of the direction vector.
            self.position[0] -= 0.002 * self.direction_vector[0]
            self.position[1] -= 0.002 * self.direction_vector[1]
            self.position[2] -= 0.002 * self.direction_vector[2]
        
        joycon_stick_h = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_right_horizontal()
        rotation_matrix_h = np.array([[0, -1, 0], [1, 0, 0],[0, 0, 1]])
        direction_vector_h = np.dot(rotation_matrix_h, self.direction_vector)

        if joycon_stick_h > 4000:
        # Forward movement: 0.1 speed in the direction of the direction vector.
            self.position[0] += 0.002 * direction_vector_h[0]
            self.position[1] += 0.002 * direction_vector_h[1]
            self.position[2] += 0.002 * direction_vector_h[2]    
        elif joycon_stick_h < 1000:
            # Backward movement: 0.1 speed in the opposite direction of the direction vector.
            self.position[0] -= 0.002 * direction_vector_h[0]
            self.position[1] -= 0.002 * direction_vector_h[1]
            self.position[2] -= 0.002 * direction_vector_h[2]
        
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
            self.position[0] = self.position[0] - 0.002 if self.position[0] > 0.002 else (self.position[0] + 0.002 if self.position[0] < -0.002 else self.position[0]) 
            self.position[1] = self.position[1] - 0.002 if self.position[1] > 0.002 else (self.position[1] + 0.002 if self.position[1] < -0.002 else self.position[1])
            self.position[2] = self.position[2] - 0.002 if self.position[2] > 0.002 else (self.position[2] + 0.002 if self.position[2] < -0.002 else self.position[2])
            
            self.yaw_diff = self.yaw_diff - 0.02 if self.orientation[2] > 0.04 else (self.yaw_diff + 0.02 if self.orientation[2] < -0.04 else self.yaw_diff)
            self.orientation_sensor.set_yaw_diff(self.yaw_diff)
            
            print(f'{self.orientation[2]=}')
            if self.orientation[2] < 0.04 and self.orientation[2] > 0.04:
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
        self.orientation = self.orientation_sensor.update(self.gyro.gyro_in_rad[0], self.gyro.accel_in_g[0])
        roll, pitch, yaw = self.orientation
        
        self.direction_vector = vec3(math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
        
        if out_format == "euler_deg":
            orientation_output = np.rad2deg(self.orientation)
        elif out_format == "quaternion":
            r4 = R.from_euler('xyz', self.orientation, degrees=False)
            orientation_output = r4.as_quat()
        else:
            orientation_output = self.orientation
        
        return orientation_output

    def update(self, out_format="euler_rad"):
        roll, pitch, yaw = self.get_orientation(out_format=out_format)
        self.position, gripper = self.common_update()
        
        self.orientation = [roll, pitch, yaw]
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