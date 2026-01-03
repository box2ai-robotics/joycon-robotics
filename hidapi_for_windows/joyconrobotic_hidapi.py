#!/usr/bin/env python3
"""
JoyCon控制 - hidapi版本

使用hidapi直接读取JoyCon，性能更好，数据更稳定

安装:
    pip install hidapi

前置要求:
    1. JoyCon已通过蓝牙连接到Windows
    2. 运行一次BetterJoy(最小化)

运行:
    python .\hidapi_for_windows\joyconrobotic_hidapi.py
"""

import numpy as np
import math
import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import hid
# 导入hidapi版本的JoyCon读取器
from joycon_hidapi_reader import JoyConHIDAPIReader

class JoyConController:
    """JoyCon控制器（hidapi版本）"""
    
    def __init__(self, reader, init_gpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], gripper_state=0):
        """初始化控制器
        
        Args:
            reader: JoyConHIDReaderHidapi实例
        """
        self.reader = reader
        self.position = list(init_gpos[0:3])
        self.position_speed = 0.003  # m/step
        
        # Pitch增益（基于实际人体工学：手腕舒适摆动60度 → 机械臂达到83度）
        self.pitch_gain = 1.5  # 适度增益，提升操作舒适度
        
        # 按钮边缘检测（防止反复触发）
        self.last_buttons = {
            'ZR': False,
            'R': False,
            'STICK': False,
            'HOME': False,
        }
        
        # 姿态初始化
        self.roll_offset = 0.0
        self.last_roll = 0.0
        
        # 夹爪状态
        self.gripper_state = gripper_state
        self.gripper_open = 0.5
        self.gripper_close = -0.15
        
        # 初始姿态（用于复位）
        self.init_position = self.position.copy()
        self.init_roll_offset = 0.0
    
    def get_control(self):
        """获取控制指令
        
        Returns:
            pose: [x, y, z, roll, pitch, yaw] (弧度)
            gripper_state: 夹爪状态
            button_control: 按钮控制字典
        """
        state = self.reader.get_state()
        
        # 按钮控制
        button_control = {}
        
        # 位置控制（摇杆 - 第一人称视角）
        stick_x = state['stick_x']
        stick_y = state['stick_y']
        
        # 死区处理（防止漂移）
        deadzone = 0.1
        if abs(stick_x) < deadzone:
            stick_x = 0.0
        if abs(stick_y) < deadzone:
            stick_y = 0.0
        
        # 获取当前姿态（用于计算方向向量）
        roll = state['roll']
        pitch = state['pitch']
        yaw = state['yaw']
        
        # 计算前向方向向量（基于末端姿态）
        # direction_vector = (cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), sin(pitch))
        direction_vector_x = math.cos(pitch) * math.cos(yaw)
        direction_vector_y = math.cos(pitch) * math.sin(yaw)
        direction_vector_z = math.sin(pitch)
        
        # 计算右向方向向量（基于末端姿态）
        # direction_vector_right = (cos(roll) * sin(-yaw), cos(roll) * cos(-yaw), sin(-roll))
        direction_right_x = math.cos(roll) * math.sin(-yaw)
        direction_right_y = math.cos(roll) * math.cos(-yaw)
        direction_right_z = math.sin(-roll)
        
        # 前后移动 - 沿着末端指向的方向（第一人称视角）
        self.position[0] += stick_y * self.position_speed * direction_vector_x
        self.position[1] += stick_y * self.position_speed * direction_vector_y
        self.position[2] += stick_y * self.position_speed * direction_vector_z
        
        # 左右移动 - 沿着末端的横向方向
        self.position[0] -= stick_x * self.position_speed * direction_right_x
        self.position[1] -= stick_x * self.position_speed * direction_right_y
        self.position[2] -= stick_x * self.position_speed * direction_right_z
        
        # 上下（Z轴）- 按键
        if state['buttons'].get('R', False):
            self.position[2] += self.position_speed  # R键上升
        if state['buttons'].get('STICK', False):
            self.position[2] -= self.position_speed  # 摇杆按压下降
        
        # 世界坐标系X轴移动 - 按键（参考说明书）
        if state['buttons'].get('X', False):
            self.position[0] += self.position_speed  # X键向前（世界坐标系X+）
        if state['buttons'].get('B', False):
            self.position[0] -= self.position_speed  # B键向后（世界坐标系X-）
        
        # 夹爪控制（ZR键）- 边缘检测（按下瞬间触发一次）
        zr_pressed = state['buttons'].get('ZR', False)
        if zr_pressed and not self.last_buttons['ZR']:
            # 按钮从未按下变为按下（上升沿）
            self.gripper_state = self.gripper_close if self.gripper_state == self.gripper_open else self.gripper_open
        self.last_buttons['ZR'] = zr_pressed
        
        # Home键复位 - 边缘检测
        home_pressed = state['buttons'].get('HOME', False)
        if home_pressed and not self.last_buttons['HOME']:
            self.position = self.init_position.copy()
            self.roll_offset = self.init_roll_offset
        self.last_buttons['HOME'] = home_pressed
        
        # 姿态控制（陀螺仪）
        # joycon_hid_reader_hidapi已经应用了所有必要的处理：
        # - 加速度计*π
        # - 互补滤波器
        # - 低通滤波器
        # - lerobot模式的Roll缩放
        # 所以这里直接使用，无需额外处理
        roll = state['roll']
        pitch = state['pitch']
        yaw = state['yaw']
        
        # 应用Linux版本的-90度Roll偏移（与lerobot末端坐标系对齐）
        # 参考lerobot_plus_joycon_gpos.py line 82
        roll = roll - np.pi / 2
        
        # Pitch取负并应用增益（基于人体工学优化）
        # 参考lerobot_plus_joycon_gpos.py line 81
        pitch = -pitch * self.pitch_gain  # 手腕摆动60度 → 机械臂83度
        
        # 返回pose
        pose = self.position + [roll, pitch, yaw]
        return pose, self.gripper_state, button_control
    
    def set_position(self, position):
        """设置位置"""
        self.position = list(position)
    
    def disconnect(self):
        """断开连接"""
        self.reader.disconnect()

class RealTimeVisualizer:
    def __init__(self, max_history=500):
        self.fig = None
        self.ax = None
        self.initialized = False
        
        # 数据累计相关属性
        self.history_positions = []  # 累计所有位置数据
        self.history_orientations = []  # 累计所有姿态数据
        self.max_history = max_history  # 最大历史数据量
        self.cumulative_x = []
        self.cumulative_y = [] 
        self.cumulative_z = []
        self.cumulative_roll = []
        self.cumulative_pitch = []
        self.cumulative_yaw = []
    
    def add_pose_data(self, x, y, z, roll, pitch, yaw):
        """累计单帧位姿数据"""
        self.cumulative_x.append(x)
        self.cumulative_y.append(y)
        self.cumulative_z.append(z)
        self.cumulative_roll.append(roll)
        self.cumulative_pitch.append(pitch)
        self.cumulative_yaw.append(yaw)
        
        # 限制历史数据长度
        if len(self.cumulative_x) > self.max_history:
            self.cumulative_x.pop(0)
            self.cumulative_y.pop(0)
            self.cumulative_z.pop(0)
            self.cumulative_roll.pop(0)
            self.cumulative_pitch.pop(0)
            self.cumulative_yaw.pop(0)
    
    def get_cumulative_data(self):
        """获取累计的数据"""
        return (self.cumulative_x, self.cumulative_y, self.cumulative_z,
                self.cumulative_roll, self.cumulative_pitch, self.cumulative_yaw)
    
    def clear_cumulative_data(self):
        """清空累计数据"""
        self.cumulative_x.clear()
        self.cumulative_y.clear()
        self.cumulative_z.clear()
        self.cumulative_roll.clear()
        self.cumulative_pitch.clear()
        self.cumulative_yaw.clear()

    def initialize(self):
        """初始化可视化窗口"""
        plt.ion()  # 开启交互模式[5](@ref)
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.initialized = True
        
        # 设置初始视图参数
        self.ax.set_xlabel('X轴')
        self.ax.set_ylabel('Y轴')
        self.ax.set_zlabel('Z轴')
        self.ax.set_title('JoyCon机器人实时3D轨迹可视化')
        
        # 设置坐标轴范围
        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([-0.1, 0.5])
        
        # 添加网格
        self.ax.grid(True)
        
        plt.tight_layout()

    def _adjust_axes_range(self, x_data, y_data, z_data):
        """动态调整坐标轴范围"""
        margin = 0.1
        
        if len(x_data) > 0:
            x_min, x_max = min(x_data), max(x_data)
            y_min, y_max = min(y_data), max(y_data)
            z_min, z_max = min(z_data), max(z_data)
            
            # 确保有足够的显示范围
            x_range = max(x_max - x_min, 0.1)
            y_range = max(y_max - y_min, 0.1)
            z_range = max(z_max - z_min, 0.1)
            
            self.ax.set_xlim([x_min - margin, x_max + margin])
            self.ax.set_ylim([y_min - margin, y_max + margin])
            self.ax.set_zlim([z_min - margin, z_max + margin])

    def _update_display_info(self, x, y, z, roll, pitch, yaw, frame_count):
        """更新显示信息"""
        # 转换为角度显示
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        self.ax.set_xlabel('X轴 (米)')
        self.ax.set_ylabel('Y轴 (米)')
        self.ax.set_zlabel('Z轴 (米)')
        
        self.ax.set_title(
            f'机器人实时运动轨迹 - 累计{frame_count}帧\n'
            f'位置: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m\n'
            f'姿态: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°'
        )
        
        self.ax.legend(loc='upper left')

    def update(self, target_pose):
        """更新可视化显示"""
        if not self.initialized:
            self.initialize()
        
        # 获取数据
        x, y, z, roll, pitch, yaw = target_pose
        
        if len(str(x)) == 0 or x is None:  # 更健壮的空值检查
            return
    
        # 累计数据
        self.add_pose_data(x, y, z, roll, -pitch, yaw)

        # 获取累计数据
        cum_x, cum_y, cum_z, cum_roll, cum_pitch, cum_yaw = self.get_cumulative_data()
        
        if len(cum_x) == 0:
            return

        # 清空当前图形
        self.ax.clear()
        
        # 绘制累计运动轨迹
        if len(cum_x) > 1:
            self.ax.plot(cum_x, cum_y, cum_z, 'r-', alpha=0.7, linewidth=2, 
                        label=f'运动轨迹 ({len(cum_x)}帧)')
        
        # 绘制当前位置点
        current_x, current_y, current_z = cum_x[-1], cum_y[-1], cum_z[-1]
        self.ax.scatter(current_x, current_y, current_z, c='blue', s=100, 
                    marker='o', label='当前位置')
        
        # 绘制方向箭头
        current_roll, current_pitch, current_yaw = cum_roll[-1], cum_pitch[-1], cum_yaw[-1]
        arrow_length = 0.05
        dx = math.cos(current_pitch) * math.cos(current_yaw) * arrow_length
        dy = math.cos(current_pitch) * math.sin(current_yaw) * arrow_length
        dz = math.sin(current_pitch) * arrow_length
        
        self.ax.quiver(current_x, current_y, current_z, dx, dy, dz, 
                    color='green', linewidth=2, arrow_length_ratio=0.3, 
                    label='末端朝向')
        
        # 动态调整坐标轴范围
        self._adjust_axes_range(cum_x, cum_y, cum_z)
        
        # 更新标题和信息显示
        self._update_display_info(current_x, current_y, current_z, 
                                current_roll, current_pitch, current_yaw, 
                                len(cum_x))
        
        # 刷新显示
        plt.draw()
        plt.pause(0.01)
        
        def close(self):
            """关闭可视化"""
            if self.initialized:
                plt.ioff()
                plt.close(self.fig)

def main():
    """主函数"""
    print("=" * 60)
    print("JoyCon机器人遥控 - hidapi版本 - 多平台兼容 Windows/Linux/Mac")
    print("=" * 60)
    
    # 连接JoyCon
    print("\n查找JoyCon...")
    reader = JoyConHIDAPIReader()
    
    if not reader.connect():
        print("❌ 无法连接JoyCon")
        print("\n请确保:")
        print("1. JoyCon已通过蓝牙连接")
        print("2. 启动BetterJoy（最小化）")
        return
    
    # 校准
    reader.calibrate(samples=100)
    
    # 创建控制器和可视化器
    controller = JoyConController(reader)
    visualizer = RealTimeVisualizer()

    print("\n" + "=" * 60)
    print("JoyCon控制说明 (hidapi版本):")
    print("  【位置控制 - 第一人称视角】")
    print("    摇杆 ↑      - 前进（相对末端姿态）")
    print("    摇杆 ↓      - 后退（相对末端姿态）")
    print("    摇杆 ←      - 左移（相对末端姿态）")
    print("    摇杆 →      - 右移（相对末端姿态）")
    print("    摇杆按压 ●  - 下降 (Z-)")
    print("    R键         - 上升 (Z+)")
    print("  【位置控制 - 世界坐标系】")
    print("    X键         - 向前（世界坐标X+）")
    print("    B键         - 向后（世界坐标X-）")
    print("  【姿态控制】(陀螺仪)")
    print("    ⚡ 倾斜JoyCon  - 控制Roll和Pitch")
    print("    ⚡ 旋转JoyCon  - 控制Yaw")
    print("  【其他功能】")
    print("    ZR键        - 开合夹爪")
    print("    Home键      - 位姿复位")
    print("=" * 60)
    print()
    
    step_count = 0

    try:
        while 1:
            # 输出图像
            # 读取控制输入
            target_pose, gripper_state, button_control = controller.get_control()
            visualizer.update(target_pose)

            # 打印target_pose（与Linux版本格式一致）
            if step_count % 100 == 0:
                px, py, pz, roll, pitch, yaw = target_pose
                print(f"target_pose: ['{px:.3f}', '{py:.3f}', '{pz:.3f}', "
                        f"'{roll:.3f}', '{pitch:.3f}', '{yaw:.3f}']")
            
            # 调试：每500步打印一次摇杆状态
            if step_count % 500 == 0:
                state_debug = controller.reader.get_state()
                print(f"  [调试] 摇杆: X={state_debug['stick_x']:.3f} Y={state_debug['stick_y']:.3f}")

            
            step_count += 1
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    
    finally:
        controller.disconnect()
        print("✅ 控制器已退出")


if __name__ == "__main__":
    main()

