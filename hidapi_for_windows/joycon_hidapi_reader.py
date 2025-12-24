#!/usr/bin/env python3
"""
JoyCon HID Reader - 使用hidapi（跨平台，更快速）

使用hidapi直接读取JoyCon的HID报告，获取陀螺仪和加速度计数据

安装:
    安装BetterJoy驱动，启动软件绑定手柄
    pip install hidapi

特点:
    - 跨平台（Windows/Linux/macOS）
    - 更快的数据读取速度
    - 更稳定的IMU数据
    - 与Linux版本joycon-python使用相同的底层库
"""

import hid
import numpy as np
import time
import threading

class JoyConHIDAPIReader:
    """使用hidapi读取JoyCon HID数据"""
    
    # JoyCon HID参数
    VENDOR_ID = 0x057E
    PRODUCT_ID_JOYCON_R = 0x2007
    
    # 采样率
    IMU_SAMPLE_RATE = 200  # Hz (实际约200Hz)
    
    def __init__(self):
        """初始化HID读取器"""
        self.device = None
        self.running = False
        self.thread = None
        
        # IMU原始数据（6轴）
        self.gyro = np.array([0.0, 0.0, 0.0])      # 陀螺仪 (rad/s)
        self.accel = np.array([0.0, 0.0, 0.0])     # 加速度计 (g)
        
        # 按钮和摇杆状态
        self.buttons = {}
        self.stick_x = 0.0  # -1.0 到 1.0
        self.stick_y = 0.0  # -1.0 到 1.0
        
        # 姿态估计（弧度）
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 校准数据
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.roll_offset = 0.0
        
        # 互补滤波器参数（参考JoyconRobotics）
        self.alpha = 0.55  # 陀螺仪权重（与Linux版本一致）
        self.dt = 0.01  # 固定时间步长（与Linux版本一致）
        
        # 低通滤波器参数（参考JoyconRobotics）
        self.lpf_alpha = 0.08  # lerobot模式
        self.lpf_roll_prev = 0.0
        self.lpf_pitch_prev = 0.0
        
        # Yaw方向向量（用于四元数旋转，简化版）
        self.yaw_integrated = 0.0
        
        # 包计数器（用于发送子命令）
        self.packet_number = 0
        
        # 数据锁
        self.lock = threading.Lock()
    
    def connect(self):
        """连接JoyCon"""
        try:
            # 打开JoyCon设备
            self.device = hid.device()
            self.device.open(self.VENDOR_ID, self.PRODUCT_ID_JOYCON_R)
            self.device.set_nonblocking(1)
            
            print(f"✅ JoyCon已连接")
            print(f"   制造商: {self.device.get_manufacturer_string()}")
            print(f"   产品: {self.device.get_product_string()}")
            
            # 启用IMU（完整初始化）
            if not self._enable_imu():
                print("⚠️  IMU初始化失败，陀螺仪可能无法工作")
                print("   提示：如果IMU数据全为0，请断开并重新连接JoyCon")
            
            # 启动读取线程
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("✅ 开始读取JoyCon数据")
            
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def _send_subcommand(self, subcommand, data):
        """发送子命令到JoyCon
        
        Args:
            subcommand: 子命令ID (例如 0x03, 0x40)
            data: 子命令参数（列表）
        """
        try:
            # 构建输出报告
            # 报告ID + 全局包计数器 + rumble data (8字节) + subcommand + data
            packet = [0x01]  # Output report ID
            packet += [self.packet_number & 0xFF]  # 包计数器
            
            # Rumble data (8字节，全为0表示无震动)
            packet += [0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40]
            
            # 子命令
            packet += [subcommand]
            packet += data
            
            # 填充到64字节
            packet += [0x00] * (64 - len(packet))
            
            # 发送
            self.device.write(bytes(packet))
            
            # 增加包计数器
            self.packet_number = (self.packet_number + 1) & 0xFF
            
            # 等待JoyCon处理
            time.sleep(0.05)
            return True
            
        except Exception as e:
            print(f"⚠️  发送子命令 0x{subcommand:02X} 失败: {e}")
            return False
    
    def _enable_imu(self):
        """启用IMU（完整初始化流程）"""
        try:
            # 初始化包计数器
            self.packet_number = 0
            
            # 步骤1: 设置输入报告模式为0x30（标准完整模式，包含IMU数据）
            # Sub-command 0x03: Set input report mode
            # 参数: 0x30 = 标准完整模式
            if not self._send_subcommand(0x03, [0x30]):
                print("⚠️  设置输入报告模式失败")
                return False
            
            # 步骤2: 启用IMU数据流
            # Sub-command 0x40: Enable IMU
            # 参数: 0x01 = 启用
            if not self._send_subcommand(0x40, [0x01]):
                print("⚠️  启用IMU数据流失败")
                return False
            
            # 步骤3（可选）: 设置IMU灵敏度
            # Sub-command 0x41: Set IMU sensitivity
            # 参数: [gyro_sensitivity, accel_sensitivity, gyro_performance, accel_filter]
            # 使用默认值: [0x03, 0x03, 0x01, 0x01]
            # self._send_subcommand(0x41, [0x03, 0x03, 0x01, 0x01])
            
            print("✅ IMU完整初始化成功")
            return True
            
        except Exception as e:
            print(f"⚠️  启用IMU失败: {e}")
            return False
    
    def _read_loop(self):
        """读取数据的线程"""
        while self.running:
            try:
                # 读取HID报告（非阻塞）
                data = self.device.read(64, timeout_ms=10)
                
                if data and len(data) >= 49:
                    # 只处理标准输入报告 (0x30)
                    if data[0] == 0x30:
                        self._parse_input_report(data)
                
            except Exception as e:
                if self.running:  # 只在运行时报告错误
                    print(f"⚠️  读取数据错误: {e}")
                time.sleep(0.001)
    
    def _parse_input_report(self, data):
        """解析输入报告 (0x30)"""
        with self.lock:
            # 按钮状态 (字节3-5)
            buttons_right = data[3]
            self.buttons = {
                'Y': bool(buttons_right & 0x01),
                'X': bool(buttons_right & 0x02),
                'B': bool(buttons_right & 0x04),
                'A': bool(buttons_right & 0x08),
                'R': bool(buttons_right & 0x40),
                'ZR': bool(buttons_right & 0x80),
                'HOME': bool(data[4] & 0x10),
                'STICK': bool(data[4] & 0x04),
            }
            
            # 摇杆数据 (字节9-11) - Windows版本位置不同！
            # 注意：Linux版本在字节6-8，Windows版本在字节9-11
            stick_raw = data[9] | ((data[10] & 0x0F) << 8)
            stick_x = (stick_raw - 2048) / 2048.0
            stick_y_raw = (data[10] >> 4) | (data[11] << 4)
            stick_y = (stick_y_raw - 2048) / 2048.0
            
            self.stick_x = np.clip(stick_x, -1.0, 1.0)
            self.stick_y = np.clip(stick_y, -1.0, 1.0)
            
            # IMU数据 (字节13开始，每个样本12字节)
            # JoyCon每个报告包含3个IMU样本，我们使用第一个
            imu_offset = 13
            
            # 加速度计 (3轴，每轴2字节，小端序)
            accel_x_raw = int.from_bytes(data[imu_offset:imu_offset+2], 'little', signed=True)
            accel_y_raw = int.from_bytes(data[imu_offset+2:imu_offset+4], 'little', signed=True)
            accel_z_raw = int.from_bytes(data[imu_offset+4:imu_offset+6], 'little', signed=True)
            
            # 陀螺仪 (3轴，每轴2字节，小端序)
            gyro_x_raw = int.from_bytes(data[imu_offset+6:imu_offset+8], 'little', signed=True)
            gyro_y_raw = int.from_bytes(data[imu_offset+8:imu_offset+10], 'little', signed=True)
            gyro_z_raw = int.from_bytes(data[imu_offset+10:imu_offset+12], 'little', signed=True)
            
            # 转换为物理单位
            # 加速度计：LSB/(m/s^2) ≈ 4096 (根据JoyCon规格)
            ACCEL_SCALE = 4096.0  # LSB/g
            self.accel[0] = accel_x_raw / ACCEL_SCALE
            self.accel[1] = accel_y_raw / ACCEL_SCALE
            self.accel[2] = accel_z_raw / ACCEL_SCALE
            
            # 陀螺仪：LSB/(°/s) ≈ 13.371 (根据JoyCon规格)
            GYRO_SCALE = 13.371  # LSB/(°/s)
            self.gyro[0] = (gyro_x_raw / GYRO_SCALE) * (np.pi / 180.0)  # 转换为 rad/s
            self.gyro[1] = (gyro_y_raw / GYRO_SCALE) * (np.pi / 180.0)
            self.gyro[2] = (gyro_z_raw / GYRO_SCALE) * (np.pi / 180.0)
            
            # 应用陀螺仪偏移校准
            self.gyro -= self.gyro_offset
            
            # 更新姿态估计
            self._update_attitude()
    
    def _update_attitude(self):
        """更新姿态估计（严格参考JoyconRobotics的AttitudeEstimator）"""
        # 重置pitch和roll（将从头计算）
        pitch_gyro = 0.0
        roll_gyro = 0.0
        
        # 加速度计数据处理（关键：乘以π，与Linux版本一致）
        ax = self.accel[0] * np.pi
        ay = self.accel[1] * np.pi
        az = self.accel[2] * np.pi
        
        # 陀螺仪数据
        gx, gy, gz = self.gyro[0], self.gyro[1], self.gyro[2]
        
        # 从加速度计计算Roll和Pitch（与Linux版本一致）
        # 注意：roll_acc使用-az（负号很重要！）
        roll_acc = np.arctan2(ay, -az)
        pitch_acc = np.arctan2(ax, np.sqrt(ay**2 + az**2))
        
        # 陀螺仪积分（注意：Roll是减号！）
        pitch_gyro += gy * self.dt
        roll_gyro -= gx * self.dt  # 关键：减号！
        
        # 互补滤波器（与Linux版本一致：alpha=0.55）
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc
        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_acc
        
        # 低通滤波器（与Linux版本一致）
        self.pitch = self.lpf_alpha * self.pitch + (1 - self.lpf_alpha) * self.lpf_pitch_prev
        self.roll = self.lpf_alpha * self.roll + (1 - self.lpf_alpha) * self.lpf_roll_prev
        
        self.lpf_pitch_prev = self.pitch
        self.lpf_roll_prev = self.roll
        
        # Yaw积分（简化版，不使用四元数）
        self.yaw_integrated += gz * self.dt
        self.yaw = -self.yaw_integrated  # 注意：负号
        
        # lerobot模式的Roll缩放（与Linux版本一致）
        self.roll = self.roll * np.pi / 2
    
    def calibrate(self, samples=100):
        """校准陀螺仪偏移"""
        print("请将JoyCon平放在桌面...")
        time.sleep(0.5)
        
        print("开始校准，请保持JoyCon静止...")
        time.sleep(0.5)
        
        print("收集陀螺仪偏移数据...")
        gyro_samples = []
        
        for i in range(samples):
            with self.lock:
                gyro_samples.append(self.gyro.copy())
            time.sleep(0.01)  # 100Hz采样
        
        # 计算平均偏移
        self.gyro_offset = np.mean(gyro_samples, axis=0)
        print(f"✅ 校准完成！陀螺仪偏移: {self.gyro_offset}")
        
        # 初始化姿态（与Linux版本一致）
        time.sleep(0.2)
        with self.lock:
            # 使用加速度计初始化（应用π缩放）
            ax = self.accel[0] * np.pi
            ay = self.accel[1] * np.pi
            az = self.accel[2] * np.pi
            
            # 计算初始Roll和Pitch（与Linux版本一致）
            roll_initial = np.arctan2(ay, -az)  # 注意：-az
            pitch_initial = np.arctan2(ax, np.sqrt(ay**2 + az**2))
            
            # 应用lerobot模式的缩放
            self.roll = roll_initial * np.pi / 2
            self.pitch = pitch_initial
            self.yaw = 0.0
            self.yaw_integrated = 0.0
            
            # 初始化低通滤波器
            self.lpf_roll_prev = self.roll
            self.lpf_pitch_prev = self.pitch
            
            self.roll_offset = 0.0
            
            print(f"✅ 初始姿态：Roll={np.degrees(self.roll):.1f}° Pitch={np.degrees(self.pitch):.1f}°")
    
    def get_state(self):
        """获取当前状态"""
        with self.lock:
            return {
                'gyro': self.gyro.copy(),
                'accel': self.accel.copy(),
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw,
                'stick_x': self.stick_x,
                'stick_y': self.stick_y,
                'buttons': self.buttons.copy()
            }
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.device:
            self.device.close()
        print("✅ JoyCon已断开")


# 测试代码
if __name__ == "__main__":
    print("=" * 60)
    print("JoyCon HID Reader测试（hidapi版本）")
    print("=" * 60)
    
    reader = JoyConHIDAPIReader()
    
    if not reader.connect():
        print("❌ 无法连接JoyCon")
        exit(1)
    
    # 校准
    reader.calibrate(samples=100)
    
    print("\n" + "=" * 60)
    print("开始读取数据（Ctrl+C停止）")
    print("=" * 60)
    
    try:
        step = 0
        while True:
            state = reader.get_state()
            
            if step % 50 == 0:  # 每50步打印一次（约0.5秒）
                print(f"\nStep {step}:")
                print(f"  Gyro: [{state['gyro'][0]:7.3f}, {state['gyro'][1]:7.3f}, {state['gyro'][2]:7.3f}] rad/s")
                print(f"  Accel: [{state['accel'][0]:6.3f}, {state['accel'][1]:6.3f}, {state['accel'][2]:6.3f}] g")
                print(f"  Attitude: Roll={np.degrees(state['roll']):6.3f}° Pitch={np.degrees(state['pitch']):6.3f}° Yaw={np.degrees(state['yaw']):6.3f}°")
                print(f"  Stick: X={state['stick_x']:5.2f} Y={state['stick_y']:5.2f}")
                print(f"  Buttons: {[k for k, v in state['buttons'].items() if v]}")
            
            step += 1
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\n🛑 用户中断")
    
    finally:
        reader.disconnect()
        print("✅ 测试完成")

