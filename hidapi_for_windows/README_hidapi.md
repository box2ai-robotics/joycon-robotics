# 🎮 Windows JoyCon控制 - 完整使用指南

## 📋 **简介**

本指南提供Windows环境下使用JoyCon输出机器人末端姿态的完整说明，包括环境安装、配置和使用方法。

感谢
---
### **前置要求**

| 项目 | 要求 | 说明 |
|------|------|------|
| **操作系统** | Windows 10/11 | 64/32位 |
| **Python** | 3.8+ | 推荐3.10 |
| **蓝牙** | 蓝牙4.0+ | 用于连接JoyCon |

## ⚡ **快速开始**（3步）

### **1. 安装依赖**

```powershell
# 安装 BetterJoy 驱动
# 进入hidapi_for_windows/BetterJoy_v7.1/Drivers
# 运行ViGEmBusSetup_x64.msi（64位系统，如果是32位系统请运行ViGEmBusSetup_x86.msi）

# python安装hidapi与numpy
pip install hidapi numpy matplotlib
pip uninstall hid # 不能与hidapi共存，否则会报错 ImportError: Unable to load any of the following libraries:hidapi.dll

# 验证安装
python -c "import hid; import numpy; print('✅ 所有依赖已安装')"
```

### **2. 连接JoyCon**

- 启动 BetterJoy应用程序（ 以确保可以与手柄建立稳定连接 hidapi_for_windows/BetterJoy_v7.1/BetterJoyForCemu.exe）
- 打开Windows蓝牙设置
- 按住JoyCon的同步按钮（右侧SR按钮旁边小圆点）
- 选择"Joy-Con (R)"并配对

### **3. 运行程序**

```powershell
# 进入joycon-robotics目录，运行测试代码
python hidapi_for_windows/joyconrobotic_hidapi.py
```

**就这么简单！** ✨

---

### **完整控制映射**

#### **【位置控制 - 第一人称视角】**
```
摇杆 ↑      - 前进（相对末端姿态）
摇杆 ↓      - 后退（相对末端姿态）
摇杆 ←      - 左移（相对末端姿态）
摇杆 →      - 右移（相对末端姿态）
R键         - 上升（Z+）
摇杆按压 ●  - 下降（Z-）
```

#### **【位置控制 - 世界坐标系】**
```
X键         - 向前（世界X+）
B键         - 向后（世界X-）
```

#### **【姿态控制】**（陀螺仪）
```
⚡ 左右倾斜  - Roll（±83°）
⚡ 上下倾斜  - Pitch（±83°，手腕60° → 机械臂83°）
⚡ 左右旋转  - Yaw（±180°）
```

#### **【其他功能】**
```
ZR键        - 开合夹爪（初始打开）
Home键      - 位姿复位
```

---

## 🔧 **常见问题**

### **Q1: JoyCon无法连接？**

**解决方案**：

1. **检查蓝牙**：
   ```powershell
   # 确认蓝牙已开启
   # Windows设置 → 设备 → 蓝牙和其他设备
   ```

2. **重新配对**：
   - 移除旧的JoyCon配对
   - 按住同步按钮重新配对

3. **重启蓝牙服务**：
   ```powershell
   # 以管理员身份运行PowerShell
   Restart-Service bthserv
   ```
