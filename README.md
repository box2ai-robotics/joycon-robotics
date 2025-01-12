# joycon-robotics

`joycon-robotics` 是一个用于 Lerobot SO100 ARM 的运动学库，结合了 Joy-Con 控制器的驱动程序。该项目为机器人提供了一套运动学计算模型，并允许通过 Joy-Con 控制器进行实时控制。

## 目录结构


## 特性

- **运动学运算**：支持运动学算法，提供正向和逆向运动学求解。
- **Joy-Con 控制**：通过 Joy-Con 控制器进行交互和控制, 。
- **跨平台支持**：兼容 Linux、macOS 和 Windows 操作系统。
- **高效矩阵运算**：使用 `numpy` 和 `scipy` 进行高效的矩阵计算。

## 安装

 **从源码进行安装：**
  ```git
  git clone https://github.com/box2ai_robotics/joycon-robotics.git
  cd joycon-robotics
  pip install -e .
  ```
  **安装ubuntu依赖：**
  ```make
  make install
  ```

## 使用示例

### 

## 参考

在开发 `joycon-robotics` 时，我们参考并借鉴了以下开源库：

- [**Robotics Toolbox for Python**](https://github.com/petercorke/robotics-toolbox-python)：这个库为我们提供了很多关于机器人运动学和控制的算法，极大地加速了我们的开发过程。
- [**hid**](https://github.com/trezor/cython-hidapi)

感谢这些开源项目的贡献，它们对我们的工作至关重要！
