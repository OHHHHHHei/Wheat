# Xiamen University RCS Team - RoboMaster 2026 Season Infantry Code (Wheat) 🌾

![Team](https://img.shields.io/badge/Team-XMU%20RCS-blue)
![Robot](https://img.shields.io/badge/Robot-Infantry%20(Mecanum)-orange)
![Chip](https://img.shields.io/badge/Chip-STM32F405-brightgreen)
![Framework](https://img.shields.io/badge/Framework-FreeRTOS%20%2B%20HAL-blueviolet)
![IDE](https://img.shields.io/badge/IDE-Visual%20Studio%20%2B%20VisualGDB-purple)

## 📖 项目简介 (Introduction)

本项目为厦门大学 RCS 战队 **2026 赛季 RoboMaster 麦轮步兵机器人** 的嵌入式控制代码。
项目代号 **Wheat**，基于 **STM32F405** 主控开发，采用 **FreeRTOS** 实时操作系统进行任务调度，实现了底盘运动、云台控制、发射机构及超级电容管理等核心功能。

## 🛠️ 开发环境 (Environment)

* **IDE**: Visual Studio 2019/2022 (配合 VisualGDB 插件)
* **Toolchain**: ARM-GCC
* **Hardware**: STM32F405RG / STM32F405ZGT6 (根据实际板卡)
* **Debugger**: ST-Link / J-Link

## 🏗️ 软件架构 (Software Architecture)

系统基于 **FreeRTOS** 构建，采用模块化设计。主要的任务调度逻辑位于 `taskslist.cpp` 中。

### 1. 任务分配 (Tasks)

| 任务名称 | 优先级 | 频率 | 功能描述 | 对应函数 |
| :--- | :--- | :--- | :--- | :--- |
| **ControlTask** | High | 1000Hz (1ms) | 核心控制循环。负责底盘、云台、发射机构的解算与 Update。 | `ControlTask` |
| **CanTransmitTask** | High | 1000Hz (1ms) | CAN 通信发送任务。分时复用发送底盘、云台及达妙电机数据。 | `CanTransmitTask` |
| **MotorUpdateTask** | High | 500Hz (2ms) | 电机状态更新任务。处理 CAN 接收回调，更新电机反馈数据。 | `MotorUpdateTask` |
| **DecodeTask** | Medium | 200Hz (5ms) | 传感器与遥控器解算。处理 RC、IMU 及裁判系统数据。 | `DecodeTask` |
| **ArmTask** | Low | Low Freq | 初始化与辅助任务。负责达妙电机初始化及功率模块通信。 | `ArmTask` |

### 2. 模块划分 (Modules)

代码按功能模块进行了封装，主要包含以下部分：

* **驱动层 (Drivers)**:
    * `can.cpp/h`: CAN 总线收发封装。
    * `imu.cpp/h`: 陀螺仪数据读取与姿态解算。
    * `usart.cpp/h`: 串口通信（遥控器、裁判系统）。
    * `tim.cpp/h`: 定时器配置。
* **设备层 (Devices)**:
    * `motor.cpp/h`: 传统电机（如 DJI 3508/6020）对象封装。
    * `HTmotor.cpp/h`: 海泰电机驱动。
    * `DMmotor`: 达妙电机驱动（用于特殊关节或云台）。
    * `supercap.cpp/h`: 超级电容充放电管理。
    * `RC.cpp/h`: 遥控器数据解析。
* **算法层 (Algorithms)**:
    * `pid.h`: PID 控制器实现。
    * `pid_feedforward.cpp`: 前馈控制算法。
    * `kalman.h`: 卡尔曼滤波算法（用于预测或数据平滑）。
    * `CRC.cpp`: 数据校验算法。

## 📂 目录结构 (Directory Structure)

```text
.
├── FreeRTOS.sln          # Visual Studio 解决方案文件
├── STM32F405/            # 核心代码目录
│   ├── taskslist.cpp     # 任务调度中心
│   ├── control.cpp       # 机器人整体控制逻辑
│   ├── algorithm/        # (建议整理) PID, Kalman, CRC 等算法
│   ├── device/           # (建议整理) Motor, IMU, RC 等设备驱动
│   ├── driver/           # (建议整理) CAN, USART, TIM, GPIO 底层驱动
│   └── ...
└── README.md             # 项目说明文档
