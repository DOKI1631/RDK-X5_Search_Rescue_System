# 基于 RDK-X5 边缘计算与视觉云台的地震灾区智能搜救系统

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-RDK--X5%20%7C%20STM32-orange.svg)]()
[![AI-Core](https://img.shields.io/badge/AI--Core-BPU%20%7C%20YOLOv2-green.svg)]()

本系统是一套面向地震灾区或矿难环境的边缘计算智能搜救控制系统。系统以 **RDK-X5** 为核心边缘算力节点，部署 YOLO 目标检测模型，并通过独创的**时空防抖队列算法**有效拦截地表红砖、杂物等固定背景干扰。系统采用前后端分离架构，实现毫秒级人体目标感知、硬件云台协同追踪、音频警报以及上位机多维数据工作站的实时联动。

---

## 🚀 功能特性

- **边缘轻量推理**：基于 RDK-X5 BPU 算力核心，实现 640x640 高清视频流的实时推理。
- **时空防抖队列**：内置形态自适应过滤器与位置滑动窗口，有效防御固定背景及瞬时噪声引起的误报。
- **前后端分离架构**：
  - **后端 (Python/Flask)**：负责硬件级推理、串口下发停机指令 (`0x05`)、以及音频驱动。
  - **前端 (HTML5/WebSocket)**：全屏科技感双栏工作站，集成实时数字时钟、边缘节点状态监视器、以及高亮事件历史记录流。
- **工业级闭环**：检测人员后触发下位机（如 STM32）即时停机，并同步向网页控制台弹窗报警。

## 🛠️ 系统架构图
[ USB 摄像头 ] ──> [ RDK-X5 边缘板 (YOLO 推理) ] ──(串行总线 0x05)──> [ 下位机小车停机 ]
│
(Flask /check_detection 接口)
│
└──> [ 远程 UI 控制台 (Web 浏览器) ]



## ⚙️ 系统整体架构拓扑

本系统由**边缘感知大脑、下位机动力驱动、全景远程控制台**三部分组成，通过软硬件紧密协同形成数据闭环：

```
                    ┌───────────────────────────────────────┐
                    │       远程控制台 (HTML5 + CSS3)        │
                    └───────────────────▲───────────────────┘
                                        │ (Flask /video_feed & /check_detection 轮询)
                                        ▼
┌──────────────┐    ┌───────────────────────────────────────┐    (UART 115200)    ┌───────────────────────────┐
│ USB HD Camera│───>│     RDK-X5 边缘计算节点 (Python 3)     │───────────────────>│  下位机底盘驱动 (MCU 节点)   │
└──────────────┘    │  - BPU 硬件加速推理 (YOLO)            │                    │  - STM32F407 (电机PID/云台)│
                    │  - 假人姿态形态学过滤算法               │                    │  - Arduino (六足/轮式步态) │
                    │  - 音频警报联动 (aplay)               │                    └───────────────────────────┘
                    └───────────────────────────────────────┘
```

---

## 📂 项目目录结构规范

```text
RDK-X5_Search_Rescue_System/
├── arduino_main/               # 📂 Arduino main chassis gait finite state machine control
│   └── Mech_hexapod.ino        
├── arduino_low_level_drivers/          # 📂 Arduino low-level bus servo, sensor, and algorithm drivers
│   ├── config.h
│   ├── gait.cpp / gait.h               # Multi-terrain locomotion gait algorithms for the robot
│   ├── servo_control.cpp               # Closed-loop servo & gimbal control
│   └── thermal.cpp                     # Infrared thermal imaging & environmental sensor processing
│   └── sensors.cpp / sensors.h
│   └── leg_motion.cpp / leg_motion.h
│   └── fight.cpp / fight.h
│   └── dance.cpp / dance.h    
├── assets/                             # 📂 Media assets directory (stores README architecture diagrams)
│   └── ui_screenshot.png               
├── RDK X5                              # 📄 RDK-X5 edge computing inference engine, stabilization gateway, and Flask routes
│   └── Detect.wav
│   └── RDK.py
│   └── detect1.bin
├── ui.html                             # 📄 High-tech dual-column UI search & rescue workstation frontend control dashboard
├── STM32F407VGT6---MOTOR.uvprojx       # 📄 STM32 MDK-ARM core motor driver & gimbal control project
├── .gitignore                          # 📄 Git ignore configurations (filters temporary cache & large model files)
├── LICENSE                             # 📄 MIT Open-Source License
├── README.md                           # 📄 English documentation (this file)
└── README_cn.md                        # 📄 Chinese detailed delivery documentation
```

---

## ✨ 核心技术亮点与算法机理

### 1. BPU 硬件加速与异构模型推理
后端运行于 RDK-X5 自带的 BPU（Brain Processing Unit）加速阵列上，加载经由量化编译的 `detect.bin` 搜救检测模型。通过高吞吐的 `hobot_dnn` 库，避免了传统 CPU 推理的算力损耗，在 640x640 高清分辨率下实现低延时、低功耗的实时边缘检测。

### 2. 独创的“时空防抖队列算法”与形态学过滤器
为了防止搜救车在野外行驶时，路面散落的红砖、橙色条状背景引起 YOLO 判定误报（频繁急停），系统在代码中内置了两道防线：
* **假人横躺自适应过滤器**：计算目标框的宽高比 (Ratio) 与面积比 (Area)。如果物体极其扁平且面积极小，直接判定为地面条状红砖干扰进行拦截；若高度扁平但面积大，则判定为横躺状态的受困假人目标予以放行。
* **滑动窗口时序状态机**：维护一个深度为 15 帧的循环队列 (deque)。只有在最近 15 帧内成功捕获目标达 11 帧以上，系统才会判定真阳性并触发停机信号。

### 3. 多模态外设同步联动
一旦触发受困人员确认信号：
* **串口指令级交互**：后端通过 `/dev/ttyS1` 总线以 `115200` 波特率向底层下发 `0x05` 紧急制动停机字节，接管下位机运动状态。
* **音频即时震慑**：利用 Linux 多线程 `nohup aplay` 异步调用 USB 声卡，播放现场救援导引语音，且设置了安全冷却机制防止音频重叠爆音。

---

## 🛠️ 生产环境配置与部署指南

### 1. 后端部署（RDK-X5 Linux 节点）
由于板子可能处于离线或断网隔离状态，推荐通过以下本地命令直接运行：

```bash
# 确保在仓库根目录下，直接拉起推理引擎
python3 RDK.py
```

### 2. 下位机编译（STM32 / Arduino）
* **STM32**：使用 Keil uVision5 打开根目录下的 `STM32F407VGT6---MOTOR.uvprojx` 工程，系统已配置好高级定时器输出 4 路带死区 PWM 用于四驱直流减速电机，并集成串口中断，收到 `0x05` 自动切入紧急刹车。
* **Arduino**：使用 Arduino IDE 载入 `arduino_chassis_main/Mech_hexapod.ino`，结合多足复杂三角步态解算，实现越障搜救。

### 3. 前端数字化控制大厅启动
用文本编辑器打开根目录下的 `ui.html`，修改参数将 `BOARD_IP` 变更为你板子的实际静态局域网 IP，随后在浏览器中双击打开 `ui.html` 即可。

---

## ⚖️ 开源协议 (License)

本项目基于 **MIT License** 协议开源。
