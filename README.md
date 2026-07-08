🚀 [🌐 English](README.md) | [简体中文](README_cn.md)

# Intelligent Search & Rescue System for Earthquake Disaster Areas Based on RDK-X5 Edge Computing and Multi-Dimensional Vision Gimbal

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-RDK--X5%20%7C%20STM32%20%7C%20Arduino-orange.svg)]()
[![AI-Core](https://img.shields.io/badge/AI--Core-BPU%20%7C%20YOLOv2-green.svg)]()
[![Language](https://img.shields.io/badge/Language-Python%20%7C%20C%2B%2B%20%7C%20HTML5-blue.svg)]()

This system is a low-latency, highly reliable, and anti-interference intelligent search and rescue robot control system designed for complex and volatile environments such as earthquake disaster zones and mine collapse sites. Adopting a decoupled frontend-backend multi-tier distributed architecture, it utilizes the **RDK-X5 (equipped with a built-in BPU acceleration core)** as the edge brain of the vehicle to achieve real-time multi-target inference on HD video streams. Through an innovative **spatiotemporal stabilization queue algorithm and an adaptive morphological filter**, it effectively filters out false positives triggered by static background hazards like scattered red bricks and gravel on wild terrain. The data link supports millisecond-level emergency stop command delivery to the lower-level MCUs (STM32/Arduino) while streaming multi-dimensional, real-time visual telemetry logs to a remote HTML5 high-tech dashboard workstation.

---

## ⚙️ Overall System Architecture Topology

The system consists of three main components: the Edge Perception Brain, Lower-Level Powertrain & Actuation, and the Panoramic Remote Control Console. They work in tight hardware-software synergy to form a complete data closed-loop:

```
                    ┌───────────────────────────────────────┐
                    │     Remote Control Console (HTML5)    │
                    └───────────────────▲───────────────────┘
                                        │ (Flask /video_feed & /check_detection Polling)
                                        ▼
┌──────────────┐    ┌───────────────────────────────────────┐   (UART 115200)    ┌───────────────────────────┐
│ USB HD Camera│───>│     RDK-X5 Edge Computing Node        │───────────────────>│Lower-Level Chassis Driver │
└──────────────┘    │  - BPU Hardware Accelerated Inference │                    │  - STM32F407 (Motor/Gimbal)│
                    │  - Adaptive Morphological Filter      │                    │  - Arduino (Hexapod Gait) │
                    │  - Audio Alert Linkage (aplay)        │                    └───────────────────────────┘
                    └───────────────────────────────────────┘
```

---

## 📂 Project Directory Structure Specification

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

## ✨ Key Technical Highlights & Algorithmic Mechanisms

### 1. BPU Hardware Acceleration & Heterogeneous Model Inference
The backend runs on the integrated BPU (Brain Processing Unit) acceleration array of the RDK-X5, loading the quantized and compiled search-and-rescue detection model `detect.bin`. By utilizing the high-throughput `hobot_dnn` library, it eliminates the computational overhead of traditional CPU inference, delivering real-time, low-latency, and low-power edge detection under a high-definition resolution of 640x640.

### 2. Innovative Spatiotemporal Stabilization Queue Algorithm & Morphological Filter
To prevent the search and rescue vehicle from triggering false positives (frequent abrupt stops) caused by scattered red bricks or orange strip backgrounds while traversing wild terrain, the system implements two lines of defense directly inside the source code:
* **Adaptive Horizontal Dummy Filter**: Computes the aspect ratio and area ratio of the bounding box. If an object is highly elongated/flat but extremely small in area, it is automatically categorized as a **ground brick artifact** and intercepted. If it is flat but possesses a substantial area, it is classified as a **trapped victim in a horizontal/lying posture** and allowed to pass.
* **Sliding Window Temporal State Machine**: Maintains a cyclic queue (`deque`) with a depth of 15 frames. A true positive detection is confirmed, and the stop command is triggered, only if the target is successfully captured in at least 11 of the last 15 frames.

### 3. Multi-Modal Synchronous Peripheral Linkage
Once a trapped victim is verified and confirmed:
* **Instruction-Level Serial Interconnect**: The backend sends a `0x05` emergency brake byte via the `/dev/ttyS1` bus at a `115200` baud rate to immediately seize vehicle locomotion control from the lower-level MCUs.
* **Instantaneous Audio Guidance**: Uses multi-threaded asynchronous Linux subprocesses (`nohup aplay`) to drive the USB sound card, broadcasting rescue guidance audio on-site. A software cooldown mechanism is implemented to prevent overlapping audio clipping.

---

## 🛠️ Production Environment Configuration & Deployment Guide

### 1. Backend Deployment (RDK-X5 Linux Node)
Since the board may operate in offline or network-isolated scenarios, it is recommended to launch the engine directly via the local terminal:

```bash
# Ensure you are in the repository root directory, then boot the inference engine
python3 RDK.py
```

### 2. Lower-Level Compilation (STM32 / Arduino)
* **STM32**: Open the `STM32F407VGT6---MOTOR.uvprojx` project at the root directory using Keil uVision5. The system is configured with advanced timers to output 4-channel dead-time PWM for four-wheel-drive DC geared motors. It integrates serial interrupts to immediately execute an emergency stop (via a closed-loop braking PID) upon receiving the `0x05` byte.
* **Arduino**: Load `arduino_chassis_main/Mech_hexapod.ino` using the Arduino IDE. Combined with multi-legged complex tripod gait kinematics, it enables obstacle-overcoming search and rescue movements.

### 3. High-Tech Workstation Dashboard Launch
Open `ui.html` at the root directory using a text editor. Modify the configuration parameters to change `BOARD_IP` to your RDK-X5's actual static local IP address, then double-click `ui.html` in any modern web browser to interface with the hardware.

---

## ⚖️ License

This project is open-sourced under the **MIT License**.
