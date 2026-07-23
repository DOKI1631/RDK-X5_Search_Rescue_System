////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Configuration
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This file contains all configuration constants, macros, and hardware settings
////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Debug Configuration - Comment out to save space
// #define DEBUG_OUTPUT  // Uncomment to enable debug output

// Debug macros to reduce code size
#ifdef DEBUG_OUTPUT
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// PROGMEM macros for storing constant strings in flash memory
#include <avr/pgmspace.h>
#define PSTR2(s) PSTR(s)
#define PRINTP(s) Serial.print(F(s))
#define PRINTLNPLN(s) Serial.println(F(s))

// Version Information
extern const char *Version;

// Hardware Configuration
#define SERVO_IIC_ADDR  (0x40)    // default servo driver IIC address
#define BeeperPin 4                // digital 4 used for beeper
#define ServoTypePin 5             // 5 is used to signal digital vs. analog servo mode
#define ServoTypeGroundPin 6       // 6 provides a ground to pull 5 low if digital servos are in use
//#define ULTRAOUTPUTPIN  A1         // TRIG for ultrasonic sensor
//#define ULTRAINPUTPIN  A0           // ECHO for ultrasonic sensor



// 左侧超声波
#define TRIG_L A2
#define ECHO_L A3

// 右侧超声波
#define TRIG_R A0
#define ECHO_R A1

// 避障安全阈值设定
#define WARNING_DIST 26  // 普通单侧避障阈值（原30cm）
#define DANGER_DIST  20  // 紧急后退阈值（原25cm）
#define OBSTACLE_CLEAR_DIST 33 // 恢复阈值（原38cm），仍比进入阈值高7cm形成滞回
#define OBSTACLE_CONFIRM_FRAMES 2 // 连续 N 帧确认后才进入普通避障

// ==========================================
// 周期性探头扫描配置（消除正前方超声波盲区）
// ==========================================
// 原理：前进时每隔 N 秒做一次短暂左-右"探头"，
//       让左右超声波轮流扫过正前方盲区。
//       频率低（4秒一次），不会造成舵机电流冲击。
//       注释掉 #define PERIODIC_SCAN 可关闭此行为。
#define PERIODIC_SCAN              // 启用周期性探头扫描
#define SCAN_CHECK_INTERVAL  6000  // 扫描间隔 (ms)，降低无障碍时的无谓转向
#define SCAN_SWEEP_DURATION   250  // 单次探头持续 (ms)，减小探头动作幅度

// ==========================================
// 直行速度与重心偏移补偿
// ==========================================
// FORWARD_PERIOD: 三脚架步态周期 (ms)，越小越快，默认 750
// DRIFT_COMPENSATION: 重心偏移补偿 (-25~+25)
//   正数 = 身体右倾，适合重心偏左的机器人
//   负数 = 身体左倾，适合重心偏右的机器人
//   0 = 不补偿
#define FORWARD_PERIOD      560   // 比600快约7%；再低会压缩MG90S落腿时间
#define DRIFT_COMPENSATION    1   // 更换故障舵机后+2轻微右偏，回调到+1作为机械修复后的直行基准
#define PITCH_COMPENSATION    0   // 六腿使用同一基础高度；实机证明+8会使前腿角度过高、前端反而偏低且更易发热
#define FRONT_KNEE_COMPENSATION  5 // 已按腿部标注确认：前腿LEG2/LEG3站立160→165，抬腿65→70

// ==========================================
// AMG8833 热成像传感器配置
// ==========================================
#define THERMAL_I2C_ADDR      0x69    // AMG8833 I2C 地址（AD0=HIGH，实际 0x69）
#define THERMAL_SDA_PIN       5       // 热成像软件I2C数据线（PD5，复用空闲超声波口）
#define THERMAL_SCL_PIN       6       // 热成像软件I2C时钟线（PD6，复用空闲超声波口）
#define HUMAN_TEMP_MIN        28.0f   // 人体热源最低温度阈值 (°C)
#define HUMAN_TEMP_MAX        42.0f   // 人体热源最高温度阈值 (°C)（提高至42°C避免传感器自热误触发）
#define THERMAL_STOP_COLS     6       // 热源覆盖 ≥6 列时，检测器标记为很近
#define THERMAL_CLOSE_COLS    4       // 热源覆盖 ≥4 列时，检测器标记为较近
#define THERMAL_CLOSE_PIXELS  12      // 热源像素 ≥12 个时，检测器标记为较近

// ★ 热成像传感器防误触发保护
#define THERMAL_WARMUP_MS      5000   // AMG8833 上电后的稳定等待时间 (ms)
#define THERMAL_BACKGROUND_DELTA 3.0f // 热点至少高于稳健背景温度 3°C
#define THERMAL_MIN_PEAK_DELTA 3.5f   // 主热斑峰值与背景的最小温差
#define THERMAL_MIN_BLOB_PIXELS 4     // 至少 4 个相邻热点，拒绝孤立坏点/反光噪声
#define THERMAL_MAX_HOT_RATIO  0.40f  // 主热斑超过画面 40% 时视为大面积热背景
#define THERMAL_CONSECUTIVE_DETECT 5  // 独立热成像连续识别 5 帧有效热斑后蜂鸣一次
#define THERMAL_REARM_CLEAR_FRAMES 3  // 热源连续消失 3 帧后重新布防，允许下一个热源再次蜂鸣

// 步态切换先让六腿落地一个短暂窗口，避免不同周期相位跳变导致两组三脚架同时抬起
#define GAIT_TRANSITION_SETTLE_MS 90

// 超声波偶发无回波时先保留最近有效距离，避免单帧超时被当成“突然清空”。
// 连续超时后仍按远距离处理（开放空间本身也会没有回波）。
#define ULTRASONIC_TIMEOUT_HOLD_FRAMES 2

// Battery Voltage Detection
#define BATTERY_PIN A7             // ADC pin for battery voltage reading
#define LED_LOW_BAT_PIN 9          // IO9 for low battery LED indicator
#define BATTERY_R1 3.3             // Upper resistor of voltage divider (kΩ)
#define BATTERY_R2 4.7             // Lower resistor of voltage divider (kΩ), connected to GND
#define VOLTAGE_DIVIDER_RATIO ((BATTERY_R1 + BATTERY_R2) / BATTERY_R2)  // Voltage divider ratio
#define LOW_BATTERY_THRESHOLD 7  // Low battery alarm threshold (V)
#define BATTERY_CHECK_INTERVAL 5000  // Battery check interval (ms)

// Beep Configuration
#define BF_ERROR  100              // deep beep for error situations
#define BD_MED    50               // medium long beep duration

// Servo Configuration
extern byte FreqMult;             // PWM frequency multiplier, use 1 for analog servos and up to about 3 for digital
#define PWMFREQUENCY (60*FreqMult)

#define SERVOMIN  (190*FreqMult)  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  (540*FreqMult)  // this is the 'maximum' pulse length count (out of 4096)

// Robot Size Configuration
#define HEXSIZE 0                  // set this to 0 for hexapod, 1 for megapod and 2 for gigapod

#if HEXSIZE == 0
#define TIMEFACTOR 8L
#define HEXAPOD
#endif

#if HEXSIZE == 1
#define TIMEFACTOR 9L              // slightly slower, 90% speed
#define MEGAPOD
#endif

#if HEXSIZE == 2
#define TIMEFACTOR 7L              // much slower
#define GIGAPOD
#endif

// Leg Configuration
#define NUM_LEGS 6
#define KNEE_OFFSET 6              // add this to a leg number to get the knee servo number
#define LEFT_START 3               // first leg that is on the left side
#define RIGHT_START 0              // first leg that is on the right side

// Leg Bit Patterns
#define ALL_LEGS      0b111111
#define LEFT_LEGS     0b000111
#define RIGHT_LEGS    0b111000
#define TRIPOD1_LEGS  0b010101
#define TRIPOD2_LEGS  0b101010
#define QUAD1_LEGS    0b001001
#define QUAD2_LEGS    0b100100
#define FRONT_LEGS    0b001100   // 实机传感器/重载端：LEG2 + LEG3
#define MIDDLE_LEGS   0b010010
#define BACK_LEGS     0b100001   // 实机后端：LEG0 + LEG5
#define NO_LEGS       0b0

// Individual Leg Bitmasks
#define LEG0 0b1
#define LEG1 0b10
#define LEG2 0b100
#define LEG3 0b1000
#define LEG4 0b10000
#define LEG5 0b100000

#define LEG0BIT  0b1
#define LEG1BIT  0b10
#define LEG2BIT  0b100
#define LEG3BIT  0b1000
#define LEG4BIT  0b10000
#define LEG5BIT  0b100000

// Leg Position Definitions
#define ISFRONTLEG(LEG) (LEG==2||LEG==3)  // 实机安装方向与原模板相差180°
#define ISMIDLEG(LEG)   (LEG==1||LEG==4)
#define ISBACKLEG(LEG)  (LEG==0||LEG==5)
#define ISFRONTKNEESERVO(LEG) (LEG==2||LEG==3) // 按实机标注确认的前腿；对应竖直舵机通道8/9
#define ISLEFTLEG(LEG)  (LEG==3||LEG==4||LEG==5)  // 与 LEFT_START=3 一致
#define ISRIGHTLEG(LEG) (LEG==0||LEG==1||LEG==2)

// Knee Angle Definitions (in degrees)
// 0°=完全折叠, 180°=完全伸直
// 越野配置：站姿更高 + 抬腿折叠更多 = 离地间隙大
#define KNEE_UP_MAX 0
#define KNEE_UP    30
#define KNEE_RELAX  60
#define KNEE_NEUTRAL 65          // 比原75°多抬10°；继续减小会显著增加负载和落腿时间
#define KNEE_CROUCH 70
#define KNEE_HALF_CROUCH 100
#define KNEE_STAND 160           // 站立时膝盖更直 → 身体更高
#define KNEE_DOWN  160           // 行走支撑腿高度同步
#define KNEE_TIPTOES 175
#define KNEE_FOLD 10

#define KNEE_SCAMPER (KNEE_NEUTRAL-20)
#define KNEE_TRIPOD_UP (KNEE_NEUTRAL-40)
#define KNEE_TRIPOD_ADJ 30
#define KNEE_RIPPLE_UP (KNEE_NEUTRAL-40)
#define KNEE_RIPPLE_DOWN (KNEE_DOWN)
#define TWITCH_ADJ 60

// Hip Angle Definitions (in degrees)
#define HIPSWING 35               // how far to swing hips on gaits like tripod or quadruped（越野加大步幅）
#define HIPSMALLSWING 10          // when in fine adjust mode how far to move hips
#define HIPSWING_RIPPLE 25
#define HIP_FORWARD_MAX 175

#define HIP_FORWARD (HIP_NEUTRAL-HIPSWING)
#define HIP_FORWARD_SMALL (HIP_NEUTRAL-HIPSMALLSWING)
#define HIP_NEUTRAL 90
#define HIP_BACKWARD (HIP_NEUTRAL+HIPSWING)
#define HIP_BACKWARD_SMALL (HIP_NEUTRAL+HIPSMALLSWING)
#define HIP_BACKWARD_MAX 0
#define HIP_FORWARD_RIPPLE (HIP_NEUTRAL-HIPSWING_RIPPLE)
#define HIP_BACKWARD_RIPPLE (HIP_NEUTRAL+HIPSWING_RIPPLE)
#define HIP_FOLD 150

// Quadruped Gait Definitions
#define FBSHIFT_QUAD 25
#define HIP_FORWARD_QUAD (HIP_FORWARD)
#define HIP_BACKWARD_QUAD (HIP_BACKWARD)
#define KNEE_QUAD_UP (KNEE_DOWN+30)
#define KNEE_QUAD_DOWN (KNEE_DOWN)
#define QUAD_CYCLE_TIME 600

// Belly Crawl Gait Definitions
#define FBSHIFT_BELLY -55
#define HIP_FORWARD_BELLY (HIP_FORWARD-10)
#define HIP_BACKWARD_BELLY (HIP_BACKWARD+10)
#define KNEE_BELLY_UP (KNEE_NEUTRAL+30)
#define KNEE_BELLY_DOWN (KNEE_NEUTRAL-30)
#define BELLY_CYCLE_TIME 600

// Special Values
#define NOMOVE (-1)               // fake value meaning this aspect of the leg (knee or hip) shouldn't move

// Gait Timing Configuration
#define TRIPOD_CYCLE_TIME 600
#define RIPPLE_CYCLE_TIME 1000
#define FIGHT_CYCLE_TIME 660

// Mode Definitions
#define MODE_WALK   'W'
#define MODE_WALK2  'X'
#define MODE_DANCE  'D'
#define MODE_DANCE2 'Y'
#define MODE_FIGHT  'F'
#define MODE_FIGHT2 'Z'
#define MODE_RECORD 'R'
#define MODE_LEG    'L'       // comes from scratch
#define MODE_GAIT   'G'       // comes from scratch
#define MODE_TRIM   'T'       // gamepad in trim mode

// Submode Definitions
#define SUBMODE_1 '1'
#define SUBMODE_2 '2'
#define SUBMODE_3 '3'
#define SUBMODE_4 '4'

// Dial Mode Definitions
#define DIALMODE_STAND 0
#define DIALMODE_ADJUST 1
#define DIALMODE_TEST 2
#define DIALMODE_DEMO 3
#define DIALMODE_RC_GRIPARM 4
#define DIALMODE_RC 5

// Battery Saver
#define BATTERYSAVER 20000       // milliseconds in stand mode before servos all detach to save power and heat buildup

// Trim Configuration
#define TRIM_ZERO 127            // this value is the midpoint of the trim range (a byte)

// Packet Processing States
#define P_WAITING_FOR_HEADER      0
#define P_WAITING_FOR_VERSION     1
#define P_WAITING_FOR_LENGTH      2
#define P_READING_DATA            3
#define P_WAITING_FOR_CHECKSUM    4
#define P_SIMPLE_WAITING_FOR_DATA 5

#define MAXPACKETDATA 48

// Raw Servo Move Types
#define RAWSERVOPOS 0
#define RAWSERVOADD 1
#define RAWSERVOSUB 2
#define RAWSERVONOMOVE 255
#define RAWSERVODETACH 254

// Gait Types
#define G_STAND 0
#define G_TURN  1
#define G_TRIPOD 2
#define G_SCAMPER 3
#define G_DANCE 4
#define G_BOOGIE 5
#define G_FIGHT 6
#define G_TEETER 7
#define G_BALLET 8

#define G_NUMGATES 9

// Dance Mode Types
#define MODETWITCH 0
#define MODESWAY 1

// Global Variables
extern byte SomeLegsUp;                      // flag to detect situations where a user rapidly switches moves
extern unsigned short ServoPos[2*NUM_LEGS];  // the last commanded position of each servo
extern unsigned short ServoTarget[2*NUM_LEGS];
extern long ServoTime[2*NUM_LEGS];           // the time that each servo was last commanded to a new position
extern byte ServoTrim[2*NUM_LEGS];           // trim values for fine adjustments to servo horn positions
extern long startedStanding;                 // the last time we started standing, or reset to -1 if we didn't stand recently
extern long LastReceiveTime;                 // last time we got a bluetooth packet
extern unsigned long LastValidReceiveTime;   // last time we got a completely valid packet including correct checksum
extern int Dialmode;                         // What's the robot potentiometer set to?
extern unsigned long SuppressScamperUntil;   // if we had to wake up the servos, suppress the power hunger scamper mode for a while

// Helper Functions
unsigned long hexmillis();  // millis that takes into account hexapod size for leg timings
void beep(int f, int t);    // beep with frequency and duration
void beep(int f);           // beep with frequency, default duration 250ms

// Battery Voltage Functions
void setupBatteryADC();     // Initialize ADC for battery voltage reading
float readBatteryVoltage(); // Read battery voltage
void checkBatteryStatus();  // Check battery status and trigger alarm if low
void updateLowBatLED();     // Update low battery LED state

#endif // CONFIG_H
