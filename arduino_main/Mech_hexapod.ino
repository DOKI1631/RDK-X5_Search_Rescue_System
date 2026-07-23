////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Main Program (Fixed & Optimized)
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This is the main program file that integrates all modules and contains
// the setup() and loop() functions.
////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "servo_control.h"
#include "leg_motion.h"
#include "gait.h"
#include "dance.h"
#include "fight.h"
#include "sensors.h"
#include "thermal.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <EEPROM.h>

// ==========================================
// 1. 函数原型声明
// ==========================================
void executeAction(int type, unsigned long duration, int dir);

// Version String
const char *Version = "#RV3r14l-independent-thermal";

// Global Variables defined in config.h
byte FreqMult = 1;   // PWM frequency multiplier
byte SomeLegsUp = 0;  // flag to detect situations where a user rapidly switches moves
unsigned short ServoPos[2*NUM_LEGS];
unsigned short ServoTarget[2*NUM_LEGS];
long ServoTime[2*NUM_LEGS];
byte ServoTrim[2*NUM_LEGS];
long startedStanding = 0;
long LastReceiveTime = 0;
unsigned long LastValidReceiveTime = 0;
int Dialmode;

// ==================== 热成像模式全局变量 ====================
bool thermalMode = false;               // 传感器初始化成功后自动开启，与 RDK-X5 独立
bool servosAvailable = false;           // PCA9685 在线标志
bool cameraStopLocked = false;          // ★ 摄像头锁定：RDK-X5识别到人后永久停车，最高优先级
bool thermalAlarmLatched = false;       // 同一热源只蜂鸣一次，热源连续消失后重新布防
float thermalPixels[THERMAL_PIXELS];    // 64 像素温度网格
HeatSourceResult thermalResult;         // 热源检测结果
unsigned long lastThermalReadTime = 0;  // 上次读取热成像的时间
bool thermalSensorOK = false;           // 热成像传感器是否正常
byte thermalConsecutiveDetect = 0;      // ★ 连续检测到热源的次数（防瞬时噪声）
byte thermalClearConsecutive = 0;       // 热源连续消失帧数，达到门限后允许下一次蜂鸣

// ==================== 左右超声波三点中值滤波 ====================
// 旧的 5 帧均值窗口初始填入 1000cm，真实障碍出现后要约 0.5 秒才降到阈值；
// 中值滤波能拒绝单次毛刺，同时在连续两帧遇障后立即响应。
#define DIST_WINDOW_SIZE 3
unsigned int distBufferL[DIST_WINDOW_SIZE] = {1000, 1000, 1000};
unsigned int distBufferR[DIST_WINDOW_SIZE] = {1000, 1000, 1000};
byte distIndexL = 0;
byte distIndexR = 0;
bool distInitializedL = false;
bool distInitializedR = false;
byte distTimeoutL = 0;
byte distTimeoutR = 0;
byte obstacleConfirmL = 0;
byte obstacleConfirmR = 0;

unsigned int median3(unsigned int a, unsigned int b, unsigned int c) {
  if (a > b) { unsigned int t = a; a = b; b = t; }
  if (b > c) { unsigned int t = b; b = c; c = t; }
  if (a > b) { unsigned int t = a; a = b; b = t; }
  return b;
}

unsigned int filterDistance(unsigned int raw, unsigned int buffer[DIST_WINDOW_SIZE],
                            byte &index, bool &initialized, byte &timeoutCount) {
  if (raw == 0) {
    if (initialized && timeoutCount < ULTRASONIC_TIMEOUT_HOLD_FRAMES) {
      timeoutCount++;
      return median3(buffer[0], buffer[1], buffer[2]);
    }
    raw = 1000;
  } else {
    timeoutCount = 0;
  }
  if (!initialized) {
    for (int i = 0; i < DIST_WINDOW_SIZE; i++) buffer[i] = raw;
    initialized = true;
  } else {
    buffer[index] = raw;
    index = (index + 1) % DIST_WINDOW_SIZE;
  }
  return median3(buffer[0], buffer[1], buffer[2]);
}

// 左侧超声波滤波刷新
unsigned int filterDistanceL(unsigned int raw) {
  return filterDistance(raw, distBufferL, distIndexL, distInitializedL, distTimeoutL);
}

// 右侧超声波滤波刷新
unsigned int filterDistanceR(unsigned int raw) {
  return filterDistance(raw, distBufferR, distIndexR, distInitializedR, distTimeoutR);
}

// Trim State Variables
byte TrimInEffect = 1;
byte TrimCurLeg = 0;
byte TrimPose = 0;

// Servo Driver Instance
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(SERVO_IIC_ADDR);

// Helper function implementations from config.h
unsigned long hexmillis() {
  unsigned long m = (millis() * TIMEFACTOR)/10L;
  return m;
}

void beep(int f, int t) {
  if (f > 0 && t > 0) {
    tone(BeeperPin, f, t);
  } else {
    noTone(BeeperPin);
  }
}

void beep(int f) {
  beep(f, 250);
}

// 高响度警报
void loudAlert(int times) {
  for (int i = 0; i < times; i++) {
    tone(BeeperPin, 3000, 80);
    delay(100);
    tone(BeeperPin, 3500, 80);
    delay(100);
    tone(BeeperPin, 2700, 80);
    delay(100);
  }
  noTone(BeeperPin);
}

// ==================== Battery Voltage Detection ====================
static unsigned long lastBatteryCheckTime = 0;
static bool lowBatteryAlarmActive = false;
static unsigned long lastBatteryAlarmTime = 0;
static unsigned long ledToggleTime = 0;
static bool ledState = false;

#define FILTER_WINDOW_SIZE 10
static int adcFilterBuffer[FILTER_WINDOW_SIZE] = {0};
static int filterIndex = 0;
static bool bufferFilled = false;

void setupBatteryADC() {
  pinMode(BATTERY_PIN, INPUT);
  pinMode(LED_LOW_BAT_PIN, OUTPUT);
  digitalWrite(LED_LOW_BAT_PIN, LOW);
}

int slidingFilterADC(int newValue) {
  adcFilterBuffer[filterIndex] = newValue;
  filterIndex = (filterIndex + 1) % FILTER_WINDOW_SIZE;

  if (!bufferFilled && filterIndex == 0) {
    bufferFilled = true;
  }

  long sum = 0;
  int count = bufferFilled ? FILTER_WINDOW_SIZE : filterIndex;
  if (count == 0) count = 1;

  for (int i = 0; i < count; i++) {
    sum += adcFilterBuffer[i];
  }
  return (int)(sum / count);
}

float readBatteryVoltage() {
  int rawAdcValue = analogRead(BATTERY_PIN);
  int filteredAdcValue = slidingFilterADC(rawAdcValue);
  float voltage = (filteredAdcValue / 1023.0) * 5.0 * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

void checkBatteryStatus() {
  unsigned long currentTime = millis();
  if (currentTime - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL) {
    lastBatteryCheckTime = currentTime;
    int rawAdcValue = analogRead(BATTERY_PIN);
    int filteredAdcValue = slidingFilterADC(rawAdcValue);
    float batteryVoltage = (filteredAdcValue / 1023.0) * 5.0 * VOLTAGE_DIVIDER_RATIO;

    Serial.print(F("ADC:"));
    Serial.print(rawAdcValue);
    Serial.print(F(","));
    Serial.print(filteredAdcValue);
    Serial.print(F(","));
    Serial.print(batteryVoltage, 2);
    Serial.println(F("V"));

    if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
      lowBatteryAlarmActive = true;
      if (currentTime - lastBatteryAlarmTime >= 5000) {
        lastBatteryAlarmTime = currentTime;
        // tone() 自身非阻塞；不要在三脚架抬腿相位里执行多段 delay() 扫频。
        beep(1200, 200);
      }
    } else {
      lowBatteryAlarmActive = false;
      digitalWrite(LED_LOW_BAT_PIN, LOW);
    }
  }
}

void updateLowBatLED() {
  if (lowBatteryAlarmActive) {
    unsigned long currentTime = millis();
    if (currentTime - ledToggleTime >= 200) {
      ledToggleTime = currentTime;
      ledState = !ledState;
      digitalWrite(LED_LOW_BAT_PIN, ledState ? HIGH : LOW);
    }
  }
}

unsigned long ReportTime = 0;
unsigned long SuppressModesUntil = 0;
short priorDialMode = -1;

void setup() {
  // 高速串口避免状态输出挤占 100ms 感知/步态刷新窗口。
  Serial.begin(115200);
  Serial.println();
  Serial.println(Version);

  // RDK-X5 通信：D2 (HCRX) 轮询串口 115200 baud
  pinMode(2, INPUT_PULLUP);
  Serial.println(F("RDK-X5: D2 polled 115200 baud"));

  pinMode(BeeperPin, OUTPUT);
  beep(200);

  if (EEPROM.read(12) == 'V') {
    Serial.print(F("TRIMS:"));
    for (int i = 0; i < NUM_LEGS*2; i++) {
      ServoTrim[i] = EEPROM.read(i);
      Serial.print(ServoTrim[i]-TRIM_ZERO); Serial.print(F(","));
    }
    Serial.println();
  } else {
    Serial.println(F("TRIMS:unset"));
    for (int i = 0; i < NUM_LEGS*2; i++) {
      ServoTrim[i] = TRIM_ZERO;
    }
  }

  pinMode(ServoTypeGroundPin, OUTPUT);
  digitalWrite(ServoTypeGroundPin, LOW);
  pinMode(ServoTypePin, INPUT_PULLUP);

  setupBatteryADC();

  // 启动前电压诊断
  {
    analogRead(BATTERY_PIN); delay(10);
    analogRead(BATTERY_PIN); delay(10);
    int rawAdc = analogRead(BATTERY_PIN);
    float battV = (rawAdc / 1023.0) * 5.0 * VOLTAGE_DIVIDER_RATIO;
    Serial.print(F("BATT: ")); Serial.print(battV, 2); Serial.println(F("V"));
    Serial.flush();
  }

  // 初始化双超声波引脚（TRIG 输出 / ECHO 输入），避免在 readDistance 热路径中重复设置
  initSensors();

  delay(300);
  delay(250);

  if (digitalRead(ServoTypePin) == LOW) {
    FreqMult = 3-FreqMult;
  }

  for (int i = 0; i < FreqMult; i++) {
    beep(800, 50);
    delay(100);
  }

  resetServoDriver();
  Wire.beginTransmission(SERVO_IIC_ADDR);
  byte pcaStatus = Wire.endTransmission();
  servosAvailable = (pcaStatus == 0);
  if (servosAvailable) {
    Serial.println(F("PCA9685: OK at 0x40"));
  } else {
    Serial.print(F("ERROR: PCA9685 not responding, I2C status="));
    Serial.println(pcaStatus);
    beep(150, 500); // 只报一次，不在loop中连续鸣叫
  }
  delay(250);

  // 初始化 AMG8833 热成像传感器（与舵机驱动共用 I2C 总线）
  thermalSensorOK = initThermal();
  if (!thermalSensorOK) {
    thermalMode = false;
    Serial.println(F("WARN: Thermal sensor not found. Thermal mode disabled."));
  } else {
    thermalMode = true;
    Serial.println(F("THERMAL MODE: AUTO ON (independent from RDK)"));
  }

  // 安全初始化：ServoPos 预填站立角度，防止任何函数 commitServos 时把未设置的舵机命令到 0°
  for (int i = 0; i < NUM_LEGS; i++) {
    ServoPos[i] = HIP_NEUTRAL;
    ServoPos[i + KNEE_OFFSET] = KNEE_STAND;
  }

  if (servosAvailable) {
    standGradual();  // 渐进站立，分 3 组避免 12 舵机同时启动的电流尖峰 → 欠压复位
    delay(300);
    beep(400);
  }
  yield();
}

// ========================================================
// 1. 方向感知增强状态枚举
// ========================================================
enum RobotState {
  STATE_FORWARD,        // 前进
  STATE_BACKWARD,       // 直退
  STATE_TURN_LEFT,      // 左转（避障）
  STATE_TURN_RIGHT,     // 右转（避障）
  STATE_RETREAT_LEFT,   // 紧急第一阶段：直退，结束后左转
  STATE_RETREAT_RIGHT,  // 紧急第一阶段：直退，结束后右转
  STATE_ESCAPE,         // 卡死后大角度逃脱
  STATE_STOP            // RDK-X5 UART 强制停车
};

RobotState currentState = STATE_FORWARD;
RobotState previousState = STATE_FORWARD; // 上一个状态（用于判断是否刚切换）
unsigned long stateEndTime = 0;
unsigned long lastSensorTime = 0;

// 暂存全局的距离值
unsigned int distL = 1000;
unsigned int distR = 1000;

// 周期性探头扫描变量（消除正前方超声波盲区）
static bool scanInProgress = false;
static int  scanStep = 0;
static unsigned long lastScanCheck = 0;

// ==================== 方向感知与后向记忆系统 ====================
// 核心思想：
//   没有罗盘也能"知道方向"——记住自己走过的安全路径。
//   前进时记录传感器读数，撤退时向记忆中最空旷的方向退。

#define DIR_MEM_SIZE 8                // ATmega328P SRAM有限；8条×750ms仍覆盖约6秒
#define SAFE_CORRIDOR_DIST 45         // 超过此距离视为"安全走廊"（原50cm）
#define OBSTACLE_STREAK_MAX 4         // 连续遇障 N 次触发逃脱模式
#define RETREAT_DURATION 800          // 撤退持续时间 (ms)
#define EMERGENCY_TURN_DURATION 900   // 完整直退后，向安全侧转弯的时间
#define TURN_DURATION 1200            // 转弯最大持续时间 (ms)
#define ESCAPE_DURATION 2500          // 逃脱模式持续时间 (ms)

// 方向记忆条目
struct DirMemory {
  unsigned long timestamp;
  unsigned int distL;
  unsigned int distR;
  byte moveType;  // 0=前进, 1=后退, 2=左转, 3=右转
};

DirMemory pathHistory[DIR_MEM_SIZE];
byte pathHistIdx = 0;
byte pathHistCount = 0;

// 方向追踪
unsigned long lastClearForwardTime = 0;  // 最后一次"前方完全畅通"的时间
unsigned long totalClearForwardMs = 0;   // 最近连续畅通前进的累计时间
unsigned long lastMemoryRecordTime = 0;  // 750ms 采样一次，8条约覆盖6秒
byte obstacleStreak = 0;                 // 连续遇障次数
int retreatBias = 0;                     // 当前撤退偏向 (-1=偏左, 0=直退, 1=偏右)
unsigned long streakResetTime = 0;       // 卡死计数重置时间

// ---- 自动漂移补偿 ----
int autoDriftCompensation = 0;           // 运行时自动漂移补偿，保守限制为 -1..+1
unsigned int driftLeftCount = 0;         // 左侧遇障计数
unsigned int driftRightCount = 0;        // 右侧遇障计数
unsigned long lastDriftAdjustTime = 0;   // 上次漂移调整时间
unsigned long lastUartStopCmd = 0;       // 最后一次收到 RDK-X5 0x05 停车指令的时间
bool uartStopActive = false;             // RDK-X5 停车激活标志（独立于状态机，不可被任何代码路径绕过）
RobotState preUartStopState = STATE_FORWARD; // 记录 UART 停车前的状态，用于自动恢复

// ---- 方向记忆函数 ----
void recordToMemory(unsigned int dL, unsigned int dR, byte moveType) {
  pathHistory[pathHistIdx].timestamp = millis();
  pathHistory[pathHistIdx].distL = dL;
  pathHistory[pathHistIdx].distR = dR;
  pathHistory[pathHistIdx].moveType = moveType;
  pathHistIdx = (pathHistIdx + 1) % DIR_MEM_SIZE;
  if (pathHistCount < DIR_MEM_SIZE) pathHistCount++;
}

// 在历史记忆中查找最安全的撤退方向
// 返回: -1 = 左后方更安全, 0 = 正后方安全(直退), 1 = 右后方更安全
int computeRetreatBias() {
  unsigned long now = millis();
  unsigned long sumLeft = 0;
  unsigned long sumRight = 0;
  byte sampleCount = 0;

  // 只统计最近前进轨迹的平均读数。最大值很容易被单次超声波无回波污染。
  for (int i = 0; i < pathHistCount; i++) {
    if (now - pathHistory[i].timestamp > 6000) continue; // 超过 6 秒的记忆过期
    if (pathHistory[i].moveType != 0) continue;
    sumLeft += (pathHistory[i].distL > 200U) ? 200U : pathHistory[i].distL;
    sumRight += (pathHistory[i].distR > 200U) ? 200U : pathHistory[i].distR;
    sampleCount++;
  }

  if (sampleCount < 2) return 0;
  int diff = (int)(sumLeft / sampleCount) - (int)(sumRight / sampleCount);

  if (diff > 15) {
    return -1; // 左边历史上更空旷 → 向左后方退
  } else if (diff < -15) {
    return 1;  // 右边历史上更空旷 → 向右后方退
  } else {
    return 0;  // 两边差不多 → 直退（原路返回）
  }
}

// 检查路径记忆中正后方是否安全（最近前进时读数是否空旷）
bool isRearPathSafe() {
  unsigned long now = millis();
  byte safeSamples = 0;
  for (int i = 0; i < pathHistCount; i++) {
    // 找最近 4 秒内的前进记录
    if (now - pathHistory[i].timestamp > 4000) continue;
    if (pathHistory[i].moveType != 0) continue; // 只看前进记录

    // 前进时两侧都超过安全走廊阈值才算后方安全
    if (pathHistory[i].distL > SAFE_CORRIDOR_DIST &&
        pathHistory[i].distR > SAFE_CORRIDOR_DIST) {
      safeSamples++;
    }
  }
  return safeSamples >= 2;
}

void loop() {
  // 心跳：每 3 秒一次，确认 loop 在运行
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.println(F("HB"));
  }

  // 1. 核心系统健康监控（保持无阻塞高频刷新）
  checkBatteryStatus();
  updateLowBatLED();

  // 舵机驱动不在线则跳过所有 I2C 操作
  if (!servosAvailable) {
    delay(50);
    return;
  }

  checkForServoSleep();
  checkForCrashingHips();

  unsigned long currentMillis = millis();
  bool sensorUpdated = false;

  // UART 停车自动恢复：5 秒内未收到 0x05 → 人员已离开 → 清除标志
  // ★ 修复：恢复 UART 停车前的状态，而不是盲目进入 FORWARD
  // UART 停车自动恢复：5 秒内未收到 0x05 → 人员已离开 → 清除标志
  // ★ 摄像头锁定（最高优先级）：cameraStopLocked 时永久停车，不自动恢复
  if (uartStopActive && (currentMillis - lastUartStopCmd > 5000)) {
    // 摄像头锁定 → 永久停车，不重置 uartStopActive
    if (cameraStopLocked) {
      currentState = STATE_STOP;
      stateEndTime = currentMillis + 999999;
      lastUartStopCmd = currentMillis;  // 续期，每 5 秒打印一次
      Serial.println(F("[RDK] CAMERA LOCKED - permanent stop, no auto-recovery"));
    }
    else {
      uartStopActive = false;
      lastUartStopCmd = 0;
      currentState = STATE_FORWARD;
      stateEndTime = currentMillis;
      beep(400, 100);
      Serial.println(F("[RDK] Auto-recovered -> FORWARD (5s no 0x05)"));
    }
  }

  // 每 2 秒打印一次 UART 停车状态
  static unsigned long lastUartStatusPrint = 0;
  if (currentMillis - lastUartStatusPrint >= 2000) {
    lastUartStatusPrint = currentMillis;
    Serial.print(F("[RDK] STATUS: "));
    Serial.print(uartStopActive ? F("STOP ACTIVE") : F("waiting for 0x05"));
    Serial.print(F(" | pin D2="));
    Serial.println((PIND & 0x04) ? F("HIGH") : F("LOW"));
  }

  // ========================================================
  // 2. 定时感知层：每 100ms 读一次传感器，经三点中值滤波
  //    【关键优化】：
  //    - pulseIn 超时 8ms（约 1.36m 探测范围）
  //    - 传感器间 delay 3ms
  //    - 原始读数经 3 帧中值滤波，并做连续帧确认
  //    - 单次传感器迭代阻塞 ~19ms
  // ========================================================
  if (currentMillis - lastSensorTime >= 100) {
    lastSensorTime = currentMillis;

    unsigned int rawL = readDistance(TRIG_L, ECHO_L);
    delay(3);
    unsigned int rawR = readDistance(TRIG_R, ECHO_R);

    distL = filterDistanceL(rawL);
    distR = filterDistanceR(rawR);
    sensorUpdated = true;

    // 进入阈值与退出阈值分离（26/33cm），消除临界距离抖动。
    if (distL < WARNING_DIST) {
      if (obstacleConfirmL < OBSTACLE_CONFIRM_FRAMES) obstacleConfirmL++;
    } else if (distL > OBSTACLE_CLEAR_DIST) {
      obstacleConfirmL = 0;
    }
    if (distR < WARNING_DIST) {
      if (obstacleConfirmR < OBSTACLE_CONFIRM_FRAMES) obstacleConfirmR++;
    } else if (distR > OBSTACLE_CLEAR_DIST) {
      obstacleConfirmR = 0;
    }

#ifdef DEBUG_OUTPUT
    Serial.print("L:"); Serial.print(distL);
    Serial.print(" | R:"); Serial.println(distR);
#endif

    // ---- 热成像传感器读取（仅在热成像模式下） ----
    if (thermalMode && thermalSensorOK) {
      // ★ 预热检查：传感器上电后需要3秒稳定，跳过预热期内的读数
      if (!isThermalWarmedUp()) {
        // 预热中，不读取
      } else {
        readThermalPixels(thermalPixels);
        thermalResult = detectHeatSource(thermalPixels);
        lastThermalReadTime = currentMillis;

        // ★ 噪声过滤：超过一半像素都是"热点"→ 传感器噪声，强制忽略
        if (thermalResult.detected) {
          float hotRatio = (float)thermalResult.hotPixelCount / (float)THERMAL_PIXELS;
          if (hotRatio > THERMAL_MAX_HOT_RATIO) {
            thermalResult.detected = false;
            thermalConsecutiveDetect = 0;
            if (thermalAlarmLatched &&
                thermalClearConsecutive < THERMAL_REARM_CLEAR_FRAMES) {
              thermalClearConsecutive++;
              if (thermalClearConsecutive >= THERMAL_REARM_CLEAR_FRAMES) {
                thermalAlarmLatched = false;
                thermalClearConsecutive = 0;
                Serial.println(F("THERMAL: REARMED"));
              }
            }
#ifdef DEBUG_OUTPUT
            Serial.print(F("THERM: NOISE "));
            Serial.print(hotRatio * 100, 0);
            Serial.println(F("% hot - ignoring"));
#endif
          } else {
            thermalClearConsecutive = 0;
            // 热成像独立累计确认帧，不读取或修改 RDK 停车状态。
            if (!thermalAlarmLatched) {
              thermalConsecutiveDetect++;
              if (thermalConsecutiveDetect > THERMAL_CONSECUTIVE_DETECT)
                thermalConsecutiveDetect = THERMAL_CONSECUTIVE_DETECT;
            } else {
              thermalConsecutiveDetect = 0;
            }
          }
        } else {
          // “连续”必须是真连续：任一无效帧立即清零，不能靠历史计数凑够。
          thermalConsecutiveDetect = 0;
          if (thermalAlarmLatched &&
              thermalClearConsecutive < THERMAL_REARM_CLEAR_FRAMES) {
            thermalClearConsecutive++;
            if (thermalClearConsecutive >= THERMAL_REARM_CLEAR_FRAMES) {
              thermalAlarmLatched = false;
              thermalClearConsecutive = 0;
              Serial.println(F("THERMAL: REARMED"));
            }
          }
        }

#ifdef DEBUG_OUTPUT
        if (thermalResult.detected) {
          Serial.print(F("THERM: Tmax=")); Serial.print(thermalResult.maxTemp, 1);
          Serial.print(F("°C N=")); Serial.print(thermalResult.hotPixelCount);
          Serial.print(F(" X=")); Serial.print(thermalResult.centerX);
          Serial.print(F(" Dir=")); Serial.print(thermalResult.directionX);
          Serial.print(F(" Cols=")); Serial.print(thermalResult.hotColumns);
          Serial.print(F(" dT=")); Serial.print(thermalResult.peakDelta, 1);
          Serial.print(F(" C=")); Serial.println(thermalConsecutiveDetect);
        } else {
          Serial.println(F("THERM: No heat source"));
        }
#endif
      }
    }
  }

  // pulseIn、热成像 I2C 和可选日志都会消耗时间；后续状态时限必须使用新时间。
  currentMillis = millis();

  // ========================================================
  // 3. 方向感知记录：每次传感器读取后写入方向记忆
  // ========================================================
  if (sensorUpdated) {
    byte moveType;
    switch (currentState) {
      case STATE_FORWARD:       moveType = 0; break;
      case STATE_BACKWARD:
      case STATE_RETREAT_LEFT:
      case STATE_RETREAT_RIGHT: moveType = 1; break;
      case STATE_TURN_LEFT:     moveType = 2; break;
      case STATE_TURN_RIGHT:    moveType = 3; break;
      default:                  moveType = 0; break;
    }
    if (currentMillis - lastMemoryRecordTime >= 750) {
      lastMemoryRecordTime = currentMillis;
      recordToMemory(distL, distR, moveType);
    }

    // 前进且前方畅通时，累计"安全走廊"时间
    if (currentState == STATE_FORWARD && distL > SAFE_CORRIDOR_DIST && distR > SAFE_CORRIDOR_DIST) {
      if (lastClearForwardTime == 0) lastClearForwardTime = currentMillis;
      totalClearForwardMs = currentMillis - lastClearForwardTime;
    } else if (currentState != STATE_FORWARD || distL < SAFE_CORRIDOR_DIST || distR < SAFE_CORRIDOR_DIST) {
      // 不再畅通，重置
      lastClearForwardTime = 0;
      totalClearForwardMs = 0;
    }
  }

  // ========================================================
  // 3.5 自动漂移补偿追踪：记录单侧遇障不对称性
  // ========================================================
  if (sensorUpdated && currentState == STATE_FORWARD) {
    if (obstacleConfirmL >= OBSTACLE_CONFIRM_FRAMES &&
        obstacleConfirmR < OBSTACLE_CONFIRM_FRAMES) {
      driftLeftCount++;   // 左侧遇障、右侧通畅 → 机器人可能偏左
    } else if (obstacleConfirmR >= OBSTACLE_CONFIRM_FRAMES &&
               obstacleConfirmL < OBSTACLE_CONFIRM_FRAMES) {
      driftRightCount++;  // 右侧遇障、左侧通畅 → 机器人可能偏右
    }
  }

  // 每 10 秒自动调整漂移补偿（从 30s 提速，更快响应偏航）
  if (currentMillis - lastDriftAdjustTime > 10000) {
    lastDriftAdjustTime = currentMillis;
    int diff = (int)driftLeftCount - (int)driftRightCount;
    if (diff > 5) {
      // 超声波只能反映哪侧更常遇障，不能直接测量航向；限制为小幅微调，
      // 避免场地边界/障碍物把静态直行补偿推得过头。
      autoDriftCompensation = constrain(autoDriftCompensation + 1, -1, 1);
      Serial.print(F("DRIFT: Left bias, comp="));
      Serial.println(autoDriftCompensation);
    } else if (diff < -5) {
      autoDriftCompensation = constrain(autoDriftCompensation - 1, -1, 1);
      Serial.print(F("DRIFT: Right bias, comp="));
      Serial.println(autoDriftCompensation);
    }
    // 衰减历史计数以适应环境变化
    driftLeftCount /= 2;
    driftRightCount /= 2;
  }

  // ========================================================
  // 4. 卡死检测：连续多次遇障 → 触发逃脱
  // ========================================================
  // 只在前进且两侧通畅时重置卡死计数（非前进状态保持计数不丢失）
  if (currentState == STATE_FORWARD && distL > WARNING_DIST && distR > WARNING_DIST) {
    if (currentMillis - streakResetTime > 8000) {
      obstacleStreak = 0;
      streakResetTime = currentMillis;
    }
  } else if (currentState != STATE_FORWARD) {
    // 非前进状态：保持计时器不过期，确保卡死计数能累积到逃脱阈值
    streakResetTime = currentMillis;
  }

  // ========================================================
  // 5. 智能决策层（带方向感知的状态机）
  //    RDK 锁定后跳过普通避障；热成像只在原地进行检测反馈
  // ========================================================
  if (cameraStopLocked) {
    // locked - do nothing
  }
  else if (currentMillis >= stateEndTime) {
    previousState = currentState;  // 记录切换前的状态
    bool leftObstacle = (obstacleConfirmL >= OBSTACLE_CONFIRM_FRAMES);
    bool rightObstacle = (obstacleConfirmR >= OBSTACLE_CONFIRM_FRAMES);
    bool immediateDanger = (distL < DANGER_DIST || distR < DANGER_DIST);

#ifdef PERIODIC_SCAN
    // 扫描只是低优先级探测动作；一旦确认障碍，立即交还给避障状态机。
    if (scanInProgress && (leftObstacle || rightObstacle || immediateDanger)) {
      scanInProgress = false;
      scanStep = 0;
      lastScanCheck = currentMillis;
    }
#endif

    // ---- 紧急撤退第二阶段：直退完成后，再按已选安全方向转弯 ----
    if (currentState == STATE_RETREAT_LEFT || currentState == STATE_RETREAT_RIGHT) {
      bool turnLeftAfterBack = (currentState == STATE_RETREAT_LEFT);
      currentState = turnLeftAfterBack ? STATE_TURN_LEFT : STATE_TURN_RIGHT;
      stateEndTime = currentMillis + EMERGENCY_TURN_DURATION;
      Serial.println(turnLeftAfterBack ?
                     F(" -> Emergency phase 2: TURN LEFT") :
                     F(" -> Emergency phase 2: TURN RIGHT"));
    }
    // ---- 逃脱模式 ----
    else if (obstacleStreak >= OBSTACLE_STREAK_MAX) {
      // 大角度转向寻找新出路
      Serial.println("!! [STUCK] Escape mode activated!");
      // 根据历史上的空旷方向决定逃脱转向
      int bias = computeRetreatBias();
      if (bias <= 0) {
        currentState = STATE_ESCAPE;
        retreatBias = -1; // 左转逃脱
        Serial.println(" -> Escape: turn LEFT to find clear path");
      } else {
        currentState = STATE_ESCAPE;
        retreatBias = 1;  // 右转逃脱
        Serial.println(" -> Escape: turn RIGHT to find clear path");
      }
      stateEndTime = currentMillis + ESCAPE_DURATION;
      obstacleStreak = 0; // 重置卡死计数
    }
    // ---- 紧急贴脸：两侧均确认有障碍，或任一侧 < 20cm → 必须先后退 ----
    else if ((leftObstacle && rightObstacle) || immediateDanger) {
      beep(800, 100);
      obstacleStreak++; // 累加卡死计数

      // 先根据当前近距读数选“远离障碍”的转向：左边更近→后退后右转；
      // 右边更近→后退后左转。两侧接近时才使用最近路径记忆。
      const int sideMargin = 5;
      if (distL + sideMargin < distR) {
        retreatBias = 1;   // 左侧障碍更近，选择右转
      } else if (distR + sideMargin < distL) {
        retreatBias = -1;  // 右侧障碍更近，选择左转
      } else {
        retreatBias = computeRetreatBias();
        if (retreatBias == 0) retreatBias = (obstacleStreak & 1) ? 1 : -1;
      }

      // RETREAT_LEFT/RIGHT 在本版本只表示“直退后准备向哪边转”，
      // 动作层在第一阶段一律执行纯直退，不再交替后退/转弯。
      currentState = (retreatBias < 0) ? STATE_RETREAT_LEFT : STATE_RETREAT_RIGHT;
      Serial.println(retreatBias < 0 ?
                     F(" -> Emergency phase 1: BACKWARD, then LEFT") :
                     F(" -> Emergency phase 1: BACKWARD, then RIGHT"));
      stateEndTime = currentMillis + RETREAT_DURATION;
      streakResetTime = currentMillis;
    }
    // ---- 仅左侧有障碍 → 右转 ----
    else if (leftObstacle) {
      obstacleStreak++;
      currentState = STATE_TURN_RIGHT;
      stateEndTime = currentMillis + TURN_DURATION;
      Serial.print(" -> Turn RIGHT (left obstacle at "); Serial.print(distL); Serial.println("cm)");
      streakResetTime = currentMillis;
    }
    // ---- 仅右侧有障碍 → 左转 ----
    else if (rightObstacle) {
      obstacleStreak++;
      currentState = STATE_TURN_LEFT;
      stateEndTime = currentMillis + TURN_DURATION;
      Serial.print(" -> Turn LEFT (right obstacle at "); Serial.print(distR); Serial.println("cm)");
      streakResetTime = currentMillis;
    }
    // ---- 前方畅通 → 前进 ----
    else {
      currentState = STATE_FORWARD;
      stateEndTime = currentMillis + 100; // 每 100ms 重新评估
      // 连续畅通时缓慢降低卡死计数（每 200ms 减 1，避免短暂畅通就清零）
      if (obstacleStreak > 0 && (currentMillis % 200 < 100)) obstacleStreak--;
    }

    // ========================================================
    // 5.5 周期性中心盲区探头扫描
    // ========================================================
#ifdef PERIODIC_SCAN
    // 撤退/后退/逃脱时取消进行中的扫描
    if (scanInProgress && (currentState == STATE_RETREAT_LEFT ||
                           currentState == STATE_RETREAT_RIGHT ||
                           currentState == STATE_BACKWARD ||
                           currentState == STATE_ESCAPE)) {
      scanInProgress = false;
      scanStep = 0;
      lastScanCheck = currentMillis;
    }
    // 扫描序列：先左后右
    if (scanInProgress && currentState == STATE_FORWARD) {
      if (scanStep == 0) {
        scanStep = 1;
        currentState = STATE_TURN_RIGHT;
        stateEndTime = currentMillis + SCAN_SWEEP_DURATION;
      } else {
        scanInProgress = false;
        scanStep = 0;
        lastScanCheck = currentMillis;
      }
    }
    // 触发新扫描（仅在两侧安全时，避免扫描干扰紧急避障）
    if (currentState == STATE_FORWARD && !scanInProgress &&
        (currentMillis - lastScanCheck > SCAN_CHECK_INTERVAL) &&
        distL > OBSTACLE_CLEAR_DIST && distR > OBSTACLE_CLEAR_DIST) {
      scanInProgress = true;
      scanStep = 0;
      currentState = STATE_TURN_LEFT;
      stateEndTime = currentMillis + SCAN_SWEEP_DURATION;
    }
#endif
  }
  else {
    // ========================================================
    // 6. Smart Interrupt：动作执行中持续监测
    // ========================================================
    if (currentState == STATE_TURN_RIGHT || currentState == STATE_TURN_LEFT) {

      // 打断 1：两边都安全了 → 立刻恢复前进
      if (!scanInProgress &&
          distL > OBSTACLE_CLEAR_DIST && distR > OBSTACLE_CLEAR_DIST) {
        Serial.println(" -> [Smart Jump] Path cleared! Forward!");
        currentState = STATE_FORWARD;
        stateEndTime = currentMillis;
      }
      // 打断 2：突然出现近距离危险 → 紧急后退
      if (distL < DANGER_DIST || distR < DANGER_DIST) {
        Serial.println(" -> [Emergency] Danger close! Retreat!");
#ifdef PERIODIC_SCAN
        scanInProgress = false;
        scanStep = 0;
        lastScanCheck = currentMillis;
#endif
        retreatBias = (distL > distR) ? -1 : 1;
        currentState = (retreatBias < 0) ? STATE_RETREAT_LEFT : STATE_RETREAT_RIGHT;
        stateEndTime = currentMillis + RETREAT_DURATION;
      }
    }

    // 普通直退状态可在前方重新畅通后提前结束；紧急两阶段撤退不可提前打断。
    if (currentState == STATE_BACKWARD &&
        distL > 35 && distR > 35) {
      // 撤退途中前方已经重新畅通，尽早停止撤退
      Serial.println(" -> [Early End] Retreat complete, path ahead clear");
      currentState = STATE_FORWARD;
      stateEndTime = currentMillis;
    }
  }

  // ========================================================
  // 6.5 热成像模式
  //     与 RDK 完全独立：连续确认热源后只蜂鸣和输出，不改变机器人运动状态。
  // ========================================================
  if (thermalMode && !thermalAlarmLatched &&
      thermalResult.detected &&
      thermalConsecutiveDetect >= THERMAL_CONSECUTIVE_DETECT) {
    thermalAlarmLatched = true;
    thermalClearConsecutive = 0;
    beep(2600, 350);
    Serial.print(F("THERMAL: HEAT CONFIRMED Tmax="));
    Serial.print(thermalResult.maxTemp, 1);
    Serial.print(F("C dT="));
    Serial.print(thermalResult.peakDelta, 1);
    Serial.print(F(" N="));
    Serial.println(thermalResult.hotPixelCount);
  }

  // ========================================================
  // 6.6 停止状态最终保护门
  // ========================================================
  if (currentState == STATE_STOP) {
    stateEndTime = currentMillis + 999999;
  }

  // ========================================================
  // 6.7 RDK-X5 UART 停车最终强制门（独立于状态机，任何代码路径均不可绕过）
  // ========================================================
  // uartStopActive 是独立布尔标志，仅在收到 0x05 时置位，仅在 5s 无 0x05 时清除
  // 此处是动作执行前的最后一道检查——即使前面任何代码修改了 currentState
  // （状态机/热成像演示/PERIODIC_SCAN/Smart Interrupt），都会被强制覆盖
  if (cameraStopLocked || uartStopActive) {
    currentState = STATE_STOP;
    stateEndTime = currentMillis + 999999;
  }

  // ========================================================
  // 7. 动作执行层
  // ========================================================
  // 不同步态函数/周期都用全局时钟取模。若直接切换，新的相位可能抬起当前
  // 唯一着地的三条腿，形成“六腿同时抬起”的瞬时趴下。运动配置变化前先让
  // 六腿落地，并等待一个很短的稳定窗口。
  int desiredMotionProfile = 0;
  switch (currentState) {
    case STATE_FORWARD:
      desiredMotionProfile = 1;
      break;
    case STATE_BACKWARD:      desiredMotionProfile = 3; break;
    case STATE_TURN_LEFT:     desiredMotionProfile = 4; break;
    case STATE_TURN_RIGHT:    desiredMotionProfile = 5; break;
    case STATE_RETREAT_LEFT:  desiredMotionProfile = 6; break;
    case STATE_RETREAT_RIGHT: desiredMotionProfile = 7; break;
    case STATE_ESCAPE:        desiredMotionProfile = (retreatBias < 0) ? 8 : 9; break;
    default:                  desiredMotionProfile = 0; break;
  }

  static int lastMotionProfile = -1;
  static unsigned long gaitSettleUntil = 0;
  if (desiredMotionProfile > 0 && desiredMotionProfile != lastMotionProfile) {
    transactServos();
    setLeg(ALL_LEGS, NOMOVE, KNEE_DOWN, 0);
    commitServos();
    gaitSettleUntil = millis() + GAIT_TRANSITION_SETTLE_MS;
    lastMotionProfile = desiredMotionProfile;
  } else if (desiredMotionProfile == 0) {
    lastMotionProfile = 0;
  }

  if (desiredMotionProfile > 0 && (long)(millis() - gaitSettleUntil) < 0) {
    // 保持六腿着地；下一轮再进入新步态。
  } else switch (currentState) {
    case STATE_FORWARD:
    {
      int totalLean = DRIFT_COMPENSATION + autoDriftCompensation;
      // 按实机标注确认前腿为LEG2/LEG3，基础165°；其余四腿160°。
      totalLean = constrain(totalLean, -10, 10);
      gait_tripod(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN,
                  FORWARD_PERIOD, totalLean);
      break;
    }

    case STATE_BACKWARD:
      // 直退：确认后方安全时使用
      gait_tripod(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 700);
      break;

    case STATE_TURN_LEFT:
      // turn() 参数是 ccw：1=逆时针/左转，0=顺时针/右转
      turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 750);
      break;

    case STATE_TURN_RIGHT:
      turn(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 750);
      break;

    case STATE_RETREAT_LEFT:
    case STATE_RETREAT_RIGHT:
      // 第一阶段无论最终向哪边转，都只做完整直退。
      gait_tripod(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 700, 0);
      break;

    case STATE_ESCAPE:
      // 逃脱模式：大幅转向找到出路
      if (retreatBias < 0) {
        turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 600);
      } else {
        turn(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 600);
      }
      break;

    case STATE_STOP:
      stand();  // UART 停车：原地站立
      break;

  }

  // RDK-X5 UART 持续轮询（D2 = HCRX, 115200 baud）
  // 10ms 窗口内不间断捕获所有字节 → 打印延迟到窗口关闭后 → 不漏字节
  {
    unsigned long pollEnd = micros() + 10000;  // 10ms 轮询窗口
    byte recvBuf[16];
    byte recvCount = 0;
    bool gotStopCmd = false;

    while (micros() < pollEnd) {
      if ((PIND & 0x04) == 0) {  // D2 LOW = 起始位
        byte b = 0;
        delayMicroseconds(4);
        for (int i = 0; i < 8; i++) {
          delayMicroseconds(9);
          if (PIND & 0x04) b |= (1 << i);
        }
        // 快速处理，不在此处打印
        if (b == 0x05) {
          gotStopCmd = true;
        }
        if (recvCount < 16) recvBuf[recvCount++] = b;
        // 不 break，继续读后续字节
      }
    }

    // 窗口关闭后统一处理
    if (gotStopCmd) {
      // 保存 UART 停车前的状态，用于后续自动恢复
      if (!uartStopActive) {
        preUartStopState = currentState;
      }
      uartStopActive = true;
      cameraStopLocked = true;  // CAMERA LOCK: permanent, highest priority
      currentState = STATE_STOP;
      stateEndTime = millis() + 999999;
      lastUartStopCmd = millis();
      stand();                  // 本轮动作已执行过，收到 RDK 指令后立即覆盖为站立停车
      beep(1200, 200);
    }
    // 打印所有收到的字节
    for (byte i = 0; i < recvCount; i++) {
      Serial.print(F("[RDK] 0x"));
      if (recvBuf[i] < 0x10) Serial.print('0');
      Serial.print(recvBuf[i], HEX);
      Serial.println(recvBuf[i] == 0x05 ? F(" ★ STOP") : F(" ?"));
    }
  }
  yield();
}
