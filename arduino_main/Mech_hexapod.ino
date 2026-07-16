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
const char *Version = "#RV3r1a";

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
bool thermalMode = true;                // 热成像搜索模式开关（默认开启）
bool servosAvailable = false;           // PCA9685 在线标志
unsigned long thermalGestureStart = 0;  // 双手遮挡开始时间（用于手势检测）
bool thermalGestureArmed = false;       // 手势计时是否已开始
bool thermalStopLocked = false;         // ★ 热成像锁定：一旦找到目标，锁定舵机禁止任何运动，仅手势可解
bool cameraStopLocked = false;          // ★ 摄像头锁定：RDK-X5识别到人后永久停车，最高优先级
float thermalPixels[THERMAL_PIXELS];    // 64 像素温度网格
HeatSourceResult thermalResult;         // 热源检测结果
unsigned long lastThermalReadTime = 0;  // 上次读取热成像的时间
bool thermalSensorOK = false;           // 热成像传感器是否正常

// ==================== 左右超声波滑动平均滤波重构 ====================
#define DIST_WINDOW_SIZE 5
unsigned int distBufferL[DIST_WINDOW_SIZE] = {1000, 1000, 1000, 1000, 1000};
unsigned int distBufferR[DIST_WINDOW_SIZE] = {1000, 1000, 1000, 1000, 1000};
int distIndexL = 0;
int distIndexR = 0;

// 左侧超声波滤波刷新
unsigned int filterDistanceL(unsigned int raw) {
  // 如果读取到0，通常是超时或错误造成的跳变噪声，转换为远距离空旷值防误触
  if (raw == 0) raw = 1000; 
  distBufferL[distIndexL] = raw;
  distIndexL = (distIndexL + 1) % DIST_WINDOW_SIZE;
  unsigned long sum = 0;
  for (int i = 0; i < DIST_WINDOW_SIZE; i++) {
    sum += distBufferL[i];
  }
  return (unsigned int)(sum / DIST_WINDOW_SIZE);
}

// 右侧超声波滤波刷新
unsigned int filterDistanceR(unsigned int raw) {
  if (raw == 0) raw = 1000; 
  distBufferR[distIndexR] = raw;
  distIndexR = (distIndexR + 1) % DIST_WINDOW_SIZE;
  unsigned long sum = 0;
  for (int i = 0; i < DIST_WINDOW_SIZE; i++) {
    sum += distBufferR[i];
  }
  return (unsigned int)(sum / DIST_WINDOW_SIZE);
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
        loudAlert(2);  // 低电量扫频警报
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
  Serial.begin(9600);
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
  servosAvailable = true;  // PCA9685 初始化成功
  delay(250);

  // 初始化 AMG8833 热成像传感器（与舵机驱动共用 I2C 总线）
  thermalSensorOK = initThermal();
  if (!thermalSensorOK) {
    Serial.println(F("WARN: Thermal sensor not found. Thermal mode disabled."));
  }

  // 安全初始化：ServoPos 预填站立角度，防止任何函数 commitServos 时把未设置的舵机命令到 0°
  for (int i = 0; i < NUM_LEGS; i++) {
    ServoPos[i] = HIP_NEUTRAL;
    ServoPos[i + KNEE_OFFSET] = KNEE_STAND;
  }

  standGradual();  // 渐进站立，分 3 组避免 12 舵机同时启动的电流尖峰 → 欠压复位
  delay(300);
  beep(400);
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
  STATE_RETREAT_LEFT,   // 向左后方撤退（带方向感知）
  STATE_RETREAT_RIGHT,  // 向右后方撤退（带方向感知）
  STATE_ESCAPE,         // 卡死后大角度逃脱
  STATE_THERMAL_STOP,   // 热成像模式：已到达热源，停止
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

#define DIR_MEM_SIZE 16               // 方向记忆容量
#define SAFE_CORRIDOR_DIST 50         // 超过此距离视为"安全走廊"
#define OBSTACLE_STREAK_MAX 4         // 连续遇障 N 次触发逃脱模式
#define RETREAT_DURATION 800          // 撤退持续时间 (ms)
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
byte obstacleStreak = 0;                 // 连续遇障次数
int retreatBias = 0;                     // 当前撤退偏向 (-1=偏左, 0=直退, 1=偏右)
unsigned long streakResetTime = 0;       // 卡死计数重置时间

// ---- 自动漂移补偿 ----
int autoDriftCompensation = 0;           // 运行时自动漂移补偿 (-25..+25)
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
  unsigned int bestLeft = 0;
  unsigned int bestRight = 0;

  // 扫描最近 6 秒内的记忆，找传感器读数最大的时刻（最空旷）
  for (int i = 0; i < pathHistCount; i++) {
    if (now - pathHistory[i].timestamp > 6000) continue; // 超过 6 秒的记忆过期

    if (pathHistory[i].distL > bestLeft) {
      bestLeft = pathHistory[i].distL;
    }
    if (pathHistory[i].distR > bestRight) {
      bestRight = pathHistory[i].distR;
    }
  }

  // 比较两侧的历史最佳读数
  int diff = (int)bestLeft - (int)bestRight;

  if (diff > 30) {
    return -1; // 左边历史上更空旷 → 向左后方退
  } else if (diff < -30) {
    return 1;  // 右边历史上更空旷 → 向右后方退
  } else {
    return 0;  // 两边差不多 → 直退（原路返回）
  }
}

// 检查路径记忆中正后方是否安全（最近前进时读数是否空旷）
bool isRearPathSafe() {
  unsigned long now = millis();
  for (int i = 0; i < pathHistCount; i++) {
    // 找最近 4 秒内的前进记录
    if (now - pathHistory[i].timestamp > 4000) continue;
    if (pathHistory[i].moveType != 0) continue; // 只看前进记录

    // 前进时两侧都 > 50cm 才算后方安全
    if (pathHistory[i].distL > SAFE_CORRIDOR_DIST &&
        pathHistory[i].distR > SAFE_CORRIDOR_DIST) {
      return true;
    }
  }
  return false;
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
    // 热成像锁定 → 保持停止
    else if (thermalStopLocked) {
      uartStopActive = false;
      lastUartStopCmd = 0;
      currentState = STATE_THERMAL_STOP;
      stateEndTime = currentMillis + 999999;
      Serial.println(F("[RDK] Auto-recovery blocked - THERMAL LOCKED"));
    }
    // 恢复到 UART 停车前的状态
    else if (preUartStopState == STATE_THERMAL_STOP) {
      uartStopActive = false;
      lastUartStopCmd = 0;
      currentState = STATE_THERMAL_STOP;
      stateEndTime = currentMillis + 999999;
      Serial.println(F("[RDK] Auto-recovered -> THERMAL_STOP (target was acquired)"));
    } else {
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
  // 2. 定时感知层：每 100ms 读一次传感器，经滑动窗口滤波
  //    【关键优化】：
  //    - pulseIn 超时 8ms（约 1.36m 探测范围）
  //    - 传感器间 delay 3ms
  //    - 原始读数经 5 帧滑动平均滤波
  //    - 单次传感器迭代阻塞 ~19ms
  // ========================================================
  if (currentMillis - lastSensorTime >= 100) {
    lastSensorTime = currentMillis;

    unsigned int rawL = readDistance(TRIG_L, ECHO_L);
    delay(3);
    unsigned int rawR = readDistance(TRIG_R, ECHO_R);

    distL = filterDistanceL(rawL);
    distR = filterDistanceR(rawR);

    Serial.print("L:"); Serial.print(distL);
    Serial.print(" | R:"); Serial.println(distR);

    // ---- 热成像传感器读取（仅在热成像模式下） ----
    if (thermalMode && thermalSensorOK) {
      readThermalPixels(thermalPixels);
      thermalResult = detectHeatSource(thermalPixels);
      lastThermalReadTime = currentMillis;

      if (thermalResult.detected) {
        Serial.print(F("THERM: Tmax=")); Serial.print(thermalResult.maxTemp, 1);
        Serial.print(F("°C N=")); Serial.print(thermalResult.hotPixelCount);
        Serial.print(F(" X=")); Serial.print(thermalResult.centerX);
        Serial.print(F(" Dir=")); Serial.print(thermalResult.directionX);
        Serial.print(F(" Cols=")); Serial.println(thermalResult.hotColumns);
      } else {
        Serial.println(F("THERM: No heat source"));
      }
    }
  }

  // ========================================================
  // 2.5 热成像模式手势切换检测
  //     双手同时遮挡左右超声波 ≥2 秒 → 切换热成像模式
  // ========================================================
  {
    bool bothCovered = (distL < DANGER_DIST && distR < DANGER_DIST);

    if (bothCovered && thermalSensorOK) {
      if (!thermalGestureArmed) {
        thermalGestureStart = currentMillis;
        thermalGestureArmed = true;
      } else if (currentMillis - thermalGestureStart >= THERMAL_GESTURE_MS) {
        // 手势触发：切换热成像模式
        thermalGestureArmed = false; // 防止连续触发

        if (thermalMode) {
          // 当前 ON → 要关闭
          // ★ 安全锁：已找到热源目标时，禁止手势关闭
          if (currentState == STATE_THERMAL_STOP) {
            // UNLOCK: gesture overrides thermal stop lock
            thermalStopLocked = false;
            thermalMode = false;
            Serial.println(F(">> THERMAL: UNLOCKED - resuming navigation"));
            beep(1000, 150);
            delay(100);
            beep(800, 150);
            currentState = STATE_FORWARD;
            stateEndTime = currentMillis;
          }
          else {
            thermalMode = false;
            Serial.println(F(">> THERMAL MODE: OFF (Normal Navigation)"));
            beep(500, 200);
            delay(100);
            beep(400, 150);
            currentState = STATE_FORWARD;
            stateEndTime = currentMillis;
          }
        } else {
          // 当前 OFF → 要开启
          thermalMode = true;
          Serial.println(F(">> THERMAL MODE: ON (Search & Rescue)"));
          beep(1000, 200);
          delay(100);
          beep(1200, 150);
          // 进入热成像模式：保持当前运动状态，热成像引力层激活
        }
      }
    } else {
      // 任一侧超声波未被遮挡 → 重置手势计时
      thermalGestureArmed = false;
    }
  }

  // ========================================================
  // 3. 方向感知记录：每次传感器读取后写入方向记忆
  // ========================================================
  {
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
    recordToMemory(distL, distR, moveType);

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
  if (currentState == STATE_FORWARD) {
    if (distL < WARNING_DIST && distR >= WARNING_DIST) {
      driftLeftCount++;   // 左侧遇障、右侧通畅 → 机器人可能偏左
    } else if (distR < WARNING_DIST && distL >= WARNING_DIST) {
      driftRightCount++;  // 右侧遇障、左侧通畅 → 机器人可能偏右
    }
  }

  // 每 10 秒自动调整漂移补偿（从 30s 提速，更快响应偏航）
  if (currentMillis - lastDriftAdjustTime > 10000) {
    lastDriftAdjustTime = currentMillis;
    int diff = (int)driftLeftCount - (int)driftRightCount;
    if (diff > 2) {
      autoDriftCompensation = constrain(autoDriftCompensation + 2, -25, 25);
      Serial.print(F("DRIFT: Left bias, comp="));
      Serial.println(autoDriftCompensation);
    } else if (diff < -2) {
      autoDriftCompensation = constrain(autoDriftCompensation - 2, -25, 25);
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
  //    热成像模式作为"引力层"叠加，不替代避障
  // ========================================================
  // THERMAL_LOCKED: skip state machine entirely
  if (thermalStopLocked || cameraStopLocked) {
    // locked - do nothing
  }
  else if (currentMillis >= stateEndTime) {
    previousState = currentState;  // 记录切换前的状态

    // ---- 逃脱模式优先 ----
    if (obstacleStreak >= OBSTACLE_STREAK_MAX) {
      // 卡死了！大角度转向寻找新出路
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
    // ---- 紧急贴脸：两侧都 < 25cm 或任一侧 < 15cm → 必须后退 ----
    else if ((distL < WARNING_DIST && distR < WARNING_DIST) || distL < DANGER_DIST || distR < DANGER_DIST) {
      beep(800, 100);
      obstacleStreak++; // 累加卡死计数

      // ★ 方向感知：根据记忆和历史决定撤退方向
      retreatBias = computeRetreatBias();

      if (retreatBias < 0 && distR > distL) {
        // 左侧更空旷 → 向左后方撤退
        currentState = STATE_RETREAT_LEFT;
        Serial.println(" -> Retreat LEFT (left side historically safer)");
      } else if (retreatBias > 0 && distL > distR) {
        // 右侧更空旷 → 向右后方撤退
        currentState = STATE_RETREAT_RIGHT;
        Serial.println(" -> Retreat RIGHT (right side historically safer)");
      } else if (isRearPathSafe()) {
        // 后方路径安全 → 直退（原路返回最安全）
        currentState = STATE_BACKWARD;
        Serial.println(" -> Retreat STRAIGHT (rear path confirmed safe)");
      } else {
        // 不确定，综合判断：哪边当前更空旷就偏哪边
        if (distL > distR) {
          currentState = STATE_RETREAT_LEFT;
          Serial.println(" -> Retreat LEFT (left currently clearer)");
        } else {
          currentState = STATE_RETREAT_RIGHT;
          Serial.println(" -> Retreat RIGHT (right currently clearer)");
        }
      }
      stateEndTime = currentMillis + RETREAT_DURATION;
      streakResetTime = currentMillis;
    }
    // ---- 仅左侧有障碍 → 右转 ----
    else if (distL < WARNING_DIST) {
      obstacleStreak++;
      currentState = STATE_TURN_RIGHT;
      stateEndTime = currentMillis + TURN_DURATION;
      Serial.print(" -> Turn RIGHT (left obstacle at "); Serial.print(distL); Serial.println("cm)");
      streakResetTime = currentMillis;
    }
    // ---- 仅右侧有障碍 → 左转 ----
    else if (distR < WARNING_DIST) {
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
        distL > WARNING_DIST && distR > WARNING_DIST) {
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
    if ((currentState == STATE_TURN_RIGHT || currentState == STATE_TURN_LEFT ||
         currentState == STATE_RETREAT_LEFT || currentState == STATE_RETREAT_RIGHT)) {

      // 打断 1：两边都安全了 → 立刻恢复前进
      if (distL > 35 && distR > 35) {
        Serial.println(" -> [Smart Jump] Path cleared! Forward!");
        currentState = STATE_FORWARD;
        stateEndTime = currentMillis;
      }
      // 打断 2：突然出现近距离危险 → 紧急后退
      if (distL < DANGER_DIST || distR < DANGER_DIST) {
        Serial.println(" -> [Emergency] Danger close! Retreat!");
        retreatBias = (distL > distR) ? -1 : 1;
        currentState = (retreatBias < 0) ? STATE_RETREAT_LEFT : STATE_RETREAT_RIGHT;
        stateEndTime = currentMillis + RETREAT_DURATION;
      }
    }

    // 撤退过程中如果后方也检测到安全（前进时记忆的），可以缩短撤退时间
    if ((currentState == STATE_RETREAT_LEFT || currentState == STATE_RETREAT_RIGHT ||
         currentState == STATE_BACKWARD) &&
        distL > 40 && distR > 40) {
      // 撤退途中前方已经重新畅通，尽早停止撤退
      Serial.println(" -> [Early End] Retreat complete, path ahead clear");
      currentState = STATE_FORWARD;
      stateEndTime = currentMillis;
    }
  }

  // ========================================================
  // 6.5 热成像引力叠加层（仅在热成像模式 + 传感器正常时激活）
  //     核心原则：避障永远是最高优先级。
  //     只在安全状态下，用热源方向"牵引"运动方向。
  //     没有热源时，保留原有避障寻路行为，不会呆住。
  // ========================================================
  // THERMAL_LOCKED: skip all thermal guidance - servos frozen
  if (thermalStopLocked || cameraStopLocked) {
  }
  else if (thermalMode && thermalSensorOK) {

    // ---- 到达判定：多级阈值 + 超声波交叉验证 ----
    // AMG8833 仅 8×8=64 像素，20 像素阈值在物理上无法触发（人在 1m 处仅占 8-16 像素）
    // 修复：三级判断 + 超声波确认，确保在 1-2m 距离可靠停止
    bool hotVeryClose = (thermalResult.hotPixelCount >= 8);     // Tier 1: 热源像素多 → ~1m
    bool hotModClose  = (thermalResult.hotPixelCount >= 4       // Tier 2: 中等热源 + 多列
                         && thermalResult.hotColumns >= 3);
    bool hotBottom    = (thermalResult.centerY >= 5             // Tier 3: 热源在画面底部
                         && thermalResult.hotPixelCount >= 3);   //         (底部=近处)
    bool ultraConfirm = (thermalResult.detected                  // 超声波交叉验证
                         && (distL < 50 || distR < 50));         // 热源 + 近距 = 确认有人

    if (thermalResult.detected && (hotVeryClose || hotModClose || hotBottom || ultraConfirm)) {
      if (currentState != STATE_THERMAL_STOP) {
        Serial.println(F("THERM: *** TARGET REACHED! Stopping. ***"));
        loudAlert(5);
        currentState = STATE_THERMAL_STOP;
        stateEndTime = currentMillis + 999999;
        thermalStopLocked = true;   // HARD LOCK: no servo movement allowed
      }
    }
    // ---- 热源接近但还没到：减速 + 温和微调转向 ----
    else if (thermalResult.detected) {
      // 紧急障碍物时避障优先，不覆盖
      bool immediateDanger = (distL < DANGER_DIST || distR < DANGER_DIST);
      bool isSafeToSteer = (currentState == STATE_FORWARD) && !immediateDanger;

      if (isSafeToSteer) {
        // 微转向：150ms（原 250ms 幅度过大），接近时更温和
        const int microTurnMs = 150;
        if (thermalResult.directionX == 0) {
          currentState = STATE_FORWARD;
          stateEndTime = currentMillis + 100;
          Serial.println(F("THERM: HEAT CENTER, approach slow"));
        } else if (thermalResult.directionX < 0) {
          if (currentState != STATE_TURN_LEFT) {
            currentState = STATE_TURN_LEFT;
            stateEndTime = currentMillis + microTurnMs;
            Serial.println(F("THERM: HEAT LEFT, micro-turn"));
          }
        } else {
          if (currentState != STATE_TURN_RIGHT) {
            currentState = STATE_TURN_RIGHT;
            stateEndTime = currentMillis + microTurnMs;
            Serial.println(F("THERM: HEAT RIGHT, micro-turn"));
          }
        }
      }
    }
    // ---- 无热源：不干预，由周期性探头扫描(PERIODIC_SCAN)覆盖盲区 ----
    //      热成像传感器有 60° 广角视野，前进时已能覆盖前方大部分区域
    else {
      // 自动扫描偏转已禁用，保持直行
    }
  }

  // ========================================================
  // 6.6 停止状态最终保护门（THERMAL_STOP 和 UART STOP 均不可被其他状态覆盖）
  // ========================================================
  if (currentState == STATE_THERMAL_STOP || currentState == STATE_STOP) {
    stateEndTime = currentMillis + 999999;
  }

  // ========================================================
  // 6.7 RDK-X5 UART 停车最终强制门（独立于状态机，任何代码路径均不可绕过）
  // ========================================================
  // uartStopActive 是独立布尔标志，仅在收到 0x05 时置位，仅在 5s 无 0x05 时清除
  // 此处是动作执行前的最后一道检查——即使前面任何代码修改了 currentState
  // （状态机/热成像引导/PERIODIC_SCAN/Smart Interrupt），都会被强制覆盖
  if (uartStopActive) {
    currentState = STATE_STOP;
    stateEndTime = currentMillis + 999999;
  }

  // ========================================================
  // 7. 动作执行层
  // ========================================================
  switch (currentState) {
    case STATE_FORWARD:
    {
      int totalLean = DRIFT_COMPENSATION + autoDriftCompensation;
      totalLean = constrain(totalLean, -40, 40);
      // 热成像检测到热源 → 减速接近
      if (thermalMode && thermalSensorOK && thermalResult.detected) {
        gait_tripod(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN,
                    THERMAL_APPROACH_SPEED, totalLean);
      } else {
        gait_tripod(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN,
                    FORWARD_PERIOD, totalLean);
      }
      break;
    }

    case STATE_BACKWARD:
      // 直退：确认后方安全时使用
      gait_tripod(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 700);
      break;

    case STATE_TURN_LEFT:
      turn(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 750);
      break;

    case STATE_TURN_RIGHT:
      turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 750);
      break;

    case STATE_RETREAT_LEFT:
      // 向左后方撤退：交替后退步态 + 左转（每 450ms 切换）
      // 形成"后退曲线撤离"效果，比纯原地旋转更有效地远离障碍物
      if ((currentMillis / 450) % 2 == 0) {
        gait_tripod(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 450, 0);
      } else {
        turn(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 450);
      }
      break;

    case STATE_RETREAT_RIGHT:
      // 向右后方撤退：交替后退步态 + 右转
      if ((currentMillis / 450) % 2 == 0) {
        gait_tripod(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 450, 0);
      } else {
        turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 450);
      }
      break;

    case STATE_ESCAPE:
      // 逃脱模式：大幅转向找到出路
      if (retreatBias < 0) {
        turn(0, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 600);
      } else {
        turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, 600);
      }
      break;

    // ---- 热成像模式：到达停止 ----
    case STATE_THERMAL_STOP:
      // 到达伤员位置，保持站立并周期性蜂鸣提示
      stand();
      {
        unsigned long stopPhase = currentMillis % 3000;
        if (stopPhase < 200) {
          beep(3000, 200);  // 谐振点附近，更响亮
        }
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
        // 快速处理，不在此处打印（Serial 阻塞 ~1ms/byte 会丢后续数据）
        if (b == 0x05) {
          gotStopCmd = true;
        }
        if (recvCount < 16) recvBuf[recvCount++] = b;
        // 不 break，继续读后续字节
      }
    }

    // 窗口关闭后统一处理
    if (gotStopCmd) {
      // ★ 保存 UART 停车前的状态，用于后续自动恢复
      if (!uartStopActive) {
        preUartStopState = currentState;
      }
      uartStopActive = true;
      cameraStopLocked = true;  // CAMERA LOCK: permanent, highest priority
      currentState = STATE_STOP;
      stateEndTime = millis() + 999999;
      lastUartStopCmd = millis();
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
