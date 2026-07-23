////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - AMG8833 Thermal Sensor
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// Lightweight driver for the AMG8833 8x8 IR thermal sensor array.
// Uses software I2C (bit-banging) on THERMAL_SDA_PIN / THERMAL_SCL_PIN
// so the sensor can share formerly-unused ultrasonic header pins instead
// of requiring the hardware I2C bus (A4/A5) which is occupied on this PCB.
////////////////////////////////////////////////////////////////////////////////

#include "thermal.h"

// ==========================================
// Software I2C Implementation (Direct Port I/O)
// ==========================================
// Uses fast DDRD / PORTD / PIND register manipulation instead of
// slow pinMode() / digitalWrite().  AMG8833 is 3.3 V — pins are set
// to INPUT (not INPUT_PULLUP) so the module's own pull-up resistors
// to 3V3 determine the logic-high level.
//
// IMPORTANT: The AMG8833 breakout MUST have on-board pull-up resistors
// (most do: Adafruit / GY-AMG8833 have 10k to 3V3).  If your module
// lacks them, solder 4.7k from SDA→3.3V and SCL→3.3V.

#ifndef THERMAL_SDA_PIN
#define THERMAL_SDA_PIN  5   // PD5 (Arduino D5)
#endif
#ifndef THERMAL_SCL_PIN
#define THERMAL_SCL_PIN  6   // PD6 (Arduino D6)
#endif

// Compute bit-masks from pin numbers (only D0-D7 / PORTD supported)
#define SWI2C_SDA_MSK  (1 << (THERMAL_SDA_PIN))   // PD5 → bit 5
#define SWI2C_SCL_MSK  (1 << (THERMAL_SCL_PIN))   // PD6 → bit 6

// Half-bit timing (μs). 10 μs → ~50 kHz for reliable software I2C with AMG8833
#define SWI2C_T  10

static inline void swi2c_dly() { delayMicroseconds(SWI2C_T); }

// --- SDA control ---
static inline void swi2c_sda_lo() {
  PORTD &= ~SWI2C_SDA_MSK;   // drive LOW first
  DDRD  |=  SWI2C_SDA_MSK;   // then switch to OUTPUT
}

static inline void swi2c_sda_hi() {
  DDRD  &= ~SWI2C_SDA_MSK;   // INPUT (hi-Z) — external pull-up to 3V3
  PORTD &= ~SWI2C_SDA_MSK;   // disable internal pull-up (safe for 3V3)
}

static inline uint8_t swi2c_sda_rd() {
  return (PIND & SWI2C_SDA_MSK) ? 1 : 0;
}

// --- SCL control ---
static inline void swi2c_scl_lo() {
  PORTD &= ~SWI2C_SCL_MSK;
  DDRD  |=  SWI2C_SCL_MSK;
}

static inline void swi2c_scl_hi() {
  DDRD  &= ~SWI2C_SCL_MSK;
  PORTD &= ~SWI2C_SCL_MSK;
}

static inline uint8_t swi2c_scl_rd() {
  return (PIND & SWI2C_SCL_MSK) ? 1 : 0;
}

// --- Init ---
static void swi2c_init() {
  swi2c_sda_hi();
  swi2c_scl_hi();
  delayMicroseconds(100);
}

// --- I2C START ---
static void swi2c_start() {
  swi2c_sda_hi();
  swi2c_scl_hi();
  swi2c_dly();
  swi2c_sda_lo();
  swi2c_dly();
  swi2c_scl_lo();
}

// --- I2C STOP ---
static void swi2c_stop() {
  swi2c_sda_lo();
  swi2c_scl_hi();
  swi2c_dly();
  swi2c_sda_hi();
  swi2c_dly();
}

// --- Write byte, return true if ACK received ---
static bool swi2c_write(uint8_t data) {
  for (uint8_t m = 0x80; m; m >>= 1) {
    if (data & m) swi2c_sda_hi(); else swi2c_sda_lo();
    swi2c_dly();
    swi2c_scl_hi();
    swi2c_dly();
    swi2c_scl_lo();
  }
  // ACK bit
  swi2c_sda_hi();
  swi2c_dly();
  swi2c_scl_hi();
  swi2c_dly();
  bool ack = (swi2c_sda_rd() == 0);
  swi2c_scl_lo();
  swi2c_dly();
  return ack;
}

// --- Read byte, send ACK (nack=false) or NACK (nack=true) ---
static uint8_t swi2c_read(bool nack) {
  uint8_t data = 0;
  swi2c_sda_hi();   // release — slave drives SDA

  for (uint8_t i = 0; i < 8; i++) {
    swi2c_dly();
    swi2c_scl_hi();
    swi2c_dly();
    data = (data << 1) | swi2c_sda_rd();
    swi2c_scl_lo();
  }

  // ACK / NACK
  if (nack) swi2c_sda_hi(); else swi2c_sda_lo();
  swi2c_dly();
  swi2c_scl_hi();
  swi2c_dly();
  swi2c_scl_lo();
  swi2c_dly();
  swi2c_sda_hi();

  return data;
}

// --- Probe address (returns true if ACK) ---
static bool swi2c_probe(uint8_t addr7) {
  swi2c_start();
  bool ack = swi2c_write((addr7 << 1) | 0);
  swi2c_stop();
  return ack;
}

// ==========================================
// Internal Sensor Communication Helpers
// ==========================================

// Active I2C address — set by initThermal() after probing
static uint8_t g_amg8833_addr = 0x69;  // default primary, may be overridden

// Warm-up tracking: sensor needs time to stabilize after power-up
static unsigned long g_thermalInitTime = 0;
static bool g_thermalInitDone = false;

// Write a single byte to an AMG8833 register
// Returns true if all three bytes (addr, reg, data) were ACK'd
static bool amgWrite8(uint8_t reg, uint8_t value) {
  swi2c_start();
  if (!swi2c_write((g_amg8833_addr << 1) | 0)) {  // 7-bit addr + WRITE bit
    swi2c_stop();
    return false;
  }
  if (!swi2c_write(reg)) {
    swi2c_stop();
    return false;
  }
  if (!swi2c_write(value)) {
    swi2c_stop();
    return false;
  }
  swi2c_stop();
  return true;
}

// Read a 16-bit signed value from two consecutive registers (LSB first)
// NOTE: does NOT check ACK to avoid false failures from software I2C timing jitter
static int16_t amgRead16(uint8_t reg) {
  swi2c_start();
  swi2c_write((g_amg8833_addr << 1) | 0);  // 7-bit addr + WRITE
  swi2c_write(reg);
  // Repeated START
  swi2c_start();
  swi2c_write((g_amg8833_addr << 1) | 1);  // 7-bit addr + READ

  uint8_t lo = swi2c_read(false);  // ACK  — more bytes follow
  uint8_t hi = swi2c_read(true);   // NACK — last byte
  swi2c_stop();

  int16_t val = (int16_t)(((uint16_t)hi << 8) | lo);

  // AMG8833 stores 12-bit two's complement in a 16-bit field
  // If the sign bit (bit 11) is set, extend the sign to bits 12-15
  if (val & 0x0800) {
    val |= 0xF000;
  }

  return val;
}

// Convert raw 12-bit signed value to temperature in degrees Celsius
// Resolution: 0.25°C per LSB
static float rawToCelsius(int16_t raw) {
  return (float)raw * 0.25f;
}

// ==========================================
// Public Functions
// ==========================================

bool initThermal() {
  // Initialize software I2C pins
  swi2c_init();

  // ---- Diagnostic: full software-I2C bus scan (debug only) ----
  // In normal operation, skip the 126-address scan to reduce startup time.
  // Only probe the two known AMG8833 addresses (0x68 / 0x69).
#ifdef DEBUG_OUTPUT
  Serial.println(F("THERMAL: Scanning software I2C bus (PD5=SDA, PD6=SCL)..."));
  {
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      if (swi2c_probe(addr)) {
        Serial.print(F("  Found device at 0x"));
        Serial.println(addr, HEX);
        found++;
      }
    }
    if (found == 0) {
      Serial.println(F("  *** NO devices found! ***"));
      Serial.println(F("  Possible causes:"));
      Serial.println(F("  1. Missing pull-up resistors (4.7k SDA→3V3, SCL→3V3)"));
      Serial.println(F("  2. AMG8833 not powered (check VIN=3.3V, GND)"));
      Serial.println(F("  3. SDA/SCL wires swapped or loose"));
    }
  }
#endif

  // Check if AMG8833 is present at primary address (0x69)
  bool foundAtPrimary = swi2c_probe(AMG8833_I2C_ADDR);
  bool foundAtAlt     = swi2c_probe(AMG8833_ALT_I2C_ADDR);

  uint8_t activeAddr;
  if (foundAtPrimary) {
    activeAddr = AMG8833_I2C_ADDR;
  } else if (foundAtAlt) {
    activeAddr = AMG8833_ALT_I2C_ADDR;
    Serial.print(F("THERMAL: Found at alternate addr 0x"));
    Serial.println(activeAddr, HEX);
  } else {
    Serial.println(F("THERMAL: Sensor not found. Thermal mode disabled."));
    return false;
  }

  // Store the detected address for all subsequent I2C communication
  g_amg8833_addr = activeAddr;

  // Software reset the sensor
  amgWrite8(AMG8833_REG_RESET, 0x3F);
  delay(100);

  // Set normal operating mode
  amgWrite8(AMG8833_REG_POWER, 0x00);
  delay(50);

  // Set frame rate to 10 FPS
  amgWrite8(AMG8833_REG_FPS, 0x00);
  delay(50);

  // ★ 启动诊断：读多个像素验证传感器是否返回不同的值
  {
    Serial.println(F("THERMAL: Init check - reading pixels 0,1,32,63:"));
    for (int pi = 0; pi < THERMAL_PIXELS; pi++) {
      if (pi == 0 || pi == 1 || pi == 32 || pi == 63) {
        int16_t raw = amgRead16(AMG8833_REG_PIXEL_BASE + (pi * 2));
        Serial.print(F("  pix["));
        Serial.print(pi);
        Serial.print(F("]="));
        Serial.print(rawToCelsius(raw), 1);
        Serial.print(F("°C (0x"));
        Serial.print(raw, HEX);
        Serial.println(F(")"));
        delay(5);  // 像素间短暂延时
      }
    }
  }

  Serial.println(F("THERMAL: AMG8833 initialized OK (software I2C)"));
  g_thermalInitTime = millis();
  g_thermalInitDone = true;
  return true;
}

bool isThermalConnected() {
  return swi2c_probe(g_amg8833_addr);
}

bool isThermalWarmedUp() {
  if (!g_thermalInitDone) return false;
  return (millis() - g_thermalInitTime) >= THERMAL_WARMUP_MS;
}

float readThermalPixel(uint8_t index) {
  if (index >= THERMAL_PIXELS) return 0.0f;

  uint8_t reg = AMG8833_REG_PIXEL_BASE + (index * 2);
  int16_t raw = amgRead16(reg);
  return rawToCelsius(raw);
}

void readThermalPixels(float pixels[THERMAL_PIXELS]) {
  // ★ 逐像素单独读取（不使用连续/批量 auto-increment 模式）
  //   每个像素独立发起 I2C 事务：START→写寄存器地址→RESTART→读2字节→STOP
  for (int i = 0; i < THERMAL_PIXELS; i++) {
    uint8_t reg = AMG8833_REG_PIXEL_BASE + (i * 2);
    int16_t raw = amgRead16(reg);
    pixels[i] = rawToCelsius(raw);
    delayMicroseconds(200);  // 像素间短暂延时，让传感器内部 ADC 稳定
  }
}

HeatSourceResult detectHeatSource(float pixels[THERMAL_PIXELS]) {
  HeatSourceResult result;
  result.detected = false;
  result.maxTemp = -100.0f;
  result.avgTemp = 0.0f;
  result.backgroundTemp = 0.0f;
  result.peakDelta = 0.0f;
  result.centerX = 3;  // default center
  result.centerY = 3;
  result.hotPixelCount = 0;
  result.hotColumns = 0;
  result.directionX = 0;
  result.directionY = 0;

  // 第一遍求全帧均值，同时拒绝断线/总线错误造成的不合理温度。
  float sumAll = 0.0f;
  int validCount = 0;
  for (int i = 0; i < THERMAL_PIXELS; i++) {
    if (pixels[i] > -20.0f && pixels[i] < 100.0f) {
      sumAll += pixels[i];
      validCount++;
    }
  }
  if (validCount < 60) return result;

  float meanTemp = sumAll / (float)validCount;

  // 用“不高于均值”的像素再次估计背景。相比全帧均值，它不会被画面中的人体
  // 大幅抬高；相比最低温度，它又不容易受单个冷坏点影响。
  float backgroundSum = 0.0f;
  int backgroundCount = 0;
  for (int i = 0; i < THERMAL_PIXELS; i++) {
    if (pixels[i] > -20.0f && pixels[i] <= meanTemp) {
      backgroundSum += pixels[i];
      backgroundCount++;
    }
  }
  if (backgroundCount == 0) return result;
  result.backgroundTemp = backgroundSum / (float)backgroundCount;

  float effectiveMin = HUMAN_TEMP_MIN;
  float adaptiveMin = result.backgroundTemp + THERMAL_BACKGROUND_DELTA;
  if (adaptiveMin > effectiveMin) {
    effectiveMin = adaptiveMin;
  }

  // 只保留最大的四连通热斑。旧算法把互不相邻的热噪点全部相加，两个孤立点
  // 就会被当成人体，而且散落在多列的噪点会被误判为“已经很近”。
  // 8x8掩码压成两个64位位图。旧版两个bool[64]占128字节栈，
  // 在ATmega328P的2KB SRAM上与队列、浮点运算叠加后可能破坏返回地址。
  uint64_t hotMask = 0;
  uint64_t visited = 0;
  for (int i = 0; i < THERMAL_PIXELS; i++) {
    float temp = pixels[i];
    if (temp > result.maxTemp) result.maxTemp = temp;
    if (temp >= effectiveMin && temp <= HUMAN_TEMP_MAX) {
      hotMask |= (UINT64_C(1) << i);
    }
  }

  int bestCount = 0;
  float bestSumTemp = 0.0f;
  float bestWeightedX = 0.0f;
  float bestWeightedY = 0.0f;
  float bestTotalWeight = 0.0f;
  float bestPeak = -100.0f;
  byte bestColMask = 0;

  for (int start = 0; start < THERMAL_PIXELS; start++) {
    uint64_t startBit = (UINT64_C(1) << start);
    if (!(hotMask & startBit) || (visited & startBit)) continue;

    uint8_t queue[THERMAL_PIXELS];
    int head = 0;
    int tail = 0;
    queue[tail++] = (uint8_t)start;
    visited |= startBit;

    int count = 0;
    float sumTemp = 0.0f;
    float weightedX = 0.0f;
    float weightedY = 0.0f;
    float totalWeight = 0.0f;
    float peak = -100.0f;
    byte colMask = 0;

    while (head < tail) {
      int idx = queue[head++];
      int row = idx / THERMAL_COLS;
      int col = idx % THERMAL_COLS;
      float temp = pixels[idx];
      float weight = temp - result.backgroundTemp;
      if (weight < 0.25f) weight = 0.25f;

      count++;
      sumTemp += temp;
      weightedX += (float)col * weight;
      weightedY += (float)row * weight;
      totalWeight += weight;
      if (temp > peak) peak = temp;
      colMask |= (byte)(1 << col);

      int neighbors[4] = {idx - THERMAL_COLS, idx + THERMAL_COLS, idx - 1, idx + 1};
      for (int n = 0; n < 4; n++) {
        int next = neighbors[n];
        if (next < 0 || next >= THERMAL_PIXELS) continue;
        if ((n == 2 || n == 3) && (next / THERMAL_COLS != row)) continue;
        uint64_t nextBit = (UINT64_C(1) << next);
        if ((hotMask & nextBit) && !(visited & nextBit)) {
          visited |= nextBit;
          queue[tail++] = (uint8_t)next;
        }
      }
    }

    if (count > bestCount || (count == bestCount && peak > bestPeak)) {
      bestCount = count;
      bestSumTemp = sumTemp;
      bestWeightedX = weightedX;
      bestWeightedY = weightedY;
      bestTotalWeight = totalWeight;
      bestPeak = peak;
      bestColMask = colMask;
    }
  }

  result.peakDelta = bestPeak - result.backgroundTemp;
  if (bestCount >= THERMAL_MIN_BLOB_PIXELS &&
      result.peakDelta >= THERMAL_MIN_PEAK_DELTA) {
    result.detected = true;
    result.hotPixelCount = bestCount;
    result.avgTemp = bestSumTemp / (float)bestCount;
    result.maxTemp = bestPeak;

    for (int c = 0; c < THERMAL_COLS; c++) {
      if (bestColMask & (1 << c)) result.hotColumns++;
    }

    if (bestTotalWeight > 0.0f) {
      result.centerX = (int)(bestWeightedX / bestTotalWeight + 0.5f);
      result.centerY = (int)(bestWeightedY / bestTotalWeight + 0.5f);
    }

    result.centerX = constrain(result.centerX, 0, 7);
    result.centerY = constrain(result.centerY, 0, 7);
    // Map center to direction hints
    result.directionX = getHeatDirectionX(result.centerX);

    // Y direction: rows 0-2 are top (typically farther away)
    // rows 5-7 are bottom (typically closer)
    if (result.centerY <= 2) {
      result.directionY = -1; // top of sensor view → farther away
    } else if (result.centerY >= 5) {
      result.directionY = 1;  // bottom of sensor view → closer
    } else {
      result.directionY = 0;  // middle
    }
  }

  return result;
}

int getHeatDirectionX(int centerX) {
  // NOTE: sensor X axis is flipped relative to robot body.
  // centerX 0→7 maps sensor left→right, which is robot right→left.
  if (centerX <= 2) {
    return 1;   // Sensor-left = robot-RIGHT → turn right
  } else if (centerX >= 5) {
    return -1;  // Sensor-right = robot-LEFT → turn left
  } else {
    return 0;   // Heat source is CENTERED
  }
}

int getHeatDistance(int hotColumns, int hotPixelCount) {
  // Heuristic distance estimation based on thermal footprint size
  if (hotColumns >= THERMAL_STOP_COLS) {
    return 0;  // NEAR — stop approaching
  } else if (hotColumns >= THERMAL_CLOSE_COLS || hotPixelCount >= THERMAL_CLOSE_PIXELS) {
    return 1;  // MEDIUM — slow approach
  } else {
    return 2;  // FAR — keep searching/approaching
  }
}
