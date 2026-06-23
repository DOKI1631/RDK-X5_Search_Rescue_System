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

// Half-bit timing (μs). 5 μs → ~100 kHz max, safe margin for AMG8833
#define SWI2C_T  5

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

// Write a single byte to an AMG8833 register
static void amgWrite8(uint8_t reg, uint8_t value) {
  swi2c_start();
  swi2c_write((g_amg8833_addr << 1) | 0);  // 7-bit addr + WRITE bit
  swi2c_write(reg);
  swi2c_write(value);
  swi2c_stop();
}

// Read a 16-bit signed value from two consecutive registers (LSB first)
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

  Serial.println(F("THERMAL: AMG8833 initialized OK (software I2C)"));
  return true;
}

bool isThermalConnected() {
  return swi2c_probe(g_amg8833_addr);
}

float readThermalPixel(uint8_t index) {
  if (index >= THERMAL_PIXELS) return 0.0f;

  uint8_t reg = AMG8833_REG_PIXEL_BASE + (index * 2);
  int16_t raw = amgRead16(reg);
  return rawToCelsius(raw);
}

void readThermalPixels(float pixels[THERMAL_PIXELS]) {
  // Read all 64 pixels sequentially.
  // Software I2C has no 32-byte buffer limit, but reading in chunks
  // keeps each transaction < 40 ms so servo timing isn't disturbed.
  #define PIXELS_PER_CHUNK 16
  #define BYTES_PER_CHUNK  (PIXELS_PER_CHUNK * 2)

  for (int chunk = 0; chunk < THERMAL_PIXELS / PIXELS_PER_CHUNK; chunk++) {
    uint8_t startReg = AMG8833_REG_PIXEL_BASE + (chunk * BYTES_PER_CHUNK);

    // Send register address
    swi2c_start();
    swi2c_write((g_amg8833_addr << 1) | 0);
    swi2c_write(startReg);

    // Repeated START for read
    swi2c_start();
    swi2c_write((g_amg8833_addr << 1) | 1);

    // Read PIXELS_PER_CHUNK pixels (2 bytes each).
    // Send ACK after every byte except the very last one (hi of last pixel).
    for (int i = 0; i < PIXELS_PER_CHUNK; i++) {
      bool isLastPixel = (i == PIXELS_PER_CHUNK - 1);
      uint8_t lo = swi2c_read(false);           // ACK
      uint8_t hi = swi2c_read(isLastPixel);     // NACK only on last byte

      int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);
      if (raw & 0x0800) {
        raw |= 0xF000;
      }

      int pixelIdx = chunk * PIXELS_PER_CHUNK + i;
      pixels[pixelIdx] = rawToCelsius(raw);
    }

    swi2c_stop();
  }
}

HeatSourceResult detectHeatSource(float pixels[THERMAL_PIXELS]) {
  HeatSourceResult result;
  result.detected = false;
  result.maxTemp = -100.0f;
  result.avgTemp = 0.0f;
  result.centerX = 3;  // default center
  result.centerY = 3;
  result.hotPixelCount = 0;
  result.hotColumns = 0;
  result.directionX = 0;
  result.directionY = 0;

  float sumTemp = 0.0f;
  float weightedX = 0.0f;
  float weightedY = 0.0f;
  bool colHasHot[THERMAL_COLS] = {false};

  // Scan all 64 pixels for human-temperature heat signatures
  for (int row = 0; row < THERMAL_ROWS; row++) {
    for (int col = 0; col < THERMAL_COLS; col++) {
      int idx = row * THERMAL_COLS + col;
      float temp = pixels[idx];

      // Track global maximum temperature
      if (temp > result.maxTemp) {
        result.maxTemp = temp;
      }

      // Check if this pixel falls in human body temperature range
      if (temp >= HUMAN_TEMP_MIN && temp <= HUMAN_TEMP_MAX) {
        result.hotPixelCount++;
        sumTemp += temp;
        colHasHot[col] = true;

        // Weighted centroid: closer pixels (higher temp) have more weight
        float weight = temp - (HUMAN_TEMP_MIN - 2.0f);
        weightedX += (float)col * weight;
        weightedY += (float)row * weight;
      }
    }
  }

  // Count how many unique columns contain at least one hot pixel
  for (int c = 0; c < THERMAL_COLS; c++) {
    if (colHasHot[c]) result.hotColumns++;
  }

  // Determine if we have a valid heat source
  // Require at least 2 hot pixels to avoid noise
  if (result.hotPixelCount >= 2) {
    result.detected = true;

    // Calculate weighted center of hot region
    float totalWeight = sumTemp - (result.hotPixelCount * (HUMAN_TEMP_MIN - 2.0f));
    if (totalWeight > 0.0f) {
      result.centerX = (int)(weightedX / totalWeight + 0.5f);
      result.centerY = (int)(weightedY / totalWeight + 0.5f);
    } else {
      // Fallback: geometric center of hot pixels
      result.centerX = 3;
      result.centerY = 3;
    }

    result.centerX = constrain(result.centerX, 0, 7);
    result.centerY = constrain(result.centerY, 0, 7);
    result.avgTemp = sumTemp / (float)result.hotPixelCount;

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
