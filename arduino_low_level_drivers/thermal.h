////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - AMG8833 Thermal Sensor
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module provides a lightweight driver for the AMG8833 8x8 infrared
// thermal sensor array over I2C. It reads all 64 pixels and detects human
// body-temperature heat sources for autonomous search-and-rescue navigation.
////////////////////////////////////////////////////////////////////////////////

#ifndef THERMAL_H
#define THERMAL_H

#include "config.h"
#include <Arduino.h>

// ==========================================
// AMG8833 I2C Address & Registers
// ==========================================
#define AMG8833_I2C_ADDR      0x69    // I2C address (AD0 = HIGH on this module)
#define AMG8833_ALT_I2C_ADDR  0x68    // Alternate address (AD0 = LOW)

// AMG8833 Internal Registers
#define AMG8833_REG_POWER     0x00    // Power control (0x00 = normal)
#define AMG8833_REG_RESET     0x01    // Reset (write 0x3F to soft-reset)
#define AMG8833_REG_FPS       0x02    // Frame rate (0x00 = 10 FPS)
#define AMG8833_REG_TTHL      0x0E    // Thermistor (device temp), 2 bytes
#define AMG8833_REG_PIXEL_BASE 0x80   // Start of 64-pixel array (128 bytes)

// ==========================================
// Thermal Grid Constants
// ==========================================
#define THERMAL_COLS          8       // 8 columns (X axis)
#define THERMAL_ROWS          8       // 8 rows (Y axis)
#define THERMAL_PIXELS        64      // Total thermal pixels (8 x 8)

// ==========================================
// Heat Source Detection Result
// ==========================================
struct HeatSourceResult {
  bool detected;            // True if a valid heat source is found
  float maxTemp;            // Highest temperature in the grid (°C)
  float avgTemp;            // Average temperature of the hot region (°C)
  int centerX;              // Weighted center X position of hot pixels (0~7)
  int centerY;              // Weighted center Y position of hot pixels (0~7)
  int hotPixelCount;        // Number of pixels above human temp threshold
  int hotColumns;           // How many columns contain at least one hot pixel
  int directionX;           // -1=left, 0=center, 1=right
  int directionY;           // -1=top(far), 0=middle, 1=bottom(near)
};

// ==========================================
// Function Declarations
// ==========================================

// Initialize the AMG8833 sensor (call once in setup())
// Returns true if sensor is detected on the I2C bus
bool initThermal();

// Read the full 8x8 pixel temperature array from the sensor
// Pixels are stored in row-major order: pixel[row*8 + col]
// Temperatures are in degrees Celsius with 0.25°C resolution
void readThermalPixels(float pixels[THERMAL_PIXELS]);

// Read a single pixel's temperature by index (0~63)
float readThermalPixel(uint8_t index);

// Detect human body-temperature heat sources in the pixel grid
// Filters for temperatures between HUMAN_TEMP_MIN and HUMAN_TEMP_MAX
// Returns a HeatSourceResult with direction and distance estimation
HeatSourceResult detectHeatSource(float pixels[THERMAL_PIXELS]);

// Map the heat source center X coordinate to a direction hint
// Returns: -1 (left), 0 (center), +1 (right)
int getHeatDirectionX(int centerX);

// Map the heat source spread to a distance estimation
// Returns: 0=near(stop), 1=medium(approach), 2=far(search)
int getHeatDistance(int hotColumns, int hotPixelCount);

// Check if the sensor is responding on the I2C bus
bool isThermalConnected();

#endif // THERMAL_H
