////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Sensors
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module handles sensor reading, including ultrasonic distance sensor
// and other analog sensors.
////////////////////////////////////////////////////////////////////////////////

#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

// Ultrasonic Sensor Functions
void initSensors();          // Set pinModes once during setup (call before readDistance)
unsigned int readDistance(int trigPin, int echoPin);  // returns number of centimeters from ultrasonic rangefinder

// Helper Functions
int flash(unsigned long t);  // Flash LED on pin 13

#endif // SENSORS_H
