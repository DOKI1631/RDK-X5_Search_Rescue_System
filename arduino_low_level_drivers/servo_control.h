////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Servo Control
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module handles servo control, including PWM driver management,
// servo positioning, trimming, and collision detection
////////////////////////////////////////////////////////////////////////////////

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "config.h"
#include <Adafruit_PWMServoDriver.h>

// External Servo Driver Instance
extern Adafruit_PWMServoDriver servoDriver;

// Servo Control Functions
void resetServoDriver();                     // Initialize/reset the servo driver
void setServo(int servonum, unsigned int position);  // Set servo position with trim support
void transactServos();                       // Begin transaction (defer servo updates)
void commitServos();                         // Commit all pending servo updates
void checkForCrashingHips();                 // Check for potential servo collisions
void attach_all_servos();                    // Attach all servos (wake them up)
void detach_all_servos();                    // Detach all servos (put to sleep for power saving)
void checkForServoSleep();                   // Check if servo driver went to sleep and wake it up
void SmoothMove(int servo);                  // Smoothly move servo toward target position
void GeneralCheckSmoothMoves();              // Check and update all servos with smooth movement

// Trim Functions
void save_trims();                           // Save trim values to EEPROM
void erase_trims();                          // Erase all trim values

// Servo State Variables
extern byte deferServoSet;                   // Flag to defer servo updates
extern int servoOffset;                      // Offset to redefine front of robot
extern int ServosDetached;                   // Flag indicating servos are detached

#endif // SERVO_CONTROL_H
