////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Gait Control
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module implements various gaits for walking, including tripod, ripple,
// quadruped, belly crawl, and scamper modes.
////////////////////////////////////////////////////////////////////////////////

#ifndef GAIT_H
#define GAIT_H

#include "config.h"

// Gait Functions
void gait_tripod(int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod);
void gait_tripod(int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod, int leanangle);
void gait_tripod_scamper(int reverse, int turn);
void gait_ripple(int turn, int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod);
void gait_ripple(int turn, int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod, int leanangle);
void gait_quad(int turn, int reverse, int hipforward, int hipbackward,
               int kneeup, int kneedown, long timeperiod, int leanangle);
void gait_belly(int turn, int reverse, int hipforward, int hipbackward,
                int kneeup, int kneedown, long timeperiod, int leanangle);

// Gait Command Function (for Scratch/programmable control)
void gait_command(int gaittype, int reverse, int hipforward, int hipbackward,
                  int kneeup, int kneedown, int leanangle, int timeperiod);

// Demo Gait Functions
void random_gait(int timingfactor);

// Gait State Variables
extern int curGait;
extern int curReverse;
extern unsigned long nextGaitTime;
extern int ScamperPhase;
extern unsigned long NextScamperPhaseTime;
extern long ScamperTracker;

// Last Gait Command Parameters (for repeat)
extern unsigned int LastGgaittype;
extern unsigned int LastGreverse;
extern unsigned int LastGhipforward;
extern unsigned int LastGhipbackward;
extern unsigned int LastGkneeup;
extern unsigned int LastGkneedown;
extern unsigned int LastGtimeperiod;
extern int LastGleanangle;

#endif // GAIT_H
