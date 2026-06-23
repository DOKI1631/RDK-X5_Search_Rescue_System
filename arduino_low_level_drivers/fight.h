////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Fight Control
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module implements fight modes and combat movements.
////////////////////////////////////////////////////////////////////////////////

#ifndef FIGHT_H
#define FIGHT_H

#include "config.h"

// Fight Mode Functions
void fight_mode(char dpad, int mode, long timeperiod);
void fight2(int cmd, int submode);

// Walk Mode 2 Functions (alternative walking modes)
void walk2(int cmd, int submode);

// Front Reverse State (for walk2 mode 4)
extern byte FrontReverse;
extern long DebounceFrontReverse;

#endif // FIGHT_H
