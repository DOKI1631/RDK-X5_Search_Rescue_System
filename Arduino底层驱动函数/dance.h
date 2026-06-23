////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Dance Control
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module implements various dance modes and movements.
////////////////////////////////////////////////////////////////////////////////

#ifndef DANCE_H
#define DANCE_H

#include "config.h"

// Dance Functions
void dance(int legs_up, int submode, int timingfactor);
void dance2(int cmd, int submode);
void dance_dab(int timingfactor);
void dance_ballet(int dpad);
void dance_hands(int dpad);
void dance_twitch(int cmd, int hipforward, int hipbackward, int kneeup, int kneedown, int twitchmode);
void dance_brownian(int cmd);
void dance_star(int cmd);
void boogie_woogie(int legs_flat, int submode, int timingfactor);
void flutter();
void wave(int dpad);
void wave_hello(int cmd);
void corgi(int begmode);

#endif // DANCE_H
