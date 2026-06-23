////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Leg Motion
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
//
// This module handles individual leg motion control, including hip and knee
// movement, as well as basic poses like standing, lying down, etc.
////////////////////////////////////////////////////////////////////////////////

#ifndef LEG_MOTION_H
#define LEG_MOTION_H

#include "config.h"

// Basic Leg Control Functions
void setHipRaw(int leg, int pos);               // Set hip position without processing
void setHip(int leg, int pos);                  // Set hip position with left/right mirroring
void setHip(int leg, int pos, int adj);         // Set hip with front/back adjustment
void setHipRawAdj(int leg, int pos, int adj);   // Set hip with adjustment but no mirroring
void setKnee(int leg, int pos);                 // Set knee position for a leg

// Multiple Leg Control Functions
void setLeg(int legmask, int hip_pos, int knee_pos, int adj);
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw);
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw, int leanangle);

// Basic Pose Functions
void stand();                                   // Put robot in standing position
void standGradual();                            // Gradually stand up in groups to avoid current spike brown-out
void stand_90_degrees();                        // Set all servos to 90 degrees (for installation)
void laydown();                                 // Lay down on the ground
void tiptoes();                                 // Stand on tiptoes
void foldup();                                  // Fold legs up

// Turn Function
void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod);
void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle);

#endif // LEG_MOTION_H
