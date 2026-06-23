////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Leg Motion Implementation
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
////////////////////////////////////////////////////////////////////////////////

#include "leg_motion.h"
#include "servo_control.h"

void setHipRaw(int leg, int pos) {
  setServo(leg, pos);
}

// this version of setHip adjusts for left and right legs so
// that 0 degrees moves "forward" i.e. toward legs 5-0 which is
// nominally the front of the robot
void setHip(int leg, int pos) {
  // reverse the left side for consistent forward motion
  if (leg >= LEFT_START) {
    pos = 180 - pos;
  }
  setHipRaw(leg, pos);
}

// this version of setHip adjusts not only for left and right,
// but also shifts the front legs a little back and the back legs
// forward to make a better balance for certain gaits like tripod or quadruped
void setHip(int leg, int pos, int adj) {
  if (ISFRONTLEG(leg)) {
    pos -= adj;
  } else if (ISBACKLEG(leg)) {
    pos += adj;
  }
  // reverse the left side for consistent forward motion
  if (leg >= LEFT_START) {
    pos = 180 - pos;
  }

  setHipRaw(leg, pos);
}

// this version of setHip doesn't do mirror images like raw, but it
// does honor the adjust parameter to shift the front/back legs
void setHipRawAdj(int leg, int pos, int adj) {
  if (leg == 5 || leg == 2) {
    pos += adj;
  } else if (leg == 0 || leg == 3) {
    pos -= adj;
  }

  setHipRaw(leg, pos);
}

void setKnee(int leg, int pos) {
  // find the knee associated with leg if this is not already a knee
  if (leg < KNEE_OFFSET) {
    leg += KNEE_OFFSET;
  }
  setServo(leg, pos);
}

// This function sets the positions of both the knee and hip in
// a single command.  For hip, the left side is reversed so
// forward direction is consistent.

// This function takes a bitmask to specify legs to move, note that
// the basic setHip and setKnee functions take leg numbers, not masks

// if a position is -1 then that means don't change that item
void setLeg(int legmask, int hip_pos, int knee_pos, int adj) {
  setLeg(legmask, hip_pos, knee_pos, adj, 0, 0);  // use the non-raw version with leanangle=0
}

// version with leanangle = 0
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw) {
  setLeg(legmask, hip_pos, knee_pos, adj, raw, 0);
}

// This is the full version of setLeg with all the features
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw, int leanangle) {
  for (int i = 0; i < NUM_LEGS; i++) {
    if (legmask & 0b1) {  // if the lowest bit is ON then we are moving this leg
      if (hip_pos != NOMOVE) {
        if (!raw) {
          setHip(i, hip_pos, adj);
        } else {
          setHipRaw(i, hip_pos);
        }
      }
      if (knee_pos != NOMOVE) {
        int pos = knee_pos;

        // 俯仰补偿：前腿角度↓=身体抬升、后腿角度↑=身体下蹲 → 后仰防磕头
        // 注意：膝盖角度与身体高度成反比（角度大→腿折叠→身体低）
        if (PITCH_COMPENSATION != 0) {
          if (ISFRONTLEG(i)) {
            pos -= PITCH_COMPENSATION;  // 角度减小 → 腿更直 → 前身抬升
          } else if (ISBACKLEG(i)) {
            pos += PITCH_COMPENSATION;  // 角度增大 → 腿更蹲 → 后身降低
          }
        }

        // 左右重心偏移补偿（leanangle）
        if (leanangle != 0) {
          switch (i) {
            case 0: case 5:
              if (leanangle < 0) pos -= leanangle;
              break;
            case 1: case 4:
              pos += abs(leanangle/2);
              break;
            case 2: case 3:
              if (leanangle > 0) pos += leanangle;
              break;
          }
        }

        setKnee(i, pos);
      }
    }
    legmask = (legmask>>1);  // shift down one bit position to check the next legmask bit
  }
}

void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod) {
  turn(ccw, hipforward, hipbackward, kneeup, kneedown, timeperiod, 0);
}

void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle) {
  // use tripod groups to turn in place
  if (ccw) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_TURN_PHASES 6
#define FBSHIFT_TURN    40   // shift front legs back, back legs forward, this much

  long t = hexmillis()%timeperiod;
  long phase = (NUM_TURN_PHASES*t)/timeperiod;

  transactServos();  // 批量处理所有舵机指令，确保转弯平滑

  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move clockwise
      // at the hips, while the rest of the legs move CCW at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT_TURN, 1);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT_TURN, 1);
      break;

    case 2:
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0);
      break;

    case 4:
      // similar to phase 1, move raised legs CW and lowered legs CCW
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT_TURN, 1);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT_TURN, 1);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0);
      break;
  }

  commitServos();  // 一次性提交所有舵机位置，消除逐个动作的顿挫感

}

void stand() {
  transactServos();
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
  commitServos();
}

// Gradual stand-up: moves legs in 3 diagonal-pair groups with delays
// to avoid the massive current spike of all 12 servos starting at once.
// This prevents brown-out reset on battery-powered setups.
void standGradual() {
  // Group 1: Leg 0 (front-right) + Leg 3 (back-left)
  Serial.println(F("SG1"));
  Serial.flush();
  transactServos();
  setLeg(LEG0 | LEG3, HIP_NEUTRAL, KNEE_STAND, 0);
  commitServos();
  delay(300);

  // Group 2: Leg 1 (mid-right) + Leg 4 (mid-left)
  Serial.println(F("SG2"));
  Serial.flush();
  transactServos();
  setLeg(LEG1 | LEG4, HIP_NEUTRAL, KNEE_STAND, 0);
  commitServos();
  delay(300);

  // Group 3: Leg 2 (back-right) + Leg 5 (front-left)
  Serial.println(F("SG3"));
  Serial.flush();
  transactServos();
  setLeg(LEG2 | LEG5, HIP_NEUTRAL, KNEE_STAND, 0);
  commitServos();
  delay(300);
}

void stand_90_degrees() {  // used to install servos, sets all servos to 90 degrees
  transactServos();
  setLeg(ALL_LEGS, 90, 90, 0);
  commitServos();
}

void laydown() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_UP, 0);
}

void tiptoes() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
}

void foldup() {
  setLeg(ALL_LEGS, NOMOVE, KNEE_FOLD, 0);
  for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, HIP_FOLD);
}
