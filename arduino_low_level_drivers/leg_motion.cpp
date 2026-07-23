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

        // 俯仰补偿：本项目定义 0°=折叠、180°=伸直。
        // 正补偿应让前腿更直、后腿略收，形成后仰并降低避障时磕头概率。
        if (PITCH_COMPENSATION != 0) {
          if (ISFRONTLEG(i)) {
            pos += PITCH_COMPENSATION;
          } else if (ISBACKLEG(i)) {
            pos -= PITCH_COMPENSATION;
          }
        }

        // 使用实机腿部标注确认的前腿映射：LEG2/LEG3（膝舵机通道 8/9）。
        if (ISFRONTKNEESERVO(i)) {
          pos += FRONT_KNEE_COMPENSATION;
        }

        // 左右重心偏移补偿（leanangle）。正值让左腿伸长、右腿收短，
        // 身体向右倾；负值反之。旧实现误按前/中/后腿分组，会造成俯仰畸变。
        if (leanangle != 0) {
          if (ISLEFTLEG(i)) pos += leanangle;
          else if (ISRIGHTLEG(i)) pos -= leanangle;
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

  // 与直行三脚架相同：阻塞后只推进一相，避免漏掉落腿相位。
  static int phase = 0;
  static unsigned long nextPhaseTime = 0;
  static unsigned long lastCallTime = 0;
  static long scheduledPeriod = 0;
  static int scheduledCcw = -1;
  unsigned long now = hexmillis();
  unsigned long phaseDuration = (unsigned long)(timeperiod / NUM_TURN_PHASES);
  if (phaseDuration < 1) phaseDuration = 1;

  bool restartSequence = (nextPhaseTime == 0 || scheduledPeriod != timeperiod ||
                          scheduledCcw != ccw ||
                          now - lastCallTime > (unsigned long)timeperiod);
  if (restartSequence) {
    phase = 0;
    nextPhaseTime = now + phaseDuration;
    scheduledPeriod = timeperiod;
    scheduledCcw = ccw;
  } else if ((long)(now - nextPhaseTime) >= 0) {
    phase = (phase + 1) % NUM_TURN_PHASES;
    nextPhaseTime = now + phaseDuration;
  }
  lastCallTime = now;

  transactServos();  // 批量处理所有舵机指令，确保转弯平滑

  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0);
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
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0);
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
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0, DRIFT_COMPENSATION);
  commitServos();
}

// Gradual stand-up: front-heavy chassis must establish both front supports first.
// Each pair moves through half-crouch before full stand to reduce startup torque.
void standGradual() {
  // Front pair first: lifting it last makes the loaded nose hard to raise.
  Serial.println(F("STAND FRONT 1/2"));
  Serial.flush();
  setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_HALF_CROUCH, 0, 0, DRIFT_COMPENSATION);
  delay(250);
  setLeg(FRONT_LEGS, NOMOVE, KNEE_STAND, 0, 0, DRIFT_COMPENSATION);
  delay(350);

  Serial.println(F("STAND MIDDLE 2/3"));
  Serial.flush();
  setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_HALF_CROUCH, 0, 0, DRIFT_COMPENSATION);
  delay(200);
  setLeg(MIDDLE_LEGS, NOMOVE, KNEE_STAND, 0, 0, DRIFT_COMPENSATION);
  delay(300);

  Serial.println(F("STAND BACK 3/3"));
  Serial.flush();
  setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_HALF_CROUCH, 0, 0, DRIFT_COMPENSATION);
  delay(200);
  setLeg(BACK_LEGS, NOMOVE, KNEE_STAND, 0, 0, DRIFT_COMPENSATION);
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
