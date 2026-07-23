////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Gait Control Implementation
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
////////////////////////////////////////////////////////////////////////////////

#include "gait.h"
#include "leg_motion.h"
#include "servo_control.h"

// Gait State Variables
int curGait = G_STAND;
int curReverse = 0;
unsigned long nextGaitTime = 0;
int ScamperPhase = 0;
unsigned long NextScamperPhaseTime = 0;
long ScamperTracker = 0;

// Last Gait Command Parameters (for repeat)
unsigned int LastGgaittype;
unsigned int LastGreverse;
unsigned int LastGhipforward;
unsigned int LastGhipbackward;
unsigned int LastGkneeup;
unsigned int LastGkneedown;
unsigned int LastGtimeperiod;
int LastGleanangle;   // this can be negative so don't make it unsigned

void gait_tripod(int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod) {
  // this version makes leanangle zero
  gait_tripod(reverse, hipforward, hipbackward,
        kneeup, kneedown, timeperiod, 0);
}

void gait_tripod(int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod, int leanangle) {

  // the gait consists of 6 phases. This code determines what phase
  // we are currently in by using the millis clock modulo the
  // desired time period that all six  phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_TRIPOD_PHASES 6
#define FBSHIFT    12   // shift front legs back, back legs forward (降低防磕头：前腿少往前伸)

  // 安全相位调度：每次最多推进一相，绝不因传感器/I2C/串口阻塞而跳相。
  // 旧的 millis()%period 算法可能从“第一组抬起”直接跳到“第二组抬起”，
  // 漏掉中间落腿相位，造成六腿同时折叠、周期性趴下。
  static int phase = 0;
  static unsigned long nextPhaseTime = 0;
  static unsigned long lastCallTime = 0;
  static long scheduledPeriod = 0;
  static int scheduledReverse = -1;
  unsigned long now = hexmillis();
  unsigned long phaseDuration = (unsigned long)(timeperiod / NUM_TRIPOD_PHASES);
  if (phaseDuration < 1) phaseDuration = 1;

  bool restartSequence = (nextPhaseTime == 0 || scheduledPeriod != timeperiod ||
                          scheduledReverse != reverse ||
                          now - lastCallTime > (unsigned long)timeperiod);
  if (restartSequence) {
    phase = 0;
    nextPhaseTime = now + phaseDuration;
    scheduledPeriod = timeperiod;
    scheduledReverse = reverse;
  } else if ((long)(now - nextPhaseTime) >= 0) {
    phase = (phase + 1) % NUM_TRIPOD_PHASES;
    nextPhaseTime = now + phaseDuration; // 从当前时刻计时，不追赶、不跳过缺失相位
  }
  lastCallTime = now;

  transactServos(); // defer leg motions until after checking for crashes
  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0, 0, leanangle);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT);
      break;

    case 2:
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0, 0, leanangle);
      break;

    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      break;
  }
  commitServos(); // implement all leg motions
}

void gait_tripod_scamper(int reverse, int turn) {

  ScamperTracker += 2;  // for tracking if the user is over-doing it with scamper

  // this is a tripod gait that tries to go as fast as possible by not waiting
  // for knee motions to complete before beginning the next hip motion

  int hipforward, hipbackward;

  if (reverse) {
    hipforward = HIP_BACKWARD;
    hipbackward = HIP_FORWARD;
  } else {
    hipforward = HIP_FORWARD;
    hipbackward = HIP_BACKWARD;
  }

#define FBSHIFT    12   // shift front legs back, back legs forward (降低防磕头：前腿少往前伸)
#define SCAMPERPHASES 6

#ifdef HEXAPOD
#define KNEEDELAY 35
#define HIPDELAY 100
#endif

#ifdef MEGAPOD
#define KNEEDELAY 45
#define HIPDELAY 120
#endif

#ifdef GIGAPOD
#define KNEEDELAY 60
#define HIPDELAY 170
#endif

  if (millis() >= NextScamperPhaseTime) {
    ScamperPhase++;
    if (ScamperPhase >= SCAMPERPHASES) {
      ScamperPhase = 0;
    }
    switch (ScamperPhase) {
      case 0: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 1: NextScamperPhaseTime = millis()+HIPDELAY; break;
      case 2: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 3: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 4: NextScamperPhaseTime = millis()+HIPDELAY; break;
      case 5: NextScamperPhaseTime = millis()+KNEEDELAY; break;
    }

  }

  transactServos();
  switch (ScamperPhase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_SCAMPER, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT, turn);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
      break;

    case 2:
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_SCAMPER, 0, turn);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0, turn);
      break;

    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT, turn);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;
  }
  commitServos();
}

void gait_ripple(int turn, int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod) {
  gait_ripple(turn, reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, 0);
}

void gait_ripple(int turn, int reverse, int hipforward, int hipbackward,
                 int kneeup, int kneedown, long timeperiod, int leanangle) {
  // the gait consists of 19 phases. This code determines what phase
  // we are currently in by using the millis clock modulo the
  // desired time period that all phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (turn) {
    reverse = 1-reverse;  // yeah this is weird but if you're turning you need to reverse the sense of reverse to make left and right turns come out correctly
  }
  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_RIPPLE_PHASES 19

  long t = hexmillis()%timeperiod;
  long phase = (NUM_RIPPLE_PHASES*t)/timeperiod;

  transactServos();

  if (phase == 18) {
    setLeg(ALL_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
  } else {
    int leg = phase/3;  // this will be a number between 0 and 5 because phase==18 is handled above
    leg = 1<<leg;
    int subphase = phase%3;

    switch (subphase) {
      case 0:
        setLeg(leg, NOMOVE, kneeup, 0);
        break;
      case 1:
        setLeg(leg, hipforward, NOMOVE, FBSHIFT, turn);  // move in "raw" mode if turn is engaged, this makes all legs ripple in the same direction
        break;
      case 2:
        setLeg(leg, NOMOVE, kneedown, 0);
        break;
    }
  }
  commitServos();
}

void gait_quad(int turn, int reverse, int hipforward, int hipbackward,
               int kneeup, int kneedown, long timeperiod, int leanangle) {
  // the gait walks using a quadruped gait with middle legs raised up. This code determines what phase
  // we are currently in by using the millis clock modulo the
  // desired time period that all phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (turn) {
    reverse = 1-reverse;  // yeah this is weird but if you're turning you need to reverse the sense of reverse to make left and right turns come out correctly
  }
  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_QUAD_PHASES 6

  long t = hexmillis()%timeperiod;
  long phase = (NUM_QUAD_PHASES*t)/timeperiod;

  transactServos();
  setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_UP_MAX, FBSHIFT_QUAD, 0);

  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at the knee
      setLeg(QUAD1_LEGS, NOMOVE, kneeup, FBSHIFT_QUAD, turn);
      // use the middle legs to try to counter balance
      if (kneeup != kneedown) { // if not standing still
        setLeg(MIDDLE_LEGS, reverse?HIP_BACKWARD_MAX:HIP_FORWARD_MAX, NOMOVE, 0, 1);
      }
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(QUAD1_LEGS, hipforward, NOMOVE, FBSHIFT_QUAD, turn);
      setLeg(QUAD2_LEGS, hipbackward, NOMOVE, FBSHIFT_QUAD, turn);
      break;

    case 2:
      // now put the first set of legs back down on the ground
      setLeg(QUAD1_LEGS, NOMOVE, kneedown, 0, turn);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(QUAD2_LEGS, NOMOVE, kneeup, 0, turn);
      if (kneeup != kneedown) {
         setLeg(MIDDLE_LEGS, reverse?HIP_FORWARD_MAX:HIP_BACKWARD_MAX, NOMOVE, 0, 1);
      }
      break;

    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(QUAD1_LEGS, hipbackward, NOMOVE, FBSHIFT_QUAD, turn);
      setLeg(QUAD2_LEGS, hipforward, NOMOVE, FBSHIFT_QUAD, turn);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(QUAD2_LEGS, NOMOVE, kneedown, 0, turn);
      break;
  }
  commitServos();
}

void gait_belly(int turn, int reverse, int hipforward, int hipbackward,
                int kneeup, int kneedown, long timeperiod, int leanangle) {
  // the gait walks using a rowboat motion while the robot rests on its belly between steps. This code determines what phase
  // we are currently in by using the millis clock modulo the
  // desired time period that all phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (turn) {
    reverse = 1-reverse;  // yeah this is weird but if you're turning you need to reverse the sense of reverse to make left and right turns come out correctly
  }
  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_BELLY_PHASES 4

  long t = hexmillis()%timeperiod;
  long phase = (NUM_BELLY_PHASES*t)/timeperiod;

  transactServos();

  switch (phase) {
    case 0: // lie down with legs out to the sides
      setLeg(ALL_LEGS, NOMOVE, kneeup, FBSHIFT_BELLY, turn);
      break;
    case 1:
      if (turn) {
        for (int i = 0; i < NUM_LEGS; i++) {
          setHipRawAdj(i, hipbackward, FBSHIFT_BELLY);
        }
      } else {
        setLeg(ALL_LEGS, hipbackward, NOMOVE, FBSHIFT_BELLY, turn);
      }
      break;
    case 2:
      setLeg(ALL_LEGS, NOMOVE, kneedown, FBSHIFT_BELLY, turn);
      break;
    case 3:
      if (turn) {
          for (int i = 0; i < NUM_LEGS; i++) {
            setHipRawAdj(i, hipforward, FBSHIFT_BELLY);
          }
      } else {
        setLeg(ALL_LEGS, hipforward, NOMOVE, FBSHIFT_BELLY, turn);
      }
      break;
  }
  commitServos();
}

void gait_command(int gaittype, int reverse, int hipforward, int hipbackward,
                  int kneeup, int kneedown, int leanangle, int timeperiod) {

       if (ServosDetached) { // wake up any sleeping servos
        attach_all_servos();
       }

       switch (gaittype) {
        case 0:
        default:
                   gait_tripod(reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, leanangle);
                   break;
        case 1:
                   turn(reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, leanangle);
                   break;
        case 2:
                   gait_ripple(0, reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, leanangle);
                   break;
        case 3:
                   //gait_sidestep(reverse, timeperiod);
                   break;
       }

       extern byte mode;
       mode = MODE_GAIT;   // this stops auto-repeat of gamepad mode commands
}

void random_gait(int timingfactor) {

#define GATETIME 3500  // number of milliseconds for each demo

  if (millis() > nextGaitTime) {
    curGait++;
    if (curGait >= G_NUMGATES) {
      curGait = 0;
    }
    nextGaitTime = millis() + GATETIME;

    // when switching demo modes, briefly go into a standing position so
    // we're starting at the same position every time.
    stand();
    delay(600);
  }

  switch (curGait) {
    case G_STAND:
      stand();
      break;
    case G_TURN:
      turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, TRIPOD_CYCLE_TIME); // 700
      break;
    case G_TRIPOD:
      gait_tripod(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, TRIPOD_CYCLE_TIME); // 900
      break;
    case G_SCAMPER:
      gait_tripod_scamper((nextGaitTime-(millis())<GATETIME/2),0);  // reverse direction halfway through
      break;
    case G_DANCE:
      stand();
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 145);
      delay(350);
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 35);
      delay(350);
      break;
    case G_BOOGIE:
       extern void boogie_woogie(int legs_flat, int submode, int timingfactor);
       boogie_woogie(NO_LEGS, SUBMODE_1, 2);
       break;
    case G_FIGHT:
      extern void fight_mode(char dpad, int mode, long timeperiod);
      fight_mode('w', SUBMODE_1, FIGHT_CYCLE_TIME);
      break;

    case G_TEETER:
      extern void wave(int dpad);
      wave('r');
      break;

    case G_BALLET:
      extern void flutter();
      flutter();
      break;
  }
}
