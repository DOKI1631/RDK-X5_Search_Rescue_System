////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Dance Control Implementation
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
////////////////////////////////////////////////////////////////////////////////

#include "dance.h"
#include "leg_motion.h"
#include "servo_control.h"
#include "gait.h"

void dance(int legs_up, int submode, int timingfactor) {
   setLeg(legs_up, NOMOVE, KNEE_UP, 0, 0);
   setLeg((legs_up^0b111111), NOMOVE, ((submode==SUBMODE_1)?KNEE_STAND:KNEE_TIPTOES), 0, 0);

#define NUM_DANCE_PHASES 2

  long t = hexmillis()%(600*timingfactor);
  long phase = (NUM_DANCE_PHASES*t)/(600*timingfactor);

  switch (phase) {
    case 0:
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 140);
      break;
    case 1:
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 40);
      break;
  }
}

void boogie_woogie(int legs_flat, int submode, int timingfactor) {

      setLeg(ALL_LEGS, NOMOVE, KNEE_UP, 0);

#define NUM_BOOGIE_PHASES 2

  long t = hexmillis()%(400*timingfactor);
  long phase = (NUM_BOOGIE_PHASES*t)/(400*timingfactor);

  switch (phase) {
    case 0:
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 140);
      break;

    case 1:
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 40);
      break;
  }
}

void flutter() {   // ballet flutter legs on pointe
#define NUM_FLUTTER_PHASES 4
#define FLUTTER_TIME 200
#define KNEE_FLUTTER (KNEE_TIPTOES+20)

  long t = hexmillis()%(FLUTTER_TIME);
  long phase = (NUM_FLUTTER_PHASES*t)/(FLUTTER_TIME);

  setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);

  switch (phase) {
    case 0:
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_FLUTTER, 0, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 1:
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 2:
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_FLUTTER, 0, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 3:
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
  }

}

void dance_ballet(int dpad) {   // ballet flutter legs on pointe

#define BALLET_TIME 250

  switch (dpad) {

    default:
    case 's': tiptoes(); return;

    case 'w': flutter(); return;

    case 'l':
      turn(1, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'r':
      turn(0, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'f':
      gait_tripod(0, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'b':
      gait_tripod(1, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;
  }
}

void dance_hands(int dpad) {

  setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0);
  setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0);

  switch (dpad) {
    case 's':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_UP, 0, 0);
      break;
    case 'f':
      setLeg(MIDDLE_LEGS, HIP_FORWARD_MAX, KNEE_UP_MAX, 0, 0);
      break;
    case 'b':
      setLeg(MIDDLE_LEGS, HIP_BACKWARD_MAX, KNEE_UP_MAX, 0, 0);
      break;
    case 'l':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
      setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, 0, 0);
      setLeg(LEG4, NOMOVE, KNEE_UP_MAX, 0, 0);
      break;
    case 'r':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
      setLeg(LEG1, NOMOVE, KNEE_UP_MAX, 0, 0);
      setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, 0, 0);
      break;
    case 'w':
      // AUTOMATIC MODE
#define NUM_HANDS_PHASES 2
#define HANDS_TIME_PERIOD 400
        {
        long t = hexmillis()%HANDS_TIME_PERIOD;
        long phase = (NUM_HANDS_PHASES*t)/HANDS_TIME_PERIOD;


        switch (phase) {
          case 0:
            setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
            setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, 0, 0);
            setLeg(LEG4, NOMOVE, KNEE_UP_MAX, 0, 0);
            break;

          case 1:
            setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
            setLeg(LEG1, NOMOVE, KNEE_UP_MAX, 0, 0);
            setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, 0, 0);
            break;

        }
      }
      break;
  }
}

void dance_dab(int timingfactor) {
#define NUM_DAB_PHASES 3

  long t = hexmillis()%(1100*timingfactor);
  long phase = (NUM_DAB_PHASES*t)/(1100*timingfactor);

  switch (phase) {
    case 0:
      stand(); break;

    case 1:
      setKnee(6, KNEE_UP); break;

    case 2:
      for (int i = 0; i < NUM_LEGS; i++)
         if (i != 0) setHipRaw(i, 40);
      setHipRaw(0, 140);
      break;
  }
}

void dance_twitch(int cmd, int hipforward, int hipbackward, int kneeup, int kneedown, int twitchmode) {
  // Dance by twitching legs. This code determines what phase
  // we are currently in by using the millis clock modulo the
  // desired time period that all phases should consume.

#define NUM_TWITCH_PHASES 4
#define TWITCH_TIME 300
  int ttime = TWITCH_TIME;

  if (twitchmode == MODESWAY) {
    ttime = 750;
  }
  long t = hexmillis()%ttime;
  long phase = (NUM_TWITCH_PHASES*t)/ttime;

  int legs = ALL_LEGS;

  int revlr = 0;

  switch (cmd) {
    case 'f':
       legs = ALL_LEGS; break;
    case 'l':
       legs = LEFT_LEGS; break;
    case 'b':
       legs = ALL_LEGS;
       revlr = 1;
       break;
    case 'r':
       legs = RIGHT_LEGS; break;
  }

  transactServos();
  if (twitchmode == MODETWITCH) {
    setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, TWITCH_ADJ);
  } else {
    setLeg(ALL_LEGS, HIP_NEUTRAL, (KNEE_TIPTOES+KNEE_DOWN)/2, TWITCH_ADJ);
    switch (cmd) {
      case 'f':
        // hands alternate from knee_neutral to knee_up_max at same time
        setLeg(MIDDLE_LEGS, NOMOVE, KNEE_NEUTRAL, TWITCH_ADJ); break;
      case 'b':
        // no hands at all, all legs down
        break;
      case 'l':
        // hands sway in opposite direction of  grounded legs
        setLeg(MIDDLE_LEGS, NOMOVE, KNEE_UP, TWITCH_ADJ);
        break;
      case 'r':
        // hands pivot at the hips
        setLeg(MIDDLE_LEGS, NOMOVE, KNEE_UP_MAX, TWITCH_ADJ, 1);
        break;
      case 'w':
         // sway both back and forth and with hip swings
         break;
    }
  }

  switch (phase) {
    case 0:
      if (twitchmode == MODETWITCH) {
        setLeg(legs, hipforward, kneeup, TWITCH_ADJ);
        if (revlr) {
          setLeg(LEFT_LEGS, hipbackward, kneeup, TWITCH_ADJ);
        }
      } else { //sway
        setLeg(LEFT_LEGS, HIP_NEUTRAL, kneeup, TWITCH_ADJ);
        setLeg(RIGHT_LEGS, HIP_NEUTRAL, kneedown, TWITCH_ADJ);
        switch (cmd) {
          case 'f':
            setLeg(MIDDLE_LEGS, NOMOVE, KNEE_UP_MAX, TWITCH_ADJ);
            break;
          case 'b':
            break;
          case 'l':
            setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, TWITCH_ADJ);
            setLeg(LEG4, NOMOVE, KNEE_UP_MAX, TWITCH_ADJ);
            break;
          case 'r':
            setLeg(MIDDLE_LEGS, HIP_FORWARD, KNEE_UP_MAX, TWITCH_ADJ);
            break;
          case 'w':
            setLeg(RIGHT_LEGS, HIP_FORWARD_MAX, kneedown, TWITCH_ADJ);
            break;
        }
      }
      break;

    case 1: break;

    case 2:
      if (twitchmode == MODETWITCH) {
        setLeg(legs, hipbackward, kneedown, TWITCH_ADJ);
        if (revlr) {
          setLeg(LEFT_LEGS, hipforward, kneeup, TWITCH_ADJ);
        }
      } else { //sway
        setLeg(LEFT_LEGS, hipforward, kneedown, TWITCH_ADJ);
        setLeg(RIGHT_LEGS, hipforward, kneeup, TWITCH_ADJ);
        switch (cmd) {
          case 'f':
            setLeg(MIDDLE_LEGS, NOMOVE, KNEE_UP, TWITCH_ADJ); break;
          case 'b':
            break;
          case 'l':
            setLeg(LEG1, NOMOVE, KNEE_UP_MAX, TWITCH_ADJ);
            setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, TWITCH_ADJ);
            break;
          case 'r':
            setLeg(MIDDLE_LEGS, HIP_BACKWARD, KNEE_UP_MAX, TWITCH_ADJ, 1);
            break;
          case 'w':
            setLeg(LEFT_LEGS, HIP_FORWARD_MAX, kneedown, TWITCH_ADJ);
            break;
        }
      }

      break;

    case 3: break;

  }
  commitServos();

}

void randomizeLeg(int legnum) {
  int hip = ServoPos[legnum];
#define RANDMAX 20

  hip += random(-RANDMAX,RANDMAX);
  hip = constrain(hip, HIP_BACKWARD, HIP_FORWARD);
  setHipRaw(legnum, hip);

  int knee = ServoPos[legnum+KNEE_OFFSET];
  knee += random(-RANDMAX, RANDMAX);
  knee = constrain(knee, KNEE_NEUTRAL-10, KNEE_UP_MAX);
  setKnee(legnum, knee);

}

void dance_brownian(int cmd) {

  switch(cmd) {
    case 'f':
      randomizeLeg(0);
      randomizeLeg(5);
      break;

    case 'b':
      randomizeLeg(2);
      randomizeLeg(3);
      break;

    case 'l':
      for (int i = 0; i < 3; i++) {
          randomizeLeg(i);
      }
      break;

    case 'r':
      for (int i = 3; i < 6; i++) {
          randomizeLeg(i);
      }
      break;

    case 'w':
      for (int i = 0; i < 6; i++) {
          randomizeLeg(i);
      }
      break;

    case 's':
      setLeg(ALL_LEGS, NOMOVE, KNEE_NEUTRAL, 0, 1);
      return;
  }
}

void dance_star(int cmd) {
  // Dance by making star patterns with legs

#define NUM_STAR_PHASES 4
#define STAR_TIME 700

  long t = hexmillis()%STAR_TIME;
  long phase = (NUM_STAR_PHASES*t)/STAR_TIME;

  int kneeposleft = KNEE_STAND;
  int kneeposright = KNEE_STAND;
  byte sidebyside = 0;

  switch (cmd) {
    case 'f':
       kneeposleft = kneeposright = KNEE_DOWN;
       break;

    case 'b':
       kneeposleft = kneeposright = KNEE_UP_MAX;
       break;

    case 'l':
      kneeposleft = KNEE_DOWN;
      kneeposright = KNEE_NEUTRAL;
      break;

    case 'r':
      kneeposleft = KNEE_NEUTRAL;
      kneeposright = KNEE_DOWN;
      break;

    case 'w':
      kneeposleft = KNEE_UP;
      kneeposright = KNEE_DOWN;
      break;

    case 's':
      setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_DOWN, 0, 1);
      return;
  }

  transactServos();

  setLeg(ALL_LEGS, HIP_NEUTRAL, (kneeposleft+kneeposright)/2, 0, 1);

  switch (phase) {
    case 0:
      setLeg(TRIPOD1_LEGS, HIP_FORWARD+5, kneeposleft+20, 0, 1);
      setLeg(TRIPOD2_LEGS, HIP_BACKWARD-5, kneeposright-10, 0, 1);
      if (sidebyside) {
        setLeg(LEFT_LEGS, NOMOVE, KNEE_UP, 0, 1);
      }
      break;

    case 1: break;

    case 2:
      setLeg(TRIPOD2_LEGS, HIP_FORWARD+5, kneeposright+20, 0, 1);
      setLeg(TRIPOD1_LEGS, HIP_BACKWARD-5, kneeposleft-10, 0, 1);
      if (sidebyside) {
        setLeg(RIGHT_LEGS, NOMOVE, KNEE_UP, 0, 1);
      }
      break;

    case 3: break;

  }
  commitServos();

}

void dance2(int cmd, int submode) {
  switch (submode) {

    case SUBMODE_1: // TWITCH DANCE

      switch (cmd) {
      case 'f':
        dance_twitch(cmd, HIP_FORWARD_RIPPLE, HIP_BACKWARD_RIPPLE, NOMOVE, NOMOVE, MODETWITCH);
        break;
      case 'b':
        dance_twitch(cmd, HIP_FORWARD_RIPPLE, HIP_BACKWARD_RIPPLE, NOMOVE, NOMOVE, MODETWITCH);
        break;
      case 'l':
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_DOWN, MODETWITCH);
        break;
      case 'r':
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_DOWN, MODETWITCH);
        break;
      case 'w': // special
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_DOWN, MODETWITCH);
        break;
      case 's': // stop mode, just stand tall in twitch configuration
        setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, TWITCH_ADJ);
        break;
      }
      break;

    case SUBMODE_2: // SWAY DANCE

      switch (cmd) {
      case 'f':
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_NEUTRAL, MODESWAY);
        break;
      case 'b':
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_NEUTRAL, MODESWAY);
        break;
      case 'l':
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_NEUTRAL, MODESWAY);
        break;
      case 'r':
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_NEUTRAL, MODESWAY);
        break;
      case 'w': // special
        dance_twitch(cmd, NOMOVE, NOMOVE, KNEE_TIPTOES, KNEE_NEUTRAL, MODESWAY);
        break;
      case 's': // stop mode, just stand tall in twitch configuration
        setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, TWITCH_ADJ);
        break;
      }
      break;


    case SUBMODE_3:  // STAR DANCE
      dance_star(cmd);
      break;

    case SUBMODE_4:
      dance_brownian(cmd);
      break;
  }
}

void wave(int dpad) {

#define NUM_WAVE_PHASES 12
#define WAVE_CYCLE_TIME 900
#define KNEE_WAVE  60
  long t = hexmillis()%WAVE_CYCLE_TIME;
  long phase = (NUM_WAVE_PHASES*t)/WAVE_CYCLE_TIME;

  if (dpad == 'b') {
    phase = 11-phase;  // go backwards
  }

  switch (dpad) {
    case 'f':
    case 'b':
      // swirl around
      setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0); // keep hips stable at 90 degrees
      if (phase < NUM_LEGS) {
        setKnee(phase, KNEE_WAVE);
      } else {
        setKnee(phase-NUM_LEGS, KNEE_STAND);
      }
      break;
    case 'l':
      // teeter totter around font/back legs

      if (phase < NUM_WAVE_PHASES/2) {
        setKnee(0, KNEE_TIPTOES);
        setKnee(5, KNEE_STAND);
        setHipRaw(0, HIP_FORWARD);
        setHipRaw(5, HIP_BACKWARD-40);
        setKnee(2, KNEE_TIPTOES);
        setKnee(3, KNEE_STAND);
        setHipRaw(2, HIP_BACKWARD);
        setHipRaw(3, HIP_FORWARD+40);

        setLeg(LEG1, HIP_NEUTRAL, KNEE_TIPTOES, 0);
        setLeg(LEG4, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
      } else {
        setKnee(0, KNEE_STAND);
        setKnee(5, KNEE_TIPTOES);
        setHipRaw(0, HIP_FORWARD+40);
        setHipRaw(5, HIP_BACKWARD);
        setKnee(2, KNEE_STAND);
        setKnee(3, KNEE_TIPTOES);
        setHipRaw(2, HIP_BACKWARD-40);
        setHipRaw(3, HIP_FORWARD);

        setLeg(LEG1, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
        setLeg(LEG4, HIP_NEUTRAL, KNEE_TIPTOES, 0);
      }
      break;
    case 'r':
      // teeter totter around middle legs
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
      if (phase < NUM_LEGS) {
        setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
        setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
      } else {
        setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
        setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
      }
      break;
    case 'w':
      // lay on ground and make legs go around in a wave
      setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0);
      int p = phase/2;
      for (int i = 0; i < NUM_LEGS; i++) {
        if (i == p) {
          setKnee(i, KNEE_UP_MAX);
        } else {
          setKnee(i, KNEE_NEUTRAL);
        }
      }
      return;
      if (phase < NUM_LEGS) {
        setKnee(phase/2, KNEE_UP);
      } else {
        int p = phase-NUM_LEGS;
        if (p < 0) p+=NUM_LEGS;
        setKnee(p/2, KNEE_NEUTRAL+10);
      }
      break;
  }
}

void wave_hello(int cmd) {

#define NUM_HELLO_PHASES 4
#define HELLO_TIME 300

  long t = hexmillis()%HELLO_TIME;
  long phase = (NUM_HELLO_PHASES*t)/HELLO_TIME;

  transactServos();
  setLeg(ALL_LEGS&(~LEG5), HIP_NEUTRAL, KNEE_STAND, 0); // every leg except leg 5

  switch (phase) {
    case 0:
      if (cmd == 'r') {
        setLeg(LEG5, HIP_FORWARD, KNEE_UP_MAX, 0);  // normal wave, up and down
      } else {
        setLeg(LEG5, HIP_FORWARD, KNEE_UP_MAX, 0); // queen's wave, side to side
      }
    case 1: break;

    case 2:
      if (cmd == 'r') {
        setLeg(LEG5, HIP_FORWARD, KNEE_UP, 0);
      } else {
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP_MAX, 0);
      }
      break;

    case 3: break;

  }
  commitServos();
}

void corgi(int begmode) {

#define NUM_DOGBEG_PHASES 4
#define DOGBEG_TIME 350

  long t = hexmillis()%DOGBEG_TIME;
  long phase = (NUM_DOGBEG_PHASES*t)/DOGBEG_TIME;

  transactServos();
  setLeg(BACK_LEGS, HIP_BACKWARD, KNEE_UP, 0);
  setLeg(MIDDLE_LEGS, HIP_FORWARD, KNEE_TIPTOES, 0);

  if (begmode == 's') { // just standing here, nothing to do
    commitServos();
    return;
  }

  switch (phase) {
    case 0:
      switch (begmode) {
        case 'f': // beg for food
          setLeg(LEG0, HIP_FORWARD, KNEE_DOWN, 0);
          setLeg(LEG5, HIP_FORWARD, KNEE_UP, 0);
          break;
        case 'b': // clap
          setLeg(LEG0, HIP_FORWARD+30, KNEE_UP, 0);
          setLeg(LEG5, HIP_FORWARD+30, KNEE_UP, 0);
          break;
        case 'w': // sway
          setLeg(LEG0, HIP_FORWARD+30, KNEE_UP, 0);
          setLeg(LEG5, HIP_FORWARD-30, KNEE_UP, 0);
          break;
        case 'l':
          setLeg(LEG0, HIP_FORWARD, KNEE_UP_MAX, 0); // queen's wave, side to side
          setLeg(LEG5, HIP_FORWARD, KNEE_DOWN, 0);
          break;
        case 'r':
          setLeg(LEG5, HIP_FORWARD, KNEE_UP_MAX, 0);  // normal wave, up and down
          setLeg(LEG0, HIP_FORWARD, KNEE_DOWN, 0);
          break;
      }
    case 1: break;

    case 2:
       switch (begmode) {
        case 'f':
          setLeg(LEG0, HIP_FORWARD, KNEE_UP, 0);
          setLeg(LEG5, HIP_FORWARD, KNEE_DOWN, 0);
          break;
        case 'b':
        case 'w':
          setLeg(LEG0, HIP_FORWARD, KNEE_UP, 0);
          setLeg(LEG5, HIP_FORWARD, KNEE_UP, 0);
          break;
        case 'r':
          setLeg(LEG5, HIP_FORWARD, KNEE_UP, 0);
          break;
        case 'l':
          setLeg(LEG0, HIP_NEUTRAL, KNEE_UP_MAX, 0);
          break;
      }

      break;

    case 3: break;

  }
  commitServos();

}
