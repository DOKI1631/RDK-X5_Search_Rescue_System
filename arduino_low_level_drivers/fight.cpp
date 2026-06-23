////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Fight Control Implementation
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
////////////////////////////////////////////////////////////////////////////////

#include "fight.h"
#include "leg_motion.h"
#include "servo_control.h"
#include "gait.h"
#include "dance.h"

// Front Reverse State
byte FrontReverse = 0;
long DebounceFrontReverse = 0;

void fight_mode(char dpad, int mode, long timeperiod) {

#define HIP_FISTS_FORWARD 130

  if (mode == SUBMODE_3) {
    // in this mode the robot leans forward, left, or right by adjusting hips only

    // this mode retains state and moves slowly, it's for getting something like the joust or
    // capture the flag accessories in position

      switch (dpad) {
      case 's':
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = ServoPos[i+NUM_LEGS];
          ServoTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          ServoTarget[i] = 90;
        }
        break;
      case 'f': // swing hips forward, mirrored
        ServoTarget[5] = ServoTarget[4] = ServoTarget[3] = 125;
        ServoTarget[0] = ServoTarget[1] = ServoTarget[2] = 55;
        break;
      case 'b': // move the knees back up to standing position, leave hips alone
        ServoTarget[5] = ServoTarget[4] = ServoTarget[3] = 55;
        ServoTarget[0] = ServoTarget[1] = ServoTarget[2] = 125;
        break;
      case 'l':
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i] = 170;
        }
        break;
      case 'r':
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i] = 10;
        }
        break;
    }

  } else if (mode == SUBMODE_4) {
    // in this mode the entire robot leans in the direction of the pushbuttons
    // and the weapon button makes the robot return to standing position.

    // Only knees are altered by this, not hips (other than the reset action for
    // the special D-PAD button)

    // this mode does not immediately set servos to final positions, instead it
    // moves them toward targets slowly.

    switch (dpad) {
      case 's':
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = ServoPos[i+NUM_LEGS];
          ServoTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          ServoTarget[i] = 90;
        }
        break;
      case 'f': // move knees into forward crouch, leave hips alone

        if (ServoPos[8] == KNEE_STAND) { // the back legs are standing, so crouch the front legs
          ServoTarget[6]=ServoTarget[11]=KNEE_CROUCH;
          ServoTarget[7]=ServoTarget[10]=KNEE_HALF_CROUCH;
          ServoTarget[8]=ServoTarget[9]=KNEE_STAND;
        } else { // bring the back legs up first
          for (int i = 0; i < NUM_LEGS; i++) {
            ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          }
        }
        break;
      case 'b': // move back legs down so robot tips backwards
        if (ServoPos[6] == KNEE_STAND) { // move the back legs down
          ServoTarget[6]=ServoTarget[11]=KNEE_STAND;
          ServoTarget[7]=ServoTarget[10]=KNEE_HALF_CROUCH;
          ServoTarget[8]=ServoTarget[9]=KNEE_CROUCH;
        } else { // front legs are down, return to stand first
            for (int i = 0; i < NUM_LEGS; i++) {
              ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
            }
        }
        break;
     case 'l':
        if (ServoPos[9] == KNEE_STAND) {
          ServoTarget[6]=ServoTarget[8] = KNEE_HALF_CROUCH;
          ServoTarget[7]=KNEE_CROUCH;
          ServoTarget[9]=ServoTarget[10]=ServoTarget[11]=KNEE_STAND;
        } else {
          for (int i = 0; i < NUM_LEGS; i++) {
            ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          }
        }
        break;
      case 'r':
        if (ServoPos[6] == KNEE_STAND) {
          ServoTarget[6]=ServoTarget[7]=ServoTarget[8] = KNEE_STAND;
          ServoTarget[9]=ServoTarget[11]=KNEE_HALF_CROUCH;
          ServoTarget[10]=KNEE_CROUCH;
        } else {
          for (int i = 0; i < NUM_LEGS; i++) {
            ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          }
        }
        break;

    }
  }

  if (mode >= SUBMODE_3) {
    return; // we're done, the smoothmove function will take care of reaching the target positions
  }

  // If we get here, we are in either submode 1 or 2
  //
  // submode 1: fight with two front legs, individual movement
  // submode 2: fight with two front legs, in unison

  setLeg(MIDDLE_LEGS, HIP_FORWARD+10, KNEE_STAND, 0);
  setLeg(BACK_LEGS, HIP_BACKWARD, KNEE_STAND, 0);

  switch (dpad) {
    case 's':  // stop mode: both legs straight out forward
      setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_NEUTRAL, 0);

      break;

    case 'f':  // both front legs move up in unison
      setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_UP, 0);
      break;

    case 'b':  // both front legs move down in unison
      setLeg(FRONT_LEGS, HIP_FORWARD, KNEE_STAND, 0);
      break;

    case 'l':  // left front leg moves left, right stays forward
      if (mode == SUBMODE_1) {
        setLeg(LEG0, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_RELAX, 0);
      } else {
        // both legs move in unison in submode B
        setLeg(LEG0, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG5, HIP_FISTS_FORWARD+30, KNEE_RELAX, 0);
      }
      break;

    case 'r':  // right front leg moves right, left stays forward
      if (mode == SUBMODE_1) {
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_RELAX, 0);
      } else { // submode B
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG0, HIP_FISTS_FORWARD+30, KNEE_RELAX, 0);
      }
      break;

    case 'w':  // automatic ninja motion mode with both legs swinging left/right/up/down furiously!

#define NUM_PUGIL_PHASES 8
        {  // we need a new scope for this because there are local variables

        long t = hexmillis()%timeperiod;
        long phase = (NUM_PUGIL_PHASES*t)/timeperiod;

        switch (phase) {
          case 0:
            // Knees down, hips forward
            setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_RELAX, 0);
            break;

          case 1:
            // Knees up, hips forward
            setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_UP, 0);
            break;

          case 2:
            // Knees neutral, hips neutral
            setLeg(FRONT_LEGS, HIP_BACKWARD, KNEE_NEUTRAL, 0);
            break;

          case 3:
            // Knees up, hips neutral
            setLeg(FRONT_LEGS, HIP_BACKWARD, KNEE_UP, 0);
            break;

          case 4:
             // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_UP, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_STAND, 0);
             break;

          case 5:
              // kick phase 2
              // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_STAND, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_UP, 0);
             break;

          case 6:
             // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_UP, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_DOWN, 0);
             break;

          case 7:
              // kick phase 2
              // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_DOWN, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_UP, 0);
             break;
        }
      }
  }
}

void fight2(int cmd, int submode) {

   switch (submode) {

    case SUBMODE_1: // Corgi Mode
      corgi(cmd);
      break;

    case SUBMODE_2: // single arm manipulator mode, right front arm
    switch (cmd) {
      case 's':
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = ServoPos[i+NUM_LEGS];
          ServoTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          ServoTarget[i] = 90;
        }
        break;
      case 'f':
          ServoTarget[5+KNEE_OFFSET] = 180;
        break;
      case 'b':
          ServoTarget[5+KNEE_OFFSET] = 0;
        break;
      case 'l':
          ServoTarget[5] = 0;
        break;
      case 'r':
          ServoTarget[5] = 180;
        break;
      }
      break; // end of submode 2


    case SUBMODE_3:  // single arm manipulator mode, left front arm
      switch (cmd) {
      case 's':
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = ServoPos[i+NUM_LEGS];
          ServoTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          ServoTarget[i] = 90;
        }
        break;
      case 'f': // raise leg 0
          ServoTarget[0+KNEE_OFFSET] = 180;
        break;
      case 'b': // lower leg 0
          ServoTarget[0+KNEE_OFFSET] = 0;
        break;
      case 'l': //move left leg 0
          ServoTarget[0] = 0;
        break;
      case 'r': //twist down
          ServoTarget[0] = 180;
        break;
      }
      break; // end of submode 3

    case SUBMODE_4: // rise/fall mode
      // this mode retains state and moves slowly, it's for getting something like the joust or
      // capture the flag accessories in position

      switch (cmd) {
      case 's':
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = ServoPos[i+NUM_LEGS];
          ServoTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i+KNEE_OFFSET] = KNEE_STAND;
          ServoTarget[i] = 90;
        }
        break;
      case 'f': // rise straight up to tiptoes
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i] = 90;
          ServoTarget[i+KNEE_OFFSET] = 0;
        }
        break;
      case 'b': // fall straight down
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i] = 90;
          ServoTarget[i+KNEE_OFFSET] = 180;
        }
        break;
      case 'l': //twist up
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i] = 0;
          ServoTarget[i+KNEE_OFFSET] = 0;
        }
        break;
      case 'r': //twist down
        for (int i = 0; i < NUM_LEGS; i++) {
          ServoTarget[i] = 180;
          ServoTarget[i+KNEE_OFFSET] = 180;
        }
        break;
    }
    break; // end of submode 4
  }
}

void walk2(int cmd, int submode) {

  switch (submode) {
    case '1': // RIPPLE GAIT

      switch (cmd) {
      case 'f':
        gait_ripple(0, 0, HIP_FORWARD_RIPPLE, HIP_BACKWARD_RIPPLE, KNEE_RIPPLE_UP, KNEE_RIPPLE_DOWN, RIPPLE_CYCLE_TIME, 0);
        break;
      case 'b':
        gait_ripple(0, 1, HIP_FORWARD_RIPPLE, HIP_BACKWARD_RIPPLE, KNEE_RIPPLE_UP, KNEE_RIPPLE_DOWN, RIPPLE_CYCLE_TIME, 0);
        break;
      case 'l':
        gait_ripple(1, 0, HIP_FORWARD_RIPPLE, HIP_BACKWARD_RIPPLE, KNEE_RIPPLE_UP, KNEE_RIPPLE_DOWN, RIPPLE_CYCLE_TIME, 0);
        break;
      case 'r':
        gait_ripple(1, 1, HIP_FORWARD_RIPPLE, HIP_BACKWARD_RIPPLE, KNEE_RIPPLE_UP, KNEE_RIPPLE_DOWN, RIPPLE_CYCLE_TIME, 0);
        break;
      case 's':
        stand();
        break;
      case 'w':
        beep(400);
        gait_ripple(0, 0, HIP_NEUTRAL, HIP_NEUTRAL, KNEE_RIPPLE_UP, KNEE_RIPPLE_DOWN, RIPPLE_CYCLE_TIME, 0);
        break;
      }
      break;

    case '2': //BELLY CRAWL GAIT
      switch (cmd) {
      case 'f':
        gait_belly(0, 0, HIP_FORWARD_BELLY, HIP_BACKWARD_BELLY, KNEE_BELLY_UP, KNEE_BELLY_DOWN, BELLY_CYCLE_TIME, 0);
        break;
      case 'b':
        gait_belly(0, 1, HIP_FORWARD_BELLY, HIP_BACKWARD_BELLY, KNEE_BELLY_UP, KNEE_BELLY_DOWN, BELLY_CYCLE_TIME, 0);
        break;
      case 'l':
        gait_belly(1, 0, HIP_FORWARD_BELLY, HIP_BACKWARD_BELLY, KNEE_BELLY_UP, KNEE_BELLY_DOWN, BELLY_CYCLE_TIME, 0);
        break;
      case 'r':
        gait_belly(1, 1, HIP_FORWARD_BELLY, HIP_BACKWARD_BELLY, KNEE_BELLY_UP, KNEE_BELLY_DOWN, BELLY_CYCLE_TIME, 0);
        break;
      case 'w':
        // stomp in place
        beep(300);
        gait_belly(0, 0, HIP_NEUTRAL, HIP_NEUTRAL, KNEE_BELLY_UP, KNEE_BELLY_DOWN, BELLY_CYCLE_TIME, 0);
        break;
      case 's':
        setLeg(ALL_LEGS, HIP_NEUTRAL, 85, 0, 0);
        break;
      }
      break;

    case '3':  // Quadruped Gait

      switch (cmd) {
      case 'f':
        gait_quad(0, 0, HIP_FORWARD_QUAD, HIP_BACKWARD_QUAD, KNEE_QUAD_UP, KNEE_QUAD_DOWN, QUAD_CYCLE_TIME, 0);
        break;
      case 'b':
        gait_quad(0, 1, HIP_FORWARD_QUAD, HIP_BACKWARD_QUAD, KNEE_QUAD_UP, KNEE_QUAD_DOWN, QUAD_CYCLE_TIME, 0);
        break;
      case 'l':
        gait_quad(1, 0, HIP_FORWARD_QUAD, HIP_BACKWARD_QUAD, KNEE_QUAD_UP, KNEE_QUAD_DOWN, QUAD_CYCLE_TIME, 0);
        break;
      case 'r':
        gait_quad(1, 1, HIP_FORWARD_QUAD, HIP_BACKWARD_QUAD, KNEE_QUAD_UP, KNEE_QUAD_DOWN, QUAD_CYCLE_TIME, 0);
        break;
      case 'w': // stomp and honk
        beep(500);
        gait_quad(1, 1, HIP_NEUTRAL, HIP_NEUTRAL, KNEE_QUAD_UP, KNEE_QUAD_DOWN, QUAD_CYCLE_TIME, 0);
        break;
      case 's':
        gait_quad(1, 1, HIP_NEUTRAL, HIP_NEUTRAL, KNEE_QUAD_DOWN, KNEE_QUAD_DOWN, QUAD_CYCLE_TIME, 0);
        break;
      }
      break;

    case '4': // redefine front gait while scampering

      switch (cmd) {
      case 'f':
        servoOffset = FrontReverse;
        gait_tripod_scamper(0,0);
        servoOffset = 0;
        break;
      case 'b':
         servoOffset = FrontReverse;
         gait_tripod_scamper(1, 0);
         servoOffset = 0;
        break;
      case 'l':
        servoOffset=1+FrontReverse;
        gait_tripod_scamper(0,0);
        servoOffset=0;
        break;
      case 'r':
        servoOffset=5+FrontReverse;
        gait_tripod_scamper(0,0);
        servoOffset=0;
        break;
      case 'w': // reverse entire sense of the robot legs 180 degrees
         {
            long now = millis();
            if (now > DebounceFrontReverse) {
              FrontReverse = 3 - FrontReverse; // it will toggle between 3 and 0
              beep(500+300*FrontReverse,50); //feedback to user: high pitch means reversed, low pitch means back to normal
              DebounceFrontReverse = now+300;
            }
         }
        break;
      case 's':
        stand();
        break;
      }
      break;
  }
}
