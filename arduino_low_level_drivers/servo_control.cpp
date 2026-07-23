////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Servo Control Implementation
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
////////////////////////////////////////////////////////////////////////////////

#include "servo_control.h"
#include <EEPROM.h>

// Global Variables
byte deferServoSet = 0;
int servoOffset = 0;  // be very careful when using this. It redefines the front of the robot.
int ServosDetached = 0;

// Servo Sleep Detection
unsigned long freqWatchDog = 0;
unsigned long SuppressScamperUntil = 0;  // if we had to wake up the servos, suppress the power hunger scamper mode for a while

// Trim State
extern byte TrimInEffect;
extern byte TrimCurLeg;
extern byte TrimPose;

void resetServoDriver() {
  servoDriver.begin();
  servoDriver.setPWMFreq(PWMFREQUENCY);  // Analog servos run at ~60 Hz updates
}

void setServo(int servonum, int position) {
  // ServoPos/ServoTime/ServoTrim 只有 12 项；禁止附件通道或负索引越界。
  if (servonum < 0 || servonum >= 2 * NUM_LEGS) return;
  position = constrain(position,0,180);

  if (servonum < 12 && servoOffset != 0) {  // we don't want this to effect accessory servos like the grip arm

    // the offset can be used to define a new "front" of the robot
    // Always be sure to reset servoOffset to 0 after making your servo moves
    // shift both the hips and legs independently in cycles 6 long
    int tmp = ((servonum + servoOffset)%6);
    if (servonum>5) { // if it was a hip, make it still be a hip (the mod 6 above would have made it too low)
      tmp += 6;
    }
    servonum = constrain(tmp, 0, 11);

  }

  if (position != ServoPos[servonum]) {
    ServoTime[servonum] = millis();
  }
  ServoPos[servonum] = position;  // keep data on where the servo was last commanded to go

  int p = map(position,0,180,SERVOMIN,SERVOMAX);

  if (TrimInEffect && servonum < 12) {
    p += ServoTrim[servonum] - TRIM_ZERO;   // adjust microseconds by trim value which is renormalized to the range -127 to 128
  }

  if (!deferServoSet) {
    servoDriver.setPWM(servonum, 0, p);
  }
}

void transactServos() {
  deferServoSet = 1;
}

void commitServos() {
  checkForCrashingHips();
  deferServoSet = 0;
  int tmp = servoOffset;
  servoOffset = 0;  // want true servo positions not virtual

  // 分批提交 12 个舵机（3 组 × 4 个），组间延迟 8ms
  // 避免 12 个舵机同时启动造成的瞬时电流尖峰 → 电池电压骤降 → PCA9685 欠压休眠
  // MG90S 启动电流 ≈700mA/个，12 个同时 = 8.4A 峰值；分 3 组降至 ≈2.8A
  for (int group = 0; group < 3; group++) {
    int startServo = group * 4;
    int endServo = (group + 1) * 4;
    if (endServo > 2 * NUM_LEGS) endServo = 2 * NUM_LEGS;
    for (int servo = startServo; servo < endServo; servo++) {
      setServo(servo, ServoPos[servo]);
    }
    if (group < 2) delay(8);  // 组间延时，让电源恢复
  }

  servoOffset = tmp; // put it back
}

// checkForCrashingHips takes a look at the leg angles and tries to figure out if the commanded
// positions might cause servo stall.
void checkForCrashingHips() {

  int tmp = servoOffset;
  servoOffset = 0; // we want this to operate on actual servo numbers not remapped numbers
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    if (ServoPos[leg] > 85) {
      continue; // it's not possible to crash into the next leg in line unless the angle is 85 or less
    }
    int nextleg = ((leg+1)%NUM_LEGS);
    if (ServoPos[nextleg] < 100) {
      continue;   // it's not possible for there to be a crash if the next leg is less than 100 degrees
    }
    int diff = ServoPos[nextleg] - ServoPos[leg];
    // There's a fairly linear relationship
    if (diff <= 85) {
      // if the difference between the two leg positions is less than about 85 then there
      // is not going to be a crash (or maybe just a slight touch that won't really cause issues)
      continue;
    }
    // if we get here then the legs are touching, we will adjust them so that the difference is less than 85
    int adjust = (diff-85)/2 + 1;  // each leg will get adjusted half the amount needed to avoid the crash

#ifdef DEBUG_OUTPUT
    Serial.print("#CRASH:");
    Serial.print(leg);Serial.print("="); Serial.print(ServoPos[leg]);
    Serial.print("/");Serial.print(nextleg);Serial.print("="); Serial.print(ServoPos[nextleg]);
    Serial.print(" Diff="); Serial.print(diff); Serial.print(" ADJ="); Serial.println(adjust);
#endif

    setServo(leg, ServoPos[leg] + adjust);
    setServo(nextleg, ServoPos[nextleg] - adjust);

  }
  servoOffset = tmp;
}

void attach_all_servos() {
  DEBUG_PRINT("ATTACH");
  // 分批唤醒，与 commitServos 同样的原因：避免 12 舵机同时启动的电流冲击
  for (int group = 0; group < 3; group++) {
    int startServo = group * 4;
    int endServo = (group + 1) * 4;
    if (endServo > 2 * NUM_LEGS) endServo = 2 * NUM_LEGS;
    for (int i = startServo; i < endServo; i++) {
      setServo(i, ServoPos[i]);
      DEBUG_PRINT(ServoPos[i]); DEBUG_PRINT(":");
    }
    if (group < 2) delay(8);
  }
  DEBUG_PRINTLN("");
  ServosDetached = 0;
  return;
}

void detach_all_servos() {
  for (int i = 0; i < 16; i++) {
    servoDriver.setPin(i,0,false); // stop pulses which will quickly detach the servo
  }
  ServosDetached = 1;
}

// Short power dips can cause the servo driver to put itself to sleep
// the checkForServoSleep() function uses IIC protocol to ask the servo
// driver if it's asleep. If it is, this function wakes it back up.
void checkForServoSleep() {

  if (millis() > freqWatchDog) {

    // See if the servo driver module went to sleep, probably due to a short power dip
    Wire.beginTransmission(SERVO_IIC_ADDR);
    Wire.write(0);  // address 0 is the MODE1 location of the servo driver, see documentation on the PCA9685 chip for more info
    byte txStatus = Wire.endTransmission();
    byte received = 0;
    int mode1 = -1;
    if (txStatus == 0) {
      received = Wire.requestFrom((uint8_t)SERVO_IIC_ADDR, (uint8_t)1);
      if (received == 1 && Wire.available()) mode1 = Wire.read();
    }

    // 只有成功读到 MODE1 才判断休眠。旧代码在 I2C 短暂失败时 Wire.read()=-1，
    // -1 的休眠位恒为 1，会反复误复位 PCA9685 并让全部舵机瞬间失去脉冲。
    if (mode1 >= 0 && (mode1 & 16)) {
      // wake it up!
      resetServoDriver();
      attach_all_servos();  // begin() 会清空输出，必须立即重发当前 12 路位置
      beep(1200,50);  // chirp to warn user of brown out on servo controller
      beep(800,50);
      SuppressScamperUntil = millis() + 10000;  // no scamper for you! (for 10 seconds)
    }
    freqWatchDog = millis() + 100;
  }
}

void GeneralCheckSmoothMoves() {
  static long lastSmoothTime = 0;

#define SMOOTHTIME 30 // milliseconds resolution

  long now = millis();
  if (now >= lastSmoothTime + SMOOTHTIME) {
    lastSmoothTime = now;
  } else {
    return; // not time yet
  }
  for (int servo = 0; servo < 12; servo++) {
    SmoothMove(servo);
  }
}

void SmoothMove(int servo) {
#define SMOOTHINC  2  // degrees per resolution time
  int tmp = deferServoSet;

  if (abs(ServoPos[servo] - ServoTarget[servo]) <= SMOOTHINC) {
    deferServoSet = 0;
    setServo(servo, ServoTarget[servo]);
    deferServoSet = tmp;
    return;
  }

  deferServoSet = 0;
  setServo(servo, ServoPos[servo] + (ServoPos[servo]>ServoTarget[servo]?(0-SMOOTHINC):SMOOTHINC));
  deferServoSet = tmp;
}

void save_trims() {
  DEBUG_PRINT("SAVE TRIMS:");
  for (int i = 0; i < NUM_LEGS*2; i++) {
    EEPROM.update(i, ServoTrim[i]);
    Serial.print(ServoTrim[i]); Serial.print(" ");
  }
  DEBUG_PRINTLN("");
  EEPROM.update(12, 'V');
}

void erase_trims() {
  DEBUG_PRINTLN("ERASE TRIMS");
  for (int i = 0; i < NUM_LEGS*2; i++) {
    ServoTrim[i] = TRIM_ZERO;
  }
}
