////////////////////////////////////////////////////////////////////////////////
//           Mech_Hexapod Control Program - Sensors Implementation
//
// Copyright (C) 2025, 2026, 2027, 2028 Guang Dian Cong Lin Robotics, LLC.
////////////////////////////////////////////////////////////////////////////////

#include "sensors.h"

// ==========================================
// 传感器引脚初始化（只在 setup() 中调用一次）
// ==========================================
void initSensors() {
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  // 初始化时将 TRIG 拉低
  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_R, LOW);
}

// ==========================================
// 超声波距离读取（非阻塞优化版）
// 超时从 18000us 降至 8000us：
//   - 8000us ≈ 1.36 米探测范围，对避障绰绰有余
//   - 大幅减少无遮挡时的阻塞时间（从 18ms → 8ms）
// ==========================================
unsigned int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 8000us 超时 ≈ 1.36m 探测范围，宁可读不到也不长时间阻塞舵机
  unsigned long duration = pulseIn(echoPin, HIGH, 8000);

  if (duration == 0) return 1000; // 超时 → 前方空旷
  return duration / 58;           // 声速公式转换为厘米
}

int flash(unsigned long t) {
  // the following code will return HIGH for t milliseconds
  // followed by LOW for t milliseconds. Useful for flashing
  // the LED on pin 13 without blocking
  return (millis()%(2*t)) > t;
}
