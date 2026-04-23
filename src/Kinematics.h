#pragma once

#include <Arduino.h>

#define ROBOT_RADIUS 0.083f
#define WHEEL_RADIUS 0.024f
#define WHEEL_THETA_1 (PI / 2.0f)
#define WHEEL_THETA_2 (PI * 7.0f / 6.0f)
#define WHEEL_THETA_3 (PI * 11.0f / 6.0f)

struct WheelVel {
    float v1;
    float v2;
    float v3;
};

struct Pose {
    float x;
    float y;
    float yaw;
};

inline WheelVel solveInverseKinematics(float vx, float vy, float w) {
  // 逆運動学の式
  float v1 = -vx * sin(WHEEL_THETA_1) + vy * cos(WHEEL_THETA_1) + ROBOT_RADIUS * w;
  float v2 = -vx * sin(WHEEL_THETA_2) + vy * cos(WHEEL_THETA_2) + ROBOT_RADIUS * w;
  float v3 = -vx * sin(WHEEL_THETA_3) + vy * cos(WHEEL_THETA_3) + ROBOT_RADIUS * w;

  // m/s -> rps
  v1 /= 2.0f * PI * WHEEL_RADIUS;
  v2 /= 2.0f * PI * WHEEL_RADIUS;
  v3 /= 2.0f * PI * WHEEL_RADIUS;

  return {v1, v2, v3};
}

inline Pose solveForwardKinematics(float v1, float v2, float v3) {
  // 制御周期計測 [us]
  static unsigned long prev_us = micros();
  unsigned long now_us = micros();
  unsigned long delta_us = now_us - prev_us;
  float delta_s = delta_us / 1000000.0f;
  prev_us = now_us;

  v1 *= 2.0f * PI * WHEEL_RADIUS;
  v2 *= 2.0f * PI * WHEEL_RADIUS;
  v3 *= 2.0f * PI * WHEEL_RADIUS;

  // 順運動学の式
  float vx = (-2.0f * v1 + v2 + v3) / 3.0f;
  float vy = (-v2 + v3) * sqrt(3.0f) / 3.0f;
  float w = (v1 + v2 + v3) / (3.0f * ROBOT_RADIUS);

  static float x = 0.0f;
  static float y = 0.0f;
  static float yaw = 0.0f;

  // ロボット座標系->ワールド座標系
  float vx_w = vx * cos(yaw) - vy * sin(yaw);
  float vy_w = vx * sin(yaw) + vy * cos(yaw);

  // ベクトルの積分
  x += vx_w * delta_s;
  y += vy_w * delta_s;
  yaw += w * delta_s;

  return {x, y, yaw};
}