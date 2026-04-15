#include <M5StampRover.h>

#define ROBOT_RADIUS 0.083f
#define WHEEL_RADIUS 0.024f
#define WHEEL_THETA_1 (PI / 2.0f)
#define WHEEL_THETA_2 (PI * 7.0f / 6.0f)
#define WHEEL_THETA_3 (PI * 11.0f / 6.0f)

#define STEPS_TO_RPS(x) ((x) / 4096.0f)
#define RPS_TO_STEPS(x) ((x) * 4096.0f)

FeetechBus feetech_bus;
FeetechServo motor1(1, &feetech_bus);
FeetechServo motor2(2, &feetech_bus);
FeetechServo motor3(3, &feetech_bus);

void inverseKinematics() {
  // BLE受信
  Command cmd = WirelessControl::getCommand();
  float vx = cmd.vx;
  float vy = cmd.vy;
  float w = cmd.w;

  // 逆運動学の式
  float v1 = -vx * sin(WHEEL_THETA_1) + vy * cos(WHEEL_THETA_1) + ROBOT_RADIUS * w;
  float v2 = -vx * sin(WHEEL_THETA_2) + vy * cos(WHEEL_THETA_2) + ROBOT_RADIUS * w;
  float v3 = -vx * sin(WHEEL_THETA_3) + vy * cos(WHEEL_THETA_3) + ROBOT_RADIUS * w;

  // m/s -> rps
  v1 /= 2.0f * PI * WHEEL_RADIUS;
  v2 /= 2.0f * PI * WHEEL_RADIUS;
  v3 /= 2.0f * PI * WHEEL_RADIUS;

  // モーター速度送信
  motor1.setVelocity(RPS_TO_STEPS(v1));
  motor2.setVelocity(RPS_TO_STEPS(v2));
  motor3.setVelocity(RPS_TO_STEPS(v3));
}

void forwardKinematics() {
  // 制御周期計測 [us]
  static unsigned long prev_us = micros();
  unsigned long now_us = micros();
  unsigned long delta_us = now_us - prev_us;
  float delta_s = delta_us / 1000000.0f;
  prev_us = now_us;

  static int16_t v1_steps = 0;
  static int16_t v2_steps = 0;
  static int16_t v3_steps = 0;

  // モーター速度受信
  motor1.getVelocity(&v1_steps);
  motor2.getVelocity(&v2_steps);
  motor3.getVelocity(&v3_steps);

  // rps -> m/s
  float v1 = STEPS_TO_RPS(v1_steps) * 2.0f * PI * WHEEL_RADIUS;
  float v2 = STEPS_TO_RPS(v2_steps) * 2.0f * PI * WHEEL_RADIUS;
  float v3 = STEPS_TO_RPS(v3_steps) * 2.0f * PI * WHEEL_RADIUS;

  // 順運動学の式
  float vx = (-2.0f * v1 + v2 + v3) / 3.0f;
  float vy = (-v2 + v3) * sqrt(3.0f) / 3.0f;
  float w = (v1 + v2 + v3) / (3.0f * ROBOT_RADIUS);

  static float yaw = 0.0f;
  static float x = 0.0f;
  static float y = 0.0f;

  // ロボット座標系->ワールド座標系
  float vx_w = vx * cos(yaw) - vy * sin(yaw);
  float vy_w = vx * sin(yaw) + vy * cos(yaw);

  // ベクトルの積分
  x += vx_w * delta_s;
  y += vy_w * delta_s;
  yaw += w * delta_s;

  // BLE送信
  Odometry odom;
  odom.x = x;
  odom.y = y;
  odom.yaw = yaw;
  WirelessControl::setOdometry(odom);
}

void setup() {
  WirelessControl::begin();
  feetech_bus.begin();

  // モーターの起動を待つ
  while (!motor1.ping())
    ;
  while (!motor2.ping())
    ;
  while (!motor3.ping())
    ;

  // モーターを速度制御モードにする
  motor1.controlMode(1);
  motor2.controlMode(1);
  motor3.controlMode(1);
}

void loop() {
  inverseKinematics();
  forwardKinematics();
}
