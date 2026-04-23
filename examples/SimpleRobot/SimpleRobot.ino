#include <M5PicomniRover.h>

FeetechBus feetech_bus;
Motor motor1(1, &feetech_bus);
Motor motor2(2, &feetech_bus);
Motor motor3(3, &feetech_bus);

void setup() {
  WirelessControl::begin();
  feetech_bus.begin();

  // モーター起動
  motor1.begin();
  motor2.begin();
  motor3.begin();
}

void loop() {
  // BLE受信
  Command cmd = WirelessControl::getCommand();
  float vx = cmd.vx;
  float vy = cmd.vy;
  float w = cmd.w;

  WheelVel wheel_vel = solveInverseKinematics(vx, vy, w);

  // モーター速度送信
  motor1.setRPS(wheel_vel.v1);
  motor2.setRPS(wheel_vel.v2);
  motor3.setRPS(wheel_vel.v3);

  ////////////////

  // モーター速度受信
  float v1 = motor1.getRPS();
  float v2 = motor2.getRPS();
  float v3 = motor3.getRPS();

  Pose pose = solveForwardKinematics(v1, v2, v3);

  // BLE送信
  Odometry odom;
  odom.x = pose.x;
  odom.y = pose.y;
  odom.yaw = pose.yaw;
  WirelessControl::setOdometry(odom);
}
