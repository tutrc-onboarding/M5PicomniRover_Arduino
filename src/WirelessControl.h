#pragma once

struct Command {
  float vx;
  float vy;
  float w;
};

struct Odometry {
  float x;
  float y;
  float yaw;
};

class WirelessControl {
public:
  static void begin();
  static Command getCommand();
  static void setOdometry(Odometry odom);
};
