#pragma once

#include "FeetechServo.h"

#define STEPS_TO_RPS(x) ((x) / 4096.0f)
#define RPS_TO_STEPS(x) ((x) * 4096.0f)

class Motor {
public:
  Motor(uint8_t id, FeetechBus *bus) : servo_(id, bus) {}

  void begin() {
    while (!servo_.ping())
      ;
    servo_.controlMode(1);
  }

  void setRPS(float rps) { servo_.setVelocity(RPS_TO_STEPS(rps)); }

  float getRPS() {
    static int16_t steps = 0;
    servo_.getVelocity(&steps);
    return STEPS_TO_RPS(steps);
  }

private:
  FeetechServo servo_;
};