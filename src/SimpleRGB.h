#pragma once

#include <cstdint>

#include <Arduino.h>

struct SimpleRGB {
  uint8_t num;
};

void pinMode(SimpleRGB pin, uint8_t mode);

void digitalWrite(SimpleRGB pin, uint8_t val);

#ifndef LED_BUILTIN
extern SimpleRGB LED_BUILTIN;
#endif