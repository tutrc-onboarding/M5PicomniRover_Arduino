#include "SimpleRGB.h"

void pinMode(SimpleRGB pin, uint8_t mode) {
  assert(mode == OUTPUT);
  pinMode(pin.num, mode);
}

void digitalWrite(SimpleRGB pin, uint8_t val) {
  uint8_t rgb_val = val != 0 ? RGB_BRIGHTNESS : 0;
  rgbLedWrite(pin.num, rgb_val, rgb_val, rgb_val);
}

#ifndef LED_BUILTIN
SimpleRGB LED_BUILTIN{27};
#endif
