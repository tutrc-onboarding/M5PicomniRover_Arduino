#pragma once

#include <cstddef>
#include <cstdint>

#include <Arduino.h>

class FeetechBus {
public:
  FeetechBus(HardwareSerial *serial = &Serial1);
  void begin(uint32_t baudrate = 1000000, uint8_t rx_pin = 22, uint8_t tx_pin = 21);
  size_t available();
  uint8_t read();
  void write(uint8_t data);

private:
  HardwareSerial *serial_;
  int16_t echo_back_ = -1;
};
