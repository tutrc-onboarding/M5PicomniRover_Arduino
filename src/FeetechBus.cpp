#include "FeetechBus.h"

FeetechBus::FeetechBus(HardwareSerial *serial) : serial_{serial} {}

void FeetechBus::begin(uint32_t baudrate, uint8_t rx_pin, uint8_t tx_pin) {
  serial_->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
}

size_t FeetechBus::available() { return serial_->available(); }

uint8_t FeetechBus::read() {
  if (echo_back_ != -1) {
    uint8_t data = echo_back_;
    echo_back_ = -1;
    return data;
  }
  return serial_->read();
}

void FeetechBus::write(uint8_t data) {
  serial_->write(data);
  echo_back_ = data;
}
