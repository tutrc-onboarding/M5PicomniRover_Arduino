#include <cmath>
#include <cstring>

#include <Arduino.h>

#include "FeetechServo.h"

constexpr unsigned long FEETECH_SERVO_TIMEOUT_US = 3000;

FeetechServo::FeetechServo(uint8_t id, FeetechBus *bus) : id_{id}, bus_{bus} {}

uint8_t FeetechServo::getError() { return error_; }

bool FeetechServo::ping() {
  uint8_t checksum = 0;

  flush();
  write(0xFF);
  write(0xFF);
  checksum += write(id_);
  checksum += write(2);
  checksum += write(0x01);
  write(~checksum);

  return checkReply(nullptr, 0);
}

bool FeetechServo::readData(uint8_t addr, uint8_t *data, size_t size) {
  uint8_t checksum = 0;

  flush();
  write(0xFF);
  write(0xFF);
  checksum += write(id_);
  checksum += write(4);
  checksum += write(0x02);
  checksum += write(addr);
  checksum += write(size);

  write(~checksum);

  return checkReply(data, size);
}

bool FeetechServo::writeData(uint8_t addr, const uint8_t *data, size_t size) {
  uint8_t checksum = 0;

  flush();
  write(0xFF);
  write(0xFF);
  checksum += write(id_);
  checksum += write(size + 3);
  checksum += write(0x03);
  checksum += write(addr);

  for (size_t i = 0; i < size; ++i) {
    checksum += write(data[i]);
  }

  write(~checksum);

  return checkReply(nullptr, 0);
}

bool FeetechServo::controlMode(uint8_t value) { return writeData(0x21, &value, sizeof(value)); }

bool FeetechServo::enableTorque(uint8_t value) { return writeData(0x28, &value, sizeof(value)); }

bool FeetechServo::setPosition(int16_t value) {
  value = constrain(value, -32766, 32766);
  if (value < 0) {
    value = std::abs(value) | 0x8000;
  }
  uint8_t buf[2];
  std::memcpy(buf, &value, sizeof(buf));
  return writeData(0x2A, buf, sizeof(buf));
}

bool FeetechServo::setVelocity(int16_t value) {
  value = constrain(value, -32766, 32766);
  if (value < 0) {
    value = std::abs(value) | 0x8000;
  }
  uint8_t buf[2];
  std::memcpy(buf, &value, sizeof(buf));
  return writeData(0x2E, buf, sizeof(buf));
}

bool FeetechServo::getPosition(int16_t *value) {
  uint8_t buf[2];
  if (!readData(0x38, buf, sizeof(buf))) {
    return false;
  }
  std::memcpy(value, buf, sizeof(*value));
  if ((*value & 0x8000) != 0) {
    *value = -(*value & 0x7FFF);
  }
  return true;
}

bool FeetechServo::getVelocity(int16_t *value) {
  uint8_t buf[2];
  if (!readData(0x3A, buf, sizeof(buf))) {
    return false;
  }
  std::memcpy(value, buf, sizeof(*value));
  if ((*value & 0x8000) != 0) {
    *value = -(*value & 0x7FFF);
  }
  return true;
}

uint8_t FeetechServo::write(uint8_t data) {
  bus_->write(data);
  return bus_->read();
}

void FeetechServo::flush() {
  while (bus_->available() > 0) {
    bus_->read();
  }
}

bool FeetechServo::checkReply(uint8_t *data, size_t size) {
  unsigned long start = micros();
  while (bus_->available() < size + 6) {
    if (static_cast<unsigned long>(micros() - start) >= FEETECH_SERVO_TIMEOUT_US) {
      return false;
    }
  }

  uint8_t checksum = id_ + size + 2;

  if (bus_->read() != 0xFF) {
    return false;
  }
  if (bus_->read() != 0xFF) {
    return false;
  }
  if (bus_->read() != id_) {
    return false;
  }
  if (bus_->read() != size + 2) {
    return false;
  }

  error_ = bus_->read();
  checksum += error_;

  for (size_t i = 0; i < size; ++i) {
    data[i] = bus_->read();
    checksum += data[i];
  }

  checksum = ~checksum;
  if (bus_->read() != checksum) {
    return false;
  }

  return true;
}
