#include "WirelessControl.h"

#include <cstring>

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <NimBLEDevice.h>

static constexpr unsigned long CMD_TIMEOUT_MS = 1000;

static constexpr char DEVICE_NAME[] = "Picomni";
static constexpr char SERVICE_UUID[] = "69321c59-8017-488e-b5e2-b6d30c834bc5";
static constexpr char CHARACTERISTIC_UUID[] =
    "87bc2dc5-2207-408d-99f6-3d35573c4472";

static QueueHandle_t cmd_queue;
static QueueHandle_t odom_queue;
static unsigned long cmd_last_update_ms;

static uint8_t odom_buf[sizeof(Odometry)];

class WirelessControlCharacteristicCallbacks
    : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic, BLEConnInfo &) override {
    NimBLEAttValue value = characteristic->getValue();
    if (value.size() != 12 && value.size() != 13) {
      return;
    }

    Command cmd{};
    std::memcpy(&cmd.vx, value.data(), 4);
    std::memcpy(&cmd.vy, value.data() + 4, 4);
    std::memcpy(&cmd.w, value.data() + 8, 4);
    if (value.size() == 13) {
      uint8_t buttons = value[12];
      cmd.a = (buttons & 0x01) != 0;
      cmd.b = (buttons & 0x02) != 0;
    }

    xQueueOverwrite(cmd_queue, &cmd);
    cmd_last_update_ms = millis();

    Odometry odom;
    if (xQueuePeek(odom_queue, &odom, 0) == pdTRUE) {
      std::memcpy(odom_buf, &odom, sizeof(odom_buf));
    }

    characteristic->setValue(odom_buf, sizeof(odom_buf));
    characteristic->notify();
  }
};

void WirelessControl::begin() {
  cmd_queue = xQueueCreate(1, sizeof(Command));
  odom_queue = xQueueCreate(1, sizeof(Odometry));

  BLEDevice::init(DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->advertiseOnDisconnect(true);

  BLEService *service = server->createService(SERVICE_UUID);
  BLECharacteristic *characteristic = service->createCharacteristic(
      CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
  characteristic->setCallbacks(new WirelessControlCharacteristicCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->enableScanResponse(true);
  BLEDevice::startAdvertising();
}

Command WirelessControl::getCommand() {
  if (static_cast<unsigned long>(millis() - cmd_last_update_ms) >=
      CMD_TIMEOUT_MS) {
    return {};
  }
  Command cmd;
  if (xQueuePeek(cmd_queue, &cmd, 0) != pdTRUE) {
    return {};
  }
  return cmd;
}

void WirelessControl::setOdometry(Odometry odom) {
  xQueueOverwrite(odom_queue, &odom);
}
