#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <vector>

namespace esphome {
namespace cubo_system {

class CuboSystem : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void add_hold_low_pin(uint8_t pin) { this->hold_low_pins_.push_back(pin); }
  void set_low_battery_pin(int8_t pin) { this->low_battery_pin_ = pin; }
  void set_usb_host_binary_sensor(binary_sensor::BinarySensor *s) { this->usb_host_binary_sensor_ = s; }
  void set_wake_cause_text_sensor(text_sensor::TextSensor *s) { this->wake_cause_text_sensor_ = s; }

  // Baja y retiene los pines configurados para que sigan en 0 durante el deep sleep.
  void prepare_sleep();
  bool is_usb_host_connected() const { return this->usb_host_; }
  const char *wake_cause() const { return this->wake_cause_; }

 protected:
  std::vector<uint8_t> hold_low_pins_;
  int8_t low_battery_pin_{-1};
  binary_sensor::BinarySensor *usb_host_binary_sensor_{nullptr};
  text_sensor::TextSensor *wake_cause_text_sensor_{nullptr};
  bool usb_host_{false};
  uint16_t last_sof_{0};
  uint8_t sof_still_{0};
  const char *wake_cause_{"?"};
};

template<typename... Ts> class PrepareSleepAction : public Action<Ts...> {
 public:
  explicit PrepareSleepAction(CuboSystem *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->prepare_sleep(); }

 protected:
  CuboSystem *parent_;
};

}  // namespace cubo_system
}  // namespace esphome
