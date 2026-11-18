#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace max17048 {

// MAX17048 (ModelGauge, 1 celda). Va colgado de BAT y queda siempre alimentado; entra solo
// en hibernación (3 µA) cuando la corriente es chica, así que no hay nada que hacer al dormir.
// Nunca poner el bit SLEEP: dejaría de seguir la celda y el porcentaje se perdería.
class MAX17048 : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_alert_threshold(uint8_t pct) { this->alert_threshold_ = pct; }
  void set_battery_level_sensor(sensor::Sensor *s) { this->battery_level_sensor_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_sensor_ = s; }
  void set_charge_rate_sensor(sensor::Sensor *s) { this->charge_rate_sensor_ = s; }
  void set_time_to_empty_sensor(sensor::Sensor *s) { this->time_to_empty_sensor_ = s; }
  void set_time_to_full_sensor(sensor::Sensor *s) { this->time_to_full_sensor_ = s; }

  float get_battery_level() const { return this->battery_level_; }
  float get_battery_voltage() const { return this->battery_voltage_; }
  float get_charge_rate() const { return this->charge_rate_; }
  bool is_hibernating() const { return this->hibernating_; }

 protected:
  bool read_u16_(uint8_t reg, uint16_t *val);
  bool write_u16_(uint8_t reg, uint16_t val);

  uint8_t alert_threshold_{10};
  uint16_t version_{0};
  float battery_level_{NAN};
  float battery_voltage_{NAN};
  float charge_rate_{NAN};
  bool hibernating_{false};

  sensor::Sensor *battery_level_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *charge_rate_sensor_{nullptr};
  sensor::Sensor *time_to_empty_sensor_{nullptr};
  sensor::Sensor *time_to_full_sensor_{nullptr};
};

}  // namespace max17048
}  // namespace esphome
