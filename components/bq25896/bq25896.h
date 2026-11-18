#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace bq25896 {

class BQ25896 : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_charge_current(uint16_t ma) { this->charge_current_ma_ = ma; }
  void set_charge_voltage(uint16_t mv) { this->charge_voltage_mv_ = mv; }
  void set_input_current_limit_config(uint16_t ma) { this->input_limit_ma_ = ma; }
  void set_disable_watchdog(bool v) { this->disable_watchdog_ = v; }
  void set_battery_range(uint16_t empty_mv, uint16_t full_mv) {
    this->battery_empty_mv_ = empty_mv;
    this->battery_full_mv_ = full_mv;
  }

  void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_sensor_ = s; }
  void set_system_voltage_sensor(sensor::Sensor *s) { this->system_voltage_sensor_ = s; }
  void set_vbus_voltage_sensor(sensor::Sensor *s) { this->vbus_voltage_sensor_ = s; }
  void set_charge_current_sensor(sensor::Sensor *s) { this->charge_current_sensor_ = s; }
  void set_battery_level_sensor(sensor::Sensor *s) { this->battery_level_sensor_ = s; }
  void set_power_good_binary_sensor(binary_sensor::BinarySensor *s) { this->power_good_binary_sensor_ = s; }
  void set_charging_binary_sensor(binary_sensor::BinarySensor *s) { this->charging_binary_sensor_ = s; }
  void set_charge_status_text_sensor(text_sensor::TextSensor *s) { this->charge_status_text_sensor_ = s; }
  void set_fault_text_sensor(text_sensor::TextSensor *s) { this->fault_text_sensor_ = s; }

  // Límite de corriente de entrada en mA (100–3250, pasos de 50). El pin ILIM recorta igual.
  void set_input_current_limit(uint16_t ma);
  // BATFET_DIS con retardo: corta la batería del sistema (ship mode). Sale al conectar 5 V.
  void ship_mode();
  // Apaga el ADC continuo antes de dormir.
  void prepare_sleep();

  bool is_power_good() const { return this->power_good_; }
  bool is_charging() const { return this->charging_; }
  float get_battery_voltage() const { return this->battery_voltage_; }

 protected:
  bool read_reg_(uint8_t reg, uint8_t *val);
  bool write_reg_(uint8_t reg, uint8_t val);
  bool update_bits_(uint8_t reg, uint8_t mask, uint8_t value);
  void read_status_();
  void read_adc_();
  void start_adc_();

  uint16_t charge_current_ma_{1024};
  uint16_t charge_voltage_mv_{4208};
  uint16_t input_limit_ma_{3250};
  bool disable_watchdog_{true};
  uint16_t battery_empty_mv_{3300};
  uint16_t battery_full_mv_{4150};

  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *system_voltage_sensor_{nullptr};
  sensor::Sensor *vbus_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_sensor_{nullptr};
  sensor::Sensor *battery_level_sensor_{nullptr};
  binary_sensor::BinarySensor *power_good_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *charging_binary_sensor_{nullptr};
  text_sensor::TextSensor *charge_status_text_sensor_{nullptr};
  text_sensor::TextSensor *fault_text_sensor_{nullptr};

  bool adc_pending_{false};
  bool power_good_{false};
  bool charging_{false};
  float battery_voltage_{0.0f};
  uint8_t last_status_{0xFF};
  uint8_t last_fault_{0xFF};
};

template<typename... Ts> class SetInputCurrentLimitAction : public Action<Ts...> {
 public:
  explicit SetInputCurrentLimitAction(BQ25896 *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint16_t, current)
  void play(const Ts &...x) override { this->parent_->set_input_current_limit(this->current_.value(x...)); }

 protected:
  BQ25896 *parent_;
};

template<typename... Ts> class ShipModeAction : public Action<Ts...> {
 public:
  explicit ShipModeAction(BQ25896 *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->ship_mode(); }

 protected:
  BQ25896 *parent_;
};

template<typename... Ts> class PrepareSleepAction : public Action<Ts...> {
 public:
  explicit PrepareSleepAction(BQ25896 *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->prepare_sleep(); }

 protected:
  BQ25896 *parent_;
};

}  // namespace bq25896
}  // namespace esphome
