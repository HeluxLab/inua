#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/i2c/i2c.h"

#include <functional>

namespace esphome {
namespace drv2605 {

enum MotorType : uint8_t {
  MOTOR_ERM = 0,      // masa excéntrica, lazo cerrado
  MOTOR_LRA = 1,      // resonador lineal, lazo abierto a lra_frequency
  MOTOR_REACTOR = 2,  // Alps Haptic Reactor: pulsos unipolares (modo ERM lazo abierto)
};

class DRV2605 : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_motor_type(uint8_t t) { this->motor_ = static_cast<MotorType>(t); }
  void set_library(uint8_t lib) { this->library_ = lib; }
  void set_lra_frequency(float hz) { this->lra_frequency_hz_ = hz; }
  void set_rated_voltage(float v) { this->rated_voltage_ = v; }
  void set_max_voltage(float v) { this->max_voltage_ = v; }

  // Reproduce un efecto (1–123) de la biblioteca ROM y vuelve a standby al terminar.
  void play(uint8_t effect);
  // N pulsos cuadrados por RTP. erm/reactor: unipolares 0→V (3 × 3,125 ms a 160 Hz es la
  // forma de onda nominal del AFT14A903A). lra: ráfaga de N ciclos. frequency_hz <= 0 usa
  // lra_frequency. amplitude 0–100 %. Ignora disparos a menos de 50 ms del anterior.
  void pulse(uint8_t pulses, float frequency_hz, uint8_t amplitude_pct);
  void stop();
  void standby();
  void wake();
  // Se llama justo antes de mover el motor (play o pulse). El cubo lo usa para que el
  // acelerómetro no tome la vibración como un toque.
  void add_on_drive_callback(std::function<void()> &&cb) { this->drive_cb_.add(std::move(cb)); }
  // Bloqueo: mientras la función devuelva true, `play` y `pulse` no mueven el motor. El cubo la
  // usa para no vibrar mientras suena el parlante (pico del motor sobre el riel de 3,3 V con el
  // Wi-Fi transmitiendo audio).
  void set_inhibit(std::function<bool()> &&f) { this->inhibit_ = std::move(f); }
  bool is_inhibited() const { return this->inhibit_ && this->inhibit_(); }

 protected:
  bool write_reg_(uint8_t reg, uint8_t val);
  bool read_reg_(uint8_t reg, uint8_t *val);
  void program_voltages_();

  MotorType motor_{MOTOR_ERM};
  uint8_t library_{1};
  float lra_frequency_hz_{0.0f};
  float rated_voltage_{0.0f};
  float max_voltage_{0.0f};
  bool in_standby_{true};
  uint32_t last_pulse_ms_{0};
  CallbackManager<void()> drive_cb_;
  std::function<bool()> inhibit_{};
};

class DriveTrigger : public Trigger<> {
 public:
  explicit DriveTrigger(DRV2605 *parent) {
    parent->add_on_drive_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts> class PlayAction : public Action<Ts...> {
 public:
  explicit PlayAction(DRV2605 *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint8_t, effect)
  void play(const Ts &...x) override { this->parent_->play(this->effect_.value(x...)); }

 protected:
  DRV2605 *parent_;
};

template<typename... Ts> class PulseAction : public Action<Ts...> {
 public:
  explicit PulseAction(DRV2605 *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint8_t, pulses)
  TEMPLATABLE_VALUE(float, frequency)
  TEMPLATABLE_VALUE(uint8_t, amplitude)
  void play(const Ts &...x) override {
    this->parent_->pulse(this->pulses_.value_or(x..., 3), this->frequency_.value_or(x..., 0.0f),
                         this->amplitude_.value_or(x..., 100));
  }

 protected:
  DRV2605 *parent_;
};

template<typename... Ts> class StopAction : public Action<Ts...> {
 public:
  explicit StopAction(DRV2605 *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->stop(); }

 protected:
  DRV2605 *parent_;
};

template<typename... Ts> class StandbyAction : public Action<Ts...> {
 public:
  explicit StandbyAction(DRV2605 *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->standby(); }

 protected:
  DRV2605 *parent_;
};

}  // namespace drv2605
}  // namespace esphome
