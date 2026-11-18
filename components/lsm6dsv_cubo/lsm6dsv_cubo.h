#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/event/event.h"

#include <string>

namespace esphome {
namespace lsm6dsv_cubo {

// Caras en el sistema de ejes del LSM6DSV16X: cuál eje apunta hacia arriba.
enum Face : uint8_t {
  FACE_X_PLUS = 0,
  FACE_X_MINUS = 1,
  FACE_Y_PLUS = 2,
  FACE_Y_MINUS = 3,
  FACE_Z_PLUS = 4,
  FACE_Z_MINUS = 5,
  FACE_UNKNOWN = 255,
};

class GestureEvent : public event::Event {};

// Despierto: acelerómetro y giróscopo a 60 Hz en alto rendimiento, fusión SFLP a 60 Hz
// batcheando el vector gravedad en la FIFO (≈ 0,65 mA). Dormido (arm_wake): giróscopo y
// fusión apagados, acelerómetro en low-power 1 a 15 Hz con la función wake-up latcheada
// en INT1 (≈ 7 µA).
// Con el toque habilitado (tap_threshold > 0) el acelerómetro pasa a 480 Hz, que es el mínimo
// que pide ST para el detector de toque del chip (AN5763 §5.5); el consumo en alto rendimiento
// no depende de la frecuencia. La salida que lee el firmware se filtra con el LPF2 a 24 Hz para
// que perilla, deslizamiento y sacudida vean la misma banda que antes.
class LSM6DSVCubo : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- configuración ---
  void set_step_degrees(float v) { this->step_degrees_ = v; }
  void set_rotation_deadband(float v) { this->rotation_deadband_ = v; }
  void set_slide_threshold(float v) { this->slide_threshold_ = v; }
  void set_idle_timeout(uint32_t ms) { this->idle_timeout_ms_ = ms; }
  void set_idle_timeout_timer_wake(uint32_t ms) { this->idle_timeout_timer_wake_ms_ = ms; }
  void set_wake_threshold(uint16_t mg) { this->wake_threshold_mg_ = mg; }
  void set_wake_duration(uint8_t n) { this->wake_duration_ = n; }
  // mg de pendiente entre muestras a 480 Hz (pasos de 125 mg con ±4 g). 0 = sin toque.
  void set_tap_threshold(uint16_t mg) { this->tap_threshold_mg_ = mg; }
  // Orden fijo: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z. También sirve con el cubo andando.
  void set_face_name(uint8_t i, const std::string &name);

  void set_knob_angle_sensor(sensor::Sensor *s) { this->knob_angle_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *s) { this->motion_binary_sensor_ = s; }
  void set_face_text_sensor(text_sensor::TextSensor *s) { this->face_text_sensor_ = s; }
  void set_gesture_event(GestureEvent *e) { this->gesture_event_ = e; }

  // --- callbacks para los triggers ---
  void add_on_rotate_callback(std::function<void(int)> &&cb) { this->rotate_cb_.add(std::move(cb)); }
  void add_on_slide_callback(std::function<void(std::string)> &&cb) { this->slide_cb_.add(std::move(cb)); }
  void add_on_tap_callback(std::function<void()> &&cb) { this->tap_cb_.add(std::move(cb)); }
  void add_on_face_callback(std::function<void(int, std::string)> &&cb) { this->face_cb_.add(std::move(cb)); }
  void add_on_idle_callback(std::function<void()> &&cb) { this->idle_cb_.add(std::move(cb)); }
  void add_on_motion_callback(std::function<void()> &&cb) { this->motion_cb_.add(std::move(cb)); }

  // --- acciones ---
  // Apaga giróscopo y fusión, deja el acelerómetro en low-power a 15 Hz con la
  // interrupción wake-up (latcheada) en INT1, y la limpia. Llamar justo antes de deep_sleep.enter.
  void arm_wake();
  // Limpia el latch de INT1 (lectura de WAKE_UP_SRC).
  void clear_interrupt();
  void set_idle_timeout_runtime(uint32_t ms) { this->idle_timeout_ms_ = ms; }
  // Ignora los toques durante `ms`. Llamarlo cada vez que vibra el háptico (drv2605: on_drive):
  // el golpe del motor es un toque perfecto para el acelerómetro.
  void inhibit_tap(uint32_t ms);

  bool is_ready() const { return this->state_ == STATE_RUNNING; }
  bool is_moving() const { return this->moving_; }
  // true desde que venció idle_timeout sin movimiento hasta el próximo movimiento.
  bool is_idle() const { return this->idle_fired_; }
  uint8_t get_face() const { return this->face_; }
  std::string get_face_name() const {
    return this->face_ < 6 ? this->face_names_[this->face_] : std::string("?");
  }
  bool woke_from_sleep() const { return this->woke_from_sleep_; }

 protected:
  enum State : uint8_t {
    STATE_INIT,
    STATE_SETTLE,
    STATE_RUNNING,
    STATE_ARMED,
    STATE_FAILED,
  };
  enum SlideState : uint8_t { SLIDE_IDLE, SLIDE_PULSE, SLIDE_COOLDOWN };

  bool write_reg_(uint8_t reg, uint8_t val);
  bool read_reg_(uint8_t reg, uint8_t *val);
  bool write_emb_(uint8_t reg, uint8_t val);  // registro de la página de funciones embebidas
  bool init_full_();
  void drain_fifo_();
  void process_sample_(const float *grv, const float *lia, const float *gyr, float dt);
  void fire_gesture_(const char *type);
  bool link_ready_() const;
  void service_pending_(uint32_t now);
  void publish_face_();
  bool tap_enabled_() const { return this->tap_threshold_mg_ > 0; }
  void set_tap_axis_(int axis);  // habilita el detector solo en el eje vertical (-1 = ninguno)
  void check_tap_(uint32_t now);
  bool was_still_before_(uint32_t now) const;

  float step_degrees_{15.0f};
  float rotation_deadband_{6.0f};
  float slide_threshold_{3.0f};
  uint32_t idle_timeout_ms_{20000};
  uint32_t idle_timeout_timer_wake_ms_{3000};
  uint16_t wake_threshold_mg_{80};
  uint8_t wake_duration_{2};
  uint16_t tap_threshold_mg_{500};
  std::string face_names_[6] = {"x+", "x-", "y+", "y-", "z+", "z-"};

  sensor::Sensor *knob_angle_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};
  text_sensor::TextSensor *face_text_sensor_{nullptr};
  GestureEvent *gesture_event_{nullptr};

  // Gestos hechos mientras Home Assistant todavía no está conectado (ver fire_gesture_).
  static const uint8_t PENDING_MAX = 12;
  static const uint32_t PENDING_MAX_AGE_MS = 8000;
  struct PendingGesture {
    const char *type;  // siempre un literal o SLIDE_NAMES: vive todo el programa
    uint8_t count;     // pasos de perilla acumulados; 1 para el resto
    uint32_t at;       // millis() del último agregado
  };
  PendingGesture pending_[PENDING_MAX];
  uint8_t pending_n_{0};

  CallbackManager<void(int)> rotate_cb_;
  CallbackManager<void(std::string)> slide_cb_;
  CallbackManager<void()> tap_cb_;
  CallbackManager<void(int, std::string)> face_cb_;
  CallbackManager<void()> idle_cb_;
  CallbackManager<void()> motion_cb_;

  State state_{STATE_INIT};
  uint32_t state_ts_{0};
  uint8_t init_retries_{0};
  uint8_t read_errors_{0};
  bool woke_from_sleep_{false};

  // gravedad del SFLP (m/s²) y cuándo llegó la última
  float grv_[3] = {0.0f, 0.0f, 0.0f};
  uint32_t grv_ts_{0};
  bool grv_valid_{false};
  bool sflp_warned_{false};

  // movimiento / inactividad
  bool moving_{false};
  uint32_t last_motion_ms_{0};
  bool idle_fired_{false};

  // cara
  uint8_t face_{FACE_UNKNOWN};
  uint8_t face_candidate_{FACE_UNKNOWN};
  uint32_t face_candidate_since_{0};
  uint32_t face_changed_ms_{0};

  // perilla
  float knob_angle_{0.0f};
  float knob_accum_{0.0f};
  uint32_t last_rotate_ms_{0};

  // deslizamiento
  SlideState slide_state_{SLIDE_IDLE};
  uint8_t slide_axis_{0};
  int8_t slide_sign_{0};
  float slide_peak_{0.0f};
  uint32_t slide_start_ms_{0};
  uint32_t last_slide_ms_{0};

  // toque: rachas de quietud (para exigir reposo antes y después del golpe) y candidato en espera
  bool still_{false};
  uint32_t still_since_ms_{0};
  uint32_t still_len_ms_{0};    // cuánto duró la quietud anterior al movimiento en curso
  uint32_t burst_start_ms_{0};  // cuándo empezó el movimiento en curso
  bool tap_pending_{false};
  uint32_t tap_pending_ms_{0};
  uint32_t last_tap_ms_{0};
  uint32_t tap_inhibit_start_ms_{0};
  uint32_t tap_inhibit_len_ms_{0};

  uint32_t last_shake_ms_{0};
  uint32_t last_sample_ms_{0};
  uint32_t last_temp_ms_{0};
};

// ---------------- triggers ----------------
class RotateTrigger : public Trigger<int> {
 public:
  explicit RotateTrigger(LSM6DSVCubo *parent) {
    parent->add_on_rotate_callback([this](int steps) { this->trigger(steps); });
  }
};

class SlideTrigger : public Trigger<std::string> {
 public:
  explicit SlideTrigger(LSM6DSVCubo *parent) {
    parent->add_on_slide_callback([this](std::string dir) { this->trigger(dir); });
  }
};

class TapTrigger : public Trigger<> {
 public:
  explicit TapTrigger(LSM6DSVCubo *parent) {
    parent->add_on_tap_callback([this]() { this->trigger(); });
  }
};

class FaceTrigger : public Trigger<int, std::string> {
 public:
  explicit FaceTrigger(LSM6DSVCubo *parent) {
    parent->add_on_face_callback([this](int face, std::string name) { this->trigger(face, name); });
  }
};

class IdleTrigger : public Trigger<> {
 public:
  explicit IdleTrigger(LSM6DSVCubo *parent) {
    parent->add_on_idle_callback([this]() { this->trigger(); });
  }
};

class MotionTrigger : public Trigger<> {
 public:
  explicit MotionTrigger(LSM6DSVCubo *parent) {
    parent->add_on_motion_callback([this]() { this->trigger(); });
  }
};

// ---------------- acciones ----------------
template<typename... Ts> class ArmWakeAction : public Action<Ts...> {
 public:
  explicit ArmWakeAction(LSM6DSVCubo *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->arm_wake(); }

 protected:
  LSM6DSVCubo *parent_;
};

template<typename... Ts> class ClearInterruptAction : public Action<Ts...> {
 public:
  explicit ClearInterruptAction(LSM6DSVCubo *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->clear_interrupt(); }

 protected:
  LSM6DSVCubo *parent_;
};

template<typename... Ts> class SetIdleTimeoutAction : public Action<Ts...> {
 public:
  explicit SetIdleTimeoutAction(LSM6DSVCubo *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint32_t, timeout)
  void play(const Ts &...x) override { this->parent_->set_idle_timeout_runtime(this->timeout_.value(x...)); }

 protected:
  LSM6DSVCubo *parent_;
};

template<typename... Ts> class InhibitTapAction : public Action<Ts...> {
 public:
  explicit InhibitTapAction(LSM6DSVCubo *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint32_t, timeout)
  void play(const Ts &...x) override { this->parent_->inhibit_tap(this->timeout_.value(x...)); }

 protected:
  LSM6DSVCubo *parent_;
};

}  // namespace lsm6dsv_cubo
}  // namespace esphome
