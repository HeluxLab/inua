#include "drv2605.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cmath>

namespace esphome {
namespace drv2605 {

static const char *const TAG = "drv2605";

static const uint8_t REG_STATUS = 0x00;
static const uint8_t REG_MODE = 0x01;
static const uint8_t REG_RTP = 0x02;
static const uint8_t REG_LIBRARY = 0x03;
static const uint8_t REG_WAVESEQ1 = 0x04;
static const uint8_t REG_WAVESEQ2 = 0x05;
static const uint8_t REG_GO = 0x0C;
static const uint8_t REG_OVERDRIVE = 0x0D;
static const uint8_t REG_SUSTAIN_POS = 0x0E;
static const uint8_t REG_SUSTAIN_NEG = 0x0F;
static const uint8_t REG_BRAKE = 0x10;
static const uint8_t REG_AUDIO_MAX = 0x13;
static const uint8_t REG_RATED_VOLTAGE = 0x16;
static const uint8_t REG_OD_CLAMP = 0x17;
static const uint8_t REG_FEEDBACK = 0x1A;
static const uint8_t REG_CONTROL2 = 0x1B;
static const uint8_t REG_CONTROL3 = 0x1D;
static const uint8_t REG_OL_LRA_PERIOD = 0x20;

static const uint8_t MODE_INT_TRIG = 0x00;
static const uint8_t MODE_RTP = 0x05;
static const uint8_t MODE_STANDBY = 0x40;

static const uint8_t FEEDBACK_N_ERM_LRA = 0x80;
static const uint8_t CONTROL3_ERM_OPEN_LOOP = 0x20;
static const uint8_t CONTROL3_LRA_OPEN_LOOP = 0x01;

// Intervalo mínimo entre disparos de `pulse` (especificación Alps: 50 ms).
static const uint32_t PULSE_MIN_INTERVAL_MS = 50;

bool DRV2605::write_reg_(uint8_t reg, uint8_t val) {
  if (!this->write_byte(reg, val)) {
    ESP_LOGW(TAG, "write 0x%02X <- 0x%02X failed", reg, val);
    return false;
  }
  return true;
}

bool DRV2605::read_reg_(uint8_t reg, uint8_t *val) { return this->read_byte(reg, val); }

void DRV2605::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DRV2605...");
  uint8_t status = 0;
  if (!this->read_reg_(REG_STATUS, &status)) {
    ESP_LOGE(TAG, "DRV2605 no responde");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "STATUS=0x%02X (device id %u)", status, status >> 5);

  this->write_reg_(REG_MODE, MODE_INT_TRIG);  // sale de standby, trigger interno (registro GO)
  this->write_reg_(REG_RTP, 0x00);
  this->write_reg_(REG_WAVESEQ1, 0x01);
  this->write_reg_(REG_WAVESEQ2, 0x00);
  this->write_reg_(REG_OVERDRIVE, 0x00);
  this->write_reg_(REG_SUSTAIN_POS, 0x00);
  this->write_reg_(REG_SUSTAIN_NEG, 0x00);
  this->write_reg_(REG_BRAKE, 0x00);
  this->write_reg_(REG_AUDIO_MAX, 0x64);

  uint8_t fb = 0, c3 = 0;
  this->read_reg_(REG_FEEDBACK, &fb);
  this->read_reg_(REG_CONTROL3, &c3);
  if (this->motor_ == MOTOR_LRA) {
    if (this->lra_frequency_hz_ <= 0.0f)
      this->lra_frequency_hz_ = 205.0f;
    this->write_reg_(REG_FEEDBACK, fb | FEEDBACK_N_ERM_LRA);
    this->write_reg_(REG_CONTROL3, c3 | CONTROL3_LRA_OPEN_LOOP);  // sin autocalibración
    // Periodo de conmutación en lazo abierto: OL_LRA_PERIOD × 98,49 µs (ec. 1 de la hoja de datos).
    int period = (int) lroundf(1.0f / (this->lra_frequency_hz_ * 98.49e-6f));
    this->write_reg_(REG_OL_LRA_PERIOD, (uint8_t) std::max(1, std::min(127, period)));
    // DRIVE_TIME = (semiperiodo − 0,5 ms) / 0,1 ms, conservando STARTUP_BOOST/AC_COUPLE.
    uint8_t c2 = 0;
    this->read_reg_(REG_CONTROL2, &c2);
    int drive = (int) lroundf((0.5f / this->lra_frequency_hz_ - 0.5e-3f) / 0.1e-3f);
    this->write_reg_(REG_CONTROL2, (c2 & 0xE0) | (uint8_t) std::max(0, std::min(31, drive)));
  } else {
    if (this->lra_frequency_hz_ <= 0.0f)
      this->lra_frequency_hz_ = 160.0f;
    this->write_reg_(REG_FEEDBACK, fb & ~FEEDBACK_N_ERM_LRA);
    if (this->motor_ == MOTOR_REACTOR) {
      // Lazo abierto: la salida sigue la amplitud pedida sin buscar back-EMF (el reactor
      // tiene muy poca y de todos modos se excita con pulsos cortos).
      this->write_reg_(REG_CONTROL3, c3 | CONTROL3_ERM_OPEN_LOOP);
    }
  }
  this->program_voltages_();
  this->write_reg_(REG_LIBRARY, this->library_);
  this->standby();
}

void DRV2605::program_voltages_() {
  // Ecuaciones 4–9 de SLOS854D. LSB: ERM 21,18 mV (nominal) / 21,59 mV (tope);
  // LRA 20,58 mV × √(1 − (4·t_sample + 300 µs)·f) (nominal, t_sample = 300 µs) / 21,22 mV (tope).
  const bool lra = this->motor_ == MOTOR_LRA;
  if (this->rated_voltage_ > 0.0f) {
    float lsb = lra ? 20.58e-3f * sqrtf(1.0f - (4.0f * 300e-6f + 300e-6f) * this->lra_frequency_hz_) : 21.18e-3f;
    int v = (int) lroundf(this->rated_voltage_ / lsb);
    this->write_reg_(REG_RATED_VOLTAGE, (uint8_t) std::max(1, std::min(255, v)));
  }
  if (this->max_voltage_ > 0.0f) {
    float lsb = lra ? 21.22e-3f : 21.59e-3f;
    int v = (int) lroundf(this->max_voltage_ / lsb);
    this->write_reg_(REG_OD_CLAMP, (uint8_t) std::max(1, std::min(255, v)));
  }
}

void DRV2605::wake() {
  if (this->in_standby_) {
    this->write_reg_(REG_MODE, MODE_INT_TRIG);
    this->in_standby_ = false;
    delay(1);
  }
}

void DRV2605::standby() {
  this->write_reg_(REG_MODE, MODE_INT_TRIG | MODE_STANDBY);
  this->in_standby_ = true;
}

void DRV2605::play(uint8_t effect) {
  if (this->is_failed())
    return;
  if (this->is_inhibited()) {
    ESP_LOGD(TAG, "Efecto %u omitido: bloqueo activo", effect);
    return;
  }
  this->drive_cb_.call();
  this->wake();
  this->write_reg_(REG_WAVESEQ1, effect);
  this->write_reg_(REG_WAVESEQ2, 0x00);
  this->write_reg_(REG_GO, 0x01);
  // Los efectos duran como mucho ~1 s; después volvemos a standby (4 µA).
  this->set_timeout("drv_standby", 1500, [this]() { this->standby(); });
}

void DRV2605::pulse(uint8_t pulses, float frequency_hz, uint8_t amplitude_pct) {
  if (this->is_failed() || pulses == 0)
    return;
  if (this->is_inhibited()) {
    ESP_LOGD(TAG, "Pulso omitido: bloqueo activo");
    return;
  }
  const uint32_t now = millis();
  if (now - this->last_pulse_ms_ < PULSE_MIN_INTERVAL_MS)
    return;
  this->last_pulse_ms_ = now;
  this->drive_cb_.call();
  if (frequency_hz <= 0.0f)
    frequency_hz = this->lra_frequency_hz_;
  if (amplitude_pct > 100)
    amplitude_pct = 100;
  const uint8_t amp = (uint8_t) ((amplitude_pct * 127 + 50) / 100);  // RTP con signo: 0–127
  const uint32_t half_us = (uint32_t) (500000.0f / frequency_hz);

  this->cancel_timeout("drv_standby");
  this->wake();
  this->write_reg_(REG_MODE, MODE_RTP);
  const uint32_t t0 = micros();
  auto wait_until = [t0](uint32_t t_rel) {
    while ((int32_t) (micros() - (t0 + t_rel)) < 0)
      delayMicroseconds(5);
  };
  if (this->motor_ == MOTOR_LRA) {
    // El chip conmuta solo a lra_frequency: basta sostener la amplitud N ciclos.
    this->write_reg_(REG_RTP, amp);
    wait_until(2 * half_us * pulses);
    this->write_reg_(REG_RTP, 0);
  } else {
    // Pulsos unipolares 0→V, ancho = semiperiodo, separación = semiperiodo (hoja Alps).
    for (uint8_t i = 0; i < pulses; i++) {
      this->write_reg_(REG_RTP, amp);
      wait_until((2 * i + 1) * half_us);
      this->write_reg_(REG_RTP, 0);
      wait_until((2 * i + 2) * half_us);
    }
  }
  this->write_reg_(REG_MODE, MODE_INT_TRIG);
  this->set_timeout("drv_standby", 200, [this]() { this->standby(); });
}

void DRV2605::stop() {
  this->write_reg_(REG_GO, 0x00);
  this->write_reg_(REG_RTP, 0x00);
  this->cancel_timeout("drv_standby");
  this->standby();
}

void DRV2605::dump_config() {
  static const char *const NAMES[] = {"ERM", "LRA (lazo abierto)", "Reactor Alps (pulsos unipolares)"};
  ESP_LOGCONFIG(TAG, "DRV2605:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Motor: %s, biblioteca %u, f=%.0f Hz, nominal %.2f V, tope %.2f V", NAMES[this->motor_],
                this->library_, this->lra_frequency_hz_, this->rated_voltage_, this->max_voltage_);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Fallo de comunicación con el DRV2605");
}

}  // namespace drv2605
}  // namespace esphome
