#include "max17048.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace max17048 {

static const char *const TAG = "max17048";

// Registros (16 bits, MSB primero).
static const uint8_t REG_VCELL = 0x02;    // 78,125 µV/LSB
static const uint8_t REG_SOC = 0x04;      // 1/256 %/LSB
static const uint8_t REG_MODE = 0x06;     // bit 14 QuickStart, bit 13 EnSleep, bit 12 HibStat
static const uint8_t REG_VERSION = 0x08;  // 0x001x
static const uint8_t REG_CONFIG = 0x0C;   // RCOMP[15:8], SLEEP bit 7, ALSC bit 6, ALRT bit 5, ATHD[4:0]
static const uint8_t REG_CRATE = 0x16;    // con signo, 0,208 %/h por LSB
static const uint8_t REG_STATUS = 0x1A;   // byte alto: RI 0x01 VH 0x02 VL 0x04 VR 0x08 HD 0x10 SC 0x20

static const uint16_t MODE_HIBSTAT = 0x1000;
static const uint16_t CONFIG_SLEEP = 0x0080;
static const uint16_t CONFIG_ALSC = 0x0040;
static const uint16_t CONFIG_ALRT = 0x0020;
static const uint16_t CONFIG_ATHD_MASK = 0x001F;
static const uint16_t STATUS_RI = 0x0100;

bool MAX17048::read_u16_(uint8_t reg, uint16_t *val) { return this->read_byte_16(reg, val); }
bool MAX17048::write_u16_(uint8_t reg, uint16_t val) { return this->write_byte_16(reg, val); }

void MAX17048::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX17048...");
  if (!this->read_u16_(REG_VERSION, &this->version_)) {
    ESP_LOGE(TAG, "MAX17048 no responde en 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  if ((this->version_ & 0xFFF0) != 0x0010)
    ESP_LOGW(TAG, "VERSION inesperada 0x%04X (se esperaba 0x001x)", this->version_);

  // RI = el medidor arrancó de cero (batería recién conectada): el % sale de la OCV y se
  // va afinando solo. Se limpia para distinguir el próximo.
  uint16_t status = 0;
  if (this->read_u16_(REG_STATUS, &status) && (status & STATUS_RI)) {
    ESP_LOGW(TAG, "Reinicio del medidor: la batería estuvo desconectada, el %% arranca estimado");
    this->write_u16_(REG_STATUS, status & ~STATUS_RI);
  }

  // Umbral de ALRT (32 − ATHD), sin alerta por cambio de 1 %, sin SLEEP, RCOMP como está.
  uint16_t cfg = 0;
  if (this->read_u16_(REG_CONFIG, &cfg)) {
    cfg &= ~(CONFIG_SLEEP | CONFIG_ALSC | CONFIG_ALRT | CONFIG_ATHD_MASK);
    cfg |= (uint16_t) ((32 - this->alert_threshold_) & CONFIG_ATHD_MASK);
    this->write_u16_(REG_CONFIG, cfg);
  }
}

void MAX17048::update() {
  uint16_t raw = 0;
  bool ok = true;

  if (this->read_u16_(REG_VCELL, &raw)) {
    this->battery_voltage_ = raw * 78.125e-6f;
    if (this->battery_voltage_sensor_ != nullptr)
      this->battery_voltage_sensor_->publish_state(this->battery_voltage_);
  } else {
    ok = false;
  }

  if (this->read_u16_(REG_SOC, &raw)) {
    this->battery_level_ = std::min(100.0f, raw / 256.0f);
    if (this->battery_level_sensor_ != nullptr)
      this->battery_level_sensor_->publish_state(this->battery_level_);
  } else {
    ok = false;
  }

  if (this->read_u16_(REG_CRATE, &raw)) {
    this->charge_rate_ = (int16_t) raw * 0.208f;
    if (this->charge_rate_sensor_ != nullptr)
      this->charge_rate_sensor_->publish_state(this->charge_rate_);
    if (this->time_to_empty_sensor_ != nullptr) {
      float h = (this->charge_rate_ < -0.1f) ? this->battery_level_ / -this->charge_rate_ : NAN;
      this->time_to_empty_sensor_->publish_state(h);
    }
    if (this->time_to_full_sensor_ != nullptr) {
      float h = (this->charge_rate_ > 0.1f) ? (100.0f - this->battery_level_) / this->charge_rate_ : NAN;
      this->time_to_full_sensor_->publish_state(h);
    }
  } else {
    ok = false;
  }

  if (this->read_u16_(REG_MODE, &raw))
    this->hibernating_ = (raw & MODE_HIBSTAT) != 0;

  // Limpia el flag ALRT para que el pin vuelva a alto hasta la próxima condición.
  uint16_t cfg = 0;
  if (this->read_u16_(REG_CONFIG, &cfg) && (cfg & CONFIG_ALRT))
    this->write_u16_(REG_CONFIG, cfg & ~CONFIG_ALRT);

  if (ok) {
    this->status_clear_warning();
  } else {
    this->status_set_warning();
  }
}

void MAX17048::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX17048:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Versión 0x%04X, umbral ALRT %u %%, %s", this->version_, this->alert_threshold_,
                this->hibernating_ ? "hibernando" : "activo");
  LOG_SENSOR("  ", "Batería", this->battery_level_sensor_);
  LOG_SENSOR("  ", "Tensión", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "Tasa", this->charge_rate_sensor_);
  LOG_SENSOR("  ", "Horas restantes", this->time_to_empty_sensor_);
  LOG_SENSOR("  ", "Horas a carga completa", this->time_to_full_sensor_);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Fallo de comunicación con el MAX17048");
}

}  // namespace max17048
}  // namespace esphome
