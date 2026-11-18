#include "bq25896.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bq25896 {

static const char *const TAG = "bq25896";

// Mapa de registros (SLUSC76C)
static const uint8_t REG00_INPUT = 0x00;      // EN_HIZ | EN_ILIM | IINLIM[5:0]
static const uint8_t REG02_ADC = 0x02;        // CONV_START | CONV_RATE | BOOST_FREQ | ICO_EN | ...
static const uint8_t REG03_CHG = 0x03;        // BAT_LOADEN | WD_RST | OTG_CONFIG | CHG_CONFIG | SYS_MIN[2:0] | MIN_VBAT_SEL
static const uint8_t REG04_ICHG = 0x04;       // EN_PUMPX | ICHG[6:0]
static const uint8_t REG06_VREG = 0x06;       // VREG[5:0] | BATLOWV | VRECHG
static const uint8_t REG07_WD = 0x07;         // EN_TERM | STAT_DIS | WATCHDOG[1:0] | EN_TIMER | CHG_TIMER[1:0] | JEITA_ISET
static const uint8_t REG09_BATFET = 0x09;     // FORCE_ICO | TMR2X_EN | BATFET_DIS | JEITA_VSET | BATFET_DLY | BATFET_RST_EN | PUMPX_UP | PUMPX_DN
static const uint8_t REG0B_STATUS = 0x0B;     // VBUS_STAT[2:0] | CHRG_STAT[1:0] | PG_STAT | - | VSYS_STAT
static const uint8_t REG0C_FAULT = 0x0C;
static const uint8_t REG0E_BATV = 0x0E;       // THERM_STAT | BATV[6:0]  2304 mV + 20 mV/LSB
static const uint8_t REG0F_SYSV = 0x0F;       // SYSV[6:0]              2304 mV + 20 mV/LSB
static const uint8_t REG11_VBUSV = 0x11;      // VBUS_GD | VBUSV[6:0]   2600 mV + 100 mV/LSB
static const uint8_t REG12_ICHGR = 0x12;      // ICHGR[6:0]             50 mA/LSB
static const uint8_t REG14_INFO = 0x14;       // REG_RST | ICO_OPTIMIZED | PN[2:0] | TS_PROFILE | DEV_REV[1:0]

bool BQ25896::read_reg_(uint8_t reg, uint8_t *val) { return this->read_byte(reg, val); }

bool BQ25896::write_reg_(uint8_t reg, uint8_t val) {
  if (!this->write_byte(reg, val)) {
    ESP_LOGW(TAG, "write 0x%02X <- 0x%02X failed", reg, val);
    return false;
  }
  return true;
}

bool BQ25896::update_bits_(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t v = 0;
  if (!this->read_reg_(reg, &v))
    return false;
  v = (v & ~mask) | (value & mask);
  return this->write_reg_(reg, v);
}

void BQ25896::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BQ25896...");
  uint8_t info = 0;
  if (!this->read_reg_(REG14_INFO, &info)) {
    ESP_LOGE(TAG, "BQ25896 no responde");
    this->mark_failed();
    return;
  }
  const uint8_t pn = (info >> 3) & 0x07;
  ESP_LOGI(TAG, "REG14=0x%02X (PN=%u, rev=%u)%s", info, pn, info & 0x03,
           pn == 0 ? "" : " — atención: el PN no es el del BQ25896 (000)");

  if (this->disable_watchdog_) {
    // Sin watchdog I2C los ajustes sobreviven al deep sleep del ESP32.
    this->update_bits_(REG07_WD, 0x30, 0x00);
  }
  // Corriente y tensión de carga.
  this->update_bits_(REG04_ICHG, 0x7F, (uint8_t) (this->charge_current_ma_ / 64));
  this->update_bits_(REG06_VREG, 0xFC, (uint8_t) (((this->charge_voltage_mv_ - 3840) / 16) << 2));
  // Límite de entrada por registro (el pin ILIM recorta a 1,36 A de todas formas).
  this->set_input_current_limit(this->input_limit_ma_);
  // Carga habilitada (CE también está a GND por R12).
  this->update_bits_(REG03_CHG, 0x10, 0x10);
  // ADC: una conversión por ciclo de update, no continuo.
  this->update_bits_(REG02_ADC, 0x40, 0x00);
  this->start_adc_();
  this->read_status_();
}

void BQ25896::set_input_current_limit(uint16_t ma) {
  if (ma < 100)
    ma = 100;
  if (ma > 3250)
    ma = 3250;
  const uint8_t code = (uint8_t) ((ma - 100) / 50);
  // EN_HIZ = 0, EN_ILIM = 1 (el pin ILIM sigue activo como techo).
  this->write_reg_(REG00_INPUT, 0x40 | (code & 0x3F));
  ESP_LOGI(TAG, "IINLIM = %u mA", 100 + code * 50);
}

void BQ25896::ship_mode() {
  ESP_LOGW(TAG, "Ship mode: BATFET_DIS con retardo. La batería se desconecta en ~10 s.");
  // BATFET_DLY = 1 (bit 3) y BATFET_DIS = 1 (bit 5).
  this->update_bits_(REG09_BATFET, 0x28, 0x28);
}

void BQ25896::prepare_sleep() {
  // CONV_RATE = 0 y sin conversión pendiente.
  this->update_bits_(REG02_ADC, 0xC0, 0x00);
  this->adc_pending_ = false;
}

void BQ25896::start_adc_() {
  // CONV_START = 1 (se limpia solo al terminar, ~1 s).
  if (this->update_bits_(REG02_ADC, 0x80, 0x80))
    this->adc_pending_ = true;
}

void BQ25896::read_status_() {
  uint8_t st = 0, flt = 0;
  if (!this->read_reg_(REG0B_STATUS, &st))
    return;
  this->read_reg_(REG0C_FAULT, &flt);

  const uint8_t vbus_stat = (st >> 5) & 0x07;
  const uint8_t chrg_stat = (st >> 3) & 0x03;
  this->power_good_ = (st & 0x04) != 0;
  this->charging_ = (chrg_stat == 1 || chrg_stat == 2);

  if (this->power_good_binary_sensor_ != nullptr)
    this->power_good_binary_sensor_->publish_state(this->power_good_);
  if (this->charging_binary_sensor_ != nullptr)
    this->charging_binary_sensor_->publish_state(this->charging_);

  if (st != this->last_status_) {
    this->last_status_ = st;
    static const char *const CHG[4] = {"not charging", "pre-charge", "fast charging", "charge done"};
    static const char *const VBUS[8] = {"no input", "USB host", "adapter", "?", "?", "?", "?", "OTG"};
    ESP_LOGI(TAG, "estado: VBUS=%s CHRG=%s PG=%d VSYS_STAT=%d", VBUS[vbus_stat], CHG[chrg_stat], this->power_good_,
             st & 0x01);
    if (this->charge_status_text_sensor_ != nullptr) {
      std::string s = this->power_good_ ? std::string(CHG[chrg_stat]) : std::string("on battery");
      this->charge_status_text_sensor_->publish_state(s);
    }
  }
  if (flt != this->last_fault_) {
    this->last_fault_ = flt;
    std::string f;
    if (flt == 0) {
      f = "ok";
    } else {
      if (flt & 0x80)
        f += "watchdog ";
      if (flt & 0x40)
        f += "boost ";
      const uint8_t chg = (flt >> 4) & 0x03;
      if (chg == 1)
        f += "input ";
      if (chg == 2)
        f += "thermal ";
      if (chg == 3)
        f += "timer ";
      if (flt & 0x08)
        f += "battery-OV ";
      const uint8_t ntc = flt & 0x07;
      if (ntc == 1)
        f += "TS-warm ";
      if (ntc == 2)
        f += "TS-cool ";
      if (ntc == 5)
        f += "TS-cold ";
      if (ntc == 6)
        f += "TS-hot ";
    }
    if (flt != 0)
      ESP_LOGW(TAG, "fault 0x%02X: %s", flt, f.c_str());
    if (this->fault_text_sensor_ != nullptr)
      this->fault_text_sensor_->publish_state(f);
  }
}

void BQ25896::read_adc_() {
  uint8_t r = 0;
  if (this->read_reg_(REG0E_BATV, &r)) {
    this->battery_voltage_ = (2304 + 20 * (r & 0x7F)) / 1000.0f;
    if (this->battery_voltage_sensor_ != nullptr)
      this->battery_voltage_sensor_->publish_state(this->battery_voltage_);
    if (this->battery_level_sensor_ != nullptr) {
      const float mv = this->battery_voltage_ * 1000.0f;
      float pct = (mv - this->battery_empty_mv_) * 100.0f / (this->battery_full_mv_ - this->battery_empty_mv_);
      if (pct < 0)
        pct = 0;
      if (pct > 100)
        pct = 100;
      this->battery_level_sensor_->publish_state(pct);
    }
  }
  if (this->system_voltage_sensor_ != nullptr && this->read_reg_(REG0F_SYSV, &r))
    this->system_voltage_sensor_->publish_state((2304 + 20 * (r & 0x7F)) / 1000.0f);
  if (this->vbus_voltage_sensor_ != nullptr && this->read_reg_(REG11_VBUSV, &r))
    this->vbus_voltage_sensor_->publish_state((r & 0x80) ? (2600 + 100 * (r & 0x7F)) / 1000.0f : 0.0f);
  if (this->charge_current_sensor_ != nullptr && this->read_reg_(REG12_ICHGR, &r))
    this->charge_current_sensor_->publish_state(50.0f * (r & 0x7F));
}

void BQ25896::update() {
  if (this->is_failed())
    return;
  this->read_status_();
  if (this->adc_pending_) {
    uint8_t adc = 0;
    if (this->read_reg_(REG02_ADC, &adc) && (adc & 0x80) == 0) {
      this->read_adc_();
      this->adc_pending_ = false;
    }
  }
  if (!this->adc_pending_)
    this->start_adc_();
}

void BQ25896::dump_config() {
  ESP_LOGCONFIG(TAG, "BQ25896:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  ICHG %u mA, VREG %u mV, IINLIM %u mA, watchdog %s", this->charge_current_ma_,
                this->charge_voltage_mv_, this->input_limit_ma_, this->disable_watchdog_ ? "off" : "on");
  ESP_LOGCONFIG(TAG, "  Batería: %u–%u mV = 0–100 %%", this->battery_empty_mv_, this->battery_full_mv_);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Fallo de comunicación con el BQ25896");
}

}  // namespace bq25896
}  // namespace esphome
