#include "cubo_system.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#endif
#ifdef USE_ESP32_VARIANT_ESP32S3
#include "soc/usb_serial_jtag_struct.h"
#endif

namespace esphome {
namespace cubo_system {

static const char *const TAG = "cubo_system";

void CuboSystem::setup() {
#ifdef USE_ESP32
  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT0:
      this->wake_cause_ = "motion";
      break;
    case ESP_SLEEP_WAKEUP_EXT1: {
      // ext1 comparte el INT del cargador (dock) y el ALRT del medidor (batería baja).
      const uint64_t st = esp_sleep_get_ext1_wakeup_status();
      if (this->low_battery_pin_ >= 0 && (st & (1ULL << this->low_battery_pin_)))
        this->wake_cause_ = "low battery";
      else
        this->wake_cause_ = "dock";
      break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
      this->wake_cause_ = "timer";
      break;
    default:
      this->wake_cause_ = "power on";
      break;
  }
#endif
  ESP_LOGI(TAG, "arranque por: %s", this->wake_cause_);
  if (this->wake_cause_text_sensor_ != nullptr)
    this->wake_cause_text_sensor_->publish_state(this->wake_cause_);
}

void CuboSystem::update() {
#ifdef USE_ESP32_VARIANT_ESP32S3
  // Un host USB manda un SOF cada 1 ms; si el contador de tramas cambia, hay PC conectada.
  // Esto es independiente del driver y funciona aunque el logger no use el USB.
  const uint16_t sof = USB_SERIAL_JTAG.fram_num.sof_frame_index;
  bool connected;
  if (sof != this->last_sof_) {
    this->sof_still_ = 0;
    connected = true;
  } else {
    if (this->sof_still_ < 5)
      this->sof_still_++;
    connected = this->sof_still_ < 3;  // 3 s sin tramas = desconectado
  }
  this->last_sof_ = sof;
  if (connected != this->usb_host_) {
    this->usb_host_ = connected;
    ESP_LOGI(TAG, "host USB %s", connected ? "conectado" : "desconectado");
  }
  if (this->usb_host_binary_sensor_ != nullptr)
    this->usb_host_binary_sensor_->publish_state(this->usb_host_);
#endif
}

void CuboSystem::prepare_sleep() {
#ifdef USE_ESP32
  for (uint8_t p : this->hold_low_pins_) {
    auto pin = (gpio_num_t) p;
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    gpio_hold_en(pin);
  }
  if (!this->hold_low_pins_.empty())
    gpio_deep_sleep_hold_en();
  if (this->low_battery_pin_ >= 0) {
    // ALRT es open-drain sin pull-up externo: pull-up del dominio RTC, que solo sigue activo
    // en deep sleep si ese dominio queda alimentado (cuesta ~1 µA).
    auto pin = (gpio_num_t) this->low_battery_pin_;
    rtc_gpio_init(pin);
    rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(pin);
    rtc_gpio_pullup_en(pin);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  }
  ESP_LOGI(TAG, "pines retenidos en bajo para deep sleep: %u", (unsigned) this->hold_low_pins_.size());
#endif
}

void CuboSystem::dump_config() {
  ESP_LOGCONFIG(TAG, "Cubo system:");
  ESP_LOGCONFIG(TAG, "  Causa del arranque: %s", this->wake_cause_);
  for (uint8_t p : this->hold_low_pins_)
    ESP_LOGCONFIG(TAG, "  Pin retenido en bajo al dormir: GPIO%u", p);
  if (this->low_battery_pin_ >= 0)
    ESP_LOGCONFIG(TAG, "  Alerta de batería baja (despertador ext1): GPIO%d", this->low_battery_pin_);
}

}  // namespace cubo_system
}  // namespace esphome
