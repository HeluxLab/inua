// Funciones de apoyo de autoprueba.yaml: anotan cada paso en el registro y juntan las fallas.
#pragma once

#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

#include <cstdarg>
#include <cstdio>
#include <string>

#include <esp_heap_caps.h>

namespace esphome {
namespace prueba {

static const char *const TAG = "prueba";

inline std::string &fallas() {
  static std::string s;
  return s;
}

inline int &total() {
  static int n = 0;
  return n;
}

inline void empezar() {
  fallas().clear();
  total() = 0;
  ESP_LOGI(TAG, "================ AUTOPRUEBA ================");
}

// Un paso que cuenta para el resultado.
inline void paso(const char *nombre, bool ok, const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  total()++;
  if (ok) {
    ESP_LOGI(TAG, "  [ OK  ] %-22s %s", nombre, buf);
  } else {
    ESP_LOGE(TAG, "  [FALLA] %-22s %s", nombre, buf);
    if (!fallas().empty())
      fallas() += ", ";
    fallas() += nombre;
  }
}

// Un dato que se muestra pero no decide el resultado.
inline void nota(const char *nombre, const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  ESP_LOGI(TAG, "  [ dato] %-22s %s", nombre, buf);
}

// Lee `len` bytes de un registro, sin pasar por el componente del chip: sirve aunque ese
// componente haya fallado al arrancar.
inline bool leer(i2c::I2CBus *bus, uint8_t direccion, uint8_t registro, uint8_t *dato, uint8_t len = 1) {
  i2c::I2CDevice dev;
  dev.set_i2c_bus(bus);
  dev.set_i2c_address(direccion);
  return dev.read_bytes(registro, dato, len);
}

}  // namespace prueba
}  // namespace esphome
