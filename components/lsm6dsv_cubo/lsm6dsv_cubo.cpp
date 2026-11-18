#include "lsm6dsv_cubo.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

#ifdef USE_ESP32
#include "esp_sleep.h"
#endif

namespace esphome {
namespace lsm6dsv_cubo {

static const char *const TAG = "lsm6dsv_cubo";

// ---- registros de usuario (AN5763 / hoja de datos) ----
static const uint8_t REG_FUNC_CFG_ACCESS = 0x01;  // bit 7: acceso a la página de funciones embebidas
static const uint8_t REG_FIFO_CTRL3 = 0x09;       // BDR_GY[7:4] BDR_XL[3:0]
static const uint8_t REG_FIFO_CTRL4 = 0x0A;       // FIFO_MODE[2:0]
static const uint8_t REG_WHO_AM_I = 0x0F;
static const uint8_t REG_CTRL1 = 0x10;  // OP_MODE_XL[6:4] ODR_XL[3:0]
static const uint8_t REG_CTRL2 = 0x11;  // OP_MODE_G[6:4]  ODR_G[3:0]
static const uint8_t REG_CTRL3 = 0x12;  // BOOT BDU - - - IF_INC - SW_RESET
static const uint8_t REG_CTRL6 = 0x15;  // FS_G[3:0]
static const uint8_t REG_CTRL8 = 0x17;  // FS_XL[1:0]
static const uint8_t REG_CTRL9 = 0x18;  // XL_FASTSETTL_MODE bit 5
static const uint8_t REG_FIFO_STATUS1 = 0x1B;     // DIFF_FIFO[7:0]; STATUS2 (0x1C) bit 0 = DIFF_FIFO[8]
static const uint8_t REG_OUT_TEMP_L = 0x20;       // 16 bits, 256 LSB/°C, 0 = 25 °C
static const uint8_t REG_OUTX_L_G = 0x22;         // 12 bytes: giróscopo XYZ, acelerómetro XYZ
static const uint8_t REG_WAKE_UP_SRC = 0x45;      // leerlo limpia el latch del wake-up
static const uint8_t REG_TAP_SRC = 0x46;          // SINGLE_TAP bit 5, TAP_SIGN bit 3, X/Y/Z_TAP bits 2/1/0; leerlo limpia el latch
static const uint8_t REG_FUNCTIONS_ENABLE = 0x50; // INTERRUPTS_ENABLE bit 7
static const uint8_t REG_INACTIVITY_DUR = 0x54;   // WU_INACT_THS_W[6:4]: peso del umbral
static const uint8_t REG_TAP_CFG0 = 0x56;         // HW_FUNC_MASK_XL_SETTL bit 5, SLOPE_FDS bit 4, TAP_X/Y/Z_EN bits 3/2/1, LIR bit 0
static const uint8_t REG_TAP_CFG1 = 0x57;         // TAP_PRIORITY[7:5] TAP_THS_X[4:0]
static const uint8_t REG_TAP_CFG2 = 0x58;         // TAP_THS_Y[4:0]
static const uint8_t REG_TAP_THS_6D = 0x59;       // TAP_THS_Z[4:0]
static const uint8_t REG_TAP_DUR = 0x5A;          // DUR[7:4] QUIET[3:2] SHOCK[1:0]
static const uint8_t REG_WAKE_UP_THS = 0x5B;      // WK_THS[5:0]
static const uint8_t REG_WAKE_UP_DUR = 0x5C;      // WAKE_DUR[6:5]
static const uint8_t REG_MD1_CFG = 0x5E;          // INT1_SINGLE_TAP bit 6, INT1_WU bit 5
static const uint8_t REG_FIFO_DATA_OUT_TAG = 0x78;  // tag + 6 bytes; la lectura en ráfaga da la vuelta sola
// ---- página de funciones embebidas ----
static const uint8_t EMB_FUNC_EN_A = 0x04;       // SFLP_GAME_EN bit 1
static const uint8_t EMB_FUNC_FIFO_EN_A = 0x44;  // SFLP_GRAVITY_FIFO_EN bit 4
static const uint8_t EMB_SFLP_ODR = 0x5E;        // 0 1 ODR[5:3] 0 1 1
static const uint8_t EMB_FUNC_INIT_A = 0x66;     // SFLP_GAME_INIT bit 1

static const uint8_t WHO_AM_I_VALUE = 0x70;
static const uint8_t CTRL3_SW_RESET = 0x01;
static const uint8_t CTRL3_IF_INC = 0x04;
static const uint8_t CTRL3_BDU = 0x40;
static const uint8_t ODR_15HZ = 0x03;
static const uint8_t ODR_60HZ = 0x05;
static const uint8_t ODR_480HZ = 0x08;
static const uint8_t OPMODE_HP = 0x00 << 4;
static const uint8_t OPMODE_LP1 = 0x04 << 4;
static const uint8_t FS_G_2000DPS = 0x04;  // 70 mdps/LSB
static const uint8_t FS_XL_4G = 0x01;      // 0,122 mg/LSB
static const uint8_t CTRL8_LPF2_ODR_DIV_20 = 0x02 << 5;  // HP_LPF2_XL_BW = 010: 24 Hz con ODR 480 Hz
static const uint8_t CTRL9_LPF2_XL_EN = 0x08;
static const uint8_t TAP_CFG0_LIR = 0x01;
static const uint8_t TAP_SRC_SINGLE = 0x20;
static const uint8_t TAP_SRC_SIGN = 0x08;
static const uint8_t TAP_DUR_SINGLE = 0x06;  // SHOCK = 2 (33 ms a 480 Hz), QUIET = 1: los del ejemplo de la AN5763 §5.5.4
static const uint8_t MD1_INT1_SINGLE_TAP = 0x40;
static const float TAP_LSB_MG = 125.0f;      // umbral de toque: 1 LSB = FS_XL / 32
static const uint8_t FIFO_MODE_BYPASS = 0x00;
static const uint8_t FIFO_MODE_CONTINUOUS = 0x06;
static const uint8_t SFLP_ODR_60HZ = 0x43 | (0x02 << 3);
static const uint8_t TAG_GRAVITY = 0x17;

static const float G_MS2 = 9.80665f;
static const float ACC_LSB_MS2 = 0.000122f * G_MS2;  // ±4 g
static const float GYR_LSB_DPS = 0.070f;             // ±2000 dps
static const float GRV_LSB_MS2 = G_MS2 / 16384.0f;   // vector gravedad SFLP: ±2 g en 16 bits

// ---- toque: el chip detecta el golpe, el firmware decide si fue un toque ----
static const uint32_t TAP_STILL_BEFORE_MS = 300;     // reposo exigido antes del golpe (apoyar el cubo no es un toque)
static const uint32_t TAP_BURST_MAX_MS = 150;        // lo más que el propio golpe puede llevar moviendo el cubo al leerlo
static const uint32_t TAP_SETTLE_MS = 60;            // reposo exigido después del golpe para confirmarlo
static const uint32_t TAP_CONFIRM_TIMEOUT_MS = 400;  // si no se aquietó en este tiempo, lo estaban levantando o empujando
static const uint32_t TAP_LOCKOUT_MS = 300;          // rebotes después de un toque

static const char *const SLIDE_NAMES[6] = {"slide_x_plus",  "slide_x_minus", "slide_y_plus",
                                           "slide_y_minus", "slide_z_plus",  "slide_z_minus"};

bool LSM6DSVCubo::write_reg_(uint8_t reg, uint8_t val) {
  if (!this->write_byte(reg, val)) {
    ESP_LOGW(TAG, "write 0x%02X <- 0x%02X failed", reg, val);
    return false;
  }
  return true;
}

bool LSM6DSVCubo::read_reg_(uint8_t reg, uint8_t *val) { return this->read_byte(reg, val); }

bool LSM6DSVCubo::write_emb_(uint8_t reg, uint8_t val) {
  bool ok = this->write_reg_(REG_FUNC_CFG_ACCESS, 0x80);
  ok &= this->write_reg_(reg, val);
  ok &= this->write_reg_(REG_FUNC_CFG_ACCESS, 0x00);
  return ok;
}

void LSM6DSVCubo::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LSM6DSV16X (cubo)...");
#ifdef USE_ESP32
  auto cause = esp_sleep_get_wakeup_cause();
  this->woke_from_sleep_ =
      (cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_EXT1 || cause == ESP_SLEEP_WAKEUP_TIMER);
  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    // Despertar programado: solo re-armar y volver a dormir rápido si nadie toca el cubo.
    this->idle_timeout_ms_ = this->idle_timeout_timer_wake_ms_;
  }
#endif
  this->last_motion_ms_ = millis();
  // El chip arranca en 10 ms tras alimentarse; tras deep sleep ya está vivo (quedó armado).
  this->state_ = STATE_INIT;
  this->state_ts_ = millis();
}

bool LSM6DSVCubo::init_full_() {
  // Reset por software: borra la configuración de wake-up que quedó armada y deja INT1 en bajo.
  if (!this->write_reg_(REG_CTRL3, CTRL3_SW_RESET))
    return false;
  delay(20);
  if (!this->write_reg_(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC))
    return false;
  if (!this->write_reg_(REG_CTRL6, FS_G_2000DPS))
    return false;
  // Con toque: LPF2 a ODR/20 en la salida que lee el firmware (24 Hz a 480 Hz). El detector de
  // toque usa la pendiente entre muestras, que en la cadena de filtros sale antes del LPF2.
  if (!this->write_reg_(REG_CTRL8, this->tap_enabled_() ? (uint8_t) (CTRL8_LPF2_ODR_DIV_20 | FS_XL_4G) : FS_XL_4G))
    return false;
  // Fusión SFLP: gravedad a 60 Hz batcheada en la FIFO (los datos SFLP solo salen por FIFO).
  this->write_emb_(EMB_FUNC_EN_A, 0x02);
  this->write_emb_(EMB_FUNC_FIFO_EN_A, 0x10);
  this->write_emb_(EMB_SFLP_ODR, SFLP_ODR_60HZ);
  this->write_emb_(EMB_FUNC_INIT_A, 0x02);
  // FIFO continua, sin batchear los sensores crudos (se leen de los registros de salida).
  this->write_reg_(REG_FIFO_CTRL3, 0x00);
  this->write_reg_(REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
  this->write_reg_(REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
  if (this->tap_enabled_()) {
    // Detector de toque del chip (AN5763 §5.5): un solo toque, el mismo umbral en los tres ejes,
    // latcheado. El latch solo actúa si la señal sale por un pin, por eso va a INT1: despierto
    // nadie mira ese pin y arm_wake() lo deja otra vez solo con el wake-up. El eje se habilita
    // cuando se conoce la cara de arriba (set_tap_axis_).
    uint8_t ths = (uint8_t) std::lround(this->tap_threshold_mg_ / TAP_LSB_MG);
    if (ths < 1)
      ths = 1;
    if (ths > 31)
      ths = 31;
    this->write_reg_(REG_CTRL9, CTRL9_LPF2_XL_EN);
    this->write_reg_(REG_TAP_CFG0, TAP_CFG0_LIR);
    this->write_reg_(REG_TAP_CFG1, ths);
    this->write_reg_(REG_TAP_CFG2, ths);
    this->write_reg_(REG_TAP_THS_6D, ths);
    this->write_reg_(REG_TAP_DUR, TAP_DUR_SINGLE);
    this->write_reg_(REG_FUNCTIONS_ENABLE, 0x80);  // INTERRUPTS_ENABLE
    this->write_reg_(REG_MD1_CFG, MD1_INT1_SINGLE_TAP);
  }
  this->tap_pending_ = false;
  // Giróscopo a 60 Hz, alto rendimiento (la SFLP exige ODR >= al suyo). Acelerómetro a 60 Hz, o a
  // 480 Hz con toque: es el mínimo que recomienda ST y en alto rendimiento consume lo mismo.
  if (!this->write_reg_(REG_CTRL1, OPMODE_HP | (this->tap_enabled_() ? ODR_480HZ : ODR_60HZ)))
    return false;
  if (!this->write_reg_(REG_CTRL2, OPMODE_HP | ODR_60HZ))
    return false;
  this->grv_valid_ = false;
  return true;
}

void LSM6DSVCubo::loop() {
  const uint32_t now = millis();
  this->service_pending_(now);
  switch (this->state_) {
    case STATE_INIT: {
      if (now - this->state_ts_ < 20)
        break;
      this->state_ts_ = now;
      uint8_t id = 0;
      if (!this->read_reg_(REG_WHO_AM_I, &id) || id != WHO_AM_I_VALUE) {
        if (++this->init_retries_ > 20) {
          ESP_LOGE(TAG, "LSM6DSV16X no responde (WHO_AM_I=0x%02X)", id);
          this->state_ = STATE_FAILED;
          this->mark_failed();
        }
        break;
      }
      if (!this->init_full_()) {
        if (++this->init_retries_ > 20) {
          this->state_ = STATE_FAILED;
          this->mark_failed();
        }
        break;
      }
      // El giróscopo tarda 70 ms en arrancar desde apagado; la SFLP necesita unas muestras.
      this->state_ = STATE_SETTLE;
      break;
    }

    case STATE_SETTLE:
      if (now - this->state_ts_ < 120)
        break;
      ESP_LOGI(TAG, "LSM6DSV16X listo (60 Hz + SFLP%s), %s", this->tap_enabled_() ? ", toque a 480 Hz" : "",
               this->woke_from_sleep_ ? "despertar" : "arranque");
      this->state_ = STATE_RUNNING;
      this->last_sample_ms_ = now;
      this->last_motion_ms_ = now;
      this->grv_ts_ = now;  // el aviso de "SFLP sin datos" cuenta desde acá
      this->idle_fired_ = false;
      break;

    case STATE_RUNNING:
      if (!this->idle_fired_ && now - this->last_motion_ms_ >= this->idle_timeout_ms_) {
        this->idle_fired_ = true;
        ESP_LOGD(TAG, "idle: %u ms sin movimiento", (unsigned) this->idle_timeout_ms_);
        this->idle_cb_.call();
      }
      break;

    default:
      break;
  }
}

// Vacía la FIFO y se queda con el último vector gravedad del SFLP.
void LSM6DSVCubo::drain_fifo_() {
  uint8_t st[2];
  if (!this->read_bytes(REG_FIFO_STATUS1, st, 2))
    return;
  uint16_t n = (uint16_t) st[0] | ((uint16_t) (st[1] & 0x01) << 8);
  uint8_t rounds = 0;
  while (n > 0 && rounds++ < 4) {
    const uint8_t k = n > 8 ? 8 : (uint8_t) n;
    uint8_t buf[8 * 7];
    if (!this->read_bytes(REG_FIFO_DATA_OUT_TAG, buf, k * 7))
      return;
    for (uint8_t i = 0; i < k; i++) {
      const uint8_t *w = buf + i * 7;
      if ((w[0] >> 3) != TAG_GRAVITY)
        continue;
      for (int a = 0; a < 3; a++) {
        const int16_t raw = (int16_t) ((uint16_t) w[1 + 2 * a] | ((uint16_t) w[2 + 2 * a] << 8));
        this->grv_[a] = raw * GRV_LSB_MS2;
      }
      this->grv_valid_ = true;
      this->grv_ts_ = millis();
    }
    n -= k;
  }
}

void LSM6DSVCubo::update() {
  if (this->state_ != STATE_RUNNING)
    return;
  const uint32_t now = millis();

  this->drain_fifo_();

  uint8_t buf[12];
  if (!this->read_bytes(REG_OUTX_L_G, buf, sizeof(buf))) {
    if (++this->read_errors_ == 10)
      ESP_LOGW(TAG, "lecturas I2C fallando");
    return;
  }
  this->read_errors_ = 0;

  auto s16 = [&](int i) { return (int16_t) ((uint16_t) buf[i] | ((uint16_t) buf[i + 1] << 8)); };
  float gyr[3], acc[3], lia[3];
  for (int a = 0; a < 3; a++) {
    gyr[a] = s16(2 * a) * GYR_LSB_DPS;        // dps
    acc[a] = s16(6 + 2 * a) * ACC_LSB_MS2;    // m/s², incluye la gravedad
  }

  // Sin gravedad del SFLP (arranque o fusión caída): pasa-bajos del acelerómetro como respaldo.
  if (!this->grv_valid_ || now - this->grv_ts_ > 500) {
    if (!this->grv_valid_) {
      for (int a = 0; a < 3; a++)
        this->grv_[a] = acc[a];
      this->grv_valid_ = true;
    } else {
      for (int a = 0; a < 3; a++)
        this->grv_[a] += 0.1f * (acc[a] - this->grv_[a]);
      if (!this->sflp_warned_ && now - this->grv_ts_ > 2000) {
        this->sflp_warned_ = true;
        ESP_LOGW(TAG, "la SFLP no entrega gravedad; usando pasa-bajos del acelerómetro");
      }
    }
  }
  for (int a = 0; a < 3; a++)
    lia[a] = acc[a] - this->grv_[a];

  float dt = (now - this->last_sample_ms_) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f)
    dt = 0.02f;
  this->last_sample_ms_ = now;
  this->process_sample_(this->grv_, lia, gyr, dt);
  this->check_tap_(now);

  if (this->temperature_sensor_ != nullptr && now - this->last_temp_ms_ >= 10000) {
    this->last_temp_ms_ = now;
    uint8_t t[2];
    if (this->read_bytes(REG_OUT_TEMP_L, t, 2)) {
      const int16_t raw = (int16_t) ((uint16_t) t[0] | ((uint16_t) t[1] << 8));
      this->temperature_sensor_->publish_state(25.0f + raw / 256.0f);
    }
  }
}

// ¿Hay un Home Assistant conectado y escuchando? Sin API en la configuración se considera que sí.
bool LSM6DSVCubo::link_ready_() const {
#ifdef USE_API
  return api::global_api_server != nullptr && api::global_api_server->is_connected_with_state_subscription();
#else
  return true;
#endif
}

// Un gesto sale enseguida si Home Assistant está conectado. Si no (el cubo se acaba de despertar
// y todavía no enganchó), se guarda y sale apenas conecte, para que el primer giro no se pierda.
// Los pasos de perilla seguidos se compactan: tres a la derecha y uno a la izquierda quedan en dos
// a la derecha. Lo que queda más de PENDING_MAX_AGE_MS sin poder salir se descarta: una luz que
// cambia sola medio minuto después confunde más de lo que ayuda.
void LSM6DSVCubo::fire_gesture_(const char *type) {
  ESP_LOGD(TAG, "gesto: %s", type);
  if (this->gesture_event_ == nullptr)
    return;
  const uint32_t now = millis();
  if (this->pending_n_ == 0 && this->link_ready_()) {
    this->gesture_event_->trigger(type);
    return;
  }
  const bool cw = std::strcmp(type, "rotate_cw") == 0;
  const bool ccw = std::strcmp(type, "rotate_ccw") == 0;
  if ((cw || ccw) && this->pending_n_ > 0) {
    PendingGesture &last = this->pending_[this->pending_n_ - 1];
    const bool last_cw = std::strcmp(last.type, "rotate_cw") == 0;
    const bool last_ccw = std::strcmp(last.type, "rotate_ccw") == 0;
    if (last_cw || last_ccw) {
      const int net = (last_cw ? last.count : -last.count) + (cw ? 1 : -1);
      if (net == 0) {
        this->pending_n_--;
      } else {
        last.type = net > 0 ? "rotate_cw" : "rotate_ccw";
        last.count = (uint8_t) (net > 0 ? net : -net);
        last.at = now;
      }
      return;
    }
  }
  if (this->pending_n_ == PENDING_MAX) {  // lleno: se cae el más viejo
    for (uint8_t i = 1; i < PENDING_MAX; i++)
      this->pending_[i - 1] = this->pending_[i];
    this->pending_n_--;
  }
  this->pending_[this->pending_n_++] = {type, 1, now};
}

// Desde loop(): manda lo guardado cuando hay conexión y tira lo que envejeció.
void LSM6DSVCubo::service_pending_(uint32_t now) {
  if (this->pending_n_ == 0)
    return;
  if (this->link_ready_()) {
    uint8_t sent = 0;
    for (uint8_t i = 0; i < this->pending_n_; i++) {
      const PendingGesture &p = this->pending_[i];
      if (now - p.at > PENDING_MAX_AGE_MS)
        continue;
      for (uint8_t k = 0; k < p.count; k++) {
        this->gesture_event_->trigger(p.type);
        sent++;
      }
    }
    ESP_LOGD(TAG, "conectado: %u gestos guardados enviados", sent);
    this->pending_n_ = 0;
    return;
  }
  // Sin conexión: descartar desde el frente los que ya son viejos.
  uint8_t drop = 0;
  while (drop < this->pending_n_ && now - this->pending_[drop].at > PENDING_MAX_AGE_MS)
    drop++;
  if (drop > 0) {
    for (uint8_t i = drop; i < this->pending_n_; i++)
      this->pending_[i - drop] = this->pending_[i];
    this->pending_n_ -= drop;
    ESP_LOGD(TAG, "sin conexión: %u gestos guardados descartados por viejos", drop);
  }
}

void LSM6DSVCubo::publish_face_() {
  if (this->face_text_sensor_ != nullptr)
    this->face_text_sensor_->publish_state(this->get_face_name());
}

// Cambia el nombre de una cara. Se puede llamar con el cubo andando (entidades "Cara 1" a
// "Cara 6"): si es la cara que está arriba, el sensor "Cara arriba" se actualiza en el acto.
void LSM6DSVCubo::set_face_name(uint8_t i, const std::string &name) {
  if (i >= 6 || name.empty() || this->face_names_[i] == name)
    return;
  this->face_names_[i] = name;
  if (i == this->face_)
    this->publish_face_();
}

// Motor de gestos: idéntico al del BNO055 (mismas unidades: m/s², m/s², dps).
void LSM6DSVCubo::process_sample_(const float *grv, const float *lia, const float *gyr, float dt) {
  const uint32_t now = millis();

  // ---- 1. movimiento / inactividad ----
  const float lin = std::sqrt(lia[0] * lia[0] + lia[1] * lia[1] + lia[2] * lia[2]);
  const float rot = std::sqrt(gyr[0] * gyr[0] + gyr[1] * gyr[1] + gyr[2] * gyr[2]);
  const bool moving_now = lin > 0.6f || rot > 3.0f;
  if (moving_now) {
    this->last_motion_ms_ = now;
    this->idle_fired_ = false;
    if (!this->moving_) {
      this->moving_ = true;
      if (this->motion_binary_sensor_ != nullptr)
        this->motion_binary_sensor_->publish_state(true);
      this->motion_cb_.call();
    }
  } else if (this->moving_ && now - this->last_motion_ms_ > 1000) {
    this->moving_ = false;
    if (this->motion_binary_sensor_ != nullptr)
      this->motion_binary_sensor_->publish_state(false);
  }
  // Rachas de quietud para el toque. Un hueco más corto que TAP_SETTLE_MS entre dos sacudones es
  // el mismo movimiento: el rebote de un golpe no borra el reposo que había antes.
  if (moving_now) {
    if (this->still_) {
      this->still_ = false;
      const uint32_t len = now - this->still_since_ms_;
      if (len >= TAP_SETTLE_MS) {
        this->still_len_ms_ = len;
        this->burst_start_ms_ = now;
      }
    }
  } else if (!this->still_) {
    this->still_ = true;
    this->still_since_ms_ = now;
  }

  // ---- 2. cara arriba: el eje con mayor componente de gravedad ----
  int axis = 0;
  for (int a = 1; a < 3; a++)
    if (std::fabs(grv[a]) > std::fabs(grv[axis]))
      axis = a;
  uint8_t cand = FACE_UNKNOWN;
  if (std::fabs(grv[axis]) > 7.5f)  // > ~0.76 g: la cara está claramente apoyada
    cand = (uint8_t) (axis * 2 + (grv[axis] > 0 ? 0 : 1));
  if (cand != this->face_candidate_) {
    this->face_candidate_ = cand;
    this->face_candidate_since_ = now;
  } else if (cand != FACE_UNKNOWN && cand != this->face_ && now - this->face_candidate_since_ >= 300) {
    this->face_ = cand;
    this->face_changed_ms_ = now;
    this->knob_accum_ = 0.0f;
    this->slide_state_ = SLIDE_IDLE;
    this->tap_pending_ = false;
    this->set_tap_axis_(cand / 2);
    ESP_LOGD(TAG, "cara: %s", this->get_face_name().c_str());
    this->publish_face_();
    this->face_cb_.call(this->face_, this->get_face_name());
    this->fire_gesture_("face_changed");
  }

  if (this->face_ == FACE_UNKNOWN)
    return;
  const bool face_settled = now - this->face_changed_ms_ > 500;
  const int up_axis = this->face_ / 2;
  const float up_sign = (this->face_ % 2 == 0) ? 1.0f : -1.0f;

  // ---- 3. perilla: giro alrededor del eje vertical ----
  // w_cw > 0 = horario visto desde arriba (regla de la mano derecha invertida).
  const float w_cw = -gyr[up_axis] * up_sign;
  if (face_settled && now - this->last_slide_ms_ > 300 && std::fabs(w_cw) > this->rotation_deadband_ &&
      std::fabs(w_cw) > 0.7f * rot) {
    this->tap_pending_ = false;  // lo están girando: el golpe fue la mano al agarrarlo
    this->knob_angle_ += w_cw * dt;
    this->knob_accum_ += w_cw * dt;
    this->last_rotate_ms_ = now;
    while (this->knob_accum_ >= this->step_degrees_) {
      this->knob_accum_ -= this->step_degrees_;
      this->rotate_cb_.call(1);
      this->fire_gesture_("rotate_cw");
      if (this->knob_angle_sensor_ != nullptr)
        this->knob_angle_sensor_->publish_state(this->knob_angle_);
    }
    while (this->knob_accum_ <= -this->step_degrees_) {
      this->knob_accum_ += this->step_degrees_;
      this->rotate_cb_.call(-1);
      this->fire_gesture_("rotate_ccw");
      if (this->knob_angle_sensor_ != nullptr)
        this->knob_angle_sensor_->publish_state(this->knob_angle_);
    }
  }

  // ---- 4. deslizamiento: pulso de aceleración lineal en un eje horizontal, seguido
  //         del pulso de frenado de signo contrario, sin giro apreciable ----
  if (face_settled && now - this->last_rotate_ms_ > 300 && rot < 40.0f) {
    if (this->slide_state_ == SLIDE_IDLE) {
      for (int a = 0; a < 3; a++) {
        if (a == up_axis)
          continue;
        if (std::fabs(lia[a]) > this->slide_threshold_) {
          this->slide_state_ = SLIDE_PULSE;
          this->tap_pending_ = false;  // empuje horizontal: es un deslizamiento, no un toque
          this->slide_axis_ = (uint8_t) a;
          this->slide_sign_ = lia[a] > 0 ? 1 : -1;
          this->slide_peak_ = std::fabs(lia[a]);
          this->slide_start_ms_ = now;
          break;
        }
      }
    } else if (this->slide_state_ == SLIDE_PULSE) {
      const float a = lia[this->slide_axis_];
      const int8_t sgn = a > 0 ? 1 : -1;
      if (sgn == this->slide_sign_ && std::fabs(a) > this->slide_peak_)
        this->slide_peak_ = std::fabs(a);
      if (sgn == -this->slide_sign_ && std::fabs(a) > 0.5f * this->slide_threshold_) {
        const uint8_t idx = (uint8_t) (this->slide_axis_ * 2 + (this->slide_sign_ > 0 ? 0 : 1));
        this->last_slide_ms_ = now;
        this->slide_state_ = SLIDE_COOLDOWN;
        this->slide_cb_.call(std::string(SLIDE_NAMES[idx]));
        this->fire_gesture_(SLIDE_NAMES[idx]);
      } else if (now - this->slide_start_ms_ > 600) {
        this->slide_state_ = SLIDE_IDLE;  // empuje continuo o traslado: no es un gesto
      }
    }
  } else if (this->slide_state_ == SLIDE_PULSE) {
    this->slide_state_ = SLIDE_IDLE;
  }
  if (this->slide_state_ == SLIDE_COOLDOWN && now - this->last_slide_ms_ > 500)
    this->slide_state_ = SLIDE_IDLE;

  // ---- 5. sacudida ----
  if (lin > 12.0f && now - this->last_shake_ms_ > 1500) {
    this->last_shake_ms_ = now;
    this->tap_pending_ = false;
    this->fire_gesture_("shake");
  }
}

// ---- 6. toque: golpe suave en la cara de arriba, con el cubo apoyado y quieto ----
// El chip marca el golpe (pendiente entre muestras a 480 Hz sobre el umbral, que vuelve a bajar
// dentro de la ventana de choque) y lo deja latcheado en TAP_SRC. Acá se descarta todo lo que no
// sea un toque: el motor háptico, apoyar o levantar el cubo, y el arranque de un giro o de un
// deslizamiento. El evento sale cuando el cubo volvió a quedar quieto, unos 60 a 150 ms después.
void LSM6DSVCubo::inhibit_tap(uint32_t ms) {
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->tap_inhibit_start_ms_;
  const uint32_t left = elapsed < this->tap_inhibit_len_ms_ ? this->tap_inhibit_len_ms_ - elapsed : 0;
  this->tap_inhibit_start_ms_ = now;
  this->tap_inhibit_len_ms_ = ms > left ? ms : left;
  this->tap_pending_ = false;
}

void LSM6DSVCubo::set_tap_axis_(int axis) {
  if (!this->tap_enabled_())
    return;
  // TAP_X_EN bit 3, TAP_Y_EN bit 2, TAP_Z_EN bit 1: solo el eje que apunta hacia arriba.
  const uint8_t en = (axis >= 0 && axis < 3) ? (uint8_t) (0x08 >> axis) : 0;
  this->write_reg_(REG_TAP_CFG0, TAP_CFG0_LIR | en);
}

bool LSM6DSVCubo::was_still_before_(uint32_t now) const {
  if (this->still_ && now - this->still_since_ms_ >= TAP_STILL_BEFORE_MS)
    return true;  // el golpe ni llegó a verse en las muestras de 50 Hz
  // El golpe movió el cubo hace un instante: vale el reposo que había antes de ese movimiento.
  return now - this->burst_start_ms_ <= TAP_BURST_MAX_MS && this->still_len_ms_ >= TAP_STILL_BEFORE_MS;
}

void LSM6DSVCubo::check_tap_(uint32_t now) {
  if (!this->tap_enabled_())
    return;
  uint8_t src = 0;
  if (this->read_reg_(REG_TAP_SRC, &src) && (src & TAP_SRC_SINGLE) != 0) {
    const int tap_axis = (src & 0x04) ? 0 : ((src & 0x02) ? 1 : ((src & 0x01) ? 2 : -1));
    const char *why = nullptr;
    if (this->face_ == FACE_UNKNOWN || now - this->face_changed_ms_ <= 500) {
      why = "la cara no está asentada";
    } else if (tap_axis != this->face_ / 2) {
      why = "no fue en el eje vertical";
    } else if (now - this->tap_inhibit_start_ms_ < this->tap_inhibit_len_ms_) {
      why = "vibró el háptico";
    } else if (now - this->last_tap_ms_ < TAP_LOCKOUT_MS) {
      why = "rebote del toque anterior";
    } else if (this->slide_state_ != SLIDE_IDLE || now - this->last_slide_ms_ < 300 ||
               now - this->last_rotate_ms_ < 300 || now - this->last_shake_ms_ < 300) {
      why = "giro, deslizamiento o sacudida";
    } else if (!this->was_still_before_(now)) {
      why = "el cubo no estaba quieto";
    }
    if (why != nullptr) {
      ESP_LOGD(TAG, "toque descartado: %s (TAP_SRC=0x%02X)", why, src);
    } else {
      // TAP_SIGN: 1 = pendiente negativa en el eje. Referido a la cara de arriba, un golpe desde
      // arriba debería empujar hacia abajo y apoyar el cubo, hacia arriba. Solo se informa: sirve
      // para ver con la placa en la mano si alcanza como filtro.
      ESP_LOGD(TAG, "toque candidato (empuje hacia %s)",
               (((src & TAP_SRC_SIGN) != 0) == (this->face_ % 2 == 0)) ? "abajo" : "arriba");
      this->tap_pending_ = true;
      this->tap_pending_ms_ = now;
    }
  }
  if (!this->tap_pending_)
    return;
  if (this->face_candidate_ != this->face_ || now - this->tap_pending_ms_ > TAP_CONFIRM_TIMEOUT_MS) {
    ESP_LOGD(TAG, "toque descartado: el cubo siguió en movimiento");
    this->tap_pending_ = false;
  } else if (now - this->tap_pending_ms_ >= TAP_SETTLE_MS && this->still_ &&
             now - this->still_since_ms_ >= TAP_SETTLE_MS) {
    this->tap_pending_ = false;
    this->last_tap_ms_ = now;
    this->last_motion_ms_ = now;  // cuenta como actividad: no dormirse justo después de un toque
    this->idle_fired_ = false;
    this->tap_cb_.call();
    this->fire_gesture_("tap");
  }
}

void LSM6DSVCubo::arm_wake() {
  ESP_LOGI(TAG, "armando wake-up para deep sleep (umbral %u mg, %u muestras a 15 Hz)", this->wake_threshold_mg_,
           this->wake_duration_);
  // Fusión y FIFO apagadas, giróscopo apagado, acelerómetro apagado para reconfigurarlo.
  this->write_reg_(REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
  this->write_emb_(EMB_FUNC_FIFO_EN_A, 0x00);
  this->write_emb_(EMB_FUNC_EN_A, 0x00);
  this->write_reg_(REG_CTRL2, 0x00);
  this->write_reg_(REG_CTRL1, 0x00);

  // Umbral: peso del LSB según el valor pedido (7,8 / 15,6 / 31,25 mg), máximo 63 LSB.
  uint8_t w = 0;
  float lsb = 7.8125f;
  while (w < 2 && this->wake_threshold_mg_ > 63.0f * lsb) {
    w++;
    lsb *= 2.0f;
  }
  uint8_t ths = (uint8_t) std::lround(this->wake_threshold_mg_ / lsb);
  if (ths < 1)
    ths = 1;
  if (ths > 63)
    ths = 63;
  // Secuencia de la AN5763 §5.? (wake-up con filtro de pendiente y latch), con el
  // enmascarado del asentamiento del filtro para no disparar en la primera muestra.
  this->write_reg_(REG_CTRL8, FS_XL_4G);               // sin LPF2: dormido queda igual que sin toque
  this->write_reg_(REG_CTRL9, 0x20);                   // XL_FASTSETTL_MODE
  this->write_reg_(REG_INACTIVITY_DUR, (uint8_t) (w << 4));
  this->write_reg_(REG_TAP_CFG0, 0x21);                // HW_FUNC_MASK_XL_SETTL | LIR (pendiente); toque apagado
  this->write_reg_(REG_WAKE_UP_THS, ths);
  this->write_reg_(REG_WAKE_UP_DUR, (uint8_t) (((this->wake_duration_ - 1) & 0x03) << 5));
  this->write_reg_(REG_MD1_CFG, 0x20);                 // wake-up -> INT1
  this->write_reg_(REG_FUNCTIONS_ENABLE, 0x80);        // INTERRUPTS_ENABLE
  this->write_reg_(REG_CTRL1, OPMODE_LP1 | ODR_15HZ);  // acelerómetro solo, 15 Hz, ~7 µA
  delay(80);
  this->clear_interrupt();
  this->state_ = STATE_ARMED;
}

void LSM6DSVCubo::clear_interrupt() {
  uint8_t src = 0;
  this->read_reg_(REG_WAKE_UP_SRC, &src);
  this->read_reg_(REG_TAP_SRC, &src);  // un toque latcheado también sostiene INT1
}

void LSM6DSVCubo::dump_config() {
  ESP_LOGCONFIG(TAG, "LSM6DSV16X cubo:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Paso perilla: %.1f°, banda muerta %.1f dps", this->step_degrees_, this->rotation_deadband_);
  ESP_LOGCONFIG(TAG, "  Umbral deslizamiento: %.1f m/s²", this->slide_threshold_);
  ESP_LOGCONFIG(TAG, "  Idle: %u ms (timer wake: %u ms)", (unsigned) this->idle_timeout_ms_,
                (unsigned) this->idle_timeout_timer_wake_ms_);
  ESP_LOGCONFIG(TAG, "  Wake-up: %u mg x %u muestras", this->wake_threshold_mg_, this->wake_duration_);
  if (this->tap_enabled_()) {
    ESP_LOGCONFIG(TAG, "  Toque: umbral %u mg (acelerómetro a 480 Hz)", this->tap_threshold_mg_);
  } else {
    ESP_LOGCONFIG(TAG, "  Toque: deshabilitado");
  }
  if (this->is_failed())
    ESP_LOGE(TAG, "  Fallo de comunicación con el LSM6DSV16X");
}

}  // namespace lsm6dsv_cubo
}  // namespace esphome
