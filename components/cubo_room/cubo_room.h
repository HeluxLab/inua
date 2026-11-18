#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace esphome {
namespace cubo_room {

// Una baliza fija: la configuración (a qué habitación pertenece y cómo se la reconoce) y lo
// último que se supo de ella.
struct Beacon {
  uint8_t room{0};          // índice en rooms_
  bool by_mac{true};        // true: por dirección; false: por datos iBeacon
  uint64_t address{0};
  uint8_t uuid[16]{};
  int32_t major{-1};        // -1 = cualquiera
  int32_t minor{-1};
  int8_t offset{0};         // dB que se suman a la lectura
  bool learned{false};      // true: aprendida con el cubo andando y guardada en su memoria
  float rssi{-127.0f};      // lectura filtrada, con el offset aplicado
  uint32_t last_seen{0};    // reloj de escaneo, ms
  uint16_t hits{0};         // anuncios seguidos desde que apareció; 0 = nunca oída o perdida
};

// La decisión, separada del resto para poder razonarla (y simularla) sola. Trabaja con el
// "reloj de escaneo": milisegundos durante los que el escáner estuvo realmente escuchando.
// Así una pausa del escáner no hace vencer ninguna baliza.
class RoomLogic {
 public:
  // Valores elegidos con `tools/sim_room.py` (balizas cada 100 ms, escucha del 25 %, ruido de
  // 4 dB): en el límite entre dos cuartos con la misma señal, 0,2 cambios por hora; caminando de
  // un cuarto a otro en 8 s, el cambio llega medio segundo antes de terminar.
  static constexpr float ALPHA = 0.25f;        // peso de cada lectura nueva en el filtro
  static constexpr uint32_t DWELL_MS = 1500;   // la otra habitación tiene que ganar este tiempo seguido

  float hysteresis{6.0f};
  float min_rssi{-85.0f};
  uint32_t timeout_ms{30000};
  uint32_t settle_ms{3000};

  std::vector<Beacon> beacons;
  int room{-1};      // índice de la habitación actual, -1 = desconocida
  int current{-1};   // baliza más fuerte de la habitación actual, -1 = ninguna viva

  // Lectura de una baliza configurada.
  void hit(size_t index, int rssi, uint32_t now) {
    Beacon &b = this->beacons[index];
    const float r = (float) rssi + (float) b.offset;
    if (b.hits == 0 || now - b.last_seen > this->timeout_ms) {
      b.rssi = r;
      b.hits = 1;
    } else {
      b.rssi += ALPHA * (r - b.rssi);
      if (b.hits < 0xFFFF)
        b.hits++;
    }
    b.last_seen = now;
  }

  // Devuelve true si cambió la habitación.
  bool evaluate(uint32_t now) {
    if (now < this->settle_ms)
      return false;  // al arrancar se juntan lecturas de todas antes de decidir
    int best = -1, cur = -1;
    for (size_t i = 0; i < this->beacons.size(); i++) {
      Beacon &b = this->beacons[i];
      if (b.hits == 0)
        continue;
      if (now - b.last_seen > this->timeout_ms) {
        b.hits = 0;  // perdida
        continue;
      }
      if (b.rssi >= this->min_rssi && (best < 0 || b.rssi > this->beacons[best].rssi))
        best = (int) i;
      if (this->room >= 0 && b.room == this->room && (cur < 0 || b.rssi > this->beacons[cur].rssi))
        cur = (int) i;
    }
    this->current = cur;
    if (best < 0 && cur < 0) {
      // No se oye nada conocido. La habitación se mantiene un `timeout` más y después se olvida.
      this->challenger_ = -1;
      if (this->room >= 0 && now - this->last_alive_ > this->timeout_ms) {
        this->room = -1;
        return true;
      }
      return false;
    }
    this->last_alive_ = now;
    if (best < 0) {
      this->challenger_ = -1;
      return false;  // solo se oye, floja, la de la habitación actual: se queda
    }
    const Beacon &b = this->beacons[best];
    if (b.room == this->room) {
      this->challenger_ = -1;
      return false;
    }
    if (cur >= 0) {
      // La habitación actual tiene una baliza viva: la otra tiene que superarla por la histéresis,
      // con al menos dos lecturas (una sola puede ser un rebote) y durante DWELL_MS seguidos.
      if (!(b.hits >= 2 && b.rssi >= this->beacons[cur].rssi + this->hysteresis)) {
        this->challenger_ = -1;
        return false;
      }
      if (this->challenger_ != b.room) {
        this->challenger_ = b.room;
        this->challenger_since_ = now;
      }
      if (now - this->challenger_since_ < DWELL_MS)
        return false;
    }
    // Sin baliza viva en la habitación actual (o sin habitación), gana la más fuerte.
    this->room = b.room;
    this->current = best;
    this->challenger_ = -1;
    return true;
  }

  // Después de agregar o quitar balizas los índices cambian: se olvida a quién se venía siguiendo.
  // La habitación actual se mantiene; si se quedó sin balizas, vence sola con `timeout`.
  void forget_tracking() {
    this->current = -1;
    this->challenger_ = -1;
  }

 protected:
  uint32_t last_alive_{0};
  int challenger_{-1};            // habitación que viene ganando, -1 = ninguna
  uint32_t challenger_since_{0};
};

class CuboRoom : public Component, public esp32_ble_tracker::ESPBTDeviceListener {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  bool parse_devices(const esp32_ble::BLEScanResult *scan_results, size_t count) override;
#ifdef USE_ESP32_BLE_DEVICE
  // Solo existe si otro componente pide el analizador completo; acá alcanza con el anuncio crudo.
  bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override { return false; }
#endif
  esp32_ble_tracker::AdvertisementParserType get_advertisement_parser_type() override {
    return esp32_ble_tracker::AdvertisementParserType::RAW_ADVERTISEMENTS;
  }

  void set_hysteresis(float db) { this->logic_.hysteresis = db; }
  void set_min_rssi(int dbm) { this->logic_.min_rssi = (float) dbm; }
  void set_timeout(uint32_t ms) { this->logic_.timeout_ms = ms; }
  void set_settle_time(uint32_t ms) { this->logic_.settle_ms = ms; }
  void set_unknown_room(const std::string &name) { this->unknown_room_ = name; }
  void set_discovery(bool on) { this->discovery_ = on; }
  void add_mac_beacon(const std::string &room, uint64_t address, int offset);
  void add_ibeacon(const std::string &room, uint64_t uuid_hi, uint64_t uuid_lo, int major, int minor, int offset);
  void set_room_text_sensor(text_sensor::TextSensor *s) { this->room_text_sensor_ = s; }
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_sensor_ = s; }
  void add_on_room_callback(std::function<void(std::string)> &&cb) { this->room_cb_.add(std::move(cb)); }

  // Para lambdas: nombre de la habitación actual, o el texto de `unknown_room`.
  const std::string &room() const;
  bool has_room() const { return this->logic_.room >= 0; }

  // --- Balizas aprendidas con el cubo andando (no hace falta recompilar) ---
  // Se apoya el cubo al lado de la baliza y se llama a `learn("Living")`: durante unos segundos
  // escucha, se queda con el aparato que llega más fuerte y lo guarda para esa habitación.
  static const size_t MAX_LEARNED = 12;
  void learn(const std::string &room);
  void forget(const std::string &room);  // olvida las aprendidas de esa habitación
  void clear_learned();                  // olvida todas las aprendidas
  bool is_learning() const { return this->learning_; }
  void set_learn_min_rssi(int dbm) { this->learn_min_rssi_ = (int8_t) dbm; }
  void set_beacons_text_sensor(text_sensor::TextSensor *s) { this->beacons_text_sensor_ = s; }
  void add_on_learned_callback(std::function<void(std::string, std::string)> &&cb) {
    this->learned_cb_.add(std::move(cb));
  }
  void add_on_learn_failed_callback(std::function<void(std::string)> &&cb) {
    this->learn_failed_cb_.add(std::move(cb));
  }

 protected:
  struct Stranger {  // aparato cercano que no está en la lista (modo `discovery`)
    uint64_t address{0};
    int8_t rssi{-127};
    bool ibeacon{false};
    uint8_t uuid[16]{};
    uint16_t major{0};
    uint16_t minor{0};
    char name[16]{};
  };

  // Lo que se guarda en la memoria del cubo por cada baliza aprendida.
  struct StoredBeacon {
    uint8_t used;
    uint8_t by_mac;
    uint8_t mac[6];
    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t offset;
    char room[25];
  } __attribute__((packed));
  struct StoredTable {
    uint32_t magic;
    StoredBeacon items[MAX_LEARNED];
  } __attribute__((packed));
  // Aparato oído mientras se aprende.
  struct Candidate {
    bool used{false};
    bool ibeacon{false};
    uint64_t address{0};
    uint8_t uuid[16]{};
    uint16_t major{0};
    uint16_t minor{0};
    int32_t rssi_sum{0};
    uint16_t count{0};
  };

  void load_learned_();
  void save_learned_();
  void publish_beacons_(const std::string &prefix = "");
  void note_candidate_(uint64_t address, int rssi, const uint8_t *ibeacon);
  void finish_learn_();
  void fail_learn_(const std::string &reason);
  size_t learned_count_() const;
  void refresh_any_ibeacon_();

  uint8_t room_index_(const std::string &room);
  void publish_room_();
  void note_stranger_(uint64_t address, int rssi, const uint8_t *ibeacon, const uint8_t *adv, size_t len);
  void log_strangers_();

  RoomLogic logic_;
  std::vector<std::string> rooms_;
  std::string unknown_room_;
  bool any_ibeacon_{false};
  bool discovery_{false};
  bool scanning_{false};
  uint32_t scan_ms_{0};          // reloj de escaneo
  uint32_t last_loop_ms_{0};
  uint32_t last_eval_ms_{0};
  uint32_t last_rssi_pub_ms_{0};
  uint32_t last_discovery_ms_{0};
  static const size_t MAX_STRANGERS = 6;
  Stranger strangers_[MAX_STRANGERS];

  text_sensor::TextSensor *room_text_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};
  CallbackManager<void(std::string)> room_cb_;

  // aprendizaje
  ESPPreferenceObject pref_;
  bool learning_{false};
  std::string learn_room_;
  uint32_t learn_scan_end_{0};   // reloj de escaneo
  uint32_t learn_deadline_{0};   // millis(): tope por si el escáner está parado
  int8_t learn_min_rssi_{-65};
  static const size_t MAX_CANDIDATES = 8;
  Candidate candidates_[MAX_CANDIDATES];
  text_sensor::TextSensor *beacons_text_sensor_{nullptr};
  CallbackManager<void(std::string, std::string)> learned_cb_;
  CallbackManager<void(std::string)> learn_failed_cb_;
};

class RoomTrigger : public Trigger<std::string> {
 public:
  explicit RoomTrigger(CuboRoom *parent) {
    parent->add_on_room_callback([this](std::string room) { this->trigger(room); });
  }
};

class LearnedTrigger : public Trigger<std::string, std::string> {
 public:
  explicit LearnedTrigger(CuboRoom *parent) {
    parent->add_on_learned_callback([this](std::string room, std::string detail) { this->trigger(room, detail); });
  }
};

class LearnFailedTrigger : public Trigger<std::string> {
 public:
  explicit LearnFailedTrigger(CuboRoom *parent) {
    parent->add_on_learn_failed_callback([this](std::string reason) { this->trigger(reason); });
  }
};

template<typename... Ts> class LearnAction : public Action<Ts...> {
 public:
  explicit LearnAction(CuboRoom *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, room)
  void play(const Ts &...x) override { this->parent_->learn(this->room_.value(x...)); }

 protected:
  CuboRoom *parent_;
};

template<typename... Ts> class ForgetAction : public Action<Ts...> {
 public:
  explicit ForgetAction(CuboRoom *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, room)
  void play(const Ts &...x) override { this->parent_->forget(this->room_.value(x...)); }

 protected:
  CuboRoom *parent_;
};

template<typename... Ts> class ClearLearnedAction : public Action<Ts...> {
 public:
  explicit ClearLearnedAction(CuboRoom *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->clear_learned(); }

 protected:
  CuboRoom *parent_;
};

}  // namespace cubo_room
}  // namespace esphome

#endif  // USE_ESP32
