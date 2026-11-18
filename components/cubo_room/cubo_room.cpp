#include "cubo_room.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <esp_attr.h>

namespace esphome {
namespace cubo_room {

static const char *const TAG = "cubo_room";

// La habitación sobrevive al deep sleep en la memoria RTC: al despertar el cubo ya sabe dónde
// estaba, sin esperar al escaneo. Un reinicio normal (alimentación, OTA) la borra.
static const uint32_t RTC_MAGIC = 0x526F6F6D;  // "Room"
static RTC_DATA_ATTR uint32_t rtc_magic = 0;
static RTC_DATA_ATTR uint32_t rtc_room_hash = 0;

static const uint32_t EVAL_INTERVAL_MS = 250;
static const uint32_t RSSI_PUBLISH_MS = 5000;
static const uint32_t DISCOVERY_LOG_MS = 10000;
static const int DISCOVERY_MIN_RSSI = -80;

// Aprendizaje de balizas: cuánto escucha, cuántos anuncios pide como mínimo y el tope de espera
// por si el escáner está parado.
static const uint32_t LEARN_SCAN_MS = 10000;
static const uint32_t LEARN_MAX_MS = 30000;
static const uint16_t LEARN_MIN_HITS = 2;
static const size_t ROOM_NAME_MAX = 24;
static const uint32_t STORE_MAGIC = 0x424C4531;  // "BLE1"

static std::string trim(const std::string &s) {
  const size_t a = s.find_first_not_of(" \t");
  if (a == std::string::npos)
    return "";
  const size_t b = s.find_last_not_of(" \t");
  return s.substr(a, b - a + 1);
}

static std::string describe_beacon(bool by_mac, uint64_t a, const uint8_t *uuid, int major, int minor) {
  char buf[64];
  if (by_mac) {
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", (unsigned) ((a >> 40) & 0xFF),
             (unsigned) ((a >> 32) & 0xFF), (unsigned) ((a >> 24) & 0xFF), (unsigned) ((a >> 16) & 0xFF),
             (unsigned) ((a >> 8) & 0xFF), (unsigned) (a & 0xFF));
  } else {
    snprintf(buf, sizeof(buf), "iBeacon %02x%02x%02x%02x %d/%d", (unsigned) uuid[0], (unsigned) uuid[1],
             (unsigned) uuid[2], (unsigned) uuid[3], major, minor);
  }
  return buf;
}

static uint32_t hash_name(const std::string &s) {  // FNV-1a
  uint32_t h = 2166136261UL;
  for (char c : s) {
    h ^= (uint8_t) c;
    h *= 16777619UL;
  }
  return h;
}

// Busca los datos iBeacon dentro de un anuncio: fabricante 0x004C (Apple), tipo 0x02, largo 0x15,
// y después UUID (16), major (2), minor (2) y potencia (1). Devuelve el puntero al UUID o nullptr.
static const uint8_t *find_ibeacon(const uint8_t *adv, size_t len) {
  size_t pos = 0;
  while (pos + 1 < len) {
    const uint8_t field_len = adv[pos];
    if (field_len == 0 || pos + 1 + field_len > len)
      break;
    const uint8_t *data = &adv[pos + 2];
    if (adv[pos + 1] == 0xFF && field_len >= 26 && data[0] == 0x4C && data[1] == 0x00 && data[2] == 0x02 &&
        data[3] == 0x15)
      return data + 4;
    pos += (size_t) field_len + 1;
  }
  return nullptr;
}

uint8_t CuboRoom::room_index_(const std::string &room) {
  for (size_t i = 0; i < this->rooms_.size(); i++) {
    if (this->rooms_[i] == room)
      return (uint8_t) i;
  }
  this->rooms_.push_back(room);
  return (uint8_t) (this->rooms_.size() - 1);
}

void CuboRoom::add_mac_beacon(const std::string &room, uint64_t address, int offset) {
  Beacon b;
  b.room = this->room_index_(room);
  b.by_mac = true;
  b.address = address;
  b.offset = (int8_t) offset;
  this->logic_.beacons.push_back(b);
}

void CuboRoom::add_ibeacon(const std::string &room, uint64_t uuid_hi, uint64_t uuid_lo, int major, int minor,
                           int offset) {
  Beacon b;
  b.room = this->room_index_(room);
  b.by_mac = false;
  for (int i = 0; i < 8; i++) {
    b.uuid[i] = (uint8_t) (uuid_hi >> (56 - 8 * i));
    b.uuid[8 + i] = (uint8_t) (uuid_lo >> (56 - 8 * i));
  }
  b.major = major;
  b.minor = minor;
  b.offset = (int8_t) offset;
  this->logic_.beacons.push_back(b);
  this->any_ibeacon_ = true;
}

const std::string &CuboRoom::room() const {
  if (this->logic_.room >= 0 && (size_t) this->logic_.room < this->rooms_.size())
    return this->rooms_[this->logic_.room];
  return this->unknown_room_;
}

// ---------------------------------------------------------------------------------------------
//  Balizas aprendidas
// ---------------------------------------------------------------------------------------------
size_t CuboRoom::learned_count_() const {
  size_t n = 0;
  for (const Beacon &b : this->logic_.beacons) {
    if (b.learned)
      n++;
  }
  return n;
}

void CuboRoom::refresh_any_ibeacon_() {
  this->any_ibeacon_ = false;
  for (const Beacon &b : this->logic_.beacons) {
    if (!b.by_mac)
      this->any_ibeacon_ = true;
  }
}

// Trae de la memoria las balizas aprendidas. Si una coincide con otra del YAML, manda la aprendida.
void CuboRoom::load_learned_() {
  this->pref_ = global_preferences->make_preference<StoredTable>(fnv1_hash("cubo_room.learned.v1"));
  StoredTable t;
  if (!this->pref_.load(&t) || t.magic != STORE_MAGIC)
    return;
  for (size_t n = 0; n < MAX_LEARNED; n++) {
    StoredBeacon s;
    memcpy(&s, &t.items[n], sizeof(s));
    if (s.used != 1 || s.room[0] == 0)
      continue;
    char name[sizeof(s.room) + 1];
    memcpy(name, s.room, sizeof(s.room));
    name[sizeof(s.room)] = 0;
    uint64_t address = 0;
    for (int i = 0; i < 6; i++)
      address = (address << 8) | s.mac[i];
    Beacon *slot = nullptr;
    for (Beacon &b : this->logic_.beacons) {
      const bool same = s.by_mac != 0 ? (b.by_mac && b.address == address)
                                      : (!b.by_mac && memcmp(b.uuid, s.uuid, 16) == 0 && b.major == (int32_t) s.major &&
                                         b.minor == (int32_t) s.minor);
      if (same) {
        slot = &b;
        break;
      }
    }
    if (slot == nullptr) {
      this->logic_.beacons.emplace_back();
      slot = &this->logic_.beacons.back();
    }
    slot->room = this->room_index_(name);
    slot->by_mac = s.by_mac != 0;
    slot->learned = true;
    slot->offset = s.offset;
    slot->address = address;
    memcpy(slot->uuid, s.uuid, 16);
    slot->major = slot->by_mac ? -1 : (int32_t) s.major;
    slot->minor = slot->by_mac ? -1 : (int32_t) s.minor;
  }
  this->refresh_any_ibeacon_();
}

void CuboRoom::save_learned_() {
  StoredTable t;
  memset(&t, 0, sizeof(t));
  t.magic = STORE_MAGIC;
  size_t n = 0;
  for (const Beacon &b : this->logic_.beacons) {
    if (!b.learned || n >= MAX_LEARNED)
      continue;
    StoredBeacon s;
    memset(&s, 0, sizeof(s));
    s.used = 1;
    s.by_mac = b.by_mac ? 1 : 0;
    for (int i = 0; i < 6; i++)
      s.mac[i] = (uint8_t) (b.address >> (40 - 8 * i));
    memcpy(s.uuid, b.uuid, 16);
    s.major = b.major < 0 ? 0 : (uint16_t) b.major;
    s.minor = b.minor < 0 ? 0 : (uint16_t) b.minor;
    s.offset = b.offset;
    strncpy(s.room, this->rooms_[b.room].c_str(), sizeof(s.room) - 1);
    memcpy(&t.items[n++], &s, sizeof(s));
  }
  this->pref_.save(&t);
  global_preferences->sync();  // a la flash ya: el cubo puede dormirse en segundos
}

// Texto para la entidad "Balizas": cuántas hay en cada habitación.
void CuboRoom::publish_beacons_(const std::string &prefix) {
  if (this->beacons_text_sensor_ == nullptr)
    return;
  std::string out = prefix;
  bool any = false;
  for (size_t r = 0; r < this->rooms_.size(); r++) {
    unsigned n = 0;
    for (const Beacon &b : this->logic_.beacons) {
      if (b.room == r)
        n++;
    }
    if (n == 0)
      continue;
    char buf[16];
    snprintf(buf, sizeof(buf), " (%u)", n);
    if (any)
      out += ", ";
    out += this->rooms_[r] + buf;
    any = true;
  }
  if (!any)
    out += "No beacons";
  if (out.size() > 250)
    out.resize(250);
  this->beacons_text_sensor_->publish_state(out);
}

void CuboRoom::learn(const std::string &room) {
  const std::string name = trim(room);
  if (name.empty() || name.size() > ROOM_NAME_MAX) {
    this->learn_room_ = name;
    this->fail_learn_("room name missing or longer than 24 characters");
    return;
  }
  this->learn_room_ = name;
  for (Candidate &c : this->candidates_)
    c = Candidate();
  this->learning_ = true;
  this->learn_scan_end_ = this->scan_ms_ + LEARN_SCAN_MS;
  this->learn_deadline_ = millis() + LEARN_MAX_MS;
  ESP_LOGI(TAG, "Aprendiendo la baliza de \"%s\": escucho %u s. El cubo tiene que estar al lado de la baliza.",
           name.c_str(), (unsigned) (LEARN_SCAN_MS / 1000));
  this->publish_beacons_("Learning " + name + "... ");
}

void CuboRoom::note_candidate_(uint64_t address, int rssi, const uint8_t *ibeacon) {
  const uint16_t major = ibeacon != nullptr ? (uint16_t) ((ibeacon[16] << 8) | ibeacon[17]) : 0;
  const uint16_t minor = ibeacon != nullptr ? (uint16_t) ((ibeacon[18] << 8) | ibeacon[19]) : 0;
  Candidate *slot = nullptr, *free_slot = nullptr, *weakest = nullptr;
  for (Candidate &c : this->candidates_) {
    if (!c.used) {
      if (free_slot == nullptr)
        free_slot = &c;
      continue;
    }
    const bool same = ibeacon != nullptr
                          ? (c.ibeacon && memcmp(c.uuid, ibeacon, 16) == 0 && c.major == major && c.minor == minor)
                          : (!c.ibeacon && c.address == address);
    if (same) {
      slot = &c;
      break;
    }
    if (weakest == nullptr || c.rssi_sum / (int32_t) c.count < weakest->rssi_sum / (int32_t) weakest->count)
      weakest = &c;
  }
  if (slot == nullptr) {
    if (free_slot != nullptr) {
      slot = free_slot;
    } else {
      // Tabla llena: entra solo si llega más fuerte que el más flojo de los anotados.
      if (weakest == nullptr || rssi <= weakest->rssi_sum / (int32_t) weakest->count)
        return;
      slot = weakest;
    }
    *slot = Candidate();
    slot->used = true;
    slot->address = address;
    if (ibeacon != nullptr) {
      slot->ibeacon = true;
      memcpy(slot->uuid, ibeacon, 16);
      slot->major = major;
      slot->minor = minor;
    }
  }
  slot->rssi_sum += rssi;
  slot->count++;
}

void CuboRoom::fail_learn_(const std::string &reason) {
  this->learning_ = false;
  ESP_LOGW(TAG, "No aprendí la baliza de \"%s\": %s", this->learn_room_.c_str(), reason.c_str());
  this->publish_beacons_("Not learned: " + reason + ". ");
  this->learn_failed_cb_.call(reason);
}

void CuboRoom::finish_learn_() {
  this->learning_ = false;
  const Candidate *best = nullptr;
  float best_mean = -200.0f;
  for (const Candidate &c : this->candidates_) {
    if (!c.used || c.count < LEARN_MIN_HITS)
      continue;
    const float mean = (float) c.rssi_sum / (float) c.count;
    if (mean > best_mean) {
      best = &c;
      best_mean = mean;
    }
  }
  if (best == nullptr) {
    this->fail_learn_("no beacon heard nearby");
    return;
  }
  const uint8_t room = this->room_index_(this->learn_room_);
  Beacon *slot = nullptr;
  for (Beacon &b : this->logic_.beacons) {
    const bool same = best->ibeacon ? (!b.by_mac && memcmp(b.uuid, best->uuid, 16) == 0 &&
                                       b.major == (int32_t) best->major && b.minor == (int32_t) best->minor)
                                    : (b.by_mac && b.address == best->address);
    if (same) {
      slot = &b;
      break;
    }
  }
  if (slot == nullptr) {
    if (this->learned_count_() >= MAX_LEARNED) {
      this->fail_learn_("beacon memory is full");
      return;
    }
    this->logic_.beacons.emplace_back();
    slot = &this->logic_.beacons.back();
    slot->by_mac = !best->ibeacon;
    slot->address = best->address;
    memcpy(slot->uuid, best->uuid, 16);
    slot->major = best->ibeacon ? (int32_t) best->major : -1;
    slot->minor = best->ibeacon ? (int32_t) best->minor : -1;
  }
  slot->room = room;
  slot->learned = true;
  slot->hits = 0;
  this->logic_.forget_tracking();
  this->refresh_any_ibeacon_();
  this->save_learned_();

  char tail[24];
  snprintf(tail, sizeof(tail), " a %.0f dBm", best_mean);
  const std::string detail =
      describe_beacon(slot->by_mac, slot->address, slot->uuid, (int) slot->major, (int) slot->minor) + tail;
  ESP_LOGI(TAG, "Baliza de \"%s\" aprendida: %s (%u anuncios)", this->learn_room_.c_str(), detail.c_str(),
           (unsigned) best->count);
  this->publish_beacons_();
  this->learned_cb_.call(this->learn_room_, detail);
}

void CuboRoom::forget(const std::string &room) {
  const std::string name = trim(room);
  std::vector<Beacon> &v = this->logic_.beacons;
  size_t removed = 0;
  for (size_t i = v.size(); i-- > 0;) {
    if (v[i].learned && this->rooms_[v[i].room] == name) {
      v.erase(v.begin() + (long) i);
      removed++;
    }
  }
  if (removed == 0) {
    ESP_LOGW(TAG, "No había balizas aprendidas en \"%s\"", name.c_str());
    this->publish_beacons_("No learned beacons in " + name + ". ");
    return;
  }
  this->logic_.forget_tracking();
  this->refresh_any_ibeacon_();
  this->save_learned_();
  ESP_LOGI(TAG, "Olvidé %u baliza(s) de \"%s\"", (unsigned) removed, name.c_str());
  this->publish_beacons_();
}

void CuboRoom::clear_learned() {
  std::vector<Beacon> &v = this->logic_.beacons;
  size_t removed = 0;
  for (size_t i = v.size(); i-- > 0;) {
    if (v[i].learned) {
      v.erase(v.begin() + (long) i);
      removed++;
    }
  }
  this->logic_.forget_tracking();
  this->refresh_any_ibeacon_();
  this->save_learned_();
  ESP_LOGI(TAG, "Olvidé todas las balizas aprendidas (%u)", (unsigned) removed);
  this->publish_beacons_();
}

void CuboRoom::setup() {
  this->load_learned_();
  this->publish_beacons_();
  if (rtc_magic == RTC_MAGIC) {
    for (size_t i = 0; i < this->rooms_.size(); i++) {
      if (hash_name(this->rooms_[i]) == rtc_room_hash) {
        this->logic_.room = (int) i;
        break;
      }
    }
  }
  this->last_loop_ms_ = millis();
  if (this->room_text_sensor_ != nullptr)
    this->room_text_sensor_->publish_state(this->room());
  if (this->logic_.room >= 0)
    ESP_LOGI(TAG, "Habitación al despertar: %s", this->room().c_str());
}

void CuboRoom::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Cubo room:\n"
                "  Balizas: %u en %u habitaciones\n"
                "  Histéresis: %.0f dB, mínimo %.0f dBm\n"
                "  Baliza perdida a los %" PRIu32 " ms, decisión inicial a los %" PRIu32 " ms de escaneo",
                (unsigned) this->logic_.beacons.size(), (unsigned) this->rooms_.size(), this->logic_.hysteresis,
                this->logic_.min_rssi, this->logic_.timeout_ms, this->logic_.settle_ms);
  for (const Beacon &b : this->logic_.beacons) {
    ESP_LOGCONFIG(TAG, "  %s: %s, offset %d dB%s", this->rooms_[b.room].c_str(),
                  describe_beacon(b.by_mac, b.address, b.uuid, (int) b.major, (int) b.minor).c_str(), (int) b.offset,
                  b.learned ? " (aprendida)" : "");
  }
  ESP_LOGCONFIG(TAG, "  Aprendidas: %u de %u; señal mínima para aprender %d dBm", (unsigned) this->learned_count_(),
                (unsigned) MAX_LEARNED, (int) this->learn_min_rssi_);
  if (this->discovery_)
    ESP_LOGCONFIG(TAG, "  Descubrimiento: encendido (lista los aparatos BLE cercanos cada 10 s)");
}

bool CuboRoom::parse_devices(const esp32_ble::BLEScanResult *scan_results, size_t count) {
  for (size_t n = 0; n < count; n++) {
    const esp32_ble::BLEScanResult &r = scan_results[n];
    if (r.rssi >= 20)
      continue;  // 127 = el controlador no pudo medir
    uint64_t address = 0;
    for (int i = 0; i < 6; i++)
      address = (address << 8) | r.bda[i];
    const size_t len = (size_t) r.adv_data_len + (size_t) r.scan_rsp_len;
    const uint8_t *ib =
        (this->any_ibeacon_ || this->discovery_ || this->learning_) ? find_ibeacon(r.ble_adv, len) : nullptr;
    if (this->learning_ && r.rssi >= this->learn_min_rssi_)
      this->note_candidate_(address, r.rssi, ib);

    bool known = false;
    for (size_t i = 0; i < this->logic_.beacons.size(); i++) {
      const Beacon &b = this->logic_.beacons[i];
      bool match;
      if (b.by_mac) {
        match = b.address == address;
      } else {
        match = ib != nullptr && memcmp(ib, b.uuid, 16) == 0 &&
                (b.major < 0 || b.major == ((ib[16] << 8) | ib[17])) &&
                (b.minor < 0 || b.minor == ((ib[18] << 8) | ib[19]));
      }
      if (match) {
        this->logic_.hit(i, r.rssi, this->scan_ms_);
        known = true;
      }
    }
    if (!known && this->discovery_ && r.rssi >= DISCOVERY_MIN_RSSI)
      this->note_stranger_(address, r.rssi, ib, r.ble_adv, len);
  }
  return false;
}

void CuboRoom::loop() {
  const uint32_t now = millis();
  uint32_t dt = now - this->last_loop_ms_;
  this->last_loop_ms_ = now;
  if (dt > 500)
    dt = 500;  // un bloqueo largo del lazo no cuenta entero como tiempo de escaneo
  this->scanning_ =
      this->parent_ != nullptr && this->parent_->get_scanner_state() == esp32_ble_tracker::ScannerState::RUNNING;
  if (this->scanning_)
    this->scan_ms_ += dt;

  if (this->learning_) {
    if (this->scan_ms_ >= this->learn_scan_end_) {
      this->finish_learn_();
    } else if ((int32_t) (now - this->learn_deadline_) >= 0) {
      this->fail_learn_("Bluetooth is not scanning");
    }
  }

  if (now - this->last_eval_ms_ >= EVAL_INTERVAL_MS) {
    this->last_eval_ms_ = now;
    if (this->logic_.evaluate(this->scan_ms_))
      this->publish_room_();
  }

  if (this->rssi_sensor_ != nullptr && now - this->last_rssi_pub_ms_ >= RSSI_PUBLISH_MS) {
    this->last_rssi_pub_ms_ = now;
    const int c = this->logic_.current;
    if (c >= 0) {
      this->rssi_sensor_->publish_state(this->logic_.beacons[c].rssi);
    } else if (this->rssi_sensor_->has_state() && !std::isnan(this->rssi_sensor_->state)) {
      this->rssi_sensor_->publish_state(NAN);
    }
  }

  if (this->discovery_ && now - this->last_discovery_ms_ >= DISCOVERY_LOG_MS) {
    this->last_discovery_ms_ = now;
    this->log_strangers_();
  }
}

void CuboRoom::publish_room_() {
  const std::string &name = this->room();
  if (this->logic_.room >= 0) {
    rtc_room_hash = hash_name(name);
    rtc_magic = RTC_MAGIC;
    const int c = this->logic_.current;
    ESP_LOGI(TAG, "Habitación: %s (%.0f dBm)", name.c_str(), c >= 0 ? this->logic_.beacons[c].rssi : NAN);
  } else {
    rtc_magic = 0;
    ESP_LOGI(TAG, "Habitación desconocida: no se oye ninguna baliza de la lista");
  }
  if (this->room_text_sensor_ != nullptr)
    this->room_text_sensor_->publish_state(name);
  this->room_cb_.call(name);
}

void CuboRoom::note_stranger_(uint64_t address, int rssi, const uint8_t *ibeacon, const uint8_t *adv, size_t len) {
  Stranger *slot = nullptr;
  for (Stranger &s : this->strangers_) {
    if (s.address == address) {
      slot = &s;
      break;
    }
  }
  if (slot == nullptr) {
    Stranger *weakest = &this->strangers_[0];
    for (Stranger &s : this->strangers_) {
      if (s.rssi < weakest->rssi)
        weakest = &s;
    }
    if (rssi <= weakest->rssi)
      return;
    slot = weakest;
    *slot = Stranger();
    slot->address = address;
  }
  if (rssi > slot->rssi)
    slot->rssi = (int8_t) rssi;
  if (ibeacon != nullptr) {
    slot->ibeacon = true;
    memcpy(slot->uuid, ibeacon, 16);
    slot->major = (uint16_t) ((ibeacon[16] << 8) | ibeacon[17]);
    slot->minor = (uint16_t) ((ibeacon[18] << 8) | ibeacon[19]);
  }
  // Nombre del aparato, si lo anuncia (campos 0x09 completo o 0x08 abreviado).
  size_t pos = 0;
  while (pos + 1 < len) {
    const uint8_t field_len = adv[pos];
    if (field_len == 0 || pos + 1 + field_len > len)
      break;
    if (adv[pos + 1] == 0x09 || adv[pos + 1] == 0x08) {
      size_t n = (size_t) field_len - 1;
      if (n > sizeof(slot->name) - 1)
        n = sizeof(slot->name) - 1;
      memcpy(slot->name, &adv[pos + 2], n);
      slot->name[n] = 0;
      for (size_t i = 0; i < n; i++) {
        if (slot->name[i] < 0x20 || slot->name[i] > 0x7E)
          slot->name[i] = '?';
      }
      break;
    }
    pos += (size_t) field_len + 1;
  }
}

void CuboRoom::log_strangers_() {
  bool any = false;
  for (size_t pass = 0; pass < MAX_STRANGERS; pass++) {  // de la más fuerte a la más floja
    Stranger *top = nullptr;
    for (Stranger &s : this->strangers_) {
      if (s.address != 0 && (top == nullptr || s.rssi > top->rssi))
        top = &s;
    }
    if (top == nullptr)
      break;
    if (!any) {
      ESP_LOGI(TAG, "Aparatos BLE cercanos que no están en la lista de balizas:");
      any = true;
    }
    const uint64_t a = top->address;
    if (top->ibeacon) {
      const uint8_t *u = top->uuid;
      ESP_LOGI(TAG,
               "  %02X:%02X:%02X:%02X:%02X:%02X  %d dBm  %s  iBeacon "
               "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x major %u minor %u",
               (unsigned) ((a >> 40) & 0xFF), (unsigned) ((a >> 32) & 0xFF), (unsigned) ((a >> 24) & 0xFF),
               (unsigned) ((a >> 16) & 0xFF), (unsigned) ((a >> 8) & 0xFF), (unsigned) (a & 0xFF), (int) top->rssi,
               top->name, (unsigned) u[0], (unsigned) u[1], (unsigned) u[2], (unsigned) u[3], (unsigned) u[4],
               (unsigned) u[5], (unsigned) u[6], (unsigned) u[7], (unsigned) u[8], (unsigned) u[9], (unsigned) u[10],
               (unsigned) u[11], (unsigned) u[12], (unsigned) u[13], (unsigned) u[14], (unsigned) u[15],
               (unsigned) top->major, (unsigned) top->minor);
    } else {
      ESP_LOGI(TAG, "  %02X:%02X:%02X:%02X:%02X:%02X  %d dBm  %s", (unsigned) ((a >> 40) & 0xFF),
               (unsigned) ((a >> 32) & 0xFF), (unsigned) ((a >> 24) & 0xFF), (unsigned) ((a >> 16) & 0xFF),
               (unsigned) ((a >> 8) & 0xFF), (unsigned) (a & 0xFF), (int) top->rssi, top->name);
    }
    *top = Stranger();
  }
}

}  // namespace cubo_room
}  // namespace esphome

#endif  // USE_ESP32
