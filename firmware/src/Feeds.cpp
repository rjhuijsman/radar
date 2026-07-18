#include "Feeds.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <esp_heap_caps.h>
#include <math.h>

#include <algorithm>
#include <set>
#include <vector>

#include "config.h"

namespace feeds {
namespace {

// Traffic fetch tuning. The radius is the airplanes.live API maximum
// regardless of display range: the scope only zooms within the data, and
// the wide circle lets the set track flights across a large slice of
// Europe from either home. The aircraft cap bounds memory and per-frame
// draw cost when the circle is busy (it can hold well over a thousand
// aircraft on a summer afternoon).
constexpr int kFetchRadiusNm = 250;
constexpr size_t kMaxAircraft = 150;
constexpr uint32_t kHttpTimeoutMs = 20000;
constexpr char kUserAgent[] = "radar-720 (ESP32-S3 flight radar)";

// ICAO callsigns currently considered special, rebuilt by `pollIcal`.
// Only ever touched from the network task, so no lock is needed.
std::set<String> g_specials;

// ArduinoJson allocator backed by PSRAM, so a large traffic response can
// never exhaust the internal heap the radio and TLS stack live on.
class SpiRamAllocator : public ArduinoJson::Allocator {
 public:
  void* allocate(size_t size) override {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  void deallocate(void* pointer) override { heap_caps_free(pointer); }
  void* reallocate(void* pointer, size_t size) override {
    return heap_caps_realloc(pointer, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
};

// Custom ArduinoJson reader over the HTTP body: refills a small buffer
// with bulk client reads instead of the per-byte reads ArduinoJson would
// otherwise issue through the TLS stack (each of which polls the whole
// mbedtls engine), and yields to the scheduler on every refill so a long
// transfer leaves the core-0 idle task time to feed the task watchdog.
//
// A momentarily empty TLS buffer is NOT the end of the body — the secure
// client reports "no data" mid-stream while the next record is still in
// flight (its readBytes() even returns 0 there, a false EOF that truncates
// the parse). End of input is only reported once the connection has closed
// with nothing left decrypted, or nothing has arrived for the timeout.
class YieldingReader {
 public:
  explicit YieldingReader(NetworkClient& client) : client_(client) {}

  int read() {
    if (pos_ >= len_ && !fill()) return -1;
    return static_cast<uint8_t>(buffer_[pos_++]);
  }

  size_t readBytes(char* out, size_t length) {
    size_t total = 0;
    while (total < length) {
      if (pos_ >= len_ && !fill()) break;
      size_t chunk = min(length - total, len_ - pos_);
      memcpy(out + total, buffer_ + pos_, chunk);
      pos_ += chunk;
      total += chunk;
    }
    return total;
  }

 private:
  bool fill() {
    uint32_t deadline = millis() + kHttpTimeoutMs;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
      vTaskDelay(1);  // One tick for the idle task per refill attempt.
      if (client_.available() > 0) {
        int n = client_.read(reinterpret_cast<uint8_t*>(buffer_),
                             sizeof(buffer_));
        if (n > 0) {
          len_ = static_cast<size_t>(n);
          pos_ = 0;
          return true;
        }
      } else if (!client_.connected()) {
        return false;  // Clean end of body.
      }
    }
    return false;  // Stalled transfer; the parse reports IncompleteInput.
  }

  NetworkClient& client_;
  char buffer_[2048];
  size_t pos_ = 0;
  size_t len_ = 0;
};

// One aircraft parsed out of the traffic response, staged locally so the
// fetch and parse run without the model mutex.
struct Parsed {
  String callsign;
  model::Vec pos;
  float track = 0;
  float groundSpeed = 0;
  int32_t altitude = 0;
  bool special = false;
};

// Maps a 2-letter IATA airline code to its 3-letter ICAO prefix. Only a few
// common carriers for now. TODO(feeds): load the full table from LittleFS.
String iataToIcao(const String& iata) {
  struct Entry {
    const char* iata;
    const char* icao;
  };
  static const Entry table[] = {
      {"BA", "BAW"}, {"LH", "DLH"}, {"AF", "AFR"}, {"KL", "KLM"},
      {"U2", "EZY"}, {"FR", "RYR"}, {"UA", "UAL"}, {"AA", "AAL"},
      {"DL", "DAL"}, {"EK", "UAE"}, {"IB", "IBE"}, {"VS", "VIR"},
  };
  for (auto& entry : table) {
    if (iata == entry.iata) return entry.icao;
  }
  return "";
}

const model::Home& activeHome(const model::Model& model) {
  static model::Home origin;  // 0,0 fallback if no homes configured.
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    return model.homes[model.ui.homeIndex];
  }
  return origin;
}

// Converts a lat/lng to world NM (east, north) relative to home, using a
// local equirectangular approximation — accurate at radar ranges.
model::Vec geoToWorld(const model::Home& home, float lat, float lon) {
  float dLatNm = (lat - home.latitude) * 60.0f;
  float dLonNm = (lon - home.longitude) * 60.0f * cosf(home.latitude * DEG_TO_RAD);
  return model::Vec{dLonNm, dLatNm};
}

bool httpGet(const String& url, String& body) {
  // Feeds cannot work without a connection; skip rather than block on a
  // doomed connect. Callers run without the model mutex, so a slow server
  // costs feed latency, never rendered frames.
  if (!WiFi.isConnected()) return false;
  WiFiClientSecure client;
  // TODO(security): install a CA bundle and validate certificates instead
  // of trusting any peer.
  client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(kHttpTimeoutMs);
  http.setTimeout(kHttpTimeoutMs);
  http.setUserAgent(kUserAgent);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    body = http.getString();
  }
  http.end();
  return code == HTTP_CODE_OK;
}

// Fetches and parses the traffic response into `out` with NO lock held.
// The body is parsed straight off the TLS stream through a field filter,
// so the multi-hundred-kilobyte response is never buffered whole; the
// filtered document lands in PSRAM via the allocator above.
bool fetchTraffic(const model::Home& home, std::vector<Parsed>& out) {
  if (!WiFi.isConnected()) return false;

  String url = "https://api.airplanes.live/v2/point/" +
               String(home.latitude, 5) + "/" + String(home.longitude, 5) +
               "/" + String(kFetchRadiusNm);

  WiFiClientSecure client;
  // TODO(security): install a CA bundle and validate certificates.
  client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(kHttpTimeoutMs);
  http.setTimeout(kHttpTimeoutMs);
  http.setUserAgent(kUserAgent);
  // HTTP/1.0 keeps the server from chunk-encoding the body, so the raw
  // stream is directly parseable (the ArduinoJson + HTTPClient recipe).
  http.useHTTP10(true);
  // Do not start a parse the heap cannot absorb: the document lands in
  // PSRAM, but the TLS session and parser scratch live on the internal
  // heap. Skipping one poll is invisible; an OOM abort is not.
  if (ESP.getFreeHeap() < 40 * 1024) {
    Serial.printf("[feeds] adsb: heap low (%u KB); skipping poll.\n",
                  ESP.getFreeHeap() / 1024);
    return false;
  }

  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[feeds] adsb: HTTP %d; keeping last data.\n", code);
    http.end();
    return false;
  }
  // Sanity-cap the body: a busy summer afternoon runs to a few hundred
  // KB; anything wildly larger is not a traffic snapshot.
  int announced = http.getSize();
  if (announced > 4 * 1024 * 1024) {
    Serial.printf("[feeds] adsb: %d-byte body refused.\n", announced);
    http.end();
    return false;
  }

  // Keep only the fields used; everything else in the ~50-field records
  // is skipped during the parse and never allocated.
  JsonDocument filter;
  JsonObject fields = filter["ac"].add<JsonObject>();
  fields["flight"] = true;
  fields["lat"] = true;
  fields["lon"] = true;
  fields["alt_baro"] = true;
  fields["gs"] = true;
  fields["track"] = true;

  static SpiRamAllocator allocator;
  JsonDocument doc(&allocator);
  YieldingReader reader(http.getStream());
  DeserializationError err =
      deserializeJson(doc, reader, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    Serial.printf("[feeds] adsb: parse failed (%s); keeping last data.\n",
                  err.c_str());
    return false;
  }
  // A response without the aircraft array is not a traffic snapshot; do
  // not let it wipe the scope. An empty array is legitimately quiet sky.
  if (!doc["ac"].is<JsonArrayConst>()) {
    Serial.println("[feeds] adsb: no 'ac' array; keeping last data.");
    return false;
  }

  for (JsonObjectConst state : doc["ac"].as<JsonArrayConst>()) {
    const char* flight = state["flight"];
    if (flight == nullptr) continue;
    String callsign = String(flight);
    callsign.trim();
    if (callsign.isEmpty()) continue;
    if (!state["lat"].is<float>() || !state["lon"].is<float>()) continue;

    Parsed parsed;
    parsed.callsign = callsign;
    parsed.pos = geoToWorld(home, state["lat"], state["lon"]);
    parsed.track = state["track"] | 0.0f;
    parsed.groundSpeed = state["gs"] | 0.0f;
    // alt_baro is the string "ground" for taxiing aircraft; the fallback
    // maps any non-numeric value to 0 ft.
    parsed.altitude = state["alt_baro"] | 0;
    parsed.special = g_specials.count(callsign) > 0;
    out.push_back(std::move(parsed));
  }
  return true;
}

// Snapshot-cap ordering: special (calendar-matched) flights always survive
// the cut, then closer aircraft beat farther ones.
bool keepFirst(const Parsed& a, const Parsed& b) {
  if (a.special != b.special) return a.special;
  float da = a.pos.x * a.pos.x + a.pos.y * a.pos.y;
  float db = b.pos.x * b.pos.x + b.pos.y * b.pos.y;
  return da < db;
}

// Finds an existing aircraft by callsign so its sweep-refresh state carries
// across polls; appends a fresh one otherwise.
model::Aircraft& upsert(model::Model& model, const String& callsign) {
  for (auto& ac : model.aircraft) {
    if (ac.callsign == callsign) return ac;
  }
  model.aircraft.push_back(model::Aircraft{});
  model.aircraft.back().callsign = callsign;
  return model.aircraft.back();
}

}  // namespace

bool pollTraffic(model::Model& model, SemaphoreHandle_t mutex) {
  // Copy the active home out under the mutex, then fetch and parse with
  // no lock held: the render loop keeps its ~25 fps while the blocking
  // HTTPS request and parse run.
  xSemaphoreTake(mutex, portMAX_DELAY);
  model::Home home = activeHome(model);
  xSemaphoreGive(mutex);

  uint32_t started = millis();
  std::vector<Parsed> snapshot;
  if (!fetchTraffic(home, snapshot)) return false;
  size_t inRange = snapshot.size();

  if (snapshot.size() > kMaxAircraft) {
    std::nth_element(snapshot.begin(), snapshot.begin() + kMaxAircraft,
                     snapshot.end(), keepFirst);
    snapshot.resize(kMaxAircraft);
  }

  // Merge under the mutex, briefly: upsert so each blip's sweep-refresh
  // state carries across polls, then drop aircraft that left the feed.
  xSemaphoreTake(mutex, portMAX_DELAY);

  // Removals shuffle indexes, so remember the followed aircraft by
  // callsign and re-point (or drop) the follow after the merge.
  String followed;
  if (model.ui.following && model.ui.followIndex >= 0 &&
      model.ui.followIndex < static_cast<int>(model.aircraft.size())) {
    followed = model.aircraft[model.ui.followIndex].callsign;
  }

  std::set<String> present;
  for (const auto& parsed : snapshot) {
    model::Aircraft& ac = upsert(model, parsed.callsign);
    ac.pos = parsed.pos;
    ac.track = parsed.track;
    ac.groundSpeed = parsed.groundSpeed;
    ac.altitude = parsed.altitude;
    ac.special = parsed.special;
    present.insert(parsed.callsign);
  }
  for (int i = static_cast<int>(model.aircraft.size()) - 1; i >= 0; --i) {
    if (present.count(model.aircraft[i].callsign) == 0) {
      model.aircraft.erase(model.aircraft.begin() + i);
    }
  }

  int count = static_cast<int>(model.aircraft.size());
  if (!followed.isEmpty()) {
    model.ui.followIndex = -1;
    for (int i = 0; i < count; ++i) {
      if (model.aircraft[i].callsign == followed) {
        model.ui.followIndex = i;
        break;
      }
    }
    if (model.ui.followIndex < 0) {  // The followed flight left the feed.
      model.ui.following = false;
      model.ui.candidate = -2;
    }
  }
  // The transient browse selections are only meaningful against the old
  // indexing; clamp any that fell off the end.
  if (model.ui.browseSel >= count) model.ui.browseSel = -1;
  if (model.ui.candidate >= count) model.ui.candidate = -2;
  xSemaphoreGive(mutex);

  Serial.printf("[feeds] adsb: %d aircraft (%u in range, %lu ms, heap %u KB)\n",
                count, static_cast<unsigned>(inRange),
                static_cast<unsigned long>(millis() - started),
                ESP.getFreeHeap() / 1024);
  return true;
}

int pollIcal(model::Model& model, SemaphoreHandle_t mutex) {
  // Copy the enabled feed URLs out under the mutex; the fetches below run
  // with no lock held.
  std::vector<String> urls;
  xSemaphoreTake(mutex, portMAX_DELAY);
  for (const auto& feed : model.feeds) {
    if (feed.enabled) urls.push_back(feed.url);
  }
  xSemaphoreGive(mutex);

  std::set<String> found;
  int fetched = 0;
  for (const auto& url : urls) {
    String body;
    if (!httpGet(url, body)) continue;
    ++fetched;

    // Heuristic extraction: scan for an IATA airline code followed by a
    // flight number (e.g. "BA 117") and convert to the ICAO callsign the
    // ADS-B feed broadcasts. TODO(feeds): unfold iCal lines, respect
    // VEVENT boundaries, and filter to flights happening today.
    for (int i = 0; i + 2 < static_cast<int>(body.length()); ++i) {
      char a = body[i], b = body[i + 1];
      if (a < 'A' || a > 'Z' || b < 'A' || b > 'Z') continue;
      int j = i + 2;
      while (j < static_cast<int>(body.length()) && body[j] == ' ') ++j;
      if (j >= static_cast<int>(body.length()) || !isdigit(body[j])) continue;
      String number;
      while (j < static_cast<int>(body.length()) && isdigit(body[j])) {
        number += body[j++];
      }
      if (number.length() < 2 || number.length() > 4) continue;
      String icao = iataToIcao(String(a) + String(b));
      if (!icao.isEmpty()) found.insert(icao + number);
    }
  }
  g_specials = found;
  return fetched;
}

void pollWeather(model::Model& model, SemaphoreHandle_t mutex) {
  // TODO(weather): fetch the latest RainViewer frame's tiles covering the
  // view and reproject them into the scope's azimuthal projection. Follow
  // the pollTraffic pattern: fetch and reproject with no lock held, then
  // merge under the mutex briefly.
  (void)model;
  (void)mutex;
}

void reprojectStatics(model::Model& model) {
  const model::Home& home = activeHome(model);
  for (auto& poi : model.pois) {
    poi.pos = geoToWorld(home, poi.latitude, poi.longitude);
  }
}

}  // namespace feeds
