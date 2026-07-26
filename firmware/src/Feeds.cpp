#include "Feeds.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PNGdec.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <esp_heap_caps.h>
#include <math.h>
#include <miniz.h>  // The S3 mask ROM's inflate (tinfl); nothing links in.
#include <time.h>

#include <algorithm>
#include <new>
#include <set>
#include <vector>

#include "Log.h"
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
// A special flight beyond the fetch circle is absent from the radius
// response entirely, so each traffic poll follows up with world-wide
// callsign lookups for the active special callsigns the circle did not
// deliver. The cap bounds the extra HTTPS requests a long specials list
// could otherwise queue every poll.
constexpr size_t kMaxGlobalLookups = 8;
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
  String hex;  // ICAO 24-bit address, lower-case hex; "" if absent.
  model::Vec pos;
  float track = 0;
  float groundSpeed = 0;
  int32_t altitude = 0;
  bool special = false;
};

// Maps a 2-character IATA airline designator to its 3-letter ICAO prefix
// (the one ADS-B callsigns start with). A generous slice of the carriers
// likely to appear in a trip calendar; anything missing still matches when
// the ICAO callsign is entered directly. TODO(feeds): load the full table
// from LittleFS.
String iataToIcao(const String& iata) {
  struct Entry {
    const char* iata;
    const char* icao;
  };
  static const Entry table[] = {
      {"AA", "AAL"}, {"AC", "ACA"}, {"AF", "AFR"}, {"AI", "AIC"},
      {"AS", "ASA"}, {"AY", "FIN"}, {"AZ", "ITY"}, {"A3", "AEE"},
      {"BA", "BAW"}, {"BR", "EVA"}, {"BT", "BTI"}, {"BY", "TOM"},
      {"B6", "JBU"}, {"CA", "CCA"}, {"CI", "CAL"}, {"CX", "CPA"},
      {"CZ", "CSN"}, {"DL", "DAL"}, {"EI", "EIN"}, {"EK", "UAE"},
      {"ET", "ETH"}, {"EW", "EWG"}, {"EY", "ETD"}, {"FI", "ICE"},
      {"FR", "RYR"}, {"F9", "FFT"}, {"GA", "GIA"}, {"HA", "HAL"},
      {"HV", "TRA"}, {"IB", "IBE"}, {"JL", "JAL"}, {"JQ", "JST"},
      {"KE", "KAL"}, {"KL", "KLM"}, {"LH", "DLH"}, {"LO", "LOT"},
      {"LS", "EXS"}, {"LX", "SWR"}, {"LY", "ELY"}, {"MH", "MAS"},
      {"MS", "MSR"}, {"MU", "CES"}, {"NH", "ANA"}, {"NK", "NKS"},
      {"NZ", "ANZ"}, {"OR", "TFL"}, {"OS", "AUA"}, {"OZ", "AAR"},
      {"PC", "PGT"}, {"QF", "QFA"}, {"QR", "QTR"}, {"SK", "SAS"},
      {"SN", "BEL"}, {"SQ", "SIA"}, {"SU", "AFL"}, {"SV", "SVA"},
      {"TG", "THA"}, {"TK", "THY"}, {"TO", "TVF"}, {"TP", "TAP"},
      {"UA", "UAL"}, {"UX", "AEA"}, {"U2", "EZY"}, {"VA", "VOZ"},
      {"VS", "VIR"}, {"VY", "VLG"}, {"WN", "SWA"}, {"WS", "WJA"},
      {"W6", "WZZ"}, {"6E", "IGO"}, {"D8", "IBK"}, {"DY", "NAX"},
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
    rlog::Log.printf("[feeds] adsb: heap low (%u KB); skipping poll.\n",
                  ESP.getFreeHeap() / 1024);
    return false;
  }

  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    rlog::Log.printf("[feeds] adsb: HTTP %d; keeping last data.\n", code);
    http.end();
    return false;
  }
  // Sanity-cap the body: a busy summer afternoon runs to a few hundred
  // KB; anything wildly larger is not a traffic snapshot.
  int announced = http.getSize();
  if (announced > 4 * 1024 * 1024) {
    rlog::Log.printf("[feeds] adsb: %d-byte body refused.\n", announced);
    http.end();
    return false;
  }

  // Keep only the fields used; everything else in the ~50-field records
  // is skipped during the parse and never allocated.
  JsonDocument filter;
  JsonObject fields = filter["ac"].add<JsonObject>();
  fields["flight"] = true;
  fields["hex"] = true;
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
    rlog::Log.printf("[feeds] adsb: parse failed (%s); keeping last data.\n",
                  err.c_str());
    return false;
  }
  // A response without the aircraft array is not a traffic snapshot; do
  // not let it wipe the scope. An empty array is legitimately quiet sky.
  if (!doc["ac"].is<JsonArrayConst>()) {
    rlog::Log.println("[feeds] adsb: no 'ac' array; keeping last data.");
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
    parsed.hex = String(state["hex"] | "");
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

// Looks up each callsign in `wanted` world-wide — api.adsb.lol serves a
// per-callsign query whose response mirrors the traffic records — and
// appends every airborne match to `out` tagged special, with NO lock
// held. One TLS session is reused across the lookups; they all hit the
// same host. A callsign whose lookup FAILED (fetch or parse, as opposed
// to a clean "not airborne" empty result) goes into `failed`, so the
// caller can keep a previously merged flight alive through the hiccup.
void fetchGlobalSpecials(const model::Home& home,
                         const std::vector<String>& wanted,
                         std::vector<Parsed>& out, std::set<String>& failed) {
  WiFiClientSecure client;
  // TODO(security): install a CA bundle and validate certificates.
  client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(kHttpTimeoutMs);
  http.setTimeout(kHttpTimeoutMs);
  http.setUserAgent(kUserAgent);
  http.setReuse(true);  // The lookups share one host (and TLS session).
  for (const String& callsign : wanted) {
    bool ok = false;
    // The internal-heap guard matches the traffic path: the TLS session
    // and scratch live there, and skipping a lookup is invisible.
    if (WiFi.isConnected() && ESP.getFreeHeap() >= 40 * 1024 &&
        http.begin(client, "https://api.adsb.lol/v2/callsign/" + callsign)) {
      int code = http.GET();
      if (code == HTTP_CODE_OK) {
        // The body is a handful of aircraft at most, so the String
        // round-trip is fine; an empty "ac" array is simply a callsign
        // not airborne right now.
        static SpiRamAllocator allocator;
        JsonDocument doc(&allocator);
        if (deserializeJson(doc, http.getString()) ==
                DeserializationError::Ok &&
            doc["ac"].is<JsonArrayConst>()) {
          ok = true;
          for (JsonObjectConst state : doc["ac"].as<JsonArrayConst>()) {
            const char* flight = state["flight"];
            if (flight == nullptr) continue;
            String broadcast = String(flight);
            broadcast.trim();
            if (broadcast.isEmpty()) continue;
            if (!state["lat"].is<float>() || !state["lon"].is<float>()) {
              continue;
            }
            Parsed parsed;
            parsed.callsign = broadcast;
            parsed.hex = String(state["hex"] | "");
            parsed.pos = geoToWorld(home, state["lat"], state["lon"]);
            parsed.track = state["track"] | 0.0f;
            parsed.groundSpeed = state["gs"] | 0.0f;
            parsed.altitude = state["alt_baro"] | 0;
            parsed.special = true;  // Looked up because it is one.
            out.push_back(std::move(parsed));
          }
        }
      } else {
        rlog::Log.printf("[feeds] adsb: global HTTP %d for %s.\n", code,
                      callsign.c_str());
      }
      http.end();
    }
    if (!ok) failed.insert(callsign);
  }
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

// ---- Rain radar (RainViewer). ----
//
// RainViewer serves the world's composited radar as Web-Mercator PNG
// tiles, a new frame every ~10 minutes. Each cycle here picks a zoom so
// the visible disc fits in at most 2x2 tiles, fetches them, decodes the
// palette back into reflectivity, and publishes the result as one
// georeferenced pixel mosaic (model.weather) for the renderer to sample.
// Everything network- and decode-heavy runs with no lock held.

// Fetch tuning. Tiles are 256 px; RainViewer stops at zoom 7. A view
// change refetches only once the view has settled on a different tile
// cover, and failed cycles back off instead of hammering.
constexpr int kWxTileSize = 256;
constexpr int kWxMaxZoom = 7;   // RainViewer's documented tile maximum.
constexpr int kWxMinZoom = 2;
constexpr float kWxCoverMargin = 1.15f;  // Fetch a hair past the disc.
constexpr uint32_t kWxSettleMs = 3000;
// Backoff after a failed tile transfer: short, so a scrub recovers quickly
// (the cache fetches one frame per poll, and each attempt is one transfer).
constexpr uint32_t kWxRetryMs = 4000;
constexpr size_t kWxMaxPngBytes = 192 * 1024;  // Sanity cap per tile.

// The palette RainViewer actually serves. The tile URL's {color}
// parameter is accepted but ignored these days — every scheme returns
// byte-identical tiles in the documented "Universal Blue" palette
// (rainviewer.com/api/color-schemes.html, CSV table) — and unsmoothed
// tiles ("0_0" options) contain only exact palette entries, so
// reflectivity is recovered by direct color match. `value` stores
// dBZ + 32 (0 = no echo); the colors the table repeats (75+ dBZ) resolve
// to the first hit, all deep inside the top intensity tier.
struct WxColor {
  uint32_t rgba;  // 0xRRGGBBAA.
  uint8_t value;  // dBZ + 32.
};
constexpr WxColor kWxPalette[] = {
    {0x63615914, 22}, {0x66635a19, 23}, {0x69665c1e, 24}, {0x6c685d24, 25},
    {0x6f6b5f29, 26}, {0x726e612e, 27}, {0x75706234, 28}, {0x78736439, 29},
    {0x7c75653e, 30}, {0x7f786744, 31}, {0x827b6949, 32}, {0x857d6a4e, 33},
    {0x88806c54, 34}, {0x8b826d59, 35}, {0x8e856f5e, 36}, {0x92887164, 37},
    {0x9e93756e, 38}, {0xaa9e7978, 39}, {0xb6a97e82, 40}, {0xc2b4828c, 41},
    {0xcec08796, 42}, {0xd2c48ba0, 43}, {0xd6c88faa, 44}, {0xdacc93b4, 45},
    {0xded097be, 46}, {0x88ddeeff, 47}, {0x6cd1ebff, 48}, {0x51c5e8ff, 49},
    {0x36bae5ff, 50}, {0x1baee2ff, 51}, {0x00a3e0ff, 52}, {0x009ad5ff, 53},
    {0x0091caff, 54}, {0x0088bfff, 55}, {0x007fb4ff, 56}, {0x0077aaff, 57},
    {0x0070a3ff, 58}, {0x00699cff, 59}, {0x006295ff, 60}, {0x005b8eff, 61},
    {0x005588ff, 62}, {0x005180ff, 63}, {0x004e78ff, 64}, {0x004a70ff, 65},
    {0x004768ff, 66}, {0xffee00ff, 67}, {0xffe000ff, 68}, {0xffd200ff, 69},
    {0xffc500ff, 70}, {0xffb700ff, 71}, {0xffaa00ff, 72}, {0xff9f00ff, 73},
    {0xff9500ff, 74}, {0xff8b00ff, 75}, {0xff8100ff, 76}, {0xff4400ff, 77},
    {0xf23600ff, 78}, {0xe62800ff, 79}, {0xd91b00ff, 80}, {0xcd0d00ff, 81},
    {0xc10000ff, 82}, {0xa80000ff, 83}, {0x8f0000ff, 84}, {0x760000ff, 85},
    {0x5d0000ff, 86}, {0xffaaffff, 87}, {0xff9fffff, 88}, {0xff95ffff, 89},
    {0xff8bffff, 90}, {0xff81ffff, 91}, {0xff77ffff, 92}, {0xff6cffff, 93},
    {0xff62ffff, 94}, {0xff58ffff, 95}, {0xff4effff, 96}, {0xffffffff, 97},
    {0x00ff00ff, 107},
};

// The 1-4 tiles covering the visible disc: top-left tile index and count
// per axis at a Web-Mercator zoom.
struct WxCover {
  uint8_t z = 0;
  int32_t x0 = 0, y0 = 0;
  int8_t nx = 0, ny = 0;
};

bool wxCoverEq(const WxCover& a, const WxCover& b) {
  return a.z == b.z && a.x0 == b.x0 && a.y0 == b.y0 && a.nx == b.nx &&
         a.ny == b.ny;
}

// Continuous Web-Mercator tile coordinates at zoom z.
float wxTileX(float lonDeg, int z) {
  return (lonDeg + 180.0f) / 360.0f * static_cast<float>(1 << z);
}
float wxTileY(float latDeg, int z) {
  float lat = max(-85.0f, min(85.0f, latDeg));
  float merc = asinhf(tanf(lat * DEG_TO_RAD));
  return (1.0f - merc / static_cast<float>(M_PI)) * 0.5f *
         static_cast<float>(1 << z);
}

// The deepest zoom whose 2x2 tiles still cover the disc of `rangeNm`
// around the view — deepest first, so the mosaic carries the most detail
// the cover allows.
WxCover wxCoverFor(float lat, float lon, float rangeNm) {
  float radius = rangeNm * kWxCoverMargin;
  float latSpan = radius / 60.0f;
  float lonSpan = radius / (60.0f * cosf(lat * DEG_TO_RAD));
  WxCover cover;
  for (int z = kWxMaxZoom; z >= kWxMinZoom; --z) {
    int32_t x0 = static_cast<int32_t>(floorf(wxTileX(lon - lonSpan, z)));
    int32_t x1 = static_cast<int32_t>(floorf(wxTileX(lon + lonSpan, z)));
    int32_t y0 = static_cast<int32_t>(floorf(wxTileY(lat + latSpan, z)));
    int32_t y1 = static_cast<int32_t>(floorf(wxTileY(lat - latSpan, z)));
    cover.z = static_cast<uint8_t>(z);
    cover.x0 = x0;
    cover.y0 = y0;
    cover.nx = static_cast<int8_t>(x1 - x0 >= 1 ? 2 : 1);
    cover.ny = static_cast<int8_t>(y1 - y0 >= 1 ? 2 : 1);
    if (x1 - x0 <= 1 && y1 - y0 <= 1) break;  // Fits; z below only loses detail.
  }
  // Keep the tile indices legal. Clamping only matters at latitudes no
  // home is anywhere near; longitude wrap is likewise ignored.
  int32_t n = 1 << cover.z;
  cover.x0 = max<int32_t>(0, min<int32_t>(cover.x0, n - cover.nx));
  cover.y0 = max<int32_t>(0, min<int32_t>(cover.y0, n - cover.ny));
  return cover;
}

// Decode state threaded through the PNGdec line callback: where in the
// mosaic this tile lands, plus a one-entry memo — radar images are long
// runs of identical pixels, so most lookups hit it.
struct WxDecodeCtx {
  uint8_t* mosaic = nullptr;
  int mosaicW = 0;
  int offX = 0, offY = 0;
  uint32_t lit = 0;      // Pixels with any echo, for the log line.
  uint32_t unknown = 0;  // Colors that missed the palette table.
  uint32_t memoRgba = 0;
  uint8_t memoVal = 0;
  bool haveMemo = false;
};

// Coarse fallback for a color that is not an exact palette entry (a
// future palette tweak, or edge smoothing if it ever gets enabled). The
// bands mirror the served palette: translucent warm gray is the drizzle
// ramp, opaque blues rain, warm hues heavy rain.
uint8_t wxClassify(uint32_t rgba) {
  uint8_t r = rgba >> 24, g = rgba >> 16, b = rgba >> 8, a = rgba;
  if (a < 8) return 0;
  if (r + g + b < 30) return 0;       // Effectively black: no echo.
  if (a < 250) return 24 + (a >> 3);  // Drizzle, graded by its alpha ramp.
  if (b > r) return 58;               // Blue family: ~26 dBZ.
  if (g > 100 && b < 100) return 70;  // Amber family: ~38 dBZ.
  return 80;                          // Red and beyond: ~48 dBZ.
}

uint8_t wxValueFor(uint32_t rgba, WxDecodeCtx& ctx) {
  if (ctx.haveMemo && rgba == ctx.memoRgba) return ctx.memoVal;
  uint8_t value = 0;
  bool found = false;
  for (const WxColor& entry : kWxPalette) {
    if (entry.rgba == rgba) {
      value = entry.value;
      found = true;
      break;
    }
  }
  if (!found) {
    ++ctx.unknown;
    value = wxClassify(rgba);
  }
  ctx.memoRgba = rgba;
  ctx.memoVal = value;
  ctx.haveMemo = true;
  return value;
}

// One pixel of a decoded line as 0xRRGGBBAA. RainViewer serves 8-bit
// RGBA tiles today; the palette and grayscale variants are handled so a
// server-side re-encode degrades into a parse, not into garbage.
uint32_t wxPixelRgba(const PNGDRAW* pDraw, int x) {
  const uint8_t* p = pDraw->pPixels;
  switch (pDraw->iPixelType) {
    case PNG_PIXEL_TRUECOLOR_ALPHA:
      p += x * 4;
      return static_cast<uint32_t>(p[0]) << 24 |
             static_cast<uint32_t>(p[1]) << 16 | p[2] << 8 | p[3];
    case PNG_PIXEL_TRUECOLOR:
      p += x * 3;
      return static_cast<uint32_t>(p[0]) << 24 |
             static_cast<uint32_t>(p[1]) << 16 | p[2] << 8 | 0xff;
    case PNG_PIXEL_INDEXED: {
      int index;
      switch (pDraw->iBpp) {
        case 8: index = p[x]; break;
        case 4: index = (p[x / 2] >> ((x & 1) ? 0 : 4)) & 0xf; break;
        case 2: index = (p[x / 4] >> (6 - 2 * (x & 3))) & 0x3; break;
        default: index = (p[x / 8] >> (7 - (x & 7))) & 0x1; break;
      }
      const uint8_t* rgb = pDraw->pPalette + index * 3;
      // PNGdec appends the tRNS alpha palette at offset 768 when present.
      uint8_t alpha = pDraw->iHasAlpha ? pDraw->pPalette[768 + index] : 0xff;
      return static_cast<uint32_t>(rgb[0]) << 24 |
             static_cast<uint32_t>(rgb[1]) << 16 | rgb[2] << 8 | alpha;
    }
    case PNG_PIXEL_GRAY_ALPHA:
      p += x * 2;
      return static_cast<uint32_t>(p[0]) << 24 |
             static_cast<uint32_t>(p[0]) << 16 | p[0] << 8 | p[1];
    case PNG_PIXEL_GRAYSCALE:
      if (pDraw->iBpp != 8) return 0;
      return static_cast<uint32_t>(p[x]) << 24 |
             static_cast<uint32_t>(p[x]) << 16 | p[x] << 8 | 0xff;
    default:
      return 0;
  }
}

// PNGdec line callback: converts one tile row to reflectivity values in
// place in the mosaic. Returns nonzero so the decode continues.
int wxPngLine(PNGDRAW* pDraw) {
  auto* ctx = static_cast<WxDecodeCtx*>(pDraw->pUser);
  uint8_t* row = ctx->mosaic +
                 static_cast<size_t>(ctx->offY + pDraw->y) * ctx->mosaicW +
                 ctx->offX;
  for (int x = 0; x < pDraw->iWidth; ++x) {
    uint32_t rgba = wxPixelRgba(pDraw, x);
    uint8_t value = 0;
    if ((rgba & 0xffu) != 0) {  // Transparent = no echo, the common case.
      value = wxValueFor(rgba, *ctx);
      if (value != 0) ++ctx->lit;
    }
    row[x] = value;
  }
  return 1;
}

// One selectable RainViewer frame: its wall time and the host+path base the
// tile URLs hang off. The frame list runs oldest-past .. newest-past, then
// any nowcast; g_wxNowIndex marks the newest past ("now").
struct WxFrame {
  time_t time;
  String base;
};

// Weather fetch state, only ever touched from the network task.
std::vector<WxFrame> g_wxFrames;  // Selectable frames (past then nowcast).
int g_wxNowIndex = -1;      // Index of the newest past frame ("now").
uint32_t g_wxIndexMs = 0;   // When the index was last fetched (0 = never).
uint32_t g_wxLastTryMs = 0; // Last attempt that did not publish (backoff).
time_t g_wxLoadedTime = 0;  // Wall time of the frame the published layer holds.
PNG* g_wxPng = nullptr;     // PNGdec instance; ~48 KB, so it lives in PSRAM.
WxCover g_wxWant;           // Last desired cover, for the settle debounce.
uint32_t g_wxWantMs = 0;

// How many past frames to retain for scrubbing: the knob rewinds at most an
// hour (WEATHER_MAX_BACK_MIN), so eight 10-minute frames is comfortably
// enough; keeping the list short bounds the String memory.
constexpr size_t kWxKeepPast = 8;

// Fetches the RainViewer frame index and rebuilds the selectable-frame list:
// the recent past frames (capped) plus any nowcast. The index is a couple of
// hundred bytes of JSON, so the String round-trip through httpGet is fine.
// Cached for WEATHER_POLL_MS, so calling it every check is a cheap early
// return between refreshes.
bool refreshWxIndex() {
  if (!g_wxFrames.empty() &&
      millis() - g_wxIndexMs < config::WEATHER_POLL_MS) {
    return true;
  }
  String body;
  if (!httpGet("https://api.rainviewer.com/public/weather-maps.json", body)) {
    rlog::Log.println("[feeds] wx: index fetch failed.");
    // A previously fetched frame set stays valid for a while; reuse it.
    return !g_wxFrames.empty();
  }
  static SpiRamAllocator allocator;
  JsonDocument doc(&allocator);
  if (deserializeJson(doc, body) != DeserializationError::Ok) return false;
  const char* host = doc["host"];
  JsonArrayConst past = doc["radar"]["past"].as<JsonArrayConst>();
  if (host == nullptr || past.size() == 0) {
    rlog::Log.println("[feeds] wx: index has no radar frames.");
    return false;
  }
  std::vector<WxFrame> frames;
  size_t start = past.size() > kWxKeepPast ? past.size() - kWxKeepPast : 0;
  for (size_t i = start; i < past.size(); ++i) {
    time_t t = past[i]["time"].as<long>();
    const char* p = past[i]["path"];
    if (p == nullptr || t == 0) continue;
    frames.push_back({t, String(host) + p});
  }
  if (frames.empty()) return false;
  int nowIdx = static_cast<int>(frames.size()) - 1;  // Newest past = "now".
  JsonArrayConst nowcast = doc["radar"]["nowcast"].as<JsonArrayConst>();
  for (JsonVariantConst f : nowcast) {
    time_t t = f["time"].as<long>();
    const char* p = f["path"];
    if (p == nullptr || t == 0) continue;
    frames.push_back({t, String(host) + p});
  }
  g_wxFrames = std::move(frames);
  g_wxNowIndex = nowIdx;
  g_wxIndexMs = millis();
  return true;
}

// Fetches one tile PNG into `pngBuf` and decodes it into the mosaic at
// (offX, offY). Modeled on fetchTraffic: bulk reads through the yielding
// reader, and no lock held anywhere near this.
bool wxFetchTile(HTTPClient& http, WiFiClientSecure& client, const String& url,
                 uint8_t* pngBuf, WxDecodeCtx& ctx, int offX, int offY) {
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    rlog::Log.printf("[feeds] wx: HTTP %d for tile.\n", code);
    http.end();
    return false;
  }
  // The tile cache always announces a length; without one the body would
  // have to be read to connection close, which a keep-alive socket turns
  // into a full timeout. Fail fast instead.
  int announced = http.getSize();
  if (announced <= 0 || announced > static_cast<int>(kWxMaxPngBytes)) {
    rlog::Log.printf("[feeds] wx: %d-byte tile refused.\n", announced);
    http.end();
    return false;
  }
  YieldingReader reader(http.getStream());
  size_t len = reader.readBytes(reinterpret_cast<char*>(pngBuf),
                                static_cast<size_t>(announced));
  http.end();
  if (len != static_cast<size_t>(announced)) {
    rlog::Log.println("[feeds] wx: short tile body.");
    return false;
  }

  ctx.offX = offX;
  ctx.offY = offY;
  if (g_wxPng->openRAM(pngBuf, static_cast<int>(len), wxPngLine) !=
      PNG_SUCCESS) {
    rlog::Log.println("[feeds] wx: tile is not a PNG.");
    return false;
  }
  if (g_wxPng->getWidth() != kWxTileSize ||
      g_wxPng->getHeight() != kWxTileSize) {
    rlog::Log.printf("[feeds] wx: unexpected %dx%d tile.\n", g_wxPng->getWidth(),
                  g_wxPng->getHeight());
    g_wxPng->close();
    return false;
  }
  int rc = g_wxPng->decode(&ctx, 0);
  g_wxPng->close();
  if (rc != PNG_SUCCESS) {
    rlog::Log.printf("[feeds] wx: PNG decode failed (%d).\n", rc);
    return false;
  }
  return true;
}

// ---- Followed-flight historic trail (adsb.lol traces). ----
//
// adsb.lol publishes a per-aircraft trace file: every position it saw in
// the current UTC day. When a flight is followed, pollTrace fetches that
// file once, decimates it, and seeds model.ui.followTrail with it, so
// the dotted trail reaches back along the flight's real path instead of
// starting where the follow began. From there the renderer owns the
// trail: it appends its live samples behind the seed, and its cap holds
// the seeded prefix plus a long live tail.

// Trace tuning. The file arrives gzip-compressed no matter what the
// request advertises (~7 KB wire, ~40 KB inflated for a typical day), so
// it is inflated on-device with the ROM's tinfl. The size caps refuse
// anything wildly bigger than a plausible full-day trace; the seed point
// count bounds the renderer's per-frame trail walk and the seed publish.
constexpr size_t kTraceSeedPoints = 80;    // History points seeded per follow.
constexpr uint32_t kTraceCheckMs = 1000;   // Follow-target check cadence.
constexpr uint32_t kTraceRetryMs = 60000;  // Refetch backoff after a failure.
constexpr size_t kTraceMaxWireBytes = 1024 * 1024;
constexpr size_t kTraceMaxJsonBytes = 2 * 1024 * 1024;

// Trace state, only ever touched from the network task.
String g_traceHex;          // Hex the published history belongs to; "" none.
String g_traceFailHex;      // Last hex whose fetch failed, for the backoff.
uint32_t g_traceFailMs = 0;
uint32_t g_traceCheckMs = 0;

// ---- Followed-flight destination (adsbdb route). ----
//
// adsbdb answers a per-callsign route lookup with the flight's origin and
// destination airports. When a flight is followed, pollDest looks that up
// once and publishes the destination's projected world position as
// model.ui.followDest, so the renderer can draw the expected onward path
// to it. The route never changes mid-flight, so it is fetched once per
// followed callsign and reused until the follow changes.

// Dest tuning. The lookup is a small JSON body, so it round-trips through
// httpGet as a String. Attempts are rate-limited, and a callsign adsbdb
// has no route for backs off before being asked again.
constexpr uint32_t kDestCheckMs = 1000;   // Lookup-attempt cadence.
constexpr uint32_t kDestRetryMs = 60000;  // Refetch backoff after a failure.

// Dest state, only ever touched from the network task.
String g_destCallsign;      // Callsign the published destination belongs to.
String g_destFailCallsign;  // Last callsign whose lookup failed, for backoff.
uint32_t g_destFailMs = 0;
uint32_t g_destCheckMs = 0;

// True for a plain 6-character lower-case ICAO address. TIS-B and
// anonymized targets ("~" prefix) have no trace file worth asking for.
bool isIcaoHex(const String& hex) {
  if (hex.length() != 6) return false;
  for (int i = 0; i < 6; ++i) {
    char c = hex[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

// Unwraps a gzip (RFC 1952) member and inflates it with the tinfl
// decompressor in the S3's mask ROM — the same inflate the ROM loader
// uses for compressed flashing, so nothing new links in. Returns a PSRAM
// buffer of `outLen` bytes the caller frees, or nullptr on any framing,
// size or inflate failure. The trailer's ISIZE sizes the output exactly;
// the CRC is not rechecked — the JSON parse behind this catches
// corruption just as surely.
uint8_t* gunzip(const uint8_t* in, size_t inLen, size_t& outLen) {
  // Fixed header: magic, method 8 (deflate), flags, mtime, xfl, os.
  if (inLen < 18 || in[0] != 0x1f || in[1] != 0x8b || in[2] != 8) {
    return nullptr;
  }
  uint8_t flags = in[3];
  size_t pos = 10;
  if (flags & 0x04) {  // FEXTRA: 2-byte little-endian length, then data.
    if (pos + 2 > inLen) return nullptr;
    pos += 2 + (static_cast<size_t>(in[pos]) |
                static_cast<size_t>(in[pos + 1]) << 8);
  }
  if (flags & 0x08) {  // FNAME, NUL-terminated.
    while (pos < inLen && in[pos] != 0) ++pos;
    ++pos;
  }
  if (flags & 0x10) {  // FCOMMENT, NUL-terminated.
    while (pos < inLen && in[pos] != 0) ++pos;
    ++pos;
  }
  if (flags & 0x02) pos += 2;  // FHCRC.
  // Past the header there must be deflate data plus the 8-byte trailer.
  if (pos + 8 >= inLen) return nullptr;

  size_t isize = static_cast<size_t>(in[inLen - 4]) |
                 static_cast<size_t>(in[inLen - 3]) << 8 |
                 static_cast<size_t>(in[inLen - 2]) << 16 |
                 static_cast<size_t>(in[inLen - 1]) << 24;
  if (isize == 0 || isize > kTraceMaxJsonBytes) return nullptr;

  uint8_t* out = static_cast<uint8_t*>(
      heap_caps_malloc(isize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  auto* inflator = static_cast<tinfl_decompressor*>(heap_caps_malloc(
      sizeof(tinfl_decompressor), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (out == nullptr || inflator == nullptr) {
    heap_caps_free(out);
    heap_caps_free(inflator);
    return nullptr;
  }
  tinfl_init(inflator);
  size_t inBytes = inLen - 8 - pos;
  size_t outBytes = isize;
  // One whole-buffer call: all input is present and the output is sized
  // to fit, so no streaming loop is needed.
  tinfl_status status =
      tinfl_decompress(inflator, in + pos, &inBytes, out, out, &outBytes,
                       TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
  heap_caps_free(inflator);
  if (status != TINFL_STATUS_DONE || outBytes != isize) {
    heap_caps_free(out);
    return nullptr;
  }
  outLen = outBytes;
  return out;
}

// Reads the whole (compressed) response body into PSRAM, growing as it
// goes — HTTP/1.0 keeps the stream unframed, but the server does not
// always announce a length. Returns nullptr past the size cap or on OOM.
uint8_t* traceReadBody(HTTPClient& http, size_t& len) {
  YieldingReader reader(http.getStream());
  size_t cap = 32 * 1024;
  uint8_t* buf = static_cast<uint8_t*>(
      heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (buf == nullptr) return nullptr;
  len = 0;
  for (;;) {
    if (len == cap) {
      if (cap >= kTraceMaxWireBytes) {
        heap_caps_free(buf);
        return nullptr;
      }
      cap = min(cap * 2, kTraceMaxWireBytes);
      uint8_t* grown = static_cast<uint8_t*>(
          heap_caps_realloc(buf, cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (grown == nullptr) {
        heap_caps_free(buf);
        return nullptr;
      }
      buf = grown;
    }
    size_t n = reader.readBytes(reinterpret_cast<char*>(buf) + len, cap - len);
    if (n == 0) break;  // End of body (or a stall; the parse decides).
    len += n;
  }
  return buf;
}

// Skips ASCII whitespace. The scanner below never leaves the buffer.
const char* skipWs(const char* p, const char* end) {
  while (p < end &&
         (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')) {
    ++p;
  }
  return p;
}

// Scans the trace JSON for the "trace" array and hands every plottable
// point's latitude, longitude (fields [1] and [2]; either may be null)
// and on-ground flag (field [3] is the literal string "ground" on the
// surface, a number aloft) to `fn(lat, lon, onGround)`, oldest first.
// A hand scanner
// instead of a real parse: a long-haul day runs to thousands of points,
// and holding them as an ArduinoJson document costs megabytes of PSRAM
// the set does not have (observed on-device: NoMemory at ~870 KB of
// trace JSON with 3.2 MB free — the reason a followed flight almost
// never showed its history). The scan holds nothing but its cursor;
// each point's remaining fields — nested objects, strings, escapes
// included — are skipped by depth.
template <typename PointFn>
void scanTracePoints(const char* json, size_t len, PointFn&& fn) {
  const char* end = json + len;

  // Locate the array: the quoted "trace" key, then ':', then '['. A
  // string VALUE containing the needle fails those two checks and the
  // search moves on.
  const char* p = nullptr;
  for (const char* s = json; s + 7 <= end - 1; ++s) {
    if (memcmp(s, "\"trace\"", 7) != 0) continue;
    const char* q = skipWs(s + 7, end);
    if (q < end && *q == ':') {
      q = skipWs(q + 1, end);
      if (q < end && *q == '[') {
        p = q + 1;
        break;
      }
    }
  }
  if (p == nullptr) return;

  while (p < end) {
    p = skipWs(p, end);
    if (p >= end || *p == ']') return;  // End of the trace array.
    if (*p == ',') {
      ++p;
      continue;
    }
    if (*p != '[') return;  // Not a point array: malformed, stop.
    ++p;

    // Fields [0..2] are scalars — timestamp, lat, lon (a missing fix
    // leaves the coordinates null) — and [3] is the altitude, the
    // literal string "ground" on the surface or a number aloft. Each
    // token is copied out and parsed bounded, so a number at the very
    // end of the buffer can never run the parse past it.
    float vals[3] = {0, 0, 0};
    bool numeric[3] = {false, false, false};
    bool onGround = false;
    int fields = 0;
    for (int f = 0; f < 4; ++f) {
      p = skipWs(p, end);
      const char* tok = p;
      while (p < end && *p != ',' && *p != ']') ++p;
      char buf[24];
      size_t n = min(static_cast<size_t>(p - tok), sizeof(buf) - 1);
      memcpy(buf, tok, n);
      buf[n] = 0;
      if (f < 3) {
        char* after = nullptr;
        vals[f] = strtof(buf, &after);
        numeric[f] = after != buf;
      } else {
        onGround = strstr(buf, "ground") != nullptr;
      }
      ++fields;
      if (p >= end || *p == ']') break;  // Short point.
      ++p;                               // Past the ','.
    }

    // Skip the remainder of the point (any nesting, strings with
    // escapes); the walk starts inside the point's '[', so a short
    // point's ']' closes it immediately.
    int depth = 1;
    bool inStr = false;
    while (p < end && depth > 0) {
      char ch = *p++;
      if (inStr) {
        if (ch == '\\' && p < end) {
          ++p;
        } else if (ch == '"') {
          inStr = false;
        }
      } else if (ch == '"') {
        inStr = true;
      } else if (ch == '[' || ch == '{') {
        ++depth;
      } else if (ch == ']' || ch == '}') {
        --depth;
      }
    }

    if (fields < 3 || !numeric[1] || !numeric[2]) continue;
    float lat = vals[1], lon = vals[2];
    if (fabsf(lat) > 90.0f || fabsf(lon) > 180.0f) continue;
    fn(lat, lon, onGround);
  }
}

// Fetches `hex`'s full-day trace from adsb.lol and decimates it into at
// most kTraceSeedPoints world positions, oldest first, with NO lock
// held. The globe front end 302-redirects to the bare host; the body is
// gzip regardless of what the request accepts, so it is read whole into
// PSRAM, inflated there, and then scanned in place (see
// scanTracePoints). Returns false on any fetch, size or scan problem —
// the caller then leaves the live-only trail alone.
bool fetchTrace(const model::Home& home, const String& hex,
                std::vector<model::Vec>& out) {
  if (!WiFi.isConnected()) return false;
  // Same guard as the traffic path: the TLS session lives on the
  // internal heap, the wire and inflated trace buffers in PSRAM.
  if (ESP.getFreeHeap() < 40 * 1024 ||
      heap_caps_get_free_size(MALLOC_CAP_SPIRAM) <
          kTraceMaxWireBytes + 512 * 1024) {
    rlog::Log.printf(
        "[feeds] trace: memory low (heap %u KB, PSRAM %u KB); skipping.\n",
        ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024);
    return false;
  }

  // The shard directory is the LAST two characters of the hex.
  String url = "https://globe.adsb.lol/data/traces/" + hex.substring(4) +
               "/trace_full_" + hex + ".json";

  uint32_t started = millis();
  WiFiClientSecure client;
  // TODO(security): install a CA bundle and validate certificates.
  client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(kHttpTimeoutMs);
  http.setTimeout(kHttpTimeoutMs);
  http.setUserAgent(kUserAgent);
  http.useHTTP10(true);  // Unframed body, directly readable (as adsb).
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    // 404 is simply a flight adsb.lol has no trace for.
    rlog::Log.printf("[feeds] trace: HTTP %d for %s.\n", code, hex.c_str());
    http.end();
    return false;
  }
  int announced = http.getSize();
  if (announced > static_cast<int>(kTraceMaxWireBytes)) {
    rlog::Log.printf("[feeds] trace: %d-byte body refused.\n", announced);
    http.end();
    return false;
  }
  size_t wire = 0;
  uint8_t* raw = traceReadBody(http, wire);
  http.end();
  if (raw == nullptr || wire == 0) {
    heap_caps_free(raw);
    rlog::Log.println("[feeds] trace: body read failed.");
    return false;
  }

  // Inflate — unless the server ever starts honoring identity encoding,
  // in which case the body already opens as JSON and scans as-is. The
  // wire copy is dropped as soon as it is decoded, ahead of the scan.
  uint8_t* json = raw;
  size_t jsonLen = wire;
  if (raw[0] == 0x1f && wire >= 2 && raw[1] == 0x8b) {
    json = gunzip(raw, wire, jsonLen);
    heap_caps_free(raw);
    if (json == nullptr) {
      rlog::Log.println("[feeds] trace: gunzip failed.");
      return false;
    }
  }

  // Two passes over the text, so the full-day path never lands anywhere
  // whole: first count the plottable points and note the most recent one
  // on the ground, then keep an even spread of at most kTraceSeedPoints
  // from the last takeoff onward, oldest first, the newest always
  // included so the history meets the live position.
  const char* text = reinterpret_cast<const char*>(json);
  size_t valid = 0;
  long lastGround = -1;  // Ordinal of the most recent on-ground point.
  scanTracePoints(text, jsonLen, [&](float, float, bool onGround) {
    if (onGround) lastGround = static_cast<long>(valid);
    ++valid;
  });
  if (valid == 0) {
    heap_caps_free(json);
    rlog::Log.printf("[feeds] trace: no plottable points for %s.\n", hex.c_str());
    return false;
  }

  // Trim to the current leg: start at the last takeoff (the most recent
  // on-ground point), so a plane that flew several legs today shows only
  // the one it is on. A trace with no ground point at all — already aloft
  // when the day's trace began, e.g. an overnight long-haul — keeps all
  // of it.
  size_t start = lastGround >= 0 ? static_cast<size_t>(lastGround) : 0;
  size_t leg = valid - start;
  size_t kept = min(leg, kTraceSeedPoints);
  out.clear();
  out.reserve(kept);
  size_t index = 0, emitted = 0;
  scanTracePoints(text, jsonLen, [&](float lat, float lon, bool) {
    size_t ord = index++;
    if (ord < start) return;  // Before the last takeoff.
    size_t si = ord - start;  // Index within the current leg.
    size_t want = kept == 1 ? leg - 1 : emitted * (leg - 1) / (kept - 1);
    if (si != want) return;
    out.push_back(geoToWorld(home, lat, lon));
    ++emitted;
  });
  heap_caps_free(json);

  rlog::Log.printf(
      "[feeds] trace: %s %u B wire -> %u B json, %u pts (%u since takeoff) "
      "-> %u kept (%lu ms, heap %u KB)\n",
      hex.c_str(), static_cast<unsigned>(wire), static_cast<unsigned>(jsonLen),
      static_cast<unsigned>(valid), static_cast<unsigned>(leg),
      static_cast<unsigned>(emitted),
      static_cast<unsigned long>(millis() - started), ESP.getFreeHeap() / 1024);
  return true;
}

// Looks up `callsign`'s route on adsbdb and projects its DESTINATION
// airport around the active home, with NO lock held. The body is a few
// hundred bytes, so the String round-trip through httpGet is fine.
// Returns false when adsbdb does not know the callsign — an "unknown
// callsign" answers with a response STRING in place of the route object,
// so the destination lookups below simply resolve to null and fail — the
// caller then draws no onward path. api.adsbdb.com needs no auth; TLS is
// setInsecure() like the other feeds.
bool fetchDest(const model::Home& home, const String& callsign,
               model::Vec& out) {
  if (!WiFi.isConnected()) return false;
  String body;
  if (!httpGet("https://api.adsbdb.com/v0/callsign/" + callsign, body)) {
    return false;
  }
  static SpiRamAllocator allocator;
  JsonDocument doc(&allocator);
  if (deserializeJson(doc, body) != DeserializationError::Ok) return false;
  JsonVariantConst dest = doc["response"]["flightroute"]["destination"];
  JsonVariantConst latV = dest["latitude"];
  JsonVariantConst lonV = dest["longitude"];
  if (latV.isNull() || lonV.isNull()) return false;
  // as<float>() parses a numeric string too, in case the API ever quotes
  // the coordinates; range-check guards a missing field that read as 0.
  float lat = latV.as<float>();
  float lon = lonV.as<float>();
  if (fabsf(lat) > 90.0f || fabsf(lon) > 180.0f) return false;
  out = geoToWorld(home, lat, lon);
  return true;
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

  // Follow up on the active special callsigns the circle did not
  // deliver: a special flight can be airborne anywhere on Earth, so the
  // absent forms are looked up globally and any match merged in beside
  // the radius traffic (a flight-number entry expands to several
  // candidate forms, most of which are legitimately never airborne).
  // Forms past the lookup cap — and lookups that failed — land in
  // `held`, which below keeps a previously merged aircraft on the scope
  // with its last data instead of blinking it off for a poll.
  std::set<String> have;
  for (const auto& parsed : snapshot) have.insert(parsed.callsign);
  std::vector<String> wanted;
  std::set<String> held;
  for (const String& callsign : g_specials) {
    if (have.count(callsign) > 0) continue;
    if (wanted.size() < kMaxGlobalLookups) {
      wanted.push_back(callsign);
    } else {
      held.insert(callsign);
    }
  }
  if (!wanted.empty()) {
    std::vector<Parsed> global;
    fetchGlobalSpecials(home, wanted, global, held);
    size_t merged = 0;
    for (auto& parsed : global) {
      // Dedupe by broadcast callsign, so a special that was in the
      // radius data after all (or answers two candidate forms) is
      // never doubled.
      if (have.insert(parsed.callsign).second) {
        snapshot.push_back(std::move(parsed));
        ++merged;
      }
    }
    rlog::Log.printf(
        "[feeds] adsb: %u global special lookup(s) -> %u merged, %u held.\n",
        static_cast<unsigned>(wanted.size()), static_cast<unsigned>(merged),
        static_cast<unsigned>(held.size()));
  }

  // Merge under the mutex, briefly: upsert so each blip's sweep-refresh
  // state carries across polls, then drop aircraft that left the feed.
  xSemaphoreTake(mutex, portMAX_DELAY);

  // The home may have been switched while the fetch was in flight; these
  // positions are relative to the old one, so drop them — the switch
  // already requested a fresh poll around the new home.
  if (activeHome(model).name != home.name) {
    xSemaphoreGive(mutex);
    rlog::Log.println("[feeds] adsb: home changed mid-fetch; snapshot dropped.");
    return false;
  }

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
    bool fresh = ac.fixMs == 0;  // First fix for this aircraft.
    ac.hex = parsed.hex;
    ac.pos = parsed.pos;
    ac.track = parsed.track;
    ac.groundSpeed = parsed.groundSpeed;
    ac.altitude = parsed.altitude;
    ac.special = parsed.special;
    ac.fixMs = millis();
    // Seed the smoothed display position; step() eases it from wherever
    // it was through later fixes' corrections.
    if (fresh) ac.est = ac.pos;
    present.insert(parsed.callsign);
  }
  // Keep any aircraft a held form covers: its global lookup failed or
  // was deferred this poll, which says nothing about the flight itself.
  for (const auto& ac : model.aircraft) {
    if (held.count(ac.callsign) > 0) present.insert(ac.callsign);
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
  // A poll has now landed: the renderer can drop the acquiring-signal
  // screen for the live scope (even if the sky happens to be empty), and
  // refresh the focus "top 10" against the new aircraft set.
  model.adsbLoaded = true;
  ++model.adsbGen;
  xSemaphoreGive(mutex);

  rlog::Log.printf("[feeds] adsb: %d aircraft (%u in range, %lu ms, heap %u KB)\n",
                count, static_cast<unsigned>(inRange),
                static_cast<unsigned long>(millis() - started),
                ESP.getFreeHeap() / 1024);
  return true;
}

// Today's date in UTC as YYYYMMDD, or "" until NTP has set the clock. Special
// flights (manual and iCal) are matched only on their own date, so with no
// clock yet nothing is flagged rather than risk a wrong-day highlight.
String todayUtc() {
  time_t t = time(nullptr);
  if (t < 1600000000L) return "";  // Before 2020-09: clock not NTP-synced yet.
  struct tm utc;
  gmtime_r(&t, &utc);
  char buf[9];
  strftime(buf, sizeof(buf), "%Y%m%d", &utc);
  return String(buf);
}

// Strips separators so "2026-07-25" and "20260725" compare equal.
String digitsOnly(const String& s) {
  String out;
  for (int i = 0; i < static_cast<int>(s.length()); ++i) {
    if (isdigit(s[i])) out += s[i];
  }
  return out;
}

// The date of an iCal VEVENT from its DTSTART (YYYYMMDD), or "" if absent.
// Handles DTSTART:..., DTSTART;VALUE=DATE:..., DTSTART;TZID=...: forms — the
// date is the run of digits right after the first ':' following DTSTART.
String eventDate(const String& event) {
  int d = event.indexOf("DTSTART");
  if (d < 0) return "";
  int colon = event.indexOf(':', d);
  if (colon < 0) return "";
  String date;
  for (int i = colon + 1;
       i < static_cast<int>(event.length()) && date.length() < 8; ++i) {
    if (!isdigit(event[i])) break;
    date += event[i];
  }
  return date.length() == 8 ? date : "";
}

// Scans free text for airline-code + number designators (e.g. "BA 117")
// and inserts the ICAO callsign the ADS-B feed broadcasts (e.g. "BAW117")
// into `out`. Each designator newly added is also recorded in `found` as
// it appeared ("BA117"), for the dashboard's from-calendar list. The code
// is a letter plus a letter or digit (U2, W6) at a word boundary, so
// prose running into a designator cannot fake one.
void scanFlights(const String& text, std::set<String>& out,
                 std::vector<String>* found) {
  int n = static_cast<int>(text.length());
  for (int i = 0; i + 2 < n; ++i) {
    // Word boundary (unsigned: iCal text is UTF-8, and a negative char
    // would index the ctype table out of bounds).
    if (i > 0 && isalnum(static_cast<unsigned char>(text[i - 1]))) continue;
    char a = text[i], b = text[i + 1];
    if (a < 'A' || a > 'Z') continue;
    if (!(b >= 'A' && b <= 'Z') && !(b >= '0' && b <= '9')) continue;
    int j = i + 2;
    while (j < n && text[j] == ' ') ++j;
    if (j >= n || !isdigit(text[j])) continue;
    String number;
    while (j < n && isdigit(text[j])) number += text[j++];
    if (number.length() < 1 || number.length() > 4) continue;
    String designator = String(a) + String(b) + number;
    String icao = iataToIcao(String(a) + String(b));
    if (!icao.isEmpty()) out.insert(icao + number);
    // List every flight the feed names, even carriers not in the table --
    // they surface as "not trackable" rather than silently vanishing.
    if (found != nullptr &&
        std::find(found->begin(), found->end(), designator) == found->end()) {
      found->push_back(designator);
    }
  }
}

// Adds the callsign forms a flight-number entry could broadcast as: the
// entry itself normalized (uppercased, separators stripped), so a
// directly-typed ICAO callsign like "QTR15K" or "BAW117" matches as-is,
// plus the IATA->ICAO conversion of a two-character airline designator
// ("QR106" -> "QTR106", keeping any suffix letter). Shared by the
// specials matching and the dashboard's live "found" badges.
void candidateCallsigns(const String& flight, std::set<String>& out) {
  String raw;
  for (int i = 0; i < static_cast<int>(flight.length()); ++i) {
    unsigned char c = static_cast<unsigned char>(flight[i]);
    if (isalnum(c)) raw += static_cast<char>(toupper(c));
  }
  if (raw.length() < 3) return;  // Too short for any airline + number.
  out.insert(raw);
  // The IATA form: a two-character designator, then a 1-4 digit number,
  // then at most one suffix letter (anything longer is already ICAO or
  // not a designator at all, and the raw insert above has it covered).
  String rest = raw.substring(2);
  int digits = 0;
  while (digits < static_cast<int>(rest.length()) && isdigit(rest[digits])) {
    ++digits;
  }
  if (digits < 1 || digits > 4) return;
  if (static_cast<int>(rest.length()) - digits > 1) return;
  String icao = iataToIcao(raw.substring(0, 2));
  if (!icao.isEmpty()) out.insert(icao + rest);
}

bool flightTracked(const model::Model& model, const String& flight) {
  std::set<String> forms;
  candidateCallsigns(flight, forms);
  for (const auto& ac : model.aircraft) {
    if (forms.count(ac.callsign) > 0) return true;
  }
  return false;
}

// Whether a flight can be matched to the ICAO callsign ADS-B broadcasts: a
// 3-letter (ICAO) prefix already is one, and a 2-character IATA code only if
// it is in the iataToIcao table. Drives the dashboard's "not trackable"
// marker, so a detected-but-unmatchable flight says so rather than pretending
// it will highlight.
bool flightTrackable(const String& flight) {
  String raw;
  for (int i = 0; i < static_cast<int>(flight.length()); ++i) {
    unsigned char c = static_cast<unsigned char>(flight[i]);
    if (isalnum(c)) raw += static_cast<char>(toupper(c));
  }
  if (raw.length() < 3) return false;
  int letters = 0;
  while (letters < static_cast<int>(raw.length()) && isalpha(raw[letters])) {
    ++letters;
  }
  if (letters >= 3) return true;  // Already an ICAO callsign form.
  if (letters < 1) return false;
  return !iataToIcao(raw.substring(0, 2)).isEmpty();
}

// The SUMMARY line's value from a (line-unfolded) VEVENT, or "". TripIt names
// the flight there ("D83515 LGW to CPH"), so scanning just this keeps booking
// references and other body text out of the flight list.
String summaryOf(const String& event) {
  int s = event.indexOf("SUMMARY");
  if (s < 0) return "";
  int colon = event.indexOf(':', s);
  if (colon < 0) return "";
  int nl = event.indexOf('\n', colon);
  if (nl < 0) nl = static_cast<int>(event.length());
  String value = event.substring(colon + 1, nl);
  value.trim();
  return value;
}

int pollIcal(model::Model& model, SemaphoreHandle_t mutex) {
  // Copy the enabled feeds and the manual specials out under the mutex;
  // the fetches below run with no lock held.
  struct FeedRef {
    String name;
    String url;
  };
  std::vector<FeedRef> enabled;
  std::vector<model::SpecialFlight> manual;
  xSemaphoreTake(mutex, portMAX_DELAY);
  for (const auto& feed : model.feeds) {
    if (feed.enabled) enabled.push_back(FeedRef{feed.name, feed.url});
  }
  manual = model.specials;
  xSemaphoreGive(mutex);

  // Both manual and iCal specials highlight only on their own date, so
  // without the clock (NTP not yet synced) nothing is flagged. The boot
  // poll can outrun the first SNTP sync — and the next interval is an
  // hour out — so while online, wait briefly (no lock held) for the
  // clock rather than run the whole day dateless.
  String today = todayUtc();
  for (int waited = 0; today.isEmpty() && WiFi.isConnected() && waited < 10000;
       waited += 100) {
    vTaskDelay(pdMS_TO_TICKS(100));
    today = todayUtc();
  }
  String todayIso;
  if (!today.isEmpty()) {
    todayIso = today.substring(0, 4) + "-" + today.substring(4, 6) + "-" +
               today.substring(6);
  }

  std::set<String> found;

  // Manually-entered specials due today.
  if (!today.isEmpty()) {
    for (const auto& s : manual) {
      if (digitsOnly(s.date) == today) candidateCallsigns(s.flight, found);
    }
  }

  // iCal feeds: parse VEVENTs and keep every flight from today onward, so
  // an upcoming trip shows in the dashboard the moment it syncs. Only
  // today's flights join the callsign set that highlights the scope;
  // future ones are listed with their date and flag when their day comes.
  std::vector<model::IcalSpecial> fromCalendar;
  std::vector<model::FeedStatus> statuses;
  int fetched = 0;
  for (const auto& feed : enabled) {
    model::FeedStatus status;
    status.url = feed.url;
    status.syncMs = millis();
    time_t wall = time(nullptr);
    status.syncWall = wall < 1600000000L ? 0 : wall;  // 0 until NTP syncs.
    String body;
    status.ok = httpGet(feed.url, body);
    if (status.ok) {
      ++fetched;
      if (!today.isEmpty()) {  // No clock: cannot date-filter safely.
        body.replace("\r\n ", "");  // Unfold folded iCal content lines.
        body.replace("\n ", "");
        size_t before = fromCalendar.size();
        int idx = 0;
        while (true) {
          int evStart = body.indexOf("BEGIN:VEVENT", idx);
          if (evStart < 0) break;
          int evEnd = body.indexOf("END:VEVENT", evStart);
          if (evEnd < 0) evEnd = static_cast<int>(body.length());
          idx = evEnd + 1;
          String event = body.substring(evStart, evEnd);
          String evDate = eventDate(event);  // YYYYMMDD, or "".
          if (evDate.length() != 8 || evDate < today) continue;  // Past/none.
          bool isToday = evDate == today;
          // Collect this event's designators through a throwaway set;
          // only today's join the scope-highlighting callsigns.
          std::set<String> evCallsigns;
          std::vector<String> designators;
          scanFlights(summaryOf(event), evCallsigns, &designators);
          if (isToday) found.insert(evCallsigns.begin(), evCallsigns.end());
          String evIso = evDate.substring(0, 4) + "-" +
                         evDate.substring(4, 6) + "-" + evDate.substring(6);
          for (const auto& designator : designators) {
            model::IcalSpecial special;
            special.flight = designator;
            special.date = evIso;
            special.source = feed.name;
            special.today = isToday;
            fromCalendar.push_back(special);
          }
        }
        status.flights = static_cast<int>(fromCalendar.size() - before);
      }
    }
    statuses.push_back(status);
  }

  // Soonest first: today's flagging flights lead, then upcoming by date.
  std::sort(fromCalendar.begin(), fromCalendar.end(),
            [](const model::IcalSpecial& a, const model::IcalSpecial& b) {
              return a.date < b.date;
            });

  // Publish. The callsign set only feeds the traffic tagging on this same
  // task, so it needs no lock; the dashboard-facing lists go into the
  // model under the mutex, briefly.
  g_specials = found;
  rlog::Log.printf(
      "[feeds] ical: %d/%u feed(s) ok, %u calendar + %u manual entries -> "
      "%u special callsign(s) for %s\n",
      fetched, static_cast<unsigned>(enabled.size()),
      static_cast<unsigned>(fromCalendar.size()),
      static_cast<unsigned>(manual.size()), static_cast<unsigned>(found.size()),
      today.isEmpty() ? "(no clock)" : today.c_str());
  xSemaphoreTake(mutex, portMAX_DELAY);
  model.icalSpecials = std::move(fromCalendar);
  model.feedStatus = std::move(statuses);
  xSemaphoreGive(mutex);
  return fetched;
}

// ---- Prefetched frame cache (instant time-scrub). ----
//
// So a scrub lands on an already-decoded frame instead of waiting on a
// fetch, every selectable frame's mosaic is decoded once — for the current
// view cover — and kept in PSRAM. pollWeather fetches the frame under the
// knob first, then fills the rest one per call; a settled pan/zoom to a
// different cover drops the cache and re-decodes there. The displayed layer
// (model.weather) is a self-owned copy, so freeing the cache never blanks
// the picture mid-refill.

struct WxCacheEntry {
  time_t time = 0;
  uint8_t* cells = nullptr;
  int16_t width = 0, height = 0;
  uint8_t zoom = 0;
  int32_t originX = 0, originY = 0;
};
std::vector<WxCacheEntry> g_wxCache;
WxCover g_wxCacheCover;         // Cover every cached mosaic shares.
bool g_wxCacheCoverSet = false;
bool g_wxRevealed = false;      // The window for this cover has fully decoded
                                // (or timed out): frames may be shown/scrubbed.
uint32_t g_wxFillStartMs = 0;   // millis() when the current cover's fill began.

int wxCacheFind(time_t t) {
  for (int i = 0; i < static_cast<int>(g_wxCache.size()); ++i) {
    if (g_wxCache[i].time == t) return i;
  }
  return -1;
}

void wxCacheClear() {
  for (auto& e : g_wxCache) heap_caps_free(e.cells);
  g_wxCache.clear();
}

// Fetches and decodes every tile of `cover` from the frame at `base` into a
// freshly allocated PSRAM mosaic (dBZ+32 per pixel). Returns nullptr on a
// memory shortfall or any transfer/decode failure — the caller keeps what it
// had. No lock is held anywhere here.
uint8_t* wxFetchMosaic(const String& base, const WxCover& cover) {
  size_t mosaicBytes = static_cast<size_t>(cover.nx) * cover.ny * kWxTileSize *
                       kWxTileSize;
  // The TLS session lives on the internal heap; the tile PNG, the decoder
  // and this mosaic in PSRAM. Leave headroom for the display copy too.
  if (ESP.getFreeHeap() < 40 * 1024 ||
      heap_caps_get_free_size(MALLOC_CAP_SPIRAM) <
          mosaicBytes + kWxMaxPngBytes + sizeof(PNG) + 384 * 1024) {
    rlog::Log.printf("[feeds] wx: memory low (heap %u KB, PSRAM %u KB); skip.\n",
                  ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024);
    return nullptr;
  }
  // The PNG decoder is ~48 KB of mostly inflate window — too big for the
  // internal heap, so it lives in PSRAM, allocated once.
  if (g_wxPng == nullptr) {
    void* mem =
        heap_caps_malloc(sizeof(PNG), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (mem == nullptr) return nullptr;
    g_wxPng = new (mem) PNG();
  }
  uint8_t* cells = static_cast<uint8_t*>(
      heap_caps_malloc(mosaicBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  uint8_t* pngBuf = static_cast<uint8_t*>(
      heap_caps_malloc(kWxMaxPngBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (cells == nullptr || pngBuf == nullptr) {
    heap_caps_free(cells);
    heap_caps_free(pngBuf);
    return nullptr;
  }
  memset(cells, 0, mosaicBytes);
  WxDecodeCtx ctx;
  ctx.mosaic = cells;
  ctx.mosaicW = cover.nx * kWxTileSize;
  WiFiClientSecure client;
  client.setInsecure();  // TODO(security): install a CA bundle.
  HTTPClient http;
  http.setConnectTimeout(kHttpTimeoutMs);
  http.setTimeout(kHttpTimeoutMs);
  http.setUserAgent(kUserAgent);
  http.setReuse(true);  // The tiles share one host (and TLS session).
  bool ok = true;
  for (int ty = 0; ty < cover.ny && ok; ++ty) {
    for (int tx = 0; tx < cover.nx && ok; ++tx) {
      // {size}/{z}/{x}/{y}/{color}/{smooth}_{snow}: unsmoothed, so pixels
      // stay exact palette entries the table can match.
      String url = base + "/" + String(kWxTileSize) + "/" + String(cover.z) +
                   "/" + String(cover.x0 + tx) + "/" + String(cover.y0 + ty) +
                   "/2/0_0.png";
      ok = wxFetchTile(http, client, url, pngBuf, ctx, tx * kWxTileSize,
                       ty * kWxTileSize);
    }
  }
  heap_caps_free(pngBuf);
  if (!ok) {
    heap_caps_free(cells);
    return nullptr;
  }
  return cells;
}

// Builds a cache entry for a fetched mosaic at the current cover.
WxCacheEntry wxMakeEntry(time_t t, uint8_t* cells, const WxCover& cover) {
  WxCacheEntry e;
  e.time = t;
  e.cells = cells;
  e.width = static_cast<int16_t>(cover.nx * kWxTileSize);
  e.height = static_cast<int16_t>(cover.ny * kWxTileSize);
  e.zoom = cover.z;
  e.originX = cover.x0 * kWxTileSize;
  e.originY = cover.y0 * kWxTileSize;
  return e;
}

// Copies a cached mosaic into a fresh buffer and swaps it into model.weather
// (freeing the previous self-owned layer). The copy runs lock-free; only the
// pointer swap is under the mutex.
void wxPublish(model::Model& model, SemaphoreHandle_t mutex,
               const WxCacheEntry& e) {
  size_t bytes = static_cast<size_t>(e.width) * e.height;
  uint8_t* copy = static_cast<uint8_t*>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (copy == nullptr) return;
  memcpy(copy, e.cells, bytes);
  xSemaphoreTake(mutex, portMAX_DELAY);
  heap_caps_free(model.weather.cells);
  model.weather.cells = copy;
  model.weather.width = e.width;
  model.weather.height = e.height;
  model.weather.zoom = e.zoom;
  model.weather.originX = e.originX;
  model.weather.originY = e.originY;
  model.weather.fetchedMs = millis();
  model.ui.wxFrameTime = e.time;
  ++model.weather.generation;
  xSemaphoreGive(mutex);
  g_wxLoadedTime = e.time;
}

void pollWeather(model::Model& model, SemaphoreHandle_t mutex) {
  if (!WiFi.isConnected()) return;

  // Snapshot the view, the display mode and the time-scrub selection.
  xSemaphoreTake(mutex, portMAX_DELAY);
  const model::Home& home = activeHome(model);
  float lat = home.latitude + model.ui.viewCenter.y / 60.0f;
  float lon = home.longitude + model.ui.viewCenter.x /
                                   (60.0f * cosf(home.latitude * DEG_TO_RAD));
  float range = model.ui.range;
  int offset = model.ui.wxOffsetSteps;
  bool weatherMode = model.ui.display == model::DisplayMode::Weather;
  xSemaphoreGive(mutex);

  // Refresh the frame index (cheap; cached between refreshes) and resolve the
  // scrub range plus the selected frame.
  if (!refreshWxIndex()) return;
  time_t realNow = time(nullptr);
  bool synced = realNow > 1600000000;  // NTP has run.
  int stepsBack = 0;
  if (synced) {
    for (int i = g_wxNowIndex - 1; i >= 0; --i) {
      if (realNow - g_wxFrames[i].time <=
          static_cast<time_t>(config::WEATHER_MAX_BACK_MIN) * 60) {
        ++stepsBack;
      } else {
        break;
      }
    }
  } else {
    stepsBack = g_wxNowIndex > 0 ? g_wxNowIndex : 0;
  }
  int stepsFwd = static_cast<int>(g_wxFrames.size()) - 1 - g_wxNowIndex;
  if (stepsFwd < 0) stepsFwd = 0;
  int selIdx = g_wxNowIndex + offset;
  if (selIdx < 0) selIdx = 0;
  if (selIdx >= static_cast<int>(g_wxFrames.size())) {
    selIdx = static_cast<int>(g_wxFrames.size()) - 1;
  }
  time_t selTime = g_wxFrames[selIdx].time;

  uint32_t now = millis();
  WxCover want = wxCoverFor(lat, lon, range);
  if (!wxCoverEq(want, g_wxWant)) {
    g_wxWant = want;
    g_wxWantMs = now;
  }

  // Commit the cache to a cover on first use, or once the view settles on a
  // different one (so a mid-pan doesn't thrash it). Dropping the cache never
  // blanks the picture: model.weather is a self-owned copy that persists
  // until a new frame is copied in.
  bool needCover = !g_wxCacheCoverSet ||
                   (!wxCoverEq(want, g_wxCacheCover) &&
                    now - g_wxWantMs >= kWxSettleMs);
  if (needCover) {
    wxCacheClear();
    g_wxCacheCover = want;
    g_wxCacheCoverSet = true;
    g_wxLoadedTime = 0;      // Force a republish for the new cover.
    g_wxRevealed = false;    // Hold weather (acquiring) until the window fills.
    g_wxFillStartMs = now;
  }
  WxCover cover = g_wxCacheCover;

  // Drop cached frames that rolled out of the selectable window (e.g. a new
  // now-frame arrived and the oldest aged past the rewind limit).
  for (int i = static_cast<int>(g_wxCache.size()) - 1; i >= 0; --i) {
    bool keep = false;
    for (int j = g_wxNowIndex - stepsBack; j <= g_wxNowIndex + stepsFwd; ++j) {
      if (j >= 0 && j < static_cast<int>(g_wxFrames.size()) &&
          g_wxFrames[j].time == g_wxCache[i].time) {
        keep = true;
        break;
      }
    }
    if (!keep) {
      heap_caps_free(g_wxCache[i].cells);
      g_wxCache.erase(g_wxCache.begin() + i);
    }
  }

  // A failed transfer backs off briefly so a bad frame doesn't hammer, but
  // not so long that a scrub stalls.
  bool canFetch = !(g_wxLastTryMs != 0 && now - g_wxLastTryMs < kWxRetryMs);

  // 1) The frame under the knob is the priority: fetch it if it isn't cached.
  int ci = wxCacheFind(selTime);
  bool fetched = false;
  if (ci < 0 && canFetch) {
    uint32_t started = millis();
    uint8_t* cells = wxFetchMosaic(g_wxFrames[selIdx].base, cover);
    fetched = true;
    if (cells != nullptr) {
      g_wxLastTryMs = 0;
      g_wxCache.push_back(wxMakeEntry(selTime, cells, cover));
      ci = static_cast<int>(g_wxCache.size()) - 1;
      rlog::Log.printf(
          "[feeds] wx: @%ld cached (%lu ms, %u frames, PSRAM %u KB)\n",
          static_cast<long>(selTime),
          static_cast<unsigned long>(millis() - started),
          static_cast<unsigned>(g_wxCache.size()), ESP.getFreePsram() / 1024);
    } else {
      g_wxLastTryMs = now;
    }
  }

  (void)ci;  // The publish waits for the full window; see the reveal below.

  // 2) Background fill (weather mode only): cache the nearest still-missing
  // selectable frame, one per call, so the rest of the timeline is ready
  // before the knob reaches it. Flights mode keeps only the live frame fresh.
  if (weatherMode && !fetched && canFetch) {
    int bestJ = -1, bestDist = 1 << 30;
    for (int j = g_wxNowIndex - stepsBack; j <= g_wxNowIndex + stepsFwd; ++j) {
      if (j < 0 || j >= static_cast<int>(g_wxFrames.size())) continue;
      if (wxCacheFind(g_wxFrames[j].time) >= 0) continue;
      int d = j > selIdx ? j - selIdx : selIdx - j;
      if (d < bestDist) {
        bestDist = d;
        bestJ = j;
      }
    }
    if (bestJ >= 0) {
      uint32_t started = millis();
      uint8_t* cells = wxFetchMosaic(g_wxFrames[bestJ].base, cover);
      if (cells != nullptr) {
        g_wxLastTryMs = 0;
        g_wxCache.push_back(wxMakeEntry(g_wxFrames[bestJ].time, cells, cover));
        rlog::Log.printf(
            "[feeds] wx: prefetch @%ld (%lu ms, %u/%d frames, PSRAM %u KB)\n",
            static_cast<long>(g_wxFrames[bestJ].time),
            static_cast<unsigned long>(millis() - started),
            static_cast<unsigned>(g_wxCache.size()),
            stepsBack + stepsFwd + 1, ESP.getFreePsram() / 1024);
      } else {
        g_wxLastTryMs = now;
      }
    }
  }

  // Reveal weather only once every frame in the window is decoded, so the
  // first scrub in any direction is instant — no visit-then-fetch. The reveal
  // latches until the cover changes (a later rollover never drops back to
  // acquiring), with a timeout so a flaky frame can't hold it up forever.
  if (!g_wxRevealed) {
    bool complete = true;
    for (int j = g_wxNowIndex - stepsBack; j <= g_wxNowIndex + stepsFwd; ++j) {
      if (j < 0 || j >= static_cast<int>(g_wxFrames.size())) continue;
      if (wxCacheFind(g_wxFrames[j].time) < 0) {
        complete = false;
        break;
      }
    }
    if (complete ||
        now - g_wxFillStartMs > config::WEATHER_FILL_TIMEOUT_MS) {
      g_wxRevealed = true;
      rlog::Log.printf("[feeds] wx: window ready (%u frames, %s)\n",
                    static_cast<unsigned>(g_wxCache.size()),
                    complete ? "complete" : "timeout");
    }
  }

  // Once revealed, show the selected frame and expose the scrub range; before
  // that the range is zero, so the knob can't scrub onto an unbuffered frame.
  int ci2 = wxCacheFind(selTime);
  if (g_wxRevealed && ci2 >= 0 && selTime != g_wxLoadedTime) {
    wxPublish(model, mutex, g_wxCache[ci2]);
  }
  xSemaphoreTake(mutex, portMAX_DELAY);
  model.ui.wxStepsBack = g_wxRevealed ? stepsBack : 0;
  model.ui.wxStepsFwd = g_wxRevealed ? stepsFwd : 0;
  xSemaphoreGive(mutex);
}

void pollTrace(model::Model& model, SemaphoreHandle_t mutex) {
  uint32_t now = millis();
  if (now - g_traceCheckMs < kTraceCheckMs) return;
  g_traceCheckMs = now;

  // Snapshot the follow target and home under the mutex; everything slow
  // below runs with no lock held.
  xSemaphoreTake(mutex, portMAX_DELAY);
  model::Home home = activeHome(model);
  bool following = model.ui.following && model.ui.followIndex >= 0 &&
                   model.ui.followIndex < static_cast<int>(model.aircraft.size());
  String hex = following ? model.aircraft[model.ui.followIndex].hex : String();
  xSemaphoreGive(mutex);

  if (!following || !isIcaoHex(hex)) {
    // Not following (or nothing a trace exists for): drop the cached
    // history, so re-following — even the same flight — reseeds fresh.
    // The trail itself is Inputs' to clear, on its follow transitions.
    g_traceHex = "";
    return;
  }

  if (hex != g_traceHex) {
    // The follow target changed: fetch its history. Failures (a flight
    // adsb.lol does not know, a bad transfer) back off and leave the
    // renderer's live-only trail in place.
    if (hex == g_traceFailHex && now - g_traceFailMs < kTraceRetryMs) return;
    std::vector<model::Vec> history;
    if (!fetchTrace(home, hex, history)) {
      g_traceFailHex = hex;
      g_traceFailMs = millis();
      return;
    }
    // Seed under the mutex: history first, then whatever live samples
    // the renderer collected while the fetch ran. Bail if the follow
    // moved on (or the home switched) mid-fetch — the positions would
    // be relative to the wrong flight or the wrong origin.
    xSemaphoreTake(mutex, portMAX_DELAY);
    bool still =
        model.ui.following && model.ui.followIndex >= 0 &&
        model.ui.followIndex < static_cast<int>(model.aircraft.size()) &&
        model.aircraft[model.ui.followIndex].hex == hex &&
        activeHome(model).name == home.name;
    if (still) {
      history.insert(history.end(), model.ui.followTrail.begin(),
                     model.ui.followTrail.end());
      model.ui.followTrail = std::move(history);
      g_traceHex = hex;
    }
    xSemaphoreGive(mutex);
    if (!still) {
      rlog::Log.println("[feeds] trace: follow changed mid-fetch; dropped.");
    }
    return;
  }

  // Same target, already seeded: the renderer owns the trail from here — it
  // samples the live tail and its raised cap holds both the history and the
  // tail without eroding the seeded prefix — so there is nothing more to do.
}

void pollDest(model::Model& model, SemaphoreHandle_t mutex) {
  // Snapshot the follow target and home under the mutex; the lookup below
  // runs with no lock held. The target comparison runs on every call so a
  // follow change drops the stale destination promptly, even though the
  // HTTPS lookup itself is rate-limited.
  xSemaphoreTake(mutex, portMAX_DELAY);
  model::Home home = activeHome(model);
  bool following =
      model.ui.following && model.ui.followIndex >= 0 &&
      model.ui.followIndex < static_cast<int>(model.aircraft.size());
  String callsign =
      following ? model.aircraft[model.ui.followIndex].callsign : String();
  bool hasDest = model.ui.followHasDest;
  xSemaphoreGive(mutex);

  // A different callsign than the published destination belongs to (the
  // follow changed or ended): drop it so no stale onward path lingers, and
  // reset the cache so the new target is looked up afresh. Only take the
  // mutex when there is actually a published destination to clear.
  if (callsign != g_destCallsign) {
    if (hasDest) {
      xSemaphoreTake(mutex, portMAX_DELAY);
      model.ui.followHasDest = false;
      xSemaphoreGive(mutex);
    }
    g_destCallsign = "";
  }

  if (!following || callsign.isEmpty()) return;  // Nothing to look up.
  if (callsign == g_destCallsign) return;        // Already resolved.

  // Rate-limit the lookups, and back off a callsign whose last lookup
  // failed (an unknown route, a bad transfer) so a followed flight adsbdb
  // has no route for is not queried every second.
  uint32_t now = millis();
  if (now - g_destCheckMs < kDestCheckMs) return;
  g_destCheckMs = now;
  if (callsign == g_destFailCallsign && now - g_destFailMs < kDestRetryMs) {
    return;
  }

  model::Vec dest;
  if (!fetchDest(home, callsign, dest)) {
    g_destFailCallsign = callsign;
    g_destFailMs = millis();
    rlog::Log.printf("[feeds] dest: no route for %s.\n", callsign.c_str());
    return;
  }

  // Publish under the mutex, briefly. Bail if the follow moved on (or the
  // home switched) while the lookup ran — the projected point would be for
  // the wrong flight or the wrong origin.
  xSemaphoreTake(mutex, portMAX_DELAY);
  bool still =
      model.ui.following && model.ui.followIndex >= 0 &&
      model.ui.followIndex < static_cast<int>(model.aircraft.size()) &&
      model.aircraft[model.ui.followIndex].callsign == callsign &&
      activeHome(model).name == home.name;
  if (still) {
    model.ui.followDest = dest;
    model.ui.followHasDest = true;
    g_destCallsign = callsign;
  }
  xSemaphoreGive(mutex);
  if (still) {
    rlog::Log.printf("[feeds] dest: %s -> %.0f,%.0f NM from home.\n",
                  callsign.c_str(), dest.x, dest.y);
  }
}

void reprojectStatics(model::Model& model) {
  const model::Home& home = activeHome(model);
  for (auto& poi : model.pois) {
    poi.pos = geoToWorld(home, poi.latitude, poi.longitude);
  }
}

}  // namespace feeds
