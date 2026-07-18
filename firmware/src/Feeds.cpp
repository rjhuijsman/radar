#include "Feeds.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PNGdec.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <esp_heap_caps.h>
#include <math.h>

#include <algorithm>
#include <new>
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
constexpr uint32_t kWxRetryMs = 30000;
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

// Weather fetch state, only ever touched from the network task.
String g_wxTileBase;        // host+path of the newest frame, from the index.
uint32_t g_wxIndexMs = 0;   // When the index was last fetched (0 = never).
uint32_t g_wxLastTryMs = 0; // Last attempt that did not publish (backoff).
PNG* g_wxPng = nullptr;     // PNGdec instance; ~48 KB, so it lives in PSRAM.
WxCover g_wxWant;           // Last desired cover, for the settle debounce.
uint32_t g_wxWantMs = 0;

// Fetches the RainViewer frame index and caches the newest past frame's
// tile base URL. The index is a couple of hundred bytes of JSON, so the
// String round-trip through httpGet is fine.
bool refreshWxIndex() {
  if (!g_wxTileBase.isEmpty() &&
      millis() - g_wxIndexMs < config::WEATHER_POLL_MS) {
    return true;
  }
  String body;
  if (!httpGet("https://api.rainviewer.com/public/weather-maps.json", body)) {
    Serial.println("[feeds] wx: index fetch failed.");
    // A previously fetched frame stays valid for a while; reuse it.
    return !g_wxTileBase.isEmpty();
  }
  static SpiRamAllocator allocator;
  JsonDocument doc(&allocator);
  if (deserializeJson(doc, body) != DeserializationError::Ok) return false;
  const char* host = doc["host"];
  JsonArrayConst past = doc["radar"]["past"].as<JsonArrayConst>();
  if (host == nullptr || past.size() == 0) {
    Serial.println("[feeds] wx: index has no radar frames.");
    return false;
  }
  const char* path = past[past.size() - 1]["path"];
  if (path == nullptr) return false;
  g_wxTileBase = String(host) + path;
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
    Serial.printf("[feeds] wx: HTTP %d for tile.\n", code);
    http.end();
    return false;
  }
  // The tile cache always announces a length; without one the body would
  // have to be read to connection close, which a keep-alive socket turns
  // into a full timeout. Fail fast instead.
  int announced = http.getSize();
  if (announced <= 0 || announced > static_cast<int>(kWxMaxPngBytes)) {
    Serial.printf("[feeds] wx: %d-byte tile refused.\n", announced);
    http.end();
    return false;
  }
  YieldingReader reader(http.getStream());
  size_t len = reader.readBytes(reinterpret_cast<char*>(pngBuf),
                                static_cast<size_t>(announced));
  http.end();
  if (len != static_cast<size_t>(announced)) {
    Serial.println("[feeds] wx: short tile body.");
    return false;
  }

  ctx.offX = offX;
  ctx.offY = offY;
  if (g_wxPng->openRAM(pngBuf, static_cast<int>(len), wxPngLine) !=
      PNG_SUCCESS) {
    Serial.println("[feeds] wx: tile is not a PNG.");
    return false;
  }
  if (g_wxPng->getWidth() != kWxTileSize ||
      g_wxPng->getHeight() != kWxTileSize) {
    Serial.printf("[feeds] wx: unexpected %dx%d tile.\n", g_wxPng->getWidth(),
                  g_wxPng->getHeight());
    g_wxPng->close();
    return false;
  }
  int rc = g_wxPng->decode(&ctx, 0);
  g_wxPng->close();
  if (rc != PNG_SUCCESS) {
    Serial.printf("[feeds] wx: PNG decode failed (%d).\n", rc);
    return false;
  }
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

  // Merge under the mutex, briefly: upsert so each blip's sweep-refresh
  // state carries across polls, then drop aircraft that left the feed.
  xSemaphoreTake(mutex, portMAX_DELAY);

  // The home may have been switched while the fetch was in flight; these
  // positions are relative to the old one, so drop them — the switch
  // already requested a fresh poll around the new home.
  if (activeHome(model).name != home.name) {
    xSemaphoreGive(mutex);
    Serial.println("[feeds] adsb: home changed mid-fetch; snapshot dropped.");
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
  if (!WiFi.isConnected()) return;

  // Snapshot the view under the mutex: where the scope is looking, and
  // what the published layer already covers.
  xSemaphoreTake(mutex, portMAX_DELAY);
  const model::Home& home = activeHome(model);
  float lat = home.latitude + model.ui.viewCenter.y / 60.0f;
  float lon = home.longitude + model.ui.viewCenter.x /
                                   (60.0f * cosf(home.latitude * DEG_TO_RAD));
  float range = model.ui.range;
  bool haveLayer = model.weather.cells != nullptr;
  uint32_t fetchedMs = model.weather.fetchedMs;
  WxCover loaded;
  if (haveLayer) {
    loaded.z = model.weather.zoom;
    loaded.x0 = model.weather.originX / kWxTileSize;
    loaded.y0 = model.weather.originY / kWxTileSize;
    loaded.nx = static_cast<int8_t>(model.weather.width / kWxTileSize);
    loaded.ny = static_cast<int8_t>(model.weather.height / kWxTileSize);
  }
  xSemaphoreGive(mutex);

  // Decide whether a fetch is due at all: the refetch interval elapsed,
  // or the view wants a different tile cover and has settled on it (so a
  // held zoom knob cannot queue a fetch per detent). Failures back off.
  WxCover want = wxCoverFor(lat, lon, range);
  uint32_t now = millis();
  if (!wxCoverEq(want, g_wxWant)) {
    g_wxWant = want;
    g_wxWantMs = now;
  }
  bool stale = !haveLayer || now - fetchedMs >= config::WEATHER_POLL_MS;
  bool moved = haveLayer && !wxCoverEq(want, loaded) &&
               now - g_wxWantMs >= kWxSettleMs;
  if (!stale && !moved) return;
  if (g_wxLastTryMs != 0 && now - g_wxLastTryMs < kWxRetryMs) return;
  g_wxLastTryMs = now;  // Cleared below if the cycle publishes.

  // Do not start a cycle the heaps cannot absorb: the TLS session lives
  // on the internal heap; the tile PNGs, the decoder state and both the
  // old and new mosaics in PSRAM. Skipping a check is invisible.
  size_t mosaicBytes = static_cast<size_t>(want.nx) * want.ny * kWxTileSize *
                       kWxTileSize;
  if (ESP.getFreeHeap() < 40 * 1024 ||
      heap_caps_get_free_size(MALLOC_CAP_SPIRAM) <
          mosaicBytes + kWxMaxPngBytes + sizeof(PNG) + 512 * 1024) {
    Serial.printf("[feeds] wx: memory low (heap %u KB, PSRAM %u KB); skipping.\n",
                  ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024);
    return;
  }
  if (!refreshWxIndex()) return;

  // The PNG decoder is ~48 KB of mostly inflate window — far too big for
  // the internal heap, so it lives in PSRAM, allocated once.
  if (g_wxPng == nullptr) {
    void* mem =
        heap_caps_malloc(sizeof(PNG), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (mem == nullptr) return;
    g_wxPng = new (mem) PNG();
  }

  uint32_t started = millis();
  uint8_t* cells = static_cast<uint8_t*>(
      heap_caps_malloc(mosaicBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  uint8_t* pngBuf = static_cast<uint8_t*>(
      heap_caps_malloc(kWxMaxPngBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (cells == nullptr || pngBuf == nullptr) {
    heap_caps_free(cells);
    heap_caps_free(pngBuf);
    return;
  }
  memset(cells, 0, mosaicBytes);

  WxDecodeCtx ctx;
  ctx.mosaic = cells;
  ctx.mosaicW = want.nx * kWxTileSize;

  WiFiClientSecure client;
  // TODO(security): install a CA bundle and validate certificates.
  client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(kHttpTimeoutMs);
  http.setTimeout(kHttpTimeoutMs);
  http.setUserAgent(kUserAgent);
  http.setReuse(true);  // The tiles share one host (and TLS session).
  bool ok = true;
  for (int ty = 0; ty < want.ny && ok; ++ty) {
    for (int tx = 0; tx < want.nx && ok; ++tx) {
      // {size}/{z}/{x}/{y}/{color}/{smooth}_{snow}: unsmoothed, so pixels
      // stay exact palette entries the table above can match.
      String url = g_wxTileBase + "/" + String(kWxTileSize) + "/" +
                   String(want.z) + "/" + String(want.x0 + tx) + "/" +
                   String(want.y0 + ty) + "/2/0_0.png";
      ok = wxFetchTile(http, client, url, pngBuf, ctx, tx * kWxTileSize,
                       ty * kWxTileSize);
    }
  }
  heap_caps_free(pngBuf);
  if (!ok) {
    heap_caps_free(cells);
    Serial.println("[feeds] wx: cycle failed; keeping last layer.");
    return;
  }
  g_wxLastTryMs = 0;

  // Publish under the mutex, briefly. The mosaic is georeferenced
  // absolutely, so a home switched mid-fetch needs no invalidation — the
  // switch only changes what the next cover check asks for.
  xSemaphoreTake(mutex, portMAX_DELAY);
  heap_caps_free(model.weather.cells);
  model.weather.cells = cells;
  model.weather.width = static_cast<int16_t>(want.nx * kWxTileSize);
  model.weather.height = static_cast<int16_t>(want.ny * kWxTileSize);
  model.weather.zoom = want.z;
  model.weather.originX = want.x0 * kWxTileSize;
  model.weather.originY = want.y0 * kWxTileSize;
  model.weather.fetchedMs = millis();
  ++model.weather.generation;
  xSemaphoreGive(mutex);

  Serial.printf(
      "[feeds] wx: %dx%d tiles z%u published (%lu lit px, %lu unknown, "
      "%lu ms, heap %u KB, PSRAM %u KB)\n",
      want.nx, want.ny, want.z, static_cast<unsigned long>(ctx.lit),
      static_cast<unsigned long>(ctx.unknown),
      static_cast<unsigned long>(millis() - started), ESP.getFreeHeap() / 1024,
      ESP.getFreePsram() / 1024);
}

void reprojectStatics(model::Model& model) {
  const model::Home& home = activeHome(model);
  for (auto& poi : model.pois) {
    poi.pos = geoToWorld(home, poi.latitude, poi.longitude);
  }
}

}  // namespace feeds
