#include "Radar.h"

#include <math.h>
#include <string.h>

#include <vector>

#include "Display.h"
#include "config.h"

namespace radar {
namespace {

using model::Aircraft;
using model::DisplayMode;
using model::Model;
using model::Vec;

constexpr float kSweepPersistenceMs = 2600.0f;
constexpr float kPanTauMs = 300.0f;

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
constexpr uint16_t C_GREEN = rgb565(61, 255, 164);
constexpr uint16_t C_CYAN = rgb565(95, 220, 255);
constexpr uint16_t C_AMBER = rgb565(255, 194, 74);
constexpr uint16_t C_WHITE = rgb565(233, 247, 240);
constexpr uint16_t C_RING = rgb565(24, 96, 66);
constexpr uint16_t C_TICK = rgb565(30, 120, 84);
constexpr uint16_t C_BG = rgb565(6, 20, 15);

// Weather mode renders the green phosphor family in fluorescent blue —
// the same brightness hierarchy with the hue shifted, like a different
// phosphor in the same tube. Amber and the other accents stay put.
constexpr uint16_t C_BLUE = rgb565(61, 164, 255);
constexpr uint16_t C_RING_BLUE = rgb565(24, 66, 96);
constexpr uint16_t C_TICK_BLUE = rgb565(30, 84, 120);

bool bluePalette(const Model& model) {
  return model.ui.display == DisplayMode::Weather;
}
uint16_t phosphorColor(const Model& model) {
  return bluePalette(model) ? C_BLUE : C_GREEN;
}
uint16_t ringColor(const Model& model) {
  return bluePalette(model) ? C_RING_BLUE : C_RING;
}
uint16_t tickColor(const Model& model) {
  return bluePalette(model) ? C_TICK_BLUE : C_TICK;
}

const float kCenterX = config::PANEL_WIDTH / 2.0f;
const float kCenterY = config::PANEL_HEIGHT / 2.0f;
const float kRadius = config::PANEL_WIDTH * 0.46f;

// Scales an RGB565 color by the current brightness, for ambient dimming.
uint16_t dim(uint16_t color, float k) {
  uint8_t r = (color >> 11) & 0x1F;
  uint8_t g = (color >> 5) & 0x3F;
  uint8_t b = color & 0x1F;
  return (static_cast<uint8_t>(r * k) << 11) |
         (static_cast<uint8_t>(g * k) << 5) | static_cast<uint8_t>(b * k);
}

float pixelsPerNm(const Model& model) { return kRadius / model.ui.range; }

// Projects a world point (NM from home) to screen pixels, relative to the
// animated view center.
void project(const Model& model, const Vec& world, float& sx, float& sy) {
  float ppn = pixelsPerNm(model);
  sx = kCenterX + (world.x - model.ui.viewCenter.x) * ppn;
  sy = kCenterY - (world.y - model.ui.viewCenter.y) * ppn;
}

// A point on the rim (or an inner ring) at a compass bearing.
void polar(float bearingDeg, float radiusPx, float& sx, float& sy) {
  float a = (bearingDeg - 90.0f) * DEG_TO_RAD;
  sx = kCenterX + radiusPx * cosf(a);
  sy = kCenterY + radiusPx * sinf(a);
}

float bearingFrom(const Model& model, const Vec& world) {
  float dx = world.x - model.ui.viewCenter.x;
  float dy = world.y - model.ui.viewCenter.y;
  float b = atan2f(dx, dy) * RAD_TO_DEG;
  return b < 0 ? b + 360.0f : b;
}

float distanceFrom(const Model& model, const Vec& world) {
  float dx = world.x - model.ui.viewCenter.x;
  float dy = world.y - model.ui.viewCenter.y;
  return sqrtf(dx * dx + dy * dy);
}

bool showFlights(const Model& model) {
  return model.ui.display != DisplayMode::Weather;
}
bool showWeather(const Model& model) {
  return model.ui.display != DisplayMode::Flights;
}

// True when the sweep, moving from `a` to `b` degrees, crossed `target`.
bool swept(float a, float b, float target) {
  if (a == b) return false;
  if (a < b) return target > a && target <= b;
  return target > a || target <= b;
}

Vec targetCenter(const Model& model) {
  if (model.ui.following && model.ui.followIndex >= 0 &&
      model.ui.followIndex < static_cast<int>(model.aircraft.size())) {
    return model.aircraft[model.ui.followIndex].pos;
  }
  return Vec{0, 0};  // Home sits at the world origin.
}

void drawTriangle(Arduino_GFX* gfx, float cx, float cy, float trackDeg,
                  float scale, uint16_t color) {
  // Local nose-up triangle, rotated to the track.
  float pts[3][2] = {{0, -9}, {6, 7}, {-6, 7}};
  float a = trackDeg * DEG_TO_RAD;
  float ca = cosf(a), sa = sinf(a);
  int16_t xs[3], ys[3];
  for (int i = 0; i < 3; ++i) {
    float x = pts[i][0] * scale, y = pts[i][1] * scale;
    xs[i] = static_cast<int16_t>(cx + x * ca - y * sa);
    ys[i] = static_cast<int16_t>(cy + x * sa + y * ca);
  }
  gfx->fillTriangle(xs[0], ys[0], xs[1], ys[1], xs[2], ys[2], color);
}

void drawLabel(Arduino_GFX* gfx, float x, float y, const String& text,
               uint16_t color) {
  gfx->setTextColor(color);
  gfx->setTextSize(1);
  gfx->setCursor(static_cast<int16_t>(x), static_cast<int16_t>(y));
  gfx->print(text);
}

void drawBackground(Arduino_GFX* gfx, const Model& model) {
  gfx->fillScreen(C_BG);
  float k = model.ui.brightness;
  // Four range rings.
  for (int i = 1; i <= 4; ++i) {
    float r = kRadius * i / 4.0f;
    gfx->drawCircle(kCenterX, kCenterY, static_cast<int16_t>(r),
                    dim(i == 4 ? tickColor(model) : ringColor(model), k));
  }
  // Bearing ticks every 30 degrees.
  for (int d = 0; d < 360; d += 30) {
    float ox, oy, ix, iy;
    polar(d, kRadius, ox, oy);
    polar(d, kRadius - (d % 90 == 0 ? kRadius * 0.06f : kRadius * 0.035f), ix,
          iy);
    gfx->drawLine(ix, iy, ox, oy, dim(tickColor(model), k));
  }
  // Compass letters.
  const char* letters[4] = {"N", "E", "S", "W"};
  int angles[4] = {0, 90, 180, 270};
  gfx->setTextSize(2);
  gfx->setTextColor(dim(phosphorColor(model), k));
  for (int i = 0; i < 4; ++i) {
    float x, y;
    polar(angles[i], kRadius + 18, x, y);
    gfx->setCursor(static_cast<int16_t>(x - 6), static_cast<int16_t>(y - 8));
    gfx->print(letters[i]);
  }
  // Range labels at 50% and 100%.
  gfx->setTextSize(1);
  for (float frac : {0.5f, 1.0f}) {
    float x, y;
    polar(45, kRadius * frac, x, y);
    drawLabel(gfx, x + 4, y - 4, String(static_cast<int>(model.ui.range * frac)) + " NM",
              dim(ringColor(model), k));
  }
}

// `fade` ramps the sweep in over the first moments online, so the handoff
// from the acquiring-signal screen is a reveal rather than a hard cut.
void drawSweep(Arduino_GFX* gfx, const Model& model, float fade) {
  float x, y;
  polar(model.ui.sweepAngle, kRadius, x, y);
  gfx->drawLine(kCenterX, kCenterY, x, y,
                dim(phosphorColor(model), model.ui.brightness * fade));
}

void drawHomeMarker(Arduino_GFX* gfx, const Model& model) {
  float x, y;
  project(model, Vec{0, 0}, x, y);
  if (hypotf(x - kCenterX, y - kCenterY) > kRadius) return;
  uint16_t c = dim(C_WHITE, model.ui.brightness);
  gfx->drawLine(x - 10, y, x + 10, y, c);
  gfx->drawLine(x, y - 10, x, y + 10, c);
  gfx->fillCircle(x, y, 3, c);
}

void drawPoi(Arduino_GFX* gfx, const Model& model, const model::Poi& poi) {
  float x, y;
  project(model, poi.pos, x, y);
  uint16_t c = dim(C_CYAN, model.ui.brightness);
  if (poi.isAirport) {
    gfx->drawCircle(x, y, 9, c);
    for (float heading : poi.runwayHeadings) {
      float a = (heading - 90) * DEG_TO_RAD;
      gfx->drawLine(x - cosf(a) * 7, y - sinf(a) * 7, x + cosf(a) * 7,
                    y + sinf(a) * 7, c);
    }
  } else {
    gfx->drawLine(x, y - 6, x + 6, y, c);
    gfx->drawLine(x + 6, y, x, y + 6, c);
    gfx->drawLine(x, y + 6, x - 6, y, c);
    gfx->drawLine(x - 6, y, x, y - 6, c);
  }
  drawLabel(gfx, x - 12, y + 14, poi.name, c);
}

void drawEdgeArrow(Arduino_GFX* gfx, const Model& model, const Aircraft& ac,
                   float distance) {
  uint16_t c = dim(ac.special ? C_AMBER : C_WHITE, model.ui.brightness);
  float bearing = bearingFrom(model, ac.shown);
  float x, y;
  polar(bearing, kRadius * 0.985f, x, y);
  drawTriangle(gfx, x, y, bearing + 180, 1.1f, c);  // Chevron points out.
  float lx, ly;
  polar(bearing, kRadius * 0.86f, lx, ly);
  drawLabel(gfx, lx - 16, ly - 6, ac.callsign, c);
  drawLabel(gfx, lx - 16, ly + 6, String(static_cast<int>(distance)) + " NM", c);
}

void drawAircraft(Arduino_GFX* gfx, const Model& model, Aircraft& ac,
                  int index) {
  bool followed = model.ui.following && index == model.ui.followIndex;
  bool selected = !model.ui.following && index == model.ui.browseSel;
  bool candidate = model.ui.following && index == model.ui.candidate;

  const Vec& at = followed ? ac.pos : ac.shown;
  if (!followed && !ac.seen) return;

  float distance = distanceFrom(model, at);
  if (distance > model.ui.range) {
    if (ac.special || selected || candidate) drawEdgeArrow(gfx, model, ac, distance);
    return;
  }

  float k = model.ui.brightness;
  float bright = followed ? 1.0f : (0.35f + 0.65f * ac.freshness);
  uint16_t color = dim(ac.special ? C_AMBER : C_GREEN, k * bright);

  float x, y;
  project(model, at, x, y);

  // Leading line: one minute ahead along the track.
  float ahead = ac.groundSpeed / 60.0f;
  Vec tip{at.x + sinf(ac.track * DEG_TO_RAD) * ahead,
          at.y + cosf(ac.track * DEG_TO_RAD) * ahead};
  float tx, ty;
  project(model, tip, tx, ty);
  gfx->drawLine(x, y, tx, ty, color);

  if (ac.special) gfx->drawCircle(x, y, 13, dim(C_AMBER, k * bright));
  drawTriangle(gfx, x, y, ac.track, followed ? 1.2f : 1.0f, color);

  drawLabel(gfx, x + 12, y - 4, ac.callsign, color);
  String block = String(ac.altitude) + "  " + String(static_cast<int>(ac.groundSpeed)) + "kt";
  drawLabel(gfx, x + 12, y + 8, block, dim(C_WHITE, k * bright));

  if (selected || candidate) {
    uint16_t rc = dim(candidate ? C_AMBER : C_WHITE, k);
    gfx->drawRect(x - 17, y - 17, 34, 34, rc);  // Selection reticle.
  }
  if (followed) drawLabel(gfx, x + 12, y - 16, "FOLLOW", dim(C_AMBER, k));
}

// TODO(feeds): replace with coastlines/borders/cities loaded from the
// Natural Earth basemap in LittleFS. This placeholder keeps the layer
// visible so the Geography toggle has an effect.
void drawGeography(Arduino_GFX* gfx, const Model& model) {
  static const float coast[][2] = {{-40, -31}, {-22, -15}, {-8, -19},
                                    {6, -7},   {20, -11},  {38, 1}};
  uint16_t c = dim(C_CYAN, model.ui.brightness * 0.6f);
  for (size_t i = 1; i < sizeof(coast) / sizeof(coast[0]); ++i) {
    float x0, y0, x1, y1;
    project(model, Vec{coast[i - 1][0], coast[i - 1][1]}, x0, y0);
    project(model, Vec{coast[i][0], coast[i][1]}, x1, y1);
    gfx->drawLine(x0, y0, x1, y1, c);
  }
}

// TODO(feeds): replace with reprojected RainViewer tiles. Sample cells so
// the Weather layer is visible.
void drawWeather(Arduino_GFX* gfx, const Model& model) {
  static const float cells[][3] = {{-16, 9, 13}, {13, -13, 17}, {26, 19, 10}};
  float ppn = pixelsPerNm(model);
  uint16_t c = dim(rgb565(60, 150, 90), model.ui.brightness);
  for (auto& cell : cells) {
    float x, y;
    project(model, Vec{cell[0], cell[1]}, x, y);
    gfx->fillCircle(x, y, static_cast<int16_t>(cell[2] * ppn * 0.5f), c);
  }
}

// Shown until Wi-Fi associates: "ACQUIRING SIGNAL" over the center, pulsing
// in and out. `fade` is 1 while offline; once signal is acquired the title
// is drawn a few more frames with a falling fade so it dissolves into the
// live scope instead of hard-cutting (each frame erases and repaints it,
// so the fade to black leaves no dark residue behind).
void drawAcquiringSignal(Arduino_GFX* gfx, const Model& model, float fade) {
  float k = model.ui.brightness * fade;
  // Gentle brightness pulse, floored so the amber never darkens toward black —
  // fully dimming would punch dark holes in the phosphor graphics it sits over.
  float pulse = 0.68f + 0.32f * sinf(millis() / 360.0f);
  uint16_t c = dim(C_AMBER, k * pulse);

  const char* msg = "ACQUIRING SIGNAL";
  const int len = 16;
  const int size = 3;
  const int charW = 6 * size, charH = 8 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(c);
  gfx->setCursor(static_cast<int>(kCenterX) - (len * charW) / 2,
                 static_cast<int>(kCenterY) - charH / 2);
  gfx->print(msg);

  // Input-test line: flashes the most recent control event below the title so
  // the offline screen doubles as a wiring check. Fades out over ~2500 ms.
  // Drawn INSIDE the acquiring-signal element's tracked span (see drawDynamic),
  // so the dirty-rect box grows to cover it and it is erased and repainted
  // every frame with no ghosting.
  if (model.ui.lastInput.length() > 0) {
    uint32_t age = millis() - model.ui.lastInputMs;
    if (age < 2500) {
      float lineFade = 1.0f - age / 2500.0f;
      uint16_t ic = dim(C_CYAN, k * lineFade);
      const int isize = 2;
      const int icharW = 6 * isize;
      int ilen = model.ui.lastInput.length();
      gfx->setTextSize(isize);
      gfx->setTextColor(ic);
      gfx->setCursor(static_cast<int>(kCenterX) - (ilen * icharW) / 2,
                     static_cast<int>(kCenterY) + charH / 2 + 12);
      gfx->print(model.ui.lastInput);
    }
  }
}

// Toggle feedback on the live scope: when the Display or Geography switch
// changes, the new state flashes centered below the home marker and fades
// out over 2.5 s, echoing the offline input-test line. Knob events show
// nothing here — the reticle and zoom are their own feedback. Drawn inside
// a tracked span (see drawDynamic), so the dirty-rect system erases it
// cleanly with no ghosting.
void drawToggleFlash(Arduino_GFX* gfx, const Model& model) {
  if (model.ui.lastInput.isEmpty()) return;
  uint32_t age = millis() - model.ui.lastInputMs;
  if (age >= 2500) return;

  String text;
  if (model.ui.lastInput == "DISPLAY: WEATHER") {
    text = "WEATHER";
  } else if (model.ui.lastInput == "DISPLAY: FLIGHTS") {
    text = "FLIGHTS";
  } else if (model.ui.lastInput.startsWith("GEOGRAPHY")) {
    text = model.ui.lastInput;
  } else {
    return;
  }

  float fade = 1.0f - age / 2500.0f;
  uint16_t c = dim(C_CYAN, model.ui.brightness * fade);
  const int size = 2;
  const int charW = 6 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(c);
  gfx->setCursor(static_cast<int>(kCenterX) -
                     (static_cast<int>(text.length()) * charW) / 2,
                 static_cast<int>(kCenterY) + 40);
  gfx->print(text);
}

// ---- Incremental (dirty-rect) renderer. ----
//
// The static reference holds the unchanging scene (background, rings, weather,
// geography). Each frame we erase the previous positions of the moving
// elements by copying those regions back from the reference, then redraw the
// elements at their new positions. Erasing every old box before drawing any
// new one keeps overlapping elements correct.

display::LiveGfx* g_live = nullptr;  // Live surface (the scanned framebuffer).
Arduino_GFX* g_static = nullptr;     // Off-screen static-reference surface.
uint16_t* g_liveFb = nullptr;
uint16_t* g_staticFb = nullptr;
int g_w = 0, g_h = 0;

bool g_needStatic = true;  // Rebuild the static reference before the next frame.
bool g_needFull = true;    // Repaint the whole live buffer before the next frame.

// Fingerprint of everything the static reference depends on. When it changes,
// the reference is rebuilt and the live buffer is fully repainted from it.
struct StaticSig {
  int32_t range100 = 0;  // Range in NM * 100 (ring labels, projection scale).
  int16_t cx = 0, cy = 0;  // View center in pixels (weather/geography shift).
  uint8_t mode = 0;        // Display mode (weather layer on/off).
  uint8_t geo = 0;         // Geography overlay on/off.
  int16_t bright = 0;      // Brightness * 64 (ambient dimming of static colors).
  uint8_t online = 0;      // Online: gates the weather layer into the reference.
};
StaticSig g_sig;

struct Rect {
  int16_t x, y, w, h;
};
std::vector<Rect> g_prevRects;  // Moving-element boxes drawn last frame.
std::vector<Rect> g_curRects;   // Moving-element boxes drawn this frame.

// Previous sweep line endpoint; its start is always the panel center.
int16_t g_prevSweepX = 0, g_prevSweepY = 0;
int16_t g_curSweepX = 0, g_curSweepY = 0;
bool g_haveSweep = false;     // A previous sweep line is present to erase.
bool g_curHaveSweep = false;  // This frame drew a sweep (offline suppresses it).

// Offline-to-online handoff: when signal is acquired, the amber title
// fades out and the sweep fades in over these windows, so the switch to
// the live scope is a dissolve rather than a hard cut.
bool g_wasOnline = false;
uint32_t g_onlineSinceMs = 0;  // millis() when online last became true.
constexpr uint32_t kSignalFadeMs = 900;
constexpr uint32_t kSweepFadeMs = 1800;

bool sigEqual(const StaticSig& a, const StaticSig& b) {
  return a.range100 == b.range100 && a.cx == b.cx && a.cy == b.cy &&
         a.mode == b.mode && a.geo == b.geo && a.bright == b.bright &&
         a.online == b.online;
}

StaticSig staticSigOf(const Model& model) {
  float ppn = pixelsPerNm(model);
  StaticSig s;
  s.range100 = static_cast<int32_t>(lroundf(model.ui.range * 100.0f));
  s.cx = static_cast<int16_t>(lroundf(model.ui.viewCenter.x * ppn));
  s.cy = static_cast<int16_t>(lroundf(model.ui.viewCenter.y * ppn));
  s.mode = static_cast<uint8_t>(model.ui.display);
  s.geo = model.ui.geography ? 1 : 0;
  s.bright = static_cast<int16_t>(lroundf(model.ui.brightness * 64.0f));
  s.online = model.ui.online ? 1 : 0;
  return s;
}

// Copies a clamped rectangle from the static reference into the live buffer,
// restoring the static scene under a moving element that is about to shift.
void restoreRect(const Rect& r) {
  int x0 = r.x < 0 ? 0 : r.x;
  int y0 = r.y < 0 ? 0 : r.y;
  int x1 = r.x + r.w;
  int y1 = r.y + r.h;
  if (x1 > g_w) x1 = g_w;
  if (y1 > g_h) y1 = g_h;
  if (x0 >= x1 || y0 >= y1) return;
  size_t bytes = static_cast<size_t>(x1 - x0) * 2;
  for (int y = y0; y < y1; ++y) {
    size_t off = static_cast<size_t>(y) * g_w + x0;
    memcpy(g_liveFb + off, g_staticFb + off, bytes);
  }
}

void restorePixel(int x, int y) {
  if (x < 0 || x >= g_w || y < 0 || y >= g_h) return;
  size_t off = static_cast<size_t>(y) * g_w + x;
  g_liveFb[off] = g_staticFb[off];
}

// Restores the static pixels under the previous sweep line, with a 3x3 brush
// so a one-pixel rasteriser difference cannot leave a glowing trail behind.
void restoreSweep(int x1, int y1) {
  int x0 = static_cast<int>(kCenterX);
  int y0 = static_cast<int>(kCenterY);
  int dx = x1 - x0;
  if (dx < 0) dx = -dx;
  int dy = y1 - y0;
  if (dy < 0) dy = -dy;
  dy = -dy;
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    for (int by = -1; by <= 1; ++by) {
      for (int bx = -1; bx <= 1; ++bx) restorePixel(x0 + bx, y0 + by);
    }
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void restoreAll() {
  memcpy(g_liveFb, g_staticFb, static_cast<size_t>(g_w) * g_h * 2);
}

// Paints the unchanging scene into the static-reference surface. The weather
// layer is suppressed while offline, so the acquiring-signal screen stays
// uncluttered by the placeholder cells; it appears once signal is acquired.
void renderStatic(Model& model) {
  drawBackground(g_static, model);
  if (model.ui.online && showWeather(model)) drawWeather(g_static, model);
  if (model.ui.geography) drawGeography(g_static, model);
}

// Records the bounding box of the element just drawn into the live surface so
// it can be erased next frame. Empty draws (off-screen elements) add nothing.
void endTrackPush() {
  int16_t x, y, w, h;
  if (g_live->endTrack(x, y, w, h)) g_curRects.push_back(Rect{x, y, w, h});
}

// Draws every moving element into the live surface, recording each one's box.
// Order matches the original scene so the composite z-order is unchanged: the
// sweep sits under the markers, which sit under the aircraft and overlay.
void drawDynamic(Model& model) {
  uint32_t onlineAge = millis() - g_onlineSinceMs;

  // The sweep only turns once signal is acquired; while offline it is neither
  // drawn nor tracked, so nothing needs erasing for it next frame. It fades
  // in over the handoff window when signal has just been acquired.
  if (model.ui.online) {
    float fade = onlineAge >= kSweepFadeMs
                     ? 1.0f
                     : onlineAge / static_cast<float>(kSweepFadeMs);
    drawSweep(g_live, model, fade);
    float sx, sy;
    polar(model.ui.sweepAngle, kRadius, sx, sy);
    g_curSweepX = static_cast<int16_t>(sx);
    g_curSweepY = static_cast<int16_t>(sy);
    g_curHaveSweep = true;
  } else {
    g_curHaveSweep = false;
  }

  g_live->beginTrack();
  drawHomeMarker(g_live, model);
  endTrackPush();

  if (showFlights(model)) {
    for (const auto& poi : model.pois) {
      if (distanceFrom(model, poi.pos) > model.ui.range) continue;
      g_live->beginTrack();
      drawPoi(g_live, model, poi);
      endTrackPush();
    }
    for (int i = 0; i < static_cast<int>(model.aircraft.size()); ++i) {
      g_live->beginTrack();
      drawAircraft(g_live, model, model.aircraft[i], i);
      endTrackPush();
    }
  }

  // Toggle feedback flashes on the live scope; offline, the input-test
  // line inside drawAcquiringSignal already covers every control event.
  if (model.ui.online) {
    g_live->beginTrack();
    drawToggleFlash(g_live, model);
    endTrackPush();
  }

  if (!model.ui.online) {
    g_live->beginTrack();
    drawAcquiringSignal(g_live, model, 1.0f);
    endTrackPush();
  } else if (onlineAge < kSignalFadeMs) {
    // Signal was just acquired: dissolve the title into the live scope.
    g_live->beginTrack();
    drawAcquiringSignal(g_live, model,
                        1.0f - onlineAge / static_cast<float>(kSignalFadeMs));
    endTrackPush();
  }
}

// ---- Incremental power on/off transitions. ----
//
// The transitions share the single framebuffer with the panel scan-out, so
// they must never rewrite the whole frame at once — a full-frame write races
// the scan and tears, which is exactly what the old fillScreen-per-frame
// versions did. Instead each frame touches only a small delta: a growing
// shape covers its predecessor, or a band of rows is copied/blanked at the
// edges of the region that changed.

constexpr float kBloomDotEnd = 0.18f;      // Bloom: dot until here...
constexpr float kBloomLineEnd = 0.4f;      // ...line until here, then reveal.
constexpr float kCollapseLineStart = 0.55f;  // Collapse: squeeze until here...
constexpr float kCollapseDotStart = 0.82f;   // ...line until here, then dot.

constexpr uint16_t kGlow = rgb565(200, 255, 220);

int g_transPhase = -1;    // -1 idle; else the current phase of a transition.
Rect g_transShape;        // Bloom dot/line drawn last frame (for hand-offs).
int16_t g_transTop = 0;   // Revealed/surviving row band: first row...
int16_t g_transBot = 0;   // ...and one past the last.
int16_t g_transL = 0;     // Collapse line: first lit column...
int16_t g_transR = 0;     // ...and one past the last.

// One frame of the power-on bloom: a growing dot, a widening line, then the
// scene revealed in a band opening out from the line, with a glowing edge.
// The reveal copies rows from the static reference, so on completion the
// live buffer matches it exactly and the incremental renderer can take over
// without a full repaint.
bool renderBloom(Arduino_GFX* gfx, Model& model, float p) {
  const int16_t cx = static_cast<int16_t>(kCenterX);
  const int16_t cy = static_cast<int16_t>(kCenterY);

  if (g_transPhase < 0) {
    // First frame. The backlight is still off (main.cpp lights it only after
    // this frame is presented), so the one full clear here is never seen
    // mid-write. Bake the scene the reveal will copy from into the static
    // reference now, and adopt its fingerprint so the hand-off is seamless.
    gfx->fillScreen(RGB565_BLACK);
    renderStatic(model);
    g_sig = staticSigOf(model);
    g_needStatic = false;
    g_transShape = Rect{0, 0, 0, 0};
    g_transPhase = 0;
  }

  if (p < kBloomDotEnd) {  // Dot: grows in place, covering its predecessor.
    int r = 2 + static_cast<int>(8 * p / kBloomDotEnd);
    gfx->fillCircle(cx, cy, r, kGlow);
    g_transShape = Rect{static_cast<int16_t>(cx - r),
                        static_cast<int16_t>(cy - r),
                        static_cast<int16_t>(2 * r + 1),
                        static_cast<int16_t>(2 * r + 1)};
    return false;
  }

  if (p < kBloomLineEnd) {  // Line: widens in place over the dot's rows.
    if (g_transPhase == 0) {
      // The dot pokes above and below the line's rows; black it out once.
      gfx->fillRect(g_transShape.x, g_transShape.y, g_transShape.w,
                    g_transShape.h, RGB565_BLACK);
      g_transPhase = 1;
    }
    int w = static_cast<int>(g_w * (p - kBloomDotEnd) /
                             (kBloomLineEnd - kBloomDotEnd));
    if (w < 6) w = 6;
    gfx->fillRect(cx - w / 2, cy - 2, w, 4, kGlow);
    g_transShape = Rect{static_cast<int16_t>(cx - w / 2),
                        static_cast<int16_t>(cy - 2),
                        static_cast<int16_t>(w), 4};
    return false;
  }

  if (g_transPhase != 2) {  // Line hands off to the opening reveal band.
    gfx->fillRect(g_transShape.x, g_transShape.y, g_transShape.w,
                  g_transShape.h, RGB565_BLACK);
    g_transTop = g_transBot = cy;  // Band starts empty at the fold line.
    g_transPhase = 2;
  }

  // Reveal: the band [top, bot) shows the static reference; each frame only
  // the newly exposed rows are copied in, and a 2 px glow edge rides the
  // opening edges (restored to scene rows once they become interior).
  float rp = (p - kBloomLineEnd) / (1.0f - kBloomLineEnd);
  int h = static_cast<int>(g_h * rp);
  int top = cy - h / 2;
  if (top < 0) top = 0;
  int bot = top + h;
  if (bot > g_h) bot = g_h;
  if (top > g_transTop) top = g_transTop;  // Never un-reveal on jitter.
  if (bot < g_transBot) bot = g_transBot;

  if (g_transBot > g_transTop) {  // Consume last frame's edge glow.
    restoreRect(Rect{0, g_transTop, static_cast<int16_t>(g_w), 2});
    restoreRect(Rect{0, static_cast<int16_t>(g_transBot - 2),
                     static_cast<int16_t>(g_w), 2});
  }
  if (top < g_transTop) {
    restoreRect(Rect{0, static_cast<int16_t>(top), static_cast<int16_t>(g_w),
                     static_cast<int16_t>(g_transTop - top)});
  }
  if (bot > g_transBot) {
    restoreRect(Rect{0, g_transBot, static_cast<int16_t>(g_w),
                     static_cast<int16_t>(bot - g_transBot)});
  }
  g_transTop = top;
  g_transBot = bot;

  if (p >= 1.0f) {
    // The band spans the panel and the glow has been consumed: the live
    // buffer now matches the static reference exactly. Hand the scope to
    // the incremental renderer with no pending full repaint — it only has
    // to add the moving elements on top.
    g_prevRects.clear();
    g_haveSweep = false;
    g_needFull = false;
    g_transPhase = -1;
    return true;
  }
  gfx->fillRect(0, top, g_w, 2, kGlow);
  gfx->fillRect(0, bot - 2, g_w, 2, kGlow);
  return false;
}

// One frame of the power-off collapse: the live scene is consumed in place —
// rows blank toward a bright fold line, the line shrinks to a dot, and deep
// sleep (which cuts the backlight) takes it from there. Only the newly
// blanked rows/columns are written each frame.
bool renderCollapse(Arduino_GFX* gfx, float p) {
  const int16_t cx = static_cast<int16_t>(kCenterX);
  const int16_t cy = static_cast<int16_t>(kCenterY);

  if (g_transPhase < 0) {
    g_transTop = 0;  // The whole steady-state frame is still lit.
    g_transBot = g_h;
    g_transL = 0;
    g_transR = g_w;
    g_transPhase = 0;
    gfx->fillRect(0, cy - 2, g_w, 4, kGlow);  // The fold line appears.
  }

  // Squeeze: blank rows from both edges toward the fold line. Runs every
  // frame (a no-op once converged), so a stalled frame can never leave
  // stragglers behind.
  int h = p < kCollapseLineStart
              ? static_cast<int>(g_h * (1.0f - p / kCollapseLineStart))
              : 4;
  if (h < 4) h = 4;
  int top = cy - h / 2;
  int bot = top + h;
  if (top < g_transTop) top = g_transTop;  // Shrink only.
  if (bot > g_transBot) bot = g_transBot;
  if (top > g_transTop) {
    gfx->fillRect(0, g_transTop, g_w, top - g_transTop, RGB565_BLACK);
  }
  if (bot < g_transBot) {
    gfx->fillRect(0, bot, g_w, g_transBot - bot, RGB565_BLACK);
  }
  g_transTop = top;
  g_transBot = bot;

  if (p >= kCollapseLineStart && p < kCollapseDotStart) {
    // The line burns down from both ends toward the center.
    int lw = static_cast<int>(g_w * (1.0f - (p - kCollapseLineStart) /
                                                (kCollapseDotStart -
                                                 kCollapseLineStart)));
    if (lw < 6) lw = 6;
    int l = cx - lw / 2;
    int r = l + lw;
    if (l > g_transL) {
      gfx->fillRect(g_transL, cy - 2, l - g_transL, 4, RGB565_BLACK);
      g_transL = l;
    }
    if (r < g_transR) {
      gfx->fillRect(r, cy - 2, g_transR - r, 4, RGB565_BLACK);
      g_transR = r;
    }
    g_transPhase = 1;
  } else if (p >= kCollapseDotStart && g_transPhase != 2) {
    // What is left of the line becomes the ember dot.
    gfx->fillRect(g_transL, cy - 2, g_transR - g_transL, 4, RGB565_BLACK);
    gfx->fillCircle(cx, cy, 4, kGlow);
    g_transPhase = 2;
  }

  if (p >= 1.0f) {
    g_transPhase = -1;
    return true;
  }
  return false;
}

}  // namespace

void beginRenderer(display::LiveGfx* live, Arduino_GFX* staticRef,
                   uint16_t* liveFb, uint16_t* staticFb, int16_t width,
                   int16_t height) {
  g_live = live;
  g_static = staticRef;
  g_liveFb = liveFb;
  g_staticFb = staticFb;
  g_w = width;
  g_h = height;
  invalidate();
}

void invalidate() {
  g_needStatic = true;
  g_needFull = true;
  g_haveSweep = false;
  g_prevRects.clear();
}

void renderIncremental(Model& model) {
  // Track the offline-to-online edge that drives the handoff dissolve
  // (title fading out, sweep fading in) in drawDynamic.
  if (model.ui.online != g_wasOnline) {
    g_wasOnline = model.ui.online;
    if (model.ui.online) g_onlineSinceMs = millis();
  }

  // Rebuild the static reference when the view, range, mode, geography or
  // brightness changed; that also forces one full repaint of the live buffer.
  StaticSig sig = staticSigOf(model);
  if (g_needStatic || !sigEqual(sig, g_sig)) {
    renderStatic(model);
    g_sig = sig;
    g_needStatic = false;
    g_needFull = true;
  }

  if (g_needFull) {
    restoreAll();  // Establish a known live buffer from the reference.
    g_curRects.clear();
    drawDynamic(model);
    g_prevRects.swap(g_curRects);
    g_prevSweepX = g_curSweepX;
    g_prevSweepY = g_curSweepY;
    g_haveSweep = g_curHaveSweep;
    g_needFull = false;
    return;
  }

  // Erase everything drawn last frame, then redraw at the new positions.
  if (g_haveSweep) restoreSweep(g_prevSweepX, g_prevSweepY);
  for (const auto& r : g_prevRects) restoreRect(r);

  g_curRects.clear();
  drawDynamic(model);

  g_prevRects.swap(g_curRects);
  g_prevSweepX = g_curSweepX;
  g_prevSweepY = g_curSweepY;
  g_haveSweep = g_curHaveSweep;
}

void step(Model& model, uint32_t dtMs) {
  float dt = static_cast<float>(dtMs);

  // Ease the view center toward its target (home origin or followed plane).
  Vec target = targetCenter(model);
  float kEase = 1.0f - expf(-dt / kPanTauMs);
  model.ui.viewCenter.x += (target.x - model.ui.viewCenter.x) * kEase;
  model.ui.viewCenter.y += (target.y - model.ui.viewCenter.y) * kEase;

  // Advance the sweep and refresh whatever it crossed. Derive the angle from
  // absolute time rather than accumulating frame deltas, so frame-timing jitter
  // never makes the sweep step unevenly or appear to jump back and forth.
  float previous = model.ui.sweepAngle;
  uint32_t period = config::SWEEP_PERIOD_MS;
  model.ui.sweepAngle = (millis() % period) * 360.0f / period;
  float decay = expf(-dt / kSweepPersistenceMs);

  for (int i = 0; i < static_cast<int>(model.aircraft.size()); ++i) {
    Aircraft& ac = model.aircraft[i];
    ac.freshness *= decay;
    if (swept(previous, model.ui.sweepAngle, bearingFrom(model, ac.pos))) {
      ac.shown = ac.pos;
      ac.seen = true;
      ac.freshness = 1.0f;
    }
  }
}

bool renderTransition(Arduino_GFX* gfx, Model& model, float progress,
                      bool poweringOn) {
  float p = min(1.0f, progress);
  return poweringOn ? renderBloom(gfx, model, p) : renderCollapse(gfx, p);
}

void showMissingKey(Arduino_GFX* gfx) {
  // Styled like the acquiring-signal title: amber, size 3, centered. Drawn
  // once onto black while the backlight is still off, so it appears whole;
  // main.cpp owns the dwell time and the deep sleep that follows.
  gfx->fillScreen(RGB565_BLACK);
  const char* msg = "MISSING KEY";
  const int len = 11;
  const int size = 3;
  const int charW = 6 * size, charH = 8 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(C_AMBER);
  gfx->setCursor(static_cast<int>(kCenterX) - (len * charW) / 2,
                 static_cast<int>(kCenterY) - charH / 2);
  gfx->print(msg);
}

}  // namespace radar
