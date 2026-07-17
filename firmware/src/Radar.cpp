#include "Radar.h"

#include <math.h>

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
                    dim(i == 4 ? C_TICK : C_RING, k));
  }
  // Bearing ticks every 30 degrees.
  for (int d = 0; d < 360; d += 30) {
    float ox, oy, ix, iy;
    polar(d, kRadius, ox, oy);
    polar(d, kRadius - (d % 90 == 0 ? kRadius * 0.06f : kRadius * 0.035f), ix,
          iy);
    gfx->drawLine(ix, iy, ox, oy, dim(C_TICK, k));
  }
  // Compass letters.
  const char* letters[4] = {"N", "E", "S", "W"};
  int angles[4] = {0, 90, 180, 270};
  gfx->setTextSize(2);
  gfx->setTextColor(dim(C_GREEN, k));
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
              dim(C_RING, k));
  }
}

void drawSweep(Arduino_GFX* gfx, const Model& model) {
  float x, y;
  polar(model.ui.sweepAngle, kRadius, x, y);
  gfx->drawLine(kCenterX, kCenterY, x, y, dim(C_GREEN, model.ui.brightness));
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

void drawAirportsAndPois(Arduino_GFX* gfx, const Model& model) {
  float k = model.ui.brightness;
  for (const auto& poi : model.pois) {
    if (distanceFrom(model, poi.pos) > model.ui.range) continue;
    float x, y;
    project(model, poi.pos, x, y);
    uint16_t c = dim(C_CYAN, k);
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
// in and out, while the scope keeps sweeping behind it.
void drawAcquiringSignal(Arduino_GFX* gfx, const Model& model) {
  float k = model.ui.brightness;
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
}

}  // namespace

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

void renderScene(Arduino_GFX* gfx, Model& model) {
  drawBackground(gfx, model);
  if (showWeather(model)) drawWeather(gfx, model);
  if (model.ui.geography) drawGeography(gfx, model);
  drawSweep(gfx, model);
  drawHomeMarker(gfx, model);
  if (showFlights(model)) {
    drawAirportsAndPois(gfx, model);
    for (int i = 0; i < static_cast<int>(model.aircraft.size()); ++i) {
      drawAircraft(gfx, model, model.aircraft[i], i);
    }
  }
  if (!model.ui.online) drawAcquiringSignal(gfx, model);
}

bool renderTransition(Arduino_GFX* gfx, Model& model, float progress,
                      bool poweringOn) {
  gfx->fillScreen(RGB565_BLACK);
  float p = min(1.0f, progress);
  uint16_t glow = rgb565(200, 255, 220);

  if (!poweringOn) {  // CRT collapse: scene squeezes to a line, then a dot.
    if (p < 0.55f) {
      int h = max(2, static_cast<int>(config::PANEL_HEIGHT * (1 - p / 0.55f)));
      gfx->fillRect(0, kCenterY - h / 2, config::PANEL_WIDTH, h,
                    rgb565(10, 40, 28));
      gfx->fillRect(0, kCenterY - 2, config::PANEL_WIDTH, 4, glow);
    } else if (p < 0.82f) {
      int w = max(6, static_cast<int>(config::PANEL_WIDTH * (1 - (p - 0.55f) / 0.27f)));
      gfx->fillRect(kCenterX - w / 2, kCenterY - 2, w, 4, glow);
    } else {
      gfx->fillCircle(kCenterX, kCenterY, 4, glow);
    }
  } else {  // Bloom: dot, line, then the live scene opens vertically.
    if (p < 0.18f) {
      gfx->fillCircle(kCenterX, kCenterY, 2 + static_cast<int>(8 * p / 0.18f), glow);
    } else if (p < 0.4f) {
      int w = static_cast<int>(config::PANEL_WIDTH * (p - 0.18f) / 0.22f);
      gfx->fillRect(kCenterX - w / 2, kCenterY - 2, max(6, w), 4, glow);
    } else {
      renderScene(gfx, model);  // Reveal is approximate on-panel.
    }
  }
  return p >= 1.0f;
}

}  // namespace radar
