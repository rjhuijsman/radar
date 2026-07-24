#include "Radar.h"

#include <math.h>
#include <string.h>

#include <vector>

#include "Display.h"
#include "airports.h"
#include "config.h"
#include "geo_data.h"

namespace radar {
namespace {

using model::Aircraft;
using model::DisplayMode;
using model::Model;
using model::Vec;

// Blips fade from full brightness to the dim floor linearly over most of a
// sweep rotation, hitting the floor just before the sweep swings back around
// to repaint them — a slow phosphor afterglow instead of a quick blink.
// Expressed as a fraction of one sweep period (config::SWEEP_PERIOD_MS).
constexpr float kBlipFadeSpan = 0.92f;
constexpr float kPanTauMs = 300.0f;
// Easing constant for the followed flight's displayed position; long
// enough that a fix's correction glides in, short enough to feel live.
constexpr float kFollowTauMs = 1200.0f;
// The followed flight's past path: sample its smoothed position this often
// into a bounded trail, drawn as a dotted line behind it.
constexpr uint32_t kTrailSampleMs = 1500;
constexpr int kTrailMaxPoints = 40;  // ~kTrailMaxPoints * kTrailSampleMs of path.

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
// The scope is nudged down from a true centre and the ring shrunk a little,
// so the top of the dial (the "N" and outer ring) clears the case bezel that
// covers the topmost strip of the panel as mounted. The whole image projects
// through these, so it shifts and scales together and stays round.
const float kCenterY = config::PANEL_HEIGHT / 2.0f + 10.0f;
const float kRadius = config::PANEL_WIDTH * 0.435f;

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

// ---- Erase bookkeeping (shared with the incremental renderer below). ----

struct Rect {
  int16_t x, y, w, h;
};
// A line segment erased by walking its pixels instead of restoring its
// bounding box; a leading line's box is large and nearly empty, and box
// restores dominate the erase cost in a busy sky.
struct Seg {
  int16_t x0, y0, x1, y1;
};

display::LiveGfx* g_live = nullptr;  // Draw surface, retargeted per frame.
std::vector<Rect> g_curRects;  // Moving-element boxes drawn this frame.
std::vector<Seg> g_curSegs;    // Leading lines drawn this frame.

// Records the bounding box of the element just drawn into the live surface
// so it can be erased when this framebuffer is next drawn over. Empty draws
// (off-screen elements) add nothing.
void endTrackPush() {
  int16_t x, y, w, h;
  if (g_live->endTrack(x, y, w, h)) g_curRects.push_back(Rect{x, y, w, h});
}

// Bulk-traffic text labels are hidden beyond this range: with the whole
// fetch circle on screen they overlap into noise, and their erase boxes
// dominate the frame budget in a busy sky. Special, selected and followed
// flights keep their labels at any range.
constexpr float kLabelMaxRangeNm = 120.0f;

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

// The ring metalwork, ticks, compass letters and ring labels. The field
// fill itself happens in renderStatic — the weather underlay slots in
// between the fill and this, so the rings read over the rain.
void drawBackground(Arduino_GFX* gfx, const Model& model) {
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

// `at` is where the flight is being drawn (sweep-painted or smoothed), so
// the arrow points at the same spot the blip will appear.
void drawEdgeArrow(Arduino_GFX* gfx, const Model& model, const Aircraft& ac,
                   float distance, const Vec& at) {
  uint16_t c = dim(ac.special ? C_AMBER : C_WHITE, model.ui.brightness);
  float bearing = bearingFrom(model, at);
  float x, y;
  polar(bearing, kRadius * 0.985f, x, y);
  drawTriangle(gfx, x, y, bearing + 180, 1.1f, c);  // Chevron points out.
  float lx, ly;
  polar(bearing, kRadius * 0.86f, lx, ly);
  drawLabel(gfx, lx - 16, ly - 6, ac.callsign, c);
  drawLabel(gfx, lx - 16, ly + 6, String(static_cast<int>(distance)) + " NM", c);
}

// Draws one aircraft, recording its own erase bookkeeping: the leading
// line as a segment, everything else as a tracked box.
void drawAircraft(Arduino_GFX* gfx, const Model& model, Aircraft& ac,
                  int index) {
  bool followed = model.ui.following && index == model.ui.followIndex;
  bool selected = !model.ui.following && index == model.ui.browseSel;
  bool candidate = model.ui.following && index == model.ui.candidate;

  // The followed flight draws at its smoothed estimate (dead-reckoned and
  // eased in step()), everything else where the sweep last painted it.
  const Vec& at = followed ? ac.est : ac.shown;
  if (!followed && !ac.seen) return;

  float distance = distanceFrom(model, at);
  if (distance > model.ui.range) {
    // A followed flight can sit outside the circle while the view is still
    // panning to it; the arrow keeps it pointed at until the pan lands.
    if (ac.special || selected || candidate || followed) {
      g_live->beginTrack();
      drawEdgeArrow(gfx, model, ac, distance, at);
      endTrackPush();
    }
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
  g_curSegs.push_back(Seg{static_cast<int16_t>(x), static_cast<int16_t>(y),
                          static_cast<int16_t>(tx), static_cast<int16_t>(ty)});

  g_live->beginTrack();
  if (ac.special) gfx->drawCircle(x, y, 13, dim(C_AMBER, k * bright));
  drawTriangle(gfx, x, y, ac.track, followed ? 1.2f : 1.0f, color);

  if (model.ui.range <= kLabelMaxRangeNm || followed || selected ||
      candidate || ac.special) {
    drawLabel(gfx, x + 12, y - 4, ac.callsign, color);
    String block = String(ac.altitude) + "  " +
                   String(static_cast<int>(ac.groundSpeed)) + "kt";
    drawLabel(gfx, x + 12, y + 8, block, dim(C_WHITE, k * bright));
  }

  if (selected || candidate) {
    uint16_t rc = dim(candidate ? C_AMBER : C_WHITE, k);
    gfx->drawRect(x - 17, y - 17, 34, 34, rc);  // Selection reticle.
  }
  if (followed) drawLabel(gfx, x + 12, y - 16, "FOLLOW", dim(C_AMBER, k));
  endTrackPush();
}

// ---- Geography overlay (embedded Natural Earth basemap). ----
//
// European coastlines and country borders from geo_data.h (generated
// by tools/make_geo_data.py), stored as 1/400-degree fixed-point
// polylines in flash. The overlay draws into the static reference
// only, so the continent-wide scan runs once per view/range/mode
// rebuild — never per frame.

constexpr float kGeoClipPad = 8.0f;  // Clip box: the panel plus a margin.

// Outcode of a screen point against the padded panel box, for
// Cohen-Sutherland segment clipping.
uint8_t geoOutcode(float x, float y) {
  uint8_t code = 0;
  if (x < -kGeoClipPad) code |= 1;
  if (x > config::PANEL_WIDTH + kGeoClipPad) code |= 2;
  if (y < -kGeoClipPad) code |= 4;
  if (y > config::PANEL_HEIGHT + kGeoClipPad) code |= 8;
  return code;
}

// Clips a segment to the padded panel box; false means fully outside.
// Zoomed in, an unclipped vertex can sit hundreds of thousands of
// pixels off-screen — far past what drawLine's int16 coordinates can
// carry — so every drawn segment must pass through here.
bool geoClipSegment(float& x0, float& y0, float& x1, float& y1, uint8_t c0,
                    uint8_t c1) {
  for (;;) {
    if ((c0 | c1) == 0) return true;  // Both endpoints inside.
    if (c0 & c1) return false;        // Both off the same side.
    uint8_t out = c0 != 0 ? c0 : c1;
    float x, y;
    if (out & 1) {  // Off the left edge.
      x = -kGeoClipPad;
      y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    } else if (out & 2) {  // Right.
      x = config::PANEL_WIDTH + kGeoClipPad;
      y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    } else if (out & 4) {  // Top.
      y = -kGeoClipPad;
      x = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    } else {  // Bottom.
      y = config::PANEL_HEIGHT + kGeoClipPad;
      x = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    }
    if (out == c0) {
      x0 = x;
      y0 = y;
      c0 = geoOutcode(x0, y0);
    } else {
      x1 = x;
      y1 = y;
      c1 = geoOutcode(x1, y1);
    }
  }
}

// Draws one polyline set through the folded projection sx = lon*ax+bx,
// sy = lat*ay+by (lat/lon in fixed-point units): two multiply-adds
// and an outcode per vertex, so culling the whole of Europe down to
// the visible segments costs almost nothing.
void drawGeoSet(Arduino_GFX* gfx, const int16_t (*pts)[2],
                const uint16_t* lens, int lineCount, float ax, float bx,
                float ay, float by, uint16_t color) {
  int base = 0;
  for (int line = 0; line < lineCount; ++line) {
    int n = lens[line];
    float px = pts[base][1] * ax + bx;
    float py = pts[base][0] * ay + by;
    uint8_t pc = geoOutcode(px, py);
    for (int i = base + 1; i < base + n; ++i) {
      float cx = pts[i][1] * ax + bx;
      float cy = pts[i][0] * ay + by;
      uint8_t cc = geoOutcode(cx, cy);
      float x0 = px, y0 = py, x1 = cx, y1 = cy;
      if (geoClipSegment(x0, y0, x1, y1, pc, cc)) {
        gfx->drawLine(static_cast<int16_t>(x0), static_cast<int16_t>(y0),
                      static_cast<int16_t>(x1), static_cast<int16_t>(y1),
                      color);
      }
      px = cx;
      py = cy;
      pc = cc;
    }
    base += n;
  }
}

// Draws the coastlines and country borders around the active home.
// Dim cyan in both palettes — it reads as background terrain under
// the green and the blue phosphor alike — with borders a step dimmer
// than the coast so the physical outline dominates.
void drawGeography(Arduino_GFX* gfx, const Model& model) {
  // Anchor the projection at the active home, matching the feeds'
  // geoToWorld: world NM east/north via the local equirectangular
  // approximation, then project()'s view-center shift and NM-to-pixel
  // scale, all folded into one multiply-add per axis.
  float homeLat = 0, homeLon = 0;
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    homeLat = model.homes[model.ui.homeIndex].latitude;
    homeLon = model.homes[model.ui.homeIndex].longitude;
  }
  float ppn = pixelsPerNm(model);
  float lonNm = 60.0f * cosf(homeLat * DEG_TO_RAD);  // NM per degree lon.
  float ax = geodata::kDegPerUnit * lonNm * ppn;
  float bx = kCenterX - (homeLon * lonNm + model.ui.viewCenter.x) * ppn;
  float ay = -geodata::kDegPerUnit * 60.0f * ppn;
  float by = kCenterY + (homeLat * 60.0f + model.ui.viewCenter.y) * ppn;

  float k = model.ui.brightness;
  drawGeoSet(gfx, geodata::kBorderPts, geodata::kBorderLen,
             geodata::kBorderLines, ax, bx, ay, by, dim(C_CYAN, k * 0.35f));
  drawGeoSet(gfx, geodata::kCoastPts, geodata::kCoastLen,
             geodata::kCoastLines, ax, bx, ay, by, dim(C_CYAN, k * 0.55f));
}

// ---- Airport basemap (embedded OurAirports major airports). ----
//
// Major European airports from airports.h (generated by
// tools/make_airports.py), drawn in the design's airport symbology:
// a cyan ring with runway strips at their real headings, plus the
// IATA code. Airports are fixed geographic points, so they render
// into the static reference next to the geography — never per frame.

// Medium airports appear only zoomed in; wide views keep the large
// hubs alone so the basemap never crowds the traffic.
constexpr float kMediumAirportMaxRangeNm = 80.0f;

// An embedded airport this close (NM) to the active home or to a user
// airport POI is skipped: the home marker and drawPoi own that spot.
constexpr float kAirportDedupeNm = 1.5f;

// Draws the airports around the active home. Sits a step above the
// geography's dim cyan and below the full-cyan user POIs, so the
// basemap hierarchy reads terrain, then airports, then your places.
void drawAirports(Arduino_GFX* gfx, const Model& model) {
  // The same home-anchored fold as drawGeography: screen x/y are one
  // multiply-add from the fixed-point lat/lon.
  float homeLat = 0, homeLon = 0;
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    homeLat = model.homes[model.ui.homeIndex].latitude;
    homeLon = model.homes[model.ui.homeIndex].longitude;
  }
  float ppn = pixelsPerNm(model);
  float lonNm = 60.0f * cosf(homeLat * DEG_TO_RAD);  // NM per degree lon.
  float ax = airportdata::kDegPerUnit * lonNm * ppn;
  float bx = kCenterX - (homeLon * lonNm + model.ui.viewCenter.x) * ppn;
  float ay = -airportdata::kDegPerUnit * 60.0f * ppn;
  float by = kCenterY + (homeLat * 60.0f + model.ui.viewCenter.y) * ppn;

  // Declutter: medium fields reveal only when zoomed in, and the IATA
  // labels obey the same cutoff as the bulk-traffic labels.
  bool labels = model.ui.range <= kLabelMaxRangeNm;
  int count = model.ui.range <= kMediumAirportMaxRangeNm
                  ? airportdata::kCount
                  : airportdata::kLargeCount;
  float k = model.ui.brightness;
  uint16_t glyph = dim(C_CYAN, k * 0.8f);
  uint16_t text = dim(C_CYAN, k * 0.65f);
  gfx->setTextSize(1);
  gfx->setTextColor(text);
  for (int i = 0; i < count; ++i) {
    const airportdata::Airport& ap = airportdata::kAirports[i];
    float x = ap.lon * ax + bx;
    float y = ap.lat * ay + by;
    // Cheap cull, with margin for the glyph and label.
    if (x < -20 || x > config::PANEL_WIDTH + 20 || y < -20 ||
        y > config::PANEL_HEIGHT + 20) {
      continue;
    }
    // World NM from home, for the double-draw checks below.
    float east = (ap.lon * airportdata::kDegPerUnit - homeLon) * lonNm;
    float north = (ap.lat * airportdata::kDegPerUnit - homeLat) * 60.0f;
    if (hypotf(east, north) < kAirportDedupeNm) continue;  // Home is here.
    bool owned = false;
    for (const auto& poi : model.pois) {
      if (poi.isAirport &&
          hypotf(east - poi.pos.x, north - poi.pos.y) < kAirportDedupeNm) {
        owned = true;  // The user's POI already draws this airport.
        break;
      }
    }
    if (owned) continue;

    // Medium fields draw a size down from the large hubs.
    float ring = i < airportdata::kLargeCount ? 9.0f : 6.0f;
    float strip = i < airportdata::kLargeCount ? 7.0f : 5.0f;
    gfx->drawCircle(x, y, ring, glyph);
    for (int r = 0; r < ap.rwyCount; ++r) {
      float a =
          (airportdata::kRunwayHeadings[ap.rwyStart + r] - 90.0f) * DEG_TO_RAD;
      gfx->drawLine(x - cosf(a) * strip, y - sinf(a) * strip,
                    x + cosf(a) * strip, y + sinf(a) * strip, glyph);
    }
    if (labels) {
      gfx->setCursor(static_cast<int16_t>(x) - 9,
                     static_cast<int16_t>(y) + ring + 4);
      gfx->print(ap.iata);
    }
  }
}

// ---- Weather underlay (RainViewer reflectivity mosaic). ----
//
// feeds::pollWeather publishes model.weather: real radar reflectivity
// decoded into a Web-Mercator pixel mosaic. The scope samples it on a
// coarse cell grid — rain is diffuse, so cells a fiftieth of the panel
// wide read as weather, not pixels — mapping each cell center through
// the same home-anchored equirectangular world the feeds use, then
// through exact Mercator once per row. Draws into the static reference
// only, so the sampling runs per rebuild, never per frame.

constexpr int kWxCellPx = 15;  // Cell size; 48 cells across the panel.
// A layer this old is worse than none — the sky has moved on. It also
// stops counting toward the static-scene fingerprint (staticSigOf), so
// the expiry itself repaints the rain away.
constexpr uint32_t kWxStaleMs = 30 * 60 * 1000;

// Reflectivity tiers, in the layer's dBZ + 32 encoding. Below the first
// is drizzle and noise the scope leaves dark.
constexpr uint8_t kWxLight = 42;     // 10 dBZ.
constexpr uint8_t kWxModerate = 57;  // 25 dBZ.
constexpr uint8_t kWxHeavy = 67;     // 35 dBZ.
constexpr uint8_t kWxIntense = 77;   // 45 dBZ.

// The mockup's rain treatment as opaque RGB565: dim green through green
// to an amber then red core, each pre-blended toward C_BG at the mockup
// underlay's translucency, so the wash reads as sitting behind the ring
// metalwork (which renderStatic draws over it).
constexpr uint16_t C_WX_LIGHT = rgb565(22, 62, 43);
constexpr uint16_t C_WX_MODERATE = rgb565(56, 108, 62);
constexpr uint16_t C_WX_HEAVY = rgb565(110, 96, 34);
constexpr uint16_t C_WX_INTENSE = rgb565(110, 50, 46);

bool weatherUsable(const Model& model) {
  return model.weather.cells != nullptr &&
         millis() - model.weather.fetchedMs < kWxStaleMs;
}

void drawWeather(Arduino_GFX* gfx, const Model& model) {
  if (!weatherUsable(model)) return;  // No data (yet): a clear scope.
  const model::WeatherLayer& wx = model.weather;

  float homeLat = 0, homeLon = 0;
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    homeLat = model.homes[model.ui.homeIndex].latitude;
    homeLon = model.homes[model.ui.homeIndex].longitude;
  }
  float ppn = pixelsPerNm(model);
  float cosLat = cosf(homeLat * DEG_TO_RAD);
  float scale = 256.0f * static_cast<float>(1 << wx.zoom);

  // Screen x to global Mercator pixel x is one multiply-add (longitude
  // is linear in screen x under the feeds' equirectangular world);
  // latitude goes through exact Mercator once per cell row.
  float ax = scale / (360.0f * 60.0f * cosLat * ppn);
  float bx = (homeLon +
              (model.ui.viewCenter.x - kCenterX / ppn) / (60.0f * cosLat) +
              180.0f) /
                 360.0f * scale -
             static_cast<float>(wx.originX);

  float k = model.ui.brightness;
  uint16_t tier[4] = {dim(C_WX_LIGHT, k), dim(C_WX_MODERATE, k),
                      dim(C_WX_HEAVY, k), dim(C_WX_INTENSE, k)};
  float r2 = kRadius * kRadius;

  for (int y0 = 0; y0 < config::PANEL_HEIGHT; y0 += kWxCellPx) {
    float cy = y0 + kWxCellPx * 0.5f;
    float dy = cy - kCenterY;
    float worldY = model.ui.viewCenter.y + (kCenterY - cy) / ppn;
    float lat = homeLat + worldY / 60.0f;
    if (lat > 85.0f || lat < -85.0f) continue;
    float merc = asinhf(tanf(lat * DEG_TO_RAD));
    float fy = (1.0f - merc / static_cast<float>(M_PI)) * 0.5f * scale -
               static_cast<float>(wx.originY);
    if (fy < 0 || fy >= wx.height) continue;
    const uint8_t* row = wx.cells + static_cast<size_t>(fy) * wx.width;
    for (int x0 = 0; x0 < config::PANEL_WIDTH; x0 += kWxCellPx) {
      float cx = x0 + kWxCellPx * 0.5f;
      float dx = cx - kCenterX;
      if (dx * dx + dy * dy > r2) continue;  // Outside the scope disc.
      float fx = ax * cx + bx;
      if (fx < 0 || fx >= wx.width) continue;
      int ix = static_cast<int>(fx);
      uint8_t v = row[ix];
      if (v < kWxLight) continue;  // No echo (or drizzle below the floor).
      int t = v >= kWxIntense ? 3 : v >= kWxHeavy ? 2 : v >= kWxModerate ? 1 : 0;
      gfx->fillRect(x0, y0, kWxCellPx, kWxCellPx, tier[t]);
    }
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
  } else if (model.ui.lastInput.startsWith("GEOGRAPHY") ||
             model.ui.lastInput.startsWith("HOME")) {
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

// ---- Incremental (dirty-rect) renderer, double-buffered. ----
//
// The static reference holds the unchanging scene (background, rings, weather,
// geography, airports). Each frame we draw into the BACK framebuffer: erase
// the moving elements that buffer showed when it was last drawn (two frames
// ago) by copying those regions back from the reference, redraw them at new
// positions, then flip. The driver latches the flip only at a frame boundary,
// so the panel never scans a partially drawn frame — with a single shared
// buffer the scan raced the erase-then-redraw pass, which showed as flickering
// blips (worst in the late-scanned bottom half) and a half-erased sweep.
// Erasing every old box before drawing any new one keeps overlaps correct.

Arduino_GFX* g_static = nullptr;  // Off-screen static-reference surface.
uint16_t* g_targetFb = nullptr;   // Framebuffer the restores write into.
uint16_t* g_staticFb = nullptr;
int g_w = 0, g_h = 0;

bool g_needStatic = true;  // Rebuild the static reference before the next frame.

// Rebuilding the reference and repainting a framebuffer from it are ~90 ms
// each at PSRAM copy speed, and continuous zooming or panning changes the
// signature every frame. Rate-limiting the rebuild keeps the scope live
// during the gesture: between rebuilds the moving elements render at the
// new scale over the briefly stale static scene, and the final state lands
// once the input settles.
constexpr uint32_t kRebuildMinIntervalMs = 200;
uint32_t g_lastRebuildMs = 0;

// The range readout stays emphasized this long after the last zoom detent.
constexpr uint32_t kZoomEmphasisMs = 1500;

// Fingerprint of everything the static reference depends on. When it changes,
// the reference is rebuilt and both framebuffers are repainted from it.
struct StaticSig {
  int32_t range100 = 0;  // Range in NM * 100 (ring labels, projection scale).
  int16_t cx = 0, cy = 0;  // View center in pixels (weather/geography shift).
  uint8_t mode = 0;        // Display mode (weather layer on/off).
  uint8_t geo = 0;         // Geography overlay on/off.
  int16_t bright = 0;      // Brightness * 16 (ambient dimming of static colors).
  uint8_t online = 0;      // Online: gates the weather layer into the reference.
  int8_t home = 0;         // Active home (geography anchor, name label).
  uint8_t emph = 0;        // Range readout emphasized (recent zoom detent).
  uint32_t wxGen = 0;      // Weather layer generation, while it is drawn.
  int16_t trail = 0;       // Followed-flight trail length (grows as it flies).
};
StaticSig g_sig;

// What each framebuffer currently shows, so rendering into it again (two
// frames later) can erase exactly that. `dirty` marks a buffer whose whole
// content must be repainted from the static reference first.
struct FbState {
  std::vector<Rect> rects;  // Moving-element boxes.
  std::vector<Seg> segs;    // Leading lines.
  int16_t sweepX = 0, sweepY = 0;  // Sweep endpoint (start is the center).
  bool haveSweep = false;
  bool dirty = true;
};
FbState g_fb[2];

int16_t g_curSweepX = 0, g_curSweepY = 0;
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
         a.online == b.online && a.home == b.home && a.emph == b.emph &&
         a.wxGen == b.wxGen && a.trail == b.trail;
}

StaticSig staticSigOf(const Model& model) {
  float ppn = pixelsPerNm(model);
  StaticSig s;
  s.range100 = static_cast<int32_t>(lroundf(model.ui.range * 100.0f));
  s.cx = static_cast<int16_t>(lroundf(model.ui.viewCenter.x * ppn));
  s.cy = static_cast<int16_t>(lroundf(model.ui.viewCenter.y * ppn));
  s.mode = static_cast<uint8_t>(model.ui.display);
  s.geo = model.ui.geography ? 1 : 0;
  // Coarse brightness steps, so ambient dither cannot trigger a ~180 ms
  // rebuild-and-repaint every second.
  s.bright = static_cast<int16_t>(lroundf(model.ui.brightness * 16.0f));
  s.online = model.ui.online ? 1 : 0;
  s.home = static_cast<int8_t>(model.ui.homeIndex);
  s.emph = millis() - model.ui.lastZoomMs < kZoomEmphasisMs ? 1 : 0;
  // A newly published weather layer re-fingerprints the scene, but only
  // while the layer is actually drawn; its expiry drops the generation
  // back to 0, which repaints the stale rain away.
  if (model.ui.online && showWeather(model) && weatherUsable(model)) {
    s.wxGen = model.weather.generation;
  }
  s.trail = static_cast<int16_t>(model.ui.followTrail.size());
  return s;
}

// Points the restore helpers and the draw surface at one framebuffer: the
// back buffer for a steady incremental frame, or the front (scanned) one
// for the deliberately in-place power transitions.
void bindTarget(uint16_t* fb) {
  g_targetFb = fb;
  g_live->retarget(fb);
}

// Copies a clamped rectangle from the static reference into the target
// buffer, restoring the static scene under a moving element that shifted.
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
    memcpy(g_targetFb + off, g_staticFb + off, bytes);
  }
}

void restorePixel(int x, int y) {
  if (x < 0 || x >= g_w || y < 0 || y >= g_h) return;
  size_t off = static_cast<size_t>(y) * g_w + x;
  g_targetFb[off] = g_staticFb[off];
}

// Restores the static pixels under a previously drawn line (the sweep or a
// leading line), with a 3x3 brush so a one-pixel rasteriser difference
// cannot leave a glowing trail behind.
void restoreSeg(int x0, int y0, int x1, int y1) {
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
  memcpy(g_targetFb, g_staticFb, static_cast<size_t>(g_w) * g_h * 2);
}

// The active home's name, pinned just below the home marker. Part of the
// static scene: it only moves on a view or home change, both of which
// rebuild the reference.
void drawHomeName(Arduino_GFX* gfx, const Model& model) {
  if (model.ui.homeIndex < 0 ||
      model.ui.homeIndex >= static_cast<int>(model.homes.size())) {
    return;
  }
  float x, y;
  project(model, Vec{0, 0}, x, y);
  if (hypotf(x - kCenterX, y - kCenterY) > kRadius) return;
  const String& name = model.homes[model.ui.homeIndex].name;
  gfx->setTextSize(1);
  gfx->setTextColor(dim(C_WHITE, model.ui.brightness * 0.85f));
  gfx->setCursor(static_cast<int16_t>(x) - static_cast<int>(name.length()) * 3,
                 static_cast<int16_t>(y) + 16);
  gfx->print(name);
}

// The current-range readout: the outer ring's range in large text near the
// bottom of the scope, so the zoom level reads at a glance (the tiny ring
// labels do not). Bright phosphor while actively zooming, receding to the
// ring metalwork's dim once the knob rests — the emphasis flip 1.5 s later
// re-fingerprints the static scene and rebuilds it dim.
void drawRangeReadout(Arduino_GFX* gfx, const Model& model) {
  bool emph = millis() - model.ui.lastZoomMs < kZoomEmphasisMs;
  String text = String(static_cast<int>(lroundf(model.ui.range))) + " NM";
  const int size = 3;
  const int charW = 6 * size;
  float k = model.ui.brightness;
  gfx->setTextSize(size);
  gfx->setTextColor(emph ? dim(phosphorColor(model), k)
                         : dim(tickColor(model), k));
  gfx->setCursor(static_cast<int>(kCenterX) -
                     (static_cast<int>(text.length()) * charW) / 2,
                 static_cast<int>(kCenterY + kRadius * 0.72f));
  gfx->print(text);
}

// Paints the unchanging scene into the static-reference surface. The rain
// underlay goes down first, straight onto the field, so the ring
// metalwork, geography and labels all read over it (the mockup's
// layering; the sweep and blips draw over everything later). It is
// suppressed while offline, keeping the acquiring-signal screen clean.
// The followed flight's recent path, as a dotted line trailing behind it.
// World-anchored like the geography, so it lives in the static scene and
// scrolls with the view as the map tracks the flight.
void drawFollowTrail(Arduino_GFX* gfx, const Model& model) {
  const auto& trail = model.ui.followTrail;
  if (trail.size() < 2) return;
  bool special = model.ui.followIndex >= 0 &&
                 model.ui.followIndex <
                     static_cast<int>(model.aircraft.size()) &&
                 model.aircraft[model.ui.followIndex].special;
  uint16_t c = dim(special ? C_AMBER : C_GREEN, model.ui.brightness * 0.7f);
  float r2 = kRadius * kRadius;
  float px = 0, py = 0;
  bool have = false;
  for (size_t i = 0; i < trail.size(); ++i) {
    float x, y;
    project(model, trail[i], x, y);
    if (have) {  // A 3 px dot every ~7 px along the segment, inside the scope.
      float dx = x - px, dy = y - py;
      int steps = static_cast<int>(sqrtf(dx * dx + dy * dy) / 7.0f);
      for (int s = 1; s <= steps; ++s) {
        float t = static_cast<float>(s) / steps;
        float xp = px + dx * t, yp = py + dy * t;
        float ox = xp - kCenterX, oy = yp - kCenterY;
        if (ox * ox + oy * oy <= r2) {
          gfx->fillCircle(static_cast<int16_t>(xp), static_cast<int16_t>(yp), 1,
                          c);
        }
      }
    }
    px = x;
    py = y;
    have = true;
  }
}

void renderStatic(Model& model) {
  g_static->fillScreen(C_BG);
  if (model.ui.online && showWeather(model)) drawWeather(g_static, model);
  drawBackground(g_static, model);
  if (model.ui.geography) drawGeography(g_static, model);
  // Airports ride the flights layer (they hide in weather-only mode,
  // like the traffic and POIs), independent of the Geography toggle.
  if (showFlights(model)) drawAirports(g_static, model);
  if (model.ui.following) drawFollowTrail(g_static, model);
  drawHomeName(g_static, model);
  drawRangeReadout(g_static, model);
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
      drawAircraft(g_live, model, model.aircraft[i], i);  // Tracks itself.
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
// The transitions draw in place on the front framebuffer, sharing it with
// the panel scan-out, so they must never rewrite the whole frame at once —
// a full-frame write races the scan and tears, which is exactly what the
// old fillScreen-per-frame versions did. Instead each frame touches only a
// small delta: a growing shape covers its predecessor, or a band of rows
// is copied/blanked at the edges of the region that changed.

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
    // The band spans the panel and the glow has been consumed: the front
    // buffer now matches the static reference exactly. Hand the scope to
    // the incremental renderer with that buffer clean — it only has to add
    // the moving elements on top. The back buffer still holds boot black
    // and stays dirty, so its first frame repaints it from the reference.
    FbState& front = g_fb[1 - display::backIndex()];
    front.rects.clear();
    front.segs.clear();
    front.haveSweep = false;
    front.dirty = false;
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
                   uint16_t* staticFb, int16_t width, int16_t height) {
  g_live = live;
  g_static = staticRef;
  g_staticFb = staticFb;
  g_targetFb = display::frontFb();
  g_w = width;
  g_h = height;
  invalidate();
}

void invalidate() {
  g_needStatic = true;
  for (FbState& fb : g_fb) {
    fb.dirty = true;
    fb.haveSweep = false;
    fb.rects.clear();
    fb.segs.clear();
  }
}

void renderIncremental(Model& model) {
  // Track the offline-to-online edge that drives the handoff dissolve
  // (title fading out, sweep fading in) in drawDynamic.
  if (model.ui.online != g_wasOnline) {
    g_wasOnline = model.ui.online;
    if (model.ui.online) g_onlineSinceMs = millis();
  }

  // Rebuild the static reference when the view, range, mode, geography,
  // home or brightness changed; that marks both framebuffers for a full
  // repaint from it. Rate-limited (see kRebuildMinIntervalMs): a pending
  // change is picked up here on a later frame once the interval allows.
  StaticSig sig = staticSigOf(model);
  if (g_needStatic || !sigEqual(sig, g_sig)) {
    uint32_t now = millis();
    if (g_needStatic || now - g_lastRebuildMs >= kRebuildMinIntervalMs) {
      renderStatic(model);
      g_sig = sig;
      g_needStatic = false;
      g_lastRebuildMs = now;
      g_fb[0].dirty = true;
      g_fb[1].dirty = true;
    }
  }

  // Draw this frame into the back buffer; the panel keeps scanning the
  // front one until the flip below is latched at a frame boundary.
  FbState& fb = g_fb[display::backIndex()];
  bindTarget(display::backFb());

  if (fb.dirty) {
    restoreAll();  // Repaint the whole buffer from the reference.
    fb.rects.clear();
    fb.segs.clear();
    fb.haveSweep = false;
    fb.dirty = false;
  } else {
    // Erase everything this buffer showed when it was last drawn (two
    // frames ago), then redraw at the new positions below.
    if (fb.haveSweep) {
      restoreSeg(static_cast<int>(kCenterX), static_cast<int>(kCenterY),
                 fb.sweepX, fb.sweepY);
    }
    for (const auto& s : fb.segs) restoreSeg(s.x0, s.y0, s.x1, s.y1);
    for (const auto& r : fb.rects) restoreRect(r);
  }

  g_curRects.clear();
  g_curSegs.clear();
  drawDynamic(model);

  fb.rects.swap(g_curRects);
  fb.segs.swap(g_curSegs);
  fb.sweepX = g_curSweepX;
  fb.sweepY = g_curSweepY;
  fb.haveSweep = g_curHaveSweep;

  display::flip();  // Scan out this frame from the next boundary on.
}

void step(Model& model, uint32_t dtMs) {
  float dt = static_cast<float>(dtMs);
  uint32_t now = millis();

  // Smooth the followed flight: dead-reckon from its last fix along the
  // track, and ease the displayed position through each fix's correction,
  // so new data arriving at the sweep top shifts the blip smoothly instead
  // of snapping it.
  bool haveFollow = model.ui.following && model.ui.followIndex >= 0 &&
                    model.ui.followIndex < static_cast<int>(model.aircraft.size());
  if (haveFollow) {
    Aircraft& ac = model.aircraft[model.ui.followIndex];
    // Cap the extrapolation so a stale fix cannot fly the blip away.
    float ageS = min((now - ac.fixMs) / 1000.0f, 90.0f);
    float a = ac.track * DEG_TO_RAD;
    float nmPerS = ac.groundSpeed / 3600.0f;
    Vec reckoned{ac.pos.x + sinf(a) * nmPerS * ageS,
                 ac.pos.y + cosf(a) * nmPerS * ageS};
    float kF = 1.0f - expf(-dt / kFollowTauMs);
    ac.est.x += (reckoned.x - ac.est.x) * kF;
    ac.est.y += (reckoned.y - ac.est.y) * kF;

    // Sample the smoothed position into the bounded past-path trail.
    if (now - model.ui.lastTrailMs >= kTrailSampleMs) {
      model.ui.lastTrailMs = now;
      model.ui.followTrail.push_back(ac.est);
      if (static_cast<int>(model.ui.followTrail.size()) > kTrailMaxPoints) {
        model.ui.followTrail.erase(model.ui.followTrail.begin());
      }
    }
  }

  // Ease the view center toward its target: the home origin when idle
  // (chased continuously, snapping once converged so the static-scene
  // signature settles), or the followed flight when following (tracked
  // continuously so the map moves along with it). A follow paused by weather
  // mode instead FREEZES the view where it is — the map holds the flight's
  // last spot until tracking resumes or a deliberate go-home releases it.
  Vec target{0, 0};
  bool freeze = false;
  if (haveFollow) {
    target = model.aircraft[model.ui.followIndex].est;
  } else if (model.ui.pausedFollow.length() > 0) {
    freeze = true;
  }
  if (!freeze) {
    float kEase = 1.0f - expf(-dt / kPanTauMs);
    model.ui.viewCenter.x += (target.x - model.ui.viewCenter.x) * kEase;
    model.ui.viewCenter.y += (target.y - model.ui.viewCenter.y) * kEase;
    float ppn = pixelsPerNm(model);
    if (fabsf(target.x - model.ui.viewCenter.x) * ppn < 0.5f &&
        fabsf(target.y - model.ui.viewCenter.y) * ppn < 0.5f) {
      model.ui.viewCenter = target;  // Sub-pixel: stop dithering the scene.
    }
  }

  // Advance the sweep and refresh whatever it crossed. Derive the angle from
  // absolute time rather than accumulating frame deltas, so frame-timing jitter
  // never makes the sweep step unevenly or appear to jump back and forth.
  float previous = model.ui.sweepAngle;
  uint32_t period = config::SWEEP_PERIOD_MS;
  model.ui.sweepAngle = (millis() % period) * 360.0f / period;

  // The sweep crossing 12 o'clock requests a traffic poll: the sweep
  // period equals the poll interval, so fresh data is phase-locked to the
  // sweep reaching top — one rotation, one poll.
  if (model.ui.sweepAngle < previous) model.adsbPollDue = true;

  float fadeStep = dt / (config::SWEEP_PERIOD_MS * kBlipFadeSpan);

  for (int i = 0; i < static_cast<int>(model.aircraft.size()); ++i) {
    Aircraft& ac = model.aircraft[i];
    ac.freshness -= fadeStep;
    if (ac.freshness < 0.0f) ac.freshness = 0.0f;
    if (swept(previous, model.ui.sweepAngle, bearingFrom(model, ac.pos))) {
      ac.shown = ac.pos;
      ac.seen = true;
      ac.freshness = 1.0f;
    }
  }
}

bool renderTransition(Arduino_GFX* gfx, Model& model, float progress,
                      bool poweringOn) {
  // The transitions deliberately draw in place on the scanned (front)
  // buffer: each frame writes only a small delta, which cannot tear.
  bindTarget(display::frontFb());
  float p = min(1.0f, progress);
  return poweringOn ? renderBloom(gfx, model, p) : renderCollapse(gfx, p);
}

void showMissingKey(Arduino_GFX* gfx) {
  // Styled like the acquiring-signal title: amber, size 3, centered. Drawn
  // once onto black on the scanned buffer while the backlight is still off,
  // so it appears whole; main.cpp owns the dwell time and the deep sleep
  // that follows.
  bindTarget(display::frontFb());
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
