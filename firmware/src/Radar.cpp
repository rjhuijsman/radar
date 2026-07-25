#include "Radar.h"

#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

#include <utility>
#include <vector>

#include "Display.h"
#include "airports.h"
#include "cities.h"
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
// Holds the historic seed (<= kTraceSeedPoints from Feeds) plus a long live
// tail without eroding the seeded prefix; the renderer is the trail's sole
// owner (Feeds only prepends the history once, on follow).
constexpr int kTrailMaxPoints = 250;

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
constexpr uint16_t C_GREEN = rgb565(61, 255, 164);
constexpr uint16_t C_CYAN = rgb565(95, 220, 255);
constexpr uint16_t C_AMBER = rgb565(255, 194, 74);
// City markers: a warm phosphor red, clearly apart from the cyan
// basemap, the green traffic and the amber specials.
constexpr uint16_t C_RED = rgb565(255, 88, 74);
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

// The afterglow fade is drawn in discrete steps: the step index feeds
// both the blip's brightness and its redraw fingerprint, so a blip
// repaints sixteen times per sweep instead of every frame. A four-
// percent brightness step every ~1.7 s reads as a continuous ramp, and
// in a packed sky the step rate sets the whole erase-and-repaint load.
constexpr float kFadeSteps = 16.0f;
int fadeStepOf(const Aircraft& ac) {
  return static_cast<int>(ac.freshness * kFadeSteps);
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
  x = floorf(x);  // Whole-pixel anchor; see drawAircraft.
  y = floorf(y);
  if (hypotf(x - kCenterX, y - kCenterY) > kRadius) return;
  uint16_t c = dim(C_WHITE, model.ui.brightness);
  gfx->drawLine(x - 10, y, x + 10, y, c);
  gfx->drawLine(x, y - 10, x, y + 10, c);
  gfx->fillCircle(x, y, 3, c);
}

void drawPoi(Arduino_GFX* gfx, const Model& model, const model::Poi& poi) {
  float x, y;
  project(model, poi.pos, x, y);
  x = floorf(x);  // Whole-pixel anchor; see drawAircraft.
  y = floorf(y);
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
// the arrow points at the same spot the blip will appear. The bearing is
// quantized to the same grid the redraw fingerprint uses, so a repaint
// between fingerprint steps reproduces the previous pixels exactly.
// `selected`/`candidate` mark the flight the encoder browse is resting
// on: its chevron grows and gains the blips' selection reticle, so an
// off-scope flight reads as "about to be committed" as clearly as an
// on-scope blip does.
void drawEdgeArrow(Arduino_GFX* gfx, const Model& model, const Aircraft& ac,
                   float distance, const Vec& at, bool selected,
                   bool candidate) {
  float k = model.ui.brightness;
  uint16_t c = dim(ac.special ? C_AMBER : C_WHITE, k);
  float bearing = lroundf(bearingFrom(model, at) * 4.0f) / 4.0f;
  float x, y;
  polar(bearing, kRadius * 0.985f, x, y);
  // Chevron points out; the browse target's is half again as large.
  drawTriangle(gfx, x, y, bearing + 180, (selected || candidate) ? 1.6f : 1.1f,
               c);
  if (selected || candidate) {
    // The same reticle the on-scope blips carry, in the same colors.
    uint16_t rc = dim(candidate ? C_AMBER : C_WHITE, k);
    gfx->drawRect(static_cast<int16_t>(x) - 17, static_cast<int16_t>(y) - 17,
                  34, 34, rc);
  }
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
      drawEdgeArrow(gfx, model, ac, distance, at, selected, candidate);
      endTrackPush();
    }
    return;
  }

  float k = model.ui.brightness;
  float bright =
      followed ? 1.0f : (0.35f + 0.65f * fadeStepOf(ac) / kFadeSteps);
  uint16_t color = dim(ac.special ? C_AMBER : C_GREEN, k * bright);

  // The non-followed blip draws entirely from the sweep-painted
  // snapshots — position, leading line, nose and label all hold what the
  // sweep last saw, instead of the direction vectors swinging on every
  // poll ahead of it. The followed flight is actively dead-reckoned, so
  // it keeps the live values.
  float track = followed ? ac.track : ac.shownTrack;
  float speed = followed ? ac.groundSpeed : ac.shownSpeed;
  int32_t altitude = followed ? ac.altitude : ac.shownAlt;

  float x, y;
  project(model, at, x, y);
  x = floorf(x);  // Whole-pixel anchors: a repaint between fingerprint
  y = floorf(y);  // steps reproduces the previous pixels exactly.

  // Leading line: one minute ahead along the track.
  float ahead = speed / 60.0f;
  Vec tip{at.x + sinf(track * DEG_TO_RAD) * ahead,
          at.y + cosf(track * DEG_TO_RAD) * ahead};
  float tx, ty;
  project(model, tip, tx, ty);
  tx = floorf(tx);
  ty = floorf(ty);
  gfx->drawLine(x, y, tx, ty, color);
  g_curSegs.push_back(Seg{static_cast<int16_t>(x), static_cast<int16_t>(y),
                          static_cast<int16_t>(tx), static_cast<int16_t>(ty)});

  // Glyph cluster and label block track as separate boxes (see Elem):
  // the union of the two is mostly empty space.
  g_live->beginTrack();
  if (ac.special) gfx->drawCircle(x, y, 13, dim(C_AMBER, k * bright));
  drawTriangle(gfx, x, y, track, followed ? 1.2f : 1.0f, color);
  if (selected || candidate) {
    uint16_t rc = dim(candidate ? C_AMBER : C_WHITE, k);
    gfx->drawRect(x - 17, y - 17, 34, 34, rc);  // Selection reticle.
  }
  endTrackPush();

  g_live->beginTrack();
  if (model.ui.range <= kLabelMaxRangeNm || followed || selected ||
      candidate || ac.special) {
    drawLabel(gfx, x + 12, y - 4, ac.callsign, color);
    String block = String(altitude) + "  " +
                   String(static_cast<int>(speed)) + "kt";
    drawLabel(gfx, x + 12, y + 8, block, dim(C_WHITE, k * bright));
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
// the visible segments costs almost nothing. Resumable for the sliced
// rebuild: draws at most `maxLines` polylines starting at `firstLine`
// and returns the index to continue from.
int drawGeoSet(Arduino_GFX* gfx, const int16_t (*pts)[2],
               const uint16_t* lens, int lineCount, int firstLine,
               int maxLines, float ax, float bx, float ay, float by,
               uint16_t color) {
  int base = 0;
  for (int line = 0; line < firstLine; ++line) base += lens[line];
  int last = min(firstLine + maxLines, lineCount);
  for (int line = firstLine; line < last; ++line) {
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
  return last;
}

// The home-anchored projection fold shared by the geography and airport
// layers: anchor at the active home, matching the feeds' geoToWorld —
// world NM east/north via the local equirectangular approximation, then
// project()'s view-center shift and NM-to-pixel scale, all folded into
// one multiply-add per axis (lat/lon in `degPerUnit` fixed-point).
struct GeoFold {
  float ax, bx, ay, by;
};
GeoFold geoFoldOf(const Model& model, float degPerUnit) {
  float homeLat = 0, homeLon = 0;
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    homeLat = model.homes[model.ui.homeIndex].latitude;
    homeLon = model.homes[model.ui.homeIndex].longitude;
  }
  float ppn = pixelsPerNm(model);
  float lonNm = 60.0f * cosf(homeLat * DEG_TO_RAD);  // NM per degree lon.
  GeoFold f;
  f.ax = degPerUnit * lonNm * ppn;
  f.bx = kCenterX - (homeLon * lonNm + model.ui.viewCenter.x) * ppn;
  f.ay = -degPerUnit * 60.0f * ppn;
  f.by = kCenterY + (homeLat * 60.0f + model.ui.viewCenter.y) * ppn;
  return f;
}

// Geography colors: dim cyan in both palettes — it reads as background
// terrain under the green and the blue phosphor alike — with borders a
// step dimmer than the coast so the physical outline dominates.
constexpr float kGeoBorderDim = 0.35f;
constexpr float kGeoCoastDim = 0.55f;

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
// Resumable for the sliced rebuild: draws at most `maxCount` airports
// starting at `firstIdx` and returns the index to continue from (the
// visible count when finished).
int drawAirports(Arduino_GFX* gfx, const Model& model, int firstIdx,
                 int maxCount) {
  // The same home-anchored fold as the geography: screen x/y are one
  // multiply-add from the fixed-point lat/lon.
  GeoFold f = geoFoldOf(model, airportdata::kDegPerUnit);
  float homeLat = 0, homeLon = 0;
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    homeLat = model.homes[model.ui.homeIndex].latitude;
    homeLon = model.homes[model.ui.homeIndex].longitude;
  }
  float lonNm = 60.0f * cosf(homeLat * DEG_TO_RAD);  // NM per degree lon.

  // Declutter: medium fields reveal only when zoomed in, and the IATA
  // labels obey the same cutoff as the bulk-traffic labels.
  bool labels = model.ui.range <= kLabelMaxRangeNm;
  int count = model.ui.range <= kMediumAirportMaxRangeNm
                  ? airportdata::kCount
                  : airportdata::kLargeCount;
  int last = min(firstIdx + maxCount, count);
  float k = model.ui.brightness;
  uint16_t glyph = dim(C_CYAN, k * 0.8f);
  uint16_t text = dim(C_CYAN, k * 0.65f);
  gfx->setTextSize(1);
  gfx->setTextColor(text);
  for (int i = firstIdx; i < last; ++i) {
    const airportdata::Airport& ap = airportdata::kAirports[i];
    float x = ap.lon * f.ax + f.bx;
    float y = ap.lat * f.ay + f.by;
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
  return last;
}

// ---- City basemap (embedded GeoNames population centers). ----
//
// The world's largest cities from cities.h (generated by
// tools/make_cities.py), toggled by the Cities switch and drawn in
// warm red — population context under the traffic, distinct from
// everything operational. Cities are fixed geographic points, so
// they render into the static reference next to the geography —
// never per frame.

// Only the biggest few cities in view draw, so the layer adapts to
// the zoom by itself: wide views keep the world's giants, and
// zooming in surfaces the smaller local centers as the giants leave
// the scope.
constexpr int kCityMaxShown = 10;

// Draws the largest cities in view. kCities is sorted by population,
// largest first, so a single scan from the top that stops after
// kCityMaxShown in-view hits IS the top-N-by-population selection —
// recomputed each rebuild (pan and zoom re-fingerprint the scene).
void drawCities(Arduino_GFX* gfx, const Model& model) {
  // The same home-anchored fold as the geography: screen x/y are one
  // multiply-add from the fixed-point lat/lon.
  GeoFold f = geoFoldOf(model, citydata::kDegPerUnit);
  float k = model.ui.brightness;
  uint16_t glyph = dim(C_RED, k * 0.9f);
  uint16_t halo = dim(C_RED, k * 0.5f);
  gfx->setTextSize(1);
  gfx->setTextColor(dim(C_RED, k * 0.7f));
  float r2 = kRadius * kRadius;
  int shown = 0;
  for (int i = 0; i < citydata::kCount && shown < kCityMaxShown; ++i) {
    const citydata::City& city = citydata::kCities[i];
    float x = city.lon * f.ax + f.bx;
    float y = city.lat * f.ay + f.by;
    float dx = x - kCenterX, dy = y - kCenterY;
    if (dx * dx + dy * dy > r2) continue;  // Outside the scope disc.
    ++shown;
    gfx->fillCircle(x, y, 2, glyph);
    gfx->drawCircle(x, y, 5, halo);
    int len = static_cast<int>(strlen(city.name));
    gfx->setCursor(static_cast<int16_t>(x) - len * 3,
                   static_cast<int16_t>(y) + 10);
    gfx->print(city.name);
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

// Draws the weather cells whose cell-row indices lie in [rowFrom,
// rowTo) — resumable for the sliced rebuild; blocking callers pass the
// full range. One cell row is y0 = row * kWxCellPx.
void drawWeatherRows(Arduino_GFX* gfx, const Model& model, int rowFrom,
                     int rowTo) {
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

  for (int cellRow = rowFrom; cellRow < rowTo; ++cellRow) {
    int y0 = cellRow * kWxCellPx;
    if (y0 >= config::PANEL_HEIGHT) break;
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
  // Drawn INSIDE the acquiring-signal element's tracked span (see drawElem),
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

// Toggle feedback on the live scope: when the Display or Cities switch
// changes, the new state flashes centered below the home marker and fades
// out over 2.5 s, echoing the offline input-test line. Knob events show
// nothing here — the reticle and zoom are their own feedback. Drawn inside
// a tracked span (see drawElem), so the dirty-rect system erases it
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
  } else if (model.ui.lastInput.startsWith("CITIES") ||
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
// geography, cities, airports). Each frame we draw into the BACK framebuffer:
// erase the moving elements that buffer showed when it was last drawn (two
// frames ago) by copying those regions back from the reference, redraw them at
// new positions, then flip. The driver latches the flip only at a frame
// boundary, so the panel never scans a partially drawn frame — with a single
// shared buffer the scan raced the erase-then-redraw pass, which showed as
// flickering blips (worst in the late-scanned bottom half) and a half-erased
// sweep. Erasing every old box before drawing any new one keeps overlaps
// correct.
//
// Two refinements keep a busy sky inside the ~40 ms frame budget:
//   * Change-driven redraw: every moving element carries a fingerprint
//     (key) of everything that affects its pixels. An element whose key
//     is unchanged — and whose pixels nothing else disturbed — is left
//     in place instead of being erased and repainted; the sweep-gated
//     snapshots and the stepped afterglow keep bulk-traffic keys stable
//     between sweep passes, so a steady frame repaints a handful of
//     elements, not the whole sky.
//   * The follow-pan pipeline (below): view drift rebuilds the static
//     scene in time-budgeted slices instead of one blocking pass.

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
  uint8_t geo = 0;         // Geography drawn (view settled, no smear).
  uint8_t cities = 0;      // City-markers overlay on/off.
  int16_t bright = 0;      // Brightness * 16 (ambient dimming of static colors).
  uint8_t online = 0;      // Online: gates the weather layer into the reference.
  int8_t home = 0;         // Active home (geography anchor, name label).
  uint8_t emph = 0;        // Range readout emphasized (recent zoom detent).
  uint32_t wxGen = 0;      // Weather layer generation, while it is drawn.
  int16_t trail = 0;       // Followed-flight trail length (grows as it flies).
};
StaticSig g_sig;

// A moving element drawn into a framebuffer: identity, the fingerprint
// of everything that affected its pixels, and the erase bookkeeping
// captured when it was drawn — up to two boxes (an aircraft tracks its
// glyph and its label block separately: their union is mostly empty
// space, and in a packed sky that dead area is what makes every erase
// disturb its neighbors) and one line segment.
struct Elem {
  uint32_t id = 0;
  uint32_t key = 0;
  Rect rect{0, 0, 0, 0};
  Rect rect2{0, 0, 0, 0};
  Seg seg{0, 0, 0, 0};
  bool hasRect = false;
  bool hasRect2 = false;
  bool hasSeg = false;
};

// What each framebuffer currently shows, so rendering into it again (two
// frames later) can erase or carry exactly that. `dirty` marks a buffer
// whose whole content must be repainted from the static reference
// first. `repaintRow` is the pipeline's progressive-repaint watermark:
// rows above it match the active reference, rows below may still show
// the previous one (see the follow-pan pipeline below).
struct FbState {
  std::vector<Elem> elems;  // Moving elements, ascending by id.
  int16_t sweepX = 0, sweepY = 0;  // Sweep endpoint (start is the center).
  bool haveSweep = false;
  bool dirty = true;
  int16_t repaintRow = 0;
};
FbState g_fb[2];

int16_t g_curSweepX = 0, g_curSweepY = 0;
bool g_curHaveSweep = false;  // This frame drew a sweep (offline suppresses it).
bool g_viewSettled = true;    // View locked on its target (home / a followed
                              // flight), not mid-transition; gates the
                              // geography and city overlays.

// Offline-to-online handoff: when signal is acquired, the amber title
// fades out and the sweep fades in over these windows, so the switch to
// the live scope is a dissolve rather than a hard cut.
bool g_wasOnline = false;
uint32_t g_onlineSinceMs = 0;  // millis() when online last became true.
constexpr uint32_t kSignalFadeMs = 900;
constexpr uint32_t kSweepFadeMs = 1800;

bool sigEqual(const StaticSig& a, const StaticSig& b) {
  return a.range100 == b.range100 && a.cx == b.cx && a.cy == b.cy &&
         a.mode == b.mode && a.geo == b.geo && a.cities == b.cities &&
         a.bright == b.bright && a.online == b.online && a.home == b.home &&
         a.emph == b.emph && a.wxGen == b.wxGen && a.trail == b.trail;
}

StaticSig staticSigOf(const Model& model) {
  float ppn = pixelsPerNm(model);
  StaticSig s;
  s.range100 = static_cast<int32_t>(lroundf(model.ui.range * 100.0f));
  s.cx = static_cast<int16_t>(lroundf(model.ui.viewCenter.x * ppn));
  s.cy = static_cast<int16_t>(lroundf(model.ui.viewCenter.y * ppn));
  s.mode = static_cast<uint8_t>(model.ui.display);
  // Geography always draws once the view settles; the settle state
  // alone re-fingerprints it in and out around a transition pan.
  s.geo = g_viewSettled ? 1 : 0;
  s.cities = (model.ui.cities && g_viewSettled) ? 1 : 0;
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

// Restores the static pixels under a previously drawn line (the sweep or
// a leading line). The brush spans one pixel to each side along the
// line's minor axis, so a one-pixel rasteriser tie-break difference
// cannot leave a glowing trail behind — at a third of the cost of the
// full 3x3 brush this erase used to sweep along the line (the sweep's
// ~300-pixel erase runs every frame, so its constant factor matters).
void restoreSeg(int x0, int y0, int x1, int y1) {
  int dx = x1 - x0;
  if (dx < 0) dx = -dx;
  int dy = y1 - y0;
  if (dy < 0) dy = -dy;
  bool steep = dy > dx;
  dy = -dy;
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    if (steep) {  // Mostly vertical: brush across x.
      restorePixel(x0 - 1, y0);
      restorePixel(x0, y0);
      restorePixel(x0 + 1, y0);
    } else {  // Mostly horizontal: brush across y.
      restorePixel(x0, y0 - 1);
      restorePixel(x0, y0);
      restorePixel(x0, y0 + 1);
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
  // Dots land every ~7 px of CUMULATIVE path length, carried across the
  // points. The live tail samples sit only a pixel or two apart at
  // typical range and speed, so per-segment spacing (each segment's dot
  // count rounded down independently) would draw nothing at all for it —
  // the trail vector grew while the drawn path never did.
  constexpr float kDotSpacingPx = 7.0f;
  float px = 0, py = 0;
  float toNext = 0;  // Path distance left until the next dot.
  bool have = false;
  for (size_t i = 0; i < trail.size(); ++i) {
    float x, y;
    project(model, trail[i], x, y);
    if (have) {
      float dx = x - px, dy = y - py;
      float dist = sqrtf(dx * dx + dy * dy);
      if (dist > 0.0f) {
        float along = toNext;
        while (along <= dist) {
          float t = along / dist;
          float xp = px + dx * t, yp = py + dy * t;
          if (xp >= 0 && xp < g_w && yp >= 0 && yp < g_h) {
            gfx->fillCircle(static_cast<int16_t>(xp),
                            static_cast<int16_t>(yp), 1, c);
          }
          along += kDotSpacingPx;
        }
        toNext = along - dist;
      }
    }
    px = x;
    py = y;
    have = true;
  }
}

// ---- Static-scene painter, resumable in time-budgeted slices. ----
//
// The scene is painted through a small unit machine so the follow-pan
// pipeline can spread the ~90 ms of PSRAM-bound work over many frames;
// blocking callers just run the machine to completion. The layer order
// matches the original renderStatic: field fill, rain, ring metalwork,
// geography, cities, airports, trail, labels.

// Unit sizes: each is a few milliseconds of work, so the slice runner's
// deadline overshoot stays small.
constexpr int kFillRowsPerUnit = 18;
constexpr int kWxRowsPerUnit = 1;
constexpr int kGeoLinesPerUnit = 96;
constexpr int kAirportsPerUnit = 24;

enum class BuildStep : uint8_t {
  Fill,
  Weather,
  Background,
  GeoBorders,
  GeoCoast,
  Cities,
  Airports,
  Trail,
  Labels,
  Done,
};

struct BuildState {
  Arduino_GFX* gfx = nullptr;  // Target surface...
  uint16_t* fb = nullptr;      // ...and its raw framebuffer (row fills).
  BuildStep step = BuildStep::Done;
  int row = 0;      // Fill: next panel row.
  int wxRow = 0;    // Weather: next cell row.
  int line = 0;     // Geography: next polyline.
  int airport = 0;  // Airports: next index.
  Vec center;       // View center latched at buildBegin, with the
  float bright = 1.0f;  // brightness, for cross-slice consistency.
};
BuildState g_build;

// Fills panel rows [y0, y1) with `color` through a tight 32-bit store
// loop — the fill is pure PSRAM bandwidth, so skip the canvas overhead.
void fillRows(uint16_t* fb, int y0, int y1, uint16_t color) {
  uint32_t v2 = (static_cast<uint32_t>(color) << 16) | color;
  uint32_t* p =
      reinterpret_cast<uint32_t*>(fb + static_cast<size_t>(y0) * g_w);
  size_t words = static_cast<size_t>(y1 - y0) * g_w / 2;
  for (size_t i = 0; i < words; ++i) p[i] = v2;
}

void buildBegin(Arduino_GFX* gfx, uint16_t* fb, const Model& model) {
  g_build.gfx = gfx;
  g_build.fb = fb;
  g_build.step = BuildStep::Fill;
  g_build.row = 0;
  g_build.wxRow = 0;
  g_build.line = 0;
  g_build.airport = 0;
  g_build.center = model.ui.viewCenter;
  g_build.bright = model.ui.brightness;
}

// Runs build units until the scene is complete or the budget (µs since
// `startUs`) is spent; at least one unit always runs, so a contended
// frame cannot stall the build forever. The latched view center and
// brightness are swapped in around the units so every slice projects
// identically, however far the live view has eased since the build
// began. Returns true once the scene is complete.
bool buildRun(Model& model, uint32_t startUs, uint32_t budgetUs) {
  if (g_build.step == BuildStep::Done) return true;
  Vec liveCenter = model.ui.viewCenter;
  float liveBright = model.ui.brightness;
  model.ui.viewCenter = g_build.center;
  model.ui.brightness = g_build.bright;
  Arduino_GFX* gfx = g_build.gfx;
  do {
    switch (g_build.step) {
      case BuildStep::Fill: {
        int to = min(g_build.row + kFillRowsPerUnit, g_h);
        fillRows(g_build.fb, g_build.row, to, C_BG);
        g_build.row = to;
        if (to >= g_h) g_build.step = BuildStep::Weather;
        break;
      }
      case BuildStep::Weather: {
        if (!(model.ui.online && showWeather(model) &&
              weatherUsable(model))) {
          g_build.step = BuildStep::Background;
          break;
        }
        int rows = (g_h + kWxCellPx - 1) / kWxCellPx;
        int to = min(g_build.wxRow + kWxRowsPerUnit, rows);
        drawWeatherRows(gfx, model, g_build.wxRow, to);
        g_build.wxRow = to;
        if (to >= rows) g_build.step = BuildStep::Background;
        break;
      }
      case BuildStep::Background:
        drawBackground(gfx, model);
        g_build.step = BuildStep::GeoBorders;
        break;
      case BuildStep::GeoBorders: {
        // Geography always draws, but hides while the view is in motion
        // (any transition pan): it can't scroll smoothly with the map, so
        // a lagging coastline mid-pan is worse than none. It returns once
        // the view locks on home or a followed flight (g_viewSettled).
        if (!g_viewSettled) {
          g_build.step = BuildStep::Cities;
          break;
        }
        GeoFold f = geoFoldOf(model, geodata::kDegPerUnit);
        g_build.line = drawGeoSet(
            gfx, geodata::kBorderPts, geodata::kBorderLen,
            geodata::kBorderLines, g_build.line, kGeoLinesPerUnit, f.ax,
            f.bx, f.ay, f.by,
            dim(C_CYAN, model.ui.brightness * kGeoBorderDim));
        if (g_build.line >= geodata::kBorderLines) {
          g_build.line = 0;
          g_build.step = BuildStep::GeoCoast;
        }
        break;
      }
      case BuildStep::GeoCoast: {
        GeoFold f = geoFoldOf(model, geodata::kDegPerUnit);
        g_build.line = drawGeoSet(
            gfx, geodata::kCoastPts, geodata::kCoastLen,
            geodata::kCoastLines, g_build.line, kGeoLinesPerUnit, f.ax,
            f.bx, f.ay, f.by,
            dim(C_CYAN, model.ui.brightness * kGeoCoastDim));
        if (g_build.line >= geodata::kCoastLines) {
          g_build.step = BuildStep::Cities;
        }
        break;
      }
      case BuildStep::Cities:
        // Cities follow their toggle, and share the geography's
        // settled-view gate so they never smear through a pan. The
        // whole layer is one unit: ~10 markers cost less than a
        // single geography slice.
        if (model.ui.cities && g_viewSettled) drawCities(gfx, model);
        g_build.step = BuildStep::Airports;
        break;
      case BuildStep::Airports: {
        // Airports ride the flights layer (they hide in weather-only
        // mode, like the traffic and POIs), independent of Geography.
        if (!showFlights(model)) {
          g_build.step = BuildStep::Trail;
          break;
        }
        int next =
            drawAirports(gfx, model, g_build.airport, kAirportsPerUnit);
        if (next == g_build.airport || next >= airportdata::kCount) {
          g_build.step = BuildStep::Trail;
        }
        g_build.airport = next;
        break;
      }
      case BuildStep::Trail:
        if (model.ui.following) drawFollowTrail(gfx, model);
        g_build.step = BuildStep::Labels;
        break;
      case BuildStep::Labels:
        drawHomeName(gfx, model);
        drawRangeReadout(gfx, model);
        g_build.step = BuildStep::Done;
        break;
      case BuildStep::Done:
        break;
    }
  } while (g_build.step != BuildStep::Done &&
           micros() - startUs < budgetUs);
  model.ui.viewCenter = liveCenter;
  model.ui.brightness = liveBright;
  return g_build.step == BuildStep::Done;
}

// Paints the whole static scene into the ACTIVE reference, blocking.
// Boot, the power-on bloom and the urgent rebuild path use this; view
// drift while following goes through the sliced pipeline instead.
void renderStatic(Model& model) {
  buildBegin(g_static, g_staticFb, model);
  buildRun(model, micros(), UINT32_MAX);
}

// ---- Follow-pan pipeline: rebuilding without a blocking frame. ----
//
// Following a flight moves the view about a pixel a second, and every
// pixel of drift used to trigger the blocking rebuild-and-repaint — a
// ~250 ms freeze, once a second, squarely while the user is watching
// their flight. Instead, gradual drifts of the static signature are
// absorbed here: the new scene is painted into a spare reference
// buffer a time-budgeted slice per frame (Build), the spare is swapped
// in to become the active reference, and the two scan framebuffers
// then converge on it a band of rows per frame (Repaint). No frame
// blocks. While a framebuffer still shows rows of the previous scene,
// erases restore from the new one — a transient, pixel-scale mismatch
// that the row sweep repaints within a few frames.
//
// Discrete, user-visible switches (range, mode, cities, home, online)
// keep the blocking rebuild: they should land instantly, and a single
// hitch on a deliberate control input reads as response, not stutter.

enum class Pipe : uint8_t { Idle, Build, Repaint };
Pipe g_pipe = Pipe::Idle;
StaticSig g_pipeSig;             // Signature the in-flight build satisfies.
Arduino_GFX* g_spare = nullptr;  // Spare reference surface...
uint16_t* g_spareFb = nullptr;   // ...and its framebuffer.

// Per-frame time budgets, measured from the start of the frame. The
// panel scans a frame every ~40.6 ms; pipeline work stops at its
// deadline so the frame still makes the boundary. Repaint bands run
// before the dynamic pass and reserve room for it (a band also forces
// the elements it crosses to repaint, so a modest band spreads that
// load); build slices run after it and take whatever is left.
constexpr uint32_t kRepaintDeadlineUs = 12000;
constexpr uint32_t kFrameBudgetUs = 34000;
constexpr int kRepaintChunkRows = 16;

// Successive pipeline starts are spaced out: the drifting signature
// changes two or three times a second (view x, view y and the trail
// tick over independently), and each pass costs two band sweeps of
// repaint. One pass per second coalesces them — the same cadence the
// old blocking rebuild had, minus the stall.
constexpr uint32_t kPipeMinIntervalMs = 1000;
uint32_t g_lastPipeStartMs = 0;

// Contention awareness: while the network core's traffic poll is
// running, every PSRAM access here slows by a third, and frames that
// would fit the panel period start missing it. Pipeline work (bands and
// slices) yields on frames whose predecessor missed its boundary — the
// pipeline can pause at any point, and the only cost is the map briefly
// lagging the pan a little further before converging. Every third frame
// runs the pipeline regardless, so a long stretch of late frames (a
// poll in a packed sky) slows the map's tracking instead of freezing
// it.
constexpr uint32_t kFrameLateUs = 60000;  // ~1.5 panel periods.
uint32_t g_prevFrameStartUs = 0;
bool g_prevFrameLate = false;
uint32_t g_frameCount = 0;

bool pipeYield() { return g_prevFrameLate && g_frameCount % 3 != 0; }

// True when a signature change needs the blocking rebuild (a discrete
// switch), false for the gradual drifts the pipeline absorbs (view
// center, ambient brightness, readout emphasis, weather generation,
// trail growth).
bool sigUrgent(const StaticSig& a, const StaticSig& b) {
  return a.range100 != b.range100 || a.mode != b.mode || a.geo != b.geo ||
         a.cities != b.cities || a.online != b.online || a.home != b.home;
}

// Abandons in-flight pipeline work: the spare's half-painted scene is
// scratch, and the blocking caller re-syncs both framebuffers itself.
void pipeCancel() { g_pipe = Pipe::Idle; }

// ---- Change-driven redraw of the moving elements. ----

// One entry of the frame's desired-element list, in draw (z) order:
// the sweep sits under the markers, which sit under the aircraft and
// the overlays, exactly as the original full-redraw pass drew them.
enum class EKind : uint8_t { Home, Poi, Aircraft, Flash, Signal };

struct Desired {
  uint32_t id;    // Stable identity, ascending in list order.
  uint32_t key;   // Fingerprint of the drawn pixels.
  EKind kind;
  int16_t index;  // Model index for Poi/Aircraft.
  int16_t old;    // Index into the framebuffer's previous elements.
  bool redraw;
};

std::vector<Desired> g_desired;
std::vector<Elem> g_newElems;
std::vector<Rect> g_eraseRects;  // Regions restored to static this frame.
std::vector<Seg> g_eraseSegs;
uint32_t g_seq = 0;  // Per-frame sequence for always-redraw keys.

uint32_t mix(uint32_t h, uint32_t v) {
  h ^= v + 0x9e3779b9u + (h << 6) + (h >> 2);
  return h;
}
uint32_t mixf(uint32_t h, float v) {
  union {
    float f;
    uint32_t u;
  } c;
  c.f = v;
  return mix(h, c.u);
}
uint32_t hashStr(const String& s) {
  uint32_t h = 2166136261u;
  for (unsigned i = 0; i < s.length(); ++i) {
    h = (h ^ static_cast<uint8_t>(s[i])) * 16777619u;
  }
  return h;
}

// Fingerprints one aircraft's drawn state; false when it draws nothing
// this frame. Mirrors drawAircraft: every model input that can change a
// drawn pixel feeds the key, quantized exactly as the draw quantizes.
bool aircraftDesire(const Model& model, const Aircraft& ac, int index,
                    uint32_t& key) {
  bool followed = model.ui.following && index == model.ui.followIndex;
  bool selected = !model.ui.following && index == model.ui.browseSel;
  bool candidate = model.ui.following && index == model.ui.candidate;
  const Vec& at = followed ? ac.est : ac.shown;
  if (!followed && !ac.seen) return false;

  uint32_t flags = (followed ? 1u : 0) | (selected ? 2u : 0) |
                   (candidate ? 4u : 0) | (ac.special ? 8u : 0);
  int brightStep = static_cast<int>(model.ui.brightness * 16.0f);
  float distance = distanceFrom(model, at);
  uint32_t h = 2166136261u;
  if (distance > model.ui.range) {
    if (!(ac.special || selected || candidate || followed)) return false;
    // Edge arrow: the rim position from the (quantized) bearing, the
    // label from the whole-NM distance and callsign.
    h = mix(h, 1);
    h = mix(h, flags);
    h = mix(h, static_cast<uint32_t>(
                   lroundf(bearingFrom(model, at) * 4.0f)));
    h = mix(h, static_cast<uint32_t>(static_cast<int>(distance)));
    h = mix(h, static_cast<uint32_t>(brightStep));
    h = mix(h, hashStr(ac.callsign));
    key = h;
    return true;
  }

  float track = followed ? ac.track : ac.shownTrack;
  float speed = followed ? ac.groundSpeed : ac.shownSpeed;
  int32_t altitude = followed ? ac.altitude : ac.shownAlt;
  float x, y;
  project(model, at, x, y);
  float ahead = speed / 60.0f;
  Vec tip{at.x + sinf(track * DEG_TO_RAD) * ahead,
          at.y + cosf(track * DEG_TO_RAD) * ahead};
  float tx, ty;
  project(model, tip, tx, ty);
  bool labels = model.ui.range <= kLabelMaxRangeNm || followed ||
                selected || candidate || ac.special;
  h = mix(h, 2);
  h = mix(h, flags);
  h = mix(h, static_cast<uint32_t>(static_cast<int32_t>(floorf(x))));
  h = mix(h, static_cast<uint32_t>(static_cast<int32_t>(floorf(y))));
  h = mix(h, static_cast<uint32_t>(static_cast<int32_t>(floorf(tx))));
  h = mix(h, static_cast<uint32_t>(static_cast<int32_t>(floorf(ty))));
  h = mixf(h, track);
  h = mix(h, static_cast<uint32_t>(followed ? 999 : fadeStepOf(ac)));
  h = mix(h, static_cast<uint32_t>(brightStep));
  h = mix(h, labels ? 1u : 0u);
  if (labels) {
    h = mix(h, hashStr(ac.callsign));
    h = mix(h, static_cast<uint32_t>(altitude));
    h = mix(h, static_cast<uint32_t>(static_cast<int>(speed)));
  }
  key = h;
  return true;
}

// Builds the frame's desired-element list. Ids ascend in list order, so
// the erase pass can diff against a framebuffer's previous elements in
// one merge walk.
void computeDesired(const Model& model) {
  g_desired.clear();
  int brightStep = static_cast<int>(model.ui.brightness * 16.0f);
  uint32_t now = millis();
  uint32_t onlineAge = now - g_onlineSinceMs;

  {
    float x, y;
    project(model, Vec{0, 0}, x, y);
    uint32_t h = mix(2166136261u,
                     static_cast<uint32_t>(static_cast<int32_t>(floorf(x))));
    h = mix(h, static_cast<uint32_t>(static_cast<int32_t>(floorf(y))));
    h = mix(h, static_cast<uint32_t>(brightStep));
    g_desired.push_back(Desired{0x08000000u, h, EKind::Home, 0, -1, false});
  }

  if (showFlights(model)) {
    for (int i = 0; i < static_cast<int>(model.pois.size()); ++i) {
      const model::Poi& poi = model.pois[i];
      if (distanceFrom(model, poi.pos) > model.ui.range) continue;
      float x, y;
      project(model, poi.pos, x, y);
      uint32_t h = mix(2166136261u,
                       static_cast<uint32_t>(static_cast<int32_t>(floorf(x))));
      h = mix(h, static_cast<uint32_t>(static_cast<int32_t>(floorf(y))));
      h = mix(h, static_cast<uint32_t>(brightStep));
      h = mix(h, hashStr(poi.name));  // A web edit can rename in place.
      g_desired.push_back(Desired{0x10000000u + i, h, EKind::Poi,
                                  static_cast<int16_t>(i), -1, false});
    }
    for (int i = 0; i < static_cast<int>(model.aircraft.size()); ++i) {
      uint32_t key;
      if (aircraftDesire(model, model.aircraft[i], i, key)) {
        g_desired.push_back(Desired{0x20000000u + i, key, EKind::Aircraft,
                                    static_cast<int16_t>(i), -1, false});
      }
    }
  }

  // Toggle feedback flashes on the live scope; offline, the input-test
  // line inside drawAcquiringSignal already covers every control event.
  if (model.ui.online && !model.ui.lastInput.isEmpty() &&
      now - model.ui.lastInputMs < 2500) {
    uint32_t h = mix(hashStr(model.ui.lastInput),
                     (now - model.ui.lastInputMs) / 80);
    h = mix(h, static_cast<uint32_t>(brightStep));
    g_desired.push_back(Desired{0x30000001u, h, EKind::Flash, 0, -1, false});
  }

  // The acquiring-signal title pulses while offline and dissolves just
  // after signal is acquired; it repaints every frame either way.
  if (!model.ui.online || onlineAge < kSignalFadeMs) {
    g_desired.push_back(
        Desired{0x30000002u, ++g_seq, EKind::Signal, 0, -1, false});
  }
}

// Draws one desired element into the live surface and captures its
// erase bookkeeping (at most one box and one line segment each).
void drawElem(Model& model, const Desired& d, Elem& out) {
  g_curRects.clear();
  g_curSegs.clear();
  switch (d.kind) {
    case EKind::Home:
      g_live->beginTrack();
      drawHomeMarker(g_live, model);
      endTrackPush();
      break;
    case EKind::Poi:
      g_live->beginTrack();
      drawPoi(g_live, model, model.pois[d.index]);
      endTrackPush();
      break;
    case EKind::Aircraft:
      drawAircraft(g_live, model, model.aircraft[d.index], d.index);
      break;
    case EKind::Flash:
      g_live->beginTrack();
      drawToggleFlash(g_live, model);
      endTrackPush();
      break;
    case EKind::Signal: {
      uint32_t onlineAge = millis() - g_onlineSinceMs;
      float fade =
          model.ui.online
              ? 1.0f - onlineAge / static_cast<float>(kSignalFadeMs)
              : 1.0f;
      if (fade > 0.0f) {
        g_live->beginTrack();
        drawAcquiringSignal(g_live, model, fade);
        endTrackPush();
      }
      break;
    }
  }
  out.id = d.id;
  out.key = d.key;
  out.hasRect = !g_curRects.empty();
  if (out.hasRect) out.rect = g_curRects[0];
  out.hasRect2 = g_curRects.size() > 1;
  if (out.hasRect2) {
    Rect r = g_curRects[1];
    for (size_t i = 2; i < g_curRects.size(); ++i) {  // Union, for safety.
      const Rect& b = g_curRects[i];
      int16_t x1 = max(static_cast<int16_t>(r.x + r.w),
                       static_cast<int16_t>(b.x + b.w));
      int16_t y1 = max(static_cast<int16_t>(r.y + r.h),
                       static_cast<int16_t>(b.y + b.h));
      r.x = min(r.x, b.x);
      r.y = min(r.y, b.y);
      r.w = x1 - r.x;
      r.h = y1 - r.y;
    }
    out.rect2 = r;
  }
  out.hasSeg = !g_curSegs.empty();
  if (out.hasSeg) out.seg = g_curSegs[0];
}

// Restores the static pixels under one previous element and records the
// regions for the disturb check below.
void eraseElem(const Elem& e) {
  if (e.hasSeg) {
    restoreSeg(e.seg.x0, e.seg.y0, e.seg.x1, e.seg.y1);
    g_eraseSegs.push_back(e.seg);
  }
  if (e.hasRect) {
    restoreRect(e.rect);
    g_eraseRects.push_back(e.rect);
  }
  if (e.hasRect2) {
    restoreRect(e.rect2);
    g_eraseRects.push_back(e.rect2);
  }
}

// Overlap tests for the disturb check, padded for the 3x3 erase brush.
constexpr int kDisturbPad = 2;

bool rectsOverlap(const Rect& a, const Rect& b) {
  return a.x < b.x + b.w + kDisturbPad && b.x < a.x + a.w + kDisturbPad &&
         a.y < b.y + b.h + kDisturbPad && b.y < a.y + a.h + kDisturbPad;
}

// Line segment vs box, via the same outcode walk the geography clipper
// uses; conservative only in the padding.
bool segRectOverlap(const Seg& s, const Rect& r) {
  float x0 = s.x0, y0 = s.y0, x1 = s.x1, y1 = s.y1;
  float lx = r.x - kDisturbPad, rx = r.x + r.w + kDisturbPad;
  float ty = r.y - kDisturbPad, by = r.y + r.h + kDisturbPad;
  auto code = [&](float x, float y) {
    return static_cast<uint8_t>((x < lx ? 1 : 0) | (x > rx ? 2 : 0) |
                                (y < ty ? 4 : 0) | (y > by ? 8 : 0));
  };
  uint8_t c0 = code(x0, y0), c1 = code(x1, y1);
  for (;;) {
    if ((c0 | c1) == 0) return true;
    if (c0 & c1) return false;
    uint8_t out = c0 != 0 ? c0 : c1;
    float x, y;
    if (out & 1) {
      x = lx;
      y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    } else if (out & 2) {
      x = rx;
      y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    } else if (out & 4) {
      y = ty;
      x = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    } else {
      y = by;
      x = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    }
    if (out == c0) {
      x0 = x;
      y0 = y;
      c0 = code(x0, y0);
    } else {
      x1 = x;
      y1 = y;
      c1 = code(x1, y1);
    }
  }
}

Rect segBox(const Seg& s) {
  int16_t x = min(s.x0, s.x1), y = min(s.y0, s.y1);
  return Rect{x, y, static_cast<int16_t>(max(s.x0, s.x1) - x + 1),
              static_cast<int16_t>(max(s.y0, s.y1) - y + 1)};
}

// True when something restored this frame (or the freshly drawn sweep)
// touched this carried element's pixels, so it must repaint even though
// its own state is unchanged.
bool disturbed(const Elem& e, const Seg* newSweep) {
  for (const Rect& r : g_eraseRects) {
    if (e.hasRect && rectsOverlap(e.rect, r)) return true;
    if (e.hasRect2 && rectsOverlap(e.rect2, r)) return true;
    if (e.hasSeg && segRectOverlap(e.seg, r)) return true;
  }
  for (const Seg& s : g_eraseSegs) {
    if (e.hasRect && segRectOverlap(s, e.rect)) return true;
    if (e.hasRect2 && segRectOverlap(s, e.rect2)) return true;
    if (e.hasSeg && segRectOverlap(s, segBox(e.seg))) return true;
  }
  if (newSweep != nullptr) {
    if (e.hasRect && segRectOverlap(*newSweep, e.rect)) return true;
    if (e.hasRect2 && segRectOverlap(*newSweep, e.rect2)) return true;
    if (e.hasSeg && segRectOverlap(*newSweep, segBox(e.seg))) return true;
  }
  return false;
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
    front.elems.clear();
    front.haveSweep = false;
    front.dirty = false;
    front.repaintRow = static_cast<int16_t>(g_h);  // Matches the reference.
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

  // The follow-pan pipeline's spare reference: a third full-panel
  // buffer in PSRAM. If it cannot be allocated, the pipeline stays off
  // and every rebuild takes the blocking path.
  if (g_spareFb == nullptr) {
    g_spareFb = static_cast<uint16_t*>(heap_caps_aligned_alloc(
        16, static_cast<size_t>(width) * height * 2, MALLOC_CAP_SPIRAM));
    if (g_spareFb != nullptr) {
      g_spare = new display::StaticGfx(width, height, g_spareFb);
    } else {
      Serial.println("[radar] spare reference alloc failed; pipeline off.");
    }
  }
  invalidate();
}

void invalidate() {
  g_needStatic = true;
  for (FbState& fb : g_fb) {
    fb.dirty = true;
    fb.haveSweep = false;
    fb.elems.clear();
  }
}

void renderIncremental(Model& model) {
  uint32_t frameStartUs = micros();
  if (g_prevFrameStartUs != 0) {
    g_prevFrameLate = frameStartUs - g_prevFrameStartUs > kFrameLateUs;
  }
  g_prevFrameStartUs = frameStartUs;
  ++g_frameCount;

  // Track the offline-to-online edge that drives the handoff dissolve
  // (title fading out, sweep fading in).
  if (model.ui.online != g_wasOnline) {
    g_wasOnline = model.ui.online;
    if (model.ui.online) g_onlineSinceMs = millis();
  }

  // React to static-scene changes. Discrete switches take the blocking
  // rebuild-and-repaint (rate-limited, see kRebuildMinIntervalMs);
  // gradual drifts start the sliced pipeline, and whatever drifts
  // further while it runs is picked up by the next one.
  StaticSig sig = staticSigOf(model);
  if (g_needStatic || !sigEqual(sig, g_sig)) {
    bool urgent =
        g_needStatic || sigUrgent(sig, g_sig) || g_spareFb == nullptr;
    if (urgent) {
      uint32_t now = millis();
      if (g_needStatic || now - g_lastRebuildMs >= kRebuildMinIntervalMs) {
        pipeCancel();
        renderStatic(model);
        g_sig = sig;
        g_needStatic = false;
        g_lastRebuildMs = now;
        g_fb[0].dirty = true;
        g_fb[1].dirty = true;
      }
    } else if (g_pipe == Pipe::Idle &&
               millis() - g_lastPipeStartMs >= kPipeMinIntervalMs) {
      // Latch the drifted scene and start building it into the spare.
      g_pipeSig = sig;
      buildBegin(g_spare, g_spareFb, model);
      g_pipe = Pipe::Build;
      g_lastPipeStartMs = millis();
    }
  }

  // Draw this frame into the back buffer; the panel keeps scanning the
  // front one until the flip below is latched at a frame boundary.
  FbState& fb = g_fb[display::backIndex()];
  bindTarget(display::backFb());

  Rect band{0, 0, 0, 0};
  bool haveBand = false;

  if (fb.dirty) {
    restoreAll();  // Repaint the whole buffer from the reference.
    fb.elems.clear();
    fb.haveSweep = false;
    fb.dirty = false;
    fb.repaintRow = static_cast<int16_t>(g_h);  // Fully in sync now.
  } else if (g_pipe == Pipe::Repaint && fb.repaintRow < g_h &&
             !pipeYield()) {
    // Progressive repaint: converge this buffer on the freshly swapped
    // reference a chunk of rows at a time, stopping at the deadline
    // (at least one chunk per frame, so a stretch of late frames slows
    // the convergence instead of stalling it).
    int from = fb.repaintRow;
    int row = from;
    while (row < g_h) {
      int to = min(row + kRepaintChunkRows, g_h);
      restoreRect(Rect{0, static_cast<int16_t>(row),
                       static_cast<int16_t>(g_w),
                       static_cast<int16_t>(to - row)});
      row = to;
      if (micros() - frameStartUs >= kRepaintDeadlineUs) break;
    }
    fb.repaintRow = static_cast<int16_t>(row);
    band = Rect{0, static_cast<int16_t>(from), static_cast<int16_t>(g_w),
                static_cast<int16_t>(row - from)};
    haveBand = true;
  }

  // The pipeline is done once both buffers have converged on the
  // swapped-in reference.
  if (g_pipe == Pipe::Repaint && g_fb[0].repaintRow >= g_h &&
      g_fb[1].repaintRow >= g_h) {
    g_pipe = Pipe::Idle;
  }

  computeDesired(model);

  // The sweep this frame will draw, for the disturb check: elements it
  // crosses repaint, keeping the blips-over-sweep z-order.
  Seg newSweep{0, 0, 0, 0};
  bool haveNewSweep = false;
  if (model.ui.online) {
    float sx, sy;
    polar(model.ui.sweepAngle, kRadius, sx, sy);
    newSweep =
        Seg{static_cast<int16_t>(kCenterX), static_cast<int16_t>(kCenterY),
            static_cast<int16_t>(sx), static_cast<int16_t>(sy)};
    haveNewSweep = true;
  }

  // Erase pass: restore the old sweep, then walk this buffer's previous
  // elements against the desired list (both ascend by id). An element
  // that is gone or whose fingerprint changed is erased; the rest are
  // candidates to carry. Every erase lands before any redraw, which
  // keeps overlaps correct.
  g_eraseRects.clear();
  g_eraseSegs.clear();
  if (haveBand && band.h > 0) g_eraseRects.push_back(band);
  if (fb.haveSweep) {
    restoreSeg(static_cast<int>(kCenterX), static_cast<int>(kCenterY),
               fb.sweepX, fb.sweepY);
    g_eraseSegs.push_back(Seg{static_cast<int16_t>(kCenterX),
                              static_cast<int16_t>(kCenterY), fb.sweepX,
                              fb.sweepY});
  }
  size_t oi = 0;
  for (Desired& d : g_desired) {
    while (oi < fb.elems.size() && fb.elems[oi].id < d.id) {
      eraseElem(fb.elems[oi]);  // Gone this frame.
      ++oi;
    }
    if (oi < fb.elems.size() && fb.elems[oi].id == d.id) {
      if (fb.elems[oi].key != d.key) {
        eraseElem(fb.elems[oi]);
        d.redraw = true;
      } else {
        d.old = static_cast<int16_t>(oi);
      }
      ++oi;
    } else {
      d.redraw = true;  // New this frame.
    }
  }
  while (oi < fb.elems.size()) {
    eraseElem(fb.elems[oi]);  // Gone this frame.
    ++oi;
  }

  // Disturb pass: a carried element whose pixels anything touched —
  // an erased region, the repaint band, or the new sweep — repaints
  // (with identical pixels, so nothing visibly changes).
  for (Desired& d : g_desired) {
    if (d.redraw || d.old < 0) continue;
    if (disturbed(fb.elems[d.old], haveNewSweep ? &newSweep : nullptr)) {
      d.redraw = true;
    }
  }

  // Draw pass: the sweep first (under everything), then the changed
  // elements in z order; unchanged elements keep their pixels and carry
  // their bookkeeping forward.
  uint32_t onlineAge = millis() - g_onlineSinceMs;
  if (model.ui.online) {
    // The sweep only turns once signal is acquired, fading in over the
    // handoff window; offline it is neither drawn nor tracked.
    float fade = onlineAge >= kSweepFadeMs
                     ? 1.0f
                     : onlineAge / static_cast<float>(kSweepFadeMs);
    drawSweep(g_live, model, fade);
    g_curSweepX = newSweep.x1;
    g_curSweepY = newSweep.y1;
    g_curHaveSweep = true;
  } else {
    g_curHaveSweep = false;
  }

  g_newElems.clear();
  for (const Desired& d : g_desired) {
    Elem e;
    if (d.redraw) {
      drawElem(model, d, e);
    } else {
      e = fb.elems[d.old];
    }
    g_newElems.push_back(e);
  }
  fb.elems.swap(g_newElems);
  fb.sweepX = g_curSweepX;
  fb.sweepY = g_curSweepY;
  fb.haveSweep = g_curHaveSweep;

  // Build slices: spend whatever remains of the frame budget painting
  // the pipeline's spare reference (off-screen, so only time matters).
  if (g_pipe == Pipe::Build && !pipeYield()) {
    if (buildRun(model, frameStartUs, kFrameBudgetUs)) {
      // The spare now holds the drifted scene: make it the active
      // reference and start converging both framebuffers on it.
      std::swap(g_static, g_spare);
      std::swap(g_staticFb, g_spareFb);
      g_sig = g_pipeSig;
      g_fb[0].repaintRow = 0;
      g_fb[1].repaintRow = 0;
      g_pipe = Pipe::Repaint;
    }
  }

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

  // Geography draws only when the view is locked on its target (home, or a
  // followed flight it is steady-tracking) — not while transitioning between
  // them, where a lagging coastline mid-pan looks worse than none. Hysteresis
  // on the view-to-target gap keeps it from flickering at the boundary.
  float gapPx = freeze ? 0.0f
                       : hypotf(target.x - model.ui.viewCenter.x,
                                target.y - model.ui.viewCenter.y) *
                             pixelsPerNm(model);
  if (gapPx > 8.0f) {
    g_viewSettled = false;
  } else if (gapPx < 2.0f) {
    g_viewSettled = true;
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
      ac.shownTrack = ac.track;
      ac.shownSpeed = ac.groundSpeed;
      ac.shownAlt = ac.altitude;
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
