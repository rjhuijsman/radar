// Shared data model: the world (aircraft, points of interest, homes, feeds)
// and the transient UI state (range, display mode, follow selection). One
// `Model` instance is owned by `main` and shared between the network task
// (which fills the world from feeds) and the display task (which renders
// it). Access is guarded by a mutex held in `main`.

#pragma once

#include <Arduino.h>

#include <vector>

namespace model {

// A position in world nautical miles relative to the active home: x east,
// y north. Home sits at the origin.
struct Vec {
  float x = 0;
  float y = 0;
};

struct Aircraft {
  String callsign;   // ICAO callsign as broadcast, e.g. "BAW117".
  Vec pos;           // Live position, world NM from home.
  float track = 0;   // Degrees, 0 = north.
  float groundSpeed = 0;  // Knots.
  int32_t altitude = 0;   // Feet.
  bool special = false;   // Matched to an enabled iCal feed (or manual).
  uint16_t color = 0;     // Feed color (special flights); 0 = default amber.

  // Sweep-refresh bookkeeping: `shown` is the last position the sweep
  // painted; the blip is drawn there until the sweep passes again.
  Vec shown;
  bool seen = false;
  float freshness = 0;  // 1 just after a sweep pass, decays toward 0.

  // Smooth-follow bookkeeping: `fixMs` is millis() when the feed last
  // updated `pos` (0 until first merged), and `est` is the smoothed
  // display position the followed flight is drawn at — dead-reckoned
  // along the track between fixes, eased through each fix's correction.
  uint32_t fixMs = 0;
  Vec est;
};

struct Poi {
  String name;
  float latitude = 0;   // Geographic anchor, persisted in config.
  float longitude = 0;
  Vec pos;              // World NM relative to the active home (derived).
  bool isAirport = false;
  std::vector<float> runwayHeadings;  // Degrees; empty for non-airports.
};

struct Home {
  String name;
  float latitude = 0;
  float longitude = 0;
};

// A calendar feed that contributes special flights.
struct Feed {
  String name;
  String url;      // Private iCal URL (the token is the credential).
  uint16_t color = 0;
  bool enabled = true;
};

// The rain-radar layer published by feeds::pollWeather: RainViewer
// reflectivity decoded into a Web-Mercator pixel mosaic the renderer
// samples at static-rebuild time. The mosaic is georeferenced absolutely
// (tile origin in global pixels at `zoom`), so it stays correct across
// home switches and zooms; only its coverage goes stale. `cells` lives in
// PSRAM and is swapped (old buffer freed) under the model mutex.
struct WeatherLayer {
  uint8_t* cells = nullptr;   // Per-pixel dBZ + 32; 0 = no echo. Row-major.
  int16_t width = 0;          // Mosaic size in pixels.
  int16_t height = 0;
  uint8_t zoom = 0;           // Web-Mercator zoom of the source tiles.
  int32_t originX = 0;        // Mosaic top-left, in global pixels at `zoom`
  int32_t originY = 0;        // (tile index * 256).
  uint32_t fetchedMs = 0;     // millis() when fetched; gates staleness.
  uint32_t generation = 0;    // Bumped per publish; re-fingerprints statics.
};

// The 2-position Display toggle: flights layer or weather layer.
enum class DisplayMode { Flights, Weather };

// Screen power, including the transition animations.
enum class Screen { On, PoweringOff, Off, PoweringOn };

// Transient interaction state driven by the encoder and toggles.
struct Ui {
  float range = 40.0f;         // Current display range in NM.
  DisplayMode display = DisplayMode::Flights;
  bool geography = false;      // Geography overlay on/off.
  Screen screen = Screen::On;

  bool zoomHold = false;       // Encoder held: turning zooms, not browses.
  bool following = false;      // Following a flight vs. a home view.
  int followIndex = -1;        // Index into `aircraft` when following.
  int homeIndex = 0;           // Active home when not following.
  int browseSel = -1;          // Home-mode reticle target, -1 = none.
  int candidate = -2;          // Follow-mode browse: -2 none, -1 home, >=0 ac.

  Vec viewCenter;              // Animated center; eases toward its target.
  float sweepAngle = 200;      // Degrees.
  float brightness = 1.0f;     // Ambient-light dimming scalar.
  bool otaActive = false;      // An OTA update is in progress; defer sleep.
  bool online = false;         // Wi-Fi associated; false shows "acquiring".

  String lastInput;            // Last control event, flashed as a wiring test.
  uint32_t lastInputMs = 0;    // millis() when lastInput was recorded.
  uint32_t lastZoomMs = 0;     // Last zoom detent; emphasizes the range readout.
};

// The whole shared world plus UI state.
struct Model {
  Ui ui;
  std::vector<Aircraft> aircraft;
  std::vector<Poi> pois;
  std::vector<Home> homes;
  std::vector<Feed> feeds;
  WeatherLayer weather;

  // Traffic poll request, consumed (cleared) by the network task. Set by
  // the renderer when the sweep crosses 12 o'clock — phase-locking fresh
  // data to the sweep reaching top — and by a home switch, so the scope
  // repopulates promptly around the new home. Starts true for an
  // immediate first poll on boot.
  bool adsbPollDue = true;
};

}  // namespace model
