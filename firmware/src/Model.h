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
};

// The whole shared world plus UI state.
struct Model {
  Ui ui;
  std::vector<Aircraft> aircraft;
  std::vector<Poi> pois;
  std::vector<Home> homes;
  std::vector<Feed> feeds;
};

}  // namespace model
