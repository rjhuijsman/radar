#include "Inputs.h"

#include <Adafruit_VEML7700.h>
#include <Modulino.h>
#include <Wire.h>

#include <vector>

#include "config.h"

namespace inputs {
namespace {

using model::DisplayMode;
using model::Model;
using model::Screen;

ModulinoKnob g_knob;  // Arduino Modulino Knob (I2C, auto-discovered at 0x3B).
Adafruit_VEML7700 g_light;
bool g_knobOk = false;
bool g_lightOk = false;

bool g_buttonDown = false;
bool g_turnedWhileDown = false;  // A turn during a hold means "zoom", not "click".
bool g_lastSleep = false;
uint32_t g_lastLightMs = 0;

// Records a control event so the offline "acquiring signal" screen can flash
// it as a wiring test. Called under the model mutex (poll() already holds it).
void noteInput(Model& model, const char* msg) {
  model.ui.lastInput = msg;
  model.ui.lastInputMs = millis();
}

// Steps `current` one detent along `list` (wrapping) and returns the new
// value. Used to browse the target/candidate rings.
int stepInList(int current, int direction, const std::vector<int>& list) {
  int index = 0;
  for (size_t i = 0; i < list.size(); ++i) {
    if (list[i] == current) {
      index = static_cast<int>(i);
      break;
    }
  }
  int size = static_cast<int>(list.size());
  index = (index + direction + size) % size;
  return list[index];
}

void startFollow(Model& model, int aircraftIndex) {
  model.ui.following = true;
  model.ui.followIndex = aircraftIndex;
  model.ui.candidate = -2;
  model.ui.browseSel = -1;
}

void goHome(Model& model) {
  model.ui.following = false;
  model.ui.followIndex = -1;
  model.ui.candidate = -2;
  model.ui.browseSel = -1;
}

// One detent turn while browsing (encoder not held).
void rotarySelect(Model& model, int direction) {
  int count = static_cast<int>(model.aircraft.size());
  if (!model.ui.following) {
    std::vector<int> list = {-1};  // -1 = no target selected.
    for (int i = 0; i < count; ++i) list.push_back(i);
    model.ui.browseSel = stepInList(model.ui.browseSel, direction, list);
  } else {
    std::vector<int> list = {-2};  // -2 = no candidate.
    for (int i = 0; i < count; ++i) list.push_back(i);
    list.push_back(-1);  // -1 = the Home entry.
    model.ui.candidate = stepInList(model.ui.candidate, direction, list);
  }
}

// One detent turn while the encoder is held.
void rotaryZoom(Model& model, int direction) {
  float factor = direction > 0 ? (1.0f / 1.16f) : 1.16f;
  float range = model.ui.range * factor;
  range = max(config::MIN_RANGE_NM, min(config::MAX_RANGE_NM, range));
  model.ui.range = range;
}

// A committing press.
void press(Model& model) {
  if (!model.ui.following) {
    if (model.ui.browseSel >= 0) {
      startFollow(model, model.ui.browseSel);
    } else if (!model.homes.empty()) {
      model.ui.homeIndex = (model.ui.homeIndex + 1) %
                           static_cast<int>(model.homes.size());
    }
  } else {
    if (model.ui.candidate >= 0) {
      startFollow(model, model.ui.candidate);
    } else {
      goHome(model);  // Candidate is Home or none.
    }
  }
}

void readToggles(Model& model) {
  // 2-position Display toggle: closed (LOW) selects Weather, open (HIGH)
  // selects Flights. The polarity is trivially flipped by swapping the two
  // modes here. Note only on an actual change, so it does not spam the poll.
  DisplayMode display = digitalRead(config::PIN_DISPLAY) == LOW
                            ? DisplayMode::Weather
                            : DisplayMode::Flights;
  if (display != model.ui.display) {
    model.ui.display = display;
    noteInput(model, display == DisplayMode::Weather ? "DISPLAY: WEATHER"
                                                     : "DISPLAY: FLIGHTS");
  }

  bool geo = digitalRead(config::PIN_GEO) == LOW;
  if (geo != model.ui.geography) {
    model.ui.geography = geo;
    noteInput(model, geo ? "GEOGRAPHY: ON" : "GEOGRAPHY: OFF");
  }

  // Sleep (power switch, inverted): OPEN (HIGH) is the OFF position, CLOSED
  // (LOW) is ON. Request deep sleep on the transition into OFF (HIGH), while
  // the screen is on and no OTA update is running (never interrupt a flash).
  bool sleep = digitalRead(config::PIN_SLEEP) == HIGH;
  if (sleep && !g_lastSleep && model.ui.screen == Screen::On &&
      !model.ui.otaActive) {
    model.ui.screen = Screen::PoweringOff;
  }
  g_lastSleep = sleep;
}

void readLight(Model& model) {
  if (!g_lightOk || millis() - g_lastLightMs < 1000) {
    return;
  }
  g_lastLightMs = millis();
  float lux = g_light.readLux();
  // Map room brightness to a dimming scalar; the backlight is on/off only,
  // so this scales the drawn colors instead. Dark room floors at 0.35.
  float scalar = 0.35f + 0.65f * min(1.0f, lux / 400.0f);
  model.ui.brightness = scalar;
}

void readKnob(Model& model) {
  if (!g_knobOk) {
    return;
  }
  bool down = g_knob.isPressed();  // Active HIGH on the Modulino (no inversion).
  if (down && !g_buttonDown) {
    g_buttonDown = true;
    g_turnedWhileDown = false;
  }

  // getDirection() debounces the quadrature into one clean +1/-1 per physical
  // detent. The raw get() count jitters mid-detent (a single click can briefly
  // read the reverse), which showed up as a spurious CCW right after a CW turn.
  int8_t dir = g_knob.getDirection();
  if (dir != 0) {
    int direction = dir > 0 ? 1 : -1;
    noteInput(model, direction > 0 ? "KNOB CW" : "KNOB CCW");
    if (g_buttonDown) {
      g_turnedWhileDown = true;
      rotaryZoom(model, direction);
    } else {
      rotarySelect(model, direction);
    }
  }
  model.ui.zoomHold = g_buttonDown;

  if (!down && g_buttonDown) {
    g_buttonDown = false;
    if (!g_turnedWhileDown) {
      noteInput(model, "KNOB PRESS");
      press(model);  // A quick click with no turn commits.
    }
  }
}

}  // namespace

void begin() {
  pinMode(config::PIN_SLEEP, INPUT_PULLUP);
  pinMode(config::PIN_DISPLAY, INPUT_PULLUP);
  pinMode(config::PIN_GEO, INPUT_PULLUP);

  // display::begin() already ran Wire.begin(SDA=8, SCL=18) before this task
  // starts, so the bus is up. Modulino.begin() calls the no-arg Wire.begin(),
  // which reuses that bus — do NOT re-init Wire with different pins here.
  Modulino.begin();
  g_knobOk = g_knob.begin();  // Auto-discovers the Knob at 7-bit 0x3B.
  if (g_knobOk) {
    g_knob.set(0);  // Zero the count so getDirection() starts clean.
  }
  g_lightOk = g_light.begin(&Wire);
}

void poll(Model& model) {
  readKnob(model);
  readToggles(model);
  readLight(model);
}

}  // namespace inputs
