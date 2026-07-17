#include "Inputs.h"

#include <Adafruit_VEML7700.h>
#include <Adafruit_seesaw.h>
#include <Wire.h>

#include <vector>

#include "config.h"

namespace inputs {
namespace {

using model::DisplayMode;
using model::Model;
using model::Screen;

// The seesaw exposes the encoder push button on this pin.
constexpr uint8_t SEESAW_SWITCH = 24;

Adafruit_seesaw g_encoder;
Adafruit_VEML7700 g_light;
bool g_encoderOk = false;
bool g_lightOk = false;

int32_t g_lastEncoder = 0;
bool g_buttonDown = false;
bool g_turnedWhileDown = false;  // A turn during a hold means "zoom", not "click".
bool g_lastSleep = false;
uint32_t g_lastLightMs = 0;

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
  // 3-way Display, on-off-on: each throw shorts one pin to GND. Center
  // (both open) is "both".
  bool a = digitalRead(config::PIN_DISPLAY_A) == LOW;
  bool b = digitalRead(config::PIN_DISPLAY_B) == LOW;
  if (a && !b) {
    model.ui.display = DisplayMode::Flights;
  } else if (!a && b) {
    model.ui.display = DisplayMode::Weather;
  } else {
    model.ui.display = DisplayMode::Both;
  }

  model.ui.geography = digitalRead(config::PIN_GEO) == LOW;

  // Sleep: closed (LOW) requests deep sleep. Trigger the power-down once,
  // on the transition, while the screen is on and no OTA update is running
  // (an update must not be interrupted mid-flash).
  bool sleep = digitalRead(config::PIN_SLEEP) == LOW;
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

void readEncoder(Model& model) {
  if (!g_encoderOk) {
    return;
  }
  bool down = !g_encoder.digitalRead(SEESAW_SWITCH);  // Active low.
  if (down && !g_buttonDown) {
    g_buttonDown = true;
    g_turnedWhileDown = false;
  }

  int32_t position = g_encoder.getEncoderPosition();
  int32_t delta = position - g_lastEncoder;
  g_lastEncoder = position;
  int direction = delta > 0 ? 1 : -1;
  for (int32_t i = 0; i < abs(delta); ++i) {
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
      press(model);  // A quick click with no turn commits.
    }
  }
}

}  // namespace

void begin() {
  pinMode(config::PIN_SLEEP, INPUT_PULLUP);
  pinMode(config::PIN_DISPLAY_A, INPUT_PULLUP);
  pinMode(config::PIN_DISPLAY_B, INPUT_PULLUP);
  pinMode(config::PIN_GEO, INPUT_PULLUP);

  g_encoderOk = g_encoder.begin(config::ENCODER_ADDR);
  if (g_encoderOk) {
    g_encoder.pinMode(SEESAW_SWITCH, INPUT_PULLUP);
    g_lastEncoder = g_encoder.getEncoderPosition();
  }
  g_lightOk = g_light.begin(&Wire);
}

void poll(Model& model) {
  readEncoder(model);
  readToggles(model);
  readLight(model);
}

}  // namespace inputs
