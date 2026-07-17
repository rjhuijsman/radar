// Entry point and orchestration.
//
// Core 1 (the Arduino `loop`) owns the panel and the render loop, including
// the power on/off CRT animation. Core 0 runs two tasks: reading the
// controls at ~50 Hz, and polling the network feeds. The shared model is
// guarded by one mutex.
//
// Waking from deep sleep is a fresh boot, so there is no persisted runtime
// state to restore — `setup()` simply runs again and plays the power-on
// bloom.

#include <Arduino.h>

#include "Display.h"
#include "Feeds.h"
#include "Inputs.h"
#include "Model.h"
#include "Net.h"
#include "Power.h"
#include "Radar.h"
#include "config.h"

namespace {

model::Model g_model;
SemaphoreHandle_t g_mutex = nullptr;
Arduino_GFX* g_gfx = nullptr;

void seedDefaultHomeIfEmpty() {
  if (!g_model.homes.empty()) return;
  model::Home home;
  home.name = "Home";
  g_model.homes.push_back(home);
}

// Core 0: read the encoder, toggles and light sensor at ~50 Hz.
void inputsTask(void*) {
  inputs::begin();
  for (;;) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    inputs::poll(g_model);
    xSemaphoreGive(g_mutex);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Core 0: bring up Wi-Fi and the config server, then poll the feeds.
void networkTask(void*) {
  bool ok = net::begin(g_model, g_mutex);
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  g_model.ui.online = ok;
  seedDefaultHomeIfEmpty();
  feeds::reprojectStatics(g_model);
  xSemaphoreGive(g_mutex);

  bool wasOnline = ok;
  uint32_t lastAdsb = 0, lastIcal = 0, lastWeather = 0;
  for (;;) {
    net::loopOta();  // Pick up any pending Wi-Fi update promptly.

    // Reflect Wi-Fi drops/reconnects to the renderer (lock only on change).
    bool nowOnline = net::online();
    if (nowOnline != wasOnline) {
      wasOnline = nowOnline;
      xSemaphoreTake(g_mutex, portMAX_DELAY);
      g_model.ui.online = nowOnline;
      xSemaphoreGive(g_mutex);
    }

    uint32_t now = millis();
    if (now - lastIcal >= config::ICAL_POLL_MS || lastIcal == 0) {
      lastIcal = now;
      xSemaphoreTake(g_mutex, portMAX_DELAY);
      feeds::pollIcal(g_model);
      xSemaphoreGive(g_mutex);
    }
    if (now - lastAdsb >= config::ADSB_POLL_MS) {
      lastAdsb = now;
      xSemaphoreTake(g_mutex, portMAX_DELAY);
      feeds::pollTraffic(g_model);
      xSemaphoreGive(g_mutex);
    }
    if (now - lastWeather >= config::WEATHER_POLL_MS) {
      lastWeather = now;
      xSemaphoreTake(g_mutex, portMAX_DELAY);
      feeds::pollWeather(g_model);
      xSemaphoreGive(g_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);  // Let the USB-CDC console attach before the first logs.
  Serial.printf("\n\n[radar] === boot ===\n");
  Serial.printf("[radar] chip %s rev %d, %d core(s) @ %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(),
                ESP.getChipCores(), getCpuFrequencyMhz());
  Serial.printf("[radar] flash %u MB, PSRAM %u/%u KB free, heap %u KB free\n",
                ESP.getFlashChipSize() / (1024 * 1024),
                ESP.getFreePsram() / 1024, ESP.getPsramSize() / 1024,
                ESP.getFreeHeap() / 1024);

  g_mutex = xSemaphoreCreateMutex();

  g_gfx = display::begin();
  Serial.printf("[radar] display: %s\n",
                g_gfx ? "up" : "FAILED (gfx=null, rendering disabled)");
  if (g_gfx != nullptr) {
    radar::beginRenderer(display::live(), display::staticRef(),
                         display::liveFb(), display::staticFb(),
                         display::width(), display::height());
  }
  seedDefaultHomeIfEmpty();

  // Every boot (cold or wake-from-sleep) opens with the power-on bloom.
  g_model.ui.screen = model::Screen::PoweringOn;

  xTaskCreatePinnedToCore(inputsTask, "inputs", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0);
  Serial.println("[radar] setup complete; core-0 tasks running.");
}

void loop() {
  static uint32_t last = 0;
  static float animProgress = 0;
  uint32_t now = millis();
  uint32_t dt = last == 0 ? 16 : (now - last);
  if (dt > 80) dt = 80;  // Clamp after a stall so animations do not jump.
  last = now;

  if (g_gfx == nullptr) {
    delay(100);
    return;
  }

  xSemaphoreTake(g_mutex, portMAX_DELAY);
  model::Screen screen = g_model.ui.screen;

  if (screen == model::Screen::PoweringOn) {
    animProgress += static_cast<float>(dt) / 820.0f;
    if (radar::renderTransition(g_gfx, g_model, animProgress, true)) {
      g_model.ui.screen = model::Screen::On;
      animProgress = 0;
      // The transition left arbitrary content in the live buffer; make the
      // first steady-state frame rebuild the reference and repaint in full.
      radar::invalidate();
    }
  } else if (screen == model::Screen::PoweringOff) {
    animProgress += static_cast<float>(dt) / 720.0f;
    bool done = radar::renderTransition(g_gfx, g_model, animProgress, false);
    xSemaphoreGive(g_mutex);
    g_gfx->flush();  // Present the collapse frame (drawing is off-screen).
    if (done) {
      power::deepSleep();  // Does not return; wakes into a clean boot.
    }
    return;
  } else {
    radar::step(g_model, dt);
    radar::renderIncremental(g_model);
  }
  xSemaphoreGive(g_mutex);
  g_gfx->flush();  // Block until the next vsync to pace the render loop.
}
