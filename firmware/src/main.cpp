// Entry point and orchestration.
//
// Core 1 (the Arduino `loop`) owns the panel and the render loop, including
// the power on/off CRT animation. Core 0 runs two tasks: reading the
// controls at ~50 Hz, and polling the network feeds. The shared model is
// guarded by one mutex.
//
// Waking from deep sleep is a fresh boot, so there is no persisted runtime
// state to restore — `setup()` simply runs again and plays the power-on
// bloom. A cold power-up (or reset) with the key switch OFF instead shows a
// brief "MISSING KEY" notice and goes straight back to deep sleep; the set
// then only comes up when the key is turned on (the ext0 wake).
//
// The backlight is dark through all of bring-up: display::begin() leaves it
// off, and `loop` lights it only after the first finished frame has been
// presented, so panel-init garbage and first-paint are never seen.

#include <Arduino.h>

#include "Display.h"
#include "Feeds.h"
#include "Inputs.h"
#include "Model.h"
#include "Net.h"
#include "Power.h"
#include "Radar.h"
#include "config.h"

// Startup-latency breadcrumbs: timestamped serial lines for pinning down
// where boot time goes (the wake-to-first-picture investigation). Flip to 0
// once the timings are confirmed good on hardware; the [fps] probe in loop()
// is under the same switch.
#define BOOT_TRACE 0
#if BOOT_TRACE
#define BOOT_LOG(...) Serial.printf(__VA_ARGS__)
#else
#define BOOT_LOG(...)
#endif

namespace {

model::Model g_model;
SemaphoreHandle_t g_mutex = nullptr;
Arduino_GFX* g_gfx = nullptr;

#if BOOT_TRACE
// Boot checkpoints, replayed once the USB console has attached (boot-time
// prints are lost while the CDC port is still re-enumerating).
uint32_t g_tSetupStart = 0, g_tDisplayUp = 0, g_tFirstFrame = 0,
         g_tBloomDone = 0;
#endif

void seedDefaultHomeIfEmpty() {
  if (!g_model.homes.empty()) return;
  // Fallbacks for a fresh or wiped filesystem, so the set still comes up
  // on the real homes; a saved config replaces these when it loads.
  model::Home gorssel;
  gorssel.name = "GORSSEL";
  gorssel.latitude = 52.19958603499725f;
  gorssel.longitude = 6.198159997488504f;
  g_model.homes.push_back(gorssel);
  model::Home carl;
  carl.name = "CARL";
  carl.latitude = 55.66595793935845f;
  carl.longitude = 12.531574542965174f;
  g_model.homes.push_back(carl);
}

// Core 0: read the encoder, toggles and light sensor at ~50 Hz.
void inputsTask(void*) {
  inputs::begin();
  BOOT_LOG("[boot] t=%6lu inputs up (core 0)\n", millis());
  for (;;) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    inputs::poll(g_model);
    xSemaphoreGive(g_mutex);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Core 0: bring up Wi-Fi and the config server, then poll the feeds. The
// polls take the model mutex themselves, and only briefly — to copy their
// inputs out and merge parsed results in — never across a blocking HTTP
// fetch, which would starve the render loop of the mutex for seconds and
// visibly stall the picture every poll.
void networkTask(void*) {
  BOOT_LOG("[boot] t=%6lu net::begin start (core 0)\n", millis());
  bool ok = net::begin(g_model, g_mutex);
  BOOT_LOG("[boot] t=%6lu net::begin done, online=%d (core 0)\n", millis(),
           ok ? 1 : 0);
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
      feeds::pollIcal(g_model, g_mutex);
    }
    if (now - lastAdsb >= config::ADSB_POLL_MS || lastAdsb == 0) {
      lastAdsb = now;
      feeds::pollTraffic(g_model, g_mutex);
    }
    if (now - lastWeather >= config::WEATHER_POLL_MS) {
      lastWeather = now;
      feeds::pollWeather(g_model, g_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  // Never let logging slow the boot: with no USB host attached, HWCDC writes
  // otherwise stall until a timeout once its buffer fills. Drop instead.
  Serial.setTxTimeoutMs(0);
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

#if BOOT_TRACE
  g_tSetupStart = millis();
#endif
  BOOT_LOG("[boot] t=%6lu setup start\n", millis());
  g_gfx = display::begin();
#if BOOT_TRACE
  g_tDisplayUp = millis();
#endif
  BOOT_LOG("[boot] t=%6lu display::begin returned\n", millis());
  Serial.printf("[radar] display: %s\n",
                g_gfx ? "up" : "FAILED (gfx=null, rendering disabled)");
  if (g_gfx != nullptr) {
    radar::beginRenderer(display::live(), display::staticRef(),
                         display::liveFb(), display::staticFb(),
                         display::width(), display::height());
  }
  seedDefaultHomeIfEmpty();

  // The key switch is inverted: OFF is OPEN, which reads HIGH. A cold
  // power-up (or reset) with the key off should not boot the radar: show a
  // brief notice and go back to sleep, exactly as if the key had just been
  // turned off. A key-turn wake reads LOW here and boots normally.
  pinMode(config::PIN_SLEEP, INPUT_PULLUP);
  if (digitalRead(config::PIN_SLEEP) == HIGH) {
    BOOT_LOG("[boot] t=%6lu key is OFF: MISSING KEY, then deep sleep\n",
             millis());
    if (g_gfx != nullptr) {
      radar::showMissingKey(g_gfx);  // Drawn dark; the backlight is off.
      g_gfx->flush();                // Present it, then light the panel.
      display::setBacklight(true);
      delay(1500);
    }
    power::deepSleep();  // Does not return; the key turning on wakes us.
  }

  // Every boot (cold or wake-from-sleep) opens with the power-on bloom.
  g_model.ui.screen = model::Screen::PoweringOn;

  // The network task runs at idle priority: a multi-second TLS fetch is
  // near-continuous CPU on core 0, and at any higher priority it starves
  // the idle task until the task watchdog aborts (observed as a reboot
  // loop on the first traffic poll). At priority 0 the scheduler
  // round-robins it with the idle task, so the watchdog is always fed.
  xTaskCreatePinnedToCore(inputsTask, "inputs", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(networkTask, "network", 12288, nullptr, 0, nullptr, 0);
  BOOT_LOG("[boot] t=%6lu core-0 tasks created\n", millis());
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
      // The bloom's reveal left the live buffer matching the static
      // reference and the renderer's bookkeeping consistent, so the scope
      // continues incrementally — no invalidate(), whose full repaint on the
      // scanned buffer would tear.
#if BOOT_TRACE
      g_tBloomDone = millis();
#endif
      BOOT_LOG("[boot] t=%6lu power-on bloom done; steady state\n", millis());
    }
  } else if (screen == model::Screen::PoweringOff) {
    animProgress += static_cast<float>(dt) / 720.0f;
    bool done = radar::renderTransition(g_gfx, g_model, animProgress, false);
    xSemaphoreGive(g_mutex);
    g_gfx->flush();  // Pace the collapse to the panel refresh.
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

  // The backlight is dark through bring-up, so the first thing the panel
  // ever shows is the frame just presented — never init garbage. Light it
  // exactly once, here. (The first frame is always the bloom's opening dot:
  // PoweringOff cannot be the first state, so its early return is safe.)
  static bool backlightLit = false;
  if (!backlightLit) {
    backlightLit = true;
    display::setBacklight(true);
#if BOOT_TRACE
    g_tFirstFrame = millis();
#endif
    BOOT_LOG("[boot] t=%6lu first frame presented; backlight on\n", millis());
  }

#if BOOT_TRACE
  // The console usually attaches after boot (the CDC port re-enumerates on
  // reset), so replay the whole boot timeline once, well after the fact.
  static bool traceDumped = false;
  if (!traceDumped && now >= 6000) {
    traceDumped = true;
    Serial.printf("[boot] ---- boot timeline replay ----\n");
    Serial.printf("[boot] t=%6lu setup start\n", g_tSetupStart);
    display::printBootTrace();
    Serial.printf("[boot] t=%6lu display::begin returned\n", g_tDisplayUp);
    Serial.printf("[boot] t=%6lu first frame presented; backlight on\n",
                  g_tFirstFrame);
    Serial.printf("[boot] t=%6lu power-on bloom done\n", g_tBloomDone);
  }

  // Render-rate probe: proves the steady state still paces at the panel's
  // ~25 Hz refresh. Goes away with BOOT_TRACE.
  static uint32_t fpsStart = 0;
  static uint16_t fpsFrames = 0;
  ++fpsFrames;
  if (now - fpsStart >= 5000) {
    if (fpsStart != 0) {
      BOOT_LOG("[fps] %.1f\n", fpsFrames * 1000.0f / (now - fpsStart));
    }
    fpsStart = now;
    fpsFrames = 0;
  }
#endif
}
