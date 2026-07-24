// Networking: Wi-Fi provisioning (captive portal on first run), mDNS, and
// the configuration web server, plus loading/saving the config to flash.

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Model.h"

namespace net {

// Mounts the filesystem, loads saved config into `model`, and brings up
// Wi-Fi: joins the strongest saved network in range, falling back to the
// `Radar-Setup` portal when none connects (a network entered there is
// appended to the saved list). On the first connection it also starts
// the web server and arms ArduinoOTA. `mutex` guards `model` against the
// display task. Returns true once connected; false after a portal
// timeout, after which loopWifi() keeps rescanning in the background.
bool begin(model::Model& model, SemaphoreHandle_t mutex);

// Services ArduinoOTA. Call frequently from the network task so a pending
// update is picked up promptly.
void loopOta();

// Keeps Wi-Fi alive after boot: while the station is offline, rescans
// for any saved network every WIFI_RETRY_MS — the core's auto-reconnect
// only retries the AP it lost, so this is what roams the set between
// known networks. Call from the network task; a due rescan blocks for a
// few seconds, so never call this holding the model mutex.
void loopWifi();

// True while the station is associated with an access point.
bool online();

}  // namespace net
