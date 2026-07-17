// Networking: Wi-Fi provisioning (captive portal on first run), mDNS, and
// the configuration web server, plus loading/saving the config to flash.

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Model.h"

namespace net {

// Mounts the filesystem, loads saved config into `model`, brings up Wi-Fi
// (raising the `Radar-Setup` portal if needed), starts the web server, and
// arms ArduinoOTA. `mutex` guards `model` against the display task. Blocks
// until Wi-Fi is provisioned. Returns true once connected.
bool begin(model::Model& model, SemaphoreHandle_t mutex);

// Services ArduinoOTA. Call frequently from the network task so a pending
// update is picked up promptly.
void loopOta();

}  // namespace net
