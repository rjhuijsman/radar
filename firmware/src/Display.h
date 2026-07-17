// Panel bring-up for the Qualia + 5793 round display, plus backlight
// control (through the PCA9554 expander) used when entering and leaving
// deep sleep.

#pragma once

#include <Arduino_GFX_Library.h>

namespace display {

// Brings up the RGB-666 panel. Returns the GFX surface everything draws
// onto, or nullptr if initialization failed. Call once from `setup()`.
Arduino_GFX* begin();

// Turns the panel backlight on or off via the expander. I2C must still be
// alive, so call this before `esp_deep_sleep_start()`, never after.
void setBacklight(bool on);

}  // namespace display
