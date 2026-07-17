// Board and project configuration: pin map, I2C addresses, and the few
// tunable constants shared across modules.
//
// The GPIO numbers for the RGB-666 panel come from the Qualia ESP32-S3
// board definition (product 5800). The panel's init-SPI, backlight and
// two onboard buttons are driven through the on-board PCA9554 I2C
// expander at 0x3F, because the RGB bus consumes almost every native
// GPIO. The pins listed under "Controls" are what is left broken out.

#pragma once

#include <stdint.h>

namespace config {

// ---- Panel geometry (Adafruit 5793, 4-inch round). ----
constexpr int PANEL_WIDTH = 720;
constexpr int PANEL_HEIGHT = 720;

// ---- RGB-666 panel bus (native GPIO on the Qualia). ----
constexpr int8_t TFT_DE = 2;
constexpr int8_t TFT_VSYNC = 42;
constexpr int8_t TFT_HSYNC = 41;
constexpr int8_t TFT_PCLK = 1;
// Red R1..R5, green G0..G5, blue B1..B5, in the order the panel expects.
constexpr int8_t TFT_R[5] = {11, 10, 9, 46, 3};
constexpr int8_t TFT_G[6] = {48, 47, 21, 14, 13, 12};
constexpr int8_t TFT_B[5] = {40, 39, 38, 0, 45};

// ---- On-board PCA9554 expander (drives the panel and backlight). ----
// These are expander pin indices (0..7), not GPIO numbers.
constexpr uint8_t EXPANDER_ADDR = 0x3F;
constexpr uint8_t EXP_TFT_SCK = 0;
constexpr uint8_t EXP_TFT_CS = 1;
constexpr uint8_t EXP_TFT_RESET = 2;
constexpr uint8_t EXP_TP_IRQ = 3;
constexpr uint8_t EXP_BACKLIGHT = 4;
constexpr uint8_t EXP_BTN_UP = 5;
constexpr uint8_t EXP_BTN_DN = 6;
constexpr uint8_t EXP_TFT_MOSI = 7;

// ---- Controls (the remaining native GPIO). ----
// Toggles are wired switch-to-GND and read with INPUT_PULLUP, so a closed
// contact reads LOW. Sleep is on an RTC-capable pin so it can wake the
// chip from deep sleep via ext0.
// Pad labels are the Qualia S3 breakout silkscreen. Note the board exposes
// only the TX0 pad (GPIO43), NOT RX0 (GPIO44), so Geography lives on a spare
// SPI-header pad instead.
constexpr int8_t PIN_SLEEP = 17;      // A0 pad, RTC-capable (deep-sleep wake).
constexpr int8_t PIN_DISPLAY_A = 16;  // A1 pad, one throw of the 3-way toggle.
constexpr int8_t PIN_DISPLAY_B = 43;  // TX0 pad, other throw of the 3-way.
constexpr int8_t PIN_GEO = 6;         // MISO pad, geography overlay on/off.

// ---- I2C bus (STEMMA QT). Shared by the expander, encoder and sensor. ----
constexpr int8_t I2C_SDA = 8;
constexpr int8_t I2C_SCL = 18;
constexpr uint8_t ENCODER_ADDR = 0x36;  // Adafruit seesaw QT rotary encoder.
constexpr uint8_t LIGHT_ADDR = 0x10;    // Adafruit VEML7700 light sensor.

// ---- Behavior constants. ----
constexpr uint32_t SWEEP_PERIOD_MS = 6000;   // One revolution of the sweep.
constexpr uint32_t ADSB_POLL_MS = 8000;      // Traffic feed poll interval.
constexpr uint32_t ICAL_POLL_MS = 300000;    // iCal feed poll interval (5 min).
constexpr uint32_t WEATHER_POLL_MS = 300000;  // Rain radar poll interval.
constexpr float DEFAULT_RANGE_NM = 40.0f;
constexpr float MIN_RANGE_NM = 5.0f;
constexpr float MAX_RANGE_NM = 240.0f;

// SoftAP name for the first-run Wi-Fi captive portal.
constexpr char AP_NAME[] = "Radar-Setup";
// mDNS host: reachable as `<MDNS_HOST>.local` once on the network.
constexpr char MDNS_HOST[] = "radar-720";

// ArduinoOTA authenticates each push against this MD5 hash, so the
// password never lives in the firmware in plaintext. Replace it with the
// hash of your own password before flashing:
//   printf '%s' 'your-password' | md5sum
// The example below is the hash of "radar-ota"; pass the plaintext to the
// uploader with `--auth=radar-ota` (see platformio.ini). Set to the empty
// string to disable OTA auth entirely (not recommended).
constexpr char OTA_PASSWORD_HASH[] = "d9fd4b65be29775903e92307d8183b62";

}  // namespace config
