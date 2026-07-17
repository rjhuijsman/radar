#include "Display.h"

#include <Wire.h>

#include "config.h"

namespace display {
namespace {

// The expander doubles as the bit-banged SPI used to configure the ST7701
// and as the backlight control line.
Arduino_XCA9554SWSPI* g_expander = nullptr;
Arduino_ESP32RGBPanel* g_panel = nullptr;
Arduino_RGB_Display* g_gfx = nullptr;

}  // namespace

Arduino_GFX* begin() {
  Wire.begin(config::I2C_SDA, config::I2C_SCL);

  g_expander = new Arduino_XCA9554SWSPI(
      config::EXP_TFT_RESET, config::EXP_TFT_CS, config::EXP_TFT_SCK,
      config::EXP_TFT_MOSI, &Wire, config::EXPANDER_ADDR);

  // Panel timing (front/back porch, pulse width, polarity) is specific to
  // the 4-inch 720x720 module. TODO(hardware): replace the porch/pulse
  // values and the ST7701 init-operations table below with the ones from
  // Adafruit's Qualia S3 `Arduino_GFX` example for the 4"/720 round panel
  // before flashing — these are placeholders with the right shape.
  g_panel = new Arduino_ESP32RGBPanel(
      config::TFT_DE, config::TFT_VSYNC, config::TFT_HSYNC, config::TFT_PCLK,
      config::TFT_R[0], config::TFT_R[1], config::TFT_R[2], config::TFT_R[3],
      config::TFT_R[4], config::TFT_G[0], config::TFT_G[1], config::TFT_G[2],
      config::TFT_G[3], config::TFT_G[4], config::TFT_G[5], config::TFT_B[0],
      config::TFT_B[1], config::TFT_B[2], config::TFT_B[3], config::TFT_B[4],
      1 /* hsync_polarity */, 10 /* hsync_front_porch */,
      8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
      1 /* vsync_polarity */, 10 /* vsync_front_porch */,
      8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);

  g_gfx = new Arduino_RGB_Display(
      config::PANEL_WIDTH, config::PANEL_HEIGHT, g_panel, 0 /* rotation */,
      true /* auto_flush */, g_expander, GFX_NOT_DEFINED,
      // TODO(hardware): pass the 4"/720 panel's ST7701 init operations here.
      nullptr, 0);

  if (!g_gfx->begin()) {
    return nullptr;
  }
  g_gfx->fillScreen(BLACK);
  setBacklight(true);
  return g_gfx;
}

void setBacklight(bool on) {
  if (g_expander == nullptr) {
    return;
  }
  g_expander->pinMode(config::EXP_BACKLIGHT, OUTPUT);
  g_expander->digitalWrite(config::EXP_BACKLIGHT, on ? HIGH : LOW);
}

}  // namespace display
