#include "Display.h"

#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>

#include "config.h"

namespace display {
namespace {

// The expander doubles as the bit-banged SPI used to configure the panel
// controller and as the backlight control line.
Arduino_XCA9554SWSPI* g_expander = nullptr;

// The ESP32-S3 RGB LCD peripheral, driven directly so we can run with a
// single PSRAM framebuffer that the renderer updates in place.
esp_lcd_panel_handle_t g_panel = nullptr;

// The render task (core-1 loop) that LiveGfx::flush() blocks on, notified
// from the panel's VSYNC ISR so the loop paces to the panel refresh.
volatile TaskHandle_t g_renderTask = nullptr;

// The live surface (aimed at the scanned framebuffer) and the off-screen
// static-reference surface, plus their raw framebuffers.
LiveGfx* g_live = nullptr;
StaticGfx* g_static = nullptr;
uint16_t* g_liveFb = nullptr;
uint16_t* g_staticFb = nullptr;

// Fires from the RGB driver ISR at each VSYNC. Waking the render task here
// paces one rendered frame per panel refresh (~24.6 Hz) with no busy-wait.
bool IRAM_ATTR onVsync(esp_lcd_panel_handle_t,
                       const esp_lcd_rgb_panel_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  if (g_renderTask) vTaskNotifyGiveFromISR(g_renderTask, &woken);
  return woken == pdTRUE;
}

}  // namespace

Arduino_GFX* begin() {
  g_renderTask = xTaskGetCurrentTaskHandle();  // The core-1 render loop.

  Wire.begin(config::I2C_SDA, config::I2C_SCL);

  g_expander = new Arduino_XCA9554SWSPI(
      config::EXP_TFT_RESET, config::EXP_TFT_CS, config::EXP_TFT_SCK,
      config::EXP_TFT_MOSI, &Wire, config::EXPANDER_ADDR);
  g_expander->begin();

  // Configure the HD40015C40 controller over the expander's bit-banged SPI,
  // exactly as Arduino_RGB_Display would when no reset pin is wired: software
  // reset then the panel's init-operations table (product 5793 / HD40015C40 /
  // NV3052C — not ST7701). `hd40015c40_init_operations` ships in Arduino_GFX.
  g_expander->sendCommand(0x01);  // Software reset.
  delay(120);
  g_expander->batchOperation((uint8_t*)hd40015c40_init_operations,
                             sizeof(hd40015c40_init_operations));

  // A SINGLE framebuffer in PSRAM (no flip) plus a bounce buffer so the LCD
  // FIFO refills from internal SRAM and cannot underrun when PSRAM is busy.
  // This is IDF's stable "bounce buffer with single PSRAM frame buffer" mode:
  // the CPU refills the bounce buffer from the PSRAM framebuffer, so the CPU
  // writes we make into that framebuffer stay cache-coherent with scan-out.
  // Timings are the 4" round panel's; the pixel clock is its 16 MHz spec (the
  // library default of 12 MHz refreshes at a flickery ~18 Hz). Data-pin order
  // mirrors Arduino_ESP32RGBPanel. 720*720 is divisible by the bounce size.
  esp_lcd_rgb_panel_config_t cfg = {
      .clk_src = LCD_CLK_SRC_DEFAULT,
      .timings =
          {
              .pclk_hz = 16000000,
              .h_res = config::PANEL_WIDTH,
              .v_res = config::PANEL_HEIGHT,
              .hsync_pulse_width = 2,
              .hsync_back_porch = 44,
              .hsync_front_porch = 46,
              .vsync_pulse_width = 16,
              .vsync_back_porch = 16,
              .vsync_front_porch = 50,
              .flags =
                  {
                      .hsync_idle_low = 0,
                      .vsync_idle_low = 0,
                      .de_idle_high = 0,
                      .pclk_active_neg = 0,
                      .pclk_idle_high = 0,
                  },
          },
      .data_width = 16,
      .bits_per_pixel = 16,
      .num_fbs = 1,
      .bounce_buffer_size_px = (size_t)(config::PANEL_WIDTH * 20),
      .sram_trans_align = 8,
      .psram_trans_align = 64,
      .hsync_gpio_num = config::TFT_HSYNC,
      .vsync_gpio_num = config::TFT_VSYNC,
      .de_gpio_num = config::TFT_DE,
      .pclk_gpio_num = config::TFT_PCLK,
      .disp_gpio_num = GPIO_NUM_NC,
      .data_gpio_nums =
          {
              config::TFT_B[0], config::TFT_B[1], config::TFT_B[2],
              config::TFT_B[3], config::TFT_B[4], config::TFT_G[0],
              config::TFT_G[1], config::TFT_G[2], config::TFT_G[3],
              config::TFT_G[4], config::TFT_G[5], config::TFT_R[0],
              config::TFT_R[1], config::TFT_R[2], config::TFT_R[3],
              config::TFT_R[4],
          },
      .flags =
          {
              .disp_active_low = 1,
              .refresh_on_demand = 0,
              .fb_in_psram = 1,
              .double_fb = 0,
              .no_fb = 0,
              .bb_invalidate_cache = 0,
          },
  };

  if (esp_lcd_new_rgb_panel(&cfg, &g_panel) != ESP_OK) {
    Serial.println("[display] esp_lcd_new_rgb_panel failed.");
    return nullptr;
  }

  // Register the VSYNC callback BEFORE init (per the IDF driver), so flush()
  // can pace the render loop to the panel refresh.
  esp_lcd_rgb_panel_event_callbacks_t cbs = {};
  cbs.on_vsync = onVsync;
  esp_lcd_rgb_panel_register_event_callbacks(g_panel, &cbs, nullptr);

  esp_lcd_panel_reset(g_panel);
  esp_lcd_panel_init(g_panel);

  void* fb0 = nullptr;
  if (esp_lcd_rgb_panel_get_frame_buffer(g_panel, 1, &fb0) != ESP_OK) {
    Serial.println("[display] get_frame_buffer(1) failed.");
    return nullptr;
  }
  g_liveFb = static_cast<uint16_t*>(fb0);

  // The static-reference framebuffer is a second full-panel buffer in PSRAM.
  // The renderer paints the unchanging scene into it and copies sub-regions
  // back into the live buffer to erase moving elements.
  size_t fbSize = (size_t)config::PANEL_WIDTH * config::PANEL_HEIGHT * 2;
  g_staticFb = static_cast<uint16_t*>(
      heap_caps_aligned_alloc(16, fbSize, MALLOC_CAP_SPIRAM));
  if (g_staticFb == nullptr) {
    Serial.println("[display] static framebuffer alloc failed.");
    return nullptr;
  }

  g_live = new LiveGfx(config::PANEL_WIDTH, config::PANEL_HEIGHT, g_liveFb);
  g_static = new StaticGfx(config::PANEL_WIDTH, config::PANEL_HEIGHT, g_staticFb);
  Serial.printf(
      "[display] panel up: %dx%d, single-fb, dirty-rect, vsync-paced.\n",
      config::PANEL_WIDTH, config::PANEL_HEIGHT);

  // Clear the scanned buffer before the backlight comes on. There is no flip:
  // these writes are what the panel shows.
  g_live->fillScreen(RGB565_BLACK);

  setBacklight(true);
  return g_live;
}

LiveGfx* live() { return g_live; }
Arduino_GFX* staticRef() { return g_static; }
uint16_t* liveFb() { return g_liveFb; }
uint16_t* staticFb() { return g_staticFb; }
int16_t width() { return config::PANEL_WIDTH; }
int16_t height() { return config::PANEL_HEIGHT; }

void setBacklight(bool on) {
  if (g_expander == nullptr) {
    return;
  }
  g_expander->pinMode(config::EXP_BACKLIGHT, OUTPUT);
  g_expander->digitalWrite(config::EXP_BACKLIGHT, on ? HIGH : LOW);
}

}  // namespace display
