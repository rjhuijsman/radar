#include "Display.h"

#include <Wire.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>

#include "config.h"

namespace display {
namespace {

// The expander doubles as the bit-banged SPI used to configure the panel
// controller and as the backlight control line.
Arduino_XCA9554SWSPI* g_expander = nullptr;

// The ESP32-S3 RGB LCD peripheral, driven directly so we can allocate TWO
// framebuffers and flip between them; Arduino_RGB_Display hardcodes one.
esp_lcd_panel_handle_t g_panel = nullptr;

// The render task (core-1 loop) that flush() blocks on, notified from the
// panel's frame-complete ISR.
volatile TaskHandle_t g_renderTask = nullptr;

// Fires from the RGB driver ISR once a whole framebuffer has finished scanning
// out and the panel has switched away from the previous one — i.e. the buffer
// we flipped away from is now free to draw into again.
bool IRAM_ATTR onFrameBufComplete(esp_lcd_panel_handle_t,
                                  const esp_lcd_rgb_panel_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  if (g_renderTask) vTaskNotifyGiveFromISR(g_renderTask, &woken);
  return woken == pdTRUE;
}

// Draws through Arduino_Canvas's software rasteriser, but aims its framebuffer
// at one of the panel's two hardware framebuffers (zero per-frame copy).
// flush() presents the finished buffer, then BLOCKS until the panel has
// switched away from the previous buffer before letting the next frame reuse
// it. That wait is essential: esp_lcd_panel_draw_bitmap() only sets the target
// index — the DMA keeps scanning the OLD buffer until end-of-frame, so drawing
// into it immediately (our per-frame fillScreen) is what caused the full-screen
// blink. The wait also paces rendering to the panel refresh (~24.6 Hz).
class DoubleBufferGfx : public Arduino_Canvas {
 public:
  DoubleBufferGfx(int16_t w, int16_t h, esp_lcd_panel_handle_t panel,
                  uint16_t* fb0, uint16_t* fb1)
      : Arduino_Canvas(w, h, nullptr), _panel(panel) {
    _fb[0] = fb0;
    _fb[1] = fb1;
    _framebuffer = _fb[_cur];  // Draw into the back buffer.
  }

  // The panel and framebuffers already exist; nothing to allocate.
  bool begin(int32_t = GFX_NOT_DEFINED) override { return true; }

  void flush(bool = false) override {
    ulTaskNotifyTake(pdTRUE, 0);  // Drop any stale completion notification.
    esp_lcd_panel_draw_bitmap(_panel, 0, 0, _width, _height, _framebuffer);
    // Wait until the panel has switched away from the previous buffer (100 ms
    // is a safety cap — at 24.6 Hz a frame is ~41 ms).
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    _cur ^= 1;
    _framebuffer = _fb[_cur];  // Next frame targets the now-free buffer.
  }

 private:
  esp_lcd_panel_handle_t _panel;
  uint16_t* _fb[2];
  uint8_t _cur = 0;
};

DoubleBufferGfx* g_gfx = nullptr;

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

  // Two framebuffers in PSRAM (for the vsync flip) plus a bounce buffer so the
  // LCD FIFO refills from internal SRAM and cannot underrun when PSRAM is busy.
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
      .num_fbs = 2,
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
              .double_fb = 0,  // num_fbs = 2 already asks for two buffers.
              .no_fb = 0,
              .bb_invalidate_cache = 0,
          },
  };

  if (esp_lcd_new_rgb_panel(&cfg, &g_panel) != ESP_OK) {
    Serial.println("[display] esp_lcd_new_rgb_panel failed.");
    return nullptr;
  }

  // Register the frame-complete callback BEFORE init (per the IDF driver), so
  // flush() can block until the panel finishes switching framebuffers.
  esp_lcd_rgb_panel_event_callbacks_t cbs = {};
  cbs.on_frame_buf_complete = onFrameBufComplete;
  esp_lcd_rgb_panel_register_event_callbacks(g_panel, &cbs, nullptr);

  esp_lcd_panel_reset(g_panel);
  esp_lcd_panel_init(g_panel);

  void* fb0 = nullptr;
  void* fb1 = nullptr;
  if (esp_lcd_rgb_panel_get_frame_buffer(g_panel, 2, &fb0, &fb1) != ESP_OK) {
    Serial.println("[display] get_frame_buffer(2) failed.");
    return nullptr;
  }

  g_gfx = new DoubleBufferGfx(config::PANEL_WIDTH, config::PANEL_HEIGHT, g_panel,
                              static_cast<uint16_t*>(fb0),
                              static_cast<uint16_t*>(fb1));
  Serial.printf("[display] panel up: %dx%d, double-buffered, vsync-paced.\n",
                config::PANEL_WIDTH, config::PANEL_HEIGHT);

  // Clear both framebuffers before the backlight comes on.
  g_gfx->fillScreen(RGB565_BLACK);
  g_gfx->flush();
  g_gfx->fillScreen(RGB565_BLACK);
  g_gfx->flush();

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
