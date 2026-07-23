#include "Display.h"

#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>

#include "config.h"

// Startup-latency breadcrumbs: timestamped serial lines for pinning down
// where boot time goes (the wake-to-first-picture investigation). Flip to 0
// once the timings are confirmed good on hardware.
#define BOOT_TRACE 0
#if BOOT_TRACE
#define BOOT_LOG(...) Serial.printf(__VA_ARGS__)
#else
#define BOOT_LOG(...)
#endif

namespace display {
namespace {

// The expander doubles as the bit-banged SPI used to configure the panel
// controller and as the backlight control line.
Arduino_XCA9554SWSPI* g_expander = nullptr;

// The ESP32-S3 RGB LCD peripheral, driven directly so we can run with two
// PSRAM framebuffers the renderer flips between (see Display.h).
esp_lcd_panel_handle_t g_panel = nullptr;

// The render task (core-1 loop) that LiveGfx::flush() blocks on, notified
// from the panel's frame-complete ISR so the loop paces to the refresh.
volatile TaskHandle_t g_renderTask = nullptr;

// The live surface (retargeted at the current back framebuffer) and the
// off-screen static-reference surface, plus the raw framebuffers.
LiveGfx* g_live = nullptr;
StaticGfx* g_static = nullptr;
uint16_t* g_fbs[2] = {nullptr, nullptr};  // The two scan framebuffers.
int g_front = 0;                          // Index being scanned out.
uint16_t* g_staticFb = nullptr;

// Boot-trace checkpoints recorded during begin(). They are also printed
// immediately, but the USB console usually attaches after boot (the CDC port
// re-enumerates on reset), so printBootTrace() replays them later.
struct BootMark {
  const char* what;
  uint32_t t;
};
BootMark g_bootMarks[8];
int g_bootMarkCount = 0;

void mark(const char* what) {
  if (g_bootMarkCount < 8) {
    g_bootMarks[g_bootMarkCount++] = BootMark{what, millis()};
  }
  BOOT_LOG("[boot] t=%6lu display: %s\n", millis(), what);
}

// Fires from the RGB driver ISR each time a whole framebuffer has been
// handed to the LCD — the same boundary where the driver latches a pending
// flip. Waking the render task here paces one rendered frame per panel
// refresh (~24.6 Hz) with no busy-wait, and guarantees that when flush()
// returns after a flip, the new back buffer is fully out of scan.
bool IRAM_ATTR onFrameDone(esp_lcd_panel_handle_t,
                           const esp_lcd_rgb_panel_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  if (g_renderTask) vTaskNotifyGiveFromISR(g_renderTask, &woken);
  return woken == pdTRUE;
}

}  // namespace

Arduino_GFX* begin() {
  g_renderTask = xTaskGetCurrentTaskHandle();  // The core-1 render loop.

  Wire.begin(config::I2C_SDA, config::I2C_SCL);
  // Run the bus at Fast-mode for bring-up. Every bit of the bit-banged panel
  // init below costs a whole I2C write to the expander, so the bus clock is
  // the panel init's clock: 100 kHz put multiple seconds between wake and
  // first picture. The PCA9554 is a 400 kHz part. (Modulino.begin() later
  // drops the bus back to 100 kHz for steady-state input polling.)
  Wire.setClock(400000);

  mark("begin start");
  // Pass no reset pin (GFX_NOT_DEFINED): the library's begin() would pulse the
  // panel reset while every expander pin — the backlight among them — is still
  // floating high, so its 10+100 ms reset delay shows as a lit blank screen
  // for ~110 ms on every power-up and wake. We drive the backlight off first
  // and pulse EXP_TFT_RESET ourselves, below, in the dark.
  g_expander = new Arduino_XCA9554SWSPI(
      GFX_NOT_DEFINED, config::EXP_TFT_CS, config::EXP_TFT_SCK,
      config::EXP_TFT_MOSI, &Wire, config::EXPANDER_ADDR);
  g_expander->begin();

  // The expander's begin() releases every pin to input, which lets the
  // backlight line float on with the panel still unconfigured — the lit
  // blank/garbage screen seen on wake. Drive it low again immediately;
  // main.cpp lights it only after the first frame has been presented.
  setBacklight(false);

  // Now pulse the panel's hardware reset ourselves — in the dark, backlight
  // held off — the same 10/100 ms pulse begin() would have done, minus the
  // blink. EXP_TFT_RESET is still an input after begin(); drive it.
  g_expander->pinMode(config::EXP_TFT_RESET, OUTPUT);
  g_expander->digitalWrite(config::EXP_TFT_RESET, LOW);
  delay(10);
  g_expander->digitalWrite(config::EXP_TFT_RESET, HIGH);
  delay(100);
  mark("expander up, backlight off, panel reset");

  // Configure the HD40015C40 controller over the expander's bit-banged SPI,
  // exactly as Arduino_RGB_Display would when no reset pin is wired: software
  // reset then the panel's init-operations table (product 5793 / HD40015C40 /
  // NV3052C — not ST7701). `hd40015c40_init_operations` ships in Arduino_GFX.
  g_expander->sendCommand(0x01);  // Software reset.
  delay(120);
  mark("panel-controller init start (bit-banged SPI)");
  g_expander->batchOperation((uint8_t*)hd40015c40_init_operations,
                             sizeof(hd40015c40_init_operations));
  mark("panel-controller init done");

  // TWO framebuffers in PSRAM plus a bounce buffer so the LCD FIFO refills
  // from internal SRAM and cannot underrun when PSRAM is busy. The CPU
  // refills the bounce buffer from the PSRAM framebuffer, so the CPU writes
  // we make into these framebuffers stay cache-coherent with scan-out, and
  // the driver latches a flip only when it starts a new frame — tear-free.
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
                      .pclk_active_neg = 1,
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
              .double_fb = 0,
              .no_fb = 0,
              .bb_invalidate_cache = 0,
          },
  };

  if (esp_lcd_new_rgb_panel(&cfg, &g_panel) != ESP_OK) {
    Serial.println("[display] esp_lcd_new_rgb_panel failed.");
    return nullptr;
  }

  // Register the frame-complete callback BEFORE init (per the IDF driver),
  // so flush() can pace the render loop to the panel refresh.
  esp_lcd_rgb_panel_event_callbacks_t cbs = {};
  cbs.on_frame_buf_complete = onFrameDone;
  esp_lcd_rgb_panel_register_event_callbacks(g_panel, &cbs, nullptr);

  esp_lcd_panel_reset(g_panel);
  esp_lcd_panel_init(g_panel);

  void* fb0 = nullptr;
  void* fb1 = nullptr;
  if (esp_lcd_rgb_panel_get_frame_buffer(g_panel, 2, &fb0, &fb1) != ESP_OK) {
    Serial.println("[display] get_frame_buffer(2) failed.");
    return nullptr;
  }
  g_fbs[0] = static_cast<uint16_t*>(fb0);
  g_fbs[1] = static_cast<uint16_t*>(fb1);
  g_front = 0;  // The driver scans fb 0 first.

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

  g_live = new LiveGfx(config::PANEL_WIDTH, config::PANEL_HEIGHT, g_fbs[0]);
  g_static = new StaticGfx(config::PANEL_WIDTH, config::PANEL_HEIGHT, g_staticFb);
  Serial.printf(
      "[display] panel up: %dx%d, double-fb, dirty-rect, frame-paced.\n",
      config::PANEL_WIDTH, config::PANEL_HEIGHT);

  // Clear both scan buffers while the backlight is still dark. The backlight
  // stays off until main.cpp has presented a first finished frame, so
  // bring-up and first-paint are never visible.
  memset(g_fbs[0], 0, fbSize);
  memset(g_fbs[1], 0, fbSize);
  mark("RGB peripheral up, buffers cleared");
  return g_live;
}

void printBootTrace() {
  for (int i = 0; i < g_bootMarkCount; ++i) {
    Serial.printf("[boot] t=%6lu display: %s\n", g_bootMarks[i].t,
                  g_bootMarks[i].what);
  }
}

LiveGfx* live() { return g_live; }
Arduino_GFX* staticRef() { return g_static; }
uint16_t* staticFb() { return g_staticFb; }
int16_t width() { return config::PANEL_WIDTH; }
int16_t height() { return config::PANEL_HEIGHT; }

uint16_t* frontFb() { return g_fbs[g_front]; }
uint16_t* backFb() { return g_fbs[1 - g_front]; }
int backIndex() { return 1 - g_front; }

void flip() {
  if (g_panel == nullptr) return;
  // Passing a driver-owned framebuffer makes draw_bitmap a no-copy scan-out
  // switch, latched when the driver next starts a frame. Clear any pending
  // frame notification AFTER requesting the flip, so the flush() that
  // follows wakes on a boundary that has definitely latched it — never on
  // a boundary that already passed while this frame was being drawn.
  g_front = 1 - g_front;
  esp_err_t err = esp_lcd_panel_draw_bitmap(g_panel, 0, 0, config::PANEL_WIDTH,
                                            config::PANEL_HEIGHT,
                                            g_fbs[g_front]);
  static bool logged = false;
  if (err != ESP_OK && !logged) {
    logged = true;
    Serial.printf("[display] flip failed: %d\n", err);
  }
  TaskHandle_t task = g_renderTask;
  if (task) ulTaskNotifyValueClear(task, UINT32_MAX);
}

void setBacklight(bool on) {
  if (g_expander == nullptr) {
    return;
  }
  g_expander->pinMode(config::EXP_BACKLIGHT, OUTPUT);
  g_expander->digitalWrite(config::EXP_BACKLIGHT, on ? HIGH : LOW);
}

}  // namespace display
