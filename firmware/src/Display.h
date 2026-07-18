// Panel bring-up for the Qualia + 5793 round display, plus backlight
// control (through the PCA9554 expander) used when entering and leaving
// deep sleep.
//
// The panel runs with a SINGLE PSRAM framebuffer (plus the bounce buffer)
// that the RGB peripheral scans continuously. The renderer draws small
// dirty regions into that buffer in place, so there is no per-frame full
// clear and no framebuffer flip. Two surfaces are exposed:
//   * the live surface, an Arduino_Canvas aimed straight at the hardware
//     framebuffer, and
//   * the static-reference surface, an off-screen Arduino_Canvas holding
//     the unchanging scene (background, rings, weather, geography) that the
//     renderer copies from to erase moving elements.

#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

namespace display {

// An Arduino_Canvas that draws into a caller-supplied framebuffer rather than
// allocating its own. Used for the static-reference surface, whose buffer we
// place in PSRAM by hand.
class StaticGfx : public Arduino_Canvas {
 public:
  StaticGfx(int16_t w, int16_t h, uint16_t* fb) : Arduino_Canvas(w, h, nullptr) {
    _framebuffer = fb;
  }
  bool begin(int32_t = GFX_NOT_DEFINED) override { return true; }
  void flush(bool = false) override {}  // Never scanned out; nothing to do.
};

// The live surface: an Arduino_Canvas aimed at the single hardware
// framebuffer the panel scans continuously, so drawing lands on screen in
// place with no flip. Two extras support incremental (dirty-rect) rendering:
//   * beginTrack()/endTrack() report the bounding box of everything drawn
//     between them, so the renderer knows exactly which region to erase next
//     frame, and
//   * flush() blocks until the next VSYNC, pacing the render loop to the
//     panel's refresh instead of spinning.
class LiveGfx : public Arduino_Canvas {
 public:
  LiveGfx(int16_t w, int16_t h, uint16_t* fb) : Arduino_Canvas(w, h, nullptr) {
    _framebuffer = fb;
  }
  bool begin(int32_t = GFX_NOT_DEFINED) override { return true; }

  // Wait for the VSYNC ISR to notify this (the render) task. The 100 ms cap
  // keeps a missed interrupt from stalling the loop forever.
  void flush(bool = false) override {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
  }

  // Begin accumulating the bounding box of subsequent draws.
  void beginTrack() {
    _tx0 = INT16_MAX;
    _ty0 = INT16_MAX;
    _tx1 = INT16_MIN;
    _ty1 = INT16_MIN;
  }
  // Returns the accumulated box, or false when nothing was drawn.
  bool endTrack(int16_t& x, int16_t& y, int16_t& w, int16_t& h) {
    if (_tx1 < _tx0) return false;
    x = _tx0;
    y = _ty0;
    w = _tx1 - _tx0 + 1;
    h = _ty1 - _ty0 + 1;
    return true;
  }

  // The four framebuffer-writing primitives Arduino_Canvas uses at rotation 0;
  // each grows the tracked box, then defers to the base to draw.
  void writePixelPreclipped(int16_t x, int16_t y, uint16_t color) override {
    mark(x, y);
    Arduino_Canvas::writePixelPreclipped(x, y, color);
  }
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    mark(x, y);
    mark(x, y + (h >= 0 ? h - 1 : h + 1));
    Arduino_Canvas::writeFastVLine(x, y, h, color);
  }
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    mark(x, y);
    mark(x + (w >= 0 ? w - 1 : w + 1), y);
    Arduino_Canvas::writeFastHLine(x, y, w, color);
  }
  void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) override {
    mark(x, y);
    mark(x + w - 1, y + h - 1);
    Arduino_Canvas::writeFillRectPreclipped(x, y, w, h, color);
  }

 private:
  int16_t _tx0 = 0, _ty0 = 0, _tx1 = -1, _ty1 = -1;
  inline void mark(int16_t x, int16_t y) {
    if (x < _tx0) _tx0 = x;
    if (x > _tx1) _tx1 = x;
    if (y < _ty0) _ty0 = y;
    if (y > _ty1) _ty1 = y;
  }
};

// Brings up the RGB-666 panel. Returns the live GFX surface everything draws
// onto, or nullptr if initialization failed. Call once from `setup()`. The
// backlight is left OFF: the caller lights it (setBacklight) only after a
// first finished frame has been presented, so bring-up is never visible.
Arduino_GFX* begin();

// Accessors for the two surfaces and their raw framebuffers, used by the
// incremental renderer to copy static regions over moving elements. Valid
// only after a successful begin().
LiveGfx* live();
Arduino_GFX* staticRef();
uint16_t* liveFb();
uint16_t* staticFb();
int16_t width();
int16_t height();

// Turns the panel backlight on or off via the expander. I2C must still be
// alive, so call this before `esp_deep_sleep_start()`, never after.
void setBacklight(bool on);

// Replays the timestamped bring-up checkpoints recorded during begin().
// Boot-time serial output is lost when no USB host is attached yet (the CDC
// port re-enumerates on reset), so main.cpp calls this once, later.
void printBootTrace();

}  // namespace display
