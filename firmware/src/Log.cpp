#include "Log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace rlog {
namespace {

// A few KB of the most recent log text, as a byte ring. Small enough to sit
// in internal RAM; big enough to hold the last dozens of diagnostic lines.
constexpr size_t kCap = 8192;
char g_buf[kCap];
size_t g_head = 0;    // Next write position.
bool g_wrapped = false;

// The tee is written from the network and input tasks (core 0) and read by
// the web handler; a short critical section keeps the byte copy atomic across
// cores without allocating (so it is safe to hold).
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

Sink Log;

size_t Sink::write(uint8_t c) { return write(&c, 1); }

size_t Sink::write(const uint8_t* buf, size_t len) {
  Serial.write(buf, len);  // Mirror to the USB console as before.
  portENTER_CRITICAL(&g_mux);
  for (size_t i = 0; i < len; ++i) {
    g_buf[g_head++] = static_cast<char>(buf[i]);
    if (g_head >= kCap) {
      g_head = 0;
      g_wrapped = true;
    }
  }
  portEXIT_CRITICAL(&g_mux);
  return len;
}

void Sink::dump(String& out) {
  // Snapshot the ring bounds under the lock, then read the bytes without it:
  // a concurrent write can at worst garble a byte or two of old text, never
  // crash, and String growth must not run inside the critical section.
  portENTER_CRITICAL(&g_mux);
  size_t head = g_head;
  bool wrapped = g_wrapped;
  portEXIT_CRITICAL(&g_mux);
  out.reserve(out.length() + (wrapped ? kCap : head));
  if (wrapped) {
    for (size_t i = head; i < kCap; ++i) out += g_buf[i];
  }
  for (size_t i = 0; i < head; ++i) out += g_buf[i];
}

}  // namespace rlog
