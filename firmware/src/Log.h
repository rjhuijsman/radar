// A tiny logging tee: everything written to `rlog::Log` goes to the USB
// serial console as usual AND into a small in-RAM ring buffer, so the last
// few KB of diagnostics can be read back over Wi-Fi at `/api/log` — handy
// when the board is mounted and only reachable on the network. Use it exactly
// like `Serial` for prints (rlog::Log.printf(...), rlog::Log.println(...)).

#pragma once

#include <Arduino.h>

namespace rlog {

class Sink : public Print {
 public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t len) override;
  // Appends the buffered text (oldest first) to `out` for the web endpoint.
  void dump(String& out);
};

extern Sink Log;

}  // namespace rlog
