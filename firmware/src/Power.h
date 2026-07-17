// Deep-sleep entry. Waking is a fresh boot (reset vector, no RAM kept), so
// there is no matching "wake" call here — the device simply starts over.

#pragma once

namespace power {

// Blanks the backlight and enters deep sleep, arming ext0 wake on the Sleep
// toggle pin. Does not return: the chip wakes into a clean boot. Call only
// after the power-down animation has finished.
[[noreturn]] void deepSleep();

}  // namespace power
