#include "Power.h"

#include <esp_sleep.h>

#include "Display.h"
#include "config.h"

namespace power {

void deepSleep() {
  // I2C is dead in deep sleep, so blank the backlight now, while the
  // expander is still reachable.
  display::setBacklight(false);

  // The Sleep toggle reads LOW while closed (asleep); wake when it opens
  // and the pin rises to its pulled-up HIGH.
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(config::PIN_SLEEP), 1);
  esp_deep_sleep_start();

  // Unreachable: ext0 wake resets the chip into `setup()`.
  for (;;) {
  }
}

}  // namespace power
