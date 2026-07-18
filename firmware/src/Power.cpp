#include "Power.h"

#include <driver/rtc_io.h>
#include <esp_sleep.h>

#include "Display.h"
#include "config.h"

namespace power {

void deepSleep() {
  // I2C is dead in deep sleep, so blank the backlight now, while the
  // expander is still reachable.
  display::setBacklight(false);

  // The Sleep switch is inverted: OPEN (HIGH) is OFF, so we sleep with the pin
  // held high; wake when it CLOSES and the pin falls LOW. The IO-MUX pull-up
  // from pinMode() is powered down in deep sleep, so drive the pull-up from the
  // always-on RTC domain — otherwise the open switch floats instead of reading
  // its non-wake HIGH (which is why deep sleep worked but waking did not).
  gpio_num_t wake = static_cast<gpio_num_t>(config::PIN_SLEEP);
  rtc_gpio_pullup_en(wake);
  rtc_gpio_pulldown_dis(wake);
  esp_sleep_enable_ext0_wakeup(wake, 0);
  esp_deep_sleep_start();

  // Unreachable: ext0 wake resets the chip into `setup()`.
  for (;;) {
  }
}

}  // namespace power
