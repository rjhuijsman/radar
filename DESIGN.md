# Round ADS-B Radar Display — Design

A home-centered flight-radar display on a 4-inch round screen: it sweeps
live ADS-B traffic, airports and points of interest around a "home"
location, overlays rain radar and geography, and always pins the flights
you care about — even when they are off the edge of the map.

An interactive visual mockup of the whole concept lives in
[`mockup/radar.html`](mockup/radar.html) (open it in a browser). This
document is the written design behind it; the firmware that implements it
lives in [`firmware/`](firmware/).

## Hardware

| Part | Product | Role |
| --- | --- | --- |
| Display | Adafruit **5793** | 4" round IPS TFT, **720×720**, ST7701S, TTL RGB-666, no touch. |
| Driver | Adafruit **5800** | **Qualia ESP32-S3** — octal PSRAM, Wi-Fi, drives the RGB-666 panel and breaks out the STEMMA QT I²C bus. |

The two are a designed-to-pair combo. The ESP32-S3 module is **16 MB
flash / 8 MB PSRAM**. A 16-bit framebuffer is `720×720×2 ≈ 1.01 MB`, so
even double-buffered it fits comfortably in PSRAM.

The panel has **no touch** — it is a pure display. All interaction is
either physical (one rotary encoder plus three toggle switches) or through
the configuration web page.

## Architecture

The device is fully self-contained: it runs **no server of its own** on the
internet. It wears two hats at once.

- **Inbound — it *is* a web server.** On first boot with no known Wi-Fi it
  raises a SoftAP captive portal (`Radar-Setup`) to collect credentials.
  Once on the network it serves the configuration site directly (over
  `radar-720.local` via mDNS). Your browser talks straight to the device.
- **Outbound — it is also a web client.** It reaches out to existing public
  services for data: ADS-B traffic, rain radar, and your TripIt/calendar
  iCal feeds. Those already exist; the device only polls them.

The ESP32-S3 is dual-core, split by concern:

- **Core 1 — display.** Owns the panel and the render loop (rings, sweep,
  blips, overlays). A full-frame redraw is heavy, so the sweep and blips are
  redrawn each frame while static layers are cached.
- **Core 0 — networking & housekeeping.** Wi-Fi, the config web server,
  feed polling, and reading the encoder/toggles/light sensor over I²C.

## Data sources

| Layer | Source | Notes |
| --- | --- | --- |
| Live traffic | **airplanes.live** / **adsb.fi** (point+radius JSON), OpenSky as fallback | Poll a box around home every 5–15 s. Each state carries position, altitude, ground speed, track and callsign. |
| Rain radar | **RainViewer** (free personal use) / **EUMETNET OPERA** via **LibreWXR** | XYZ Web-Mercator PNG tiles refreshed ~every 5 min; strong Europe coverage. |
| Geography | **Natural Earth** (public domain) | Coastlines, borders, populated places. Bundled as packed polylines; a Europe subset fits flash. |
| Special flights | **iCal feeds** (TripIt private feed, Concur, iCloud, …) | A list of feeds configured in the web UI. See below. |

### Special flights via iCal (not the TripIt API)

The TripIt API authenticates with **OAuth 1.0a** (3-legged, HMAC-SHA1
signed) — painful on a microcontroller. Instead we use TripIt's (and any
calendar's) **private iCal feed URL**, where the unguessable URL token *is*
the credential. The device does a plain HTTPS `GET` and parses the flight
segments out of the `VEVENT`s.

The web UI manages a **list** of feeds (name, URL, color, enabled). Flights
matched from any enabled feed become "special": amber, always labeled, and
shown with an edge arrow even when off-map.

**Callsign matching.** iCal gives an IATA flight number (`BA117`); ADS-B
broadcasts an ICAO callsign (`BAW117`). Promoting a blip to special needs a
small IATA→ICAO airline-code table on the device.

## Display & symbology

A north-up plan-position indicator (PPI), centered on home. A key design
choice: **returns only refresh as the sweep passes them**, so the visible
step rate of each blip *is* the feed's poll cadence — the polling made
visible, like a real radar scope.

| Mark | Meaning |
| --- | --- |
| Green triangle + stalk | Traffic; points along track, stalk is a 1-minute leading line scaled to ground speed. |
| Amber ringed triangle | Special flight (from an iCal feed or flagged by hand); always labeled. |
| Amber rim chevron + distance | A special or selected flight beyond range, clamped to the rim at its true bearing. |
| Cyan ring + strips | Airport; runway strips drawn to their stored headings. |
| Cyan diamond | Point of interest (tower, overlook, landmark). |
| Center crosshair | Home (one of several saved homes). |
| Rings | Range, labeled in nautical miles. |

Overlays: a dim **rain** underlay beneath the sweep, and a **geography**
layer (coastlines, borders, city dots). Both are independent display
layers. Orientation is **north-up in all modes**, including follow.

## Interaction model

One rotary encoder (with push) plus three toggles. The scheme is
**browse-then-commit**.

### Encoder

- **Turn** — browse. At home it moves a selection reticle through targets;
  while following it scrubs a candidate list of flights that also includes a
  **Home** entry. Turning never moves the view — it is a preview.
- **Hold + turn** — zoom range in/out.
- **Press** — commit:
  - At home with a target selected → **follow it** (the map slides over).
  - Following, with a browsed candidate → **switch** to that flight, or to
    **Home** → go home.
  - Following with nothing browsed → **go home**.
  - At home with nothing selected → **step between saved homes**.

Entering follow and switching targets **animate** the view center with a
critically-damped ease, so it slides rather than jumps, then locks onto the
followed plane's live position.

### Toggles

- **Display (3-way, on-off-on):** flights-only / flights+weather / weather-only.
- **Geography (on/off):** borders + cities overlay.
- **Sleep (on/off):** deep sleep — screen fully off.

An I²C **ambient-light sensor** auto-dims the rendered image (the backlight
is on/off only, so dimming is applied in software to the drawn colors).

### Sleep = clean restart

Sleep deep-sleeps the S3 with `ext0` wake on the Sleep pin. Deep-sleep wake
is a reset-vector boot, so **no RAM is kept and it comes up as a fresh
boot** — exactly the requested behavior. The physical toggle positions
(Display, Geography) survive because firmware re-reads those pins on boot.

Because I²C is off during deep sleep, the backlight is blanked (via the
PCA9554 expander) *before* sleeping. Power-down plays a CRT-style collapse
(image squeezes to a line, then a dot, then black); power-up blooms back
open and the first sweep paints the returns in.

## Pins & wiring — native, no expander board

The RGB-666 panel consumes ~20 GPIO, so free pins are scarce; the display's
own init-SPI, backlight and two onboard buttons are offloaded to a
**PCA9554 I²C expander at 0x3F**. The remaining broken-out pins are enough
for our controls without any extra board:

| Signal | Pin(s) | Note |
| --- | --- | --- |
| Sleep toggle | **GPIO17 (A0)** | RTC-capable → `ext0` deep-sleep wake. |
| Display 3-way | **GPIO16 (A1) + GPIO43 (TX)** | on-off-on = two switch contacts. |
| Geography toggle | **GPIO44 (RX)** | non-wake input. |
| microSD (optional) | **GPIO15 (CS) + SPI** | preserved for basemap growth. |
| Encoder + light sensor | **STEMMA QT I²C** | seesaw encoder @0x36, VEML7700 @0x10. |

Notes:

- Only RTC GPIOs (0–21) can wake from deep sleep, so **Sleep must be a
  native RTC pin** (16/17) — an I²C expander pin cannot wake the chip.
- Giving up the UART (GPIO43/44) costs nothing: the S3 uses **native USB**
  (USB-Serial-JTAG) for flashing and console.
- Toggles are switch-to-GND with `INPUT_PULLUP` (`0` = closed/on).
- I²C addresses: VEML7700 `0x10`, seesaw encoder `0x36`, onboard PCA9554
  `0x3F` (avoid) — leaving room for a PCF8574 at `0x20` if more inputs are
  ever needed.

## Configuration web UI

Served by the device itself.

- **Wi-Fi onboarding:** captive portal on first run.
- **Homes & range:** several saved home locations (lat/lng) and a default
  range; the encoder steps between homes.
- **Points of interest:** cyan glyphs; airports store runway heading(s).
- **Calendar feeds (iCal):** the managed list of feeds (name, URL, color,
  enabled) that produce special flights.
- **Sources:** traffic feed and rain-radar provider.

Settings, homes, POIs and the feed list persist to flash (NVS / LittleFS).

## Storage & secrets

- Config, homes, POIs, feeds: JSON in NVS/LittleFS.
- Natural Earth basemap (Europe subset): packed polylines in a LittleFS
  partition; the microSD header is kept free for a full-world basemap.
- Wi-Fi password and private feed URLs sit in flash — fine for a home
  device, worth noting before sharing one.

## Updates (OTA)

The first flash is over USB-C; every update after that goes over Wi-Fi. The
partition table already carries two app slots (`app0`/`app1`) plus
`otadata`, so OTA and rollback are provisioned.

- **ArduinoOTA** is armed once Wi-Fi is up, pumped from the network task.
  Push firmware or the LittleFS assets from the dev machine on the same LAN:

  ```sh
  pio run -t upload   --upload-port radar-720.local   # firmware
  pio run -t uploadfs --upload-port radar-720.local   # basemap / web assets
  ```

- **Authenticated.** The push is verified against an MD5 password hash
  (`config::OTA_PASSWORD_HASH`, set with `setPasswordHash`), so the password
  is never stored in plaintext and randoms on the LAN cannot push. Note this
  authenticates the pusher; it does not encrypt the image. For a stronger
  guarantee, enable **signed images (secure boot)** so the bootloader rejects
  unsigned firmware.

- **Sleep is deferred during an update.** An in-progress OTA sets a flag that
  blocks the Sleep toggle, so an update cannot be interrupted mid-flash.

- **Not shipped: ElegantOTA / browser upload.** For a single unit that is
  always on your LAN and updated from PlatformIO, ArduinoOTA push is the
  simpler loop. Add a password-protected ElegantOTA `/update` page later only
  if you want to flash from a machine without the toolchain, or hand a unit to
  someone else. A pull-from-URL scheme (`HTTPUpdate` against a version
  manifest) is the option for updating from outside the LAN.

## Open decisions & caveats

1. **Feed choice & limits.** OpenSky now wants OAuth2 client credentials for
   its free allowance; airplanes.live / adsb.fi are simpler. Poll and cache.
2. **Callsign matching.** Needs the IATA→ICAO airline map (see above).
3. **Weather reprojection.** RainViewer tiles are Web-Mercator; the scope is
   azimuthal around home, so tiles are resampled per pixel. Cheap at ≤100 NM
   and rain only refreshes ~every 5 min.
4. **Basemap storage.** Europe fits flash; the whole world at detail wants
   the microSD on the broken-out SPI header.
5. **Sleep = clean restart.** RTC-wake pin, backlight blanked before sleep.
6. **Secrets at rest** (above).

## Status

- Concept mockup: complete (`mockup/radar.html`).
- Firmware: **first scaffold** (`firmware/`) — architecture, pin map,
  display bring-up, render loop, inputs, Wi-Fi/web-config, and feed polling
  are in place; not yet flashed to hardware. See
  [`firmware/README.md`](firmware/README.md) for what still needs verifying.
