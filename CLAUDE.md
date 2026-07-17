# CLAUDE.md — radar

Guidance for Claude Code working in this repo. Read this first: it is a
handoff from an earlier session that designed the project and wrote the
first firmware, but could not build it (that session's sandbox blocked the
PlatformIO toolchain download). You are continuing on a real machine with
the **hardware attached**, so your job starts with getting it to compile
and light the screen.

## What this project is

A home-centered ADS-B **flight-radar display** for a 4-inch round screen.
It sweeps live aircraft, airports and points of interest around a "home"
location in a phosphor-radar style, overlays rain radar and geography, and
always pins the flights you care about (imported from iCal feeds like
TripIt) with an edge arrow when they are off the map.

- **Hardware:** Adafruit **5793** (4" 720×720 round IPS, ST7701S, TTL
  RGB-666, no touch) driven by an Adafruit **5800 Qualia ESP32-S3**
  (16 MB flash / 8 MB octal PSRAM). Flashed over native USB-C.
- **Full design:** [`DESIGN.md`](DESIGN.md).
- **Interactive concept mockup:** [`mockup/radar.html`](mockup/radar.html)
  — open in a browser to see the intended look and interaction (drag the
  encoder knob, work the toggles, follow a flight). This is the visual
  spec the firmware is chasing.

## Current status (honest)

- Design + mockup: **complete**.
- Firmware (`firmware/`): **first scaffold — never compiled or flashed.**
  Architecture, pin map, render loop, inputs, Wi-Fi/web-config, feed
  polling and OTA are written and reviewable, but the only verification so
  far is `pio project config` (the ini parses). **Treat the first
  `pio run` as the real smoke test — expect compile errors.**

## Repo layout

```
DESIGN.md            Full design: hardware, architecture, data sources,
                     symbology, interaction model, pin map, caveats.
mockup/radar.html    Self-contained interactive concept mockup.
firmware/            PlatformIO + Arduino project (see firmware/README.md).
  platformio.ini     Board = esp32-s3-devkitc-1 (N16R8), OTA notes.
  src/config.h       Pin map, I2C addresses, timing constants.
  src/Model.h        Shared world + UI state.
  src/Display.*      RGB-666 panel bring-up + backlight (PCA9554 @0x3F).
  src/Radar.*        Scope renderer (rings, sweep-refresh blips, overlays,
                     CRT sleep animation).
  src/Inputs.*       Encoder + 3-way/2 toggles + light sensor; the
                     browse-then-commit interaction, hold-to-zoom.
  src/Power.*        Deep-sleep entry, ext0 wake (clean restart).
  src/Net.*          WiFiManager portal, mDNS, async config server,
                     LittleFS persistence, ArduinoOTA.
  src/Feeds.*        airplanes.live ADS-B, iCal special-flight matching,
                     weather stub.
  src/web_page.h     The config page served at `/`.
  src/main.cpp       Setup + dual-core split (core 1 render, core 0
                     inputs + network).
```

## How to proceed (in order)

1. **Get it compiling.**
   ```sh
   cd firmware
   pio run
   ```
   Fix errors. The most likely trouble spots:
   - `Display.cpp` panel constructor / init table (see step 2).
   - Library API mismatches: `Arduino_GFX`, the `ESP32Async` fork of
     ESPAsyncWebServer/AsyncTCP, and ArduinoJson v7 — pin `lib_deps`
     versions in `platformio.ini` to what resolves against the installed
     core if needed.

2. **CRITICAL — fill in the panel init table.** `src/Display.cpp` ships
   with **placeholder RGB timing and a null ST7701 init-operations table**
   (marked `TODO(hardware)`). Copy the real `Arduino_ESP32RGBPanel` porch/
   pulse timings and the ST7701 init array from Adafruit's Qualia S3
   `Arduino_GFX` example for the **4-inch 720×720 round** panel. Without
   this the panel will not light (and may not compile).

3. **Bring up display-first.** Confirm the rings + sweep render before
   wiring anything else — that validates the riskiest part (the panel).

4. **Flash and watch.**
   ```sh
   pio run -t upload         # over USB-C (native USB / USB-CDC)
   pio device monitor        # 115200
   ```

5. **First boot / Wi-Fi.** No Wi-Fi yet → the board raises a `Radar-Setup`
   SoftAP → join it → captive portal → enter your Wi-Fi → it reboots onto
   the LAN at `http://radar-720.local/` for configuration (homes, POIs,
   iCal feeds, range).

6. **Then the feeds.** Traffic first (airplanes.live), then iCal specials.
   Weather (RainViewer tiles) and the Natural Earth geography basemap are
   currently **sample stubs** in `Radar.cpp`/`Feeds.cpp` — flesh them out
   after the core scope works.

## Verification checklist

`firmware/README.md` has the full list. The board-specific items to confirm
first: panel init table (above), library versions, encoder counts-per-
detent, toggle wiring/levels, TLS certs (`Feeds.cpp` currently calls
`setInsecure()` — install a CA bundle), the iCal parser (a heuristic), and
the render-loop mutex hold (double-buffer if encoder latency suffers).

## Controls (recap; native pins, no expander board)

- **Encoder** (I²C seesaw @0x36): turn = browse targets/flights, **hold +
  turn** = zoom, **press** = follow selection / switch / go home, **press
  at home** with nothing selected = step between saved homes.
- **Display 3-way** (on-off-on) on `GPIO16 + GPIO43`: flights / flights+
  weather / weather.
- **Geography** on `GPIO44`: borders + cities overlay.
- **Sleep** on `GPIO17` (RTC-capable → `ext0` wake): deep sleep, screen
  off; waking is a **clean restart** (no RAM kept). CRT collapse/bloom
  animation on the transitions.
- **VEML7700** light sensor (I²C @0x10): auto-dims the rendered image (the
  backlight is on/off only via the PCA9554).

Full pin map: `firmware/src/config.h` and `DESIGN.md`.

## Updates (OTA)

After the first USB flash, update over Wi-Fi with ArduinoOTA:
```sh
pio run -t upload   --upload-port radar-720.local   # firmware
pio run -t uploadfs --upload-port radar-720.local   # LittleFS assets
```
Authenticated by an MD5 password hash in `config::OTA_PASSWORD_HASH`
(currently the hash of `radar-ota` — **change it**; pass the plaintext via
`--auth=`). Sleep is deferred while an update is in flight.

## Conventions

- The firmware is a **standalone PlatformIO project** — no monorepo/Bazel
  build involved.
- Comment style carried from the existing code: capitalized sentences that
  end in a period; wrap comments near 72 columns, code near 80.
- Commit in logical units; keep `firmware/.pio/` out of git (already in
  `.gitignore`).

## Provenance

Designed and scaffolded in a prior Claude Code session, then moved here
into `rjhuijsman/radar` (it was originally committed to the wrong repo).
The three original commits were design+mockup, firmware scaffold, and OTA;
they may have been squashed into the import commit when this repo was
seeded. Nothing here has been run on hardware yet — that is your task.
