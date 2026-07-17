# flight-radar firmware

First-cut firmware for the round ADS-B radar: Adafruit **5793** panel on an
Adafruit **5800 Qualia ESP32-S3**. Built with PlatformIO + Arduino.

> **Status: not yet flashed to hardware.** The architecture, pin map, render
> loop, inputs, Wi-Fi/web-config and feed polling are in place, but this has
> only been written — not compiled against the real toolchain or run on the
> board. See the verification checklist below before trusting it.

## Layout

| File | Role |
| --- | --- |
| `src/config.h` | Pin map, I²C addresses, timing constants. |
| `src/Model.h` | Shared world + UI state structs. |
| `src/Display.*` | RGB-666 panel bring-up and backlight (via the PCA9554). |
| `src/Radar.*` | The scope renderer (rings, sweep, blips, overlays, CRT animation). |
| `src/Inputs.*` | Encoder, 3-way + 2 toggles, light sensor; browse-then-commit logic. |
| `src/Power.*` | Deep-sleep entry with `ext0` wake. |
| `src/Net.*` | Wi-Fi provisioning, mDNS, config web server, persistence. |
| `src/Feeds.*` | ADS-B polling, iCal special-flight matching, weather (stub). |
| `src/web_page.h` | The configuration page served at `/`. |
| `src/main.cpp` | Setup, the two core-0 tasks, and the core-1 render loop. |

## Build & flash

```sh
# From this directory, with PlatformIO installed:
pio run                 # compile
pio run -t upload       # flash over USB-C (native USB / USB-CDC)
pio device monitor      # serial console over the same USB
```

On first boot the device has no Wi-Fi, so it raises a `Radar-Setup` SoftAP.
Join it, pick your network, and it reboots onto your LAN, reachable at
`http://radar-720.local/` for configuration.

### Updating over Wi-Fi (OTA)

The first flash must be over USB; after that, update over Wi-Fi with
ArduinoOTA (authenticated by the hashed password in `config.h`):

```sh
pio run -t upload   --upload-port radar-720.local   # firmware
pio run -t uploadfs --upload-port radar-720.local   # LittleFS assets
```

Set your own password: put its MD5 in `config::OTA_PASSWORD_HASH`
(`printf '%s' 'your-password' | md5sum`) and pass the plaintext to the
uploader via `--auth=` (see the commented `upload_flags` in
`platformio.ini`). Sleep is deferred while an update is in progress.

## Controls (recap)

- **Encoder** — turn to browse targets/flights, **hold + turn** to zoom,
  **press** to follow the selection / switch / go home, **press at home**
  with nothing selected to step between saved homes.
- **Display 3-way** — flights / flights+weather / weather.
- **Geography** — borders + cities overlay.
- **Sleep** — deep sleep (screen off); waking is a clean restart.

See [`../DESIGN.md`](../DESIGN.md) for the full behavior and pin map.

## Verification checklist (before this is real)

These are the parts that need a board and were written from documentation,
so confirm them first:

1. **Panel init table & timing** — `Display.cpp` uses placeholder porch/
   pulse values and a null ST7701 init-operations table. Copy the real ones
   from Adafruit's Qualia S3 `Arduino_GFX` example for the 4"/720 round panel.
2. **Library versions** — the `lib_deps` in `platformio.ini` are a known-good
   starting point; pin them to what resolves against your installed Arduino
   core (the ESPAsyncWebServer fork in particular).
3. **Encoder counts per detent** — `Inputs.cpp` treats each encoder count as
   one step; divide if your seesaw encoder reports multiple counts per detent.
4. **Toggle wiring & levels** — switch-to-GND with `INPUT_PULLUP`; confirm the
   Sleep wake level (`ext0` on GPIO17, wake on HIGH) matches your switch.
5. **TLS certificates** — `Feeds.cpp` currently calls `setInsecure()`. Install
   a CA bundle and validate before shipping.
6. **iCal parsing** — the special-flight extractor is a heuristic. Proper iCal
   line-unfolding, `VEVENT` boundaries, and today-filtering are TODO.
7. **Weather & basemap** — RainViewer tile fetch/reproject and the Natural
   Earth basemap load from LittleFS are stubbed with sample data.
8. **Render locking** — the render loop holds the model mutex for the whole
   frame; if that hurts encoder latency, double-buffer the model.
