# flight-radar

A home-centered ADS-B flight-radar display for a 4-inch round screen —
Adafruit **5793** panel driven by an Adafruit **5800 Qualia ESP32-S3**.

It sweeps live ADS-B traffic, airports and points of interest around a
"home" location in a radar style, overlays rain radar and geography, and
always pins the flights you care about (imported from iCal feeds such as
TripIt) with a noticeable edge arrow when they are off the map.

## Contents

- **[`DESIGN.md`](DESIGN.md)** — the full design: hardware, architecture,
  data sources, symbology, the encoder + 3-toggle interaction model, the
  pin map, and open questions.
- **[`mockup/radar.html`](mockup/radar.html)** — an interactive visual
  mockup of the whole concept. Open it in a browser: drag the encoder knob,
  work the toggles, follow a flight. Self-contained, no build step.
- **[`firmware/`](firmware/)** — the PlatformIO firmware (first scaffold).
  See [`firmware/README.md`](firmware/README.md) to build and flash.

## Status

Concept and mockup are complete. The firmware is a first scaffold — the
architecture is in place but it has not yet been flashed to hardware; see
the firmware README for the verification checklist.
