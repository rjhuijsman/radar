// The configuration page served at `/`. A compact functional form that
// reads `/api/state` and writes `/api/config`; the rich visual design lives
// in the project mockup. Stored in flash as one string.

#pragma once

#include <Arduino.h>

const char CONFIG_PAGE[] PROGMEM = R"HTML(<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Radar 720 · Config</title>
<style>
  body { font: 15px system-ui, sans-serif; margin: 0; background: #0c1a16; color: #d9e8e0; }
  main { max-width: 620px; margin: 0 auto; padding: 20px; }
  h1 { font-size: 19px; } h2 { font-size: 14px; letter-spacing: .1em; text-transform: uppercase; color: #3dffa4; margin-top: 26px; }
  .row { display: flex; gap: 8px; margin: 6px 0; }
  input { flex: 1; padding: 8px; border: 1px solid #1b2c26; border-radius: 8px; background: #071310; color: #d9e8e0; }
  button { padding: 8px 12px; border: 0; border-radius: 8px; background: #0a8f56; color: #fff; cursor: pointer; }
  .add { background: #10221c; color: #8ba396; }
  .x { background: transparent; color: #8ba396; }
  #save { width: 100%; margin-top: 22px; padding: 12px; font-size: 16px; }
</style>
<main>
  <h1>🛩 Radar 720</h1>
  <h2>Default range (NM)</h2>
  <div class="row"><input id="range" type="number" min="5" max="240"></div>
  <h2>Homes</h2><div id="homes"></div><button class="add" onclick="add('homes')">+ Home</button>
  <h2>Points of interest</h2><div id="pois"></div><button class="add" onclick="add('pois')">+ POI</button>
  <h2>Calendar feeds (iCal)</h2><div id="feeds"></div><button class="add" onclick="add('feeds')">+ Feed</button>
  <h2>Special flights</h2><div id="specials"></div><button class="add" onclick="add('specials')">+ Flight</button>
  <h2>Wi-Fi networks</h2><div id="wifi"></div><button class="add" onclick="add('wifi')">+ Network</button>
  <button id="save" onclick="save()">Save &amp; apply</button>
</main>
<script>
let data = { range: 40, homes: [], pois: [], feeds: [], specials: [], wifi: [] };
const F = {
  homes: [["name", "Name"], ["lat", "Lat"], ["lon", "Lon"]],
  pois: [["name", "Name"], ["lat", "Lat"], ["lon", "Lon"]],
  feeds: [["name", "Name"], ["url", "iCal URL"]],
  specials: [["flight", "Flight (e.g. BA117)"], ["date", "Date"]],
  wifi: [["ssid", "SSID"], ["password", "Password"]],
};
function render() {
  document.getElementById("range").value = data.range;
  for (const key of ["homes", "pois", "feeds", "specials", "wifi"]) {
    const host = document.getElementById(key);
    host.innerHTML = "";
    data[key].forEach((item, i) => {
      const row = document.createElement("div");
      row.className = "row";
      F[key].forEach(([f, ph]) => {
        const input = document.createElement("input");
        input.placeholder = ph;
        input.value = item[f] ?? "";
        // Saved Wi-Fi passwords never come back from the API; leaving
        // the field blank on save keeps the one stored on the device.
        if (f === "password") {
          input.type = "password";
          if (item.hasPassword) input.placeholder = "(saved - blank keeps it)";
        }
        if (f === "date") input.type = "date";
        input.oninput = () => { item[f] = input.value; };
        row.appendChild(input);
      });
      const del = document.createElement("button");
      del.className = "x"; del.textContent = "✕";
      del.onclick = () => { data[key].splice(i, 1); render(); };
      row.appendChild(del);
      host.appendChild(row);
    });
  }
}
function add(key) { data[key].push({ enabled: true }); render(); }
async function load() {
  data = await (await fetch("/api/state")).json();
  for (const key of ["homes", "pois", "feeds", "specials", "wifi"]) data[key] ??= [];
  render();
}
async function save() {
  data.range = Number(document.getElementById("range").value);
  await fetch("/api/config", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(data) });
  document.getElementById("save").textContent = "Saved";
  setTimeout(() => document.getElementById("save").textContent = "Save & apply", 1500);
}
load();
</script>
)HTML";
