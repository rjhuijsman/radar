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
  .row { display: flex; gap: 8px; margin: 6px 0; align-items: center; }
  input { flex: 1; min-width: 0; padding: 8px; border: 1px solid #1b2c26; border-radius: 8px; background: #071310; color: #d9e8e0; }
  input.bad { border-color: #b5533c; }
  input[readonly] { color: #8ba396; }
  button { padding: 8px 12px; border: 0; border-radius: 8px; background: #0a8f56; color: #fff; cursor: pointer; }
  .add { background: #10221c; color: #8ba396; }
  .x { background: transparent; color: #8ba396; }
  .badge { flex: none; font-size: 12px; white-space: nowrap; cursor: default; }
  .badge.on { color: #3dffa4; }
  .badge.off { color: #8ba396; }
  .tag { flex: none; font-size: 11px; color: #8ba396; border: 1px solid #1b2c26; border-radius: 6px; padding: 2px 6px; white-space: nowrap; cursor: default; }
  .status { font-size: 12px; color: #8ba396; margin: -2px 0 8px 2px; }
  .status.ok { color: #3dffa4; }
  .status.bad { color: #e0836f; }
  .note { font-size: 12px; color: #8ba396; margin: 4px 0 8px; }
  #save { width: 100%; margin-top: 22px; padding: 12px; font-size: 16px; }
</style>
<main>
  <h1>🛩 Radar 720</h1>
  <h2>Default range (NM)</h2>
  <div class="note">Default zoom at startup and on home-to-home switches, unless a home sets its own Zoom (below).</div>
  <div class="row"><input id="range" type="number" min="5" max="240"></div>
  <h2>Homes</h2><div id="homes"></div><button class="add" onclick="add('homes')">+ Home</button>
  <h2>Calendar feeds (iCal)</h2>
  <div class="note" id="feednote"></div>
  <div id="feeds"></div><button class="add" onclick="add('feeds')">+ Feed</button>
  <h2>Special flights</h2>
  <div class="note">Highlighted on the scope on their date. Flights from your
  calendar feeds — today's and upcoming — appear below automatically.</div>
  <div id="specials"></div>
  <div id="icalspecials"></div>
  <button class="add" onclick="add('specials')">+ Flight</button>
  <h2>Points of interest</h2><div id="pois"></div><button class="add" onclick="add('pois')">+ POI</button>
  <h2>Wi-Fi networks</h2><div id="wifi"></div><button class="add" onclick="add('wifi')">+ Network</button>
  <button id="save" onclick="save()">Save &amp; apply</button>
</main>
<script>
let data = { range: 40, homes: [], pois: [], feeds: [], specials: [], icalSpecials: [], wifi: [] };
const F = {
  homes: [["name", "Name"], ["lat", "Lat"], ["lon", "Lon"], ["zoom", "Zoom"]],
  pois: [["name", "Name"], ["lat", "Lat"], ["lon", "Lon"]],
  feeds: [["name", "Name"], ["url", "iCal URL"]],
  specials: [["flight", "Flight (e.g. QR106)"], ["date", "dd-mm-yyyy"]],
  wifi: [["ssid", "SSID"], ["password", "Password"]],
};
// Dates are stored ISO (YYYY-MM-DD) but entered and shown day-first.
function isoToDmy(s) {
  const m = /^(\d{4})-(\d{2})-(\d{2})$/.exec(s || "");
  return m ? `${m[3]}-${m[2]}-${m[1]}` : (s || "");
}
function dmyToIso(s) {
  const m = /^(\d{1,2})[-./](\d{1,2})[-./](\d{4})$/.exec(s.trim());
  if (!m || +m[1] < 1 || +m[1] > 31 || +m[2] < 1 || +m[2] > 12) return null;
  return `${m[3]}-${m[2].padStart(2, "0")}-${m[1].padStart(2, "0")}`;
}
function ago(s) {
  return s < 90 ? "just now" : s < 5400 ? `${Math.round(s / 60)} min ago`
                                        : `${Math.round(s / 3600)} h ago`;
}
// The live "is it on the scope" marker next to each special flight.
function badge(found) {
  const b = document.createElement("span");
  b.className = "badge " + (found ? "on" : "off");
  b.textContent = found ? "● in view" : "○ not in range";
  b.title = "Live snapshot at page load, matched against the aircraft " +
    "currently tracked (~250 NM around home) - the flight may simply " +
    "not be flying here right now.";
  return b;
}
// A calendar flight scheduled for a future day: it cannot be tracked yet,
// so it shows that it is upcoming rather than a live in-view snapshot.
function upcomingBadge() {
  const b = document.createElement("span");
  b.className = "badge off";
  b.textContent = "○ upcoming";
  b.title = "Scheduled for a future date; it will flag on the scope on " +
    "the day of the flight.";
  return b;
}
// One feed's sync line: result and age of the last fetch attempt.
function feedStatus(item) {
  const st = document.createElement("div");
  st.className = "status";
  if (item.syncAgeS === undefined) {
    st.textContent = "not yet synced - save to sync now";
  } else if (item.syncOk) {
    st.classList.add("ok");
    const n = item.syncFlights ?? 0;
    st.textContent = `✓ synced ${ago(item.syncAgeS)}, ` +
      (n ? `${n} upcoming flight${n > 1 ? "s" : ""}` : "no upcoming flights");
  } else {
    st.classList.add("bad");
    st.textContent = `✗ sync failed ${ago(item.syncAgeS)}`;
  }
  if (item.syncTime) {
    st.title = "Last sync: " + new Date(item.syncTime * 1000).toLocaleString();
  }
  return st;
}
function render() {
  document.getElementById("range").value = data.range;
  document.getElementById("feednote").textContent =
    `Synced every ${Math.round((data.icalPollMs ?? 3600000) / 60000)} min; ` +
    "saving this page syncs right away.";
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
        input.oninput = () => { item[f] = input.value; };
        // Saved Wi-Fi passwords never come back from the API; leaving
        // the field blank on save keeps the one stored on the device.
        if (f === "password") {
          input.type = "password";
          if (item.hasPassword) input.placeholder = "(saved - blank keeps it)";
        }
        // Coordinates post as numbers whenever they parse; the firmware
        // accepts either, but numbers keep the config file tidy.
        if (f === "lat" || f === "lon" || f === "zoom") {
          input.oninput = () => {
            const n = Number(input.value);
            item[f] = input.value !== "" && Number.isFinite(n) ? n
                                                              : input.value;
          };
        }
        // Dates are entered day-first and stored ISO; an entry that does
        // not parse is flagged and can never match a date on the device.
        if (f === "date") {
          input.value = isoToDmy(item[f] ?? "");
          input.oninput = () => {
            const iso = dmyToIso(input.value);
            input.classList.toggle("bad", input.value !== "" && !iso);
            item[f] = iso ?? input.value;
          };
        }
        row.appendChild(input);
      });
      if (key === "specials" && "found" in item) {
        row.appendChild(badge(item.found));
      }
      // Homes cycle on the device in list order, so let them be reordered.
      if (key === "homes") {
        const mv = (d) => () => {
          const j = i + d;
          [data.homes[i], data.homes[j]] = [data.homes[j], data.homes[i]];
          render();
        };
        const up = document.createElement("button");
        up.className = "x"; up.textContent = "↑"; up.title = "Move up";
        up.disabled = i === 0; up.onclick = mv(-1);
        const dn = document.createElement("button");
        dn.className = "x"; dn.textContent = "↓"; dn.title = "Move down";
        dn.disabled = i === data.homes.length - 1; dn.onclick = mv(1);
        row.appendChild(up); row.appendChild(dn);
      }
      const del = document.createElement("button");
      del.className = "x"; del.textContent = "✕";
      del.onclick = () => { data[key].splice(i, 1); render(); };
      row.appendChild(del);
      host.appendChild(row);
      if (key === "feeds") host.appendChild(feedStatus(item));
    });
  }
  // Today's flights out of the calendar feeds: read-only - to change
  // them, edit the trip in the calendar itself.
  const ihost = document.getElementById("icalspecials");
  ihost.innerHTML = "";
  (data.icalSpecials ?? []).forEach((item) => {
    const row = document.createElement("div");
    row.className = "row";
    for (const value of [item.flight, isoToDmy(item.date)]) {
      const input = document.createElement("input");
      input.value = value ?? "";
      input.readOnly = true;
      row.appendChild(input);
    }
    const tag = document.createElement("span");
    tag.className = "tag";
    tag.textContent = "from " + (item.source || "calendar");
    tag.title = "From a calendar feed; edit the trip there, not here.";
    row.appendChild(tag);
    // Today's flights carry the live in-view badge; upcoming ones show
    // when they will flag, since they cannot be tracked before their day.
    row.appendChild(item.today ? badge(item.found) : upcomingBadge());
    ihost.appendChild(row);
  });
}
function add(key) { data[key].push({ enabled: true }); render(); }
async function load() {
  data = await (await fetch("/api/state")).json();
  for (const key of ["homes", "pois", "feeds", "specials", "icalSpecials",
                     "wifi"]) {
    data[key] ??= [];
  }
  render();
}
async function save() {
  data.range = Number(document.getElementById("range").value);
  await fetch("/api/config", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(data) });
  document.getElementById("save").textContent = "Saved";
  setTimeout(() => document.getElementById("save").textContent = "Save & apply", 1500);
  load();  // Refresh the badges and sync lines against the new config.
}
load();
</script>
)HTML";
