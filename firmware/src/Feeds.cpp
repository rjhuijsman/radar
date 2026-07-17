#include "Feeds.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <math.h>

#include <set>

#include "config.h"

namespace feeds {
namespace {

// ICAO callsigns currently considered special, rebuilt by `pollIcal`.
std::set<String> g_specials;

// Maps a 2-letter IATA airline code to its 3-letter ICAO prefix. Only a few
// common carriers for now. TODO(feeds): load the full table from LittleFS.
String iataToIcao(const String& iata) {
  struct Entry {
    const char* iata;
    const char* icao;
  };
  static const Entry table[] = {
      {"BA", "BAW"}, {"LH", "DLH"}, {"AF", "AFR"}, {"KL", "KLM"},
      {"U2", "EZY"}, {"FR", "RYR"}, {"UA", "UAL"}, {"AA", "AAL"},
      {"DL", "DAL"}, {"EK", "UAE"}, {"IB", "IBE"}, {"VS", "VIR"},
  };
  for (auto& entry : table) {
    if (iata == entry.iata) return entry.icao;
  }
  return "";
}

const model::Home& activeHome(const model::Model& model) {
  static model::Home origin;  // 0,0 fallback if no homes configured.
  if (model.ui.homeIndex >= 0 &&
      model.ui.homeIndex < static_cast<int>(model.homes.size())) {
    return model.homes[model.ui.homeIndex];
  }
  return origin;
}

// Converts a lat/lng to world NM (east, north) relative to home, using a
// local equirectangular approximation — accurate at radar ranges.
model::Vec geoToWorld(const model::Home& home, float lat, float lon) {
  float dLatNm = (lat - home.latitude) * 60.0f;
  float dLonNm = (lon - home.longitude) * 60.0f * cosf(home.latitude * DEG_TO_RAD);
  return model::Vec{dLonNm, dLatNm};
}

bool httpGet(const String& url, String& body) {
  WiFiClientSecure client;
  // TODO(security): install a CA bundle and validate certificates instead
  // of trusting any peer.
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    body = http.getString();
  }
  http.end();
  return code == HTTP_CODE_OK;
}

// Finds an existing aircraft by callsign so its sweep-refresh state carries
// across polls; appends a fresh one otherwise.
model::Aircraft& upsert(model::Model& model, const String& callsign) {
  for (auto& ac : model.aircraft) {
    if (ac.callsign == callsign) return ac;
  }
  model.aircraft.push_back(model::Aircraft{});
  model.aircraft.back().callsign = callsign;
  return model.aircraft.back();
}

}  // namespace

bool pollTraffic(model::Model& model) {
  const model::Home& home = activeHome(model);
  // airplanes.live point + radius (nm). Range plus a margin so off-map
  // specials still resolve.
  String url = "https://api.airplanes.live/v2/point/" +
               String(home.latitude, 5) + "/" + String(home.longitude, 5) +
               "/" + String(static_cast<int>(model.ui.range * 1.5f + 20));
  String body;
  if (!httpGet(url, body)) return false;

  JsonDocument doc;
  if (deserializeJson(doc, body)) return false;

  std::set<String> present;
  for (JsonObject state : doc["ac"].as<JsonArray>()) {
    const char* flight = state["flight"];
    if (flight == nullptr) continue;
    String callsign = String(flight);
    callsign.trim();
    if (callsign.isEmpty() || !state["lat"].is<float>()) continue;

    model::Aircraft& ac = upsert(model, callsign);
    ac.pos = geoToWorld(home, state["lat"], state["lon"]);
    ac.track = state["track"] | 0.0f;
    ac.groundSpeed = state["gs"] | 0.0f;
    ac.altitude = state["alt_baro"] | 0;
    ac.special = g_specials.count(callsign) > 0;
    present.insert(callsign);
  }

  // Drop aircraft no longer in the feed.
  for (int i = static_cast<int>(model.aircraft.size()) - 1; i >= 0; --i) {
    if (present.count(model.aircraft[i].callsign) == 0) {
      model.aircraft.erase(model.aircraft.begin() + i);
    }
  }
  return true;
}

int pollIcal(model::Model& model) {
  std::set<String> found;
  int fetched = 0;
  for (const auto& feed : model.feeds) {
    if (!feed.enabled) continue;
    String body;
    if (!httpGet(feed.url, body)) continue;
    ++fetched;

    // Heuristic extraction: scan for an IATA airline code followed by a
    // flight number (e.g. "BA 117") and convert to the ICAO callsign the
    // ADS-B feed broadcasts. TODO(feeds): unfold iCal lines, respect
    // VEVENT boundaries, and filter to flights happening today.
    for (int i = 0; i + 2 < static_cast<int>(body.length()); ++i) {
      char a = body[i], b = body[i + 1];
      if (a < 'A' || a > 'Z' || b < 'A' || b > 'Z') continue;
      int j = i + 2;
      while (j < static_cast<int>(body.length()) && body[j] == ' ') ++j;
      if (j >= static_cast<int>(body.length()) || !isdigit(body[j])) continue;
      String number;
      while (j < static_cast<int>(body.length()) && isdigit(body[j])) {
        number += body[j++];
      }
      if (number.length() < 2 || number.length() > 4) continue;
      String icao = iataToIcao(String(a) + String(b));
      if (!icao.isEmpty()) found.insert(icao + number);
    }
  }
  g_specials = found;
  return fetched;
}

void pollWeather(model::Model& model) {
  // TODO(weather): fetch the latest RainViewer frame's tiles covering the
  // view and reproject them into the scope's azimuthal projection.
  (void)model;
}

void reprojectStatics(model::Model& model) {
  const model::Home& home = activeHome(model);
  for (auto& poi : model.pois) {
    poi.pos = geoToWorld(home, poi.latitude, poi.longitude);
  }
}

}  // namespace feeds
