#include "Net.h"

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFiManager.h>
#include <WiFiMulti.h>
#include <esp_wifi.h>

#include "Feeds.h"
#include "config.h"
#include "web_page.h"

namespace net {
namespace {

constexpr char CONFIG_PATH[] = "/config.json";

AsyncWebServer g_server(80);
model::Model* g_model = nullptr;
SemaphoreHandle_t g_mutex = nullptr;
// OTA, mDNS and the config server armed yet? They start on the first
// successful connection, which a set that boots offline reaches late.
bool g_servicesStarted = false;

void lock() {
  if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
}
void unlock() {
  if (g_mutex) xSemaphoreGive(g_mutex);
}

// Serializes the persistent config (everything the web UI edits) to
// JSON. `includeSecrets` selects whether saved Wi-Fi passwords are
// written out: true only for the copy persisted to flash. Anything
// served to a browser gets `hasPassword` instead, so a stored password
// can never be read back over the network.
void configToJson(const model::Model& model, JsonDocument& doc,
                  bool includeSecrets) {
  doc["range"] = model.ui.range;
  JsonArray homes = doc["homes"].to<JsonArray>();
  for (const auto& home : model.homes) {
    JsonObject object = homes.add<JsonObject>();
    object["name"] = home.name;
    object["lat"] = home.latitude;
    object["lon"] = home.longitude;
  }
  JsonArray pois = doc["pois"].to<JsonArray>();
  for (const auto& poi : model.pois) {
    JsonObject object = pois.add<JsonObject>();
    object["name"] = poi.name;
    object["lat"] = poi.latitude;
    object["lon"] = poi.longitude;
    object["airport"] = poi.isAirport;
    JsonArray runways = object["runways"].to<JsonArray>();
    for (float heading : poi.runwayHeadings) runways.add(heading);
  }
  JsonArray feeds = doc["feeds"].to<JsonArray>();
  for (const auto& feed : model.feeds) {
    JsonObject object = feeds.add<JsonObject>();
    object["name"] = feed.name;
    object["url"] = feed.url;
    object["color"] = feed.color;
    object["enabled"] = feed.enabled;
  }
  JsonArray specials = doc["specials"].to<JsonArray>();
  for (const auto& s : model.specials) {
    JsonObject object = specials.add<JsonObject>();
    object["flight"] = s.flight;
    object["date"] = s.date;
  }
  JsonArray wifi = doc["wifi"].to<JsonArray>();
  for (const auto& network : model.wifi) {
    JsonObject object = wifi.add<JsonObject>();
    object["ssid"] = network.ssid;
    if (includeSecrets) {
      object["password"] = network.password;
    } else {
      object["hasPassword"] = network.password.length() > 0;
    }
  }
}

// Applies parsed config JSON to the model, then reprojects statics.
void jsonToConfig(const JsonDocument& doc, model::Model& model) {
  if (doc["range"].is<float>()) model.ui.range = doc["range"];

  model.homes.clear();
  for (JsonObjectConst object : doc["homes"].as<JsonArrayConst>()) {
    model::Home home;
    home.name = object["name"] | "Home";
    home.latitude = object["lat"] | 0.0f;
    home.longitude = object["lon"] | 0.0f;
    model.homes.push_back(home);
  }
  if (model.ui.homeIndex >= static_cast<int>(model.homes.size())) {
    model.ui.homeIndex = 0;
  }

  model.pois.clear();
  for (JsonObjectConst object : doc["pois"].as<JsonArrayConst>()) {
    model::Poi poi;
    poi.name = object["name"] | "POI";
    poi.latitude = object["lat"] | 0.0f;
    poi.longitude = object["lon"] | 0.0f;
    poi.isAirport = object["airport"] | false;
    for (JsonVariantConst heading : object["runways"].as<JsonArrayConst>()) {
      poi.runwayHeadings.push_back(heading.as<float>());
    }
    model.pois.push_back(poi);
  }

  model.feeds.clear();
  for (JsonObjectConst object : doc["feeds"].as<JsonArrayConst>()) {
    model::Feed feed;
    feed.name = object["name"] | "Feed";
    feed.url = object["url"] | "";
    feed.color = object["color"] | 0;
    feed.enabled = object["enabled"] | true;
    model.feeds.push_back(feed);
  }

  model.specials.clear();
  for (JsonObjectConst object : doc["specials"].as<JsonArrayConst>()) {
    model::SpecialFlight s;
    s.flight = object["flight"] | "";
    s.date = object["date"] | "";
    if (s.flight.length() > 0) model.specials.push_back(s);
  }

  // Wi-Fi networks: merge rather than replace. Reads never expose the
  // stored passwords, so an entry posted back with an empty password
  // keeps the one already saved for that SSID; a non-empty one sets it.
  // Entries absent from the posted list are forgotten. A document with
  // no `wifi` array at all (a config written by older firmware, or a
  // stale browser tab) leaves the list untouched.
  if (doc["wifi"].is<JsonArrayConst>()) {
    std::vector<model::WifiNetwork> updated;
    for (JsonObjectConst object : doc["wifi"].as<JsonArrayConst>()) {
      if (updated.size() >= config::WIFI_MAX_NETWORKS) break;
      model::WifiNetwork network;
      network.ssid = object["ssid"] | "";
      network.password = object["password"] | "";
      if (network.ssid.length() == 0) continue;
      if (network.password.length() == 0) {
        for (const auto& existing : model.wifi) {
          if (existing.ssid == network.ssid) {
            network.password = existing.password;
            break;
          }
        }
      }
      updated.push_back(network);
    }
    model.wifi = std::move(updated);
  }

  feeds::reprojectStatics(model);
}

void saveConfig(const model::Model& model) {
  JsonDocument doc;
  configToJson(model, doc, /*includeSecrets=*/true);
  File file = LittleFS.open(CONFIG_PATH, "w");
  if (!file) return;
  serializeJson(doc, file);
  file.close();
}

void loadConfig(model::Model& model) {
  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) return;
  JsonDocument doc;
  if (!deserializeJson(doc, file)) jsonToConfig(doc, model);
  file.close();
}

// Accumulates a POST body across chunks, then applies and persists it.
void handleConfigBody(AsyncWebServerRequest* request, uint8_t* data,
                      size_t len, size_t index, size_t total) {
  static String buffer;
  if (index == 0) buffer = "";
  buffer.concat(reinterpret_cast<const char*>(data), len);
  if (index + len != total) return;

  JsonDocument doc;
  if (deserializeJson(doc, buffer)) {
    request->send(400, "text/plain", "bad json");
    return;
  }
  lock();
  jsonToConfig(doc, *g_model);
  saveConfig(*g_model);
  unlock();
  request->send(200, "text/plain", "ok");
}

void routes() {
  // Qualify the method enum: WiFiManager pulls in the core WebServer, whose
  // http_parser.h also defines an unscoped HTTP_GET/HTTP_POST, so the bare
  // names are ambiguous here. AsyncWebRequestMethod:: names the async one.
  g_server.on("/", AsyncWebRequestMethod::HTTP_GET,
              [](AsyncWebServerRequest* request) {
    // ESP32 flash is memory-mapped, so the PROGMEM page is directly readable.
    request->send(200, "text/html", CONFIG_PAGE);
  });
  g_server.on("/api/state", AsyncWebRequestMethod::HTTP_GET,
              [](AsyncWebServerRequest* request) {
    JsonDocument doc;
    lock();
    configToJson(*g_model, doc, /*includeSecrets=*/false);
    unlock();
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });
  g_server.on("/api/config", AsyncWebRequestMethod::HTTP_POST,
              [](AsyncWebServerRequest* request) {}, nullptr,
              handleConfigBody);
}

// Arms ArduinoOTA (Wi-Fi firmware and filesystem updates) with a hashed
// password, and flags the model while an update runs so sleep is deferred.
void setupOta() {
  ArduinoOTA.setHostname(config::MDNS_HOST);
  if (strlen(config::OTA_PASSWORD_HASH) > 0) {
    ArduinoOTA.setPasswordHash(config::OTA_PASSWORD_HASH);
  }
  ArduinoOTA.onStart([]() {
    lock();
    g_model->ui.otaActive = true;
    unlock();
  });
  ArduinoOTA.onEnd([]() {
    lock();
    g_model->ui.otaActive = false;
    unlock();
  });
  ArduinoOTA.onError([](ota_error_t) {
    lock();
    g_model->ui.otaActive = false;
    unlock();
  });
  ArduinoOTA.begin();  // Also brings up mDNS for `<host>.local`.
  MDNS.addService("http", "tcp", 80);
}

// Arms OTA, mDNS and the config server, exactly once, on the first
// successful connection. Split from begin() because a set that boots
// offline (nothing reachable, portal timed out) reaches this late,
// from loopWifi().
void startServices() {
  if (g_servicesStarted) return;
  g_servicesStarted = true;
  Serial.printf("[net] Wi-Fi up: SSID '%s', IP %s, http://%s.local/\n",
                WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                config::MDNS_HOST);
  // Start SNTP (UTC) so special flights can be matched to today's date.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setupOta();
  routes();
  g_server.begin();
}

// One scan-and-join round over the saved networks: WiFiMulti picks the
// strongest one in range. Blocks for the scan plus at most one
// association attempt, so call it from the network task only and never
// with the model mutex held. The multi is rebuilt from the model each
// round, so web edits take effect on the next attempt.
bool tryKnownNetworks() {
  std::vector<model::WifiNetwork> list;
  lock();
  list = g_model->wifi;
  unlock();
  if (list.empty()) return false;
  WiFiMulti multi;
  for (const auto& network : list) {
    multi.addAP(network.ssid.c_str(),
                network.password.length() ? network.password.c_str()
                                          : nullptr);
  }
  return multi.run(config::WIFI_JOIN_TIMEOUT_MS) == WL_CONNECTED;
}

// Folds the network the station is on right now into the saved list —
// appending, never overwriting the rest — and persists any change.
// This is how a credential entered in the captive portal becomes
// permanent.
void rememberCurrentNetwork() {
  String ssid = WiFi.SSID();
  String psk = WiFi.psk();
  if (ssid.length() == 0) return;
  lock();
  bool changed = false;
  bool known = false;
  for (auto& network : g_model->wifi) {
    if (network.ssid == ssid) {
      known = true;
      if (network.password != psk) {
        network.password = psk;
        changed = true;
      }
      break;
    }
  }
  if (!known) {
    if (g_model->wifi.size() < config::WIFI_MAX_NETWORKS) {
      model::WifiNetwork network;
      network.ssid = ssid;
      network.password = psk;
      g_model->wifi.push_back(network);
      changed = true;
    } else {
      Serial.printf("[net] saved-network list full; not remembering '%s'\n",
                    ssid.c_str());
    }
  }
  if (changed) {
    saveConfig(*g_model);
    Serial.printf("[net] remembered network '%s' (%u saved)\n", ssid.c_str(),
                  static_cast<unsigned>(g_model->wifi.size()));
  }
  unlock();
}

// One-time migration from the single-credential firmware: the config
// list starts out empty, but NVS still holds the network the set was
// last on (WiFiManager stored it there). Seed the list from it so this
// update never costs connectivity — the credential is readable without
// associating, so this works even away from that network. Requires the
// STA driver up (WiFi.mode) so the NVS config is loaded.
void migrateNvsCredential() {
  lock();
  bool empty = g_model->wifi.empty();
  unlock();
  if (!empty) return;
  wifi_config_t stored = {};
  if (esp_wifi_get_config(WIFI_IF_STA, &stored) != ESP_OK) return;
  const char* rawSsid = reinterpret_cast<const char*>(stored.sta.ssid);
  const char* rawPsk = reinterpret_cast<const char*>(stored.sta.password);
  // The driver's fields are fixed-size and not necessarily terminated.
  size_t ssidLen = strnlen(rawSsid, sizeof(stored.sta.ssid));
  if (ssidLen == 0) return;
  model::WifiNetwork network;
  network.ssid.concat(rawSsid, ssidLen);
  network.password.concat(rawPsk,
                          strnlen(rawPsk, sizeof(stored.sta.password)));
  lock();
  g_model->wifi.push_back(network);
  saveConfig(*g_model);
  unlock();
  Serial.printf("[net] migrated NVS credential '%s' into the saved list\n",
                network.ssid.c_str());
}

}  // namespace

bool begin(model::Model& model, SemaphoreHandle_t mutex) {
  g_model = &model;
  g_mutex = mutex;

  bool fsOk = LittleFS.begin(true);
  Serial.printf("[net] LittleFS mount: %s\n", fsOk ? "OK" : "FAILED");
  lock();
  loadConfig(model);
  unlock();

  WiFi.mode(WIFI_STA);
  migrateNvsCredential();

  lock();
  size_t saved = model.wifi.size();
  unlock();

  // First choice: a saved network. Scan-and-join rounds until one
  // connects or the boot budget runs out.
  bool connected = false;
  if (saved > 0) {
    Serial.printf("[net] scanning for %u saved network(s)...\n",
                  static_cast<unsigned>(saved));
    uint32_t start = millis();
    while (!connected && millis() - start < config::WIFI_BOOT_SCAN_MS) {
      connected = tryKnownNetworks();
      if (!connected) delay(2000);
    }
  }

  // Fallback: the captive portal. Whatever it joins is appended to the
  // saved list, never replacing it. With nothing saved the portal is
  // the only way online, so it reopens until provisioned; otherwise a
  // timeout hands over to the background rescans in loopWifi().
  while (!connected) {
    Serial.printf("[net] no saved network reachable; portal SSID '%s'...\n",
                  config::AP_NAME);
    WiFiManager manager;
    manager.setConfigPortalTimeout(config::WIFI_PORTAL_TIMEOUT_S);
    if (manager.startConfigPortal(config::AP_NAME)) {
      connected = true;
      rememberCurrentNetwork();
    } else if (saved > 0) {
      Serial.println("[net] portal timed out; rescanning in the background.");
      return false;
    }
  }

  startServices();
  return true;
}

void loopOta() { ArduinoOTA.handle(); }

void loopWifi() {
  static uint32_t lastAttemptMs = 0;
  if (WiFi.isConnected()) {
    startServices();           // A set that booted offline arms these late.
    lastAttemptMs = millis();  // A drop waits one full interval to rescan.
    return;
  }
  uint32_t now = millis();
  if (now - lastAttemptMs < config::WIFI_RETRY_MS) return;
  lastAttemptMs = now;
  if (tryKnownNetworks()) {
    Serial.printf("[net] rejoined '%s', IP %s\n", WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
  }
}

bool online() { return WiFi.isConnected(); }

}  // namespace net
