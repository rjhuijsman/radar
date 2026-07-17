#include "Net.h"

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFiManager.h>

#include "Feeds.h"
#include "config.h"
#include "web_page.h"

namespace net {
namespace {

constexpr char CONFIG_PATH[] = "/config.json";

AsyncWebServer g_server(80);
model::Model* g_model = nullptr;
SemaphoreHandle_t g_mutex = nullptr;

void lock() {
  if (g_mutex) xSemaphoreTake(g_mutex, portMAX_DELAY);
}
void unlock() {
  if (g_mutex) xSemaphoreGive(g_mutex);
}

// Serializes the persistent config (everything the web UI edits) to JSON.
void configToJson(const model::Model& model, JsonDocument& doc) {
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

  feeds::reprojectStatics(model);
}

void saveConfig(const model::Model& model) {
  JsonDocument doc;
  configToJson(model, doc);
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
  g_server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    // ESP32 flash is memory-mapped, so the PROGMEM page is directly readable.
    request->send(200, "text/html", CONFIG_PAGE);
  });
  g_server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;
    lock();
    configToJson(*g_model, doc);
    unlock();
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });
  g_server.on("/api/config", HTTP_POST,
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

}  // namespace

bool begin(model::Model& model, SemaphoreHandle_t mutex) {
  g_model = &model;
  g_mutex = mutex;

  LittleFS.begin(true);
  lock();
  loadConfig(model);
  unlock();

  WiFiManager manager;
  if (!manager.autoConnect(config::AP_NAME)) {
    return false;  // Portal timed out; caller may restart.
  }
  setupOta();
  routes();
  g_server.begin();
  return true;
}

void loopOta() { ArduinoOTA.handle(); }

}  // namespace net
