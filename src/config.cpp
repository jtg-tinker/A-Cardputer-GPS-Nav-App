#include "config.h"

#include <ArduinoJson.h>
#include <SD.h>

#include "storage.h"

bool AppConfig::load() {
    File f = SD.open(storage::kConfigPath, FILE_READ);
    if (!f) return false;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    wifiSsid = doc["wifi_ssid"] | "";
    wifiPass = doc["wifi_pass"] | "";
    nav::Provider p;
    if (nav::parseProvider(doc["provider"] | "", p)) provider = p;
    style = doc["style"] | nav::defaultStyle(provider);
    apiKey = doc["api_key"] | "";
    imperial = doc["imperial"] | false;
    brightness = doc["brightness"] | 160;
    lastLat = doc["last_lat"] | lastLat;
    lastLon = doc["last_lon"] | lastLon;
    lastZoom = doc["last_zoom"] | lastZoom;
    return true;
}

bool AppConfig::save() const {
    JsonDocument doc;
    doc["wifi_ssid"] = wifiSsid;
    doc["wifi_pass"] = wifiPass;
    doc["provider"] = nav::providerId(provider);
    doc["style"] = style;
    doc["api_key"] = apiKey;
    doc["imperial"] = imperial;
    doc["brightness"] = brightness;
    doc["last_lat"] = lastLat;
    doc["last_lon"] = lastLon;
    doc["last_zoom"] = lastZoom;

    const String tmp = String(storage::kConfigPath) + ".tmp";
    File f = SD.open(tmp, FILE_WRITE);
    if (!f) return false;
    const bool ok = serializeJsonPretty(doc, f) > 0;
    f.close();
    if (!ok) return false;
    SD.remove(storage::kConfigPath);
    return SD.rename(tmp, storage::kConfigPath);
}
