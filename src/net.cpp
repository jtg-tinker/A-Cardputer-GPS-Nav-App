#include "net.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <WiFi.h>
#include <tilesource.h>

#include "app.h"
#include "certs.h"
#include "input.h"
#include "mapview.h"
#include "storage.h"
#include "ui.h"

using app::canvas;

namespace net {

namespace {

constexpr uint32_t kWifiTimeoutMs = 15000;

String urlEncode(const String &s) {
    static const char *hex = "0123456789ABCDEF";
    String out;
    for (size_t i = 0; i < s.length(); ++i) {
        const uint8_t c = s[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') out += char(c);
        else if (c == ' ') out += '+';
        else {
            out += '%';
            out += hex[c >> 4];
            out += hex[c & 15];
        }
    }
    return out;
}

bool cancelRequested() {
    Key k = readKey();
    return k.back || k.del;
}

void status(const char *title, const String &body) { ui::message(title, body, false); }

}  // namespace

bool wifiSetup() {
    status("WiFi", "Scanning for networks...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    const int n = WiFi.scanNetworks();
    if (n <= 0) {
        ui::message("WiFi", "No networks found.");
        return false;
    }
    std::vector<String> items;
    std::vector<String> ssids;
    std::vector<bool> open;
    for (int i = 0; i < n && i < 30; ++i) {
        const String ssid = WiFi.SSID(i);
        if (!ssid.length()) continue;
        ssids.push_back(ssid);
        open.push_back(WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
        items.push_back(ssid + " (" + WiFi.RSSI(i) + ")" + (open.back() ? "" : " *"));
    }
    WiFi.scanDelete();
    const int sel = ui::menu("Choose WiFi", items);
    if (sel < 0) return false;
    String pass;
    if (!open[sel] && !ui::textInput("WiFi password", pass, true, ssids[sel].c_str())) return false;
    app::config.wifiSsid = ssids[sel];
    app::config.wifiPass = pass;
    app::config.save();
    return true;
}

bool wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) return true;
    mapView.releaseBase();
    while (true) {
        if (!app::config.wifiSsid.length() && !wifiSetup()) return false;
        WiFi.mode(WIFI_STA);
        WiFi.begin(app::config.wifiSsid.c_str(), app::config.wifiPass.c_str());
        status("WiFi", "Connecting to " + app::config.wifiSsid + "...\n\n` to cancel");
        const uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < kWifiTimeoutMs) {
            app::service();
            if (cancelRequested()) {
                wifiOff();
                return false;
            }
            delay(50);
        }
        if (WiFi.status() == WL_CONNECTED) return true;
        WiFi.disconnect();
        const int c = ui::menu("WiFi failed", {"Retry", "Choose another network", "Cancel"});
        if (c == 1 && !wifiSetup()) return false;
        if (c != 0 && c != 1) {
            wifiOff();
            return false;
        }
    }
}

void wifiOff() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

bool geocode(const String &query, std::vector<GeoResult> &out, String &err) {
    // Nominatim usage policy: at most 1 request per second.
    static uint32_t lastRequestAt = 0;
    while (lastRequestAt && millis() - lastRequestAt < 1100) delay(20);
    lastRequestAt = millis();
    out.clear();
    NetworkClientSecure client;
    client.setCACert(kRootCAs);
    HTTPClient http;
    const String url = "https://nominatim.openstreetmap.org/search?format=jsonv2&limit=6&q=" + urlEncode(query);
    if (!http.begin(client, url)) {
        err = "Bad URL";
        return false;
    }
    http.setUserAgent(app::kUserAgent);
    http.setTimeout(15000);
    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        err = code < 0 ? http.errorToString(code) : ("HTTP " + String(code));
        http.end();
        return false;
    }
    JsonDocument filter;
    filter[0]["display_name"] = true;
    filter[0]["lat"] = true;
    filter[0]["lon"] = true;
    filter[0]["boundingbox"] = true;
    JsonDocument doc;
    const String body = http.getString();
    http.end();
    if (deserializeJson(doc, body, DeserializationOption::Filter(filter))) {
        err = "Bad response";
        return false;
    }
    for (JsonObject o : doc.as<JsonArray>()) {
        GeoResult r;
        r.name = o["display_name"] | "";
        r.center = {atof(o["lat"] | "0"), atof(o["lon"] | "0")};
        JsonArray bb = o["boundingbox"];  // [south, north, west, east]
        if (bb.size() == 4) r.box = {atof(bb[0] | "0"), atof(bb[2] | "0"), atof(bb[1] | "0"), atof(bb[3] | "0")};
        out.push_back(r);
    }
    if (out.empty()) err = "No results";
    return !out.empty();
}

namespace {

enum class FetchResult { Ok, NotFound, AuthError, Failed };

FetchResult fetchTile(HTTPClient &http, NetworkClientSecure &client, int z, int x, int y, const char *path,
                      int &lastCode) {
    const auto &cfg = app::config;
    char url[256];
    if (!nav::buildTileUrl(cfg.provider, cfg.style.c_str(), cfg.apiKey.c_str(), z, x, y, url, sizeof url))
        return FetchResult::AuthError;
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (!http.begin(client, url)) return FetchResult::Failed;
        http.setUserAgent(app::kUserAgent);
        http.setTimeout(15000);
        http.setConnectTimeout(10000);
        static const char *kHeaders[] = {"Content-Type"};
        http.collectHeaders(kHeaders, 1);
        const int code = http.GET();
        lastCode = code;
        if (code == HTTP_CODE_OK && http.header("Content-Type").startsWith("image/")) {
            const String tmp = String(path) + ".part";
            File f = SD.open(tmp, FILE_WRITE);
            if (!f) {
                http.end();
                return FetchResult::Failed;
            }
            const int written = http.writeToStream(&f);
            f.close();
            http.end();
            if (written > 0) {
                SD.remove(path);
                if (SD.rename(tmp, path)) return FetchResult::Ok;
            }
            SD.remove(tmp);
            continue;
        }
        http.end();
        if (code == 401 || code == 403) return FetchResult::AuthError;
        if (code == 404 || code == 204 || code == 400) return FetchResult::NotFound;
        if (code == 429) {
            const uint32_t until = millis() + 10000;
            while (millis() < until) { app::service(); delay(50); }
        } else if (code < 0) {
            client.stop();
            delay(500);
        }
    }
    return FetchResult::Failed;
}

}  // namespace

String testProvider() {
    if (!app::config.apiKey.length()) return "No API key set.";
    if (!wifiConnect()) return "WiFi not connected.";
    status("Testing", "Fetching one tile...");
    NetworkClientSecure client;
    client.setCACert(kRootCAs);
    HTTPClient http;
    const char *path = "/gpsnav/test-tile.jpg";
    int code = 0;
    const FetchResult r = fetchTile(http, client, 1, 0, 0, path, code);
    SD.remove(path);
    wifiOff();
    switch (r) {
        case FetchResult::Ok: return "Success! Provider, style and key work.";
        case FetchResult::AuthError: return "Key rejected (HTTP " + String(code) + "). Check the API key and style.";
        case FetchResult::NotFound: return "HTTP " + String(code) + ": style not found? Check the style name.";
        default: return "Failed (" + (code < 0 ? HTTPClient::errorToString(code) : "HTTP " + String(code)) + ").";
    }
}

bool downloadRegion(const Region &r) {
    auto &cfg = app::config;
    if (!wifiConnect()) return false;

    NetworkClientSecure client;
    client.setCACert(kRootCAs);
    HTTPClient http;
    http.setReuse(true);

    const uint32_t total = nav::tileCount(r.box, r.zMin, r.zMax);
    uint32_t done = 0, ok = 0, skipped = 0, failed = 0, missing = 0;
    const uint32_t startedAt = millis();
    uint32_t lastDraw = 0;
    int lastCode = 0;
    String abortReason;
    char path[112], dir[96];

    auto draw = [&](int z) {
        canvas.fillScreen(ui::kBg);
        ui::header("Downloading", r.name.c_str());
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(ui::kFg);
        const float pct = total ? 100.0f * done / total : 100.0f;
        canvas.drawRect(4, 22, app::kW - 8, 12, ui::kAccent);
        canvas.fillRect(6, 24, int((app::kW - 12) * pct / 100.0f), 8, ui::kAccent);
        char line[64];
        snprintf(line, sizeof line, "%lu / %lu tiles (%.1f%%)  zoom %d", (unsigned long)done, (unsigned long)total,
                 pct, z);
        canvas.drawString(line, 4, 40);
        snprintf(line, sizeof line, "new %lu  cached %lu  failed %lu", (unsigned long)ok, (unsigned long)skipped,
                 (unsigned long)(failed + missing));
        canvas.drawString(line, 4, 52);
        const uint32_t elapsed = (millis() - startedAt) / 1000;
        const uint32_t fetched = ok + failed + missing;
        if (fetched > 5 && done < total) {
            const float rate = float(fetched) / max<uint32_t>(elapsed, 1);
            const uint32_t eta = uint32_t((total - done) / max(rate, 0.01f));
            snprintf(line, sizeof line, "%.1f tiles/s  ETA %lum%02lus", rate, (unsigned long)(eta / 60),
                     (unsigned long)(eta % 60));
            canvas.drawString(line, 4, 64);
        }
        snprintf(line, sizeof line, "%s/%s  last HTTP %d", nav::providerId(cfg.provider), cfg.style.c_str(), lastCode);
        canvas.setTextColor(ui::kDim);
        canvas.drawString(line, 4, 76);
        canvas.drawString("WiFi " + WiFi.SSID() + "  " + String(WiFi.RSSI()) + " dBm", 4, 88);
        ui::footer("` cancel (resume later from Saved regions)");
        ui::present();
    };

    for (int z = r.zMin; z <= r.zMax && !abortReason.length(); ++z) {
        const nav::TileRange tr = nav::tileRangeFor(r.box, z);
        for (int x = tr.x0; x <= tr.x1 && !abortReason.length(); ++x) {
            if (nav::buildTileDir(storage::kTilesRoot, cfg.provider, cfg.style.c_str(), z, x, dir, sizeof dir))
                storage::mkdirs(dir);
            for (int y = tr.y0; y <= tr.y1; ++y) {
                app::service();
                if (cancelRequested() && ui::confirm("Cancel download?", "Tiles fetched so far are kept.")) {
                    abortReason = "Cancelled.";
                    break;
                }
                if (!nav::buildTilePath(storage::kTilesRoot, cfg.provider, cfg.style.c_str(), z, x, y, path,
                                        sizeof path)) {
                    abortReason = "Invalid style name.";
                    break;
                }
                if (SD.exists(path)) {
                    ++skipped;
                } else {
                    switch (fetchTile(http, client, z, x, y, path, lastCode)) {
                        case FetchResult::Ok: ++ok; break;
                        case FetchResult::NotFound: ++missing; break;
                        case FetchResult::AuthError:
                            abortReason = "API key or style rejected (HTTP " + String(lastCode) +
                                          "). Check Settings.";
                            break;
                        case FetchResult::Failed:
                            ++failed;
                            if (WiFi.status() != WL_CONNECTED && !wifiConnect()) abortReason = "WiFi lost.";
                            break;
                    }
                    if (abortReason.length()) break;
                }
                ++done;
                if (millis() - lastDraw > 250) {
                    lastDraw = millis();
                    draw(z);
                }
            }
        }
    }
    http.end();
    client.stop();
    wifiOff();

    char summary[160];
    snprintf(summary, sizeof summary, "%lu new, %lu already cached, %lu failed, %lu unavailable.",
             (unsigned long)ok, (unsigned long)skipped, (unsigned long)failed, (unsigned long)missing);
    if (abortReason.length()) {
        ui::message("Download stopped", abortReason + "\n\n" + summary);
        return false;
    }
    ui::message("Download complete", String(summary) + (failed ? "\n\nRun it again to retry failed tiles." : ""));
    return true;
}

bool loadRegions(std::vector<Region> &out) {
    out.clear();
    File f = SD.open(storage::kRegionsPath, FILE_READ);
    if (!f) return false;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (!line.length() || line[0] == '#') continue;
        // name;south;west;north;east;zmin;zmax
        Region r;
        double v[6];
        int start = line.indexOf(';');
        if (start < 0) continue;
        r.name = line.substring(0, start);
        int n = 0;
        while (n < 6 && start >= 0) {
            const int next = line.indexOf(';', start + 1);
            v[n++] = line.substring(start + 1, next < 0 ? line.length() : next).toDouble();
            start = next;
        }
        if (n != 6) continue;
        r.box = {v[0], v[1], v[2], v[3]};
        r.zMin = int(v[4]);
        r.zMax = int(v[5]);
        if (nav::bboxValid(r.box)) out.push_back(r);
    }
    f.close();
    return true;
}

bool saveRegions(const std::vector<Region> &regions) {
    File f = SD.open(storage::kRegionsPath, FILE_WRITE);
    if (!f) return false;
    f.print("# name;south;west;north;east;zmin;zmax\n");
    for (const auto &r : regions) {
        String name = r.name;
        name.replace(";", ",");
        f.printf("%s;%.6f;%.6f;%.6f;%.6f;%d;%d\n", name.c_str(), r.box.south, r.box.west, r.box.north, r.box.east,
                 r.zMin, r.zMax);
    }
    f.close();
    return true;
}

}  // namespace net
