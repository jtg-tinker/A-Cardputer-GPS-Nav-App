#include "screens.h"

#include <algorithm>
#include <tilesource.h>

#include "app.h"
#include "input.h"
#include "net.h"
#include "storage.h"
#include "ui.h"

using app::canvas;

namespace screens {

namespace {

constexpr uint32_t kMaxTilesPerRegion = 60000;
constexpr uint32_t kAvgTileBytes = 20 * 1024;
constexpr int kDefaultMinZoom = 5;

String fmtDist(double m) {
    char b[16];
    nav::formatDistance(m, app::config.imperial, b, sizeof b);
    return b;
}

String fmtBytes(uint64_t b) {
    char s[16];
    if (b >= 1024ull * 1024 * 1024) snprintf(s, sizeof s, "%.1fGB", b / 1073741824.0);
    else snprintf(s, sizeof s, "%.0fMB", b / 1048576.0 + 0.5);
    return s;
}

String keyHint(const String &key) {
    if (!key.length()) return "(not set)";
    return "****" + key.substring(max(0, int(key.length()) - 4));
}

bool ensureApiKey() {
    auto &cfg = app::config;
    if (cfg.apiKey.length()) return true;
    String key;
    const String prompt = String("Free key from ") +
                          (cfg.provider == nav::Provider::MapTiler ? "cloud.maptiler.com" : "thunderforest.com") +
                          " (change provider in Settings)";
    if (!ui::textInput("API key", key, false, prompt.c_str())) return false;
    key.trim();
    if (!nav::isSafeToken(key.c_str())) {
        ui::message("API key", "That key contains unexpected characters.");
        return false;
    }
    cfg.apiKey = key;
    cfg.save();
    return true;
}

bool parseLatLon(const String &s, nav::LatLon &out) {
    String t = s;
    t.replace(',', ' ');
    t.trim();
    const int sp = t.indexOf(' ');
    if (sp < 0) return false;
    String a = t.substring(0, sp), b = t.substring(sp + 1);
    b.trim();
    if (!a.length() || !b.length()) return false;
    out.lat = a.toDouble();
    out.lon = b.toDouble();
    return out.lat >= -85 && out.lat <= 85 && out.lon >= -180 && out.lon <= 180 && (a != "0" || b != "0" || true);
}

bool pickRadius(const nav::LatLon &c, nav::BBox &box) {
    static const double kKm[] = {1, 2, 5, 10, 25, 50, 100};
    std::vector<String> items;
    for (double km : kKm) items.push_back(fmtDist(km * 1000) + " radius");
    const int sel = ui::menu("Area size", items, 2);
    if (sel < 0) return false;
    box = nav::bboxAround(c, kKm[sel] * 1000);
    return true;
}

// Lets the user choose the max zoom (with tile/size estimates), name and confirm.
bool planAndDownload(Region r) {
    if (!ensureApiKey()) return false;
    std::vector<String> items;
    std::vector<int> zooms;
    for (int z = 10; z <= nav::kMaxZoom; ++z) {
        const uint32_t n = nav::tileCount(r.box, r.zMin, z);
        items.push_back("Max zoom " + String(z) + ": " + String(n) + " tiles ~" + fmtBytes(uint64_t(n) * kAvgTileBytes) +
                        (n > kMaxTilesPerRegion ? " (too many)" : ""));
        zooms.push_back(z);
    }
    int sel = 0;
    for (size_t i = 0; i < zooms.size(); ++i)
        if (zooms[i] <= r.zMax && nav::tileCount(r.box, r.zMin, zooms[i]) <= kMaxTilesPerRegion) sel = i;
    while (true) {
        sel = ui::menu("Detail level (z16 = streets)", items, sel);
        if (sel < 0) return false;
        if (nav::tileCount(r.box, r.zMin, zooms[sel]) <= kMaxTilesPerRegion) break;
        ui::message("Too many tiles", "Pick a lower max zoom or a smaller area.");
    }
    r.zMax = zooms[sel];
    if (!ui::textInput("Region name", r.name)) return false;
    r.name.replace(";", ",");
    if (!r.name.length()) r.name = "Region";

    const uint32_t n = nav::tileCount(r.box, r.zMin, r.zMax);
    const uint64_t need = uint64_t(n) * kAvgTileBytes;
    const uint64_t free = storage::freeBytes();
    String body = String(n) + " tiles (z" + r.zMin + "-" + r.zMax + "), about " + fmtBytes(need) + ". SD free: " +
                  fmtBytes(free) + ".\nUses " + String(n) + " requests of your " +
                  nav::providerId(app::config.provider) +
                  " monthly quota (cached tiles are skipped).\n\nStart download?";
    if (need > free) body = "WARNING: not enough SD space!\n" + body;
    if (!ui::confirm("Download region", body)) return false;

    std::vector<Region> regions;
    net::loadRegions(regions);
    regions.erase(std::remove_if(regions.begin(), regions.end(), [&](const Region &o) { return o.name == r.name; }),
                  regions.end());
    regions.push_back(r);
    net::saveRegions(regions);

    const bool done = net::downloadRegion(r);
    if (done) app::requestCenter((r.box.north + r.box.south) / 2, (r.box.east + r.box.west) / 2, min(r.zMax, 16));
    return done;
}

}  // namespace

void mainMenu(const nav::LatLon &mapCenter) {
    int sel = 0;
    while (true) {
        std::vector<String> items = {
            "Download map region",
            "Saved regions",
            "Waypoints / go to",
            "Routes & tracks (GPX)",
            "Go to coordinates",
            app::navdata.recording() ? "Stop track recording" : "Start track recording",
            "Clear navigation",
            "GPS status",
            "Settings",
            "Storage",
            "Help / keys",
        };
        sel = ui::menu("GPS Nav", items, sel);
        switch (sel) {
            case 0: downloadMenu(mapCenter, 12); return;
            case 1: regionsMenu(); return;
            case 2: waypointsMenu(mapCenter); return;
            case 3: routesMenu(); return;
            case 4: gotoCoordinates(); return;
            case 5: toggleRecording(); return;
            case 6: clearNavigation(); return;
            case 7: gpsStatus(); break;
            case 8: settingsMenu(); break;
            case 9: storageInfo(); break;
            case 10: help(); break;
            default: return;
        }
    }
}

void downloadMenu(const nav::LatLon &mapCenter, int) {
    const GpsFix fix = app::gps.fix();
    const int sel = ui::menu("Download maps", {"Around my GPS position", "Around map center", "Search a place",
                                               "Enter coordinates", "Saved regions (resume/update)"});
    Region r;
    r.zMin = kDefaultMinZoom;
    char name[40];
    switch (sel) {
        case 0:
            if (!fix.valid) {
                ui::message("No GPS fix", "Wait for a fix (see GPS status) or use 'Around map center'.");
                return;
            }
            if (!pickRadius({fix.lat, fix.lon}, r.box)) return;
            snprintf(name, sizeof name, "Near %.3f,%.3f", fix.lat, fix.lon);
            r.name = name;
            break;
        case 1:
            if (!pickRadius(mapCenter, r.box)) return;
            snprintf(name, sizeof name, "Near %.3f,%.3f", mapCenter.lat, mapCenter.lon);
            r.name = name;
            break;
        case 2: {
            String q;
            if (!ui::textInput("Search place", q, false, "City, region, park... (OpenStreetMap Nominatim)") ||
                !q.length())
                return;
            if (!net::wifiConnect()) return;
            ui::message("Searching", q, false);
            std::vector<GeoResult> results;
            String err;
            const bool ok = net::geocode(q, results, err);
            if (!ok) {
                net::wifiOff();
                ui::message("Search failed", err);
                return;
            }
            std::vector<String> items;
            for (const auto &g : results) items.push_back(g.name);
            const int pick = ui::menu("Results", items);
            if (pick < 0) {
                net::wifiOff();
                return;
            }
            const GeoResult &g = results[pick];
            const double w = nav::haversineM({g.center.lat, g.box.west}, {g.center.lat, g.box.east});
            const double h = nav::haversineM({g.box.south, g.center.lon}, {g.box.north, g.center.lon});
            const int how = ui::menu("Area", {"Its boundary (" + fmtDist(w) + " x " + fmtDist(h) + ")",
                                              "Radius around its center"});
            if (how < 0) return;
            if (how == 0 && nav::bboxValid(g.box)) r.box = g.box;
            else if (!pickRadius(g.center, r.box)) return;
            r.name = g.name.substring(0, max(0, g.name.indexOf(',')) ?: g.name.length());
            if (r.name.length() > 30) r.name = r.name.substring(0, 30);
            break;
        }
        case 3: {
            String s;
            nav::LatLon c;
            if (!ui::textInput("Coordinates", s, false, "lat, lon in decimal degrees, e.g. 47.6062, -122.3321")) return;
            if (!parseLatLon(s, c)) {
                ui::message("Coordinates", "Could not parse '" + s + "'.");
                return;
            }
            if (!pickRadius(c, r.box)) return;
            snprintf(name, sizeof name, "Near %.3f,%.3f", c.lat, c.lon);
            r.name = name;
            break;
        }
        case 4: regionsMenu(); return;
        default: return;
    }
    planAndDownload(r);
}

void regionsMenu() {
    std::vector<Region> regions;
    net::loadRegions(regions);
    if (regions.empty()) {
        ui::message("Saved regions", "No regions yet. Use 'Download map region' first.");
        return;
    }
    std::vector<String> items;
    for (const auto &r : regions) items.push_back(r.name + " z" + r.zMin + "-" + r.zMax);
    const int sel = ui::menu("Saved regions", items);
    if (sel < 0) return;
    const Region r = regions[sel];
    const int act = ui::menu(r.name.c_str(), {"Show on map", "Resume / update download", "Change detail & download",
                                              "Forget region (keeps tiles)"});
    if (act == 0) app::requestCenter((r.box.north + r.box.south) / 2, (r.box.east + r.box.west) / 2, min(r.zMax, 13));
    else if (act == 1 && ensureApiKey()) net::downloadRegion(r);
    else if (act == 2) planAndDownload(r);
    else if (act == 3 && ui::confirm("Forget region", r.name + "\n\nDownloaded tiles stay on the SD card.")) {
        regions.erase(regions.begin() + sel);
        net::saveRegions(regions);
    }
}

void quickWaypoint(const nav::LatLon &mapCenter) {
    const GpsFix fix = app::gps.fix();
    const nav::LatLon at = fix.valid ? nav::LatLon{fix.lat, fix.lon} : mapCenter;
    String name = app::navdata.nextWaypointName();
    const String prompt = String(fix.valid ? "At GPS position " : "At map center ") + String(at.lat, 5) + ", " +
                          String(at.lon, 5);
    if (!ui::textInput("New waypoint", name, false, prompt.c_str())) return;
    if (!name.length()) name = app::navdata.nextWaypointName();
    if (!app::navdata.addWaypoint(name, at.lat, at.lon)) ui::message("Waypoint", "Could not save (limit 200?).");
}

void waypointsMenu(const nav::LatLon &mapCenter) {
    auto &nd = app::navdata;
    const GpsFix fix = app::gps.fix();
    const nav::LatLon ref = fix.valid ? nav::LatLon{fix.lat, fix.lon} : mapCenter;
    std::vector<size_t> order(nd.waypoints.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return nav::haversineM(ref, {nd.waypoints[a].lat, nd.waypoints[a].lon}) <
               nav::haversineM(ref, {nd.waypoints[b].lat, nd.waypoints[b].lon});
    });
    std::vector<String> items = {"+ New waypoint here"};
    for (size_t i : order) {
        const auto &w = nd.waypoints[i];
        const nav::LatLon p{w.lat, w.lon};
        items.push_back(String(w.name) + "  " + fmtDist(nav::haversineM(ref, p)) + " " +
                        nav::compassPoint(nav::initialBearingDeg(ref, p)));
    }
    const int sel = ui::menu("Waypoints", items);
    if (sel < 0) return;
    if (sel == 0) {
        quickWaypoint(mapCenter);
        return;
    }
    const size_t idx = order[sel - 1];
    const Waypoint w = nd.waypoints[idx];
    const int act = ui::menu(w.name, {"Navigate here", "Show on map", "Rename", "Delete"});
    if (act == 0) {
        nd.clearRoute();
        nd.setTarget(w);
    } else if (act == 1) {
        app::requestCenter(w.lat, w.lon);
    } else if (act == 2) {
        String name = w.name;
        if (ui::textInput("Rename", name) && name.length()) {
            strlcpy(nd.waypoints[idx].name, name.c_str(), sizeof(Waypoint::name));
            nd.saveWaypoints();
        }
    } else if (act == 3 && ui::confirm("Delete waypoint", w.name)) {
        nd.removeWaypoint(idx);
    }
}

void routesMenu() {
    auto &nd = app::navdata;
    std::vector<String> items, paths;
    if (nd.hasRoute()) {
        items.push_back("Reverse current route");
        paths.push_back("");
        items.push_back("Clear current route");
        paths.push_back("");
    }
    for (const char *dir : {storage::kRoutesDir, storage::kTracksDir}) {
        File d = SD.open(dir);
        if (!d) continue;
        for (File f = d.openNextFile(); f; f = d.openNextFile()) {
            String name = f.name();
            if (!f.isDirectory() && name.endsWith(".gpx") && paths.size() < 80) {
                paths.push_back(String(dir) + "/" + name);
                items.push_back(String(dir == storage::kTracksDir ? "track: " : "route: ") + name);
            }
            f.close();
        }
        d.close();
    }
    if (items.empty()) {
        ui::message("Routes", "Copy .gpx files to /gpsnav/routes on the SD card, or record a track.");
        return;
    }
    const int sel = ui::menu("Routes & tracks", items);
    if (sel < 0) return;
    if (!paths[sel].length()) {
        if (items[sel].startsWith("Reverse")) nd.reverseRoute();
        else nd.clearRoute();
        return;
    }
    ui::message("Loading", paths[sel], false);
    String err;
    if (!nd.loadRoute(paths[sel], err)) {
        ui::message("Route", err);
        return;
    }
    nd.clearTarget();
    double length = 0;
    for (size_t i = 1; i < nd.route.size(); ++i) length += nav::haversineM(nd.route[i - 1], nd.route[i]);
    const int act = ui::menu((String(nd.route.size()) + " pts, " + fmtDist(length)).c_str(),
                             {"Follow from start", "Follow reversed (backtrack)", "Just show it"});
    if (act == 1) nd.reverseRoute();
    if (act < 0) nd.clearRoute();
    else app::requestCenter(nd.route.front().lat, nd.route.front().lon);
}

void gotoCoordinates() {
    String s;
    nav::LatLon c;
    if (!ui::textInput("Go to", s, false, "lat, lon in decimal degrees")) return;
    if (!parseLatLon(s, c)) {
        ui::message("Go to", "Could not parse '" + s + "'.");
        return;
    }
    Waypoint w;
    strlcpy(w.name, "Coords", sizeof(w.name));
    w.lat = c.lat;
    w.lon = c.lon;
    app::navdata.clearRoute();
    app::navdata.setTarget(w);
    if (ui::confirm("Go to", "Also save as a waypoint?")) {
        String name = app::navdata.nextWaypointName();
        if (ui::textInput("Waypoint name", name) && name.length()) app::navdata.addWaypoint(name, c.lat, c.lon);
    }
}

void toggleRecording() {
    auto &nd = app::navdata;
    if (nd.recording()) {
        nd.stopTrack();
        ui::message("Track saved", nd.trackPath() + "\n" + String(nd.trackPoints()) + " points");
        return;
    }
    char stamp[24];
    if (!app::gps.fileStamp(stamp, sizeof stamp)) snprintf(stamp, sizeof stamp, "track-%lu", millis() / 1000);
    String err;
    if (!nd.startTrack(stamp, err)) ui::message("Track", err);
}

void clearNavigation() {
    if (!app::navdata.hasTarget() && !app::navdata.hasRoute()) return;
    if (!ui::confirm("Clear navigation", "Stop guiding to the current target / route?")) return;
    app::navdata.clearTarget();
    app::navdata.clearRoute();
}

void gpsStatus() {
    while (true) {
        app::service();
        Key k = readKey();
        if (k.back || k.enter || k.del) return;
        const GpsFix f = app::gps.fix();
        canvas.fillScreen(ui::kBg);
        ui::header("GPS status", f.valid ? "FIX" : "NO FIX");
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(ui::kFg);
        char line[64], t[32];
        int y = 18;
        auto row = [&](const char *s) {
            canvas.drawString(s, 4, y);
            y += 10;
        };
        if (!app::gps.receivingData()) {
            canvas.setTextColor(ui::kBad);
            row("No NMEA data from the Cap LoRa-1262.");
            row("Check the cap is seated; power cycle.");
            canvas.setTextColor(ui::kFg);
        }
        snprintf(line, sizeof line, "Lat  %.6f", f.lat);
        row(line);
        snprintf(line, sizeof line, "Lon  %.6f", f.lon);
        row(line);
        snprintf(line, sizeof line, "Alt  %.1f m   HDOP %.1f", f.altM, f.hdop);
        row(line);
        snprintf(line, sizeof line, "Speed %.1f km/h  Course %.0f", f.speedKmh, f.courseDeg);
        row(line);
        snprintf(line, sizeof line, "Satellites %lu   fix age %lus", (unsigned long)f.sats,
                 (unsigned long)(f.ageMs > 999000 ? 999 : f.ageMs / 1000));
        row(line);
        if (app::gps.isoTime(t, sizeof t)) {
            snprintf(line, sizeof line, "UTC  %s", t);
            row(line);
        }
        snprintf(line, sizeof line, "NMEA chars %lu  bad %lu", (unsigned long)app::gps.charsProcessed(),
                 (unsigned long)app::gps.failedChecksums());
        canvas.setTextColor(ui::kDim);
        row(line);
        ui::footer("Cold start can take ~30s outdoors.  ` back");
        ui::present();
        delay(200);
    }
}

void settingsMenu() {
    auto &cfg = app::config;
    int sel = 0;
    while (true) {
        std::vector<String> items = {
            "WiFi: " + (cfg.wifiSsid.length() ? cfg.wifiSsid : String("(not set)")),
            String("Provider: ") + nav::providerId(cfg.provider),
            "Map style: " + cfg.style,
            "API key: " + keyHint(cfg.apiKey),
            "Test provider / key",
            String("Units: ") + (cfg.imperial ? "imperial" : "metric"),
            "Brightness: " + String(cfg.brightness * 100 / 255) + "%",
        };
        sel = ui::menu("Settings", items, sel);
        switch (sel) {
            case 0:
                net::wifiSetup();
                net::wifiOff();
                break;
            case 1: {
                const int p = ui::menu("Tile provider", {"MapTiler (cloud.maptiler.com)", "Thunderforest (thunderforest.com)"},
                                       int(cfg.provider));
                if (p >= 0 && nav::Provider(p) != cfg.provider) {
                    cfg.provider = nav::Provider(p);
                    cfg.style = nav::defaultStyle(cfg.provider);
                    cfg.apiKey = "";
                    ui::message("Provider changed", "Enter the API key for this provider next. Tiles are cached "
                                                    "separately per provider and style.");
                }
                break;
            }
            case 2: {
                String s = cfg.style;
                const char *hint = cfg.provider == nav::Provider::MapTiler
                                       ? "e.g. streets-v2, outdoor-v2, topo-v2, satellite, basic-v2"
                                       : "e.g. outdoors, cycle, transport, landscape, atlas";
                if (ui::textInput("Map style", s, false, hint)) {
                    s.trim();
                    if (nav::isSafeToken(s.c_str())) cfg.style = s;
                    else ui::message("Map style", "Invalid style name.");
                }
                break;
            }
            case 3: {
                String k = cfg.apiKey;
                if (ui::textInput("API key", k)) {
                    k.trim();
                    if (!k.length() || nav::isSafeToken(k.c_str())) cfg.apiKey = k;
                    else ui::message("API key", "That key contains unexpected characters.");
                }
                break;
            }
            case 4: ui::message("Provider test", net::testProvider()); break;
            case 5: cfg.imperial = !cfg.imperial; break;
            case 6:
                cfg.brightness = cfg.brightness >= 240 ? 40 : min(255, cfg.brightness + 50);
                M5Cardputer.Display.setBrightness(cfg.brightness);
                break;
            default: return;
        }
        cfg.save();
    }
}

void storageInfo() {
    const uint64_t total = storage::totalBytes(), free = storage::freeBytes();
    ui::message("Storage", "SD card: " + fmtBytes(total) + " total, " + fmtBytes(free) +
                               " free.\n\nTiles: /gpsnav/tiles\nRoutes (GPX in): /gpsnav/routes\nTracks: "
                               "/gpsnav/tracks\nWaypoints: /gpsnav/waypoints.gpx\nSettings: /gpsnav/config.json");
}

void help() {
    ui::message("Keys",
                ";/.,/ (arrows) pan    = / - zoom\n"
                "space follow GPS     Enter menu\n"
                "d download maps      g waypoints\n"
                "w save waypoint      r record track\n"
                "i GPS status         x clear nav\n"
                "` back in menus      fn+` cancel text\n"
                "Map: maptiler.com/copyright,\n"
                "thunderforest.com; (c) OpenStreetMap");
}

}  // namespace screens
