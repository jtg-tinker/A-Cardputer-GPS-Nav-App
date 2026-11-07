#include "mapview.h"

#include <math.h>
#include <tilesource.h>

#include "app.h"
#include "input.h"
#include "screens.h"
#include "storage.h"
#include "streetdata.h"
#include "ui.h"

using app::canvas;

MapView mapView;

namespace {
M5Canvas base;
constexpr int kPanStep = 48;
constexpr uint16_t kNoTile = 0xDEFB;
constexpr uint16_t kRouteColor = 0xF81F;
constexpr uint16_t kTrackColor = 0xFC00;
constexpr uint16_t kTargetColor = 0x07FF;
constexpr uint16_t kPosColor = 0x041F;
}  // namespace

void MapView::begin(double lat, double lon, int zoom) {
    center_ = {lat, lon};
    zoom_ = constrain(zoom, nav::kMinZoom, nav::kMaxZoom);
    base.setColorDepth(16);
    baseDirty_ = true;
}

void MapView::releaseBase() {
    base.deleteSprite();
    baseValid_ = false;
    baseDirty_ = true;
}

void MapView::persistView() {
    app::config.lastLat = center_.lat;
    app::config.lastLon = center_.lon;
    app::config.lastZoom = zoom_;
    app::config.save();
    viewChanged_ = false;
    lastPersist_ = millis();
}

void MapView::setZoom(int z) {
    z = constrain(z, nav::kMinZoom, nav::kMaxZoom);
    if (z == zoom_) return;
    zoom_ = z;
    baseDirty_ = viewChanged_ = true;
}

void MapView::pan(int dx, int dy) {
    nav::WorldPx p = nav::latLonToWorldPx(center_.lat, center_.lon, zoom_);
    center_ = nav::worldPxToLatLon(p.x + dx, p.y + dy, zoom_);
    follow_ = false;
    baseDirty_ = viewChanged_ = true;
}

bool MapView::toScreen(const nav::LatLon &ll, int &sx, int &sy) const {
    nav::WorldPx p = nav::latLonToWorldPx(ll.lat, ll.lon, zoom_);
    const double x = p.x - originX_, y = p.y - originY_;
    if (fabs(x) > 4000 || fabs(y) > 4000) return false;
    sx = int(lround(x));
    sy = int(lround(y));
    return true;
}

void MapView::drawTile(int z, int x, int y, int dx, int dy) {
    char path[112];
    const auto &cfg = app::config;
    const int n = 1 << z;
    if (y < 0 || y >= n) {
        base.fillRect(dx, dy, nav::kTileSize, nav::kTileSize, 0x0000);
        return;
    }
    x = ((x % n) + n) % n;
    if (nav::buildTilePath(storage::kTilesRoot, cfg.provider, cfg.style.c_str(), z, x, y, path, sizeof path) &&
        base.drawJpgFile(SD, path, dx, dy)) {
        return;
    }
    // Missing tile: upscale the closest downloaded ancestor.
    base.setClipRect(dx, dy, nav::kTileSize, nav::kTileSize);
    bool drawn = false;
    for (int k = 1; k <= 5 && z - k >= 0 && !drawn; ++k) {
        const int scale = 1 << k;
        const int sub = nav::kTileSize >> k;
        const int sx = (x & (scale - 1)) * sub;
        const int sy = (y & (scale - 1)) * sub;
        if (!nav::buildTilePath(storage::kTilesRoot, cfg.provider, cfg.style.c_str(), z - k, x >> k, y >> k, path,
                                sizeof path))
            break;
        drawn = base.drawJpgFile(SD, path, dx - sx * scale, dy - sy * scale, 0, 0, 0, 0, float(scale), float(scale));
    }
    base.clearClipRect();
    if (!drawn) {
        base.fillRect(dx, dy, nav::kTileSize, nav::kTileSize, kNoTile);
        base.drawRect(dx, dy, nav::kTileSize, nav::kTileSize, 0xBDF7);
    }
}

void MapView::renderBase() {
    if (!baseValid_) {
        baseValid_ = base.createSprite(app::kW, app::kH) != nullptr;
        if (!baseValid_) return;
    }
    nav::WorldPx c = nav::latLonToWorldPx(center_.lat, center_.lon, zoom_);
    originX_ = floor(c.x) - app::kW / 2;
    originY_ = floor(c.y) - app::kH / 2;
    const int tx0 = int(floor(originX_ / nav::kTileSize));
    const int ty0 = int(floor(originY_ / nav::kTileSize));
    const int tx1 = int(floor((originX_ + app::kW - 1) / nav::kTileSize));
    const int ty1 = int(floor((originY_ + app::kH - 1) / nav::kTileSize));
    for (int ty = ty0; ty <= ty1; ++ty)
        for (int tx = tx0; tx <= tx1; ++tx)
            drawTile(zoom_, tx, ty, int(tx * nav::kTileSize - originX_), int(ty * nav::kTileSize - originY_));
    baseDirty_ = false;
}

static void drawPolyline(MapView &mv, const std::vector<nav::LatLon> &pts, uint16_t color, float width,
                         bool (MapView::*proj)(const nav::LatLon &, int &, int &) const) {
    int px = 0, py = 0;
    bool havePrev = false;
    for (const auto &p : pts) {
        int x, y;
        if (!(mv.*proj)(p, x, y)) { havePrev = false; continue; }
        if (havePrev && (x != px || y != py)) {
            const bool visible = !((x < 0 && px < 0) || (y < 0 && py < 0) || (x >= app::kW && px >= app::kW) ||
                                   (y >= app::kH && py >= app::kH));
            if (visible) canvas.drawWideLine(px, py, x, y, width, color);
        }
        px = x;
        py = y;
        havePrev = true;
    }
}

void MapView::drawOverlays() {
    auto &nd = app::navdata;
    if (nd.hasRoute()) drawPolyline(*this, nd.route, kRouteColor, 1.6f, &MapView::toScreen);
    if (!nd.breadcrumb.empty()) drawPolyline(*this, nd.breadcrumb, kTrackColor, 1.0f, &MapView::toScreen);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    for (const auto &w : nd.waypoints) {
        int x, y;
        if (!toScreen({w.lat, w.lon}, x, y) || x < -40 || x > app::kW || y < 0 || y > app::kH) continue;
        canvas.fillRect(x - 2, y - 2, 5, 5, 0xF800);
        canvas.drawRect(x - 3, y - 3, 7, 7, 0xFFFF);
        if (zoom_ >= 12) {
            canvas.setTextColor(0x0000);
            canvas.drawString(w.name, x + 5, y + 1);
            canvas.setTextColor(0xFFFF);
            canvas.drawString(w.name, x + 4, y);
        }
    }
    canvas.setTextDatum(top_left);

    const GpsFix fix = app::gps.fix();
    const Guidance g = nd.guidance(fix);
    int gx = 0, gy = 0;
    const bool havePos = fix.valid && toScreen({fix.lat, fix.lon}, gx, gy);
    if (g.active) {
        int tx, ty;
        if (toScreen(g.next, tx, ty)) {
            if (havePos) canvas.drawWideLine(gx, gy, tx, ty, 1.2f, kTargetColor);
            canvas.fillCircle(tx, ty, 4, kTargetColor);
            canvas.drawCircle(tx, ty, 5, 0x0000);
        }
    } else if (nd.hasTarget()) {
        int tx, ty;
        if (toScreen({nd.target().lat, nd.target().lon}, tx, ty)) {
            canvas.fillCircle(tx, ty, 4, kTargetColor);
            canvas.drawCircle(tx, ty, 5, 0x0000);
        }
    }

    if (havePos) {
        if (fix.courseValid) {
            const double a = fix.courseDeg * DEG_TO_RAD;
            const int x0 = gx + int(9 * sin(a)), y0 = gy - int(9 * cos(a));
            const int x1 = gx + int(6 * sin(a + 2.5)), y1 = gy - int(6 * cos(a + 2.5));
            const int x2 = gx + int(6 * sin(a - 2.5)), y2 = gy - int(6 * cos(a - 2.5));
            canvas.fillTriangle(x0, y0, x1, y1, x2, y2, kPosColor);
            canvas.drawTriangle(x0, y0, x1, y1, x2, y2, 0xFFFF);
        } else {
            canvas.fillCircle(gx, gy, 5, kPosColor);
            canvas.drawCircle(gx, gy, 5, 0xFFFF);
            canvas.drawCircle(gx, gy, 6, 0x0000);
        }
    }
    if (!follow_ || !fix.valid) {
        const int cx = app::kW / 2, cy = app::kH / 2;
        canvas.drawFastHLine(cx - 6, cy, 13, 0x0000);
        canvas.drawFastVLine(cx, cy - 6, 13, 0x0000);
    }

    // Scale bar
    const double mpp = cos(center_.lat * DEG_TO_RAD) * 2 * M_PI * nav::kEarthRadiusM / nav::worldSizePx(zoom_);
    static const double kSteps[] = {5,    10,   20,   50,    100,   200,   500,    1000,   2000,
                                    5000, 1e4,  2e4,  5e4,   1e5,   2e5,   5e5,    1e6,    2e6};
    double step = kSteps[0];
    for (double s : kSteps)
        if (s / mpp <= 60) step = s;
    const int len = int(step / mpp);
    char buf[24];
    nav::formatDistance(step, app::config.imperial, buf, sizeof buf);
    const int sy = app::kH - 16;
    canvas.fillRect(3, sy - 9, max(len, 30) + 2, 12, 0xFFFF);
    canvas.drawFastHLine(4, sy, len, 0x0000);
    canvas.drawFastVLine(4, sy - 3, 4, 0x0000);
    canvas.drawFastVLine(4 + len, sy - 3, 4, 0x0000);
    canvas.setTextColor(0x0000);
    canvas.drawString(buf, 6, sy - 8);

    if (g.active && g.arrived && !arrivedShown_) {
        arrivedShown_ = true;
        M5Cardputer.Speaker.tone(2200, 150);
    }
    if (!g.arrived) arrivedShown_ = false;
}

void MapView::drawStatus() {
    const GpsFix fix = app::gps.fix();
    char left[48], right[40];
    // Top bar: GNSS state, speed, recording, battery; second line street name when downloaded
    char street[64];
    const nav::LatLon here = (follow_ && fix.valid) ? nav::LatLon{fix.lat, fix.lon} : center_;
    const int streetState = streetdata::lookup(here, street, sizeof street);
    const int topBarH = streetState >= 0 ? 32 : 20;
    canvas.fillRect(0, 0, app::kW, topBarH, 0x0000);
    uint16_t c = ui::kGood;
    if (!app::gps.receivingData()) {
        snprintf(left, sizeof left, "NO GNSS DATA");
        c = ui::kBad;
    } else if (!fix.valid) {
        snprintf(left, sizeof left, "Searching sats:%lu", (unsigned long)fix.sats);
        c = ui::kWarn;
    } else {
        char spd[16];
        nav::formatSpeed(fix.speedKmh, app::config.imperial, spd, sizeof spd);
        snprintf(left, sizeof left, "Sat:%lu %s", (unsigned long)fix.sats, spd);
    }
    const int bat = M5Cardputer.Power.getBatteryLevel();
    snprintf(right, sizeof right, "%sZ%d%s %s", app::navdata.recording() ? "REC " : "", zoom_, follow_ ? " F" : "",
             bat >= 0 ? (String(bat) + "%").c_str() : "");
    canvas.setTextSize(1);
    const uint16_t rightColor = app::navdata.recording() ? ui::kBad : ui::kFg;
    if (streetState >= 0) {
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(c);
        canvas.setTextDatum(top_left);
        canvas.drawString(left, 2, 1);
        canvas.setTextColor(rightColor);
        canvas.setTextDatum(top_right);
        canvas.drawString(right, app::kW - 2, 1);
        canvas.setFont(&fonts::Font2);
        String label = streetState > 0 ? String(street) : String("-");
        if (canvas.textWidth(label.c_str()) > app::kW - 4) {
            while (label.length() > 1 && canvas.textWidth((label + "...").c_str()) > app::kW - 4)
                label.remove(label.length() - 1);
            label += "...";
        }
        canvas.setTextColor(streetState > 0 ? ui::kFg : ui::kDim);
        canvas.setTextDatum(middle_center);
        canvas.drawString(label.c_str(), app::kW / 2, 21);
    } else {
        canvas.setFont(&fonts::Font2);
        if (canvas.textWidth(left) + canvas.textWidth(right) + 8 > app::kW - 4) canvas.setFont(&fonts::Font0);
        canvas.setTextColor(c);
        canvas.setTextDatum(middle_left);
        canvas.drawString(left, 2, topBarH / 2);
        canvas.setTextColor(rightColor);
        canvas.setTextDatum(middle_right);
        canvas.drawString(right, app::kW - 2, topBarH / 2);
    }
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);

    // Bottom bar: guidance or coordinates
    const int by = app::kH - 11;
    canvas.fillRect(0, by, app::kW, 11, 0x0000);
    const Guidance g = app::navdata.guidance(fix);
    if (g.active) {
        char d[16], rem[16];
        nav::formatDistance(g.distNextM, app::config.imperial, d, sizeof d);
        nav::formatDistance(g.remainingM, app::config.imperial, rem, sizeof rem);
        if (g.arrived) snprintf(left, sizeof left, "ARRIVED %.20s", g.name);
        else if (g.route) snprintf(left, sizeof left, "%s %s%s  left %s", d, nav::compassPoint(g.bearingDeg),
                                   g.offRouteM > 60 ? " OFF" : "", rem);
        else snprintf(left, sizeof left, "%.12s %s %s %.0f", g.name, d, nav::compassPoint(g.bearingDeg), g.bearingDeg);
        canvas.setTextColor(g.arrived ? ui::kGood : ui::kAccent);
        canvas.drawString(left, 2, by + 2);
        // Relative arrow: up = direction of travel when moving, otherwise north
        const double rel = (g.bearingDeg - (fix.courseValid ? fix.courseDeg : 0)) * DEG_TO_RAD;
        const int ax = app::kW - 8, ay = by + 5;
        canvas.fillTriangle(ax + int(5 * sin(rel)), ay - int(5 * cos(rel)), ax + int(4 * sin(rel + 2.4)),
                            ay - int(4 * cos(rel + 2.4)), ax + int(4 * sin(rel - 2.4)), ay - int(4 * cos(rel - 2.4)),
                            ui::kAccent);
    } else {
        const nav::LatLon p = (follow_ && fix.valid) ? nav::LatLon{fix.lat, fix.lon} : center_;
        snprintf(left, sizeof left, "%.5f %.5f", p.lat, p.lon);
        canvas.setTextColor(ui::kFg);
        canvas.drawString(left, 2, by + 2);
        canvas.setTextColor(ui::kDim);
        canvas.setTextDatum(top_right);
        canvas.drawString("Enter=menu h=help", app::kW - 2, by + 2);
        canvas.setTextDatum(top_left);
    }
}

void MapView::handleKey() {
    Key k = readKey();
    if (!k.any) return;
    if (k.up) pan(0, -kPanStep);
    else if (k.down) pan(0, kPanStep);
    else if (k.left) pan(-kPanStep, 0);
    else if (k.right) pan(kPanStep, 0);
    else if (k.ch == '=' || k.ch == '+') setZoom(zoom_ + 1);
    else if (k.ch == '-' || k.ch == '_') setZoom(zoom_ - 1);
    else if (k.ch == ' ' || k.ch == 'f') {
        follow_ = !follow_;
        baseDirty_ = true;
    } else if (k.enter || k.ch == 'm') screens::mainMenu(center_);
    else if (k.ch == 'h' || k.ch == '?') screens::help();
    else if (k.ch == 'i') screens::gpsStatus();
    else if (k.ch == 'd') screens::downloadMenu(center_, zoom_);
    else if (k.ch == 'g') screens::waypointsMenu(center_);
    else if (k.ch == 'w') screens::quickWaypoint(center_);
    else if (k.ch == 'r') screens::toggleRecording();
    else if (k.ch == 'x') screens::clearNavigation();
    else return;
    lastFrame_ = 0;  // redraw immediately
    if (!k.up && !k.down && !k.left && !k.right && k.ch != '=' && k.ch != '+' && k.ch != '-' && k.ch != '_')
        baseDirty_ = true;  // returning from a full-screen dialog
}

void MapView::tick() {
    handleKey();

    double lat, lon;
    int z;
    if (app::takeCenterRequest(lat, lon, z)) {
        center_ = {lat, lon};
        if (z > 0) zoom_ = constrain(z, nav::kMinZoom, nav::kMaxZoom);
        follow_ = false;
        baseDirty_ = viewChanged_ = true;
    }

    const GpsFix fix = app::gps.fix();
    if (follow_ && fix.valid) {
        nav::WorldPx a = nav::latLonToWorldPx(center_.lat, center_.lon, zoom_);
        nav::WorldPx b = nav::latLonToWorldPx(fix.lat, fix.lon, zoom_);
        if (fabs(a.x - b.x) >= 2 || fabs(a.y - b.y) >= 2) {
            center_ = {fix.lat, fix.lon};
            baseDirty_ = viewChanged_ = true;
        }
    }

    const uint32_t now = millis();
    if (!baseDirty_ && now - lastFrame_ < 500) return;
    lastFrame_ = now;
    if (baseDirty_ || !baseValid_) renderBase();
    if (baseValid_) base.pushSprite(&canvas, 0, 0);
    else canvas.fillScreen(kNoTile);
    drawOverlays();
    drawStatus();
    ui::present();

    if (viewChanged_ && now - lastPersist_ > 60000) persistView();
}
