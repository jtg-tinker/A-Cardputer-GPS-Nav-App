#include "mapview.h"

#include <math.h>
#include <tilesource.h>

#include "app.h"
#include "input.h"
#include "routing.h"
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
constexpr uint16_t kRouteColor = 0x001F;
constexpr uint16_t kStartColor = 0x0600;
constexpr uint16_t kEndColor = 0xF800;
constexpr uint16_t kTrackColor = 0xFC00;
constexpr uint16_t kTargetColor = 0x07FF;
constexpr uint16_t kPosColor = 0x041F;
constexpr uint16_t kStreetColor = 0x07E0;
constexpr uint16_t kStreetHlColor = 0xFFE0;
constexpr int kStreetOverlayZoom = 16;
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
    upscaled_ = true;
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
    upscaled_ = false;
    for (int ty = ty0; ty <= ty1; ++ty)
        for (int tx = tx0; tx <= tx1; ++tx)
            drawTile(zoom_, tx, ty, int(tx * nav::kTileSize - originX_), int(ty * nav::kTileSize - originY_));
    if (zoom_ >= kStreetOverlayZoom) drawStreetLines();
    baseDirty_ = false;
}

namespace {

// Draws street lines given in z14 tile units onto a canvas; (ox, oy) is the tile's top-left on screen.
struct LineCtx {
    M5Canvas *dst;
    double ox, oy, tilePx;
    uint16_t color;
    float width;
};

void drawStreetLine(void *p, const char *, const int16_t *xy, size_t n, uint16_t extent) {
    const auto &c = *static_cast<LineCtx *>(p);
    const double k = c.tilePx / extent;
    int px = 0, py = 0;
    for (size_t i = 0; i < n; ++i) {
        const int x = int(lround(c.ox + xy[2 * i] * k)), y = int(lround(c.oy + xy[2 * i + 1] * k));
        if (i > 0 && (x != px || y != py)) {
            const bool visible = !((x < 0 && px < 0) || (y < 0 && py < 0) || (x >= app::kW && px >= app::kW) ||
                                   (y >= app::kH && py >= app::kH));
            if (visible) c.dst->drawWideLine(px, py, x, y, c.width, c.color);
        }
        px = x;
        py = y;
    }
}

}  // namespace

void MapView::drawStreetLines() {
    const double tilePx = nav::kTileSize * double(1 << (zoom_ - nav::kStreetZoom));
    const int tx0 = int(floor(originX_ / tilePx)), tx1 = int(floor((originX_ + app::kW - 1) / tilePx));
    const int ty0 = int(floor(originY_ / tilePx)), ty1 = int(floor((originY_ + app::kH - 1) / tilePx));
    for (int ty = ty0; ty <= ty1; ++ty) {
        for (int tx = tx0; tx <= tx1; ++tx) {
            LineCtx c{&base, tx * tilePx - originX_, ty * tilePx - originY_, tilePx, kStreetColor, 1.5f};
            streetdata::forEachLine(tx, ty, &drawStreetLine, &c);
        }
    }
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
    const GpsFix sfix = app::gps.fix();
    const nav::LatLon here = (follow_ && sfix.valid) ? nav::LatLon{sfix.lat, sfix.lon} : center_;
    streetState_ = streetdata::lookup(here, street_, sizeof street_);
    if (streetState_ > 0 && zoom_ >= kStreetOverlayZoom) {
        const auto &h = streetdata::highlight();
        const double tilePx = nav::kTileSize * double(1 << (zoom_ - nav::kStreetZoom));
        LineCtx c{&canvas, h.tx * tilePx - originX_, h.ty * tilePx - originY_, tilePx, kStreetHlColor, 3.0f};
        for (const auto &l : h.lines) drawStreetLine(&c, nullptr, l.data(), l.size() / 2, h.extent);
    }

    auto &nd = app::navdata;
    if (nd.hasRoute()) drawPolyline(*this, nd.route, kRouteColor, 2.5f, &MapView::toScreen);
    if (!nd.breadcrumb.empty()) drawPolyline(*this, nd.breadcrumb, kTrackColor, 1.0f, &MapView::toScreen);

    canvas.setFont(&fonts::Font0);
    auto endpoint = [&](bool show, const nav::LatLon &p, uint16_t color, const char *label) {
        int x, y;
        if (!show || !toScreen(p, x, y) || x < -8 || x > app::kW + 8 || y < -8 || y > app::kH + 8) return;
        canvas.fillCircle(x, y, 6, color);
        canvas.drawCircle(x, y, 6, 0xFFFF);
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(0xFFFF);
        canvas.drawString(label, x + 1, y + 1);
    };
    endpoint(nd.hasRouteStart, nd.routeStart, kStartColor, "S");
    endpoint(nd.hasRouteEnd, nd.routeEnd, kEndColor, "E");
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
    const int streetState = streetState_;
    const char *street = street_;
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
    snprintf(right, sizeof right, "%sZ%d%s%s %s", app::navdata.recording() ? "REC " : "", zoom_, upscaled_ ? "~" : "",
             follow_ ? " F" : "",
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
    if (noteUntil_ && int32_t(millis() - noteUntil_) < 0) {
        canvas.setTextColor(ui::kAccent);
        canvas.drawString(note_.c_str(), 2, by + 2);
    } else if (g.active) {
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
    else if (k.ch == 's' || k.ch == 'e') setRoutePoint(k.ch == 's');
    else return;
    lastFrame_ = 0;  // redraw immediately
    if (!k.up && !k.down && !k.left && !k.right && k.ch != '=' && k.ch != '+' && k.ch != '-' && k.ch != '_')
        baseDirty_ = true;  // returning from a full-screen dialog
}

void MapView::showNote(const String &s, uint32_t ms) {
    note_ = s;
    noteUntil_ = millis() + ms;
    if (!noteUntil_) noteUntil_ = 1;
}

// s / e: route start or end at the crosshair (the GPS position while following it).
void MapView::setRoutePoint(bool start) {
    auto &nd = app::navdata;
    if (start) {
        nd.routeStart = center_;
        nd.hasRouteStart = true;
    } else {
        nd.routeEnd = center_;
        nd.hasRouteEnd = true;
    }
    if (!nd.hasRouteStart || !nd.hasRouteEnd) {
        showNote(start ? "Start set. Press e at the end point." : "End set. Press s at the start point.");
        return;
    }
    nd.clearTarget();
    if (nd.routeFromMap) nd.clearRoute();
    releaseBase();  // the route search needs the memory
    String msg;
    if (routing::compute(nd.routeStart, nd.routeEnd, msg)) showNote(msg, 8000);
    else ui::message("No route", msg);
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
