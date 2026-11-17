#include "navdata.h"

#include <SD.h>
#include <gpx.h>

#include "storage.h"

namespace {
constexpr size_t kMaxWaypoints = 200;
constexpr size_t kMaxRoutePoints = 3000;
constexpr size_t kMaxBreadcrumb = 1500;
constexpr double kArriveM = 25.0;
constexpr double kLookAheadM = 20.0;

void decodeEntities(char *s) {
    static const struct { const char *ent; char ch; } kEnts[] = {
        {"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}};
    char *out = s;
    for (char *in = s; *in;) {
        bool matched = false;
        if (*in == '&') {
            for (const auto &e : kEnts) {
                const size_t n = strlen(e.ent);
                if (strncmp(in, e.ent, n) == 0) {
                    *out++ = e.ch;
                    in += n;
                    matched = true;
                    break;
                }
            }
        }
        if (!matched) *out++ = *in++;
    }
    *out = 0;
}

template <typename Fn>
bool parseGpxFile(const String &path, Fn &&fn) {
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    auto cb = [](void *ctx, nav::GpxPointKind kind, double lat, double lon, const char *name) {
        (*static_cast<Fn *>(ctx))(kind, lat, lon, name);
    };
    nav::GpxParser parser(cb, &fn);
    char buf[512];
    while (f.available()) {
        const int n = f.read(reinterpret_cast<uint8_t *>(buf), sizeof(buf));
        if (n <= 0) break;
        parser.feed(buf, n);
    }
    f.close();
    return true;
}
}  // namespace

bool NavData::loadWaypoints() {
    waypoints.clear();
    return parseGpxFile(storage::kWaypointsPath, [this](nav::GpxPointKind kind, double lat, double lon, const char *name) {
        if (kind != nav::GpxPointKind::Waypoint || waypoints.size() >= kMaxWaypoints) return;
        Waypoint w;
        strlcpy(w.name, name, sizeof(w.name));
        decodeEntities(w.name);
        w.lat = lat;
        w.lon = lon;
        waypoints.push_back(w);
    });
}

bool NavData::saveWaypoints() const {
    const String tmp = String(storage::kWaypointsPath) + ".tmp";
    File f = SD.open(tmp, FILE_WRITE);
    if (!f) return false;
    f.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            "<gpx version=\"1.1\" creator=\"cardputer-gps-nav\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n");
    char line[160];
    for (const auto &w : waypoints) {
        nav::gpxWaypoint(line, sizeof(line), w.lat, w.lon, w.name);
        f.print(line);
    }
    f.print("</gpx>\n");
    f.close();
    SD.remove(storage::kWaypointsPath);
    return SD.rename(tmp, storage::kWaypointsPath);
}

bool NavData::addWaypoint(const String &name, double lat, double lon) {
    if (waypoints.size() >= kMaxWaypoints) return false;
    Waypoint w;
    strlcpy(w.name, name.c_str(), sizeof(w.name));
    w.lat = lat;
    w.lon = lon;
    waypoints.push_back(w);
    return saveWaypoints();
}

void NavData::removeWaypoint(size_t idx) {
    if (idx >= waypoints.size()) return;
    waypoints.erase(waypoints.begin() + idx);
    saveWaypoints();
}

String NavData::nextWaypointName() const {
    for (int n = waypoints.size() + 1;; ++n) {
        char buf[12];
        snprintf(buf, sizeof buf, "WP%03d", n);
        bool used = false;
        for (const auto &w : waypoints) used |= strcmp(w.name, buf) == 0;
        if (!used) return buf;
    }
}

void NavData::setTarget(const Waypoint &w) {
    target_ = w;
    hasTarget_ = true;
}

void NavData::clearTarget() { hasTarget_ = false; }

bool NavData::loadRoute(const String &path, String &err) {
    std::vector<nav::LatLon> pts, wpts;
    size_t stride = 1, seen = 0;
    const bool ok = parseGpxFile(path, [&](nav::GpxPointKind kind, double lat, double lon, const char *) {
        if (kind == nav::GpxPointKind::Waypoint) {
            if (wpts.size() < kMaxRoutePoints) wpts.push_back({lat, lon});
            return;
        }
        if (seen++ % stride != 0) return;
        pts.push_back({lat, lon});
        if (pts.size() >= kMaxRoutePoints) {  // thin to half and keep going with a coarser stride
            size_t j = 0;
            for (size_t i = 0; i < pts.size(); i += 2) pts[j++] = pts[i];
            pts.resize(j);
            stride *= 2;
        }
    });
    if (!ok) {
        err = "Could not open file";
        return false;
    }
    if (pts.size() < 2) pts.swap(wpts);
    if (pts.size() < 2) {
        err = "No track, route or waypoint list found";
        return false;
    }
    route.swap(pts);
    route.shrink_to_fit();
    routeName = path.substring(path.lastIndexOf('/') + 1);
    routeFromMap = false;
    clearRoutePoints();
    rebuildRouteDistances();
    return true;
}

void NavData::setRoute(std::vector<nav::LatLon> &&pts, const String &name) {
    if (pts.size() > kMaxRoutePoints) {
        const size_t stride = (pts.size() + kMaxRoutePoints - 1) / kMaxRoutePoints;
        size_t j = 0;
        for (size_t i = 0; i < pts.size(); i += stride) pts[j++] = pts[i];
        pts[j - 1] = pts.back();
        pts.resize(j);
    }
    route = std::move(pts);
    route.shrink_to_fit();
    routeName = name;
    routeFromMap = true;
    rebuildRouteDistances();
}

void NavData::reverseRoute() {
    std::reverse(route.begin(), route.end());
    rebuildRouteDistances();
}

void NavData::clearRoute() {
    route.clear();
    route.shrink_to_fit();
    routeCum_.clear();
    routeCum_.shrink_to_fit();
    routeName = "";
    routeFromMap = false;
}

void NavData::rebuildRouteDistances() {
    routeCum_.assign(route.size(), 0.0f);
    for (size_t i = 1; i < route.size(); ++i) routeCum_[i] = routeCum_[i - 1] + nav::haversineM(route[i - 1], route[i]);
    routeIdx_ = 0;
    routeIdxValid_ = false;
}

Guidance NavData::guidance(const GpsFix &fix) {
    Guidance g;
    if (!fix.valid) return g;
    const nav::LatLon pos{fix.lat, fix.lon};
    if (hasRoute()) {
        // Track progress near the previous position, searching the whole route on the first fix.
        size_t lo = routeIdxValid_ ? (routeIdx_ > 10 ? routeIdx_ - 10 : 0) : 0;
        size_t hi = routeIdxValid_ ? min(route.size(), routeIdx_ + 200) : route.size();
        double best = 1e18;
        size_t bestIdx = routeIdx_;
        for (size_t i = lo; i < hi; ++i) {
            const double d = nav::haversineM(pos, route[i]);
            if (d < best) { best = d; bestIdx = i; }
        }
        routeIdx_ = bestIdx;
        routeIdxValid_ = true;
        size_t next = bestIdx;
        while (next + 1 < route.size() && nav::haversineM(pos, route[next]) < kLookAheadM) ++next;
        g.active = g.route = true;
        g.name = routeName.c_str();
        g.next = route[next];
        g.distNextM = nav::haversineM(pos, g.next);
        g.bearingDeg = nav::initialBearingDeg(pos, g.next);
        g.remainingM = g.distNextM + (routeCum_.back() - routeCum_[next]);
        g.offRouteM = best;
        g.arrived = nav::haversineM(pos, route.back()) < kArriveM;
        return g;
    }
    if (hasTarget_) {
        g.active = true;
        g.name = target_.name;
        g.next = {target_.lat, target_.lon};
        g.distNextM = g.remainingM = nav::haversineM(pos, g.next);
        g.bearingDeg = nav::initialBearingDeg(pos, g.next);
        g.arrived = g.distNextM < kArriveM;
    }
    return g;
}

bool NavData::startTrack(const String &stamp, String &err) {
    if (recording_) return true;
    trackPath_ = String(storage::kTracksDir) + "/" + stamp + ".gpx";
    File f = SD.open(trackPath_, FILE_WRITE);
    if (!f) {
        err = "Cannot create " + trackPath_;
        return false;
    }
    char buf[320];
    nav::gpxTrackHeader(buf, sizeof buf, stamp.c_str());
    f.print(buf);
    f.print(nav::kGpxTrackFooter);
    f.close();
    recording_ = true;
    trackPoints_ = 0;
    pending_ = "";
    pendingCount_ = 0;
    lastLogAt_ = 0;
    lastFlushAt_ = millis();
    breadcrumb.clear();
    return true;
}

void NavData::stopTrack() {
    if (!recording_) return;
    flushTrack();
    recording_ = false;
}

void NavData::updateTrack(const GpsFix &fix, const char *isoTime) {
    if (!recording_) return;
    const uint32_t now = millis();
    if (fix.valid && fix.ageMs < 3000 && now - lastLogAt_ >= 2000) {
        const nav::LatLon pos{fix.lat, fix.lon};
        const bool moved = lastLogAt_ == 0 || nav::haversineM(lastLogged_, pos) >= 5.0;
        if (moved || now - lastLogAt_ >= 30000) {
            char buf[160];
            nav::gpxTrackPoint(buf, sizeof buf, fix.lat, fix.lon, fix.altM, isoTime);
            pending_ += buf;
            ++pendingCount_;
            ++trackPoints_;
            lastLogAt_ = now;
            lastLogged_ = pos;
            if (breadcrumb.size() >= kMaxBreadcrumb) breadcrumb.erase(breadcrumb.begin(), breadcrumb.begin() + 300);
            breadcrumb.push_back(pos);
        }
    }
    if (pendingCount_ >= 10 || (pendingCount_ && now - lastFlushAt_ > 30000)) flushTrack();
}

void NavData::flushTrack() {
    lastFlushAt_ = millis();
    if (!pendingCount_) return;
    // Overwrite the closing tags so the file stays a valid GPX document after every flush.
    File f = SD.open(trackPath_, "r+");
    if (!f) return;
    const size_t footerLen = strlen(nav::kGpxTrackFooter);
    const size_t size = f.size();
    f.seek(size >= footerLen ? size - footerLen : size);
    f.print(pending_);
    f.print(nav::kGpxTrackFooter);
    f.close();
    pending_ = "";
    pendingCount_ = 0;
}
