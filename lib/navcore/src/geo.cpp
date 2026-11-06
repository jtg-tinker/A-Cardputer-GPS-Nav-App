#include "geo.h"

#include <math.h>
#include <stdio.h>

namespace nav {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double rad(double d) { return d * kPi / 180.0; }
inline double deg(double r) { return r * 180.0 / kPi; }
inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

double clampLat(double lat) {
    if (lat > kMaxMercatorLat) return kMaxMercatorLat;
    if (lat < -kMaxMercatorLat) return -kMaxMercatorLat;
    return lat;
}

double wrapLon(double lon) {
    while (lon < -180.0) lon += 360.0;
    while (lon >= 180.0) lon -= 360.0;
    return lon;
}

double worldSizePx(int zoom) { return double(kTileSize) * double(1u << zoom); }

WorldPx latLonToWorldPx(double lat, double lon, int zoom) {
    const double size = worldSizePx(zoom);
    const double s = sin(rad(clampLat(lat)));
    WorldPx p;
    p.x = (wrapLon(lon) + 180.0) / 360.0 * size;
    p.y = (0.5 - log((1.0 + s) / (1.0 - s)) / (4.0 * kPi)) * size;
    return p;
}

LatLon worldPxToLatLon(double x, double y, int zoom) {
    const double size = worldSizePx(zoom);
    LatLon ll;
    ll.lon = wrapLon(x / size * 360.0 - 180.0);
    const double n = kPi - 2.0 * kPi * y / size;
    ll.lat = clampLat(deg(atan(sinh(n))));
    return ll;
}

double haversineM(LatLon a, LatLon b) {
    const double dLat = rad(b.lat - a.lat);
    const double dLon = rad(b.lon - a.lon);
    const double h = sin(dLat / 2) * sin(dLat / 2) +
                     cos(rad(a.lat)) * cos(rad(b.lat)) * sin(dLon / 2) * sin(dLon / 2);
    return 2.0 * kEarthRadiusM * atan2(sqrt(h), sqrt(1.0 - h));
}

double initialBearingDeg(LatLon from, LatLon to) {
    const double p1 = rad(from.lat), p2 = rad(to.lat);
    const double dl = rad(to.lon - from.lon);
    const double y = sin(dl) * cos(p2);
    const double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = deg(atan2(y, x));
    return fmod(b + 360.0, 360.0);
}

LatLon destination(LatLon start, double bearingDeg, double distM) {
    const double d = distM / kEarthRadiusM;
    const double b = rad(bearingDeg);
    const double p1 = rad(start.lat), l1 = rad(start.lon);
    const double p2 = asin(sin(p1) * cos(d) + cos(p1) * sin(d) * cos(b));
    const double l2 = l1 + atan2(sin(b) * sin(d) * cos(p1), cos(d) - sin(p1) * sin(p2));
    return LatLon{deg(p2), wrapLon(deg(l2))};
}

BBox bboxAround(LatLon c, double radiusM) {
    BBox b;
    b.north = clampLat(destination(c, 0, radiusM).lat);
    b.south = clampLat(destination(c, 180, radiusM).lat);
    b.east = destination(c, 90, radiusM).lon;
    b.west = destination(c, 270, radiusM).lon;
    if (b.east < b.west) {  // antimeridian crossing is not supported; clip to the center's side
        if (c.lon >= 0) b.east = 179.999999;
        else b.west = -180.0;
    }
    return b;
}

bool bboxValid(const BBox &b) {
    return b.south < b.north && b.west < b.east && b.south >= -90 && b.north <= 90 && b.west >= -180 &&
           b.east <= 180;
}

TileRange tileRangeFor(const BBox &b, int z) {
    const int maxIdx = (1 << z) - 1;
    WorldPx nw = latLonToWorldPx(b.north, b.west, z);
    WorldPx se = latLonToWorldPx(b.south, b.east, z);
    TileRange r;
    r.z = z;
    r.x0 = clampInt(int(floor(nw.x / kTileSize)), 0, maxIdx);
    r.y0 = clampInt(int(floor(nw.y / kTileSize)), 0, maxIdx);
    r.x1 = clampInt(int(floor(se.x / kTileSize)), 0, maxIdx);
    r.y1 = clampInt(int(floor(se.y / kTileSize)), 0, maxIdx);
    if (r.x1 < r.x0) r.x1 = r.x0;
    if (r.y1 < r.y0) r.y1 = r.y0;
    return r;
}

uint32_t tileCount(const BBox &b, int zMin, int zMax) {
    uint32_t total = 0;
    for (int z = zMin; z <= zMax; ++z) total += tileRangeFor(b, z).count();
    return total;
}

const char *compassPoint(double bearingDeg) {
    static const char *const kPts[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    double b = fmod(fmod(bearingDeg, 360.0) + 360.0, 360.0);
    return kPts[int((b + 22.5) / 45.0) % 8];
}

void formatDistance(double m, bool imperial, char *buf, size_t n) {
    if (imperial) {
        const double ft = m * 3.28084;
        if (ft < 1000) snprintf(buf, n, "%.0fft", ft);
        else snprintf(buf, n, m / 1609.344 < 10 ? "%.2fmi" : "%.1fmi", m / 1609.344);
    } else {
        if (m < 1000) snprintf(buf, n, "%.0fm", m);
        else snprintf(buf, n, m < 10000 ? "%.2fkm" : "%.1fkm", m / 1000.0);
    }
}

void formatSpeed(double kmh, bool imperial, char *buf, size_t n) {
    if (imperial) snprintf(buf, n, "%.0fmph", kmh * 0.621371);
    else snprintf(buf, n, "%.0fkm/h", kmh);
}

}  // namespace nav
