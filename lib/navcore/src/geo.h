#pragma once
#include <stddef.h>
#include <stdint.h>

namespace nav {

constexpr double kEarthRadiusM = 6371008.8;
constexpr int kTileSize = 256;
constexpr int kMinZoom = 1;
constexpr int kMaxZoom = 18;
constexpr double kMaxMercatorLat = 85.05112878;

struct LatLon {
    double lat;
    double lon;
};

// Global pixel coordinates at a zoom level (Web Mercator, 256px tiles).
struct WorldPx {
    double x;
    double y;
};

struct BBox {
    double south;
    double west;
    double north;
    double east;
};

// Inclusive tile index range at one zoom level.
struct TileRange {
    int z;
    int x0;
    int y0;
    int x1;
    int y1;
    uint32_t count() const { return uint32_t(x1 - x0 + 1) * uint32_t(y1 - y0 + 1); }
};

double clampLat(double lat);
double wrapLon(double lon);
double worldSizePx(int zoom);

WorldPx latLonToWorldPx(double lat, double lon, int zoom);
LatLon worldPxToLatLon(double x, double y, int zoom);

double haversineM(LatLon a, LatLon b);
double initialBearingDeg(LatLon from, LatLon to);
LatLon destination(LatLon start, double bearingDeg, double distM);

BBox bboxAround(LatLon center, double radiusM);
bool bboxValid(const BBox &b);
TileRange tileRangeFor(const BBox &b, int z);
uint32_t tileCount(const BBox &b, int zMin, int zMax);

const char *compassPoint(double bearingDeg);
void formatDistance(double meters, bool imperial, char *buf, size_t n);
void formatSpeed(double kmh, bool imperial, char *buf, size_t n);

}  // namespace nav
