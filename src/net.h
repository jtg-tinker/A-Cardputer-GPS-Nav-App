#pragma once
#include <Arduino.h>
#include <geo.h>
#include <vector>

struct Region {
    String name;
    nav::BBox box{0, 0, 0, 0};
    int zMin = 5;
    int zMax = 15;
};

struct GeoResult {
    String name;
    nav::LatLon center{0, 0};
    nav::BBox box{0, 0, 0, 0};
};

namespace net {

// Connects using the saved network, offering a scan/pick/password flow if needed.
bool wifiConnect();
void wifiOff();
bool wifiSetup();

bool geocode(const String &query, std::vector<GeoResult> &out, String &err);

// Downloads all missing tiles of the region with a progress screen. Returns true when finished.
bool downloadRegion(const Region &r);
// Fetches one tile to validate the provider/style/key; returns a human-readable result.
String testProvider();

bool loadRegions(std::vector<Region> &out);
bool saveRegions(const std::vector<Region> &regions);

}  // namespace net
