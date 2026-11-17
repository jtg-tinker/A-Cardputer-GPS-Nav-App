#pragma once
#include <Arduino.h>
#include <geo.h>
#include <streets.h>

#include <vector>

namespace streetdata {

// Turns a downloaded vector tile (plain or gzip) into the compact street file at binPath.
bool convertDownloaded(const char *pbfPath, const char *binPath);
// Builds the road graph file for routing. Needs a lot of RAM, so run it with WiFi off.
bool convertRoads(const char *pbfPath, const char *roadPath);

// Street name at p: 1 = found, 0 = street data present but no named street nearby,
// -1 = no street data downloaded for this spot.
int lookup(const nav::LatLon &p, char *name, size_t n);

// Calls cb for every street line in the z14 street tile (tx, ty). False if there is no data.
bool forEachLine(int tx, int ty, nav::StreetLineFn cb, void *ctx);

// Lines of the street found by the last successful lookup(), in coordinates of z14 tile (tx, ty).
struct Highlight {
    int tx = 0, ty = 0;
    uint16_t extent = 4096;
    char name[64] = {0};
    std::vector<std::vector<int16_t>> lines;
};
const Highlight &highlight();

void invalidate();

}  // namespace streetdata
