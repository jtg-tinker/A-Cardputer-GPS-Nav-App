#pragma once
#include <Arduino.h>
#include <geo.h>

namespace streetdata {

// Turns a downloaded vector tile (plain or gzip) into the compact street file at binPath.
bool convertDownloaded(const char *pbfPath, const char *binPath);

// Street name at p: 1 = found, 0 = street data present but no named street nearby,
// -1 = no street data downloaded for this spot.
int lookup(const nav::LatLon &p, char *name, size_t n);

void invalidate();

}  // namespace streetdata
