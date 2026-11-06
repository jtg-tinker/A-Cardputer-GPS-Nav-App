#pragma once
#include <stddef.h>

namespace nav {

enum class Provider : unsigned char { MapTiler = 0, Thunderforest = 1 };

const char *providerId(Provider p);
bool parseProvider(const char *id, Provider &out);
const char *defaultStyle(Provider p);

// API keys and style names are restricted to [A-Za-z0-9._-] so they can be
// embedded in URLs and paths without escaping.
bool isSafeToken(const char *s);

// Download URL for a 256px JPEG tile.
bool buildTileUrl(Provider p, const char *style, const char *apiKey, int z, int x, int y, char *out, size_t n);

// SD path of a cached tile: <root>/<provider>-<style>/<z>/<x>/<y>.jpg
bool buildTilePath(const char *root, Provider p, const char *style, int z, int x, int y, char *out, size_t n);

// Directory holding <y>.jpg files for one tile column.
bool buildTileDir(const char *root, Provider p, const char *style, int z, int x, char *out, size_t n);

}  // namespace nav
