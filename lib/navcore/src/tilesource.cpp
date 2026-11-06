#include "tilesource.h"

#include <stdio.h>
#include <string.h>

namespace nav {

const char *providerId(Provider p) { return p == Provider::Thunderforest ? "thunderforest" : "maptiler"; }

bool parseProvider(const char *id, Provider &out) {
    if (!id) return false;
    if (strcmp(id, "maptiler") == 0) {
        out = Provider::MapTiler;
        return true;
    }
    if (strcmp(id, "thunderforest") == 0) {
        out = Provider::Thunderforest;
        return true;
    }
    return false;
}

const char *defaultStyle(Provider p) { return p == Provider::Thunderforest ? "outdoors" : "streets-v2"; }

bool isSafeToken(const char *s) {
    if (!s || !*s) return false;
    for (const char *c = s; *c; ++c) {
        const bool ok = (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') ||
                        *c == '-' || *c == '_' || *c == '.';
        if (!ok) return false;
    }
    return strstr(s, "..") == nullptr;
}

static bool validTile(int z, int x, int y) {
    if (z < 0 || z > 22) return false;
    const int max = 1 << z;
    return x >= 0 && y >= 0 && x < max && y < max;
}

static bool fits(int written, size_t n) { return written > 0 && size_t(written) < n; }

bool buildTileUrl(Provider p, const char *style, const char *apiKey, int z, int x, int y, char *out, size_t n) {
    if (!isSafeToken(style) || !isSafeToken(apiKey) || !validTile(z, x, y)) return false;
    int w;
    if (p == Provider::Thunderforest) {
        w = snprintf(out, n, "https://tile.thunderforest.com/%s/%d/%d/%d.jpg90?apikey=%s", style, z, x, y, apiKey);
    } else {
        w = snprintf(out, n, "https://api.maptiler.com/maps/%s/256/%d/%d/%d.jpg?key=%s", style, z, x, y, apiKey);
    }
    return fits(w, n);
}

bool buildTileDir(const char *root, Provider p, const char *style, int z, int x, char *out, size_t n) {
    if (!isSafeToken(style) || z < 0 || x < 0) return false;
    return fits(snprintf(out, n, "%s/%s-%s/%d/%d", root, providerId(p), style, z, x), n);
}

bool buildTilePath(const char *root, Provider p, const char *style, int z, int x, int y, char *out, size_t n) {
    if (!isSafeToken(style) || !validTile(z, x, y)) return false;
    return fits(snprintf(out, n, "%s/%s-%s/%d/%d/%d.jpg", root, providerId(p), style, z, x, y), n);
}

}  // namespace nav
