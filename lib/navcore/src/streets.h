#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace nav {

// Street names come from MapTiler's OpenMapTiles vector tiles, layer "transportation_name",
// whose most detailed zoom is 14. Each downloaded tile is reduced to a compact file:
//   "GNS1" | u16 extent | records: u8 nameLen, name (ASCII), u16 nPts, nPts * (i16 x, i16 y)
// with coordinates in tile units (0..extent), little endian.
constexpr int kStreetZoom = 14;

class ByteSource {
public:
    virtual ~ByteSource() = default;
    virtual size_t size() = 0;
    virtual bool read(size_t offset, uint8_t *buf, size_t n) = 0;
};

class ByteSink {
public:
    virtual ~ByteSink() = default;
    virtual bool write(const uint8_t *buf, size_t n) = 0;
};

class MemSource : public ByteSource {
public:
    MemSource(const uint8_t *data, size_t n) : data_(data), n_(n) {}
    size_t size() override { return n_; }
    bool read(size_t offset, uint8_t *buf, size_t n) override {
        if (offset > n_ || n > n_ - offset) return false;
        memcpy(buf, data_ + offset, n);
        return true;
    }

private:
    const uint8_t *data_;
    size_t n_;
};

bool buildStreetUrl(const char *apiKey, int z, int x, int y, char *out, size_t n);
// <root>/<z>/<x>/<y>.bin and its column directory.
bool buildStreetPath(const char *root, int z, int x, int y, char *out, size_t n);
bool buildStreetDir(const char *root, int z, int x, char *out, size_t n);

// Converts a Mapbox Vector Tile (uncompressed) to the compact street format.
// Returns the number of street lines written, or -1 if the tile is malformed or writing fails.
int convertMvtStreets(ByteSource &mvt, ByteSink &out);

// Nearest street to (fx, fy), given as fractions (0..1) of the tile. On success `dist` is in
// tile widths. Returns false if the file is invalid or has no streets.
bool nearestStreet(ByteSource &streets, double fx, double fy, char *name, size_t n, double &dist);

// UTF-8 to printable ASCII for the built-in fonts (Latin accents folded, e.g. "Straße" -> "Strasse").
void asciiFold(const char *utf8, size_t len, char *out, size_t n);

}  // namespace nav
