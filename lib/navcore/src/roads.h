#pragma once
#include <stddef.h>
#include <stdint.h>

#include <vector>

#include "geo.h"
#include "streets.h"

namespace nav {

// Routable road graph for one z14 tile, built from the OpenMapTiles "transportation" layer
// (same download as the street names). Junctions become nodes; lines between them become
// edges with their geometry. Little-endian file layout:
//   header (32 bytes): "GNR1", u16 extent, u16 nNodes, u16 nEdges, u16 nBoundary[4] (W, E, N, S),
//                      u16 reserved, u32 nAdj, u32 nGeom, u32 reserved
//   nodes  (nNodes + 1) x { i16 x, i16 y, u32 firstAdj }      (last record is a sentinel)
//   adj    nAdj x { u16 to, i16 toX, i16 toY, u16 len, u16 edge, u8 class, u8 flags }
//   edges  nEdges x { u16 a, u16 b, u16 len, u16 nGeom, u32 firstGeom, u8 class, u8 oneway flags,
//                     i16 minX, minY, maxX, maxY }
//   geom   nGeom x { i16 x, i16 y }                            (interior points, a -> b)
//   bounds per side, sorted by position along the side: { i16 pos, u16 node }
// Coordinates and lengths are in tile units (0..extent).

enum RoadClass : uint8_t {
    kRoadMotorway,
    kRoadTrunk,
    kRoadPrimary,
    kRoadSecondary,
    kRoadTertiary,
    kRoadMinor,
    kRoadService,
    kRoadTrack,
    kRoadPath,
    kRoadClassCount
};

enum RoadFlag : uint8_t {
    kRoadOneway = 1,     // only drivable in geometry direction
    kRoadOnewayRev = 2,  // only drivable against geometry direction
    kRoadBrunnel = 4,    // bridge or tunnel: crossings don't connect
};

struct RoadLine {
    uint32_t start = 0;  // first point in RoadLines::xy
    uint16_t count = 0;  // points
    uint8_t cls = kRoadMinor;
    uint8_t flags = 0;
    int8_t layer = 0;
};

// All lines of a tile share one point array (a vector per line costs too much RAM on the device).
struct RoadLines {
    std::vector<RoadLine> lines;
    std::vector<int16_t> xy;  // x, y pairs in tile units
    const int16_t *pts(const RoadLine &l) const { return xy.data() + 2 * size_t(l.start); }
};

bool buildRoadPath(const char *root, int z, int x, int y, char *out, size_t n);
bool buildRoadDir(const char *root, int z, int x, char *out, size_t n);

// Reads road lines (motorway .. path) from the "transportation" layer of an uncompressed MVT.
bool parseMvtRoads(ByteSource &mvt, RoadLines &out, uint16_t &extent);
// Builds and writes the junction graph. Consumes `lines`. Returns the node count, or -1.
int buildRoadGraph(RoadLines &lines, uint16_t extent, ByteSink &out);
// parseMvtRoads + buildRoadGraph; -1 on malformed input, write failure or out of memory.
int convertMvtRoads(ByteSource &mvt, ByteSink &out);

// Road graph files by z14 tile. The returned source only needs to stay valid until the next call.
class RoadTiles {
public:
    virtual ~RoadTiles() = default;
    virtual ByteSource *tile(int tx, int ty) = 0;  // nullptr if the tile has no road file
};

enum class RouteMode : uint8_t { Drive, Walk };

enum class RouteStatus : uint8_t { Ok, NoData, NoRoadNearStart, NoRoadNearEnd, NoRoute, TooFar, Cancelled, Error };

struct RouteOptions {
    RouteMode mode = RouteMode::Drive;
    uint32_t maxNodes = 4000;     // search budget (memory ~ 36 bytes per node)
    float heuristicWeight = 1.5f;  // >1 trades a little optimality for a much smaller search
    double maxSnapM = 150;         // how far start / end may be from a road
    // Called every few hundred steps; return true to cancel.
    bool (*poll)(void *ctx, uint32_t visited) = nullptr;
    void *ctx = nullptr;
};

struct RouteResult {
    std::vector<LatLon> pts;
    double lengthM = 0;
    double timeS = 0;
    uint32_t visited = 0;
};

RouteStatus findRoute(RoadTiles &tiles, LatLon from, LatLon to, const RouteOptions &opt, RouteResult &out);
const char *routeStatusText(RouteStatus s);

}  // namespace nav
