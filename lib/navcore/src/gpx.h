#pragma once
#include <stddef.h>

namespace nav {

enum class GpxPointKind : unsigned char { Track, Route, Waypoint };

// Minimal streaming GPX reader: feed bytes, receive trkpt/rtept/wpt callbacks.
// Only lat/lon attributes and the <name> of waypoints are extracted.
class GpxParser {
public:
    using Callback = void (*)(void *ctx, GpxPointKind kind, double lat, double lon, const char *name);

    GpxParser(Callback cb, void *ctx) : cb_(cb), ctx_(ctx) {}
    void feed(const char *data, size_t len);
    void feed(char c);

private:
    void onTag();
    static bool attr(const char *tag, const char *name, double &out);

    Callback cb_;
    void *ctx_;
    char tag_[192] = {0};
    size_t tagLen_ = 0;
    bool inTag_ = false;
    bool inWpt_ = false;
    bool inName_ = false;
    double wptLat_ = 0, wptLon_ = 0;
    char name_[32] = {0};
    size_t nameLen_ = 0;
};

// Footer closing an open track; writers keep the file valid by seeking back
// over it before appending more points.
extern const char kGpxTrackFooter[];

int gpxTrackHeader(char *out, size_t n, const char *name);
int gpxTrackPoint(char *out, size_t n, double lat, double lon, double eleM, const char *isoTime);
int gpxWaypoint(char *out, size_t n, double lat, double lon, const char *name);

}  // namespace nav
