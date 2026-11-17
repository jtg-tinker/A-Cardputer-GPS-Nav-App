#pragma once
#include <Arduino.h>
#include <geo.h>
#include <vector>

#include "gps.h"

struct Waypoint {
    char name[24] = {0};
    double lat = 0;
    double lon = 0;
};

struct Guidance {
    bool active = false;       // a target or route is set and we have a fix
    bool route = false;        // guidance comes from a route
    const char *name = "";     // target name
    nav::LatLon next{0, 0};    // point to steer towards
    double distNextM = 0;      // distance to `next`
    double bearingDeg = 0;     // bearing to `next`
    double remainingM = 0;     // distance to destination (along the route)
    double offRouteM = 0;      // distance from nearest route vertex
    bool arrived = false;
};

class NavData {
public:
    // Waypoints (persisted in /gpsnav/waypoints.gpx)
    std::vector<Waypoint> waypoints;
    bool loadWaypoints();
    bool saveWaypoints() const;
    bool addWaypoint(const String &name, double lat, double lon);
    void removeWaypoint(size_t idx);
    String nextWaypointName() const;

    // Go-to target
    bool hasTarget() const { return hasTarget_; }
    const Waypoint &target() const { return target_; }
    void setTarget(const Waypoint &w);
    void clearTarget();

    // Route loaded from a GPX file (trk/rte points, or waypoints in file order)
    std::vector<nav::LatLon> route;
    String routeName;
    bool hasRoute() const { return route.size() >= 2; }
    bool loadRoute(const String &path, String &err);
    void reverseRoute();
    void clearRoute();
    // Route worked out from the map's start / end points (replaces the current route).
    void setRoute(std::vector<nav::LatLon> &&pts, const String &name);
    bool routeFromMap = false;

    // Start / end picked on the map with s / e
    bool hasRouteStart = false, hasRouteEnd = false;
    nav::LatLon routeStart{0, 0}, routeEnd{0, 0};
    void clearRoutePoints() { hasRouteStart = hasRouteEnd = false; }

    Guidance guidance(const GpsFix &fix);

    // Track recording to /gpsnav/tracks/*.gpx
    bool recording() const { return recording_; }
    bool startTrack(const String &stamp, String &err);
    void stopTrack();
    void updateTrack(const GpsFix &fix, const char *isoTime);
    uint32_t trackPoints() const { return trackPoints_; }
    const String &trackPath() const { return trackPath_; }
    std::vector<nav::LatLon> breadcrumb;

private:
    void flushTrack();
    void rebuildRouteDistances();

    bool hasTarget_ = false;
    Waypoint target_;
    std::vector<float> routeCum_;
    size_t routeIdx_ = 0;
    bool routeIdxValid_ = false;

    bool recording_ = false;
    String trackPath_;
    String pending_;
    uint32_t pendingCount_ = 0;
    uint32_t trackPoints_ = 0;
    uint32_t lastLogAt_ = 0;
    uint32_t lastFlushAt_ = 0;
    nav::LatLon lastLogged_{0, 0};
};
