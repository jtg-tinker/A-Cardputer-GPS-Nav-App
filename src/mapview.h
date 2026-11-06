#pragma once
#include <geo.h>

class MapView {
public:
    void begin(double lat, double lon, int zoom);
    void tick();
    // Frees the cached base layer (e.g. to give WiFi/TLS room); recreated on demand.
    void releaseBase();
    nav::LatLon center() const { return center_; }
    int zoom() const { return zoom_; }

private:
    void handleKey();
    void renderBase();
    void drawTile(int z, int x, int y, int dx, int dy);
    void drawOverlays();
    void drawStatus();
    bool toScreen(const nav::LatLon &ll, int &sx, int &sy) const;
    void pan(int dx, int dy);
    void setZoom(int z);
    void persistView();

    nav::LatLon center_{0, 0};
    int zoom_ = 3;
    bool follow_ = true;
    bool baseDirty_ = true;
    bool baseValid_ = false;
    double originX_ = 0, originY_ = 0;
    uint32_t lastFrame_ = 0;
    uint32_t lastPersist_ = 0;
    bool viewChanged_ = false;
    bool arrivedShown_ = false;
};

extern MapView mapView;
