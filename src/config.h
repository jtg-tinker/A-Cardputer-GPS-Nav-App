#pragma once
#include <Arduino.h>
#include <tilesource.h>

struct AppConfig {
    String wifiSsid;
    String wifiPass;
    nav::Provider provider = nav::Provider::MapTiler;
    String style = nav::defaultStyle(nav::Provider::MapTiler);
    String apiKey;
    bool imperial = false;
    uint8_t brightness = 160;
    bool walkRoutes = false;  // street routing: Walk instead of Drive

    double lastLat = 20.0;
    double lastLon = 0.0;
    int lastZoom = 3;

    bool load();
    bool save() const;
};
