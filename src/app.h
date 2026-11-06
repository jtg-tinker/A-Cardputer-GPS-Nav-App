#pragma once
#include <SD.h>
#include <M5Cardputer.h>

#include "config.h"
#include "gps.h"
#include "navdata.h"

namespace app {

constexpr int kW = 240;
constexpr int kH = 135;
constexpr const char *kVersion = "0.1.1";
constexpr const char *kUserAgent = "cardputer-gps-nav/0.1 (+https://github.com/jtg-tinker/cardputer-gps-nav)";

extern M5Canvas canvas;
extern AppConfig config;
extern GpsReceiver gps;
extern NavData navdata;

// Background work (GNSS parsing, track logging, arrival alerts). Call from every UI loop.
void service();

// Ask the map screen to recenter (consumed on its next frame).
void requestCenter(double lat, double lon, int zoom = -1);
bool takeCenterRequest(double &lat, double &lon, int &zoom);

}  // namespace app
