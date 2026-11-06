#include <SD.h>
#include <M5Cardputer.h>

#include "app.h"
#include "mapview.h"
#include "storage.h"
#include "ui.h"

namespace app {

M5Canvas canvas(&M5Cardputer.Display);
AppConfig config;
GpsReceiver gps;
NavData navdata;

namespace {
bool centerPending = false;
double pendingLat = 0, pendingLon = 0;
int pendingZoom = -1;
}  // namespace

void service() {
    gps.poll();
    if (navdata.recording()) {
        char iso[24];
        navdata.updateTrack(gps.fix(), gps.isoTime(iso, sizeof iso) ? iso : nullptr);
    }
}

void requestCenter(double lat, double lon, int zoom) {
    centerPending = true;
    pendingLat = lat;
    pendingLon = lon;
    pendingZoom = zoom;
}

bool takeCenterRequest(double &lat, double &lon, int &zoom) {
    if (!centerPending) return false;
    centerPending = false;
    lat = pendingLat;
    lon = pendingLon;
    zoom = pendingZoom;
    return true;
}

}  // namespace app

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setRotation(1);
    app::canvas.setColorDepth(16);
    app::canvas.createSprite(app::kW, app::kH);

    ui::message("GPS Nav", String("cardputer-gps-nav v") + app::kVersion + "\n\nStarting...", false);
    app::gps.begin();

    if (M5.getBoard() != m5::board_t::board_M5CardputerADV) {
        ui::message("Unsupported device",
                    "This app needs an M5Stack Cardputer ADV with the Cap LoRa-1262 (GNSS) attached.\n\n"
                    "Press a key to continue anyway.");
    }
    while (!storage::begin()) {
        ui::message("No SD card", "Insert a FAT32 microSD card, then press a key to retry.");
    }
    app::config.load();
    M5Cardputer.Display.setBrightness(app::config.brightness);
    app::navdata.loadWaypoints();
    mapView.begin(app::config.lastLat, app::config.lastLon, app::config.lastZoom);
}

void loop() {
    app::service();
    mapView.tick();
    delay(5);
}
