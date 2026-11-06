#include <SD.h>
#include <M5Cardputer.h>
#include <esp_system.h>

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

namespace {
// Boot-stage breadcrumb that survives a crash/reset, so a boot loop can be diagnosed on screen.
constexpr uint32_t kBootMagic = 0x6E617631;
RTC_NOINIT_ATTR uint32_t bootMagic;
RTC_NOINIT_ATTR uint32_t bootStage;
const char *const kStageNames[] = {"none",      "display",  "gps",       "board check", "sd card",
                                   "config",    "waypoints", "map init", "first map frame", "running"};

void stage(uint32_t s) {
    bootMagic = kBootMagic;
    bootStage = s;
    Serial.printf("[gpsnav] stage %u: %s\n", (unsigned)s, kStageNames[s]);
}

const char *resetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_PANIC: return "crash (panic)";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "watchdog";
        case ESP_RST_BROWNOUT: return "brownout (low power)";
        default: return nullptr;
    }
}

void showCrashReport(esp_reset_reason_t reason, uint32_t prevStage) {
    const char *why = resetReasonName(reason);
    if (!why) return;
    const char *where = prevStage < sizeof(kStageNames) / sizeof(kStageNames[0]) ? kStageNames[prevStage] : "?";
    ui::message("Crash report", String("The last run reset: ") + why + "\nStage: " + where +
                                    "\n\nPlease report this. Press a key to continue.");
}
}  // namespace

void setup() {
    const esp_reset_reason_t reason = esp_reset_reason();
    const uint32_t prevStage = bootMagic == kBootMagic ? bootStage : 0;
    Serial.begin(115200);
    stage(1);
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setRotation(1);
    app::canvas.setColorDepth(16);
    app::canvas.createSprite(app::kW, app::kH);

    showCrashReport(reason, prevStage);
    ui::message("GPS Nav", String("cardputer-gps-nav v") + app::kVersion + "\n\nStarting...", false);
    stage(2);
    app::gps.begin();
    stage(3);

    if (M5.getBoard() != m5::board_t::board_M5CardputerADV) {
        ui::message("Unsupported device",
                    "This app needs an M5Stack Cardputer ADV with the Cap LoRa-1262 (GNSS) attached.\n\n"
                    "Press a key to continue anyway.");
    }
    stage(4);
    while (!storage::begin()) {
        ui::message("No SD card", "Insert a FAT32 microSD card, then press a key to retry.");
    }
    stage(5);
    app::config.load();
    M5Cardputer.Display.setBrightness(app::config.brightness);
    stage(6);
    app::navdata.loadWaypoints();
    stage(7);
    mapView.begin(app::config.lastLat, app::config.lastLon, app::config.lastZoom);
    stage(8);
}

void loop() {
    app::service();
    mapView.tick();
    if (bootStage != 9) stage(9);
    delay(5);
}
