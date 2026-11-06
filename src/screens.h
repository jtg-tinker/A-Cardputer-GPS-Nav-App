#pragma once
#include <geo.h>

namespace screens {

void mainMenu(const nav::LatLon &mapCenter);
void downloadMenu(const nav::LatLon &mapCenter, int mapZoom);
void regionsMenu();
void waypointsMenu(const nav::LatLon &mapCenter);
void quickWaypoint(const nav::LatLon &mapCenter);
void routesMenu();
void gotoCoordinates();
void toggleRecording();
void clearNavigation();
void gpsStatus();
void settingsMenu();
void storageInfo();
void help();

}  // namespace screens
