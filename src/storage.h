#pragma once
#include <Arduino.h>

namespace storage {

constexpr const char *kRoot = "/gpsnav";
constexpr const char *kTilesRoot = "/gpsnav/tiles";
constexpr const char *kConfigPath = "/gpsnav/config.json";
constexpr const char *kWaypointsPath = "/gpsnav/waypoints.gpx";
constexpr const char *kRegionsPath = "/gpsnav/regions.csv";
constexpr const char *kRoutesDir = "/gpsnav/routes";
constexpr const char *kTracksDir = "/gpsnav/tracks";

bool begin();
bool mkdirs(const char *path);
uint64_t freeBytes();
uint64_t totalBytes();

}  // namespace storage
