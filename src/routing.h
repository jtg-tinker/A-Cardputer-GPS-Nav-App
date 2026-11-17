#pragma once
#include <Arduino.h>
#include <geo.h>

namespace routing {

// Works out a road route over the downloaded road data (Drive or Walk, from Settings), showing
// progress; ` cancels. On success it becomes the current route and `msg` holds a short summary;
// otherwise `msg` says why it failed.
bool compute(const nav::LatLon &from, const nav::LatLon &to, String &msg);

}  // namespace routing
