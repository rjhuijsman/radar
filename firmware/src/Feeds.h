// Outbound data: live ADS-B traffic, the iCal feeds that mark special
// flights, and (stubbed) rain radar. All run on the network task and
// mutate the shared model under its mutex.

#pragma once

#include "Model.h"

namespace feeds {

// Fetches traffic around the active home and merges it into
// `model.aircraft`, preserving each blip's sweep-refresh state by callsign
// and tagging special flights. Returns false on a fetch/parse failure.
bool pollTraffic(model::Model& model);

// Fetches every enabled iCal feed and rebuilds the set of special ICAO
// callsigns used by `pollTraffic`. Returns the number of feeds fetched.
int pollIcal(model::Model& model);

// TODO(weather): fetch and reproject the current RainViewer frame.
void pollWeather(model::Model& model);

// Recomputes each POI's world position from its lat/lng relative to the
// active home. Call after loading config or switching home.
void reprojectStatics(model::Model& model);

}  // namespace feeds
