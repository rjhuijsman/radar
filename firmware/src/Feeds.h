// Outbound data: live ADS-B traffic, the iCal feeds that mark special
// flights, and (stubbed) rain radar. All run on the network task. Each
// poll takes `mutex` itself, and only briefly — to copy its inputs out of
// the model and to merge parsed results back in — never across a blocking
// HTTP fetch, which would starve the render loop of the mutex and stall
// the picture for the duration of the request.

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Model.h"

namespace feeds {

// Fetches traffic around the active home and merges it into
// `model.aircraft`, preserving each blip's sweep-refresh state by callsign
// and tagging special flights. Aircraft absent from the response are
// removed. Returns false on a fetch/parse failure (the previous aircraft
// are kept).
bool pollTraffic(model::Model& model, SemaphoreHandle_t mutex);

// Fetches every enabled iCal feed and rebuilds the set of special ICAO
// callsigns used by `pollTraffic`. Returns the number of feeds fetched.
int pollIcal(model::Model& model, SemaphoreHandle_t mutex);

// Keeps the rain-radar layer current. When the refetch interval has
// elapsed — or the view has left the fetched tiles and settled — fetches
// the newest RainViewer frame's Web-Mercator tiles covering the view,
// decodes their palette back into reflectivity, and publishes the mosaic
// as `model.weather` for the renderer to sample. The fetch and decode run
// with no lock held; the mutex is taken only to snapshot the view and to
// swap the finished layer in. Network-free and cheap when nothing is due,
// so it can be called every couple of seconds.
void pollWeather(model::Model& model, SemaphoreHandle_t mutex);

// Recomputes each POI's world position from its lat/lng relative to the
// active home. Callers hold the model mutex. Call after loading config or
// switching home.
void reprojectStatics(model::Model& model);

}  // namespace feeds
