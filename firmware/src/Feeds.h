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
// callsigns used by `pollTraffic` from the feeds' today-dated flights plus
// the manually-entered specials due today. Also publishes the dashboard's
// live views: `model.icalSpecials` (the from-calendar flights matched
// today) and `model.feedStatus` (per-feed sync result and time). Returns
// the number of feeds fetched successfully.
int pollIcal(model::Model& model, SemaphoreHandle_t mutex);

// True when any ICAO callsign form of `flight` — the raw entry normalized,
// or its IATA airline code converted ("QR106" -> "QTR106") — matches an
// aircraft currently tracked. A live snapshot against the fetched traffic:
// false may just mean the flight is outside the fetch radius right now.
// Callers hold the model mutex.
bool flightTracked(const model::Model& model, const String& flight);

// Keeps the rain-radar layer current. When the refetch interval has
// elapsed — or the view has left the fetched tiles and settled — fetches
// the newest RainViewer frame's Web-Mercator tiles covering the view,
// decodes their palette back into reflectivity, and publishes the mosaic
// as `model.weather` for the renderer to sample. The fetch and decode run
// with no lock held; the mutex is taken only to snapshot the view and to
// swap the finished layer in. Network-free and cheap when nothing is due,
// so it can be called every couple of seconds.
void pollWeather(model::Model& model, SemaphoreHandle_t mutex);

// Keeps the followed flight's historic trail current. When the follow
// target changes to an aircraft with a known ICAO hex, fetches its
// full-day position trace from adsb.lol (served gzipped; inflated
// on-device), projects it around the active home, and seeds
// `model.ui.followTrail` with the decimated path so the renderer's live
// sampling extends it — the trail then reads full history + live tail.
// The fetch, gunzip and parse all run with no lock held; the mutex is
// taken only to snapshot the follow target and to publish. Network-free
// and cheap when the target is unchanged, so it can be called every
// loop. On any failure the live-only trail is left alone.
void pollTrace(model::Model& model, SemaphoreHandle_t mutex);

// Recomputes each POI's world position from its lat/lng relative to the
// active home. Callers hold the model mutex. Call after loading config or
// switching home.
void reprojectStatics(model::Model& model);

}  // namespace feeds
