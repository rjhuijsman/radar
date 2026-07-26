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

// True when `flight` can be matched to a broadcast ICAO callsign at all: an
// ICAO-form entry always, an IATA one only if its airline is in the table.
// False means detected-but-not-trackable, which the dashboard flags.
bool flightTrackable(const String& flight);

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

// Keeps the followed flight's onward-path destination current. When the
// follow target changes to a new callsign, looks its route up once from
// adsbdb, projects the destination airport's lat/lon around the active
// home, and publishes it as model.ui.followDest (with followHasDest) for
// the renderer to draw the expected onward path to. The route does not
// change mid-flight, so it is fetched once per followed callsign and
// reused. The lookup runs with no lock held; the mutex is taken only to
// snapshot the target and to publish. A follow change clears the
// destination promptly; an unknown callsign or a failed fetch leaves none
// drawn (failures back off). Network-free and cheap otherwise, so it can
// be called every loop.
void pollDest(model::Model& model, SemaphoreHandle_t mutex);

// Recomputes each POI's world position from its lat/lng relative to the
// active home. Callers hold the model mutex. Call after loading config or
// switching home.
void reprojectStatics(model::Model& model);

}  // namespace feeds
