// The scope renderer: projects the world model to the round panel and
// draws the rings, sweep, blips, overlays and follow animations. Ported
// from the interactive mockup's rendering logic.

#pragma once

#include <Arduino_GFX_Library.h>

#include "Model.h"

namespace display {
class LiveGfx;
}

namespace radar {

// Binds the incremental renderer to the live surface (retargeted at the
// panel's back framebuffer each frame) and the off-screen static-reference
// surface, for dirty-rect erase/redraw. Call once after display bring-up.
void beginRenderer(display::LiveGfx* live, Arduino_GFX* staticRef,
                   uint16_t* staticFb, int16_t width, int16_t height);

// Advances time-based state (sweep angle, the eased view center, the
// followed flight's smoothed position, and the sweep-refresh of each blip)
// by `dtMs` milliseconds. Requests a traffic poll (model.adsbPollDue) when
// the sweep crosses 12 o'clock.
void step(model::Model& model, uint32_t dtMs);

// Draws one steady-state frame incrementally into the back framebuffer and
// flips: rebuilds the static reference only when the view/range/mode/
// geography/home/brightness changed, then erases and redraws just the
// moving elements (sweep, markers, blips, overlay).
void renderIncremental(model::Model& model);

// Forces the next incremental frames to rebuild the static reference and
// repaint both framebuffers from it. Call when their content can no longer
// be trusted to match the renderer's bookkeeping.
void invalidate();

// Draws one frame of the power on/off CRT animation. `progress` runs 0..1;
// `poweringOn` selects the bloom (true) or the collapse (false). Returns
// true once the animation has completed. Both animations are incremental —
// each frame writes only a small delta of the shared framebuffer, so they
// cannot tear against the panel scan — and the bloom's reveal leaves the
// renderer in a consistent state, so no invalidate() is needed after it.
bool renderTransition(Arduino_GFX* gfx, model::Model& model, float progress,
                      bool poweringOn);

// Draws the full-screen "MISSING KEY" notice shown when the set boots with
// the key switch off, just before it goes back to deep sleep.
void showMissingKey(Arduino_GFX* gfx);

}  // namespace radar
