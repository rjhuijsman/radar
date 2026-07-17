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

// Binds the incremental renderer to the live surface (aimed at the scanned
// framebuffer) and the off-screen static-reference surface, plus their raw
// framebuffers, for dirty-rect erase/redraw. Call once after display bring-up.
void beginRenderer(display::LiveGfx* live, Arduino_GFX* staticRef,
                   uint16_t* liveFb, uint16_t* staticFb, int16_t width,
                   int16_t height);

// Advances time-based state (sweep angle, the eased view center, and the
// sweep-refresh of each blip) by `dtMs` milliseconds.
void step(model::Model& model, uint32_t dtMs);

// Draws one steady-state frame incrementally: rebuilds the static reference
// only when the view/range/mode/geography/brightness changed, then erases and
// redraws just the moving elements (sweep, markers, blips, overlay).
void renderIncremental(model::Model& model);

// Forces the next incremental frame to rebuild the static reference and
// repaint the whole live buffer. Call when returning to the live scope from a
// power on/off transition, which leaves arbitrary content in the live buffer.
void invalidate();

// Draws one full frame of the live scope into `gfx`. Used by the transitions;
// the steady-state loop uses renderIncremental() instead.
void renderScene(Arduino_GFX* gfx, model::Model& model);

// Draws one frame of the power on/off CRT animation. `progress` runs 0..1;
// `poweringOn` selects the bloom (true) or the collapse (false). Returns
// true once the animation has completed.
bool renderTransition(Arduino_GFX* gfx, model::Model& model, float progress,
                      bool poweringOn);

}  // namespace radar
