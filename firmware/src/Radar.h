// The scope renderer: projects the world model to the round panel and
// draws the rings, sweep, blips, overlays and follow animations. Ported
// from the interactive mockup's rendering logic.

#pragma once

#include <Arduino_GFX_Library.h>

#include "Model.h"

namespace radar {

// Advances time-based state (sweep angle, the eased view center, and the
// sweep-refresh of each blip) by `dtMs` milliseconds.
void step(model::Model& model, uint32_t dtMs);

// Draws one frame of the live scope.
void renderScene(Arduino_GFX* gfx, model::Model& model);

// Draws one frame of the power on/off CRT animation. `progress` runs 0..1;
// `poweringOn` selects the bloom (true) or the collapse (false). Returns
// true once the animation has completed.
bool renderTransition(Arduino_GFX* gfx, model::Model& model, float progress,
                      bool poweringOn);

}  // namespace radar
