// Physical controls: the I2C rotary encoder (with push), the three toggle
// switches, and the ambient-light sensor. `poll()` reads them and applies
// the browse-then-commit interaction directly to the shared model.
//
// The caller must hold the model mutex around `poll()`, since it mutates
// `model.ui` while the display task reads it.

#pragma once

#include "Model.h"

namespace inputs {

// Initializes the toggle GPIOs and the I2C encoder and light sensor.
void begin();

// Reads the controls once and updates `model.ui`. Call at ~50 Hz.
void poll(model::Model& model);

}  // namespace inputs
