#include "Plane.h"

void Plane::experimental_mode(bool enabled) {
	experimental_mode_enabled = enabled;
	if (enabled == true) {
		// Reset on transition to true
		g2.mlController.reset();
		}
	}
