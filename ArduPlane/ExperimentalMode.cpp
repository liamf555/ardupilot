#include "Plane.h"

void Plane::experimental_mode(bool enabled) {
	experimental_mode_enabled = enabled;
	if (enabled == false) {
		g2.mlController.reset();
		}
	}
