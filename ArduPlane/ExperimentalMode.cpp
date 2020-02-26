#include "Plane.h"

bool Plane::set_experimental_mode(bool enabled) {
	if( !allow_experimental_mode ) {
		experimental_mode_enabled = false;
		return false;
		}
	experimental_mode_enabled = enabled;
	if (enabled == true) {
		// Reset on transition to true
		g2.mlController.reset();
		}
	return true;
	}
