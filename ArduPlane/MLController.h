#pragma once

#include <AP_Param/AP_Param.h>
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

/*
PWM conversions from MorphingBixler.java

Sweep:
pwm = (sweepDeg - 94.18137) / -0.06297 // Implementation & comments
Used in MLController.cpp to generate PWM output

Elevator:
pwm = -18.036 * elevDeg + 1331.2 // In comments
pwm = 18.036 * elevDeg + 1500    // Implementation

Setup as angle(4500) type servos
high_out = 4500
Process:
 constrain to -high_out high_out
 if v > 0
  return trim + (v/4500) * (max-trim)
 else
  return trim + (v/4500) * (trim-min) // NB: v < 0 here

*/

class MLController {
	private:
		float elev_rate;
		float elevatorAngle;
		
		float sweep_rate;
		float sweepAngle;
		
		uint8_t agent_system;
		uint8_t agent_component;
		mavlink_channel_t agent_channel;
		
		bool lookupSuccess;
		bool elevatorAngleUninitialised;
		bool sweepAngleUninitialised;

		bool episodeIsComplete;

		AP_Float output2deg;
		AP_Float deg2output;
		
		//long int actionRecv_ms;
		//long int stateSend_ms;
	
	public:
		// Parameter info
		static const struct AP_Param::GroupInfo var_info[];

		int32_t lastControlTime;

		// Flag to track in_progress state for mode change abort
		bool in_progress;

		MLController();
		
		// Set the agent system, component and channel.
		void lookup_agent();
		
		// Reset the internal angle representations
		void reset();
		
		// Send position to agent
		void send_state();
		
		// Get an updated elevator position
		int16_t get_elevator_output(float timestep);
		
		// Get an updated sweep position
		int16_t get_sweep_output(float timestep);
		
		// Process incoming message from ML agent
		void handle_message(const mavlink_message_t& message);

		// Verify if episode is complete
		bool is_complete() const;

	};
