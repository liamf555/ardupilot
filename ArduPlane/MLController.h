#pragma once

#include <AP_Param/AP_Param.h>
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

class MLController {
	private:
		float elev_rate;
		float elevatorAngle;
		
		uint8_t agent_system;
		uint8_t agent_component;
		mavlink_channel_t agent_channel;
		
		bool lookupSuccess;
		bool elevatorAngleUninitialised;
	
		AP_Float output2deg;
		AP_Float deg2output;
	
	public:
		// Parameter info
		static const struct AP_Param::GroupInfo var_info[];
		
		MLController();
		
		// Set the agent system, component and channel.
		void lookup_agent();
		
		// Reset the internal elevatorAngle representation
		void reset();
		
		// Send position to agent
		void send_state();
		
		// Get an updated elevator position
		int16_t get_elevator_output(float timestep);
		
		// Process incoming message from ML agent
		void handle_message(mavlink_message_t* message);
		
	};
