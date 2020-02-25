#pragma once

#include "include/mavlink/v2.0/uk_ac_bristol/mavlink.h"

class MLController {
	private:
		float elev_rate;
		
		uint8_t agent_system;
		uint8_t agent_component;
		mavlink_channel_t agent_channel;
	
	public:
		// Set the agent system, component and channel.
		void lookup_agent();
		
		// Send position to agent
		void send_state();
		
		// Get an updated elevator position
		int16_t get_elevator_output(float timestep);
		
		// Process incoming message from ML agent
		void process_msg(mavlink_message_t message);
		
	};
