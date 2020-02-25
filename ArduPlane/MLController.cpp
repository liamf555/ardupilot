#include "Plane.h"

void Plane::update_ml_agent(void) {
	mlController.send_state();
	}

#include "MLController.h"

#define OUTPUT_TO_DEG 0.1
#define DEG_TO_OUTPUT 10.0

#include <SRV_Channel/SRV_Channel.h>

void MLController::lookup_agent() {
	// Lookup agent in routing table
	gcs().chan(0).find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, agent_system, agent_component, agent_channel);	
	}

void MLController::send_state() {
	// Setup struct
	mavlink_mlagent_state_t state;
	
	// Fill out structure
	
	Vector3f position;
	plane.ahrs.get_relative_position_NED_home(position);
	state.x = position.x;
	state.y = position.y;
	state.z = position.z;
	
	state.phi = plane.ahrs.roll;
	state.theta = plane.ahrs.pitch;
	state.psi = plane.ahrs.yaw;

	Vector3f velocity;
	plane.ahrs.get_velocity_NED(velocity);
	// Convert velocity to body axes
	Matrix3f ned2body = plane.ahrs.get_rotation_body_to_ned().transposed();
	Vector3f velocity_b = ned2body * velocity;
	state.u = velocity_b.x;
	state.v = velocity_b.y;
	state.w = velocity_b.z;
	
	Vector3f angular_rates = plane.ahrs.get_gyro_latest();
	state.p = angular_rates.x;
	state.q = angular_rates.y;
	state.r = angular_rates.z;

	state.sweep = 0.0;
	state.elevator = 0.0;
	state.tip = 0.0;
	
	// Set targets to match looked-up agent
	state.target_system = agent_system; // 1 for the aircraft?
	state.target_component = agent_component; // ?? For the ML agent?
	
	// Send message to agent
	mavlink_msg_mlagent_state_send_struct(agent_channel,&state);
	}

int16_t MLController::get_elevator_output(float timestep) {
	// Get current value
	int16_t elevatorOutput = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
	// Convert to angle
	float elevatorAngle = elevatorOutput * OUTPUT_TO_DEG;
	// Compute updated angle
	elevatorAngle = elevatorAngle + elev_rate * timestep;
	// Convert to scaled output
	return elevatorOutput * DEG_TO_OUTPUT;
	}

void MLController::process_msg(mavlink_message_t message) {
	
	}
