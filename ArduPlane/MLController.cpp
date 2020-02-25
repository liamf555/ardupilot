#include "Plane.h"

void Plane::update_ml_controller(void) {
	g2.mlController.send_state();
	}

#include "MLController.h"
#include <SRV_Channel/SRV_Channel.h>

// Parameter info
const struct AP_Param::GroupInfo MLController::var_info[] = {
	
	// @Param: MLCTRL_OUT2DEG
    // @DisplayName: Output conversion factor
    // @Description: Set the conversion between AP scaled output and physical degrees.
    // @Range: 0 inf
    // @Increment: 0.1
    // @User: User
	AP_GROUPINFO("OUT2DEG", 1, MLController, output2deg, 1.0),
	
	// @Param: MLCTRL_DEG2OUT
    // @DisplayName: Inverse output conversion factor
    // @Description: Set the conversion between physical degrees and AP scaled output. Should be reciprocal of OUT2DEG.
	// @Range: 0 inf
    // @Increment: 0.1
    // @User: User
	AP_GROUPINFO("DEG2OUT", 2, MLController, deg2output, 1.0),
	
	AP_GROUPEND
	};

MLController::MLController()
	: elev_rate(0.0),
	  elevatorAngle(0.0),
	  agent_system(-1),
	  agent_component(-1),
	  agent_channel(mavlink_channel_t(-1)),
	  lookupSuccess(false),
	  elevatorAngleUninitialised(true)
	{
	}

void MLController::lookup_agent() {
	// Lookup agent in routing table
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "Doing lookup...");
#endif
	lookupSuccess = gcs().chan(0).find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, agent_system, agent_component, agent_channel);
	}

void MLController::send_state() {
	// Lookup the agent in the routing table. Caching?
	lookup_agent();
	
	// Setup struct
	mavlink_mlagent_state_t state;
	
	// Fill out structure
	
	Vector3f position;
	plane.ahrs.get_relative_position_NED_home(position); // Result is NED (m) relative to home
	state.x = position.x;
	state.y = position.y;
	state.z = position.z;
	
	// Euler angles are in radians
	state.phi = plane.ahrs.roll;
	state.theta = plane.ahrs.pitch;
	state.psi = plane.ahrs.yaw;

	Vector3f velocity;
	plane.ahrs.get_velocity_NED(velocity); // Result is NED (m/s)
	// Convert velocity to body axes
	Matrix3f ned2body = plane.ahrs.get_rotation_body_to_ned().transposed(); // Get inverse DCM
	Vector3f velocity_b = ned2body * velocity; // Velocity in body frame
	state.u = velocity_b.x;
	state.v = velocity_b.y;
	state.w = velocity_b.z;
	
	Vector3f angular_rates = plane.ahrs.get_gyro_latest(); // Vector of rotational rates in rad/s (Assume phi,theta,psi)
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
	if( lookupSuccess ) {
		mavlink_msg_mlagent_state_send_struct(agent_channel,&state);
#ifdef MLDEBUG
		gcs().send_text(MAV_SEVERITY_INFO, "MLAGENT_STATE sent to %i", agent_channel);
#endif
		}
	}


void MLController::reset() {
	elevatorAngleUninitialised = true;
	}


int16_t MLController::get_elevator_output(float timestep) {
	if( elevatorAngleUninitialised ) {
		// Don't have current elevatorAngle, get it
		int16_t elevatorOutput = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
		// Convert to angle
		elevatorAngle = elevatorOutput * output2deg;
		elevatorAngleUninitialised = false;
		}
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Output: %i Angle: %f", elevatorOutput, elevatorAngle);
#endif
	// Compute updated angle
	elevatorAngle = elevatorAngle + elev_rate * timestep;
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Angle: %f Return %i", elevatorAngle, int16_t(elevatorAngle*deg2output));
#endif
	// Convert to scaled output
	return elevatorAngle * deg2output;
	// Rounding could stop the output from changing...
	}

void MLController::handle_message(mavlink_message_t* message) {
	mavlink_mlagent_action_t action_msg;
	mavlink_msg_mlagent_action_decode(message, &action_msg);
	
	// Update rate based on message from agent
	elev_rate = action_msg.elevator;
	}
