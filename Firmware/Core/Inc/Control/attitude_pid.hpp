/*
 * Attitude PID Controller
 *
 * Created by Paul Gabel
 * Date: 7/27/2025
 *
 * */

#ifndef ATTITUDE_PID_HPP
#define ATTITUDE_PID_HPP

#include "stm32f4xx_hal.h"

#include "Control/pid.hpp"

/*
 * Defines
 * */
#define INNER_PID_GAIN		 					10

#ifdef __cplusplus

/*
 * controller for attitude
 *
 * input -> Outer PID -> Inner PID (10x faster) -> motor output
 * Angle Error -> Angle PID -> Vel. PID -> motor output  (0% - 100%)
 * 								  ^
 * 								  |
 * Current Angular Velocity ------
 *
 * */
class AttitudePID {
public:

	AttitudePID(const pid_params& rate_vars, const pid_params& angle_vars) :

		rate_controller(rate_vars),
		angle_controller(angle_vars),

		count (1)
	{}

	/*
	 * generate a new pid output
	 *
	 * param desired_angle: angle we would like to hold
	 * param current_angle: current angle drone is at
	 * param angular_vel: current angular velocity of drone
	 * param d_time: time passed since last update (seconds)
	 *
	 * returns new desired value
	 *
	 * */
	float update(float desired_angle, float current_angle, float current_angular_vel, float d_time);

private:
	// controllers
	PID rate_controller, angle_controller;

	uint8_t count;

	// holds desired angle from outer PID
	float desired_ang_vel;
};

#endif
#endif
