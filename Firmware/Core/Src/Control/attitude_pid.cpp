#include "Control/attitude_pid.hpp"

float AttitudePID::update(float desired_angle, float current_angle, float current_angular_vel, float d_time) {

	// run velocity controller multiple times for every angle controller run
	if (count >= INNER_PID_GAIN) {
		count = 0;

		float angle_error = desired_angle - current_angle;

		desired_ang_vel = angle_controller.update(angle_error, d_time);
	}

	float ang_vel_error = desired_ang_vel - current_angular_vel;

	count++;

	return rate_controller.update(ang_vel_error, d_time);
}
