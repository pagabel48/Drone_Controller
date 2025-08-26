#include "Control/pid.hpp"

float PID::update(float error, float d_time){

	float p_term = error * k_p;

	integral += error * d_time;

	// clamp
	if (integral > u_threshold) {
		integral = u_threshold;
	} else if (integral < l_threshold) {
		integral = l_threshold;
	}

	float i_term = integral * k_i;

	float d_term = k_d * (error - past_error) / d_time;

	past_error = error;

	return p_term + i_term + d_term;
}
