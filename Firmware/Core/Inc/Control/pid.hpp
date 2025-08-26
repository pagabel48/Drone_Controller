/*
 * PID Controller
 *
 * Created by Paul Gabel
 * Date: 7/27/2025
 *
 * */

#ifndef PID_HPP
#define PID_HPP

#include "stm32f4xx_hal.h"

struct pid_params {
	float lower_threshold;
	float upper_threshold;

	float p_const, i_const, d_const;
};

#ifdef __cplusplus

class PID {
public:
	/*
	 * set up variables
	 *
	 * params: all mapped to private variables, explained below
	 *
	 * */
	PID(const pid_params& vars) :
		l_threshold(vars.lower_threshold),
		u_threshold(vars.upper_threshold),

		k_p(vars.p_const),
		k_i(vars.i_const),
		k_d(vars.d_const),

		past_error(0),
		integral(0)
		{}

	/*
	 * generate a new PID output, applies integral clamping
	 *
	 * param error: error
	 * param d_time: time passed since last update (seconds)
	 *
	 * returns new output
	 *
	 * */
	float update(float error, float d_time);

private:
	// clamping thresholds
	float l_threshold, u_threshold;

	// coefficients for each term
	float k_p, k_i, k_d;

	// dynamic, for d and i terms
	float past_error, integral;

};

#endif
#endif
