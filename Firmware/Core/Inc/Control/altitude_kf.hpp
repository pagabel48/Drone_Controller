/*
 * Altitude Kalman Filter
 *
 * Created by Paul Gabel
 * Date: 7/28/2025
 *
 * */

#ifndef ALTITUDE_EKF_HPP
#define ALTITUDE_EKF_HPP

#include "stm32f4xx_hal.h"
#include "matrix.hpp"

// gravitational constant, m/2^s
#define G									9.81f

// tuned value, scale for angle uncertainty when no measurement input present
#define N_G									0.5f

// calculated from imu datasheet
#define ACC_VAR								6.25e-4f

// found through tuning, models random walk
#define ACC_DRIFT_VAR						5.0e-8f

// combination of accuracy and noise, from barometer datasheet
#define MEAS_VAR							0.12

#ifdef __cplusplus

#include <cmath>

using std::sin;
using std::cos;
using std::sqrt;
using std::abs;
using std::pow;

/*
 * uses an Extended Kalman Filter to fuse accelerometer and barometer readings to estimate altitude and vertical velocity
 *
 * altitude is represented in meters, velocity in m/s
 *
 * */
class AltitudeKalman {
public:

	AltitudeKalman()
	:
		n(3),

		x (n, 1),
		P (n, n),
		K (n, 1),

		H (1, n, { 1, 0, 0 }),
		R (1, 1, {MEAS_VAR}),


		I (n, n, {
				1,	0,	0,
				0,	1,	0,
				0,	0,	1
			}),

		Q (n, n),
		u (1, 1),
		F (n, n),
		B (n, 1)

	{}

	/*
	 * predict using no measurement input, for time syncronization
	 *
	 * PARAM d_time: time since last predict
	 * * d_time must be in seconds
	 *
	 * returns the state vector
	 *
	 * */
	Matrix<float> predict(float d_time);

	/*
	 * predict using acceleration as a control input
	 * acceleration in z direction is calculated using trig, and approximation is used
	 * to account for non-linearity in acceleration bias
	 * this gives much better performance than using a jacobian
	 *
	 * PARAM acc: array containing the three acceleration values [x, y, z]
	 * PARAM pitch: current pitch value
	 * PARAM roll: current roll value
	 * * acceleration must be in m/s^2, pitch and roll must be in radians
	 *
	 * PARAM d_time: time since last predict
	 * * d_time must be in seconds
	 *
	 * returns the state vector
	 *
	 * */
	Matrix<float> predict(float* acc, float* rot, float d);

	/*
	 * performs Kalman update step using barometer height reading
	 *
	 * PARAM altitude: altitude in meters
	 *
	 * returns the state vector
	 *
	 * */
	Matrix<float> update(float altitude);

private:

	// length of state matrix
	int n;

	/*
	 * state vector, dynamic
	 *
	 * height
	 * velocity
	 * accelerometer drift
	 *
	 * */
	Matrix<float> x;

	// state estimation error, dynamic
	Matrix<float> P;

	// Kalman gain
	Matrix<float> K;

	// Jacobian
	Matrix<float> H;

	// measurement estimation error
	Matrix<float> R;

	// identity matrix
	Matrix<float> I;

	// state estimation error, dynamic
	Matrix<float> Q;

	// control matrix, holds gyroscope reading
	Matrix<float> u;

	// state translation matrix
	Matrix<float> F;

	// control translation matrix
	Matrix<float> B;
};

#endif
#endif
