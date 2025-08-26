/*
 * Attitude Kalman Filter
 *
 * Created by Paul Gabel
 * Date: 7/28/2025
 *
 * */

#ifndef ATTITUDE_EKF_HPP
#define ATTITUDE_EKF_HPP

#include "stm32f4xx_hal.h"
#include "matrix.hpp"

// gravitational constant, m/s^2
#define G									9.81f

// tuned value, scale for angle uncertainty when no measurement input present
#define N_G									0.5f

// calculated from IMU datasheet
#define ANGLE_VAR							1.493e-6f
#define ACC_VAR								6.25e-4f

// random walk, found through trial and error
#define ANG_DRIFT_VAR							5e-8f

// tuning constant for adaptive noise scaling
#define ADAPT_SCALE_CONST					50

#ifdef __cplusplus

#include <cmath>

using std::sin;
using std::cos;
using std::sqrt;
using std::abs;
using std::pow;

/*
 * uses an Extended Kalman Filter to fuse accelerometer and gyroscope readings, and estimate roll and pitch angle
 *
 * angles are represented in radians
 *
 * */
class AttitudeKalman {
public:

	AttitudeKalman()
	:
		n(4),
		m1(3),
		m2(2),

		x (n, 1),
		P (n, n),
		K (n, m1),

		H (m1, n, { 1, 0, 0 }),
		R (m1, m1),


		I (n, n, {
				1,	0,	0, 0,
				0,	1,	0, 0,
				0,	0,	1, 0,
				0,	0,	0, 1
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
	 * predict using angular velocity as a control input (so we can approximate as linear)
	 * this assumes that gyroscope error is negligible, though drift is accounted for though bias
	 *
	 * PARAM vel: roll, pitch
	 * * velocity must be is rad./sec.
	 *
	 * PARAM d_time: time since last predict
	 * * d_time must be in seconds
	 *
	 * returns the state vector
	 *
	 * */
	Matrix<float> predict(float* vel, float d_time);

	/*
	 * performs Kalman update step, uses adaptive measurement noise co-variance for more accuracy
	 *
	 * PARAM x_acc: acceleration in the x
	 * PARAM y_acc: acceleration in the y
	 * PARAM z_acc: acceleration in the z
	 * * acceleration must be in m/s^2
	 *
	 * returns the state vector
	 *
	 * */
	Matrix<float> update(float* acc);

private:

	// state vector length
	int n;

	// measurement vector length, accelerometer xyz
	int m1;
	// gyroscope, roll + pitch
	int m2;

	/*
	 * state vector, dynamic
	 *
	 * roll
	 * pitch
	 * roll bias
	 * pitch bias
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
