#include "Control/altitude_kf.hpp"

Matrix<float> AltitudeKalman::predict(float dt){

	float dt_2 = dt * dt;

	// pass angles through, subtract accelerometer bias
	F  = Matrix<float> (n, n, {
			1, 		dt,	   -0.5f * dt_2,
			0, 		1, 	   -dt,
			0, 		0, 		1
	});

	Q = Matrix<float> (n, n, {
			ACC_VAR * N_G * dt * dt_2 / 3.0f, 		ACC_VAR * N_G * dt_2 / 2.0f, 		0,
			ACC_VAR * N_G * dt_2 / 2.0f, 			ACC_VAR * N_G * dt, 				0,
			0, 										0, 									ACC_DRIFT_VAR * dt
	});

	// update state
	x = F * x ;

	// update uncertainty
	P = F * P * F.transpose() + Q;

	return x;
}

Matrix<float> AltitudeKalman::predict(float* acc, float* rot, float dt){

	float roll = rot[0];
	float pitch = rot[1];

	float dt_2 = dt * dt;
	float vel = -acc[0] * sin(pitch) + acc[1] * cos(pitch) * sin(roll) + acc[2] * cos(pitch) * cos(roll) - G;

	// create control input vector
	// height, velocity, acceleration
	u  = Matrix<float> (1, 1, &vel);

	// create control translation Matrix
	// add distance and velocity added from acceleration
	B  = Matrix<float> (n, 1, {
			0.5f * dt_2,
			dt,
			0
	});

	// pass angles through, subtract accelerometer bias
	F  = Matrix<float> (n, n, {
			1, 		dt,	   -0.5f * dt_2,
			0, 		1, 		   -dt,
			0, 		0, 			1
	});

	// approximate, increase uncertainty of acceleration as angles increase, nonlinearities in trig
	float drift_gain = 1 + pow(sin(roll), 2) + pow(sin(pitch), 2);

	Q = Matrix<float> (n, n, {
			ACC_VAR * dt * dt_2 / 3.0f, 		ACC_VAR * dt_2 / 2.0f, 		0,
			ACC_VAR * dt_2 / 2.0f, 				ACC_VAR * dt, 				0,
			0, 									0, 							ACC_DRIFT_VAR * dt * drift_gain
	});

	// update state
	x = F * x + B * u;

	// update uncertainty
	P = F * P * F.transpose() + Q;

	return x;
}

Matrix<float> AltitudeKalman::update(float altitude){

	// create measurement vector
	// x, y, z
	Matrix<float> z = Matrix<float> (1, 1, &altitude);

	// compute gain
	K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

	// update state
	x = x + K * (z - H * x);

	// update uncertainty
	P = (I - K * H) * P;


	return x;
}
