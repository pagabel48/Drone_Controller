#include "Control/attitude_kf.hpp"

Matrix<float> AttitudeKalman::predict(float d_time){

	// pass angles through, subtract bias
	F  = Matrix<float> (n, n, {
			1, 		0, 	   -d_time, 	0,
			0, 		1, 		0, 		   -d_time,
			0, 		0, 		1, 			0,
			0, 		0, 		0, 			1
	});

	Q  = Matrix<float> (n, n, {
			ANGLE_VAR * d_time * N_G, 		0, 							0,				0,
			0, 							ANGLE_VAR * d_time * N_G, 		0,				0,
			0, 							0, 								ANG_DRIFT_VAR, 		0,
			0, 							0, 								0, 				ANG_DRIFT_VAR
	});

	Q = Q * d_time;

	// update state
	x = F * x;

	// update uncertainty
	P = F * P * F.transpose() + Q;

	return x;
}

Matrix<float> AttitudeKalman::predict(float* vel, float d_time){

	// create control input vector
	// roll, pitch
	u  = Matrix<float> (m2, 1, vel);

	// create control translation Matrix
	// add distance rotated over d_time
	B  = Matrix<float> (n, m2, {
			d_time, 0,
			0, 		d_time,
			0, 		0,
			0, 		0
	});

	// pass angles through, subtract bias
	F  = Matrix<float> (n, n, {
			1, 		0, 	   -d_time, 	0,
			0, 		1, 		0, 		   -d_time,
			0, 		0, 		1, 			0,
			0, 		0, 		0, 			1
	});

	Q  = Matrix<float> (n, n, {
			ANGLE_VAR * d_time, 		0, 							0,				0,
			0, 							ANGLE_VAR * d_time, 		0,				0,
			0, 							0, 							ANG_DRIFT_VAR, 		0,
			0, 							0, 							0, 				ANG_DRIFT_VAR
	});

	Q = Q * d_time;

	// update state
	x = F * x + B * u;

	// update uncertainty
	P = F * P * F.transpose() + Q;

	return x;
}

Matrix<float> AttitudeKalman::update(float* acc){

	// create measurement vector
	// x, y, z
	Matrix<float> z (m1, 1, acc);

	float roll = x(0, 0);
	float pitch = x(1, 0);

	// jacobian to propagate acceleration uncertainty into attitude
	H  = Matrix<float> (m1, n, {
			0,								-G * cos(pitch),					0,		0,
			G * cos(roll) * cos(pitch),		-G * sin(roll) * sin(pitch),		0,		0,
		   -G * sin(roll) * cos(pitch),		-G * cos(roll) * sin(pitch),		0,		0,
	});

	R  = Matrix<float> (m1, m1, {
			1, 0, 0,
			0, 1, 0,
		    0, 0, 1
	});

	/*
	 * apply adaptive noise scaling to R matrix
	 * pay less attention to accelerometer trig the more acceleration magnitude deviates from 1
	 *
	 * */
	float acc_mag = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
	float acc_dev = abs(acc_mag - 1);
	float alpha = 1 + ADAPT_SCALE_CONST * (acc_dev * acc_dev);

	R = R * ACC_VAR * alpha;

	// compute gain
	K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

	// update state
	x = x + K * (z - H * x);

	// update uncertainty
	P = (I - K * H) * P;


	return x;
}
