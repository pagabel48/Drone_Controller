/*
 * Altitude Kalman Filter
 *
 * Created by Paul Gabel
 * Date: 7/29/2025
 *
 * */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "stm32f4xx_hal.h"

#include "matrix.hpp"
#include "Sensors/ICM42670p.hpp"
#include "Control/pid.hpp"
#include "Control/attitude_pid.hpp"
#include "Control/attitude_kf.hpp"
#include "Control/altitude_kf.hpp"

/*
 * defines
 *
 * */
//bounds for clamping
#define OUTER_LOW_BOUND							0.0f
#define OUTER_UPPER_BOUND						100.0f

#define ANG_VEL_LOW_BOUND					   -50.0f
#define ANG_VEL_UPPER_BOUND						50.0f

/*
 * PID Parameters found through tuning
 *
 * */
// altitude PID parameters
#define ALT_KP									1.0f
#define ALT_KI									1.0f
#define ALT_KD									1.0f

// angular velocity PID parameters
#define VEL_KP									1.0f
#define VEL_KI									1.0f
#define VEL_KD									1.0f

// angle PID parameters
#define ANGLE_KP								1.0f
#define ANGLE_KI								1.0f
#define ANGLE_KD								1.0f

// how much weight to give altitude correction vs. pitch and roll
// pitch, roll weight = (1- MIXING_CONST) / 2
#define MIXING_CONST							0.7f

// range of angle inputs, -45 to 45 degrees in radian
#define ANGLE_INPUT_SENSITIVITY					45.0f

// 3 meters per second possible set point change
#define HEIGHT_INPUT_SENSITIVITY				3.0f

// length of imu burst read
#define IMU_BURST_LENGTH						10

// conversion factor milliseconds to seconds
#define MS_S									0.001f

// holds desired set points from controller
struct control_input {
	float desired_pitch;
	float desired_roll;
	float desired_height;
};

#ifdef __cplusplus

#include <cmath>

/*
 * implements PID and Kalman filter classes
 * ties up loose ends
 *
 * */
class Controller {
public:

	Controller() :

		setpoint(),

		ang_vel_pid_params {
				ANG_VEL_LOW_BOUND,
				ANG_VEL_UPPER_BOUND,
				VEL_KP,
				VEL_KI,
				VEL_KD
		},
		angle_pid_params {
				OUTER_LOW_BOUND,
				OUTER_UPPER_BOUND,
				ANGLE_KP,
				ANGLE_KI,
				ANGLE_KD
		},
		alt_pid_params {
				OUTER_LOW_BOUND,
				OUTER_UPPER_BOUND,
				ALT_KP,
				ALT_KI,
				ALT_KD
		},

		roll_mixing (1, 4, { -1, 1, 1, -1 }),
		pitch_mixing (1, 4, { -1, -1, 1, 1 }),

		altitude_ctrl (alt_pid_params),

		roll_ctrl (ang_vel_pid_params, angle_pid_params),
		pitch_ctrl (ang_vel_pid_params, angle_pid_params),

		alt_skip_time(0)
	{
		// get time in seconds
		last_time = HAL_GetTick() * MS_S;
	}

	/*
	 * generate new motor output from sensor input
	 *
	 * PARAM imu_r: reading from the inertial measurement unit
	 *
	 * returns reference to 4 long array of motor strength percentages
	 *
	 * */
	float* update (motion_reading* imu_r, float time_unacc, float* raw_input);

	/*
	 * generate new motor output from sensor input
	 *
	 * PARAM imu_r: reading from the inertial measurement unit
	 * PARAM height: height reading from the barometer
	 *
	 * returns reference to 4 long array of motor strength percentages
	 *
	 * */
	float* update (motion_reading* imu, float time_unacc, float height, float* raw_input);

private:


	/*
	 * takes array of receiver stick positions and scales to setpoints
	 *
	 * returns a control input struct
	 *
	 * */
	void convert_receiver_input(float* raw_input, float dt);

	/*
	 * mixes the raw pid outputs in each axis to get final motor values
	 *
	 * returns array of desired motor output percentages, for motors 1 : 4
	 *
	 * */
	float* mixer();

	// holds desired setpoints
	control_input setpoint;

	// parameter structs for the pid loops
	pid_params ang_vel_pid_params;
	pid_params angle_pid_params;
	pid_params alt_pid_params;

	// convert desired roll and pitch adjustments to individual motors
	Matrix<float> roll_mixing;
	Matrix<float> pitch_mixing;

	// PID controllers
	PID altitude_ctrl;

	AttitudePID roll_ctrl;
	AttitudePID pitch_ctrl;

	// state estimation
	AttitudeKalman rotation_state;
	AltitudeKalman altitude_state;

	// holds desired motor control values
	float roll_c, pitch_c, alt_c;

	// time stamp of beginning of this measurement period, since IMU readings are burst read
	float last_time;

	// time for altitude update that is skipped due to barometer reading not being available
	float alt_skip_time;
};

#endif
#endif
