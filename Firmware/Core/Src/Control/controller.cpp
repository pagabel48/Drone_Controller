#include "Control/controller.hpp"

float* Controller::update (motion_reading* imu_r, float time_unacc, float* raw_input) {

	// intermediate format to pass accelerometer data from sensor to state estimator
	float ang_vel[2];
	float acc[3];

	// time elapsed over period
	float dt;

	for (int i = 0; i < IMU_BURST_LENGTH; i++) {
		// roll, pitch
		ang_vel[0] = imu_r[i].ang_vel_x;
		ang_vel[1] = imu_r[i].ang_vel_y;

		acc[0] = imu_r[i].acc_x;
		acc[1] = imu_r[i].acc_y;
		acc[2] = imu_r[i].acc_z;

		dt = imu_r[i].elapsed_time;

		rotation_state.predict(ang_vel, dt);
		altitude_state.predict(acc, ang_vel, dt);
	}

	// perform update in the past when reading was taken
	Matrix<float> rot = rotation_state.update(acc);

	// make up for unaccounted time between burst reads
	rotation_state.predict(time_unacc);
	altitude_state.predict(time_unacc);

	// set dt to total time elapsed over period
	float new_time = HAL_GetTick()* MS_S;
	dt = new_time - last_time;

	last_time = new_time;
	alt_skip_time += dt;

	// scale controller input (-1, 1) to set points
	convert_receiver_input (raw_input, dt);

	roll_c = roll_ctrl.update(setpoint.desired_roll, rot(1, 1), imu_r[IMU_BURST_LENGTH - 1].ang_vel_x, dt);
	pitch_c = pitch_ctrl.update(setpoint.desired_pitch, rot(1, 2), imu_r[IMU_BURST_LENGTH - 1].ang_vel_y, dt);

	return mixer();
}

float* Controller::update (motion_reading* imu_r, float time_unacc, float height, float* raw_input) {

	// intermediate format to pass accelerometer data from sensor to state estimator
	float ang_vel[2];
	float acc[3];

	// time elapsed over period
	float dt;

	for (int i = 0; i < IMU_BURST_LENGTH; i++) {
		// roll, pitch
		ang_vel[0] = imu_r[i].ang_vel_x;
		ang_vel[1] = imu_r[i].ang_vel_y;

		acc[0] = imu_r[i].acc_x;
		acc[1] = imu_r[i].acc_y;
		acc[2] = imu_r[i].acc_z;

		dt = imu_r[i].elapsed_time;

		rotation_state.predict(ang_vel, dt);
		altitude_state.predict(acc, ang_vel, dt);
	}

	// perform update in the past when reading was taken
	Matrix<float> rot = rotation_state.update(acc);

	// make up for unaccounted time between burst reads
	rotation_state.predict(ang_vel, time_unacc);
	altitude_state.predict(acc, ang_vel, time_unacc);

	Matrix<float> alt = altitude_state.update(height);

	// set dt to total time elapsed over period
	float new_time = HAL_GetTick()* MS_S;
	dt = new_time - last_time;

	last_time = new_time;

	// scale controller input (-1, 1) to set points
	convert_receiver_input (raw_input, dt);

	roll_c = roll_ctrl.update(setpoint.desired_roll, rot(1, 1), imu_r[IMU_BURST_LENGTH - 1].ang_vel_x, dt);
	pitch_c = pitch_ctrl.update(setpoint.desired_pitch, rot(1, 2), imu_r[IMU_BURST_LENGTH - 1].ang_vel_y, dt);

	alt_c = altitude_ctrl.update(setpoint.desired_height - alt(1, 1), dt + alt_skip_time);

	alt_skip_time = 0;

	return mixer();
}

void Controller::convert_receiver_input(float* raw_input, float dt) {

	// update height setpoint
	setpoint.desired_height += raw_input[0] * HEIGHT_INPUT_SENSITIVITY * dt;

	setpoint.desired_roll = raw_input[1] * ANGLE_INPUT_SENSITIVITY;
	setpoint.desired_pitch = raw_input[2] * ANGLE_INPUT_SENSITIVITY;

}

float* Controller::mixer(){

	float angle_weight = (1.0f - MIXING_CONST) / 2.0f;

	// set throttle component
	float th_base = alt_c * MIXING_CONST;
	// motor 1 2 3 4
	Matrix<float> motors = Matrix<float>(1, 4, {th_base, th_base, th_base, th_base});

	// set roll component
	motors = motors + roll_c * angle_weight * roll_mixing;

	// set pitch component
	motors = motors + pitch_c * angle_weight * pitch_mixing;

	// decompact and return
	float* ret = new float[]{motors(1,1), motors(1,2), motors(1,3), motors(1,4)};
	return ret;
}
