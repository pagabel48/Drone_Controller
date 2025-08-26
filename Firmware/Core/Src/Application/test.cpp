#include "Application/test.hpp"

BMP280* barometer = nullptr;
ICM42670P* imu = nullptr;
Motors* m = nullptr;
AttitudeKalman Attkf;
AltitudeKalman Altkf;

bool imu_initialized = false;

void init_testing (I2C_HandleTypeDef* n_i2c_handle, TIM_HandleTypeDef *htim2) {
	barometer = new BMP280 (n_i2c_handle);
	imu = new ICM42670P (n_i2c_handle);
	m = new Motors (htim2);
}

bool test_imu_initialized() {
	return imu_initialized;
}

void set_imu_ready_test () {
	imu->set_sensor_ready();
}

void run_tests () {

	// give time for USB connection, message to verify connection
	for (int i = 0; i < 5; i++) {

		 std::cout << "Hello from STM32 #" << i << "\r\n";

		 HAL_Delay(1000);
	}

	//blink_twice();
	//blink_twice();

	// unit test matrix class
	//test_mat_mult ();
	//test_mat_add ();
	//test_mat_sub ();
	//test_mat_transpose();
	//test_mat_inverse ();

	test_IMU_Conn ();
	//test_Barometer_Conn ();

	bool IMU_status = test_IMU_Init ();
	//bool barometer_status = test_Barometer_Init ();

	test_motors ();

	if (IMU_status) {

		test_sensors ();
		test_Kalmans ();
	}
}

void blink_once () {

	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_SET);

	HAL_Delay(SEC);

	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

	HAL_Delay(2 * SEC);
}

void blink_twice () {

	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_SET);

	HAL_Delay(SEC / 2);

	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

	HAL_Delay(SEC / 2);

	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_SET);

	HAL_Delay(SEC / 2);

	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

	// 1 second off-padding at the end
	HAL_Delay(SEC + SEC / 2);
}

void test_motors () {

	std::cout << "\nTesting the motors\r\n";

	float on_signal[] = {100.0f, 100.0f, 100.0f, 100.0f};

	// turn status led on
	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_SET);

	m->init();

	m->update_duty_cycle(on_signal);

	// motors on for 3 seconds
	HAL_Delay(3 * SEC);

	float off_signal[] = {0.0f, 0.0f, 0.0f, 0.0f};
	m->update_duty_cycle(off_signal);

	// turn status led off
	HAL_GPIO_WritePin(STATUS_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

	std::cout << "\nMotor Test Finished\r\n";

	// 1 second delay for padding
	HAL_Delay(SEC);

	delete m;
}

void test_IMU_Conn () {
	if (imu->ping()) {

		blink_twice();
		std::cout << "\nIMU Properly Connected\r\n";

	} else {

		blink_once();
		std::cout << "\nIMU not connected\r\n";

	}
}

void test_Barometer_Conn () {
	if (barometer->ping()) {

		blink_twice();
		std::cout << "\nBarometer Properly Connected\r\n";

	} else {

		blink_once();
		std::cout << "\nBarometer not connected\r\n";

	}
}

bool test_IMU_Init () {
	if (imu->init()) {

		imu_initialized = true;

		blink_twice();
		std::cout << "\nIMU Properly Initialized\r\n";

		return true;

	} else {

		blink_once();
		std::cout << "\nFailure: IMU Initialization\r\n";

		return false;
	}
}

bool test_Barometer_Init () {
	if (barometer->init()) {

		blink_twice();
		std::cout << "\nBarometer Properly Initialized\r\n";

		return true;

	} else {

		blink_once();
		std::cout << "\nFailure: Barometer Initialization\r\n";

		return false;
	}
}

void test_sensors() {
	for (int i = 0; i < 1000; i++) {

		float time_unacc;
		float time_curr = HAL_GetTick() * M_S;

		uint32_t time_passed;

		while (!imu->is_sensor_ready()) {
			HAL_Delay(1);
		}

		motion_reading* raw_reading = imu->take_motion_reading(time_curr, time_unacc);
		//float height = barometer->read_pressure_meters(&time_passed);

		std::cout << "\r\nIMU (acc, gyro, time): ";
		std::cout << printf("%.4f", raw_reading[0].acc_x) << ", " << printf("%.4f", raw_reading[0].acc_y) << ", " << printf("%.4f", raw_reading[0].acc_z) << ", ";
		std::cout << printf("%.4f", raw_reading[0].ang_vel_x) << ", " << printf("%.4f", raw_reading[0].ang_vel_y) << ", " << printf("%.4f", raw_reading[0].ang_vel_z) << ", ";
		std::cout << printf("%.4f", raw_reading[0].elapsed_time) << "\r\n";

		//std::cout << "Barometer Altitude (Meters): : " << printf("%.4f", height) << "\r\n\n";

		HAL_Delay(SEC/2);
	}
}

void test_Kalmans() {

	while (1) {
		if (barometer->sensor_ready() && imu->is_sensor_ready()) {

			float time_unacc;
			float time_curr = HAL_GetTick() * M_S;

			uint32_t time_passed;

			motion_reading* raw_reading = imu->take_motion_reading(time_curr, time_unacc);
			float height = barometer->read_pressure_meters(&time_passed);

			// intermediate format to pass accelerometer data from sensor to state estimator
			float ang_vel[2];
			float acc[3];

			// time elapsed over period
			float dt;

			for (int i = 0; i < IMU_PACKET_LENGTH; i++) {
				// roll, pitch
				ang_vel[0] = raw_reading[i].ang_vel_x;
				ang_vel[1] = raw_reading[i].ang_vel_y;

				acc[0] = raw_reading[i].acc_x;
				acc[1] = raw_reading[i].acc_y;
				acc[2] = raw_reading[i].acc_z;

				dt = raw_reading[i].elapsed_time;

				Attkf.predict(ang_vel, dt);
				Altkf.predict(acc, ang_vel, dt);
			}

			Attkf.predict(time_unacc);
			Altkf.predict(time_unacc);


			Matrix<float> rot = Attkf.update(acc);
			Matrix<float> alt = Altkf.update(height);


			std::cout << "\nAltitude (Height, Velocity, Acceleration Bias): ";
			std::cout << printf("%.4f", rot(1, 1)) << ", ";
			std::cout << printf("%.4f", rot(1, 2)) << ", ";
			std::cout << printf("%.4f", rot(1, 3)) << "\r\n";

			std::cout << "\nAttitude (Roll, Pitch, Roll Bias, Pitch Bias): ";
			std::cout << printf("%.4f", rot(1, 1)) << ", ";
			std::cout << printf("%.4f", rot(1, 2)) << ", ";
			std::cout << printf("%.4f", rot(1, 3)) << ", ";
			std::cout << printf("%.4f", rot(1, 4))  << "\r\n";

		} else if (imu->is_sensor_ready()) {


			float time_unacc;
			float time_curr = HAL_GetTick() * M_S;

			motion_reading* raw_reading = imu->take_motion_reading(time_curr, time_unacc);

			// intermediate format to pass accelerometer data from sensor to state estimator
			float ang_vel[2];
			float acc[3];

			// time elapsed over period
			float dt;

			for (int i = 0; i < IMU_PACKET_LENGTH; i++) {
				// roll, pitch
				ang_vel[0] = raw_reading[i].ang_vel_x;
				ang_vel[1] = raw_reading[i].ang_vel_y;

				acc[0] = raw_reading[i].acc_x;
				acc[1] = raw_reading[i].acc_y;
				acc[2] = raw_reading[i].acc_z;

				dt = raw_reading[i].elapsed_time;

				Attkf.predict(ang_vel, dt);
				Altkf.predict(acc, ang_vel, dt);
			}

			Attkf.predict(time_unacc);
			Altkf.predict(time_unacc);


			Matrix<float> rot = Attkf.update(acc);

			std::cout << "\nAttitude (Roll, Pitch, Roll Bias, Pitch Bias): ";
			std::cout << printf("%.4f", rot(1, 1)) << ", ";
			std::cout << printf("%.4f", rot(1, 2)) << ", ";
			std::cout << printf("%.4f", rot(1, 3)) << ", ";
			std::cout << printf("%.4f", rot(1, 4))  << "\r\n";
		}
	}
}

void test_mat_mult(){
    Matrix<int> a (4, 2, {
        1, 2,
        2, 0,
        0, 2,
        2, 1
    });

    Matrix<int> b (2, 3, {
        2, 0, 2,
        1, 1, 2
    });

    Matrix<int> res (4, 3, {
        4, 2, 6,
        4, 0, 4,
        2, 2, 4,
        5, 1, 6
    });

    assert(a * b == res);
    std::cout << "\nMatrix multiplication successful \r\n";
}

void test_mat_add() {

    Matrix<int> a (2, 2, {
        1, 2,
        2, 0
    });

    Matrix<int> b (2, 2, {
        2, 0,
        1, 1
    });

    Matrix<int> res (2, 2, {
        3, 2,
        3, 1
    });

    assert(a + b == res);
    std::cout << "\nMatrix add successful \r\n";
}

void test_mat_sub(){

    Matrix<int> a (2, 2, {
        1, 2,
        2, 0
    });

    Matrix<int> b (2, 2, {
        2, 0,
        1, 1
    });

    Matrix<int> res (2, 2, {
       -1, 2,
        1, -1
    });

    assert (a - b == res);
    std::cout << "\nMatrix subtraction successful \r\n";
}

void test_mat_transpose() {

    Matrix<int> a (3, 2, {
        1, 1,
        2, 0,
        2, 0
    });

    Matrix<int> res (2, 3, {
        1, 2, 2,
        1, 0, 0
    });

    assert (a.transpose() == res);
    std::cout << "\nMatrix transpose successful \r\n";
}

void test_mat_inverse(){


    Matrix<float> a (3, 3, {
        1, 2, 3,
        0, 1, 4,
        5, 6, 0
    });

    Matrix<float> res (3, 3, {
       -24,  18, 5,
        20, -15, -4,
       -5,   4,  1
    });

    assert (a.inverse() == res);
    std::cout << "\nMatrix inverse successful \r\n";
}
