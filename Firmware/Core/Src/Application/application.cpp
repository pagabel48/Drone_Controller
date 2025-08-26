#include "Application/application.hpp"

void Application::init() {

	uint8_t IMU_Tries = 9;
	uint8_t Barometer_Tries = 9;

	receiver.init();
	motors.init();

	// try 10 times to initialize sensors
	uint8_t IMU_Status = IMU.init();

	while (IMU_Status > 0 && IMU_Tries > 0) {

		IMU_Status = IMU.init();
		IMU_Tries--;
	}

	uint8_t Barometer_Status = Barometer.init();

	while (Barometer_Status > 0 && Barometer_Tries > 0) {

		Barometer_Status = Barometer.init();
		Barometer_Tries--;
	}

	// if sensors failed to initialize nop loop, alert over USB every second
	while (IMU_Status > 0 || Barometer_Status > 0) {

		if (IMU_Status > 0) {
			std::cout << "\nError Initializing IMU\r\n";
		}
		if (Barometer_Status > 0) {
			std::cout << "\nError Initializing Barometer\r\n";
		}
		HAL_Delay(1000);
	}

	// turn on power CTRL GPIO
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	std::cout << "Software Started!\r\n";

	// turn on status light
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

	battery_level = read_battery(adc_handle);

	while (true) {
		loop();
	}
}

void Application::loop(){

	// update battery reading, LPF is applied to account for noise
	battery_level = (1 - BATTERY_FILTER_CONST) * battery_level + BATTERY_FILTER_CONST * read_battery(adc_handle);

	// turn off drone if battery is below level, set battery CTRL pin to low
	// circuitry overrides this command if plugged into USB
	if (battery_level < VOLTAGE_MIN) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	}

	// get receiver input
	float* raw_input = receiver.get_duty_cycle();

	// depending on which inputs are available update control system
	if (Barometer.sensor_ready() && IMU.is_sensor_ready()) {

		// holds unaccounted time from waiting after IMU burst read, processing, etc.
		float time_unacc;
		float time_curr = HAL_GetTick() * MS_S;

		motion_reading* imu_reading = IMU.take_motion_reading(time_curr, time_unacc);

		// Barometer.sensor_ready() goes to false once reading is taken
		uint32_t time_passed_barometer;
		float pressure = Barometer.read_pressure_meters(&time_passed_barometer);

		float* signals = controller.update(imu_reading, time_unacc, pressure, raw_input);

		motors.update_duty_cycle(signals);

	} else if (IMU.is_sensor_ready()) {

		// holds unaccounted time from waiting after IMU burst read, processing, etc.
		float time_unacc;
		float time_curr = HAL_GetTick() * MS_S;

		motion_reading* imu_reading = IMU.take_motion_reading(time_curr, time_unacc);

		float* signals = controller.update(imu_reading, time_unacc, raw_input);

		motors.update_duty_cycle(signals);

	}
}

float Application::read_battery(ADC_HandleTypeDef* adc_handle) {
	// wait until new reading is ready and store it
	HAL_ADC_PollForConversion(adc_handle, HAL_MAX_DELAY);
	uint32_t adc_value = HAL_ADC_GetValue(adc_handle);

	return (adc_value / ADC_RESOLUTION) * REF_VOLTAGE * BATTERY_SCALE;
}

// reroute std::cout to print over usb, plans to extend to also print over uart and SPI
extern "C" {
int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        return -1;
    }

    const uint32_t timeout_ms = 1000;
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS(reinterpret_cast<uint8_t*>(data), len) == USBD_BUSY) {
        if ((HAL_GetTick() - start) > timeout_ms) {

        	// timeout
            //return -1;
        }
    }

    return len;
}
}

// entry point for application, called in main.c
extern "C" void start (I2C_HandleTypeDef *n_i2c_handle, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, ADC_HandleTypeDef* adc_handle_p) {

	// comment out for testing
	//Application app (n_i2c_handle, htim2, htim3, adc_handle_p);
	//app_ptr = &app;
	//app.init();

	// uncomment for testing

	init_testing(n_i2c_handle, htim2);
	run_tests ();
}

// HAL callback for EXTI12 interrupt
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12 && app_ptr) {
    	// if object instantiated modify it
    	// comment for testing
        //app_ptr->set_accelerometer_r();
    }

    if (GPIO_Pin == GPIO_PIN_12 && test_imu_initialized()) {
        // testing function
    	// uncomment for testing
        set_imu_ready_test();
    }
}
