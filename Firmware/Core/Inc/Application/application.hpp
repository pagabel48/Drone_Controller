#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#ifdef __cplusplus
#include <cstring>
#include <unistd.h>
#endif

// stm32 libraries
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

// firmware components
#include "Sensors/BMP280.hpp"
#include "Sensors/ICM42670P.hpp"

#include "Control/controller.hpp"

#include "motors.hpp"
#include "receiver.hpp"

// testing library
#include "Application/test.hpp"

#define STATUS_LED_PIN 						GPIO_PIN_12
#define STATUS_GPIO_PORT 					GPIOB

// minimum voltage of the LIPO battery before we turn it off
#define	VOLTAGE_MIN							14.8f

// account for voltage divider (to scale battery voltage to be less than 3.3v
#define BATTERY_SCALE						7.66f

// low pass filter weight
#define BATTERY_FILTER_CONST				0.2f

// ADC variables
#define ADC_RESOLUTION						4095.0f
#define REF_VOLTAGE							3.3f

// conversion factor milliseconds to seconds
#define MS_S								0.001f

#ifdef __cplusplus
extern "C" {
#endif

void setup(I2C_HandleTypeDef* n_i2c_handle);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Application {
public:

	/*
	 * constructor
	 *
	 * Application class
	 *
	 * */
	Application(I2C_HandleTypeDef* n_i2c_handle, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, ADC_HandleTypeDef* adc_handle_p) :
		Barometer(n_i2c_handle),
		IMU(n_i2c_handle),

		receiver(htim3),
		motors(htim2),

		controller(),

		adc_handle(adc_handle_p)
	{
		init();
	}

	void set_accelerometer_r () {
		IMU.set_sensor_ready();
	}

	/*
	 * init function, starts main loop
	 *
	 * */
	void init();

private:

	/*
	 * main loop run repeatedly, takes sensor readings and uses components to produce
	 * control outputs and run engines
	 *
	 * */
	void loop ();

	/*
	 * reads the battery input GPIO using ADC, scales in voltage
	 *
	 * returns the reading
	 *
	 * */
	float read_battery (ADC_HandleTypeDef* adc_handle);

	BMP280 Barometer;
	ICM42670P IMU;

	Receiver receiver;
	Motors motors;

	Controller controller;

	ADC_HandleTypeDef* adc_handle;

	float battery_level;
};

// singleton reference to application, so that its state can be modified via interrupt
static Application* app_ptr = nullptr;

/*
 * Sends a message over usb virtual com port
 *
 * msg is char array to be sent
 * */
void sendMessage(const char* msg);

#endif
#endif
