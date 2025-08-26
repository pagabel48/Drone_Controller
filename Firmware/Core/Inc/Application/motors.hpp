/*
 * Motor ESC Controller
 *
 * Created by Paul Gabel
 * Date: 7/16/2025
 *
 * */

#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "stm32f4xx_hal.h"

/*
 * Defines
 * */
#define MOTOR_COUNT 4


#ifdef __cplusplus

class Motors {
public:

	Motors (TIM_HandleTypeDef *htim) :
		tim (htim)
	{}
	/*
	 * Initializes the motors
	 *
	 * parameter htim: reference to the PWM timer (typically 2)
	 *
	 * */
	void init ();

	/*
	 * Updates the duty cycle for each channel to change each motor's speed
	 *
	 * parameter signals: array containing each desired motor strength, between 0.0f and 100.0f
	 *
	 * */
	void update_duty_cycle (float* signals);

private:
	// pointer to PWM timer
	TIM_HandleTypeDef *tim;
	// holds the desired PWM pulse length for each channel
	uint32_t pulse_length[4];
};

#endif
#endif
