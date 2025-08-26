#include "Application/motors.hpp"

void Motors::init() {

	// start PWM
	HAL_TIM_PWM_Start(tim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(tim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(tim, TIM_CHANNEL_4);

	//start with 50% duty cycle (off for BLDC motor), 5 second delay to initialize motors
	uint32_t period = (tim->Init.Period + 1) / 2;

	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, period);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, period);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_3, period);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_4, period);

	HAL_Delay(5000);
}

void Motors::update_duty_cycle(float* signals){

	// normalize duty cycle, convert to timer pulse length between 50% and 100%
	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (signals[i] > 100.0f) signals[i] = 100.0f;
		if (signals[i] < 0.0f) signals[i] = 0.0f;

		pulse_length[i] = (tim->Init.Period + 1) * (100 + signals[i]) / 200;
	}

	// update the PWM pulse length
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, pulse_length[0]);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, pulse_length[1]);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_3, pulse_length[2]);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_4, pulse_length[3]);
}
