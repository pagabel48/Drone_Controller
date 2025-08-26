/*
 * captures input of drone receiver
 *
 * Created by Paul Gabel
 * Date: 7/16/2025
 *
 * */

#ifndef RECEIVER_DRIVER_HPP
#define RECEIVER_DRIVER_HPP

#include "stm32f4xx_hal.h"

/*
 * Defines
 * */
#define CHANNEL_COUNT							4
#define BUFFER_LENGTH							4
#define TIMER_FREQUENCY							84.0f 			// mhz
#define INPUT_PERIOD_MICRO_SEC					20000.0f		// microseconds
#define INPUT_SIGNAL_RANGE						500.0f
#define INPUT_SIGNAL_START						1500.0f

// receiver channel to axis mappings
#define HEIGHT_CHANNEL							0
#define ROLL_CHANNEL							1
#define PITCH_CHANNEL							2
#define YAW_CHANNEL								3

#ifdef __cplusplus

#include <cmath>

class Receiver {
public:

	/*
	 * htim = timer channel (typically 3)
	 *
	 * */
	Receiver (TIM_HandleTypeDef *htim) :
		tim_h (htim)
	{}

	/*
	 * Initializes the motors
	 *
	 * */
	void init();

	/*
	 * gets raw controller input, scaled between -1 and 1
	 *
	 * parameter signals: array containing each controller input, 1 through 4
	 * 		order: height, roll, pitch, yaw
	 *
	 * */
	float* get_duty_cycle();

private:
	// holds the DMA capture arrays
	uint16_t capture_buffer[CHANNEL_COUNT][BUFFER_LENGTH];
	// handle for input capture timer
	TIM_HandleTypeDef* tim_h;

	// holds the final captured signals
	float final_signals[BUFFER_LENGTH];

	/*
	 * clamp and apply dead zoning to final_signals
	 *
	 * */
	void normalize();
};

#endif
#endif
