#include "Application/receiver.hpp"

void Receiver::init(){

	// initialize DMA input capture
    HAL_TIM_IC_Start_DMA(tim_h, TIM_CHANNEL_1, (uint32_t*)capture_buffer[0], BUFFER_LENGTH);
    HAL_TIM_IC_Start_DMA(tim_h, TIM_CHANNEL_2, (uint32_t*)capture_buffer[1], BUFFER_LENGTH);
    HAL_TIM_IC_Start_DMA(tim_h, TIM_CHANNEL_3, (uint32_t*)capture_buffer[2], BUFFER_LENGTH);
    HAL_TIM_IC_Start_DMA(tim_h, TIM_CHANNEL_4, (uint32_t*)capture_buffer[3], BUFFER_LENGTH);
}

float* Receiver::get_duty_cycle(){

	float signals[4];

	// find the number of remaining writes until buffer roll over, for finding current index
	uint32_t remaining[CHANNEL_COUNT];

	remaining[0] = __HAL_DMA_GET_COUNTER(tim_h->hdma[TIM_DMA_ID_CC1]);
	remaining[1] = __HAL_DMA_GET_COUNTER(tim_h->hdma[TIM_DMA_ID_CC2]);
	remaining[2] = __HAL_DMA_GET_COUNTER(tim_h->hdma[TIM_DMA_ID_CC3]);
	remaining[3] = __HAL_DMA_GET_COUNTER(tim_h->hdma[TIM_DMA_ID_CC4]);

	for (int i = 0; i < CHANNEL_COUNT; i++) {

		uint16_t current_index = BUFFER_LENGTH - remaining[i];
		uint16_t last_index = (current_index == 0) ? (BUFFER_LENGTH - 1) : (current_index - 1);

		// timer counter is 16bit so we don't have to worry about overflow when subtracting
		uint16_t pulse_length_ticks = capture_buffer[i][current_index] - capture_buffer[i][last_index];

		// convert to microseconds
		// time = ticks * (prescalar + 1) * 1,000,000 / timer_clock
		// prescalar = 0, timer_clock is in MHZ
		// time = ticks / timer_clock
		signals[i] = pulse_length_ticks / TIMER_FREQUENCY;

		// ensure reading is high time
		// typical duty cycle is roughly between 5% and 10%
		// if too large we know invert (low time -> high time)
		if (signals[i] > (INPUT_PERIOD_MICRO_SEC / 2)) {
			signals[i] = INPUT_PERIOD_MICRO_SEC - final_signals[i];
		}




		// normalize to be a percentage expressed between -1 and 1
		signals[i] -= INPUT_SIGNAL_START;
		signals[i] /= INPUT_SIGNAL_RANGE;
	}

	final_signals[0] = signals[HEIGHT_CHANNEL];
	final_signals[1] = signals[ROLL_CHANNEL];
	final_signals[2] = signals[PITCH_CHANNEL];
	final_signals[3] = signals[YAW_CHANNEL];

	// clamp and dead-zone
	normalize();

	return final_signals;
}

void Receiver::normalize (){
	for (int i = 0; i < CHANNEL_COUNT; i++) {

		// clamp
		float bound = 1.0f;

		if (final_signals[i] > bound)
			final_signals[i] = bound;

		if (final_signals[i] < -bound)
					final_signals[i] = -bound;

		// apply dead-zoning to eliminate stick drift
		bound = 0.3f;

		if (fabsf(final_signals[i]) < bound)
			final_signals[i] = 0;

	}
}
