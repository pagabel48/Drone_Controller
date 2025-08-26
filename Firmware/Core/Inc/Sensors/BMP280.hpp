/*
 * BMP280 i2c Driver
 *
 * Created by Paul Gabel
 * Date: 7/12/2025
 *
 * */

#ifndef BMP280_DRIVER_HPP
#define BMP280_DRIVER_HPP

#ifdef __cplusplus
#include <stdint.h>
#include <cmath>
#include <iostream>
#include "stm32f4xx_hal.h"
#endif

/*
 * Defines
 * */
#define BMP280_I2C_ADDR 		(0x76 << 1) // address is 7 bits
#define BMP280_I2C_ID 			0x58
#define BMP280_CAL_SIZE			25

#define PASCAL_METER_CONST		0.0843f

// [7-5] = [000] inactive duration of 0.5ms
// [4-2] = [100] IIR filter coefficient of 16
// [0] = [0]
// 0x00010000 = 0x10
#define BMP280_CONFIG_STATE		0x10

// [7-5] = [010] 2x temperature over sampling (recommended for pressure reading accuracy)
// [4-2] = [101] 16x pressure over sampling (high accuracy)
// [1-0] = [01] forced mode
// 0x01010101 = 0x
#define BMP280_MEAS_STATE		0x55

/*
 * Registers
 * */
#define BMP280_REG_T_XLSB 		0xfc
#define BMP280_REG_T_LSB		0xfb
#define BMP280_REG_T_MSB 		0xfa
#define BMP280_REG_P_XLSB 		0xf9
#define BMP280_REG_P_LSB 		0xf8
#define BMP280_REG_P_MSB 		0xf7
#define BMP280_REG_CONFIG 		0xf5
#define BMP280_REG_CTRL_MEAS 	0xf4
#define BMP280_REG_STATUS 		0xf3
#define BMP280_REG_RESET 		0xe0
#define BMP280_REG_ID 			0xd0
#define BMP280_REG_CAL_START	0x88

// calibration
#define CALIB_MEAS_SIZE			10

#ifdef __cplusplus
/*
 * Data Structure
 *
 * uses polling to read pressure data from the sensor
 *
 * */
class BMP280 {
public:

	BMP280(I2C_HandleTypeDef* i2cHandle):
		handle (i2cHandle),
		start_point (0)
		{}

	/*
	 * High-Level Functions
	 *
	 * Initialize Sensor and variables
	 * not a constructor so a return type can be used
	 *
	 * return whether this was a success
	 * */
	bool init();

	/*
	 * ping the sensor over I2C
	 *
	 * return whether device responded
	 * */
	bool ping();

	/*
	 * check if the sensor is done taking measurement
	 *
	 * return true if sensor is done, false if busy or read failed
	 * */
	bool sensor_ready();

	/*
	 * perform a pressure reading
	 *
	 * param time_passed: time passed (in ms) since last reading, passed by reference
	 *
	 * returns pressure in pascals as float
	 * */
	float read_pressure(uint32_t* time_passed);

	/*
	 * perform a pressure reading
	 *
	 * param time_passed: time passed (in ms) since last reading, passed by reference
	 *
	 * returns pressure in meters above sea level as float
	 * */
	float read_pressure_meters(uint32_t* time_passed);

private:
	// holds information related to i2c operation (for HAL)
	I2C_HandleTypeDef* handle;

	// used to find time passed since last operation
	uint32_t last_reading;

	// must be defined a second time (other than define) to reference by pointer
	uint8_t config_reg_state;
	uint8_t meas_reg_state;

	// configuration variables (filled from configuration register)
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	// arrays to hold readings from i2c
	uint8_t pressure_reading[3];
	uint8_t temp_reading[3];
	uint8_t calibration_data[25];

	float start_point;

	/*
	 * outputs the sensitive part of the temperature reading
	 *
	 * provided by Bosch
	 * used for compensating pressure reading
	 *
	 * returns data as integer
	 * */
	uint32_t temp_fine();

	/*
	 * convert raw pressure reading into polished reading; compensate for sensor abnormalities
	 *
	 * Provided by Bosch
	 * uses temperature reading, configuration data and pressure reading
	 *
	 * outputs the final pressure in Pascal as integer in Q24.8 format
	 * */
	uint32_t compensate_pres();

	/*
	 * Low level i2c wrapper functions
	 * */
	HAL_StatusTypeDef read_registers(uint8_t reg, uint8_t* data, uint8_t length);
	HAL_StatusTypeDef write_register(uint8_t reg, uint8_t* data);
};

#endif
#endif
