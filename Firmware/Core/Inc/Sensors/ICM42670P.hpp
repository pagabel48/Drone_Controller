/*
 * ICM42670P I2C Driver
 *
 * Created by Paul Gabel
 * Date: 7/14/2025
 *
 * */

#ifndef ICM42670P_DRIVER_HPP
#define ICM42670P_DRIVER_HPP

#include "stm32f4xx_hal.h"

/*
 * Defines
 * */

#define ICM42670P_I2C_ADDR				(0x68 << 1) // 7 bit address

#define ICM42670P_BURST_LENGTH 			10
#define ICM42670P_PACKET_LENGTH 		20
#define ICM42670P_COUNTER_SIZE			65535

#define GRAV							9.8066f
#define DEG_TO_RAD						3.14159f / 180.0f

// also converts from g's to m/s^2
#define ICM42670_ACC_LSB				32768.0f

// also converts from degrees to radians
#define	ICM42670_GYRO_LSB				16.384f

#define	ICM42670_COUNTER_SPEED			32768.0f

/*
 * Registers
 * */
// MREG0
#define ICM42670P_REG_DEVICE_CONFIG		0x01
#define ICM42670P_REG_SIGNAL_PATH_RESET	0x02
#define ICM42670P_REG_INT_CONFIG		0x06
#define ICM42670P_REG_PWR_MGMT0			0x1f
#define ICM42670P_REG_GYRO_CONFIG0		0x20
#define ICM42670P_REG_ACCEL_CONFIG0		0x21
#define ICM42670P_REG_GYRO_CONFIG1		0X23
#define ICM42670P_REG_ACCEL_CONFIG1		0x24
#define ICM42670P_REG_FIFO_CONFIG1		0x28
#define ICM42670P_REG_FIFO_CONFIG2		0x29
#define ICM42670P_REG_FIFO_CONFIG3		0x2a
#define	ICM42670P_REG_INT_SOURCE0		0x2b
#define	ICM42670P_REG_INT_SOURCE1		0x2c
#define	ICM42670P_REG_INT_SOURCE3		0x2d
#define	ICM42670P_REG_INT_SOURCE4		0x2e
#define	ICM42670P_REG_INTF_CONFIG0		0x35
#define	ICM42670P_REG_INTF_CONFIG1		0x36
#define ICM42670P_REG_FIFO_DATA			0x3f
#define ICM42670P_REG_BLK_SEL_W			0x79
#define ICM42670P_REG_MADDR_W			0x7a
#define ICM42670P_REG_M_W				0x7b
#define ICM42670P_REG_BLK_SEL_R			0x7c

// MREG1
#define ICM42670P_REG_FIFO_CONFIG5		0x01

#ifdef __cplusplus

#include <stdint.h>
#include <iostream>
#include <iomanip>
/*
 * Data Structures
 * */
struct motion_reading {

	// acceleration in the axis, units of m/s^2
	float acc_x, acc_y, acc_z;

	// angular velocity around the axis, units of radians / second
	float ang_vel_x, ang_vel_y, ang_vel_z;

	// in seconds
	float elapsed_time;
};

/*
 * Uses FIFO on sensor to hold many samples which are burst read all at once
 *
 *
 * */
class ICM42670P {
public:

	ICM42670P (I2C_HandleTypeDef* n_i2c_handle) :
		handle (n_i2c_handle)
	{}

	/*
	 * initialize the sensor, init variables
	 *
	 * return whether initialization was a success, if not sensor read should not be performed
	 * */
	bool init();

	/*
	 * ping the sensor over I2C
	 *
	 * return whether device responded
	 * */
	bool ping();

	/*
	 * take a sensor reading
	 *
	 * PARAM time_curr: current system time, seconds
	 * PARAM time_unacc: time unaccounted for at end of burst read, seconds
	 *
	 * returns reading struct array, including acceleration, angular velocity and elapsed time for each reading
	 * */
	motion_reading* take_motion_reading (float time_curr, float& time_unacc);

	/*
	 * set sensor ready variable to true
	 * */
	void set_sensor_ready();

	/*
	 * check if sensor is ready to read from
	 *
	 * returns whether condition is true
	 * */
	bool is_sensor_ready();

private:

	// beginning of burst read period, MCU frame
	float cpu_time_start;

	// beginning of burst read period, sensor frame
	float sensor_time_start;

	// holds whether the sensor is ready to read from
	bool sensor_ready;

	// holds i2c configuration data
	I2C_HandleTypeDef* handle;

	// holds raw data packet
	uint8_t packets[ICM42670P_BURST_LENGTH * ICM42670P_PACKET_LENGTH];

	// holds finished reading
	motion_reading reading[ICM42670P_BURST_LENGTH];

	//holds offset
	motion_reading constant_offset[1];


	/*
	 * finds 100 readings, finds average and uses this as a constant offset
	 *
	 * */
	void find_offset();

	/*
	 * convert raw array of data packets into polished readings from the sensor
	 *
	 * fills output array by reference
	 * */
	void compensate();

	/*
	 * take raw resister values and convert to 32int
	 *
	 * */
	int32_t decode_registers(uint8_t msb, uint8_t lsb, uint8_t xlsb_nibble);

	/*
	 * Low level i2c wrapper functions
	 * */
	HAL_StatusTypeDef read_registers(uint8_t reg, uint8_t* data, uint8_t length);
	HAL_StatusTypeDef write_register(uint8_t reg, uint8_t* data);
};

#endif
#endif
