/*
 * Tests barometer, IMU, Kalman Filters and Motor PWM Output
 * one blink for failure, two for success, outputs results over USB Virtual COM Port
 *
 * controller and automation classes are tested in flight, cannot be automated
 *
 * Created by Paul Gabel
 * Date: 8/5/2025
 *
 * */

#ifndef TEST_HPP
#define TEST_HPP

#include "stm32f4xx_hal.h"

#include "Application/Motors.hpp"
#include "Control/Matrix.hpp"
#include "Control/attitude_kf.hpp"
#include "Control/altitude_kf.hpp"
#include "Sensors/BMP280.hpp"
#include "Sensors/ICM42670P.hpp"

#ifdef __cplusplus

#define STATUS_LED_PIN 						GPIO_PIN_12
#define STATUS_GPIO_PORT 					GPIOB

#define IMU_PACKET_LENGTH					10

#define M_S									1/1000.0f
#define SEC									1000

// use heap allocation, never deleted since program loop continues forever
extern BMP280* barometer;
extern ICM42670P* imu;
extern Motors* m;
extern AttitudeKalman Attkf;
extern AltitudeKalman Altkf;

extern bool imu_initialized;

#ifdef __cplusplus

#include <iostream>

/*
 * initializes classes
 *
 * */
void init_testing (I2C_HandleTypeDef* n_i2c_handle, TIM_HandleTypeDef *htim2);

bool test_imu_initialized();

void set_imu_ready_test ();

/*
 * blinks four times, then runs through tests - order:
 * matrix unit tests
 * sensor connections (ping)
 * motors
 * Kalman filters (reads from sensors then outputs)
 *
 * if error is encountered during matrix unit tests, the program fails
 * if error is encountered during other tests, error is logged over usb
 * */
void run_tests ();

/*
 * blink the green status led once
 *
 * on for 1 sec, off for 2
 *
 * */
void blink_once ();

/*
 * blink the green status led twice
 *
 * on/off 0.5 sec each, then 1 sec. off padding at the end
 *
 * */
void blink_twice ();

/*
 * initializes motors and turns on high for 3 seconds, then off
 * status led on for the duration of the test
 *
 * physical test, mcu can't determine outcome
 *
 * */
void test_motors ();

/*
 * Ping IMU to test if it's connected
 *
 * */
void test_IMU_Conn ();

/*
 * Ping Barometer to test if it's connected
 *
 * */
void test_Barometer_Conn ();

/*
 * try to initialize IMU
 *
 * */
bool test_IMU_Init ();

/*
 * try to initialize barometer
 *
 * */
bool test_Barometer_Init ();

/*
 * take readings from the sensor and output them
 *
 * */
void test_sensors();

/*
 * continually read from sensors and run the kalman filter, output reading with cout
 *
 * */
void test_Kalmans();

/*
 * test matrix multiplication with different sized int matrices
 *
 * */
void test_mat_mult();

/*
 * test matrix addition int matrices
 *
 * */
void test_mat_add();

/*
 * test matrix subtraction int matrices
 *
 * */
void test_mat_sub();

/*
 * test matrix transpose with mxn int matrix
 *
 * */
void test_mat_transpose();

/*
 * test matrix inverse using float
 *
 * inverse doesn't work when using int due to int truncation, this does not affect current use of class
 *
 * */
void test_mat_inverse();

#endif
#endif
#endif
