#include "Sensors/BMP280.hpp"

bool BMP280::init(){

	// holds outcome of i2c read
	HAL_StatusTypeDef success;

	// hold desired register states so can be referenced by pointer
	config_reg_state = BMP280_CONFIG_STATE;
	meas_reg_state = BMP280_MEAS_STATE;

	// load calibration data into array
	success = read_registers(BMP280_REG_CAL_START, calibration_data, BMP280_CAL_SIZE);
	if (success != HAL_OK) {

		std::cout << "\nerror reading calibration data\r\n";
		return false;
	}

	// write to configuration register, verify
	success = write_register(BMP280_REG_CONFIG, &config_reg_state);
	if (success != HAL_OK) {

		std::cout << "\nerror writing to configuration register\r\n";
		return false;
	}

	// write to ctrl_meas register, verify
	success = write_register(BMP280_REG_CTRL_MEAS, &meas_reg_state);
	if (success != HAL_OK) {

		std::cout << "\nerror writing to ctrl_meas register\r\n";
		return false;
	}

	// update so that we can get reading time
	last_reading = HAL_GetTick();

	// set calibration variables from calibration array
	dig_T1 = ((uint16_t) calibration_data[1]) | calibration_data[0];
	dig_T2 = ((int16_t) calibration_data[3])  | calibration_data[2];
	dig_T3 = ((int16_t) calibration_data[5])  | calibration_data[4];
	dig_P1 = ((uint16_t) calibration_data[7]) | calibration_data[6];
	dig_P2 = ((int16_t) calibration_data[9])  | calibration_data[8];
	dig_P3 = ((int16_t) calibration_data[11]) | calibration_data[10];
	dig_P4 = ((int16_t) calibration_data[13]) | calibration_data[12];
	dig_P5 = ((int16_t) calibration_data[15]) | calibration_data[14];
	dig_P6 = ((int16_t) calibration_data[17]) | calibration_data[16];
	dig_P7 = ((int16_t) calibration_data[19]) | calibration_data[18];
	dig_P8 = ((int16_t) calibration_data[21]) | calibration_data[20];
	dig_P9 = ((int16_t) calibration_data[23]) | calibration_data[22];

	// find a steady starting value to subtract from each new reading

	float start_point_i = 0;

	for (int i = 0; i < CALIB_MEAS_SIZE; i++) {
		while (sensor_ready() == false) {
			HAL_Delay(1);
		}

		uint32_t place_holder = 0;
		start_point_i += read_pressure(&place_holder);
	}

	start_point = start_point_i /= (float)CALIB_MEAS_SIZE;

	// return that initialization was a success
	return true;
}

bool BMP280::ping () {
	HAL_StatusTypeDef success = HAL_I2C_IsDeviceReady(handle, BMP280_I2C_ADDR, 3, 100);

	return success == HAL_OK;
}

bool BMP280::sensor_ready(){
	uint8_t* status = 0;
	// holds status of i2c operation
	HAL_StatusTypeDef success;

	// read status register into status, verify success
	success = read_registers(BMP280_REG_STATUS, status, 1);
	if (success != HAL_OK) {
		return false;
	}

	// status is 0 if not currently taking measurement
	return *status == 0;
}

float BMP280::read_pressure(uint32_t* time_passed){
	uint32_t pres_Q24_8;	   // holds the pressure reading
	HAL_StatusTypeDef success; // holds status of i2c operation

	// read temperature and pressure, verify read success
	success = read_registers(BMP280_REG_P_MSB, pressure_reading, 3);
	if (success != HAL_OK) {
		return 0;
	}
	success = read_registers(BMP280_REG_T_MSB, temp_reading, 3);
	if (success != HAL_OK) {
		return 0;
	}

	// get time passed
	*time_passed = HAL_GetTick() - last_reading;

	// compensate
	pres_Q24_8 = compensate_pres();

	// tell sensor to take another reading
	write_register(BMP280_REG_CTRL_MEAS, &meas_reg_state);
	last_reading = HAL_GetTick();

	// 256.0f accounts for the 8 decimal points
	return ((float)pres_Q24_8 / 256.0f) - start_point;
}

float BMP280::read_pressure_meters(uint32_t* time_passed) {
	float pressure = read_pressure(time_passed);

	return PASCAL_METER_CONST * pressure;
}

/*
 * compensation functions
 *
 * unsure what these do but were provided in Bosch BMP280 data sheet
 *
 * */
uint32_t BMP280::temp_fine(){
	int32_t var1, var2, adc_T;

	adc_T = ((int32_t)temp_reading[0] << 12) | ((int32_t)temp_reading[1] << 4) | ((int32_t)temp_reading[2] >> 4);

	var1 = (((adc_T) - ((int32_t)dig_T1 << 1)) * ((int32_t)dig_T2)) >> 1;
	var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

	// get full temperature:
	// T = ((var1 + var2) * 5 + 128) >> 8;

	return var1 + var2;
}

uint32_t BMP280::compensate_pres(){
	int32_t var1, var2, adc_P, t_fine;
	uint32_t pressure;

	// 20 bit raw data reading
	adc_P = ((int32_t)pressure_reading[0] << 12) | ((int32_t)pressure_reading[1] << 4) | ((int32_t)pressure_reading[2] >> 4);

	t_fine = temp_fine();

	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
	var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)dig_P4) < 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);

	if (var1 == 0) {
		return 0;
	}

	pressure = ((uint32_t)(((int32_t)1048576) - adc_P)-(var2 >> 12)) * 3125;

	if (pressure < 0x80000000) {
		pressure = (pressure << 1) / ((uint32_t)var1);
	}
	else {
		pressure = (pressure / (uint32_t)var1) * 2;
	}

	var1 = (((int32_t)dig_P9) * ((int32_t)(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(pressure >> 2)) * ((int32_t)dig_P8)) >> 13;

	pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + dig_P7) >> 4));

	return pressure;
}

HAL_StatusTypeDef BMP280::read_registers(uint8_t reg, uint8_t* data, uint8_t length){
	return HAL_I2C_Mem_Read(handle, BMP280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMP280::write_register(uint8_t reg, uint8_t* data){
	return HAL_I2C_Mem_Write(handle, BMP280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
