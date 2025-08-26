#include "Sensors/ICM42670P.hpp"

bool ICM42670P::init(){
	// holds outcome of the i2c interaction
	HAL_StatusTypeDef success;

	sensor_ready = false;

	// wait for full reset
	// 0x00010000 = software reset
	uint8_t config_state = 0x10;
	success = write_register(ICM42670P_REG_SIGNAL_PATH_RESET, &config_state);
	if (success != HAL_OK) {

		std::cout << "\nerror initiating ICM42670P software reset\r\n";
		return false;
	}
	HAL_Delay(100);

	// configure interrupt pin as push pull
	// 0x00000010
	uint8_t int_config_state = 0x02;
	success = write_register(ICM42670P_REG_INT_CONFIG, &int_config_state);
	if (success != HAL_OK) {

		std::cout << "\nerror configuring ICM42670P interupt pin\r\n";
		return false;
	}
	HAL_Delay(1);

	// put gyroscope and accelerometer into low noise mode
	//0x00001111
	uint8_t pwr_mgmt_state = 0x8f;
	success = write_register(ICM42670P_REG_PWR_MGMT0, &pwr_mgmt_state);
	if (success != HAL_OK) {

		std::cout << "\nerror initiating ICM42670P into low noise mode\r\n";
		return false;
	}
	HAL_Delay(50);

	// set acceleration and gyroscope configuration
	// set DPS to 2000deg/s, ODR to 400Hz
	// 0x00000111
	uint8_t gyro_config_state = 0x07;
	// set range to 16g, ODR to 400Hz
	// 0x00000111
	uint8_t accel_config_state = 0x07;

	success = write_register(ICM42670P_REG_GYRO_CONFIG0, &gyro_config_state);
	if (success != HAL_OK) {

		std::cout << "\nerror initiating ICM42670P gyroscope configuration\r\n";
		return false;
	}

	success = write_register(ICM42670P_REG_ACCEL_CONFIG0, &accel_config_state);
	if (success != HAL_OK) {

		std::cout << "\nerror initiating ICM42670P accelerometer configuration\r\n";
		return false;
	}

	// enable 20 byte extension
	// ensure BLK_SEL_W is set to 0x00
	uint8_t reg_sel = 0x00;
	success = write_register(ICM42670P_REG_BLK_SEL_W, &reg_sel);
	if (success != HAL_OK) {

		std::cout << "\nerror enabling ICM42670P byte extension\r\n";
		return false;
	}

	// enable FIFO
	// enable stop on full mode
	// 0x00000010
	uint8_t fifo_1_state = 0x02;
	success = write_register(ICM42670P_REG_FIFO_CONFIG1, &fifo_1_state);
	if (success != HAL_OK) {

		std::cout << "\nerror enabling ICM42670P FIFO\r\n";
		return false;
	}

	// set FIFO watermark
	// packet length 20, FIFO length 10, watermark size of 200
	// 0xc8 = 200
	uint8_t fifo_2_state = 0xc8;
	success = write_register(ICM42670P_REG_FIFO_CONFIG2, &fifo_2_state);
	if (success != HAL_OK) {

		std::cout << "\nerror setting ICM42670P FIFO watermark\r\n";
		return false;
	}

	// route FIFO threshold interrupt to INT1 pin
	// 0x00000100
	uint8_t int_0_state = 0x04;
	success = write_register(ICM42670P_REG_INT_SOURCE0, &int_0_state);
	if (success != HAL_OK) {

		std::cout << "\nerror routing ICM42670P interput pin\r\n";
		return false;
	}

	// set FIFO mode
	// select RCC clock
	// 0x00000001
	uint8_t intf1_state = 0x01;
	success = write_register(ICM42670P_REG_INTF_CONFIG1, &intf1_state);
	if (success != HAL_OK) {

		std::cout << "\nerror setting ICM42670P fifo mode/rcc clock\r\n";
		return false;
	}

	// enable 20 bit extension
	// we must access MREG1
	// 0x00 = bank 1
	uint8_t reg_bank_sel = 0x00;
	uint8_t desired_reg = ICM42670P_REG_FIFO_CONFIG5;

	success = write_register(ICM42670P_REG_BLK_SEL_W, &reg_bank_sel);
	if (success != HAL_OK) {

		std::cout << "\nerror accessing ICM42670P MREG1\r\n";
		return false;
	}

	success = write_register(ICM42670P_REG_MADDR_W, &desired_reg);
	if (success != HAL_OK) {

		std::cout << "\nerror writing ICM42670P MREG1\r\n";
		return false;
	}

	// finally write desired value to input register
	// 0x00001011
	uint8_t reading_bit_ext = 0x0b;
	success = write_register(ICM42670P_REG_M_W, &reading_bit_ext);
	if (success != HAL_OK) {

		std::cout << "\nerror writing desired value to ICM42670P input register\r\n";
		return false;
	}
	HAL_Delay(1);

	// flush FIFO to start with clean state
	// 0x00000100 = FIFO flush
	config_state = 0x04;
	success = write_register(ICM42670P_REG_SIGNAL_PATH_RESET, &config_state);
	if (success != HAL_OK) {

		std::cout << "\nerror initiating ICM42670P FIFO flush\r\n";
		return false;
	}
	HAL_Delay(1);

	//starting with an empty FIFO, no readings
	sensor_ready = false;

	find_offset ();

	return true;
}

bool ICM42670P::ping () {
	HAL_StatusTypeDef success = HAL_I2C_IsDeviceReady(handle, ICM42670P_I2C_ADDR, 3, 100);

	return success == HAL_OK;
}

motion_reading* ICM42670P::take_motion_reading (float time_curr, float& time_unacc){

	sensor_ready = false;

	// burst read FIFO data
	read_registers(ICM42670P_REG_FIFO_DATA, packets, ICM42670P_BURST_LENGTH * ICM42670P_PACKET_LENGTH);

	// flush FIFO to begin another reading
	// 0x00000100 = FIFO flush
	uint8_t config_state = 0x4;
	write_register(ICM42670P_REG_SIGNAL_PATH_RESET, &config_state);
	HAL_Delay(1);

	compensate();

	float time_sum = 0;
	for (int i = ICM42670P_BURST_LENGTH - 1; i >= 0; i--){
		time_sum += reading[i].elapsed_time;
	}

	// calculate unaccounted for time, this is the total period minus the sum of reading times
	time_unacc = time_curr - cpu_time_start - time_sum;

	// update both times to beginning of next period,
	// both at the same point in time, different frames
	cpu_time_start = time_curr;
	sensor_time_start = reading[ICM42670P_BURST_LENGTH - 1].elapsed_time + time_unacc;

	return reading;
}

void ICM42670P::find_offset(){

	uint8_t trials = 0;

	float time_curr = 0;
	float time_unac = 0;

	uint32_t acc_x = 0;
	uint32_t acc_y = 0;
	uint32_t acc_z = 0;

	uint32_t gyro_x = 0;
	uint32_t gyro_y = 0;
	uint32_t gyro_z = 0;

	for (uint8_t i = 0; i < trials; i++) {

		// wait until sensor is ready
		while (!sensor_ready) ;

		motion_reading* cal_reading = take_motion_reading(time_curr, time_unac);

		for (int j = 0; j < ICM42670P_BURST_LENGTH; j++) {
			acc_x += cal_reading[i].acc_x;
			acc_y += cal_reading[i].acc_y;
			acc_z += cal_reading[i].acc_z;

			gyro_x += cal_reading[i].ang_vel_x;
			gyro_y += cal_reading[i].ang_vel_y;
			gyro_z += cal_reading[i].ang_vel_z;
		}
	}

	uint8_t d = trials * ICM42670P_BURST_LENGTH;

	constant_offset[0].acc_x = acc_x / d;
	constant_offset[0].acc_y = acc_y / d;
	constant_offset[0].acc_z = acc_z / d;

	constant_offset[0].ang_vel_x = gyro_x / d;
	constant_offset[0].ang_vel_y = gyro_y / d;
	constant_offset[0].ang_vel_z = gyro_z / d;
}

void ICM42670P::compensate(){

	// load raw data into floats stored in array of structs, each struct representing a reading
	for (int i = 0; i < ICM42670P_BURST_LENGTH; i++) {

		/*
		 * bit shift to combine msb (8 bits) + lsb (8bits) + xlsb (4bits, stored in 8 bit integer) in uint32_t
		 *
		 * magic number is the offset showing which position the data occupies in the array, modulus 8
		 * magic number is used since it reduces header file length significantly
		 *
		 * bit mask used for xlsb since each register is shared between multiple values
		 *
		 * divided by lsb to convert from vague units to real units
		 *
		 *
		 * */

		uint16_t offset = i * ICM42670P_PACKET_LENGTH;

		uint8_t acc_x_xlsb = (packets[offset + 17] >> 4) & 0x0F;
		uint8_t gyro_x_xlsb = packets[offset + 17] & 0x0F;

		uint8_t acc_y_xlsb = (packets[offset + 18] >> 4) & 0x0F;
		uint8_t gyro_y_xlsb = packets[offset + 18] & 0x0F;

		uint8_t acc_z_xlsb = (packets[offset + 19] >> 4) & 0x0F;
		uint8_t gyro_z_xlsb = packets[offset + 19] & 0x0F;

		int32_t acc_x_raw = decode_registers(packets[offset + 1], packets[offset + 2], acc_x_xlsb);
		int32_t acc_y_raw = decode_registers(packets[offset + 3], packets[offset + 4], acc_y_xlsb);
		int32_t acc_z_raw = decode_registers(packets[offset + 5], packets[offset + 6], acc_z_xlsb);

		int32_t gyro_x_raw = decode_registers(packets[offset + 7], packets[offset + 8], gyro_x_xlsb);
		int32_t gyro_y_raw = decode_registers(packets[offset + 9], packets[offset + 10], gyro_y_xlsb);
		int32_t gyro_z_raw = decode_registers(packets[offset + 11], packets[offset + 12], gyro_z_xlsb);

		reading[i].acc_x = (((int32_t)acc_x_raw * GRAV / ICM42670_ACC_LSB) - constant_offset[0].acc_x);
		reading[i].acc_y = (((int32_t)acc_y_raw * GRAV / ICM42670_ACC_LSB) - constant_offset[0].acc_y);
		reading[i].acc_z = (((int32_t)acc_z_raw * GRAV / ICM42670_ACC_LSB) - constant_offset[0].acc_z);

		reading[i].ang_vel_x = (((int32_t)gyro_x_raw * DEG_TO_RAD / ICM42670_GYRO_LSB) - constant_offset[0].ang_vel_x);
		reading[i].ang_vel_y = (((int32_t)gyro_y_raw * DEG_TO_RAD / ICM42670_GYRO_LSB) - constant_offset[0].ang_vel_y);
		reading[i].ang_vel_z = (((int32_t)gyro_z_raw * DEG_TO_RAD / ICM42670_GYRO_LSB) - constant_offset[0].ang_vel_z);

		// time of reading, later converted to time since last reading (seconds)
		reading[i].elapsed_time =  (uint32_t)((packets[i * ICM42670P_PACKET_LENGTH + 15] << 8)
				| (packets[i * ICM42670P_PACKET_LENGTH + 16]))
				/ ICM42670_COUNTER_SPEED;
	}

	// calculate elapsed time, currently the variable holds raw time steps
	for (int i = ICM42670P_BURST_LENGTH - 1; i > 0; i--){

		// counter overflowed
		if (reading[i].elapsed_time < reading[i - 1].elapsed_time) {

			// find the total time for the counter (in ms)
			float counter_time = (ICM42670P_COUNTER_SIZE * 1000.0f) / ICM42670_COUNTER_SPEED;

			reading[i].elapsed_time += counter_time - reading[i-1].elapsed_time;

		// normal case
		} else {
			reading[i].elapsed_time -= reading[i-1].elapsed_time;
		}
	}

	// update the first reading's measurement time
	// instead of being the time since last reading,
	// it is the time since beginning of frame

	// if counter overflowed
	if (reading[0].elapsed_time < sensor_time_start) {

		// find the total time for the counter (in ms)
		float counter_time = (ICM42670P_COUNTER_SIZE * 1000.0f) / ICM42670_COUNTER_SPEED;

		reading[0].elapsed_time += counter_time - sensor_time_start;

	// normal case
	} else {
		reading[0].elapsed_time -= sensor_time_start;
	}
}

int32_t ICM42670P::decode_registers(uint8_t msb, uint8_t lsb, uint8_t xlsb_nibble) {
    uint32_t raw20 = ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | (uint32_t)(xlsb_nibble & 0x0F);

    // Sign-extend manually using shift (safe for 20-bit values)
    int32_t signed_value = (int32_t)(raw20 << 12) >> 12;

    return signed_value;
}

void ICM42670P::set_sensor_ready(){
	sensor_ready = true;
}

bool ICM42670P::is_sensor_ready(){
	return sensor_ready;
}

HAL_StatusTypeDef ICM42670P::read_registers(uint8_t reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Read(handle, ICM42670P_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ICM42670P::write_register(uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(handle, ICM42670P_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
