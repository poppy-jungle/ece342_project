#include "lsm303agr.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;
uint16_t addr;
uint8_t tempL;
uint8_t tempH;
int temp;
struct ac acceleration;
struct mg magneticField;
struct ac accelerationBuffer[FIFO_SIZE] = { 0 };
uint8_t burstBuf[6 * FIFO_SIZE] = { 0 };

// states
int int_acc;
int int_crash = 0;

// hopefully the read write addr should be same
uint8_t lsm303agr_read(uint8_t reg, char dev) {
	addr = ADDR_LSM303AGR_ACCEL;
	if (dev == 'm') addr = ADDR_LSM303AGR_MAGNET;
	while (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, &reg, 1, 1000) != HAL_OK) {}
	uint8_t value;
	while (HAL_I2C_Master_Receive(&hi2c1, addr << 1, &value, 1, 100) != HAL_OK) {}
	return value;
}

HAL_StatusTypeDef lsm303agr_write(uint8_t reg, uint8_t val, char dev) {
	addr = ADDR_LSM303AGR_ACCEL;
	if (dev == 'm') addr = ADDR_LSM303AGR_MAGNET;
	uint8_t packet[2];
	packet[0] = reg;
	packet[1] = val;
	return HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, packet, 2, 100);
}

void lsm303agr_poll_xyz(struct ac* data) {
	uint8_t xh, xl;
	uint8_t yh, yl;
	uint8_t zh, zl;
	xh = lsm303agr_read(OUT_X_H_A, 'a');
	xl = lsm303agr_read(OUT_X_L_A, 'a');
	yh = lsm303agr_read(OUT_Y_H_A, 'a');
	yl = lsm303agr_read(OUT_Y_L_A, 'a');
	zh = lsm303agr_read(OUT_Z_H_A, 'a');
	zl = lsm303agr_read(OUT_Z_L_A, 'a');
	data->x = twosCompToInt_10bit((uint16_t)((xh << 2) | (xl >> 6)));
	data->y = twosCompToInt_10bit((uint16_t)((yh << 2) | (yl >> 6)));
	data->z = twosCompToInt_10bit((uint16_t)((zh << 2) | (zl >> 6)));
	data->x = (data->x) / 512.0 * 2.0 ;
	data->y = (data->y) / 512.0 * 2.0 ;
	data->z = (data->z) / 512.0 * 2.0 - 1;			// offset gravity
}

void lsm303agr_poll_xyz_mag() {
	struct mg* data = &magneticField;
	uint8_t xh, xl;
	uint8_t yh, yl;
	uint8_t zh, zl;
	xh = lsm303agr_read(OUTX_H_REG_M, 'm');
	xl = lsm303agr_read(OUTX_L_REG_M, 'm');
	yh = lsm303agr_read(OUTY_H_REG_M, 'm');
	yl = lsm303agr_read(OUTY_L_REG_M, 'm');
	zh = lsm303agr_read(OUTZ_H_REG_M, 'm');
	zl = lsm303agr_read(OUTZ_L_REG_M, 'm');
	data->x = twosCompToInt_16bit((uint16_t)((xh << 8) | (xl)));
	data->y = twosCompToInt_16bit((uint16_t)((yh << 8) | (yl)));
	data->z = twosCompToInt_16bit((uint16_t)((zh << 8) | (zl)));		
	double headingRadians = atan2(-data->y, data->x);
	double headingDegrees = headingRadians * 180 / M_PI;
	headingDegrees += MAG_DECLINATION_TORONTO - 45;
	if (headingDegrees < 0)
		headingDegrees += 360;
	if (headingDegrees > 348.75 || headingDegrees < 11.25) {
    data->direction = " N";
  }
  else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
    data->direction = " NNE";
  }
  else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
    data->direction = " NE";
  }
  else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
    data->direction = " ENE";
  }
  else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
    data->direction = " E";
  }
  else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
    data->direction = " 	ESE";
  }
  else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
    data->direction = " SE";
  }
  else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
    data->direction = " SSE";
  }
  else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
    data->direction = " S";
  }
  else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
    data->direction = " SSW";
  }
  else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
    data->direction = " SW";
  }
  else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
    data->direction = " WSW";
  }
  else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
    data->direction = " W";
  }
  else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
    data->direction = " WNW";
  }
  else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
    data->direction = " NW";
  }
  else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
    data->direction = " NNW";
  }
}


void lsm303agr_init(void) {
	
	// sanity check
	char buffer[100];
	sprintf(buffer, "\n\nSanity check\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	uint8_t whoAmIA = lsm303agr_read((uint8_t)LSM303AGR_WHO_AM_I_A, 'a');
	sprintf(buffer, "Who am I (accel): %d (expecting: 51)\r\n", (int)whoAmIA);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	uint8_t whoAmIM = lsm303agr_read((uint8_t)LSM303AGR_WHO_AM_I_M, 'm');
	sprintf(buffer, "Who am I (magneto): %d (expecting: 64)\r\n", (int)whoAmIM);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	
	// check accelerometer
	if (HAL_I2C_IsDeviceReady(&hi2c1, ADDR_LSM303AGR_ACCEL_R, 10, 1000) == HAL_OK) {
		sprintf(buffer, "LSM303AGR Accelerometer is ready!\r\n");
	} else {
		sprintf(buffer, "LSM303AGR Accelerometer is not ready!\r\n");
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	
	// check magnetometer
	if (HAL_I2C_IsDeviceReady(&hi2c1, ADDR_LSM303AGR_MAGNET_R, 10, 1000) == HAL_OK) {
		sprintf(buffer, "LSM303AGR Magnetometer is ready!\r\n");
	} else {
		sprintf(buffer, "LSM303AGR Magnetometer is not ready!\r\n");
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	
	// configure the accelerometer
	sprintf(buffer, "Initializing accelerometer...\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	lsm303agr_write(CTRL_REG1_A, 0b01010111, 'a');				// data rate select: 100Hz
	// lsm303agr_write(CTRL_REG6_A, 0b00100000, 'a');
	lsm303agr_bypass_en();																// default mode: bypass
	/* skip high-pass filter config */
	/* skip interrupt pin config */
	lsm303agr_write(INT2_CFG_A, 0b00000010, 'a');					// Interrupt 2 on x high event (if more -ve and more +ve)
	lsm303agr_write(INT2_THS_A, 0b01111111, 'a');					// high/low threshold -> 1g
	lsm303agr_write(INT2_DURATION_A, 0b00000000, 'a');		// interrupt duration, just make more than 0 -> DO NOT CHANGE
	lsm303agr_write(CTRL_REG6_A, 0b00100000, 'a');
	
	// configure the magnetometer
	sprintf(buffer, "Initializing magnetometer...\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	lsm303agr_write(CFG_REG_A_M, 0b00000000, 'm');				// selected data rate: 10Hz
	lsm303agr_write(CFG_REG_C_M, 0b00010001, 'm');				// enable interrupt, BDU enable?
	
	// turn on temperature sensor
	sprintf(buffer, "Turning on temperature sensor...\r\n\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	lsm303agr_write(TEMP_CFG_REG_A, 0b11000000, 'a');			// temp enable
	lsm303agr_write(CTRL_REG4_A, 0b10000000, 'a');				// enable block data update
	
	// measure the current temperature
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start(&htim6);
	tempL = lsm303agr_read(OUT_TEMP_L_A, 'a');
	tempH = lsm303agr_read(OUT_TEMP_H_A, 'a');
	temp = twosCompToInt(tempH) + 25;
	HAL_TIM_Base_Stop(&htim6);
	sprintf(buffer, "Current temperature: %dC\r\n", temp);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "This poll took: %dms\r\n\n", __HAL_TIM_GET_COUNTER(&htim6) / 10);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	
	
	// measure the x, y, z acceleration
	sprintf(buffer, "Polling x, y, z acceleration...\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start(&htim6);
	lsm303agr_poll_xyz(&acceleration);
	HAL_TIM_Base_Stop(&htim6);
	sprintf(buffer, "Current x acceleration: %fG\r\n", acceleration.x);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "Current y acceleration: %fG\r\n", acceleration.y);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "Current z acceleration: %fG\r\n", acceleration.z);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "This poll took: %ds\r\n\n",  __HAL_TIM_GET_COUNTER(&htim6) / 10);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	
	// measure the x, y, z magnetometer
	sprintf(buffer, "Polling x, y, z magnet...\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start(&htim6);
	lsm303agr_poll_xyz_mag();
	HAL_TIM_Base_Stop(&htim6);
	sprintf(buffer, "Current x magnetic field: %funits\r\n", magneticField.x);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "Current y magnetic field: %funits\r\n", magneticField.y);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "Current z magnetic field: %funits\r\n", magneticField.z);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "Direction: %s\r\n", magneticField.direction);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	sprintf(buffer, "This poll took: %ds\r\n",  __HAL_TIM_GET_COUNTER(&htim6) / 10);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	
	
	// now change the mode of accelerometer into FIFO mode and pure FIFO mode
	HAL_Delay(1000);
	sprintf(buffer, "Changing accelermometer into FIFO-streaming mode...\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	/*
	lsm303agr_bypass_en();
	lsm303agr_fifo_en();																			// FIFO enable
	*/
	
	
	// lsm303agr_write(FIFO_CTRL_REG_A, 0x00, 'a');				// disable FIFO mode
	// lsm303agr_write(CTRL_REG3_A, 0b00000010, 'a');				// interrupt set on pin1
	// lsm303agr_write(FIFO_CTRL_REG_A, 0b01011111, 'a');				// select FIFO mode, interrupt after 32 samples
	// lsm303agr_bypass_en();
	int_acc = 0;
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start(&htim6);
	lsm303agr_fifo_en();
	/*
	sprintf(buffer, "Waiting for interrupt...\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	*/
	while (int_acc == 0) {
		HAL_Delay(0);
		/*
		sprintf(buffer, "Waiting for interrupt...\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
		*/
	}
	HAL_TIM_Base_Stop(&htim6);
	int_acc = 0;
	sprintf(buffer, "Interrupt arrived, %d samples collected! Took %dms\r\n", FIFO_SIZE, __HAL_TIM_GET_COUNTER(&htim6) / 10);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	// lsm303agr_burst_read();
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start(&htim6);
	lsm303agr_fifo_save();
	lsm303agr_bypass_en();
	HAL_TIM_Base_Stop(&htim6);
	// burst_buf_to_acc_data(accelerationBuffer);
	for (int i = 0; i < FIFO_SIZE; i++) {
		sprintf(buffer, "FIFO[%d]: %f, %f, %f, %d\r\n", i, accelerationBuffer[i].x, accelerationBuffer[i].y, accelerationBuffer[i].z, __HAL_TIM_GET_COUNTER(&htim6) / 10);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	}
	

	/*
	lsm303agr_fifo_en();
	while (int_acc == 0) {
		HAL_Delay(10);
		sprintf(buffer, "Waiting for interrupt...\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	}
	for (int i = 0; i < FIFO_SIZE; i++) {
		lsm303agr_poll_xyz(&accelerationBuffer[i]);
	}
	for (int i = 0; i < FIFO_SIZE; i++) {
		sprintf(buffer, "FIFO[%d]: %f, %f, %f\r\n", i, accelerationBuffer[i].x, accelerationBuffer[i].y, accelerationBuffer[i].z);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	}
	lsm303agr_bypass_en();
	*/
	
	HAL_Delay(1000);
	
}

void lsm303agr_fifo_en(void) {
	lsm303agr_write(CTRL_REG3_A, 0b00000010, 'a');						// enable overflow interrupt
	lsm303agr_write(FIFO_CTRL_REG_A, (uint8_t)(0b01000000 | (uint8_t)FIFO_SIZE), 'a');				// fifo mode, fifo threshold is 32
	lsm303agr_write(CTRL_REG5_A, 0b01000000, 'a');						// enable fifo
}

void lsm303agr_bypass_en(void) {
	lsm303agr_write(CTRL_REG5_A, 0b00000000, 'a');
	lsm303agr_write(CTRL_REG3_A, 0b00000000, 'a');
	lsm303agr_write(FIFO_CTRL_REG_A, 0b00000000, 'a');
}

void lsm303agr_fifo_save(void) {
	for (int i = 0; i < FIFO_SIZE; i++) {
		lsm303agr_poll_xyz(&accelerationBuffer[i]);
	}
}

void lsm303agr_poll_temp(void) {
	tempL = lsm303agr_read(OUT_TEMP_L_A, 'a');
	tempH = lsm303agr_read(OUT_TEMP_H_A, 'a');
	temp = twosCompToInt(tempH) + 25;
}











// fix this later, spent 3 days in total, still unsure why it doesn't work, move on
void lsm303agr_burst_read(void) {
	addr = ADDR_LSM303AGR_ACCEL;
	uint8_t reg = OUT_X_L_A;
	// do not do this, this will not work;
	while (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, &reg, 1, 1000) != HAL_OK) {}
	while (HAL_I2C_Master_Receive(&hi2c1, addr << 1, burstBuf, 6 * FIFO_SIZE, 1000) != HAL_OK) {
		HAL_Delay(100);
		char* buffer;
		sprintf(buffer, "not working?...\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	}
	/*
	for (int i = 0; i < FIFO_SIZE; i++) {
		if (HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)burstBuf + i, 1, 1000) != HAL_OK) {
			char* buffer;
			sprintf(buffer, "HAL_I2C_Mem_Read() failed?...\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
		}
	}
	*/
	/*
	uint16_t a, b;
	while (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, &reg, 1, 1000) != HAL_OK) {}
	while (HAL_I2C_Master_Receive(&hi2c1, addr << 1, &a, 2, 1000) != HAL_OK) {}
	reg += 2;
	while (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, &reg, 1, 1000) != HAL_OK) {}
	while (HAL_I2C_Master_Receive(&hi2c1, addr << 1, &b, 2, 1000) != HAL_OK) {}
	// burstBuf = temp;

	burstBuf[2] = a;
	burstBuf[3] = b;
	while (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, &reg, 1, 1000) != HAL_OK) {}
	while (HAL_I2C_Master_Receive(&hi2c1, addr << 1, temp, 192, 1000) != HAL_OK) {}
		
	// HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, burstBuf, 6 * FIFO_SIZE, 1000);
	
	while (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1) + 1, &reg, 1, 1000) != HAL_OK) {}
	while (HAL_I2C_Master_Receive(&hi2c1, addr << 1, burstBuf, sizeof(burstBuf), 100000) != HAL_OK) {}
	*/
}

void burst_buf_to_acc_data(struct ac* data) {
	for (int i = 0, j = 0; i < FIFO_SIZE * 6; i += 6, j ++) {
		data[j].x = twosCompToInt_10bit((uint16_t)((burstBuf[i+1] << 2) | (burstBuf[i] >> 6)));
		data[j].y = twosCompToInt_10bit((uint16_t)((burstBuf[i+3] << 2) | (burstBuf[i+2] >> 6)));
		data[j].z = twosCompToInt_10bit((uint16_t)((burstBuf[i+5] << 2) | (burstBuf[i+4] >> 6)));
		data[j].x = (data[j].x) / 512.0 * 2.0 ;
		data[j].y = (data[j].y) / 512.0 * 2.0 ;
		data[j].z = (data[j].z) / 512.0 * 2.0 - 1;			// offset gravity
	}
}