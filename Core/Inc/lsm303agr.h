#ifndef LSM303AGR_H
#define LSM303AGR_H

#include "main.h"

// #define ADDR_LSM303AGR ((uint16_t)0x19)			// or 0x1E

// slave device address of accelerometer
// Active low for write
#define ADDR_LSM303AGR_ACCEL_R 0b00110011
#define ADDR_LSM303AGR_ACCEL_W 0b00110010
#define ADDR_LSM303AGR_ACCEL 0b0011001


// base address of magnetometer
// Active low for write
#define ADDR_LSM303AGR_MAGNET_R 0b00111101
#define ADDR_LSM303AGR_MAGNET_W 0b00111100
#define ADDR_LSM303AGR_MAGNET 0b0011110


// who am I register address -> belongs to accelerometer and magnetometer
#define LSM303AGR_WHO_AM_I_A 0x0f
#define LSM303AGR_WHO_AM_I_M 0x4f

// Accelerometer registers
#define TEMP_CFG_REG_A 0x1f
#define OUT_TEMP_L_A 0x0c
#define OUT_TEMP_H_A 0x0d
#define CTRL_REG1_A 0x20
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2a
#define OUT_Y_H_A 0x2b
#define OUT_Z_L_A 0x2c
#define OUT_Z_H_A 0x2d
#define FIFO_CTRL_REG_A 0x2e
#define INT2_CFG_A 0x34
#define INT2_THS_A 0x36
#define INT2_DURATION_A 0x37

// Magnetometer registers
#define CFG_REG_A_M 0x60
#define CFG_REG_C_M 0x62
#define OUTX_L_REG_M 0x68
#define OUTX_H_REG_M 0x69
#define OUTY_L_REG_M 0x6A
#define OUTY_H_REG_M 0x6B
#define OUTZ_L_REG_M 0x6C
#define OUTZ_H_REG_M 0x6D

// constants
#define FIFO_SIZE 10

// interrupt var and state
extern int int_acc;
extern int int_crash;
extern int xState;	

/* 
	Calibration data:
	
	1. magnetic declination based on location for direction accuracy
	2. Typical crash G-force values: <1 for normal driving conditions, else crash
*/

// location based declination angle for direction, (-10, 14deg)
#define MAG_DECLINATION_TORONTO -10.23333333333
#define CRASH_G_FORCE_THRESHOLD 1.0

// structs
struct ac {
	double x;
	double y;
	double z;
};

// extern stuff
extern struct ac accelerationBuffer[FIFO_SIZE];
extern int temp;

struct mg {
	double x;
	double y;
	double z;
	char* direction;
};

uint8_t lsm303agr_read(uint8_t reg, char dev);
HAL_StatusTypeDef lsm303agr_write(uint8_t reg, uint8_t val, char dev);
void lsm303agr_init(void);
void lsm303agr_poll_xyz(struct ac* data);
void lsm303agr_poll_xyz_mag(void);
void lsm303agr_fifo_en(void);
void lsm303agr_bypass_en(void);
void lsm303agr_burst_read(void);
void burst_buf_to_acc_data(struct ac* data);
void lsm303agr_fifo_save(void);
void lsm303agr_poll_temp(void);
extern struct mg magneticField;

#endif