/*
 * MPU6050.h
 *
 *  Created on: Apr 20, 2025
 *      Author: Grétar
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "main.h"
#include "stm32f4xx_hal.h"  // For I2C functions
#include "main.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
//extern SPI_HandleTypeDef   hi2c1;

//I2C addresses are 8 bits on the form [ A6 A5 A4 A3 A2 A1 A0 R/W ].
//MPU6050 addresses are 7 bits: 0x68 is 0110 1000 and we shift it to get 1101 0000 using (0x68 << 1).
//We shift everything left by one bit, making room for the read/write bit.
//To write, we set R/W = 0
//To read, we set R/W = 1: 0x68 << 1 | 0x01  ->  0b11010000 | 0b00000001 = 0b11010001 = 0xD1

#define MPU6050_ADDR 		0xD0 //0x68 << 1 = 0xD0  208

#define SMPLRT_DIV_REG		0x19

#define GYRO_CONFIG_REG		0X1B
#define GYRO_XOUT_H_REG		0x43

#define ACCEL_CONFIG_REG	0x1C
#define ACCEL_XOUT_H_REG	0x3B

#define FIFO_EN				0x23 //Controls what data gets written into FIFO buffer

#define TEMP_OUT_H_REG		0x41

#define PWR_MGMT_1_REG		0x6B
#define WHO_AM_I_REG		0x75


/*
typedef struct {
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
    float Temperature;
} MPU6050_Data;
*/

/*
 * Add:
 * int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;


???
uint16_t ADC_Data = 0;
int ADC_Converted = 0;
???
 */


long map(long x, long in_min, long in_max, long out_min, long out_max);
void MPU6050_Init(void); //Initialize the communication with the MPU through I2C
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az); //Read acceleration data
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz); //Read Gyroscope data
void MPU6050_Read_Temp(float *tempC); //Read temperature data
void MPU6050_AngleEstimates(float *Ax, float *Ay, float *Az);


#endif /* INC_MPU6050_H_ */
