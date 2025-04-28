/*
 * MPU6050.c
 *
 *  Created on: Apr 20, 2025
 *      Author: Grétar
 */

#include "MPU6050.h"

float PI = 3.14159265358979323846f;
float GRAVITY = 9.82308f;


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}


//Initialize the communication with the MPU through I2C
void MPU6050_Init(void)
{
	uint8_t check, Data;

	// Check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if(check == 104)
	{
		//Power management register 0x6B, we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		//Set DATA RATE to 1kHz by writing the SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		//Set accelerometer configuration in ACCEL_CONFIG register
		//Writin 0x00: Full scale range will be +/- 250 °/s (can be checked in the respectiv register)
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		//Set gyroscope configuration in GYRO_CONFIG register
		//Writin 0x00: Full scale range will be +/- 2g (can be checked in the respectiv register)
		//Data = 0x00; //+/- 2g
		Data = 0x08; //+/- 4g
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

	}
}


//Read acceleration data
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az)
{
	uint8_t Rec_Data[6]; //(Ax (8bits),Ax (8bits),Ay,Ay,Az,Az)
	int16_t rawAx, rawAy, rawAz;
	//MPU6050_Data data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	rawAx = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); //(ACCEL_XOUT_H << 8 | ACCEL_XOUT_L)
	rawAy = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); //(ACCEL_XOUT_H << 8 | ACCEL_XOUT_L)
	rawAz = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); //(ACCEL_XOUT_H << 8 | ACCEL_XOUT_L)

	//We have to divide according to the full scale value set in FS_SEL
	//I have configured FS_SEL = 0. Therefore we divide by 16384.0
	//according to the datasheet.
	*Ax = rawAx/16384.0;
	*Ay = rawAy/16384.0;
	*Az = rawAz/16384.0;
}

//Read Gyroscope data
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz)
{
	uint8_t Rec_Data[6];
	int16_t rawGx, rawGy, rawGz;
	//MPU6050_Data data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	rawGx = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	rawGy = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	rawGz = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	*Gx = rawGx/131.0;
	*Gy = rawGy/131.0;
	*Gz = rawGz/131.0;
}

//Read temperature
void MPU6050_Read_Temp(float *tempC)
{
	// #define TEMP_OUT_H_REG		0x41
	uint8_t Rec_Data[2]; //TEMP_OUT_H and TEMP_OUT_L
	int16_t rawTemp;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, 1000);

	rawTemp = (Rec_Data[0] << 8) | Rec_Data[1];
	*tempC = (rawTemp / 340.0) + 36.53;
}


//Roll, Pitch, Yaw angle estimates using accelerometer readings
void MPU6050_AngleEstimates(float *Ax, float *Ay, float *Az)
{
	//1. Using this function only is close to true at rest... We need more than only this function!
	//2. Typically we have high-frequency noise -> Apply low-pass filter to measurements!
	//3. Time-varying bias term :( -> How can we estimate and cancel that? -> Initial calibration
	//   We can do initial calibration by averaging some readings at rest and estimate the bias

	float RAD_TO_DEG, phiHat_deg, thetaHat_deg;

	RAD_TO_DEG = 180.0 / PI;
	phiHat_deg   = atanf(*Ay / *Az) * RAD_TO_DEG;  //Roll estimate
	thetaHat_deg = asinf(*Ax / GRAVITY) * RAD_TO_DEG; //Pitch estimate
}


//Roll, Pitch, Yaw angle estimates using gyroscope readings
//Roll is forward (usually x-axis), Pitch is left/right (usually y-axis), yaw is up/down (usually z-axis)
/*
void MPU6050_Gyroscope()
{
	//Gyroscope measures the angular rate of rotation around body axes (how fast our system rotates)
	//omega_b = [p, q, r]   rad/s
	//Gyroscope model: omega_b = omega_true + beta(t) + y(t) where beta is time-varying drift and y is noise
	//Model: [phi_dot ; theta_dot] = [1 sin(phi)tan(theta) cos(phi)tan(theta) ; 0 cos(phi) -sin(theta)]*[p, q, r]
	//Can we now just integrate euler rates phi_dot and theta_dot w.r.t. time? NO! due to time-varying bias + noise
	//It leads to gyro drift
	//To compensate for gyro drift we need something like complementary filter or Kalman filter (we combine the measurements).
	//Accelerometer gives estimates on non-accelerating conditions and gyroscope gives estimate over shorter period of time
	//accelerometers have problems when they are in accelerating motion, gyroscopes on the other hand have rather annoying bias term
}
*/

//#define MPU6050_DEVICEADDRESS 	0x68 << 1;   //0x68 << 1 = 0xD0  208
/*
  HAL_StatusTypeDef IMU6050_RET = HAL_I2C_IsDeviceReady(&hi2c1, 0b1101000 << 1, 1, 100);
  if(IMU6050_RET == HAL_OK)
  {
	  printf("The device is ready.\n");
  }
  else
  {
	  printf("The device is not ready. Check cables.\n");
  }
*/

  //uint8_t checkMPU;
  //uint8_t DataMPU;

	// Check device ID WHO_AM_I
	//HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
  //HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x0D, 1, &DataMPU, 1, 1000);
  //printf("0x0D = 0x%02X\r\n", DataMPU);
  //Prints out 0x0D = 0x50

  //sprintf(logBuf, "%.3f, %.3f\r\n",Ax,Ay);
  //HAL_I2C_Mem_Read(&hi2c1, 0x0D, MemAddress, MemAddSize, pData, Size, Timeout)
  //HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
/*
  HAL_StatusTypeDef IMU6050_RET = HAL_I2C_IsDeviceReady(&hi2c1, 0b1101000 << 1, 1, 100);
    if(IMU6050_RET == HAL_OK)
    {
  	  printf("The device is ready.\n");
    }
    else
    {
  	  printf("The device is not ready. Check cables.\n");
    }
*/
