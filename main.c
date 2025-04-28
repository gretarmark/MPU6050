//This code is supposed to be used in a STM32 project.

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include <stdio.h>
/* USER CODE END Includes */



/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Ax, Ay, Az, Gx, Gy, Gz, tempC;

/* USER CODE END 0 */



 MPU6050_Init();




int main(void)
{

  /* USER CODE BEGIN 2 */

  MPU6050_Init();

  /* USER CODE END 2 */




  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    MPU6050_Read_Accel(&Ax, &Ay, &Az); //Read acceleration data
    MPU6050_Read_Gyro(&Gx, &Gy, &Gz); //Read Gyroscope data
    MPU6050_Read_Temp(&tempC);

  }
  /* USER CODE END 3 */
}
