/*
 * MPU-6050.h
 *
 *  Created on: Apr 23, 2021
 *      Author: Osman ÇİÇEK
 */

#ifndef INC_MPU_6050_H_
#define INC_MPU_6050_H_

#include "stm32f3xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

 // MPU_6050 I2C ADRESS
#define MPU_6050_READ_ADRESS  0xD1
#define MPU_6050_WRITE_ADRESS  0xD0

/*#define SMPRT_DIV  0x19
#define CONFIG  0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG  0x1C
#define PWR_MGMT_1  0x6B
#define MPU_6050_ACCELERATION_REG  0x3B
#define MPU_6050_GYRO_REG  0x43*/

#define MPU_6050_ACCELERATION_LENGTH 6
#define MPU_6050_GYRO_LENGTH 6

#define MPU_6050_TIMEOUT  100

void MPU6050_Init(void);
void MPU6050_SetCalibration(void);
void MPU6050_Read_Accelarition(void);
void MPU6050_Read_Gyro(void);
float MPU6050_AccX(void);
float MPU6050_AccY(void);
float MPU6050_AccZ(void);
float MPU6050_GyroX(void);
float MPU6050_GyroY(void);
float MPU6050_GyroZ(void);
float Array_sort_MPU(float *array, int n);
float MEDIAN_AXIS_FILTER(char device, char axis);
void CalculateError(void);

#endif /* INC_MPU_6050_H_ */
