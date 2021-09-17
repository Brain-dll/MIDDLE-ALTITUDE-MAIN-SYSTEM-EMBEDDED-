/*
 * MPU-6050.h
 *
 *  Created on: Apr 23, 2021
 *      Author: Osman ÇİÇEK
 */

#include "MPU-6050.h"
#include "math.h"

const float PI = 3.1415926535;

uint8_t Acc[6] = {0};
uint8_t Gyro[6] = {0};
int16_t Accx, Accy, Accz;
int16_t Gyrox, Gyroy, Gyroz;
uint8_t count = 0;

uint8_t SMPRT_DIV = 0x19;
uint8_t CONFIG = 0x1A;
uint8_t GYRO_CONFIG = 0x1B;
uint8_t ACCEL_CONFIG = 0x1C;
uint8_t PWR_MGMT_1 = 0x6B;
uint8_t MPU_6050_ACCELERATION_REG = 0x3B;
uint8_t MPU_6050_GYRO_REG = 0x43;

uint8_t SMPRT_DIV_RATE = 0x07;
uint8_t GYRO_CONFIG_RATE = 0x08;
uint8_t CONFIG_RATE = 0x02;
uint8_t ACCEL_CONFIG_RATE = 0x10;
uint8_t PWR_MGMT_1_RATE = 0x00;

extern float AcceX, AcceY, AcceZ;
extern float GyroX, GyroY, GyroZ;

float AcceErrorX = .0, AcceErrorY = .0, GyroErrorX = .0, GyroErrorY = .0, GyroErrorZ = .0;


void MPU6050_Init()
{
	if(HAL_I2C_IsDeviceReady(&hi2c2, MPU_6050_WRITE_ADRESS, 1, MPU_6050_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	}

	MPU6050_SetCalibration();
	//CalculateError();
}

void MPU6050_SetCalibration()
{
	// This register allows the user to configure the power mode and clock source.
	HAL_I2C_Mem_Write(&hi2c2, MPU_6050_WRITE_ADRESS, PWR_MGMT_1, 1, &PWR_MGMT_1_RATE, 1, MPU_6050_TIMEOUT);

	// The Sample Rate is determined by dividing the gyroscope output rate by this value.
	HAL_I2C_Mem_Write(&hi2c2, MPU_6050_WRITE_ADRESS, SMPRT_DIV, 1, &SMPRT_DIV_RATE, 1, MPU_6050_TIMEOUT);

	// This register is used to trigger gyroscope self-test and configure the gyroscopes’ full scale range.
	HAL_I2C_Mem_Write(&hi2c2, MPU_6050_WRITE_ADRESS, GYRO_CONFIG, 1, &GYRO_CONFIG_RATE, 1, MPU_6050_TIMEOUT);

	// This register is used to trigger accelerometer self test and configure the accelerometer full scale range.
	HAL_I2C_Mem_Write(&hi2c2, MPU_6050_WRITE_ADRESS, ACCEL_CONFIG, 1, &ACCEL_CONFIG_RATE, 1, MPU_6050_TIMEOUT);

	HAL_Delay(50);

}

void MPU6050_Read_Accelarition()
{
	HAL_I2C_Mem_Read(&hi2c2, MPU_6050_READ_ADRESS, MPU_6050_ACCELERATION_REG, 1, Acc, MPU_6050_ACCELERATION_LENGTH, MPU_6050_TIMEOUT);
	//HAL_Delay(20);
	//HAL_Delay(12);
	Accx = (int16_t)((Acc[count] << 8) | (Acc[count + 1])); count +=2;
	Accy = (int16_t)((Acc[count] << 8) | (Acc[count + 1])); count +=2;
	Accz = (int16_t)((Acc[count] << 8) | (Acc[count + 1])); count +=2;
	count = 0;
}

void MPU6050_Read_Gyro()
{
	HAL_I2C_Mem_Read(&hi2c2, MPU_6050_READ_ADRESS, MPU_6050_GYRO_REG, 1, Gyro, MPU_6050_GYRO_LENGTH, MPU_6050_TIMEOUT);
	//HAL_Delay(20);
	//HAL_Delay(20);
	Gyrox = (int16_t)((Gyro[count] << 8) | (Gyro[count + 1])); count +=2;
	Gyroy = (int16_t)((Gyro[count] << 8) | (Gyro[count + 1])); count +=2;
	Gyroz = (int16_t)((Gyro[count] << 8) | (Gyro[count + 1])); count +=2;
	count = 0;
}

float MPU6050_AccX()
{
	MPU6050_Read_Accelarition();
	return (float)Accx / 4096.0;
}
float MPU6050_AccY()
{
	MPU6050_Read_Accelarition();
	return (float)Accy / 4096.0;
}
float MPU6050_AccZ()
{
	MPU6050_Read_Accelarition();
	return (float)Accz / 4096.0;
}
float MPU6050_GyroX()
{
	MPU6050_Read_Gyro();
	return (float)Gyrox / 65.5;
}
float MPU6050_GyroY()
{
	MPU6050_Read_Gyro();
	return (float)Gyroy / 65.5;
}
float MPU6050_GyroZ()
{
	MPU6050_Read_Gyro();
	return (float)Gyroz / 65.5;
}

float Array_sort_MPU(float *array, int n) {
	int i = 0, j = 0;
	float temp = 0.0;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n - 1; j++) {
			if (array[j] > array[j + 1]) {
				temp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = temp;
			}
		}
	}
	//return array[2];
	return array[n / 2];
}

float MEDIAN_AXIS_FILTER(char device, char axis)
{
	float X[5] = {0}, Y[5] = {0}, Z[5] = {0};
	switch (device) {
	case 'A':
	switch (axis) {
	case 'X':
		for (uint8_t i = 0; i < 5; i++)
			X[i] = MPU6050_AccX();
		return Array_sort_MPU(X, 5);
		break;
	case 'Y':
		for (uint8_t i = 0; i < 5; i++)
			Y[i] = MPU6050_AccY();
		return Array_sort_MPU(Y, 5);
		break;
	case 'Z':
		for (uint8_t i = 0; i < 5; i++)
			Z[i] = MPU6050_AccZ();
		return Array_sort_MPU(Z, 5);
		break;
	default:
		for (uint8_t i = 0; i < 5; i++)
			Z[i] = MPU6050_AccZ();
		return Array_sort_MPU(Z, 5);
	}
	break;
	case 'G':
	switch (axis) {
	case 'X':
		for (uint8_t i = 0; i < 5; i++)
			X[i] = MPU6050_GyroX();
		return Array_sort_MPU(X, 5);
		break;
	case 'Y':
		for (uint8_t i = 0; i < 5; i++)
			Y[i] = MPU6050_GyroY();
		return Array_sort_MPU(Y, 5);
		break;
	case 'Z':
		for (uint8_t i = 0; i < 5; i++)
			Z[i] = MPU6050_GyroZ();
		return Array_sort_MPU(Z, 5);
		break;
	default:
		for (uint8_t i = 0; i < 5; i++)
			Z[i] = MPU6050_GyroZ();
		return Array_sort_MPU(Z, 5);
	}
	break;
	default:
		for (uint8_t i = 0; i < 5; i++)
			Z[i] = MPU6050_AccZ();
		return Array_sort_MPU(Z, 5);
	}
}

void CalculateError(void){
	for (uint16_t i = 0; i < 2000; i++) {
		AcceX = MPU6050_AccX();
		AcceY = MPU6050_AccY();
		AcceZ = MPU6050_AccZ();

		AcceErrorX += ((atan((AcceY) / sqrt(pow((AcceX), 2) + pow((AcceZ), 2)))
				* 180 / PI));
		AcceErrorY += ((atan(
				-1 * (AcceX) / sqrt(pow((AcceY), 2) + pow((AcceZ), 2))) * 180
				/ PI));
	}

	AcceErrorX /= 2000.0;
	AcceErrorY /= 2000.0;

	for (uint16_t i = 0; i < 2000; i++) {
		GyroX = MPU6050_GyroX();
		GyroY = MPU6050_GyroY();
		GyroZ = MPU6050_GyroZ();

		GyroErrorX += GyroX;
		GyroErrorY += GyroY;
		GyroErrorZ += GyroZ;
	}


	GyroErrorX /= 2000.0;
	GyroErrorY /= 2000.0;
	GyroErrorZ /= 2000.0;
}



