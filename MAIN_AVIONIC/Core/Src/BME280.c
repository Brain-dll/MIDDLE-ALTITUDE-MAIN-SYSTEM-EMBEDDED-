/*
 * BME280.c
 *
 *  Created on: Apr 18, 2021
 *      Author: Osman ÇİÇEK
 */

#include "BME280.h"
#include"math.h"

//float BASE_P[9] = {0}, BASE = 0;
float BASE_P[3] = {0}, BASE = 0;
float P0 = 1013.25;

void BME280_Init()
{
	if(HAL_I2C_IsDeviceReady(&hi2c2, BME280_DEVICE_WRITE_ADRESS, 1, 100) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	}

	BME280_GetCalibration_Value();
	BME280_SetCalibration();

	BASE = 0;
	for (uint8_t i = 0; i < sizeof(BASE_P) / sizeof(BASE_P[0]); i++) {
		//BASE_P[i] = BME280_Get_Altitude();
		BASE_P[i] = BME280_ALT_MEDIANFILTER();
	}
	BASE = Array_sort_BME(BASE_P, sizeof(BASE_P) / sizeof(BASE_P[0]));
}

void BME280_GetCalibration_Value()
{
	uint8_t a = 0;
	uint8_t Cal_Buff[BME280_DEVICE_CALIBRATION_WORDS_LENGHT] = {0};

	HAL_I2C_Mem_Read(&hi2c2, BME280_DEVICE_READ_ADRESS, BME280_CALIBRATION_START_ADDRESS, 1,
			Cal_Buff, BME280_DEVICE_CALIBRATION_WORDS_LENGHT, 100);

	dig_T1 = (uint16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_T2 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_T3 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;

	dig_P1 = (uint16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P2 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P3 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P4 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P5 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P6 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P7 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P8 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
	dig_P9 = (int16_t)((Cal_Buff[a+1] << 8) | Cal_Buff [a]); a+=2;
}

void BME280_SetCalibration(void)
{
	WData[0] = 0x08; /*0xF0;*/   //  0.5ms t(standby)  X2 IIR filter coefficient
	WData[1] = 0xB7;  // X16 sample rate of temperature and pressure measurement
//	WData[0] = 0x08;	// 0.5ms t(standby)  X2 IIR filter coefficient
//	WData[1] = 0x77;	// X4 sample rate of temperature measurement, X16 sample rate of pressure measurement
	HAL_I2C_Mem_Write(&hi2c2, BME280_DEVICE_WRITE_ADRESS, 0xF5, 1, &WData[0], 1, 100);
	HAL_Delay(20);
	HAL_I2C_Mem_Write(&hi2c2, BME280_DEVICE_WRITE_ADRESS, 0xF4, 1, &WData[1], 1, 100);
	HAL_Delay(20);
}

int32_t BME280_Get_Uncompansated_Temperature(void)
{
	uint8_t tData[3] = {0};

	HAL_I2C_Mem_Read(&hi2c2, BME280_DEVICE_READ_ADRESS, 0xFA, 1, tData, 3, 100);

	unCompTemperature = (uint32_t)((tData[0] << 12) | (tData[1] << 4) | (tData[2] >> 4));

	return (int32_t)unCompTemperature;
}

float BME280_Get_Temperature(void)
{
	int32_t adc_T = BME280_Get_Uncompansated_Temperature();
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = (var1 + var2);
	T = (t_fine * 5 + 128) >> 8;
	return ((float)T) / 100.0;
}

uint32_t BME280_Get_Uncompansated_Pressure(void)
{
	uint8_t PData[3] = {0};

	HAL_I2C_Mem_Read(&hi2c2, BME280_DEVICE_READ_ADRESS, 0xF7, 1, PData, 3, 100);

	unCompPressure = (uint32_t)((PData[0] << 12) | (PData[1] << 4) | (PData[2] >> 4));

	return unCompPressure;
}

float BME280_Get_Pressure(void)
{
	int32_t adc_P = BME280_Get_Uncompansated_Pressure();
	int32_t varp1 = 0,varp2 = 0;
	uint32_t p;
	varp1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	varp2 = (((varp1 >> 2) * (varp1 >> 2)) >> 11) * ((int32_t)dig_P6);
	varp2 = varp2 + ((varp1 * ((int32_t)dig_P5)) << 1);
	varp2 = (varp2 >> 2) + (((int32_t)dig_P4) << 16);
	varp1 = (((dig_P3 * (((varp1 >> 2) * (varp1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * varp1) >> 1)) >> 18;
	varp1 = ((((32768 + varp1)) * ((int32_t)dig_P1)) >> 15);
	if (varp1 == 0)
		return 0;

	p = (((uint32_t)(((int32_t)1048576) - adc_P) - (varp2 >> 12))) * 3125;
	if (p < 0x80000000)
		p = (p << 1) / ((uint32_t)varp1);
	else
		p = (p / (uint32_t)varp1) * 2;

	varp1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >>12;
	varp2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((varp1 + varp2 + dig_P7) >> 4));
	return (float)p / 100.0;
}

float BME280_Get_Altitude()
{
	float BPressure = BME280_Get_Pressure();
	float temperature = BME280_Get_Temperature();
	//Altitude = 44330 * (1 - pow((BPressure / 1013.25),(1/5.255)));
	Altitude = ((pow((P0/BPressure), (1/5.257))-1) * (temperature + 273.15)) / 0.0065;
	return (Altitude - BASE);
}

float Array_sort_BME(float *array, int n) {
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

float BME280_ALT_MEDIANFILTER(void)
{
	float AL[filter_number] = {0}, result = 0.0;
	for (uint8_t i = 0 ; i < sizeof(AL) / sizeof(AL[0]) ; i++)
		AL[i] = BME280_Get_Altitude();
	result = Array_sort_BME(AL,sizeof(AL) / sizeof(AL[0]));
	return result;
}











