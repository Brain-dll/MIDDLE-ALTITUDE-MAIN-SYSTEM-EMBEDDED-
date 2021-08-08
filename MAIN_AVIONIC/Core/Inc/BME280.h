/*
 * BME280.h
 *
 *  Created on: Apr 18, 2021
 *      Author: Osman ÇİÇEK
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "stm32f3xx_hal.h"

/* BME280 Device Addresses */
#define BME280_DEVICE_WRITE_ADRESS 0xEC
#define BME280_DEVICE_READ_ADRESS 0xED

/* BME280 Calibration Value Length*/
#define BME280_DEVICE_HUMIDITIY_LENGHT 6
#define BME280_DEVICE_TEMPERATURE_LENGHT 3
#define BME280_DEVICE_PRESSURE_LENGHT 6
#define BME280_DEVICE_CALIBRATION_WORDS_LENGHT 24

/* BME280 Calibration Address */
#define BME280_CALIBRATION_START_ADDRESS 0x88

#define filter_number 30

/* Calibration Values */
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

/* Temperature Values */
int32_t unCompTemperature;
int32_t unCompPressure;
int32_t t_fine;
uint8_t WData[2];

float Altitude;

extern I2C_HandleTypeDef hi2c2;

void BME280_Init(void);
void BME280_GetCalibration(void);
void BME280_GetCalibration_Value(void);
int32_t BME280_Get_Uncompansated_Temperature(void);
float BME280_Get_Temperature(void);
uint32_t BME280_Get_Uncompansated_Pressure(void);
float BME280_Get_Pressure(void);
void BME280_SetCalibration(void);
float BME280_Get_Altitude(void);
float Array_sort_BME(float *array, int n);
float BME280_ALT_MEDIANFILTER(void);
//uint32_t BME280_Get_Uncompansated_All(void);
//double BME280_Get_Pressure_D(int32_t adc_P);
//double compensate_pressure(int32_t adc_P);
//static int32_t compensate_temperature(adc_T);
#endif /* INC_BME280_H_ */
