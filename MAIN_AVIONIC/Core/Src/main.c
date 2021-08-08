/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280.h"
#include "MPU-6050.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define t 200
#define PI 3.14159265358979323846
#define ACC_l 1.5  // kalkisi anlamak icin gerekli ivme eşigi (pozitif olmali)***

// değişebilir offset değerleri
#define gyroOffsetX -0.22
#define gyroOffsetY  1.11
#define gyroOffsetZ -5.64
#define acceOffsetX  0.38
#define acceOffsetY -1.93

float AcceX = .0, AcceY = .0, AcceZ = .0;
float GyroX = .0, GyroY = .0, GyroZ = .0;
float AcceAngleX = .0, AcceAngleY = .0, GyroAngleX = .0, GyroAngleY = .0, GyroAngleZ = .0;
float GyroBound = 3.0;
float roll = .0, pitch = .0;


uint32_t CurrentTime = 0, PreviousTime = 0;
float ElapsedTime = .0;

float calcoor(float x);
void LORA_READ_PARAMETER(void);
void LORA_CONFG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);

float A = 0, T, P, V;
float BM_V[6] = {0}, BM_A[6] = {0};
float MPU_ACC_Z[6] = {0}, roll, pitch, rollFilter, pitchFilter;
float lat;
float lng;

int tim1 = 0, tim2 = 0, dif = 0;
int time = 0;

uint8_t BM_S = 0;
uint8_t apogee = 0, alt_l = 0;
uint8_t MPU_FAIL = 0, MPU = 0, MPU_ACC_S = 0, BME = 0;
uint8_t GGA = 0;
uint8_t k = 0;
uint8_t v = 0;
uint8_t warning = 0;
uint8_t apg = 0;
uint8_t mn = 0;
uint8_t MPU_RP = 0;
uint8_t counter_Tel = 0;

char RX_BUF[512];
char GPGGA[75];
char LAT[9];
char LONG[10];
char TIME[10];
char COOR[60];

uint8_t RF = 0;
uint8_t ADDH = 0x6;
uint8_t ADDL = 0x4A;
uint8_t CHN = 0xA;
uint8_t MODE = 1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	for (uint16_t i = 0; i < 512; i++) {

		if (RX_BUF[i] == '$' && RX_BUF[i + 1] == 'G' && RX_BUF[i + 2] == 'P'
				&& RX_BUF[i + 3] == 'G' && RX_BUF[i + 4] == 'G'
				&& RX_BUF[i + 5] == 'A' && RX_BUF[i + 6] == ',') {
			for (uint8_t n = 0; n < sizeof(GPGGA); n++) {
				GPGGA[n] = RX_BUF[i + n];
				if (n > 5 && GPGGA[n - 1] == '\r' && GPGGA[n] == '\n') {
					GGA = 1;
					break;
				}
			}
			if (GGA == 1) {

				uint8_t v = 0, pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0, pos5 = 0,
						ok1 = 0, ok2 = 0, ok3 = 0, ok4 = 0;
				for (uint8_t n = 0; n < sizeof(GPGGA); n++) {
					if (GPGGA[n] == ',')
						v++;
					if (v == 1 && ok1 == 0) {
						pos1 = n;
						ok1 = 1;
					}
					if (v == 2 && ok2 == 0) {
						pos2 = n;
						for (uint8_t count = 0; count < pos2 - pos1; count++)
							TIME[count] = GPGGA[pos1 + count + 1];
						ok2 = 1;
					}
					if (v == 3 && ok3 == 0) {
						pos3 = n;
						for (uint8_t count = 0; count < pos3 - pos2; count++)
							LAT[count] = GPGGA[pos2 + count + 1];
						ok3 = 1;
					}
					if (v == 4 && ok4 == 0) {
						pos4 = n;
						ok4 = 1;
					}
					if (v == 5) {
						pos5 = n;
						for (uint8_t count2 = 0; count2 < pos5 - pos4; count2++)
							LONG[count2] = GPGGA[pos4 + count2 + 1];

						strcpy(TIME, TIME);
						strcpy(LAT, LAT);
						strcpy(LONG, LONG);
						time = atof(TIME);
						time = time % 100;
						lat = calcoor(atof(LAT));
						lng = calcoor(atof(LONG));
						/*sprintf(COOR, ":%2.7f:%2.7f\n", lat, lng);
						 HAL_UART_Transmit(&huart3, (uint8_t*) COOR,
						 sizeof(COOR), 1000);*/
						for (uint16_t c = 0; c < sizeof(RX_BUF); c++)
							RX_BUF[c] = '\0';
						for (uint16_t c = 0; c < sizeof(GPGGA); c++)
							GPGGA[c] = '\0';
						for (uint16_t c = 0; c < sizeof(LAT); c++)
							LAT[c] = '\0';
						for (uint16_t c = 0; c < sizeof(LONG); c++)
							LONG[c] = '\0';
						for (uint16_t c = 0; c < sizeof(TIME); c++)
							TIME[c] = '\0';
						break;
					}
				}
			}
		}
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
//extern uint32_t SystemCoreClock;
//uint32_t Systemlock = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);*/
	LORA_READ_PARAMETER();  // don't make commit all of these two functions
	LORA_CONFG(0x6, 0x4A, 0xA, 1);   // HIGH ADDRESS, LOW ADDRESS, CHANNEL, MODE (0 : TRANSPARENT, 1 : FIXED)

	BME280_Init();
	MPU6050_Init();

	enum device {
		Ephemerish, Payload
	} dev_ID;
	//dev_ID = Ephemerish;
	dev_ID = Payload;

	enum rocket {
		Rail, Launch, Burnout, Apogee, Descent, Main, Recovery
	};
	enum rocket EPHEMERISH;

	for (uint8_t i = 0; i < 12; i++) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(50);
	}
	HAL_TIM_Base_Start_IT(&htim2);
	if (MPU_FAIL == 0) {
		EPHEMERISH = Rail;
		HAL_TIM_Base_Start_IT(&htim6);
	} else if (MPU_FAIL == 1) {
		EPHEMERISH = Launch;

	}
	HAL_TIM_Base_Start(&htim15);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim16);
	//
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)RX_BUF, 512);
	//
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //ElapsedTime = 0, CurrentTime = 0, PreviousTime = 0;
	  PreviousTime = CurrentTime;
	  CurrentTime = __HAL_TIM_GET_COUNTER(&htim15);
	  if(CurrentTime < PreviousTime)
		  CurrentTime += 65535;
	  ElapsedTime = (float)(CurrentTime - PreviousTime) / 1000.0;

		if (EPHEMERISH != Rail && MPU_RP == 1) {

			AcceX = MPU6050_AccX();
			AcceY = MPU6050_AccY();
			AcceZ = MPU6050_AccZ();

			AcceAngleX = ((atan(
					(AcceY) / sqrt(pow((AcceX), 2) + pow((AcceZ), 2))) * 180
					/ PI)) - acceOffsetX;
			AcceAngleY = ((atan(
					-1 * (AcceX) / sqrt(pow((AcceY), 2) + pow((AcceZ), 2)))
					* 180 / PI)) - acceOffsetY;

			GyroX = MPU6050_GyroX();
			GyroY = MPU6050_GyroY();
			GyroZ = MPU6050_GyroZ();

			// handle gyro errors
			GyroX = GyroX - gyroOffsetX;
			GyroY = GyroY - gyroOffsetY;
			GyroZ = GyroZ - gyroOffsetZ;

			if (abs(GyroX) < GyroBound) {
				GyroX = 0;
			}
			if (abs(GyroY) < GyroBound) {
				GyroY = 0;
			}
			if (abs(GyroZ) < GyroBound + 3) {
				GyroZ = 0;
			}

		GyroAngleX += (GyroX * ElapsedTime);
		GyroAngleY += (GyroY * ElapsedTime);

		roll = 0.90 * GyroAngleX + 0.10 * AcceAngleX;
		pitch = 0.90 * GyroAngleY + 0.10 * AcceAngleY;

		MPU_RP = 0;
		}

		/*
		 roll = atan(Az / sqrt(pow(Ay, 2) + pow(Ax, 2))) * 180 / PI;
		 pitch = atan(-1 * Ay / sqrt(pow(Az, 2) + pow(Ax, 2))) * 180 / PI;

		 A1 = BME280_Get_Altitude();
		 V = (A - A1);

		 rollFilter = 0.94 * rollFilter + 0.06 * roll;
		 pitchFilter = 0.94 * pitchFilter + 0.06 * pitch;
		 BM[0] = BME280_ALT_MEDIANFILTER();*/
		switch (EPHEMERISH) {
		case Rail:
			if (MPU == 1) {
				MPU_ACC_Z[MPU_ACC_S] = MEDIAN_AXIS_FILTER('A', 'Z');
				if (MPU_ACC_S == 5) {
					uint8_t C1 = 0;
					for (uint8_t i = 0; i <= MPU_ACC_S; i++) {
						if (MPU_ACC_Z[i] < (-1 * ACC_l) || MPU_ACC_Z[i] > ACC_l)
							C1++;
					}
					if (C1 >= 3) {
						EPHEMERISH = Launch;
						for (uint8_t i = 0; i < (EPHEMERISH * 2); i++) {
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
							HAL_Delay(50);
						}
						HAL_TIM_Base_Start(&htim1);
					}
					MPU_ACC_S = 0;
				}
				MPU_ACC_S++;
				MPU = 0;
			}
			break;
		case Launch:
			if (BME == 1) {
				if (alt_l == 0) {
					tim1 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[0] = BME280_ALT_MEDIANFILTER();
					alt_l = 1;
				} else if (alt_l == 1) {
					tim2 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[1] = BME280_ALT_MEDIANFILTER();
					if (tim2 < tim1) {
						tim2 = tim2 + 65535;
						dif = tim2 - tim1;
					} else
						dif = tim2 - tim1;
					BM_V[BM_S] = (BM_A[1] - BM_A[0]) / (0.001 * dif);
					if (BM_S == 5) {
						uint8_t M = 0;
						for (uint8_t i = 0; i <= BM_S; i++) {
							if (BM_V[i] > 20) // TEST BURNOUT VALUE = 20 READ BURNOUT VALUE = 100
								M++;
						}
						if (M >= 3) {
							//**********************
							EPHEMERISH = Burnout;
							for (uint8_t i = 0; i < (EPHEMERISH * 2); i++) {
								HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
								HAL_Delay(50);
							}
						}
						for (uint8_t k = 0; k <= 5; k++)
							BM_V[k] = 0;
						BM_S = 0;
					}
					BM_S++;
					alt_l = 0;
				}
				BME = 0;
			}
			break;
		case Burnout:
			if (BME == 1) {
				if (alt_l == 0) {
					tim1 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[0] = BME280_ALT_MEDIANFILTER();
					alt_l = 1;
				} else if (alt_l == 1) {
					tim2 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[1] = BME280_ALT_MEDIANFILTER();
					if (tim2 < tim1) {
						tim2 = tim2 + 65535;
						dif = tim2 - tim1;
					} else
						dif = tim2 - tim1;
					BM_V[BM_S] = (BM_A[1] - BM_A[0]) / (0.001 * dif);
					if (BM_S == 5) {
						uint8_t M = 0;
						for (uint8_t i = 0; i <= BM_S; i++) {
							if (BM_V[i] < 10)    //  if (BM_V[i] < 20)
								M++;
						}
						if (M >= 3) {
							EPHEMERISH = Apogee;
							HAL_TIM_Base_Stop_IT(&htim2);
							tim1 = __HAL_TIM_GET_COUNTER(&htim1);
							for (uint8_t i = 0; i < (EPHEMERISH * 2); i++) {
								HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
								HAL_Delay(50);
							}
						}
						for (uint8_t k = 0; k <= 5; k++)
							BM_V[k] = 0;
						BM_S = 0;
					}
					BM_S++;
					alt_l = 0;
				}
				BME = 0;
			}
			break;
		case Apogee:
			//EPHEMERISH = Recovery;
			/******  GYRO ALGORITHM  ******/
			tim2 = __HAL_TIM_GET_COUNTER(&htim1);
			if (tim2 < tim1) {
				tim2 = tim2 + 65535;
				dif = tim2 - tim1;
			} else
				dif = tim2 - tim1;
			if (dif > 1700 /*||  GYRO  */) {
				EPHEMERISH = Descent;
				for (uint8_t i = 0; i < (EPHEMERISH * 2); i++) {
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					HAL_Delay(50);
				}
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, SET);
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SET);  // ***************
				HAL_Delay(600);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET);  // **************
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				apg = 1;
				HAL_TIM_Base_Start_IT(&htim2);
			}
			break;
		case Descent:
			if (BME == 1) {

				if (alt_l == 0) {
					tim1 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[0] = BME280_ALT_MEDIANFILTER();
					alt_l = 1;
				} else if (alt_l == 1) {
					tim2 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[1] = BME280_ALT_MEDIANFILTER();
					if (tim2 < tim1) {
						tim2 = tim2 + 65535;
						dif = tim2 - tim1;
					} else
						dif = tim2 - tim1;
					BM_V[BM_S/*(BM_S - 1) / 2*/] = (BM_A[1] - BM_A[0]) / (0.001 * dif);
					alt_l = 0;  //**************
				}

				BM_A[BM_S] = BME280_ALT_MEDIANFILTER();
				if (BM_S == 5) {
					uint8_t M = 0;
					for (uint8_t i = 0; i <= BM_S; i++) {
						if (BM_A[i] < 500)
							M++;
					}
					if (M >= 3) {
						// MAIN FIRE
						EPHEMERISH = Main;
						for (uint8_t i = 0; i < (EPHEMERISH * 2); i++) {
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
							HAL_Delay(50);
						}
					}
					for (uint8_t k = 0; k <= 5; k++)
						BM_A[k] = 0;
					BM_S = 0;
				}
				BM_S++;
				BME = 0;
			}
			break;
		case Main:
			EPHEMERISH = Recovery;
			for (uint8_t i = 0; i < (EPHEMERISH * 2); i++) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_Delay(50);
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SET);
			HAL_Delay(600);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
			mn = 1;
			break;
		case Recovery:
			if (BME == 1) {
				if (alt_l == 0) {
					tim1 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[0] = BME280_ALT_MEDIANFILTER();
					alt_l = 1;
				} else if (alt_l == 1) {
					tim2 = __HAL_TIM_GET_COUNTER(&htim1);
					BM_A[1] = BME280_ALT_MEDIANFILTER();
					if (tim2 < tim1) {
						tim2 = tim2 + 65535;
						dif = tim2 - tim1;
					} else
						dif = tim2 - tim1;
					BM_V[BM_S] = (BM_A[1] - BM_A[0]) / (0.001 * dif);
					//BM_V[(BM_S - 1) / 2] = (BM_A[1] - BM_A[0]) / (0.001 * dif);
					alt_l = 0;  //**************
					BM_S++;
					if(BM_S == 5) BM_S = 0;
				}
			}
			if (warning == 1) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_Delay(50);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				warning = 0;
			}
			break;
		}
		//if (RF == 1 && ((time % 6) == 0 || (time % 6) == 1))  	// EPHEMERISH
		if(RF == 1 && ((time % 6) == 3 || (time % 6) == 4)) // PAYLOAD
				{
			if (EPHEMERISH == Rail && counter_Tel < 10) {
				COOR[0] = ADDH;
				COOR[1] = ADDL;
				COOR[2] = CHN;
				sprintf(COOR, "%s%d:%d:%2.7f:%2.7f:%.2f:%.2f:%.2f,%.2f:%d:%d\n",
						COOR, dev_ID, time, lat, lng, BM_A[1], BM_V[2], roll,
						pitch, apg, mn);
				HAL_UART_Transmit(&huart2, (uint8_t*) COOR, sizeof(COOR), 1000);
				for (uint16_t X = 0; X < sizeof(COOR); X++) {
					COOR[X] = '\0';
				}
//				lat = .0;
//				lng = .0;
//				RF = 0;
				counter_Tel++;
			} else if (EPHEMERISH != Rail) {
				COOR[0] = ADDH;
				COOR[1] = ADDL;
				COOR[2] = CHN;
				sprintf(COOR, "%s%d:%d:%2.7f:%2.7f:%.2f:%.2f:%.2f,%.2f:%d:%d\n",
						COOR, dev_ID, time, lat, lng, BM_A[1], BM_V[2], roll,
						pitch, apg, mn);
				HAL_UART_Transmit(&huart2, (uint8_t*) COOR, sizeof(COOR), 1000);
				for (uint16_t X = 0; X < sizeof(COOR); X++) {
					COOR[X] = '\0';
				}
//				lat = .0;
//				lng = .0;
//				RF = 0;
			}
			lat = .0;
			lng = .0;
			RF = 0;
		}
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 8999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M0_Pin|M1_Pin|APOGEE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin APOGEE_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|APOGEE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MAIN_Pin */
  GPIO_InitStruct.Pin = MAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LORA_CONFG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	HAL_Delay(50);

	char cfg_buff[6] = {0};
	enum lora{Transparent, Fixed} mode;
	mode = MODE;

	cfg_buff[0] = 0xC0;  // header for saving paramater when power down C0
	cfg_buff[1] = ADDH;  // high address
	cfg_buff[2] = ADDL;  // low address
	cfg_buff[3] = 0x19;  // SPED (parity, baud, data rate)  1.2 kbps air data rate
	cfg_buff[4] = CHN;   // channel

	switch(mode){
	case Transparent:
		cfg_buff[5] = 0x44;  // option
		break;
	case Fixed:
		cfg_buff[5] = 0xC4;  // option
		break;
	default:
		cfg_buff[5] = 0x44;  // option
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) cfg_buff, 6, 1000);

	HAL_Delay(25);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET);
	HAL_Delay(25);
}

void LORA_READ_PARAMETER()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	HAL_Delay(50);

	char buff_read[6] = {0};
	buff_read[0] = 0xC1;
	buff_read[1] = 0xC1;
	buff_read[2] = 0xC1;

	HAL_UART_Transmit(&huart2, (uint8_t*) buff_read, 3, 1000);
	HAL_UART_Receive(&huart2, (uint8_t*) buff_read, 6, 1000);

	HAL_Delay(25);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET);
	HAL_Delay(25);
}

float calcoor(float x)
{
	float a = (int)x / 100;
	float b = (x - (a * 100.0)) / 60.0;
	return a+b;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
