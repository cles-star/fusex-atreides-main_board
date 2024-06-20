/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "mpu9250.h"
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCEL_SENSITIVITY_SCALE_FACTOR 4096.0
#define GYRO_SENSITIVITY_SCALE_FACTOR 32.8
#define TEMP_SCALE_FACTOR 100.0 //chatgpt, to be verify
#define PRESSURE_SCALE_FACTOR 25600.0 // idem
#define DATA_BUFFER_SIZE 10000
#define ACQUISITION_DELAY 20 //ms
#define WRITE_SD_DELAY 2000 //ms
//FATFS fs;
//FIL fil;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_SD_card(const char *data);
void init_SD_card(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  BMP280 bmp;
  char init_buffer[100];
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  MPU9250_Init(hspi1);
  bmp280_init(&bmp, &hspi1);
  init_SD_card();


  sprintf(init_buffer, "########################## New flight ##########################\n");
  process_SD_card(init_buffer);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  char buffer[DATA_BUFFER_SIZE];
  while (1)
  {
	static uint32_t ackTick = 0;
	static uint32_t sdTick = 0;
	static uint32_t currentTick = 0;
	static int index = 0;

	currentTick = HAL_GetTick();

	if ((currentTick-ackTick) > ACQUISITION_DELAY) {

		int16_t accelData[3];
		int16_t gyroData[3];
		int32_t temperature;
		uint32_t pressure;

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
		// Lecture des données d'accélération et gryoscope du MP9250
		MPU9250_ReadAccel(accelData, &hspi1);
		MPU9250_ReadGyro(gyroData, &hspi1);
		//Lecture de la température
		bmp280_read_temperature_and_pressure(&bmp, &temperature, &pressure);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);

		int accel_size = sprintf(buffer+index, "Accel_X: %.2f, Accel_Y: %.2f, Accel_Z: %.2f\n", (double)accelData[0]/ACCEL_SENSITIVITY_SCALE_FACTOR, (double)accelData[1]/ACCEL_SENSITIVITY_SCALE_FACTOR, (double)accelData[2]/ACCEL_SENSITIVITY_SCALE_FACTOR);
		index += accel_size;
		int gyro_size = sprintf(buffer+index, "Gyro_X: %.2f, Gyro_Y: %.2f, Gyro_Z: %.2f\n", (double)gyroData[0]/GYRO_SENSITIVITY_SCALE_FACTOR, (double)gyroData[1]/GYRO_SENSITIVITY_SCALE_FACTOR, (double)gyroData[2]/GYRO_SENSITIVITY_SCALE_FACTOR);
		index+= gyro_size;
		int bmp_data_size = sprintf(buffer+index, "Temperature: %.2f C, Pressure: %.2f hPa\n", temperature / TEMP_SCALE_FACTOR, pressure / PRESSURE_SCALE_FACTOR);
		index+= bmp_data_size;
		ackTick = currentTick;

	}

	if((currentTick-sdTick) > WRITE_SD_DELAY){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
		process_SD_card(buffer);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
		index = 0;
		sdTick = currentTick;
		memset(buffer, 0, DATA_BUFFER_SIZE);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void init_SD_card(void){
	FATFS       FatFs;
	FRESULT     fres;
	FIL         fil;

	//Mount the SD Card
	fres = f_mount(&FatFs, "", 1);    //1=mount now
	if (fres != FR_OK)
	{
	  printf("No SD Card found : (%i)\r\n", fres);
	}
	printf("SD Card Mounted Successfully!!!\r\n");
	//Create file for record
	fres = f_open(&fil, "test.txt", FA_WRITE | FA_READ | FA_CREATE_NEW);
	if(fres != FR_OK)
	{
	  printf("File creation/open Error : (%i)\r\n", fres);
	}
	f_mount(NULL, "", 0);
	f_close(&fil);
}

void process_SD_card(const char *datas)
{
	FIL         fil;                  //File handle
	FRESULT     fres;                 //Result after operations
	FATFS       FatFs;

	//Mount the SD Card
	fres = f_mount(&FatFs, "", 1);    //1=mount now
	if (fres != FR_OK)
	{
	  printf("No SD Card found : (%i)\r\n", fres);
	}

	//Open the file
	fres = f_open(&fil, "test.txt", FA_OPEN_APPEND | FA_WRITE );
	if(fres != FR_OK)
	{
	  printf("File creation/open Error : (%i)\r\n", fres);
	}
	//printf("Writing data!!!\r\n");
	//write the data
	f_printf(&fil, datas);
	//close your file
	f_close(&fil);
	//We're done, so de-mount the drive
	f_mount(NULL, "", 0);
	//printf("SD Card Unmounted Successfully!!!\r\n");
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */