/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LSM303_CRA_REG_M    0x00 // Registro de control CRA_REG_M
#define LSM303_CRB_REG_M    0x01 // Registro de control CRB_REG_M
#define LSM303_MR_REG_M     0x02 // Registro de modo MR_REG_M
#define LSM303_OUT_X_L_M    0x03 // Registro de salida X (bajo)
#define LSM303_OUT_X_H_M    0x04 // Registro de salida X (alto)
#define LSM303_OUT_Y_L_M    0x05 // Registro de salida Y (bajo)
#define LSM303_OUT_Y_H_M    0x06 // Registro de salida Y (alto)
#define LSM303_OUT_Z_L_M    0x07 // Registro de salida Z (bajo)
#define LSM303_OUT_Z_H_M    0x08 // Registro de salida Z (alto)
uint8_t slave_address_read_m=0x3D;
uint8_t slave_address_write_m=0x3C;
int8_t mag_h_x, mag_h_y, mag_h_z;

uint8_t i2cTxBuf[2];
uint8_t i2cRxBuf;
uint8_t i2cRxBuf2[6];
uint8_t slave_address_read=0x33;
uint8_t slave_address_write=0x32;
int8_t accel_x, accel_y, accel_z;
int16_t a_x, a_y, a_z;
int16_t g_x, g_y, g_z;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  i2cTxBuf[0]=0x20;
  i2cTxBuf[1]=0x47;
  HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY);

  i2cRxBuf=0;
  HAL_I2C_Master_Transmit(&hi2c1, slave_address_read, i2cTxBuf, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, slave_address_read, &i2cRxBuf, 1, HAL_MAX_DELAY);

  HAL_Delay(100);

  //Magnetómetro
  i2cTxBuf[0]=LSM303_MR_REG_M;
  i2cTxBuf[1]=0x00; //continuous conversion mode
  HAL_I2C_Master_Transmit(&hi2c1, slave_address_write_m, i2cTxBuf, 2, HAL_MAX_DELAY);

  //Comprobación de CRA_REG_M
  i2cTxBuf[0]=LSM303_CRA_REG_M;
  i2cRxBuf=0;
  HAL_I2C_Master_Transmit(&hi2c1, slave_address_read_m, i2cTxBuf, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, slave_address_read_m, &i2cRxBuf, 1, HAL_MAX_DELAY);

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	/*Lectura uno a uno cada eje, X, Y, Z*/
  	/*i2cTxBuf[0]=0x29;
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read, &accel_x, 1, HAL_MAX_DELAY);
  	HAL_Delay(100);

  	i2cTxBuf[0]=0x2B;
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read, &accel_y, 1, HAL_MAX_DELAY);
  	HAL_Delay(100);

  	i2cTxBuf[0]=0x2D;
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read, &accel_z, 1, HAL_MAX_DELAY);
  	HAL_Delay(100);*/

  	/*Lectura de los 3 ejes a la vez (6 registros)*/
  	i2cTxBuf[0]=0x28|0x80; // El 0x80 es para leer varios bytes a la vez.
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read, &i2cRxBuf2[0], 6, HAL_MAX_DELAY);
  	HAL_Delay(100);
  	a_x= (i2cRxBuf2[1]<<8 | i2cRxBuf2[0]);
  	a_y= (i2cRxBuf2[3]<<8 | i2cRxBuf2[2]);
  	a_z= (i2cRxBuf2[5]<<8 | i2cRxBuf2[4]);

  	//Magnetometro
  	/*i2cTxBuf[0]=LSM303_OUT_X_H_M;
    HAL_I2C_Master_Transmit(&hi2c1, slave_address_read_m, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read_m, &mag_h_x, 1, HAL_MAX_DELAY);
  	HAL_Delay(100);

  	i2cTxBuf[0]=LSM303_OUT_Y_H_M;
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read_m, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read_m, &mag_h_y, 1, HAL_MAX_DELAY);
  	HAL_Delay(100);

  	i2cTxBuf[0]=LSM303_OUT_Z_H_M;
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read_m, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read_m, &mag_h_z, 1, HAL_MAX_DELAY);
  	HAL_Delay(100);*/

  	i2cTxBuf[0]=0x03; //Magnetometro
  	HAL_I2C_Master_Transmit(&hi2c1, slave_address_read_m, i2cTxBuf, 1, HAL_MAX_DELAY);
  	HAL_I2C_Master_Receive(&hi2c1, slave_address_read_m, &i2cRxBuf2[0], 6, HAL_MAX_DELAY);
  	HAL_Delay(100);
  	g_x= (i2cRxBuf2[1]<<8 | i2cRxBuf2[0]);
  	g_y= (i2cRxBuf2[3]<<8 | i2cRxBuf2[2]);
  	g_z= (i2cRxBuf2[5]<<8 | i2cRxBuf2[4]);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
