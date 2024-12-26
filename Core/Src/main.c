/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {G2 = 0b00, G4 = 0b01, G8 = 0b10, G16 = 0b11 } range_accel;
typedef enum {G1_3 = 0b001, G1_9 = 010, G2_5 = 0b011, G4_0 = 0b100, G4_7 = 0b101, G5_6 = 0b110 ,G8_1 = 0b111} range_comp;
typedef enum {Hz1 = 0b0001, Hz10 = 0b0010, Hz25 = 0b0011, Hz50 = 0b0100, Hz100 = 0b0101, Hz200 = 0b0110, Hz400 = 0b0111, Lp_def = 0b1000, Nr_def = 0b1001} ODR_accel;
typedef enum {Hz0_75 = 0b000, Hz1_5 = 0b001, Hz3_0 = 0b010, Hz7_5 = 0b011, Hz15 = 0b100, Hz30 = 0b101, Hz75 = 0b110, Hz220 = 0b111} ODR_comp;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Accel_Dir 0b0011001
#define Comp_Dir 0b0011110

#define Ctrl_accel_dir1 0x20
#define Ctrl_accel_dir2 0x23
#define Data_accel_dir 0x28

#define Ctrl_comp_dir 0x00
#define Data_comp_dir 0x03

#define MAX_LSB 32768 //2^15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
const int16_t factor_accel[4] = {2,4,8,16};
const uint16_t factor_compXY[7] = {1100,855,670,450,400,330,230};
const uint16_t factor_compZ[7] = {980,760,600,400,355,295,205};

uint8_t f_accel, f_comp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Init_Accel_Comp(uint8_t low_power_mode, ODR_accel Data_rate_accel, range_accel sensitivity_accel, ODR_comp data_rate_comp, range_comp sensitivity_comp);


void GetAccel(int16_t* X_accel, int16_t* Y_accel, int16_t* Z_accel);
void GetAccelRaw(uint8_t* raw_accel);
void ProcessAccel(uint8_t* raw_accel, int16_t* X_accel, int16_t* Y_accel, int16_t* Z_accel);

void GetComp(int16_t* X_comp, int16_t* Y_comp, int16_t* Z_comp);
void GetCompRaw(uint8_t* raw_comp);
void ProcessComp(uint8_t* raw_comp, int16_t* X_comp, int16_t* Y_comp, int16_t* Z_comp);

void Lectura_accel(I2C_HandleTypeDef *hi2c, uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);
void Escritura_accel(I2C_HandleTypeDef *hi2c, uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t X_ac, Y_ac, Z_ac;
int16_t X_comp, Y_comp, Z_comp;

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

  Init_Accel_Comp(0, Hz25, G2, Hz30, G4_7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  GetAccel(&X_ac, &Y_ac, &Z_ac);
	  //GetComp(&X_comp, &Y_comp, &Z_comp);
	  HAL_Delay(200);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
void Init_Accel_Comp(uint8_t low_power_mode, ODR_accel Data_rate_accel, range_accel sensitivity_accel, ODR_comp data_rate_comp, range_comp sensitivity_comp){
	uint8_t i2c_TxBuf[2];
	uint8_t i2c_RxBuf[2];
	//init accel
	i2c_TxBuf[0] = (Data_rate_accel<<4)|(low_power_mode<<3)|(0b111);
	Escritura_accel(&hi2c1, Accel_Dir, Ctrl_accel_dir1, i2c_TxBuf, 1);


	Lectura_accel(&hi2c1, Accel_Dir, Ctrl_accel_dir2, i2c_RxBuf, 1);
	i2c_RxBuf[0] &= (0xff & ((sensitivity_accel<<4)|0b1111)) ;	//0b11--_1111
	i2c_RxBuf[0] |= sensitivity_accel<<4;						//0b00--_0000
	i2c_TxBuf[0] = i2c_RxBuf[0];
	Escritura_accel(&hi2c1, Accel_Dir, Ctrl_accel_dir2, i2c_TxBuf, 1);

/*
 	//init compas
	Lectura_compass(&hi2c1, Comp_Dir, Ctrl_comp_dir, i2c_RxBuf, 2);
	i2c_RxBuf[0] &= (0xff & ((data_rate_comp<<2)|0b11)) ;	//0b111-_--11
	i2c_RxBuf[0] |= data_rate_comp<<2;						//0b000-_--00
	i2c_TxBuf[0] = i2c_RxBuf[0];

	i2c_RxBuf[1] &= ((sensitivity_comp<<5)|0b11111) ;	//0b---1_1111
	i2c_RxBuf[1] |= sensitivity_comp<<5;				//0b---0_0000
	i2c_TxBuf[1] = i2c_RxBuf[1];
	Escritura_compass(&hi2c1, Comp_Dir, Ctrl_comp_dir, i2c_TxBuf, 2);

*/
	switch(sensitivity_accel){
	case G2:
	{
		f_accel = 0;
		break;
	}
	case G4:
	{
		f_accel = 1;
		break;
	}
	case G8:
	{
		f_accel = 2;
		break;
	}
	case G16:
	{
		f_accel = 3;
		break;
	}
	}
	switch(sensitivity_comp){
	case G1_3:
	{
		f_comp = 0;
		break;
	}
	case G1_9:
	{
		f_comp = 1;
		break;
	}
	case G2_5:
	{
		f_comp = 2;
		break;
	}
	case G4_0:
	{
		f_comp = 3;
		break;
	}
	case G4_7:
	{
		f_comp = 4;
		break;
	}
	case G5_6:
	{
		f_comp = 5;
		break;
	}
	case G8_1:
	{
		f_comp = 6;
		break;
	}
	}
}



void GetAccel(int16_t* X_accel, int16_t* Y_accel, int16_t* Z_accel){
	uint8_t raw_data[6];
	GetAccelRaw(raw_data);
	ProcessAccel(raw_data, X_accel, Y_accel, Z_accel);
}

void GetAccelRaw(uint8_t* raw_accel){
	Lectura_accel(&hi2c1, Accel_Dir, Data_accel_dir, raw_accel, 6);
}
void ProcessAccel(uint8_t* raw_accel, int16_t* X_accel, int16_t* Y_accel, int16_t* Z_accel){
	int16_t X_raw=0, Y_raw=0, Z_raw=0;

	X_raw = (raw_accel[1]<<8)|raw_accel[0];
	Y_raw = (raw_accel[3]<<8)|raw_accel[2];
	Z_raw = (raw_accel[5]<<8)|raw_accel[4];

	*(X_accel) = X_raw * 1000 * factor_accel[f_accel] / MAX_LSB;	//resultdo en mG
	*(Y_accel) = Y_raw * 1000 * factor_accel[f_accel] / MAX_LSB;
	*(Z_accel) = Z_raw * 1000 * factor_accel[f_accel] / MAX_LSB;
}

void GetComp(int16_t* X_compass, int16_t* Y_compass, int16_t* Z_compass){
	uint8_t raw_data[6];
	GetCompRaw(raw_data);
	ProcessComp(raw_data, X_compass, Y_compass, Z_compass);
}

void GetCompRaw(uint8_t* raw_compass){
	Lectura_accel(&hi2c1, Comp_Dir, Data_comp_dir, raw_compass, 6);
}
void ProcessComp(uint8_t* raw_compass, int16_t* X_compass, int16_t* Y_compass, int16_t* Z_compass){
	int16_t X_raw, Y_raw, Z_raw;

	X_raw = (raw_compass[0]<<8)|raw_compass[1];
	Y_raw = (raw_compass[4]<<8)|raw_compass[5];
	Z_raw = (raw_compass[2]<<8)|raw_compass[3];

	*(X_compass) = X_raw / factor_compXY[f_comp];	//resultdo en Gauss
	*(Y_compass) = Y_raw / factor_compXY[f_comp];
	*(Z_compass) = Z_raw / factor_compZ[f_comp];
}


void Escritura_accel(I2C_HandleTypeDef *hi2c, uint16_t address, uint8_t reg_dir, uint8_t * msg, uint16_t msg_size){
	uint16_t device = (address<<1);
	uint8_t trama[2];

	for(int i = 0; i< msg_size;i++)
	{
		trama[0] = reg_dir+i;
		trama[1] = *(msg+i);
		HAL_I2C_Master_Transmit(&hi2c1, device, trama, 2, HAL_MAX_DELAY);
	}
}
void Lectura_accel(I2C_HandleTypeDef *hi2c, uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size){
	uint16_t device = (address<<1)|1;
	uint8_t direccion = reg_dir;
	if (msg_size > 1) direccion |= 0x80;

	HAL_I2C_Master_Transmit(&hi2c1, device, &direccion, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, device, msg, msg_size, HAL_MAX_DELAY);
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
