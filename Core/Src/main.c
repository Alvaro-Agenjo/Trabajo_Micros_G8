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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float X, Y, Z;
}Deg;

typedef struct{
	int16_t X, Y, Z;
}Data;

//////////////////////////
//		Sensores		//
//////////////////////////
typedef enum {G2 = 0b00, G4 = 0b01, G8 = 0b10, G16 = 0b11 } range_accel;
typedef enum {G1_3 = 0b001, G1_9 = 010, G2_5 = 0b011, G4_0 = 0b100, G4_7 = 0b101, G5_6 = 0b110 ,G8_1 = 0b111} range_comp;
typedef enum {Hz1 = 0b0001, Hz10 = 0b0010, Hz25 = 0b0011, Hz50 = 0b0100, Hz100 = 0b0101, Hz200 = 0b0110, Hz400 = 0b0111, Lp_def = 0b1000, Nr_def = 0b1001} ODR_accel;
typedef enum {Hz0_75 = 0b000, Hz1_5 = 0b001, Hz3_0 = 0b010, Hz7_5 = 0b011, Hz15 = 0b100, Hz30 = 0b101, Hz75 = 0b110, Hz220 = 0b111} ODR_comp;

//////////////////////
//		Botón		//
//////////////////////

typedef enum {Potenciometro = 0, MEMS} estado;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Accel_Dir 0b0011001
#define Comp_Dir 0b0011110

#define Ctrl_accel_dir1 0x20
#define Ctrl_accel_dir2 0x23
#define Data_accel_dir 0x28

#define Ctrl_comp_dir 0x00
#define Data_comp_dir 0x03

#define MAX_LSB 32768 //2^15

#define Res_CAD 4095.0 //Resolución del CAD = 12 bits = 4096-1 valores
#define VREF 3.3 //Voltaje de referencia = 3.3V
#define Range_Deg 180.0 //Rango de grados [0-180]
#define Num_Pot 3

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
Deg orientacion;
estado mode = Potenciometro;
//////////////////////////
//		Sensores		//
//////////////////////////
const int16_t factor_accel[4] = {2,4,8,16};
const uint16_t factor_compXY[7] = {1100,855,670,450,400,330,230};
const uint16_t factor_compZ[7] = {980,760,600,400,355,295,205};

uint8_t f_accel, f_comp;
Data Accel, offset_accel;
Data Comp, offset_comp;
Deg Deg_MEMS;

HAL_StatusTypeDef status_; //auxiliar


//////////////////////////////
//		Potenciómetros		//
//////////////////////////////
Deg Deg_pot;
uint32_t Pot_in[Num_Pot];
uint8_t flag_pot = 0;

//////////////////////
//		Servos		//
//////////////////////

////////////////////INICIO FUERA DEL MAIN//////////////////////////////
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim1; //puntero del temporizador
void MX_GPIO_Init(void){ // Inicialización de la GPIO del TIM1
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct={0};        // Inicializa con valores estándar
	GPIO_InitStruct.Pin = GPIO_PIN_8;            // Se coloca en el pin8, PA8
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // Alternate function push pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pullup o pulldown
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Baja freq es suficiente para el servo
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;   // Función alternativa 1 del TIM1
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void MX_TIM1_Init(void){ // Inicializacion del temporizador
	__HAL_RCC_TIM1_CLK_ENABLE();
	htim1.Instance = TIM1;                         // Usamos el temporizador TIM1
	htim1.Init.Prescaler = 96-1;                   // Prescaler, reduce de 96MHz a 1MHz
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;   // Modo de conteo ascendente
	htim1.Init.Period = 19999;                     // Periodo, ARR
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;              // No se usa en PWM

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
	        // Manejo de error
	        Error_Handler();
	}
}
void MX_TIM1_PWM_Config(void){
	TIM_OC_InitTypeDef sConfigOC={0};                // Inicializa con valores estándar, rellena todos los campos con 0

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;                          // Pulso estandar 90º, punto medio
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)!= HAL_OK){
		// Manejo de error
		Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}
void Set_Servo_Angle(uint8_t angle){
	uint16_t pulse;
	bool of;
	if (angle>180){
		angle=180;  // Limita el ángulo máximo a 180º
	}
	pulse = 1000 + (angle*1000)/180;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}
////////////////////FIN FUERA DEL MAIN//////////////////////////////
/*
El ciclo de trabajo define la posición del servomotor.
Para un PWM con un periodo de 20 ms:
1 ms (ángulo mínimo, 0°): Esto equivale a un ciclo de trabajo del 5%.
1.5 ms (ángulo neutro, 90°): Esto equivale a un ciclo de trabajo del 7.5%.
2 ms (ángulo máximo, 180°): Esto equivale a un ciclo de trabajo del 10%
*/
/////////////////////INICIO DENTRO DEL MAIN////////////////////////////
HAL_Init();
SystemClock_Config();

MX_GPIO_Init();
MX_TIM1_Init();
MX_TIM1_PWM_Config();
while(1){
	Set_Servo_Angle(0); // Se da la entrada en forma de ángulo y la propia función provoca la comunicación con el pwm
	HAL_Delay(1000);

	Set_Servo_Angle(60);
	HAL_Delay(1000);

	Set_Servo_Angle(120);
	HAL_Delay(1000);

	Set_Servo_Angle(180);
	HAL_Delay(1000);

	Set_Servo_Angle(0);
	HAL_Delay(1000);
}
/////////////////////FIN DENTRO DEL MAIN////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void Luces(void);
uint8_t Diff(Deg* act, Deg* obj);
//////////////////////////
//		Sensores		//
//////////////////////////
void Init_Accel_Comp(uint8_t low_power_mode, ODR_accel Data_rate_accel, range_accel sensitivity_accel, ODR_comp data_rate_comp, range_comp sensitivity_comp);

void CalibrateAccel(Data* accel);
void GetAccel(Data* accel, Data offset);
void GetAccelRaw(uint8_t* raw_accel);
void ProcessAccel(uint8_t* raw_accel, Data * accel);
void Accel2Deg(Deg * deg, Data accel);

void GetComp(int16_t* X_comp, int16_t* Y_comp, int16_t* Z_comp);
void GetCompRaw(uint8_t* raw_comp);
void ProcessComp(uint8_t* raw_comp, int16_t* X_comp, int16_t* Y_comp, int16_t* Z_comp);

void Lectura_accel(uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);
void Escritura_accel(uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);

void Lectura_compass(uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);
void Escritura_compass(uint16_t address, uint8_t reg_dir, uint8_t * msg, uint16_t msg_size);

void SlowMove(Deg* orient);

//////////////////////////////
//		Potenciómetros		//
//////////////////////////////
void CalculoDegPot(void);

//////////////////////
//		Servos		//
//////////////////////

/*añadir aquí*/



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t flag_bt = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		flag_bt = 1;
		mode = !mode;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc->Instance == ADC1){
		flag_pot = 1;
	}
}

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  //inicializaiones necesarias

  Init_Accel_Comp(0, Hz50, G2, Hz15, G1_9);
  CalibrateAccel(&offset_accel);

  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_ADC_Start_DMA(&hadc1, Pot_in, Num_Pot);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag_bt == 1 && mode == MEMS){
		  SlowMove(&orientacion); //llevar a origen 0, 50 orientacion = 0.1 *0 +0.9 *orientacio
	  }
	  else
	  {
		  HAL_NVIC_DisableIRQ(EXTI0_IRQn);	//evita cambiar de modo durante la obtencion de datos
		  if(mode == Potenciometro){
			  if(flag_pot == 1){
				  CalculoDegPot();
			  }
			  orientacion = Deg_pot;
		  }
		  else if (mode == MEMS){
			  GetAccel(&Accel, offset_accel);
			  Accel2Deg(&Deg_MEMS, Accel);

			  orientacion = Deg_MEMS;
		  }
		  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	  }
	  /*Funcion que controla servos*/
	  //añadir aquí


	  /*Comprobación objetivo cumplido*/
//	  if(mode == MEMS && Diff(Deg_MEMS, Deg_pot)<=10){
//		  Luces();
//	  }

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//////////////////////////
//		Sensores		//
//////////////////////////
void Init_Accel_Comp(uint8_t low_power_mode, ODR_accel Data_rate_accel, range_accel sensitivity_accel, ODR_comp data_rate_comp, range_comp sensitivity_comp){
	uint8_t i2c_TxBuf[2];
	uint8_t i2c_RxBuf[3];

	//init accel
	i2c_TxBuf[0] = (Data_rate_accel<<4)|(low_power_mode<<3)|(0b111);
	Escritura_accel(Accel_Dir, Ctrl_accel_dir1, i2c_TxBuf, 1);


	Lectura_accel(Accel_Dir, Ctrl_accel_dir2, i2c_RxBuf, 1);
	i2c_RxBuf[0] &= (0xff & ((sensitivity_accel<<4)|0b1111)) ;	//0b11--_1111
	i2c_RxBuf[0] |= sensitivity_accel<<4;						//0b00--_0000
	i2c_TxBuf[0] = i2c_RxBuf[0];
	Escritura_accel(Accel_Dir, Ctrl_accel_dir2, i2c_TxBuf, 1);


/*
	//init compas
	Lectura_accel(&hi2c1, Comp_Dir, Ctrl_comp_dir, i2c_RxBuf, 3);
	i2c_RxBuf[0] &= (0xff & ((data_rate_comp<<2)|0b11)) ;	//0b111-_--11
	i2c_RxBuf[0] |= data_rate_comp<<2;						//0b000-_--00
	i2c_TxBuf[0] = i2c_RxBuf[0];

	i2c_RxBuf[1] &= ((sensitivity_comp<<5)|0b11111) ;	//0b---1_1111
	i2c_RxBuf[1] |= sensitivity_comp<<5;				//0b---0_0000
	i2c_TxBuf[1] = i2c_RxBuf[1];

	i2c_RxBuf[1] &= (0xff &(0xfc)) ;	//0b1111_11--
	i2c_TxBuf[2] = i2c_RxBuf[2];
	Escritura_accel(&hi2c1, Comp_Dir, Ctrl_comp_dir, i2c_TxBuf, 3);
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
void CalibrateAccel(Data * offset){
	Data temp;
	uint8_t raw_data[6];
	const uint8_t max = 20;

	offset->X = 0;	offset->Y = 0;	offset->Z = 0;

	for(int i = 0; i< max; i++){
		GetAccelRaw(raw_data);
		ProcessAccel(raw_data, &temp);
		offset->X += temp.X;	offset->Y += temp.Y;	offset->Z += temp.Z;
	}
	offset->X /= max;	offset->Y /= max;	offset->Z /= max;
	offset->X = 0- offset->X;	offset->Y = 0- offset->Y;	offset->Z = 1000- offset->Z;
}
void GetAccel(Data* accel, Data offset){
	uint8_t raw_data[6];
	GetAccelRaw(raw_data);
	ProcessAccel(raw_data, accel);

	accel->X += offset.X;	accel->Y += offset.Y;	accel->Z += offset.Z;
}
void GetAccelRaw(uint8_t* raw_accel){
	Lectura_accel(Accel_Dir, Data_accel_dir, raw_accel, 6);
}
void ProcessAccel(uint8_t* raw_accel, Data * accel){
	Data raw;

	raw.X = (raw_accel[1]<<8)|raw_accel[0];
	raw.Y = (raw_accel[3]<<8)|raw_accel[2];
	raw.Z = (raw_accel[5]<<8)|raw_accel[4];

	accel->X = raw.X * 1000 * factor_accel[f_accel] / MAX_LSB;	//resultdo en mG
	accel->Y = raw.Y * 1000 * factor_accel[f_accel] / MAX_LSB;
	accel->Z = raw.Z * 1000 * factor_accel[f_accel] / MAX_LSB;
}

void Accel2Deg(Deg * deg, Data accel){
	deg->X = atan(accel.Y / sqrt(accel.X * accel.X + accel.Z * accel.Z)) * 180.0 / 3.1416;
	deg->Y = atan(-accel.X / sqrt(accel.Y * accel.Y + accel.Z * accel.Z)) * 180.0 / 3.1416;

}

void GetComp(int16_t* X_compass, int16_t* Y_compass, int16_t* Z_compass){
	uint8_t raw_data[6];
	GetCompRaw(raw_data);
	ProcessComp(raw_data, X_compass, Y_compass, Z_compass);
}

void GetCompRaw(uint8_t* raw_compass){
	Lectura_accel(Comp_Dir, Data_comp_dir, raw_compass, 6);
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


void Escritura_accel(uint16_t address, uint8_t reg_dir, uint8_t * msg, uint16_t msg_size){
	uint16_t device = (address<<1);
	uint8_t trama[2];

	for(int i = 0; i< msg_size;i++)
	{
		trama[0] = reg_dir+i;
		trama[1] = *(msg+i);
		status_ = HAL_I2C_Master_Transmit(&hi2c1, device, trama, 2, HAL_MAX_DELAY);
	}
}
void Lectura_accel(uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size){
	uint16_t device = (address<<1)|1;
	uint8_t direccion = reg_dir;
	if (msg_size > 1) direccion |= 0x80;

	status_ = HAL_I2C_Master_Transmit(&hi2c1, device, &direccion, 1, HAL_MAX_DELAY);
	status_ = HAL_I2C_Master_Receive(&hi2c1, device, msg, msg_size, HAL_MAX_DELAY);
}

void Escritura_compass(uint16_t address, uint8_t reg_dir, uint8_t * msg, uint16_t msg_size){
	uint16_t device = (address<<1);//0b001_1110 = 3C
	uint8_t trama[msg_size +1];

	trama[0] = reg_dir;
	for(int i = 0; i< msg_size;i++){
		trama[i+1] = msg[i];
	}
	status_ = HAL_I2C_Master_Transmit(&hi2c1, device, trama, msg_size+1, HAL_MAX_DELAY);

}
void Lectura_compass(uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size){
	uint16_t device = (address<<1)|1;	//0b001_1110 | 1 = 0b0011_1101 = 3D
	uint8_t direccion = reg_dir;
	if (msg_size > 1) direccion |= 0x80;

	status_ = HAL_I2C_Master_Transmit(&hi2c1, device, &direccion, 1, HAL_MAX_DELAY);
	status_ = HAL_I2C_Master_Receive(&hi2c1, device, msg, msg_size, HAL_MAX_DELAY);
}
void SlowMove(Deg* orient){
	Deg nulo = {0,0,0};
	float factor = 0.9;
	orient->X += nulo.X *(1-factor) + orient->X * factor;
	//ejes Y Z ...

	if(Diff(&nulo, orient))
		flag_bt = 0;
}

uint8_t Diff(Deg* act, Deg* obj){return 1;}

//////////////////////////////
//		Potenciómetros		//
//////////////////////////////
void CalculoDegPot(){

	//Conversión de señal CAD a grados del servo. Res_CAD = 12 bits = 4096-1 ; Range_Deg = 180º
	Deg_pot.X = ( Pot_in[0] / Res_CAD ) * Range_Deg;
	Deg_pot.Y = ( Pot_in[1] / Res_CAD ) * Range_Deg;
	Deg_pot.Z = ( Pot_in[2] / Res_CAD ) * Range_Deg;

	//Reinicio bandera
	flag_pot = 0;
}


//////////////////////
//		Servos		//
//////////////////////


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
