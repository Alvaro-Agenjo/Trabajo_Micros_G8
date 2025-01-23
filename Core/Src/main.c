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

typedef struct {
	int16_t min, max;
}limits;

//////////////////////////
//		Sensores		//
//////////////////////////
typedef enum {G2 = 0b00, G4 = 0b01, G8 = 0b10, G16 = 0b11 } range_accel;
typedef enum {G1_3 = 0b001, G1_9 = 010, G2_5 = 0b011, G4_0 = 0b100, G4_7 = 0b101, G5_6 = 0b110 ,G8_1 = 0b111} range_comp;
typedef enum {Hz1 = 0b0001, Hz10 = 0b0010, Hz25 = 0b0011, Hz50 = 0b0100, Hz100 = 0b0101, Hz200 = 0b0110, Hz400 = 0b0111, Lp_def = 0b1000, Nr_def = 0b1001} ODR_accel;
typedef enum {Hz0_75 = 0b000, Hz1_5 = 0b001, Hz3_0 = 0b010, Hz7_5 = 0b011, Hz15 = 0b100, Hz30 = 0b101, Hz75 = 0b110, Hz220 = 0b111} ODR_comp;

//Tolerancia en comparación, Diff
typedef enum {XYZ_OUT_TOL = 0, X_IN_TOL = 1, Y_IN_TOL = 2, Z_IN_TOL = 3, XY_IN_TOL = 4, XZ_IN_TOL = 5, YZ_IN_TOL = 6, XYZ_IN_TOL = 7} diferencia;

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

#define Ctrl_comp_dir1    	0x00 // Registro de control CRA_REG_M
#define Ctrl_comp_dir2    	0x02 // Registro de modo MR_REG_M
#define Data_comp_dir		0x03 // Registro de salida X (bajo)


#define MAX_LSB 32768 //2^15

//Potenciómetros
#define Res_CAD 4095.0 //Resolución del CAD = 12 bits = 4096-1 valores
#define VREF 3.3 //Voltaje de referencia = 3.3V
#define Range_Deg 180.0 //Rango de grados [0-180]
#define Num_Pot 3

//Diff+Luces
#define tolerance 5 //tolerancia en grados +-º

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
Deg orientacion;
estado mode = Potenciometro;
const Deg NULO = {90,90,90};
//////////////////////////
//		Sensores		//
//////////////////////////
const int16_t factor_accel[4] = {2,4,8,16};
const uint16_t factor_compXY[7] = {1100,855,670,450,400,330,230};
const uint16_t factor_compZ[7] = {980,760,600,400,355,295,205};

uint8_t f_accel, f_comp;
Data offset_accel;
Data offset_comp;
Deg Deg_MEMS;

HAL_StatusTypeDef status_; //auxiliar

float diff_x; //valores comparados en función Diff
float diff_y;
float diff_z;
diferencia comp_tol;
uint8_t aux_luces = 0;


//////////////////////////////
//		Potenciómetros		//
//////////////////////////////
Deg Deg_pot;
uint32_t Pot_in[Num_Pot];
uint8_t flag_pot = 0;

//////////////////////
//		Servos		//
//////////////////////

/*añadir aquí*/


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
void LucesMEMS(diferencia d);
void LucesPOT(void);
void LucesSlowMove(void);

diferencia Diff(Deg* act, Deg* obj);
//////////////////////////
//		Sensores		//
//////////////////////////
void Init_Accel_Comp(uint8_t low_power_mode, ODR_accel Data_rate_accel, range_accel sensitivity_accel, ODR_comp data_rate_comp, range_comp sensitivity_comp);

Deg GetAngle(Data offset_accel, Data offset_comp);

void CalibrateAccel(Data* accel);
void GetAccel(Data* accel, Data offset);
void GetAccelRaw(uint8_t* raw_accel);
void ProcessAccel(uint8_t* raw_accel, Data * accel);
void Accel2Deg(Deg * deg, Data accel);

void CalibrateComp(Data * offset);
void GetComp(Data* comp, Data offset);
void GetCompRaw(uint8_t* raw_compass);
void ProcessComp(uint8_t* raw_comp, Data * comp);
void Comp2Deg(Deg* result, Data comp);

void Lectura (uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);
void Escritura (uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size);



//////////////////////////////
//		Potenciómetros		//
//////////////////////////////
void CalculoDegPot(void);

//////////////////////
//		Servos		//
//////////////////////
void SlowMove(Deg* orient, Deg ref);
void SetAngle(Deg grados);
Data Angle2PWM(Deg grados);
void SetPWM(Data ms);
int16_t map(limits input, limits output, float signal);

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
	CalibrateComp(&offset_comp);

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_ADC_Start_DMA(&hadc1, Pot_in, Num_Pot);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_Delay(500);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(flag_bt){ //Cambio de pulsado de botón Potenciómetro-MEMS viceversa
			if(mode == MEMS){
				SlowMove(&orientacion, NULO); //llevar a origen 90.90.0, 50 orientacion = 0.1 *0 +0.9 *orientacion
				Luces();
			}
			else{
				//flag_bt = 0;
				SlowMove(&orientacion, Deg_pot);
			}
		}
		else
		{
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);	//evita cambiar de modo durante la obtencion de datos
			if(mode == Potenciometro){
				if(flag_pot == 1){
					CalculoDegPot();
				}
				orientacion = Deg_pot;
				Luces();
			}
			else if (mode == MEMS){
				Deg_MEMS = GetAngle(offset_accel, offset_comp);
				orientacion = Deg_MEMS;

				/*Comprobación objetivo cumplido*/
				comp_tol = Diff(&Deg_MEMS, &Deg_pot);
				if(comp_tol == XYZ_IN_TOL){ //Si he llegado a objetivo, cambio de modo
					Luces();
					mode = Potenciometro;
				}
				else { Luces(); }

			}
			HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		}
		/*Funcion que controla servos*/
		//añadir aquí
		SetAngle(orientacion);
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
	htim1.Init.Prescaler = 959;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1999;
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
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
	uint8_t i2c_RxBuf[2];

	//init accel
	i2c_TxBuf[0] = (Data_rate_accel<<4)|(low_power_mode<<3)|(0b111);
	Escritura(Accel_Dir, Ctrl_accel_dir1, i2c_TxBuf, 1);


	Lectura(Accel_Dir, Ctrl_accel_dir2, i2c_RxBuf, 1);
	i2c_RxBuf[0] &= (0xff & ((sensitivity_accel<<4)|0b1111)) ;	//0b11--_1111
	i2c_RxBuf[0] |= sensitivity_accel<<4;						//0b00--_0000
	i2c_TxBuf[0] = i2c_RxBuf[0];
	Escritura(Accel_Dir, Ctrl_accel_dir2, i2c_TxBuf, 1);



	//init compas
	i2c_TxBuf[0] = 0x00;
	Escritura(Comp_Dir, Ctrl_comp_dir2, i2c_TxBuf, 1);

	Lectura(Comp_Dir, Ctrl_comp_dir1, i2c_RxBuf, 2);
	i2c_RxBuf[0] &= (0xff & ((data_rate_comp<<2)|0b11)) ;	//0b111-_--11
	i2c_RxBuf[0] |= data_rate_comp<<2;						//0b000-_--00
	i2c_TxBuf[0] = i2c_RxBuf[0];

	i2c_RxBuf[1] &= ((sensitivity_comp<<5)|0b11111) ;	//0b---1_1111
	i2c_RxBuf[1] |= sensitivity_comp<<5;				//0b---0_0000
	i2c_TxBuf[1] = i2c_RxBuf[1];
	Escritura(Comp_Dir, Ctrl_comp_dir1, i2c_TxBuf, 2);

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
Deg GetAngle(Data offset_accel, Data offset_comp)
{
	Deg Deg;
	Data accel, comp;

	GetAccel(&accel, offset_accel);
	Accel2Deg(&Deg, accel);

	GetComp(&comp, offset_comp);
	Comp2Deg(&Deg, comp);

	Deg.X += 90;	Deg.Y += 90;	Deg.Z += 90;
	return Deg;
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
	Lectura(Accel_Dir, Data_accel_dir, raw_accel, 6);
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
	deg->X = (atan(accel.Y / sqrt(accel.X * accel.X + accel.Z * accel.Z)) * 180.0 / 3.1416);
	deg->Y = (atan(-accel.X / sqrt(accel.Y * accel.Y + accel.Z * accel.Z)) * 180.0 / 3.1416);

}

void CalibrateComp(Data * offset){
	Data temp;
	uint8_t raw_data[6];
	const uint8_t max = 20;

	offset->X = 0;	offset->Y = 0;	offset->Z = 0;

	for(int i = 0; i< max; i++){
		GetCompRaw(raw_data);
		ProcessComp(raw_data, &temp);
		offset->X += temp.X;	offset->Y += temp.Y;	offset->Z += temp.Z;
	}
	offset->X /= max;	offset->Y /= max;	offset->Z /= max;
	offset->X = 0 - offset->X;	offset->Y = 0 - offset->Y;	offset->Z = 650 - offset->Z;
	//revisar max intentisy de campo magnetico terrestre.
}
void GetComp(Data* comp, Data offset){
	uint8_t raw_data[6];
	GetCompRaw(raw_data);
	ProcessComp(raw_data, comp);

	comp->X += offset.X;	comp->Y += offset.Y;	comp->Z += offset.Z;
}
void GetCompRaw(uint8_t* raw_compass){
	Lectura(Comp_Dir, Data_comp_dir, raw_compass, 6);
}
void ProcessComp(uint8_t* raw_comp, Data * comp){
	Data raw;

	raw.X = (raw_comp[0]<<8)|raw_comp[1];
	raw.Y = (raw_comp[4]<<8)|raw_comp[5];
	raw.Z = (raw_comp[2]<<8)|raw_comp[3];

	comp->X = raw.X * 1000 / factor_compXY[f_comp];	//resultdo en mG
	comp->Y = raw.Y * 1000 / factor_compXY[f_comp];
	comp->Z = raw.Z * 1000 / factor_compZ[f_comp];
}

void Comp2Deg(Deg *result, Data comp){
	float Xm, Ym;
	Xm = comp.X * cos(result->Y) + comp.Z * sin(result->Y);
	Ym = comp.X * sin(result->X)* sin(result->Y) + comp.Y * cos(result->X) - comp.Z * sin(result->X)* cos(result->Y);

	result->Z = atan(Ym/Xm);
}


void Escritura(uint16_t address, uint8_t reg_dir, uint8_t * msg, uint16_t msg_size){
	uint16_t device = (address<<1);
	uint8_t trama[2];

	for(int i = 0; i< msg_size;i++)
	{
		trama[0] = reg_dir+i;
		trama[1] = *(msg+i);
		status_ = HAL_I2C_Master_Transmit(&hi2c1, device, trama, 2, HAL_MAX_DELAY);
	}
}
void Lectura(uint16_t address, uint8_t reg_dir, uint8_t *msg, uint16_t msg_size){
	uint16_t device = (address<<1)|1;
	uint8_t direccion = reg_dir;
	if (msg_size > 1) direccion |= 0x80;

	status_ = HAL_I2C_Master_Transmit(&hi2c1, device, &direccion, 1, HAL_MAX_DELAY);
	status_ = HAL_I2C_Master_Receive(&hi2c1, device, msg, msg_size, HAL_MAX_DELAY);
}


void SlowMove(Deg* orient, Deg ref){
	float factor = 0.9;
	orient->X = ref.X *(1-factor) + orient->X * factor;
	orient->Y = ref.Y *(1-factor) + orient->Y * factor;
	orient->Z = ref.Z *(1-factor) + orient->Z * factor;

	if(Diff(&ref, orient) == XYZ_IN_TOL){flag_bt = 0;}
}

diferencia Diff(Deg* act, Deg* obj){
	//Comparo valores
	diff_x = fabs(act->X - obj->X); //fabs = float abs
	diff_y = fabs(act->Y - obj->Y);
	diff_z = fabs(act->Z - obj->Z);

	if     ( (diff_x > tolerance) && (diff_y > tolerance) && (diff_z > tolerance) ) {return XYZ_OUT_TOL;}
	else if( (diff_x < tolerance) && (diff_y < tolerance) && (diff_z < tolerance) ) {return XYZ_IN_TOL;}
	else if( (diff_y < tolerance) && (diff_z < tolerance) ) {return YZ_IN_TOL;}
	else if( (diff_x < tolerance) && (diff_z < tolerance) ) {return XZ_IN_TOL;}
	else if( (diff_x < tolerance) && (diff_y < tolerance) ) {return XY_IN_TOL;}
	else if( (diff_z < tolerance) ) {return Z_IN_TOL;}
	else if( (diff_y < tolerance) ) {return Y_IN_TOL;}
	else if( (diff_x < tolerance) ) {return X_IN_TOL;}
	else {return XYZ_OUT_TOL;}

}

void Luces(void){

	if (mode == Potenciometro){
		LucesPOT();
	}
	else if (mode == MEMS && flag_bt == 0){
		LucesMEMS(comp_tol);
	}
	else if (mode == MEMS && flag_bt == 1){
		LucesSlowMove();
	}
}

void LucesSlowMove(void){
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	HAL_Delay(10);
}

void LucesPOT(void){
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	HAL_Delay(20);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	HAL_Delay(20);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	HAL_Delay(20);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	HAL_Delay(20);
}

void LucesMEMS(diferencia d){

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);

	switch(d){
	case XYZ_IN_TOL:
		for (int i=0; i<5; i++){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1); //X
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1); //Y
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1); //Z
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);

			HAL_Delay(200);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);

			HAL_Delay(200);
		}
		break;

	case XYZ_OUT_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0); //Z
		break;

	case YZ_IN_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1); //Z
		break;

	case XZ_IN_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1); //Z
		break;

	case XY_IN_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0); //Z
		break;

	case Z_IN_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1); //Z
		break;

	case Y_IN_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0); //Z
		break;

	case X_IN_TOL:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0); //Z
		break;

	default:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0); //X
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0); //Y
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0); //Z
		break;
	}
}


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

void SetAngle(Deg obj) {
	Data PWM = Angle2PWM(obj);
	SetPWM(PWM);
}
Data Angle2PWM(Deg angulo) {
	Data output;
	limits deg = { 0,180 }, ms = { 50,250 };
	output.X = map(deg, ms, angulo.X);
	output.Y = map(deg, ms, angulo.Y);
	output.Z = map(deg, ms, angulo.Z);
	return output;
}
int16_t map(limits input, limits output, float signal) {
	if (signal > input.max) signal = input.max;
	else if (signal < input.min) signal = input.min;

	int16_t out = ((float)(output.max - output.min) / (float)(input.max - input.min) * signal) + (float)output.min;
	return out;
}

void SetPWM(Data output) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, output.X);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, output.Y);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, output.Z);
	HAL_Delay(500);
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
