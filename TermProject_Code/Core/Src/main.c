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

#include <stdio.h>
#include "math.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "pid.h"
#include "motor_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

PID_TypeDef VEL_PID;
PID_TypeDef ACCEL_PID;
PID_TypeDef PITCH_PID;

typedef struct{
	int16_t velocity;
	int64_t position;
	uint32_t last_counter_value;
}encoder_instance;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int dc;
int dc2;
int test;
#define TIMCLOCK   96000000
#define TIMCLOCK2   96000000
#define PRESCALAR  0
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;
int ifc = 0;
uint32_t IC_Val12 = 0;
uint32_t IC_Val22 = 0;
uint32_t Difference2 = 0;
uint32_t usWidth = 0;
uint32_t usWidth2 = 0;

int n;
char buffer[50];

int BNO_SampleRate;

int Heading;
int Roll;

double Pitch, PITCH_PIDOut;


char AngleBuf[50];

int Vel_x;
int Vel_y;
int Vel_z;

int Vel_x_Cap;
int Vel_y_Cap;
int Vel_z_Cap;

int Vel_Capture = 0;

double Vel_total, VEL_PIDOut, Vel_Setpoint = 0;

int Vel_Setpoint_Factor;

char VelBuf[50];

double Accel_Total, ACC_PIDOut;

char AngleBuf[50];

motor_t mot1 = {.duty    = 0,
				  .channel = 1,
				  .timer = TIM1};
motor_t mot2 = {.duty    = 0,
				  .channel = 2,
    			  .timer = TIM1};

encoder_instance enc_instance_mot = {.position = 0,
											.velocity = 0};

int encoder_position;
int encoder_velocity;
int timer_counter;

int duty;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize BNO

  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();

  // Initialize PID's ( NEED TO LOOK INTO SAMPLETIME AND STUFF FOR OUR MCU I THINK?)

  PID_SetMode(&VEL_PID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&VEL_PID, 500);
  PID_SetOutputLimits(&VEL_PID, 1, 100);

  PID(&VEL_PID, &Vel_total, &VEL_PIDOut, &Vel_Setpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);

  PID_SetMode(&ACCEL_PID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&ACCEL_PID, 500);
  PID_SetOutputLimits(&ACCEL_PID, 1, 100);

  PID(&ACCEL_PID, &Accel_Total, &ACC_PIDOut, &VEL_PIDOut, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);

  PID_SetMode(&PITCH_PID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&PITCH_PID, 500);
  PID_SetOutputLimits(&PITCH_PID, 1, 100);

  PID(&PITCH_PID, &Pitch, &PITCH_PIDOut, &ACC_PIDOut, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Enable Encoder mode on TIM3
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // Enable Encoder mode on TIM1

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);  // Enable Interrupts for E-Stop
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);    // Enable PWM for motor driver input
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //n = sprintf(buffer, "Chan 1 Width: %d \r\n", usWidth);
	  //HAL_UART_Transmit(&huart1,buffer,n,400);
	  //n = sprintf(buffer, "Chan 2 Width: %d \r\n", usWidth2);
	  //HAL_UART_Transmit(&huart1,buffer,n,400);
	  //HAL_Delay(250);

	  if (usWidth > 1750){
		  // Turn Motors Off after E-Stop Hit
	  }

	  // Code to set Velocity Setpoint to RC input

	  // Not sure if a negative vel setpoint will work or what magnitude
	  // will work for the controller but we can tune the factor with the controllers

	  Vel_Setpoint = ((usWidth2 - 1500) / Vel_Setpoint_Factor);


	  // Get BNO Angles for PID
	  // (Really only need the pitch; Also depends on where we secure it)

	  bno055_vector_t a = bno055_getVectorEuler();
	  Heading = a.x;
	  Roll = a.y;
	  Pitch = a.z;


	  // Get BNO055 Linear Accel Data to Calculate Vel for PID
	  // And use Accel for other PID

	  bno055_vector_t v = bno055_getVectorLinearAccel();
	  if ( (Vel_Capture = 1) ){
		  Vel_x = (Vel_x_Cap - v.x) / BNO_SampleRate;   // BNO_SampleRate is a placeholder until
		  Vel_y = (Vel_y_Cap - v.y) / BNO_SampleRate;   // I can figure out how to set it or
		  Vel_z = (Vel_z_Cap - v.z) / BNO_SampleRate;   // Find it

		  Vel_total = sqrt(pow(Vel_x,2) + pow(Vel_y,2));  // Vel Plugged into PID
	  }

	  // There is code to get the velocity from BNO as well as encoder

	  Vel_x_Cap = v.x;  // First Captures for Vel Calcs
	  Vel_y_Cap = v.y;
	  Vel_z_Cap = v.z;

	  Accel_Total = sqrt(pow(Vel_x_Cap,2) + pow(Vel_y_Cap,2));

	  Vel_Capture = 1;    // Initial Accel Captured for Vel Calc

	  // Compute PID's

	  PID_Compute(&VEL_PID);			// Might need to have some sort of delays in all of this
	  PID_Compute(&ACCEL_PID);			// But the idea is there for sure
	  PID_Compute(&PITCH_PID);

	  duty = &PITCH_PIDOut;   // Not sure about the types here for the pointer
	  	  	  	  	  	  	  // (It's only a warning but idk if it'll mess up the code)

	  set_duty(&mot1,duty);	// Set Duty Cycles to Output of Final PID in Cascade
	  set_duty(&mot2,-duty);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Encoder Updating Code (Only Reading data from one encoder right now but maybe
// We wanna read both and compare to get an average or something)

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  timer_counter = __HAL_TIM_GET_COUNTER(&htim3);
  // Only Reading Data from One Encoder Right now
  update_encoder(&enc_instance_mot, &htim3);
  encoder_position = enc_instance_mot.position;
  encoder_velocity = enc_instance_mot.velocity;
}

// IC Interrupt for E-Stop and its Calcs

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
			{
				if (Is_First_Captured==0) // if the first value is not captured
				{
					IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
					Is_First_Captured = 1;  // set the first captured as true
				}

				else   // if the first is already captured
				{
					IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value


					Difference = IC_Val2-IC_Val1;


					float refClock = TIMCLOCK;
					float mFactor = 1000000/refClock;

					usWidth = Difference*mFactor;
					if (usWidth > 2050){
						IC_Val1 = IC_Val2;

					}
					else{
						//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
						Is_First_Captured = 0; // set it back to false
						dc = usWidth;
						dc = (dc - 1500)/2;
						//dc = 0;
						//set_duty(&mot1,dc);
					}

				}
			}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel1
		{
			if (ifc==0) // if the first value is not captured
			{
				IC_Val12 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
				ifc = 1;  // set the first captured as true
			}

			else   // if the first is already captured
			{
				IC_Val22 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value


				Difference2 = IC_Val22-IC_Val12;


				float refClock2 = TIMCLOCK2;
				float mFactor2 = 1000000/refClock2;

				usWidth2 = Difference2*mFactor2;
				if (usWidth2 > 2050){
					IC_Val12 = IC_Val22;

				}
				else{
					//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
					ifc = 0; // set it back to false
					dc2 = usWidth2;
					dc2 = (dc2 - 1500)/2;
					//dc = 0;
					//set_duty(&mot2,dc2);
				}
			}
		}
}

// Encoder Updating Function

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
 {
uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
static uint8_t first_time = 0;
if(!first_time)
{
   encoder_value ->velocity = 0;
   first_time = 1;
}
else
{
  if(temp_counter == encoder_value ->last_counter_value)
  {
    encoder_value ->velocity = 0;
  }
  else if(temp_counter > encoder_value ->last_counter_value)
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
      encoder_value ->velocity = -encoder_value ->last_counter_value -
	(__HAL_TIM_GET_AUTORELOAD(htim)-temp_counter);
    }
    else
    {
      encoder_value ->velocity = temp_counter -
           encoder_value ->last_counter_value;
    }
  }
  else
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
	encoder_value ->velocity = temp_counter -
            encoder_value ->last_counter_value;
    }
    else
    {
	encoder_value ->velocity = temp_counter +
	(__HAL_TIM_GET_AUTORELOAD(htim) -
              encoder_value ->last_counter_value);
    }
   }
}
encoder_value ->position += encoder_value ->velocity;
encoder_value ->last_counter_value = temp_counter;
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
