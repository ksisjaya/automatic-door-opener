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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*** GLOBAL VARIABLES ***/

/* Useful LCD Instructions */

uint8_t LCD_InstRcv = 0xFE;
uint8_t LCD_NewLine = 0xC0;
uint8_t LCD_Beginning = 0x80;
uint8_t LCD_Clear = 1;

/* Button Types for User Input */

typedef enum {
	start_e		= 1,
	enter_e		= 2,
	scroll_e	= 4
} button;

button next_valid_button = start_e;

/* Operational Mode Variables */

typedef enum {
	locked_mode_e,
	setup_mode_e,
	run_mode_e,
	choosing_mode_e
} state;

int first_entering_state = 0;
state mode_state = choosing_mode_e;
state current_mode_display = 0;
uint8_t operation_modes[3][5] = {"Lock ", "Setup", "Run  "};

/* Locked Type Modes */

typedef enum {
	locked_closed_e,
	locked_opened_e
} lock_mode;

lock_mode current_locked_display = locked_closed_e;
uint8_t locked_modes[2][10] = {"Close Door", "Open Door "};

/* User Input Parameters */

typedef enum {
	P1_e,
	P2_e,
	P3_e,
	P4_e,
	P5_e
} parameter;

parameter current_parameter = P1_e;
int current_parameter_index = 0;

int p1_p2_values[4] = {8, 2, 4, 6};
int p3_values[4] = {20, 5, 10, 15};
uint32_t p4_p5_values[4] = {40000, 17000, 25000, 30000};

uint8_t p1_p2_display[4][11] = {"8ft Y:Enter", "2ft Y:Enter", "4ft Y:Enter", "6ft Y:Enter"};
uint8_t p3_display[4][13] = {"20sec Y:Enter", "05sec Y:Enter", "10sec Y:Enter", "15sec Y:Enter"};
uint8_t p4_p5_display[4][15] = {"Speed 4 Y:Enter", "Speed 1 Y:Enter", "Speed 2 Y:Enter", "Speed 3 Y:Enter"};

int P1 = 0;
int P2 = 0;
int P3 = 0;
uint32_t P4 = 0;
uint32_t P5 = 0;

/* Misc. Flags */

int collision = 0;
int running = 0;
int door_closed = 0;
int door_opened = 0;

/* Ultrasonic Variables */

typedef enum {
	  not_within_boundaries_e = 0,
	  within_inside_ultrasonic_e = 1,
	  within_outside_ultrasonic_e = 2
} boundary;

boundary object_in_boundaries = not_within_boundaries_e;

int inside_us_rising_edge = 1;
uint32_t inside_us_prev_time = 0;
uint32_t inside_us_curr_time = 0;

int outside_us_rising_edge = 1;
uint32_t outside_us_prev_time = 0;
uint32_t outside_us_curr_time = 0;


/*** HELPER FUNCTION DECLARATIONS ***/


/* @brief Helper function to display strings to LCD
 * @param line1: uint8_t array to display on first line of the LCD
 * @param line1_size: size of line1
 * @param line2: uint8_t array to display on seconds line of the LCD
 * @param line2_size: size of line2
 */

void display_lcd(uint8_t line1[], size_t line1_size, uint8_t line2[], size_t line2_size);

/* @brief Helper function to rotate the motor in forward direction (+ve RPM)
 * @param ch1_limit: int32_t max voltage for the motor driver IN1
 * @param ch1_step: int32_t step size for PWM
 */

void motor_forward(int32_t ch1_limit, int32_t ch1_step);

/* @brief Helper function to rotate the motor in reverse direction (-ve RPM)
 * @param ch2_limit: int32_t max voltage for the motor driver IN2
 * @param ch2_step: int32_t step size for PWM
 */

void motor_reverse(int32_t ch2_limit, int32_t ch2_step);

/* @brief Helper function to turn off the motor
 */

void motor_off();

/* @brief Helper function to calculate microseconds given two clock counters
 * @param previous: uint32_t clock counter value at beginning of measurement
 * @param current: uint32_t clock counter value at end of measurement
 */

uint32_t get_microseconds(uint32_t previous, uint32_t current);

/* @brief Helper function to calculate inches of object from ultrasonic sensor
 * @param time: uint32_t microseconds of active high from ultrasonic
 */

int get_ultrasonic_inches(uint32_t time_us);


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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1); // start the TIM1 module
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // STart PWM on Channel 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // STart PWM on Channel 2
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // STart PWM on Channel 3
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // STart PWM on Channel 4


  HAL_TIM_Base_Start_IT(&htim2); //Start the TIM2 module with interrupts enabled
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Start PWM on CH1
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2); //Start Output Compare on CH2 with interrupts enabled


  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // STart PWM on Channel 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // STart PWM on Channel 3
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // STart PWM on Channel 4

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

start:
  while (1)
  {

	  /* Collision Flag
	   * Stop everything until collision is over
	   */
	  while (collision) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  }


	  if (running) {


		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

		  uint32_t start_time = __HAL_TIM_GetCounter(&htim2);
		  while (__HAL_TIM_GetCounter(&htim2) <= start_time + 10) {}

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

		  HAL_Delay(60);

	  }

	  if (object_in_boundaries & (within_outside_ultrasonic_e | within_inside_ultrasonic_e)) {

		  object_in_boundaries = not_within_boundaries_e;

		  motor_reverse (P4, 1000);
		  while (!door_opened) {
			  if (collision) {
				  goto start;
			  }
		  }

		  HAL_Delay(P3 * 1000);

		  motor_forward (P4, 1000);
		  while (!door_closed) {
			  if (collision) {
				  goto start;
			  }
		  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 PA10
                           PA11 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5) {	// Start Button

		current_parameter = P1_e;
		current_parameter_index = 0;
		running = 0;
		mode_state = choosing_mode_e;
		uint8_t line1[] = "Press Scroll";
		uint8_t line2[] = "To Choose a Mode";
		display_lcd(line1, sizeof(line1), line2, sizeof(line2));
		next_valid_button = scroll_e;

	}
	else if(GPIO_Pin == GPIO_PIN_6 && next_valid_button & enter_e) // Enter Button
    {

		switch(mode_state){

		case choosing_mode_e:

			first_entering_state = 1;
			mode_state = current_mode_display;
			uint8_t line1[] = "Press Enter";
			uint8_t line2[] = "Again To Begin";
			display_lcd(line1, sizeof(line1), line2, sizeof(line2));
			next_valid_button = enter_e;

			break;

		case locked_mode_e:

			if(first_entering_state){

				first_entering_state = 0;
				uint8_t line1[] = "In Locked Mode";
				uint8_t line2[] = "Press Scroll";
				display_lcd(line1, sizeof(line1), line2, sizeof(line2));
				next_valid_button = scroll_e;

			}
			else {

				next_valid_button = start_e;
				if(current_locked_display == locked_closed_e) {
					uint8_t line1[] = "In Locked Mode";
					uint8_t line2[] = "Closing Door";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

					motor_forward(17000, 1000);

				}
				else {
					uint8_t line1[] = "In Locked Mode";
					uint8_t line2[] = "Opening Door";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

					motor_reverse(17000, 1000);

				}
			}
			break;

		case setup_mode_e:

			if(first_entering_state){

				first_entering_state = 0;
				uint8_t line1[] = "In Setup Mode";
				uint8_t line2[] = "Press Scroll";
				display_lcd(line1, sizeof(line1), line2, sizeof(line2));
				next_valid_button = scroll_e;

			}
			else {

				if(current_parameter == P1_e) {

					P1 = p1_p2_values[current_parameter_index];
					current_parameter = P2_e;
					next_valid_button = scroll_e;

	    			uint8_t line1[] = "P1 Set. Press";
	    			uint8_t line2[] = "scroll to set P2";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

				}
				else if(current_parameter == P2_e) {

					P2 = p1_p2_values[current_parameter_index];
					current_parameter = P3_e;
					next_valid_button = scroll_e;

					uint8_t line1[] = "P2 Set. Press";
					uint8_t line2[] = "scroll to set P3";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

				}
				else if(current_parameter == P3_e) {

					P3 = p3_values[current_parameter_index];
					current_parameter = P4_e;
					next_valid_button = scroll_e;

					uint8_t line1[] = "P3 Set. Press";
					uint8_t line2[] = "scroll to set P4";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

				}
				else if(current_parameter == P4_e) {

					P4 = p4_p5_values[current_parameter_index];
					current_parameter = P5_e;
					next_valid_button = scroll_e;

					uint8_t line1[] = "P4 Set. Press";
					uint8_t line2[] = "scroll to set P5";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

				}
				else if(current_parameter == P5_e) {

					P5 = p4_p5_values[current_parameter_index];
					next_valid_button = start_e;

					uint8_t line1[] = "Setup Complete";
					uint8_t line2[] = "Press Start";
					display_lcd(line1, sizeof(line1), line2, sizeof(line2));

				}

				current_parameter_index = 0;

			}

			break;

		case run_mode_e:

			if (!P1 || !P2 || !P3 || !P4 || !P5) {
				uint8_t line1[] = "ERROR:SETUP";
				uint8_t line2[] = "Press START";
				display_lcd(line1, sizeof(line1), line2, sizeof(line2));
				next_valid_button = start_e;
				break;
			}


			running = 1;
			uint8_t line3[] = "In Run Mode";
			uint8_t line4[] = "Exit:Press Start";
			display_lcd(line3, sizeof(line3), line4, sizeof(line4));

			break;


		default:
			break;

		}
    }
    else if(GPIO_Pin == GPIO_PIN_7 && next_valid_button & scroll_e)	// Scroll Button
    {

    	uint8_t line2[] = "Y:Enter N:Start";
    	switch(mode_state){

    	case choosing_mode_e:

    		current_mode_display = (current_mode_display + 1) % 3;
    		display_lcd(operation_modes[current_mode_display], sizeof(operation_modes[current_mode_display]), line2, sizeof(line2));
    		next_valid_button = enter_e | scroll_e;
    		break;

    	case locked_mode_e:

    		current_locked_display = (current_locked_display + 1) % 2;
    		display_lcd(locked_modes[current_locked_display], sizeof(locked_modes[current_locked_display]), line2, sizeof(line2));
    		next_valid_button = enter_e | scroll_e;
    		break;

    	case setup_mode_e:

    		current_parameter_index = (current_parameter_index + 1) % 4;

    		if(current_parameter == P1_e) {

    			uint8_t line1[] = "Setup Mode. P1:";
				display_lcd(line1, sizeof(line1), p1_p2_display[current_parameter_index], sizeof(p1_p2_display[current_parameter_index]));

			}
    		else if(current_parameter == P2_e) {

    			uint8_t line1[] = "Setup Mode. P2:";
				display_lcd(line1, sizeof(line1), p1_p2_display[current_parameter_index], sizeof(p1_p2_display[current_parameter_index]));

			}
    		else if(current_parameter == P3_e) {

    			uint8_t line1[] = "Setup Mode. P4:";
				display_lcd(line1, sizeof(line1), p3_display[current_parameter_index], sizeof(p3_display[current_parameter_index]));

			}
    		else if(current_parameter == P4_e) {

				uint8_t line1[] = "Setup Mode. P4:";
				display_lcd(line1, sizeof(line1), p4_p5_display[current_parameter_index], sizeof(p4_p5_display[current_parameter_index]));

			}
    		else if(current_parameter == P5_e) {

    			uint8_t line1[] = "Setup Mode. P5:";
				display_lcd(line1, sizeof(line1), p4_p5_display[current_parameter_index], sizeof(p4_p5_display[current_parameter_index]));

			}

			next_valid_button = enter_e | scroll_e;

    		break;

    	case run_mode_e:
    		break;

    	default:
    		break;

    	}

    }
    else if(GPIO_Pin == GPIO_PIN_0)	// Open Limit Switch
	{
    	door_opened = 1;
    	motor_forward(20000, 1000);
    	motor_off();
	}
    else if(GPIO_Pin == GPIO_PIN_1)	// Close Limit Switch
	{
    	door_closed = 1;
    	motor_reverse(20000, 1000);
		motor_off();
	}
    else if(GPIO_Pin == GPIO_PIN_9)	// Inside Ultrasonic Interrupt
	{

    	if (inside_us_rising_edge) {

    		inside_us_rising_edge = 0;
    		inside_us_prev_time = __HAL_TIM_GetCounter(&htim2);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    	}
    	else {

    		inside_us_rising_edge = 1;
    		inside_us_curr_time = __HAL_TIM_GetCounter(&htim2);
    		uint32_t time = get_microseconds(inside_us_prev_time, inside_us_curr_time);

    		if (get_ultrasonic_inches(time) <= P2 * 12) {

    			object_in_boundaries |= within_inside_ultrasonic_e;
    			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    		}

    	}

	}
    else if(GPIO_Pin == GPIO_PIN_10)	// Outside Ultrasonic Interrupt
	{

		if (outside_us_rising_edge) {

			outside_us_rising_edge = 0;
			outside_us_prev_time = __HAL_TIM_GetCounter(&htim2);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		}
		else {

			outside_us_rising_edge = 1;
			outside_us_curr_time = __HAL_TIM_GetCounter(&htim2);
			uint32_t time = get_microseconds(outside_us_prev_time, outside_us_curr_time);

			if (get_ultrasonic_inches(time) <= P3 * 12) {

				object_in_boundaries |= within_outside_ultrasonic_e;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

			}

		}

	}
    else if(GPIO_Pin == GPIO_PIN_2)	// Collision Switch
	{

    	if (!collision) {

    		collision = 1;
    		motor_off();

    	}
    	else {

    		collision = 0;
    		motor_forward (17000, 1000);

    	}

	}
    else if(GPIO_Pin == GPIO_PIN_8)	// Q1 Signal
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
	}
    else if(GPIO_Pin == GPIO_PIN_12)	// IDX Signal
    {
    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
    }
}


/*** HELPER FUNCTION DEFINITIONS ***/

/* @brief Helper function to display strings to LCD
 * @param line1: uint8_t array to display on first line of the LCD
 * @param line1_size: size of line1
 * @param line2: uint8_t array to display on seconds line of the LCD
 * @param line2_size: size of line2
 */

void display_lcd(uint8_t line1[], size_t line1_size, uint8_t line2[], size_t line2_size) {

	HAL_UART_Transmit(&huart6, &LCD_InstRcv, 1, 1000);
	HAL_UART_Transmit(&huart6, &LCD_Clear, 1, 1000);


	if (line1 != NULL && line1_size != 0){
		HAL_UART_Transmit(&huart6, &LCD_InstRcv, 1, 1000);
		HAL_UART_Transmit(&huart6, &LCD_Beginning, 1, 1000);
		HAL_UART_Transmit(&huart6, line1, line1_size, 1000);
	}

	if (line2 != NULL && line2_size != 0){
		HAL_UART_Transmit(&huart6, &LCD_InstRcv, 1, 1000);
		HAL_UART_Transmit(&huart6, &LCD_NewLine, 1, 1000);
		HAL_UART_Transmit(&huart6, line2, line2_size, 1000);
	}

}

/* @brief Helper function to rotate the motor in forward direction (+ve RPM)
 * @param ch1_limit: int32_t max voltage for the motor driver IN1
 * @param ch1_step: int32_t step size for PWM
 */

void motor_forward(int32_t ch1_limit, int32_t ch1_step) {

	door_opened = 0;
	int32_t CH1_DC = 0;

	/* Move the Motor Forward */
	while (CH1_DC < ch1_limit) {
		TIM1->CCR1 = CH1_DC;
		TIM1->CCR2 = 0;
		CH1_DC += 1000;
	}
}

/* @brief Helper function to rotate the motor in reverse direction (-ve RPM)
 * @param ch2_limit: int32_t max voltage for the motor driver IN2
 * @param ch2_step: int32_t step size for PWM
 */

void motor_reverse(int32_t ch2_limit, int32_t ch2_step) {

	door_closed = 0;
	int32_t CH2_DC = 0;

	/* Move the Motor Backward */
	while (CH2_DC < ch2_limit) {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = CH2_DC;
		CH2_DC += ch2_step;
	}
}

/* @brief Helper function to turn off the motor
 */

void motor_off() {

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;

}


/* @brief Helper function to calculate microseconds given two clock counters
 * @param previous: uint32_t clock counter value at beginning of measurement
 * @param current: uint32_t clock counter value at end of measurement
 */

uint32_t get_microseconds(uint32_t previous, uint32_t current) {

	return previous < current ? current - previous : (100000 - previous) + current;

}

/* @brief Helper function to calculate inches of object from ultrasonic sensor
 * @param time: uint32_t microseconds of active high from ultrasonic
 */

int get_ultrasonic_inches(uint32_t time_us) {

	return time_us / 148;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
