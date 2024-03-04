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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "iks01a3_motion_sensors.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t pulse = 1000; // Used to initialize the pulse of the OC of TIM3. Working frequency: 100Hz -> One interrupt every 10ms
uint8_t isSampling = 0; // Flag used to acquire the captured values from the sensor board every 10ms

// Reading of the raw accelerations from the accelerometer
int32_t xAxisReading; // useful for the monitor
int32_t yAxisReading;
int32_t zAxisReading;

// Reading of the filtered accelerations from the accelerometer (moving average filter)
int32_t xAxisReading_filt;
int32_t yAxisReading_filt;
int32_t zAxisReading_filt;

// Initialize variables for FilterMovingAvg (Moving average filter function)
int32_t filt_value[3] = {};
int32_t sum_readings[3] = {};
uint8_t i[3] = {};
#define SHIFT 25
int32_t accelero_readings[3][SHIFT] = {};

// Flag used to understand if the blue button (GPIO_PIN_13) has been pressed
uint8_t isBtnPressed =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/**
  * @brief Filter the value in input using the Moving average approach
  * @param Noisy input value to be filtered
  * @param Axis the measure is referring to. Could be :
  * - Axis x for instance 0
  * - Axis y for instance 1
  * - Axis z for instance 2
  * @retval filtered value according to the moving average approach
  */
int32_t FilterMovingAvg(int32_t noisy_value, uint8_t axis);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /**
    * @brief Initializes the motion sensors
    * @param Instance Motion sensor instance
    * @param Functions Motion sensor functions. Could be :
    * - MOTION_GYRO and/or MOTION_ACCELERO for instance 0
    * - MOTION_ACCELERO for instance 1
    * - MOTION_MAGNETO for instance 2
    * @retval BSP status
    */
    IKS01A3_MOTION_SENSOR_Init(1, MOTION_ACCELERO);
    IKS01A3_MOTION_SENSOR_Enable(1, MOTION_ACCELERO);
    HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
    IKS01A3_MOTION_SENSOR_Axes_t axes;

    char msg[7];
    // Initialize flags to understand the player status in the game
    uint8_t isJumping = 0;
    uint8_t isRolling = 0;
    uint8_t isShiftingLeft = 0;
    uint8_t isShiftingRight = 0;
    uint8_t isBoosting = 0;

    // Define the threshold used in the x, y, z Axes loop controls
	#define xThreshold 300
	#define yThreshold 250
	#define zThreshold 1200
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (isSampling == 1) //TIM3 OC triggered every 10ms
	  {
		  isSampling = 0;
		  IKS01A3_MOTION_SENSOR_GetAxes(1, MOTION_ACCELERO, &axes);
		  xAxisReading = axes.x;
		  xAxisReading_filt = FilterMovingAvg(xAxisReading,0);
		  yAxisReading = axes.y;
		  yAxisReading_filt = FilterMovingAvg(yAxisReading,1);
		  zAxisReading = axes.z;
		  zAxisReading_filt = FilterMovingAvg(zAxisReading,2);

		  /* x Axis threshold loop control.
		   * If the x positive threshold is being overcome -> JUMP (Insert the correspondent code)
		   * If the x negative threshold is being overcome -> ROLL (Insert the correspondent code)
		   * Used to jump and roll in the game
		  */
		  if(xAxisReading_filt > xThreshold && isJumping == 0)
		  {
			  isJumping = 1; // to ensure to capture only one jump command
			  sprintf(msg,"JUMP \n");
			  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen( msg),HAL_MAX_DELAY);
		  }
		  else if(xAxisReading_filt < -xThreshold && isRolling == 0)
		  {
			  isRolling = 1; // to ensure to capture only one jump command
			  sprintf(msg,"ROLL \n");
			  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		  }
		  else if(xAxisReading_filt > -xThreshold && xAxisReading_filt < xThreshold)
		  {
			  isJumping = 0;
			  isRolling = 0;
		  }

		  /* y Axis threshold loop control.
		   * If the y positive threshold is being overcome -> LEFT (Insert the correspondent code)
		   * If the y negative threshold is being overcome -> RIGHT (Insert the correspondent code)
		   * If between the two y threshold -> keeps the player in the middle
		   * Used to shift left or right in the game
		  */
		  if(yAxisReading_filt > yThreshold && isShiftingLeft == 0) // to ensure to capture only one left command
		  {
			  isShiftingLeft = 1;
			  sprintf(msg,"LEFT \n");
			  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		  }
		  else if(yAxisReading_filt < -yThreshold && isShiftingRight == 0) // to ensure to capture only one right command
		  {
			  isShiftingRight = 1;
			  sprintf(msg,"RIGHT\n");
			  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		  }
		  else if(yAxisReading_filt > -yThreshold && yAxisReading_filt < yThreshold)
		  {
			  if (isShiftingLeft == 1)
			  {
				  sprintf(msg,"RIGHT\n");
				  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
				  isShiftingLeft = 0;
			  }
			  if (isShiftingRight == 1)
			  {
				  sprintf(msg,"LEFT \n");
				  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
				  isShiftingRight = 0;
			  }
		  }

		 /* z Axis threshold loop control.
		  * If the z positive threshold is being overcome -> SPACE (Insert the correspondent code)
		  * Used to start a new game or resume the game after having paused it. Used also to activate the boost in the game
		 */
		 if(zAxisReading_filt > zThreshold && isBoosting == 0) // to ensure to capture only one boost command
		 {
			 isBoosting = 1;
			 sprintf(msg,"SPACE\n");
			 HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		 }
		 else if(zAxisReading_filt < zThreshold)
		 {
			 isBoosting = 0;
		 }

		 /* Button press loop control.
		  * If the blue button (GPIO_PIN_13) is pressed -> ESC (Insert the correspondent code)
		  * Used to pause the game
		 */
		 if (isBtnPressed==1)
		 {
			 isBtnPressed=0;
			 sprintf(msg,"ESC  \n");
			 HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 840-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef *htim) {
// Operations to be performed each time the OC is called (every 10ms)
	if (htim == &htim3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			pulse = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) + 1000);
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
			isSampling = 1;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13 )
	{
		// Operations to be performed each time the blue button (GPIO_PIN_13) is pressed
		isBtnPressed = 1;
	}
}

/**
  * @brief Filter the value in input using the Moving average approach
  * @param Noisy input value to be filtered
  * @param Axis the measure is referring to. Could be :
  * - Axis x for instance 0
  * - Axis y for instance 1
  * - Axis z for instance 2
  * @retval filtered value according to the moving average approach
  */
int32_t FilterMovingAvg(int32_t noisy_value, uint8_t axis) // i=0 at the beginning
{
	int32_t old_noisy_value;
	old_noisy_value = accelero_readings[axis][i[axis]];
	accelero_readings[axis][i[axis]] = noisy_value;
	sum_readings[axis] = sum_readings[axis] + accelero_readings[axis][i[axis]]-old_noisy_value;
	filt_value[axis] = (int32_t)sum_readings[axis]/SHIFT;
	i[axis]++;
	if (i[axis] == SHIFT)
	{
		i[axis] = 0;
	}
	return filt_value[axis];
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
