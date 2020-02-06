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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
UART_HandleTypeDef huart2;

/* Definitions for norm_task */
osThreadId_t norm_taskHandle;
const osThreadAttr_t norm_task_attributes = {
  .name = "norm_task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for high_task */
osThreadId_t high_taskHandle;
const osThreadAttr_t high_task_attributes = {
  .name = "high_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128
};
/* Definitions for low_task */
osThreadId_t low_taskHandle;
const osThreadAttr_t low_task_attributes = {
  .name = "low_task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128
};
/* Definitions for bin_semaphore */
osSemaphoreId_t bin_semaphoreHandle;
const osSemaphoreAttr_t bin_semaphore_attributes = {
  .name = "bin_semaphore"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void start_norm_task(void *argument);
void start_high_task(void *argument);
void start_low_task(void *argument);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of bin_semaphore */
  bin_semaphoreHandle = osSemaphoreNew(1, 1, &bin_semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of norm_task */
  norm_taskHandle = osThreadNew(start_norm_task, NULL, &norm_task_attributes);

  /* creation of high_task */
  high_taskHandle = osThreadNew(start_high_task, NULL, &high_task_attributes);

  /* creation of low_task */
  low_taskHandle = osThreadNew(start_low_task, NULL, &low_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_norm_task */
/**
  * @brief  Function implementing the norm_task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_start_norm_task */
void start_norm_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		char * entry_str = "Running Normal task.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) entry_str, strlen(entry_str), 100);
				
		char * leave_str = "Leaving Normal task.\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) leave_str, strlen(leave_str), 100);

    osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_start_high_task */
/**
* @brief Function implementing the high_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_high_task */
void start_high_task(void *argument)
{
  /* USER CODE BEGIN start_high_task */
  /* Infinite loop */
  for(;;)
  {
		char * entry_str = "Running High task.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) entry_str, strlen(entry_str), 100);
		
		osSemaphoreAcquire(bin_semaphoreHandle, osWaitForever);
		char * semaphore_str = "Semaphore acquired by High!\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) semaphore_str, strlen(semaphore_str), 100);
		
		char * leave_str = "Leaving High task and releasing semaphore.\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) leave_str, strlen(leave_str), 100);

	  osSemaphoreRelease(bin_semaphoreHandle);
    osDelay(500);
  }
  /* USER CODE END start_high_task */
}

/* USER CODE BEGIN Header_start_low_task */
/**
* @brief Function implementing the low_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_low_task */
void start_low_task(void *argument)
{
  /* USER CODE BEGIN start_low_task */
  /* Infinite loop */
  for(;;)
  {
		char * entry_str = "Running Low task.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) entry_str, strlen(entry_str), 100);
		
		osSemaphoreAcquire(bin_semaphoreHandle, osWaitForever);
		char * semaphore_str = "Semaphore acquired by Low!\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) semaphore_str, strlen(semaphore_str), 100);
		
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	
		char * leave_str = "Leaving Low task.\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) leave_str, strlen(leave_str), 100);

		osSemaphoreRelease(bin_semaphoreHandle);
    osDelay(500);
  }
  /* USER CODE END start_low_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
