/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include "hd44780_driver.h"
#include "pca9685.h"
#include "jsmn.h"
#include "ringbuffer_dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_GPIO_ADDR	0x80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId TaskLedsHandle;
osThreadId TaskAdcHandle;
osThreadId TaskAt_debugHandle;
osMessageQId myQueueCmdsHandle;
osMessageQId myQueueDozeWaterHandle;
osMessageQId myQueueMsgHandle;
osTimerId myTimerPumpHandle;
osSemaphoreId mBinSemDozeWaterHandle;
osStaticSemaphoreDef_t mBinSemDozeWaterControlBlock;
/* USER CODE BEGIN PV */
struct Status {
	int8_t isPumpOn;
	int8_t isFanOn;
	int8_t temperature;
	uint32_t gas;
} curStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskLeds(void const * argument);
void StartTaskAdc(void const * argument);
void task_at_debug(void const * argument);
void CallbackPump(void const * argument);

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  pca9685_init(&hi2c1, I2C_GPIO_ADDR);
  pca9685_pwm(&hi2c1, I2C_GPIO_ADDR, 1, 0, 0);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of mBinSemDozeWater */
  osSemaphoreStaticDef(mBinSemDozeWater, &mBinSemDozeWaterControlBlock);
  mBinSemDozeWaterHandle = osSemaphoreCreate(osSemaphore(mBinSemDozeWater), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimerPump */
  osTimerDef(myTimerPump, CallbackPump);
  myTimerPumpHandle = osTimerCreate(osTimer(myTimerPump), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueueCmds */
  osMessageQDef(myQueueCmds, 128, uint8_t);
  myQueueCmdsHandle = osMessageCreate(osMessageQ(myQueueCmds), NULL);

  /* definition and creation of myQueueDozeWater */
  osMessageQDef(myQueueDozeWater, 1, int8_t);
  myQueueDozeWaterHandle = osMessageCreate(osMessageQ(myQueueDozeWater), NULL);

  /* definition and creation of myQueueMsg */
  osMessageQDef(myQueueMsg, 16, uint32_t);
  myQueueMsgHandle = osMessageCreate(osMessageQ(myQueueMsg), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 228);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskLeds */
  osThreadDef(TaskLeds, StartTaskLeds, osPriorityNormal, 0, 128);
  TaskLedsHandle = osThreadCreate(osThread(TaskLeds), NULL);

  /* definition and creation of TaskAdc */
  osThreadDef(TaskAdc, StartTaskAdc, osPriorityNormal, 0, 128);
  TaskAdcHandle = osThreadCreate(osThread(TaskAdc), NULL);

  /* definition and creation of TaskAt_debug */
  osThreadDef(TaskAt_debug, task_at_debug, osPriorityBelowNormal, 0, 128);
  TaskAt_debugHandle = osThreadCreate(osThread(TaskAt_debug), NULL);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
	// recv interrupt force enable
	USART2->CR1 |= USART_CR1_RXNEIE;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
	// recv interrupt force enable
	USART3->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D4_Pin 
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_D4_Pin LCD_D5_Pin 
                           LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin 
                          |LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RW_Pin */
  GPIO_InitStruct.Pin = LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD12 PD13 PD14 
                           PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void TurnFanOff(void){
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	pca9685_pwm(&hi2c1, I2C_GPIO_ADDR, 1, 0, 0);
	osMessagePut(myQueueMsgHandle, (uint32_t)"Fan stopped", 1);
	curStatus.isFanOn = 0;
}

static void TurnFanOn(void){
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	pca9685_pwm(&hi2c1, I2C_GPIO_ADDR, 1, 0, 4095);
	osMessagePut(myQueueMsgHandle, (uint32_t)"Fan started", 1);
	curStatus.isFanOn = 1;
}

static void TurnPumpOff(void){
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	pca9685_pwm(&hi2c1, I2C_GPIO_ADDR, 0, 0, 0);
	osMessagePut(myQueueMsgHandle, (uint32_t)"Pump stopped", 1);
	curStatus.isPumpOn = 0;
}

static void TurnPumpOn(void){
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	pca9685_pwm(&hi2c1, I2C_GPIO_ADDR, 0, 0, 4095);
	osMessagePut(myQueueMsgHandle, (uint32_t)"Pump started", 1);
	curStatus.isPumpOn = 1;
}

void DoseWater(void) {
	osStatus res = osMessagePut(myQueueDozeWaterHandle, 1, 1);
	if (res == osOK) {
		osMessagePut(myQueueCmdsHandle, PUMP_ON, 1);
		osTimerStart(myTimerPumpHandle, 50);
	}
}

static uint8_t LRC(const char *data) {
	uint8_t lrc = 0;
	//   for (const byte of data) {
	//     lrc += byte;
	//   }
	while(*data) {
	   lrc += *data++;
	}
	return ((lrc ^ 0xff) + 1) & 0xff;
}

const int8_t SOH = 0x01;
const int8_t STX = 0x02;
const int8_t ETX = 0x03;
const int8_t EOT = 0x04;

static void makePacket(const char *data, uint8_t *buf) {


	const int8_t HEADER_SIZE = 4;
//	const int8_t TRAILER_SIZE = 3;
//	const int8_t OVERHEAD_SIZE = HEADER_SIZE + TRAILER_SIZE;

//	const buf = Buffer.alloc(data.length + OVERHEAD_SIZE);
	buf[0] = SOH;
	buf[1] = strlen(data) & 0xff;
	buf[2] = (strlen(data) >> 8) & 0xff;
	buf[3] = STX;
	int32_t trailer = HEADER_SIZE + strlen(data);
	memcpy((char *)(buf + HEADER_SIZE), data, strlen(data));
	//const trailer = buf.slice(data.length + HEADER_SIZE);
	*(buf + trailer + 0) = ETX;
	*(buf + trailer + 1) = LRC(data);
	*(buf + trailer + 2) = EOT;
//	trailer[1] = Packet.LRC(data);
//	trailer[2] = EOT;
//	return buf;
}


void uputs(UART_HandleTypeDef* huart,char* str) {
	size_t sz = strlen(str);
	HAL_UART_Transmit(huart, (uint8_t*)str, sz, 10);
	HAL_UART_Transmit(huart, (uint8_t*)"\n\r", 2, 10);
}

size_t ugets(UART_HandleTypeDef*huart, char*str){}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	LcdInit();
	LcdShow2Lines("WirelessFirmware", "0.0.0");
	osDelay(3000);
	uputs(DEBUG_SERIAL, "WirelessFirmware started");
	at_available();
  /* Infinite loop */
	for(;;)
	{
		osDelay(100);
		osEvent evt = osMessageGet(myQueueMsgHandle, osWaitForever);
		if (evt.status == osEventMessage) {
//			const char *msg = (const char *)evt.value.p;
//			LcdShowMessage(msg);
			char l1[9] = { 0 };
			char l2[9] = { 0 };
			char l3[9] = { 0 };
			char l4[9] = { 0 };
			sprintf(l1, "Temp:%2d", curStatus.temperature);
			sprintf(l2, "Pump:%3s", (curStatus.isPumpOn ? "ON" : "OFF"));
			sprintf(l3, "Fan:%3s", (curStatus.isFanOn ? "ON" : "OFF"));
			sprintf(l4, "Gas:%4ld", curStatus.gas);
			LcdShow4Lines(l1, l2, l3, l4);
			osDelay(100);
		}
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTaskLeds */
/**
* @brief Function implementing the myTaskLeds thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLeds */
void StartTaskLeds(void const * argument)
{
  /* USER CODE BEGIN StartTaskLeds */
  /* Infinite loop */
	uint32_t dval = 250;
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		osDelay(dval);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		osDelay(dval);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		osDelay(dval);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		osDelay(dval);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	}
  /* USER CODE END StartTaskLeds */
}

/* USER CODE BEGIN Header_StartTaskAdc */
/**
* @brief Function implementing the myTaskAdc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskAdc */
void StartTaskAdc(void const * argument)
{
  /* USER CODE BEGIN StartTaskAdc */
  /* Infinite loop */
	int8_t isFanOn = 0;
	for(;;)
	{
		osDelay(3000);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		uint32_t temperature = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
//		static char buf[100];
//		sprintf(buf, "%ld", value);
//		osMessagePut(myQueueMsgHandle, (uint32_t)buf, 1);
		double voltage = 3.3 * temperature / 4095;
		curStatus.temperature = (2.5 - voltage) * 50 - 25;
		if (curStatus.temperature <= 28 && isFanOn) {
			osMessagePut(myQueueCmdsHandle, FAN_OFF, 1);
			isFanOn = 0;
		}
		else if (curStatus.temperature > 28 && !isFanOn) {
			osMessagePut(myQueueCmdsHandle, FAN_ON, 1);
			isFanOn = 1;
		}

//		if (0) { DMA!!!!!
//			uint8_t buf[100] = { 0 };
//			sprintf((char *)buf, "value=%lu\n", value);
//			HAL_UART_Transmit(&huart3, buf, strlen((char *)buf), 1000);
//		}

		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 1000);
		uint32_t pollution = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);
		static char buf[100];
		sprintf(buf, "%ld", pollution);
		curStatus.gas = pollution;
		osMessagePut(myQueueMsgHandle, (uint32_t)buf, 1);
	}
  /* USER CODE END StartTaskAdc */
}

/* USER CODE BEGIN Header_task_at_debug */
/**
* @brief Function implementing the TaskAt_debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_at_debug */
void task_at_debug(void const * argument)
{
  /* USER CODE BEGIN task_at_debug */
  /* Infinite loop */
	uputs(DEBUG_SERIAL, "task at debug started");
  for(;;)
  {
	  /* output input from esp */
	  uint8_t r;
		if(HAL_OK == HAL_UART_Receive(AT_SERIAL, &r, 1, 100)) {
			HAL_UART_Transmit(DEBUG_SERIAL, &r,1,0);
		}

  }
  /* USER CODE END task_at_debug */
}

/* CallbackPump function */
void CallbackPump(void const * argument)
{
  /* USER CODE BEGIN CallbackPump */
	//osTimerStop(myTimerPumpHandle);
	osDelay(3000);
	osMessageGet(myQueueDozeWaterHandle, 100);
	osMessagePut(myQueueCmdsHandle, PUMP_OFF, 1);
  /* USER CODE END CallbackPump */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
	while(1);
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
