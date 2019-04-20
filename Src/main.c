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
#include "setDef.h"
#include "ssd1306.h"
#include "bmp280.h"
#include "ws2812.h"
/*
post-build steps command:
arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"
*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//const char *ver = "ver. 2.4";//16.04.2019 major changes : add freeRTOS
//const char *ver = "ver. 2.5";//19.04.2019 major changes : add mailQueue
const char *ver = "ver. 2.6";//20.04.2019 major changes : add new feature - support ws2812

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defTaskHandle;
osThreadId bmpTaskHandle;
osThreadId pwmTaskHandle;
/* USER CODE BEGIN PV */

osMailQId mailQueue = NULL;

HAL_StatusTypeDef i2cError = HAL_OK;
const uint32_t min_wait_ms = 350;
const uint32_t max_wait_ms = 1000;
result_t sensors = {0.0, 0.0, 0.0};
volatile static uint32_t secCounter = 0;
volatile static uint64_t HalfSecCounter = 0;
volatile static float dataADC = 0.0;

static const char *_extDate = "date=";
volatile uint32_t extDate = 0;
bool setDate = false;
static char RxBuf[MAX_UART_BUF];
volatile uint8_t rx_uk;
volatile uint8_t uRxByte = 0;

uint8_t GoTxDMA = 0;
const rgb_t ws2812_const[LED_COUNT] = {
	RGB_SET(L64,   0,   0),
	RGB_SET(0,   L64,   0),
	RGB_SET(0,     0, L64),
	RGB_SET(L64, L64,   0),
	RGB_SET(0,   L64, L64),
	RGB_SET(L64,   0, L64),
	RGB_SET(L64, L64, L64),
	RGB_SET(L32,   0,   0)
};
const dir_mode_t scena[] = {
   	ZERO_ALL,
	RAINBOW,
   	COLOR_DOWN,
	COLOR_UP,
	WHITE_DOWN,
	WHITE_UP,
	ZERO_ALL
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
void StartDefTask(void const * argument);
void StartBmpTask(void const * argument);
void StartPwmTask(void const * argument);

/* USER CODE BEGIN PFP */

int sec_to_str_time(uint32_t sec, char *stx);
void inc_secCounter();
uint32_t get_secCounter();
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
void Report(const char *txt, bool addTime);
void errLedOn(const char *from);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED_ERROR, GPIO_PIN_SET);//LEDs OFF

  // start timer1 + interrupt
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim1);
  //start ADC1 + interrupt
  HAL_ADC_Start_IT(&hadc1);
  //"start" rx_interrupt
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&uRxByte, 1);


  ssd1306_on(true);//screen ON
  if (!i2cError) {
	  ssd1306_init();//screen INIT
	  if (!i2cError) {
		  ssd1306_pattern();//set any params for screen
		  if (!i2cError) {
#ifdef SET_SSD1306_INVERT
			  ssd1306_invert();//set inverse color mode
			  if (!i2cError)
#endif
				  ssd1306_clear();//clear screen
		  }
      }
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  osMailQDef(nameQueue, 4, result_t);
  mailQueue = osMailCreate(osMailQ(nameQueue), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defTask */
  osThreadDef(defTask, StartDefTask, osPriorityAboveNormal, 0, 512);
  defTaskHandle = osThreadCreate(osThread(defTask), NULL);

  /* definition and creation of bmpTask */
  osThreadDef(bmpTask, StartBmpTask, osPriorityNormal, 0, 384);
  bmpTaskHandle = osThreadCreate(osThread(bmpTask), NULL);

  /* definition and creation of pwmTask */
  osThreadDef(pwmTask, StartPwmTask, osPriorityNormal, 0, 384);
  pwmTaskHandle = osThreadCreate(osThread(pwmTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	  LOOP_FOREVER();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
	secCounter     = 0; //1 sec counter (32bit)
	HalfSecCounter = 0; // 0.5 sec counter (64bit)
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 17999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
	memset(RxBuf, 0, MAX_UART_BUF);
	rx_uk = 0;
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-----------------------------------------------------------------------------
void set_Date(time_t epoch)
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;

	gmtime_r(&epoch, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
	else {
		sTime.Hours   = ts.tm_hour;
		sTime.Minutes = ts.tm_min;
		sTime.Seconds = ts.tm_sec;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
		else {
			setDate = true;
		}
	}
}
//-----------------------------------------------------------------------------
int sec_to_str_time(uint32_t sec, char *stx)
{
	uint32_t day = sec / (60 * 60 * 24);
	sec %= (60 * 60 * 24);
    uint32_t hour = sec / (60 * 60);
    sec %= (60 * 60);
    uint32_t min = sec / (60);
    sec %= 60;
    return (sprintf(stx, "%lu.%02lu:%02lu:%02lu", day, hour, min, sec));
}
//-----------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return secCounter;
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------
uint64_t get_hsCounter()
{
	return HalfSecCounter;
}
//-----------------------------------------------------------------------------
void inc_hsCounter()
{
	HalfSecCounter++;
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//------------------------------------------------------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//------------------------------------------------------------------------------------------
uint64_t get_hstmr(uint64_t hs)
{
	return (get_hsCounter() + hs);
}
//------------------------------------------------------------------------------------------
bool check_hstmr(uint64_t hs)
{
	return (get_hsCounter() >= hs ? true : false);
}
//------------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);//LED ON
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);//LED OFF
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);//LED ON

	if (from) {
		char *stx = (char *)malloc(strlen(from) + 16);
		if (stx) {
			sprintf(stx, "Error in '%s'\r\n", from);
			Report(stx, true);
			free(stx);
		}
	}
}
//----------------------------------------------------------------------------------------
//  if (return pointer != NULL) you must free this pointer after used
uint16_t sec_to_string(uint32_t sec, char *stx)
{
uint16_t ret = 0;

	//char *stx = (char *)malloc(24);
	//if (stx) {
		if (!setDate) {//no valid date in RTC
			uint32_t day = sec / (60 * 60 * 24);
			sec %= (60 * 60 * 24);
			uint32_t hour = sec / (60 * 60);
			sec %= (60 * 60);
			uint32_t min = sec / (60);
			sec %= 60;
			ret = sprintf(stx, "%03lu.%02lu:%02lu:%02lu | ", day, hour, min, sec);
		} else {//in RTC valid date (epoch time)
			RTC_TimeTypeDef sTime;
			RTC_DateTypeDef sDate;
			if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
			else {
				if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
				else {
					ret = sprintf(stx, "%04u.%02u.%02u %02u:%02u:%02u | ",
							sDate.Year + 1900, sDate.Month, sDate.Date,
							sTime.Hours, sTime.Minutes, sTime.Seconds);
				}
			}
		}
	//}

    return ret;
}
//----------------------------------------------------------------------------------------
//	Out to UART1 data from buffer (like printf)
//     txt - string for send to uart
//     addTime - flag insert or not secCount before txt
void Report(const char *txt, bool addTime)
{
HAL_StatusTypeDef er = HAL_OK;

	if (txt) {

		//if (osSemaphoreWait(semUartHandle, 1000/*osWaitForever*/) == osOK) {
		//if (osMutexWait(uMutexHandle, 1000) == osOK) {

			if (addTime) {
				uint32_t ep;
				char st[24] = {0};
				if (!setDate) ep = get_secCounter();
						 else ep = extDate;
				uint16_t dl = sec_to_string(ep, st);
				if (dl) er = HAL_UART_Transmit(&huart1, (uint8_t *)st, dl, 1000);
			}

			if (er == HAL_OK) er = HAL_UART_Transmit(&huart1, (uint8_t *)txt, strlen(txt), 2000);

		//	osMutexRelease(uMutexHandle);
		//	osSemaphoreRelease(semUartHandle);

		//} else er = HAL_ERROR;

	} else er = HAL_ERROR;

	if (er != HAL_OK) errLedOn(NULL);
}
//------------------------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		dataADC = ((float)HAL_ADC_GetValue(hadc)) * 3.3 / 4096;
	}
}
//------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		RxBuf[rx_uk & 0xff] = (char)uRxByte;
		if ((uRxByte == 0x0a) || (uRxByte == 0x0d)) {//end of line
			char *uk = strstr(RxBuf, _extDate);//const char *_extDate = "date=";
			if (uk) {
				uk += strlen(_extDate);
				if (*uk != '?') {
					if (strlen(uk) < 10) setDate = false;
					else {
						extDate = atoi(uk);
						sprintf(RxBuf, "%s:%lu\r\n", _extDate, extDate);
						set_Date((time_t)extDate);
					}
				} else setDate = true;
			}
			Report(RxBuf, false);
			memset(RxBuf, 0, MAX_UART_BUF);
			rx_uk = 0;
		} else rx_uk++;

		HAL_UART_Receive_IT(huart, (uint8_t *)&uRxByte, 1);

	}
}
//------------------------------------------------------------------------------------------

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefTask */
/**
  * @brief  Function implementing the defTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefTask */
void StartDefTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	char stx[192] = {0};
	char toScreen[64] = {0};
	result_t *ones = NULL;
	result_t evt = {0.0, 0.0, 0.0, 0};
	osEvent event;
/*
	HAL_Delay(10);
	char tmp[32];
	sprintf(tmp, "free %u", xPortGetFreeHeapSize());
	uint8_t col = ssd1306_calcx(strlen(tmp));
	ssd1306_text_xy(tmp, col, 1);
*/
	while (1) {
/**/
		if (mailQueue) {
			event = osMailGet(mailQueue, 100);
			if (event.status == osEventMail) ones = event.value.p;
		}
		if (ones) {
			memcpy((uint8_t *)&evt, (uint8_t *)ones, sizeof(result_t));
			osMailFree(mailQueue, ones);
			ones = NULL;
			sprintf(stx, "Vcc=%.3f v, ", dataADC);
			switch (evt.chip) {
					case BMP280_SENSOR : strcat(stx, "BMP280:"); break;
					case BME280_SENSOR : strcat(stx, "BME280:"); break;
						default : strcat(stx, "Unknown:");
			}
			sprintf(stx+strlen(stx), " Press=%.2f mmHg, Temp=%.2f DegC", evt.pres, evt.temp);
			if (evt.chip == BME280_SENSOR) sprintf(stx+strlen(stx), " Humidity=%.2f %%rH", evt.humi);
			strcat(stx, "\r\n");
			//
			if (i2cError == HAL_OK) {
				sprintf(toScreen, "mmHg : %.2f\nDegC : %.2f", evt.pres, evt.temp);
				if (evt.chip == BME280_SENSOR) sprintf(toScreen+strlen(toScreen), "\nHumi:%.2f %%rH", evt.humi);
#ifdef SET_SSD1306_INVERT
				ssd1306_invert();
				if (!i2cError)
#endif
					ssd1306_text_xy(toScreen, 1, 6);//send string to screen
			}
			Report(stx, true);//send data to UART1
		}
/**/
		osDelay(100);
	}

  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartBmpTask */
/**
* @brief Function implementing the bmpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmpTask */
void StartBmpTask(void const * argument)
{
  /* USER CODE BEGIN StartBmpTask */
	/* Infinite loop */

	// variable declaration for BMP280 sensor

	uint8_t data_rdx[DATA_LENGTH] = {0};
	uint8_t reg_id   = 0;
	uint8_t reg_stat = 0;
	uint8_t reg_mode = 0;
	uint8_t reg_conf = 0;
	size_t d_size    = 1;
	int32_t temp, pres, humi = 0;
/*
	HAL_Delay(20);
	char tmp[32];
	sprintf(tmp, "free %u", xPortGetFreeHeapSize());
	uint8_t col = ssd1306_calcx(strlen(tmp));
	ssd1306_text_xy(tmp, col, 3);
*/
	result_t sens;
	result_t *ones = NULL;
	uint32_t wait_sensor = get_tmr(2);

	while (1) {
/**/
		if (check_tmr(wait_sensor)) {
			wait_sensor = get_tmr(wait_sensor_def);
			if (i2c_master_reset_sensor(&reg_id) != HAL_OK) {
				wait_sensor = get_tmr(2);
				continue;
			}
			switch (reg_id) {
				case BMP280_SENSOR : d_size = 6; break;
				case BME280_SENSOR : d_size = 8; break;
				default : {
					wait_sensor = get_tmr(2);
					continue;
				}
			}
			if (i2c_master_test_sensor(&reg_stat, &reg_mode, &reg_conf, reg_id) != HAL_OK) {
				wait_sensor = get_tmr(2);
				continue;
			}
			reg_stat &= 0x0f;
			memset(data_rdx, 0, DATA_LENGTH);
			if (i2c_master_read_sensor(BMP280_REG_PRESSURE, data_rdx, d_size) == HAL_OK) {
				if (bmp280_readCalibrationData(reg_id) == HAL_OK) {
					pres = temp = 0;
					pres = (data_rdx[0] << 12) | (data_rdx[1] << 4) | (data_rdx[2] >> 4);
					temp = (data_rdx[3] << 12) | (data_rdx[4] << 4) | (data_rdx[5] >> 4);
					if (reg_id == BME280_SENSOR) humi = (data_rdx[6] << 8) | data_rdx[7];
					bmp280_CalcAll(&sens, reg_id, temp, pres, humi);
					if (mailQueue) {
						ones = (result_t *)osMailAlloc(mailQueue, 1000);//osWaitForever);
						if (ones) {
							memcpy((uint8_t *)ones, (uint8_t *)&sens, sizeof(result_t));
							osMailPut(mailQueue, (void *)ones);
						}
					}
				}
			}
		}
/**/
		osDelay(10);
	}

  /* USER CODE END StartBmpTask */
}

/* USER CODE BEGIN Header_StartPwmTask */
/**
* @brief Function implementing the pwmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPwmTask */
void StartPwmTask(void const * argument)
{
  /* USER CODE BEGIN StartPwmTask */

	//for WS2812 : tim2_channel2 + dma1_channel7
	uint8_t scenaINDEX = 0;
	bool new_scena = false;
	bool proc = true;
	const uint8_t maxSCENA = sizeof(scena);
	dir_mode_t ledMode = scena[scenaINDEX];
	static rgb_t ws2812_arr[LED_COUNT];

	ws2812_init();

	//for ledMode : RAINBOW, WHITE_X, COLOR_X
	const uint8_t anim_step = 10;
	const uint8_t anim_max = 150;
	const uint32_t cntMode_def = 250;
	rgb_t color1, color2;
	uint8_t step1 = 0, step2 = 0, shift = 0, c_max = anim_max, c_min = 0, ind;
	uint32_t cntMode = cntMode_def;
	bool line_first = true;

	uint32_t msec = 10;

	/* Infinite loop */
	uint64_t wait_pwm = get_hstmr(2);//1 sec

	while (1) {
	  	if (check_hstmr(wait_pwm) && !GoTxDMA) {
	  		msec = 2;//10
	  		switch (ledMode) {
	  			case RAINBOW :
	  				if (line_first) {
	  					color1 = color2 = RGB_SET(anim_max, 0, 0);
	  					step1 = step2 = 0;
	  					line_first = false;
	  				}
	  				color1 = color2;
	  				step1 = step2;
	  				for (ind = 0; ind < LED_COUNT; ind++) {
	  					ws2812_arr[ind] = color1;
	  					if (ind == 1) {
	  						color2 = color1;
	  						step2 = step1;
	  					}
	  					switch (step1) {
	  						case 0:
	  							color1.green += anim_step;
	  							if (color1.green >= anim_max) step1++;
	  						break;
	  						case 1:
	  							color1.red -= anim_step;
	  							if (color1.red == 0) step1++;
	  						break;
	  						case 2:
	  							color1.blue += anim_step;
	  							if (color1.blue >= anim_max) step1++;
	  						break;
	  						case 3:
	  							color1.green -= anim_step;
	  							if (color1.green == 0) step1++;
	  						break;
	  						case 4:
	  							color1.red += anim_step;
	  							if (color1.red >= anim_max) step1++;
	  						break;
	  						case 5:
	  							color1.blue -= anim_step;
	  							if (color1.blue == 0) step1 = 0;
	  						break;
	  					}
	  				}
	  				proc = true;
	  				new_scena = false;
	  				cntMode--;
	  				if (!cntMode) {
	  					cntMode = cntMode_def;
	  					new_scena = true;
	  					line_first = true;
	  				}
	  			break;
	  			case COLOR_UP :
	  			case COLOR_DOWN :
	  			case WHITE_UP :
	  			case WHITE_DOWN :
	  				if (line_first) {
	  					if (ledMode == COLOR_UP) {
	  						shift = 0;
	  						c_min = 0; c_max = anim_max;
	  					} else if (ledMode == COLOR_DOWN) {
	  						shift = LED_COUNT - 1;
	  						c_min = 0; c_max = anim_max;
	  					} else if (ledMode == WHITE_UP) {
	  						shift = 0;
	  						c_min = c_max = anim_max;
	  					} else if (ledMode == WHITE_DOWN) {
	  						shift = LED_COUNT - 1;
	  						c_min = c_max = anim_max;
	  					}
	  					line_first = false;
	  				}
	  				step1 = 0;
	  				color1 = color2 = RGB_SET(0,0,0);
	  				for (ind = 0; ind < LED_COUNT; ind++) {
	  					switch (step1) {
	  						case 0:
	  							color1.red = c_max;
	  							color1.green = color1.blue = c_min;
	  							step1++;
	  						break;
	  						case 1:
	  							color1.green = c_max;
	  							color1.red = color1.blue = c_min;
	  							step1++;
	  						break;
	  						case 2:
	  							color1.blue = c_max;
	  							color1.red = color1.green = c_min;
	  							step1++;
	  						break;
	  						case 3:
	  							color1.red = color1.green = c_max;
	  							color1.blue = c_min;
	  							step1++;
	  						break;
	  						case 4:
	  							color1.red = color1.blue = c_max;
	  							color1.green = c_min;
	  							step1++;
	  						break;
	  						case 5:
	  							color1.green = color1.blue =  c_max;
	  							color1.red = c_min;
	  							step1 = 0;
	  						break;
	  					}
	  					if (ind == shift)
	  						ws2812_arr[ind] = color1;
	  					else
	  						ws2812_arr[ind] = color2;
	  				}
	  				if ((ledMode == COLOR_UP) || (ledMode == WHITE_UP)) {
	  					shift++;
	  					if (shift >= LED_COUNT) shift = 0;
	  				} else if ((ledMode == COLOR_DOWN) || (ledMode == WHITE_DOWN)) {
	  					if (shift > 0)
	  						shift--;
	  					else
	  						shift = LED_COUNT - 1;
	  				}
	  				proc = true;
	  				new_scena = false;
	  				cntMode--;
	  				if (!cntMode) {
	  					cntMode = cntMode_def;
	  					new_scena = true;
	  					line_first = true;
	  				}
	  			break;
	  			case ZERO_ALL :
	  				memset((uint8_t *)&ws2812_arr, 0, sizeof(rgb_t)*LED_COUNT);
	  				new_scena = proc = true;
	  			break;
	  			case COLOR_ALL :
	  				memcpy((uint8_t *)&ws2812_arr, (uint8_t *)&ws2812_const, sizeof(rgb_t)*LED_COUNT);
	  				new_scena = proc = true;
	  			break;
	  		}
	  		if (new_scena) {
	  			new_scena = false;
	  			scenaINDEX++; if (scenaINDEX >= maxSCENA) scenaINDEX = 0;
	  			ledMode = scena[scenaINDEX];
	  		}
	  		if (proc) {
	  			ws2812_setData((void *)&ws2812_arr);
	  			ws2812_start();
	  			if ((ledMode == COLOR_ALL) || (ledMode == ZERO_ALL)) {
	  				cntMode = cntMode_def;
	  				line_first = true;
	  				wait_pwm = get_hstmr(2);//1 sec
	  			} else msec = 20;//30
	  		}
	  	}
	  	//
		osDelay(msec);
	}
  /* USER CODE END StartPwmTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM1) {
	  if (get_hsCounter() & 1) {//second interrupt - 1 sec
		  inc_secCounter();
		  HAL_GPIO_TogglePin(GPIOB, LED1_Pin);//set ON/OFF LED1

		  if (!i2cError) {
			  char buf[64];
			  uint8_t col = ssd1306_calcx(sec_to_str_time(get_secCounter(), buf));
			  sprintf(buf+strlen(buf), "\n\nvolt : %.3f", dataADC);
			  ssd1306_text_xy(buf, col, 2);
		  }
	  }
	  inc_hsCounter();
  }
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
