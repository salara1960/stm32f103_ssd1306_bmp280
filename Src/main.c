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
#include "ssd1306.h"
#include "bmp280.h"

#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_uart.h"
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

//const char *ver = "ver. 0.1";
//const char *ver = "ver. 1.1";//02.04.2019
//const char *ver = "ver. 1.2";//02.04.2019
//const char *ver = "ver. 1.3";//03.04.2019
//const char *ver = "ver. 1.4";//05.04.2019
//const char *ver = "ver. 1.5";//05.04.2019 put user code in corresponding block
//const char *ver = "ver. 1.6";//06.04.2019 used TIM2 (1 sec. period) with interrupt
//const char *ver = "ver. 1.7";//07.04.2019 used ADC1 with interrupt for measure the supply voltage
//const char *ver = "ver. 1.8";//09.04.2019 used DMA for trasmit data to usart1 (500000 8N1)
//const char *ver = "ver. 1.9";//10.04.2019 used interrupt for receive data from uart1 (echo mode)
const char *ver = "ver. 2.0";//11.04.2019 set date in RTC from GMT epoch time using uart1 (for example : date=1554977111)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

HAL_StatusTypeDef i2cError = HAL_OK;
const uint32_t min_wait_ms = 350;
const uint32_t max_wait_ms = 1000;
result_t sensors = {0.0, 0.0, 0.0};
volatile static uint32_t secCounter = 0;
volatile static float dataADC = 0.0;

const char *_extDate = "date=";
volatile static uint32_t extDate = 0;
static bool setDate = false;
static char RxBuf[MAX_UART_BUF];
volatile uint8_t rx_uk;
volatile uint8_t uRxByte = 0;
//static uint32_t TxLine = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

uint32_t get_secCounter();
char *sec_to_string(uint32_t sec);
void Report(const char *txt, bool addCRLF, bool addTime);
void errLedOn(const char *from);
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED_ERROR, GPIO_PIN_SET);//LEDs OFF

    // start timer2 + interrupt
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);
    //start ADC1 + interrupt
    HAL_ADC_Start_IT(&hadc1);
    //"start" rx_interrupt
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uRxByte, 1);

    HAL_Delay(1000);

    ssd1306_on(true);//screen ON
    if (!i2cError) {
    	ssd1306_init();//screen INIT
    	if (!i2cError) {
    		ssd1306_pattern();//set any params for screen
    		if (!i2cError) {
    			//ssd1306_invert();//set inverse color mode
    			//if (!i2cError)
    				ssd1306_clear();//clear screen
    		}
    	}
    }

    HAL_Delay(1000);

    char stx[256] = {0};
    char toScreen[128] = {0};

    sprintf(stx, "Start main %s", ver);
    Report(stx, true, true);//send data to UART1

    // variable declaration for BMP280 sensor
    uint8_t data_rdx[256] = {0};
    uint8_t reg_id   = 0;
    uint8_t reg_stat = 0;
    uint8_t reg_mode = 0;
    uint8_t reg_conf = 0;
    size_t d_size    = 1;
    int32_t temp, pres, humi = 0;
    char sensorType[32] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    uint32_t wait_sensor = get_tmr(2);//set wait time to 1 sec.

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    	HAL_Delay(10);

    	//------------------------------------------------------------------------

    	if (check_tmr(wait_sensor)) {
    		//
    		wait_sensor = get_tmr(wait_sensor_def);
    		//
    		if (i2c_master_reset_sensor(&reg_id) != HAL_OK) continue;
    		sprintf(stx,"Vcc=%.3f v, ADDR=0x%02X ", dataADC, BMP280_ADDR);
    		sensorType[0] = 0;
    		switch (reg_id) {
    			case BMP280_SENSOR : strcpy(sensorType,"BMP280"); d_size = 6; break;
    			case BME280_SENSOR : strcpy(sensorType,"BME280"); d_size = 8; break;
    				default : strcpy(sensorType,"Unknown chip");
    		}
    		sprintf(stx+strlen(stx),"(%s)", sensorType);
    		if (i2c_master_test_sensor(&reg_stat, &reg_mode, &reg_conf, reg_id) != HAL_OK) {
    			sprintf(stx+strlen(stx)," No ack...try again. (reg_id=0x%02x)", reg_id);
    			Report(stx, true, true);//send data to UART1
    			continue;
    		}
    		reg_stat &= 0x0f;
    		memset(data_rdx, 0, DATA_LENGTH);
    		if (i2c_master_read_sensor(BMP280_REG_PRESSURE, &data_rdx[0], d_size) == HAL_OK) {
    			if (bmp280_readCalibrationData(reg_id) != HAL_OK) {
    				sprintf(stx+strlen(stx)," Reading Calibration Data ERROR");
    			} else {
    				pres = temp = 0;
    				pres = (data_rdx[0] << 12) | (data_rdx[1] << 4) | (data_rdx[2] >> 4);
    				temp = (data_rdx[3] << 12) | (data_rdx[4] << 4) | (data_rdx[5] >> 4);
    				if (reg_id == BME280_SENSOR) humi = (data_rdx[6] << 8) | data_rdx[7];
    				bmp280_CalcAll(&sensors, reg_id, temp, pres, humi);
    				sprintf(stx+strlen(stx)," Press=%.2f mmHg, Temp=%.2f DegC", sensors.pres, sensors.temp);
    				if (reg_id == BME280_SENSOR) sprintf(stx+strlen(stx)," Humidity=%.2f %%rH", sensors.humi);

    				if (i2cError == HAL_OK) {
    					sprintf(toScreen, "mmHg : %.2f\nDegC : %.2f", sensors.pres, sensors.temp);
    					if (reg_id == BME280_SENSOR) sprintf(toScreen+strlen(toScreen),"\nHumi:%.2f %%rH", sensors.humi);
    					ssd1306_text_xy(toScreen, 1, 6);//send string to screen
    				}

    			}
    		} else strcat(stx,"Read sensor error");
    		Report(stx, true, true);//send data to UART1
    		//
    	}

    	//------------------------------------------------------------------------

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	secCounter = 0;
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 500000;//115200;
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
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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

//-----------------------------------------------------------------------------
void set_Date(time_t epoch)
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;

	gmtime_r(&epoch, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month = ts.tm_mon + 1;
	sDate.Date = ts.tm_mday;
	sDate.Year = ts.tm_year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
	else {
		sTime.Hours = ts.tm_hour;
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
uint32_t s = sec;

	uint32_t day = s / (60 * 60 * 24);
	s %= (60 * 60 * 24);

    uint32_t hour = s / (60 * 60);
    s %= (60 * 60);

    uint32_t min = s / (60);
    s %= 60;

    return (sprintf(stx, "%lu.%02lu:%02lu:%02lu", day, hour, min, s));
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
//----------------------------------------------------------------------------------------
//  if (return pointer != NULL) you must free this pointer after used
char *sec_to_string(uint32_t sec)
{
	char *stx = (char *)calloc(1, 32);
	if (stx) {
		if (!setDate) {//no valid date in RTC
			uint32_t day = sec / (60 * 60 * 24);
			sec %= (60 * 60 * 24);
			uint32_t hour = sec / (60 * 60);
			sec %= (60 * 60);
			uint32_t min = sec / (60);
			sec %= 60;
			sprintf(stx, "%03lu.%02lu:%02lu:%02lu", day, hour, min, sec);
		} else {//in RTC valid date (epoch time)
			RTC_TimeTypeDef sTime;
			RTC_DateTypeDef sDate;
			if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
			else {
				if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
				else {
					sprintf(stx,"%04u.%02u.%02u %02u:%02u:%02u",
							sDate.Year + 1900, sDate.Month, sDate.Date,
							sTime.Hours, sTime.Minutes, sTime.Seconds);
				}
			}
		}
	}

    return stx;
}
//----------------------------------------------------------------------------------------
//	Out to UART1 data from buffer (like printf)
//     txt - string for send to uart
//     addCRLF - flag append or not "\r\n" to data before sending
//     addTime - flag insert or not TickCount before data
void Report(const char *txt, bool addCRLF, bool addTime)
{
	if (!txt)  return;

	uint16_t len = strlen(txt);
	if (addCRLF) len += 2;
	if (addTime) len += 32;
	char *buf = (char *)calloc(1, len + 1);//get buffer for data in heap memory
	if (!buf) return;

	if (addTime) {
		uint32_t ep;
		if (!setDate) ep = get_secCounter();
				 else ep = extDate;
		char *stime = sec_to_string(ep);
		if (stime) {
			strcpy(buf, stime);
			strcat(buf, " | ");
			free(stime);//release memory by pointer stime
		}
	}
	strcat(buf, txt);
	if (addCRLF) strcat(buf, "\r\n");
	len = strlen(buf);

	// send data to UART1 via DMA1_Channel4
	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) != HAL_OK) errLedOn(NULL);

	free(buf);

}
//------------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);//LED ON
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);//LED OFF
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);//LED ON

	if (from) {
		char *stx = (char *)calloc(1, strlen(from) + 16);
		if (stx) {
			sprintf(stx, "Error in '%s'", from);
			Report(stx, true, true);
			free(stx);
		}
	}
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		dataADC = ((float)HAL_ADC_GetValue(hadc)) * 3.3 / 4096;
	}
}
//------------------------------------------------------------------------------------------
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		TxLine++;
		char stx[16];
		ssd1306_calcx(sprintf(stx, "- %lu -", TxLine));
		ssd1306_text_xy(stx, ssd1306_calcx(sprintf(stx, "- %lu -", TxLine)), 1);
	}
}
*/
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
			Report(RxBuf, false, false);
			memset(RxBuf, 0, MAX_UART_BUF);
			rx_uk = 0;
		} else rx_uk++;

		HAL_UART_Receive_IT(huart, (uint8_t *)&uRxByte, 1);

	}
}
//------------------------------------------------------------------------------------------
/* USER CODE END 4 */

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
  if (htim->Instance == TIM2) {
	  if (!i2cError) {
		  char buf[64];
		  uint8_t col = ssd1306_calcx(sec_to_str_time(get_secCounter(), buf));
		  sprintf(buf+strlen(buf), "\n\nvolt : %.3f", dataADC);
		  ssd1306_text_xy(buf, col, 2);
	  }
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
