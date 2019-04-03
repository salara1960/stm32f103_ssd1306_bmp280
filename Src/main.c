#include "ssd1306.h"
#include "bmp280.h"

//-----------------------------------------------------------------------------

//const char *ver = "ver. 0.1";
//const char *ver = "ver. 1.1";//02.04.2019
const char *ver = "ver. 1.2";//02.04.2019


I2C_HandleTypeDef hi2c2;
HAL_StatusTypeDef i2cError = HAL_OK;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

const uint32_t min_wait_ms = 350;
const uint32_t max_wait_ms = 1000;

result_t sensors = {0.0, 0.0, 0.0};

static uint32_t msCounter = wait_tick_def;
static uint32_t secCounter = 0;

//-----------------------------------------------------------------------------

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


//-----------------------------------------------------------------------------

int sec_to_str_time(uint32_t sec, char *stx)
{
long day, min, hour, seconda;

	day = sec / (60 * 60 * 24);
	sec = sec % (60 * 60 * 24);

    hour = sec / (60 * 60);
    sec = sec % (60 * 60);

    min = sec / (60);
    sec = sec % 60;

    seconda = sec;

    return (sprintf(stx, "%lu.%02lu:%02lu:%02lu", day, hour, min, seconda));
}

//-----------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return (secCounter);
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------

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
	if (htim->Instance == TIM4) {

		HAL_IncTick();

		//-------------   LED ON/OFF and show tickCounter to Screen   -----------
		if (msCounter) msCounter--;
		if (!msCounter) {
			inc_secCounter();
			msCounter = wait_tick_def;
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, (!HAL_GPIO_ReadPin(GPIOB, LED1_Pin)) & 1);//set ON/OFF LED1

			char buf[32];
			ssd1306_text_xy(buf, ssd1306_calcx(sec_to_str_time(get_secCounter(), buf)), 2);//send string to Screen
		}
		//---------------------------------------------------------------------

	}
}

//------------------------------------------------------------------------------------------


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	while (1) {}
}

//------------------------------------------------------------------------------------------

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

}

//------------------------------------------------------------------------------------------

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin   = LED1_Pin | LED2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

//------------------------------------------------------------------------------------------

static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  // Initialize RTC Only
  hrtc.Instance          = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut       = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler();

  // Initialize RTC and set the Time and Date
  sTime.Hours   = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month   = RTC_MONTH_JANUARY;
  DateToUpdate.Date    = 0x1;
  DateToUpdate.Year    = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();

}

//------------------------------------------------------------------------------------------

static void MX_I2C2_Init(void)
{

  hi2c2.Instance             = I2C2;
  hi2c2.Init.ClockSpeed      = 400000;
  hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1     = 0;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();

}

//------------------------------------------------------------------------------------------

static void MX_USART1_Init(void)
{

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

}

//----------------------------------------------------------------------------------------
//	Out to UART1 data from buffer (like printf)
//     txt - buffer with data for send to uart
//     len - total bytes for sending
//     addCRLF - flag append or not "\r\n" to data before sending
//     addTime - flag insert or not TickCount before data
void Report(const char *txt, bool addCRLF, bool addTime)
{
	if (!txt)  return;

	uint16_t len = strlen(txt);
	if (addCRLF) len += 2;
	if (addTime) len += 16;
	if (len > 255) len = 255;
	char *buf = (char *)calloc(1, len + 1);//get buffer for data in heap memory
	if (!buf) return;

	//create buffer with data for sending to UART1 and USB
	if (addTime) sprintf(buf, "[%08lu] | ", get_secCounter());
	strcat(buf, txt);
	if (addCRLF) strcat(buf, "\r\n");
	len = strlen(buf);
#ifdef SET_UART
	// send data to UART1
	if (HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, max_wait_ms) != HAL_OK) errLedOn(NULL);
#endif

	free(buf);//release buffer's memory

}

//------------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);//LED ON
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_SET);//LED OFF
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, LED_ERROR, GPIO_PIN_RESET);//LED ON

	if (from) {
		char *stx = (char *)calloc(1, strlen(from) + 32);
		if (stx) {
			sprintf(stx,"Error in %s function\r\n", from);
			Report(stx, false, true);
			free(stx);
		}
	}
}

//-----------------------------------------------------------------------------

uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}

//----------------------------------------------------------------------------------------

bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************

int main(void)
{
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED_ERROR, GPIO_PIN_SET);//LEDs OFF
    MX_RTC_Init();
    MX_I2C2_Init();
#ifdef SET_UART
    MX_USART1_Init();
#endif

    char stx[256] = {0};
    char toScreen[128] = {0};

#ifdef DISPLAY
    ssd1306_on(true);//screen ON
    if (!i2cError) {
    	ssd1306_init();//screen INIT
    	if (!i2cError) ssd1306_pattern();//set any params for screen
    }

    sprintf(stx, "Start main %s", ver);
    if (!i2cError) {
    	ssd1306_invert();//set inverse color mode
    	if (!i2cError) ssd1306_clear();//clear screen
    }
    Report(stx, true, true);//send data to UART1
#endif

#ifdef SET_BMP
    // variable declaration for BMP280 sensor
    uint8_t data_rdx[256] = {0};
    uint8_t reg_id   = 0;
    uint8_t reg_stat = 0;
    uint8_t reg_mode = 0;
    uint8_t reg_conf = 0;
    uint8_t col      = 0;
    size_t d_size    = 1;
    int32_t temp, pres, humi = 0;
    char sensorType[64] = {0};

    uint32_t wait_sensor = get_tmr(wait_sensor_def);//set wait time to 1 sec.
#endif

    //----------------------     MAIN LOOP    -----------------------------------

    while (1) {

    	//------------------------------------------------------------------------
#ifdef SET_BMP
    	if (check_tmr(wait_sensor)) {
    		wait_sensor = get_tmr(wait_sensor_def);
    		//
    		if (i2c_master_reset_sensor(&reg_id) != HAL_OK) continue;
    		sprintf(stx,"ADDR=0x%02X ", BMP280_ADDR);
    		sensorType[0] = 0;
    		switch (reg_id) {
    			case BMP280_SENSOR : strcpy(sensorType,"BMP280"); d_size = 6; break;
    			case BME280_SENSOR : strcpy(sensorType,"BME280"); d_size = 8; break;
    				default : strcpy(sensorType,"Unknown chip");
    		}
    		sprintf(stx+strlen(stx),"(%s)", sensorType);
    		if (i2c_master_test_sensor(&reg_stat, &reg_mode, &reg_conf, reg_id) != HAL_OK) {
    			sprintf(stx+strlen(stx)," No ack...try again. (reg_id=0x%02x)\r\n", reg_id);
    			Report(stx, false, true);//send data to UART1
    			continue;
    		}
    		reg_stat &= 0x0f;
    		memset(data_rdx, 0, DATA_LENGTH);
    		if (i2c_master_read_sensor(BMP280_REG_PRESSURE, &data_rdx[0], d_size) == HAL_OK) {
    			if (bmp280_readCalibrationData(reg_id) != HAL_OK) {
    				sprintf(stx+strlen(stx)," Reading Calibration Data ERROR\r\n");
    			} else {
    				pres = temp = 0;
    				pres = (data_rdx[0] << 12) | (data_rdx[1] << 4) | (data_rdx[2] >> 4);
    				temp = (data_rdx[3] << 12) | (data_rdx[4] << 4) | (data_rdx[5] >> 4);
    				if (reg_id == BME280_SENSOR) humi = (data_rdx[6] << 8) | data_rdx[7];
    				bmp280_CalcAll(&sensors, reg_id, temp, pres, humi);
    				sprintf(stx+strlen(stx)," Press=%.2f mmHg, Temp=%.2f DegC", sensors.pres, sensors.temp);
    				if (reg_id == BME280_SENSOR) sprintf(stx+strlen(stx)," Humidity=%.2f %%rH", sensors.humi);
    				strcat(stx,"\r\n");
#ifdef DISPLAY
    				if (i2cError == HAL_OK) {
    					col = ssd1306_calcx(strlen(sensorType));
    					sprintf(toScreen, "%s\n\nmmHg : %.2f\nDegC : %.2f", sensorType, sensors.pres, sensors.temp);
    					if (reg_id == BME280_SENSOR) sprintf(toScreen+strlen(toScreen),"\nHumi:%.2f %%rH", sensors.humi);
    					ssd1306_text_xy(toScreen, col, 4);//send string to screen
    				}
#endif
    			}
    		} else strcat(stx,"Read sensor error\r\n");
    		Report(stx, false, true);//send data to UART1
    		//
    	}
#endif
    	//------------------------------------------------------------------------
    }//end of while(1)

}//end of main



//****************************************************************************************
//****************************************************************************************
//****************************************************************************************
