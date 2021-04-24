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
#include "wifi_config.h"
#include "esp8266.h"
#include "uart_term.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APPLICATION_VERSION 	"1.0.0"
#define APP_NAME              "Ubidots"
#define LPTIM_PERIOD					0xFFFF

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
ESP8266_ConnectionInfoTypeDef ConnectionInfo = {0};
ESP8266_StatusTypeDef Status = {0};
uint32_t Trial = 0;
static uint8_t IpAddress[15];
static char sendBuffer[256];
static char message[64];
static uint16_t dataSend = 100;

__IO bool butHitDone = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPTIM1_Init(void);
/* USER CODE BEGIN PFP */
static void DisplayBanner(char * AppName);
static bool Wifi_Connect(void);
static bool Wifi_Disconnect(void);
static void Ubidots_Send(uint16_t value, bool debug);

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
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */

  /**> Init Debug Terminal */
  InitTerm(115200);
  /**> Display debug information */
  DisplayBanner(APP_NAME);

  bool ret = false;

  /**> Start LPTIM1 and generate interrupt every 64sec to post data
   	*	 LPTIM1 is clocked by LSE and Divided by 32
   	*/
  HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIM_PERIOD);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (butHitDone)
  	{
  		butHitDone = false;
  		/**> Stop LPTimer counter and Interrups */
  		HAL_LPTIM_Counter_Stop_IT(&hlptim1);
  		UART_PRINT("User Button Pressed or LPTimer1 Compare match\r\n");

  		/* Limit data format to 3 digits (ie 999) otherwise need to modify the lenght of string */
			if(dataSend > 500)
				dataSend = 100;
			else
				dataSend += 10;

			UART_PRINT("Data to send %d\r\n", dataSend);

			/**> Connect to Wifi AP */
  	  ret = Wifi_Connect();
  	  /**> Send data to Ubidots */
  	  Ubidots_Send(dataSend, true);

  	  /**> Blink LED4 when datas is send */
  	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  	  HAL_Delay(400);
  	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  	  /**> Disconnect from Wifi AP */
  	  ret = Wifi_Disconnect();

  	  /**> Re Start LPTIM1 and generate interrupt every 64sec to post data */
  	  HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIM_PERIOD);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_ENDOFPERIOD;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WIFI_RST_Pin|LD4_Pin|WIFI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_USER_Pin */
  GPIO_InitStruct.Pin = B1_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_RST_Pin LD4_Pin WIFI_EN_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin|LD4_Pin|WIFI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void DisplayBanner(char * AppName)
{
	UART_PRINT("\n\n\n\r");
	UART_PRINT("\t\t *************************************************\n\r");
	UART_PRINT("\t\t     NucleoL452RE %s Application       \n\r", AppName);
	UART_PRINT("\t\t *************************************************\n\r");
	UART_PRINT("\n\n\n\r");
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_USER_Pin)
  {
  	butHitDone = true;
  }
}

/**
  * @brief  Autoreload match callback in non-blocking mode.
  * @param  hlptim LPTIM handle
  * @retval None
  */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	butHitDone = true;
}

/**
  * @brief  Connect ESP device to Wifi AP
  * @param  None
  * @retval True or false
  */
static bool Wifi_Connect(void)
{
  /* ESP8266-01 Initialization */
	UART_PRINT("Initialize wifi module\r\n");

	Status = ESP8266_Init();

	if (Status != ESP8266_OK)
	{
		UART_PRINT("Initializing Failed!\r\n");
		return false;
	}
	else
	{
		UART_PRINT("Initializing Success!\r\n");
	}

	/**> Join Access Point defined in wifi_config.h */
	UART_PRINT("Joining Access Point ' "WIFI_SSID" ' ...\r\n");

	while (ESP8266_JoinAccessPoint((uint8_t *)WIFI_SSID,
																 (uint8_t *)WIFI_PASSWORD) != ESP8266_OK)
	{
		UART_PRINT("Retrying ( %d) to join Access Point "WIFI_SSID"\r\n", (uint16_t)++Trial);
		if (Trial == MAX_NUM_TRIAL) break;
	}

	if (Trial == MAX_NUM_TRIAL)
	{
		UART_PRINT("Joining Access Point ' "WIFI_SSID" ' Failed!\r\n");
		return false;
	}
	else
	{
		/**> Reset the IP adress field to 0 */
		memset(IpAddress, '\0', 15);
		/**> Get IP Adress */
		ESP8266_GetIPAddress(ESP8266_STATION_MODE, IpAddress);
		UART_PRINT("OK!\r\nGot IP Adress: %s\r\n", (char*)IpAddress);
	}
	return true;
}

/**
  * @brief  Disconnect ESP device to Wifi AP
  * @param  None
  * @retval True or false
  */
static bool Wifi_Disconnect(void)
{
	/**> Quit Wifi access point */
	Status = ESP8266_QuitAccessPoint();

	if (Status != ESP8266_OK)
	{
		UART_PRINT("Quit access point ' "WIFI_SSID" ' Failed!\r\n");
		return false;
	}
	else
	{
		UART_PRINT("Quit access point ' "WIFI_SSID" ' Success!\r\n");
		return true;
	}
}

/**
  * @brief  Send data to Ubitots cloud server
  * @param1 Value to send
  * @param2 Terminal debug
  * @retval None
  */
static void Ubidots_Send(uint16_t value, bool debug)
{
	/* Connect to Thingspeak */
	Trial = 0;
	/* Fill IP adress with 0 */
	memset(&ConnectionInfo, '\0', sizeof (ESP8266_ConnectionInfoTypeDef));

	/* Configure connection structure */
	ConnectionInfo.connectionType = ESP8266_TCP_CONNECTION;		// TCP Connection
	ConnectionInfo.ipAddress = (uint8_t *)HOST_ADDRESS;				// Thingspeak ip adress
	ConnectionInfo.isServer = ESP8266_FALSE;
	ConnectionInfo.port = HOST_PORT;													// Port 80 http

	/* Wait for communication establishment */
	while (ESP8266_EstablishConnection(&ConnectionInfo) != ESP8266_OK)
	{
		if(debug)
		{
			UART_PRINT("Retrying( %d ) to connect to %s:%d \r\n", (uint16_t)++Trial, HOST_ADDRESS, HOST_PORT);
		}
		HAL_Delay(1000);

		if (Trial == MAX_NUM_TRIAL)
		{
			break;
		}
	}

	if (Trial == MAX_NUM_TRIAL)
	{
		if(debug)
		{
			UART_PRINT("Connecting to %s:%d  Failed!\r\n", HOST_ADDRESS, HOST_PORT);
		}
		if(debug)
		{
			UART_PRINT("Quitting Access Point...\r\n");
		}

		/* Call the error Handler */
		Error_Handler();
	}
	else
	{
		if(debug) UART_PRINT("OK!\r\n");
		HAL_Delay(1000);
	}

	/* Construct content of HTTP command */
	sprintf(message, "{\"count\": %d}", value);
	if(debug) UART_PRINT("Content Length = %d\r\n", (int)strlen(message));

	/* Construct HTTP command to send */
	sprintf(sendBuffer, "POST /api/v1.6/devices/%s/?token=%s HTTP/1.1\r\nHost: things.ubidots.com\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s", UBIDOTS_DEVICE, UBIDOTS_TOKEN, (int)strlen(message),message);
	if(debug) UART_PRINT("HTTP command %s\r\n", sendBuffer);
	HAL_Delay(1000);

	/* Data ready to send to Thingspeak */
	Status = ESP8266_SendData((uint8_t *)sendBuffer, strlen((char *)sendBuffer));

	if(Status != ESP8266_OK)
	{
		if(debug) UART_PRINT("Sending Datas Failed!\r\n");
		Error_Handler();
	}
	else
	{
		if(debug)
		{
			UART_PRINT("Sending %d to Ubidots\r\n", value);
			UART_PRINT("Sending Datas Success!\r\n");
		}
	}

	/* Close Thingspeak connection */
	Status = ESP8266_CloseConnection(0);

	if(Status != ESP8266_OK)
	{
		if(debug) UART_PRINT("Close Connection Failed!\r\n");
		Error_Handler();
	}
	else
	{
		if(debug) UART_PRINT("Close Connection Success!\r\n");
	}
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
