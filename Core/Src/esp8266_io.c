/*
 * esp8266_io.c
 *
 *  Created on: 19 janv. 2017
 *      Author: romain reicher
 */


/* Includes ------------------------------------------------------------------*/
#include "wifi_config.h"
#include "esp8266_io.h"
#include "main.h"
#include <stdbool.h>

/* Private define ------------------------------------------------------------*/
#define RING_BUFFER_SIZE			(1024 * 2)

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
	uint8_t  data[RING_BUFFER_SIZE];
	uint16_t tail;
	uint16_t head;
}RingBuffer_t;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RingBuffer_t WiFiRxBuffer;
extern UART_HandleTypeDef huart1;
extern __IO bool RxComplete;


/* Private function prototypes -----------------------------------------------*/
static void WIFI_Handler(void);


/* Private functions ---------------------------------------------------------*/

/**
 * @brief  ESP8266 IO Initalization.
 *         This function inits the UART interface to deal with the esp8266,
 *         then starts asynchronous listening on the RX port.
 * @param None
 * @retval 0 on success, -1 otherwise.
 */
int8_t ESP8266_IO_Init(void)
{
	/* Set the WiFi USART configuration parameters */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if(HAL_UART_Init(&huart1) != HAL_OK)
	{
		return -1;
	}

	/* Once the WiFi UART is intialized, start an asynchrounous recursive
     listening. the HAL_UART_Receive_IT() call below will wait until one char is
     received to trigger the HAL_UART_RxCpltCallback(). The latter will recursively
     call the former to read another char.  */
	WiFiRxBuffer.head = 0;
	WiFiRxBuffer.tail = 0;

	HAL_UART_Receive_IT(&huart1, (uint8_t *)&WiFiRxBuffer.data[WiFiRxBuffer.tail], 1);

	return 0;
}

/**
 * @brief  ESP8266 IO Deinitialization.
 *         This function Deinits the UART interface of the ESP8266. When called
 *         the esp8266 commands can't be executed anymore.
 * @param  None.
 * @retval None.
 */
void ESP8266_IO_DeInit(void)
{
	/* Reset USART configuration to default */
	HAL_UART_DeInit(&huart1);
}

/**
 * @brief  Send Data to the ESP8266 module over the UART interface.
 *         This function allows sending data to the  ESP8266 WiFi Module, the
 *          data can be either an AT command or raw data to send over
             a pre-established WiFi connection.
 * @param pData: data to send.
 * @param Length: the data length.
 * @retval 0 on success, -1 otherwise.
 */
int8_t ESP8266_IO_Send(uint8_t* pData, uint32_t Length)
{
	/* Unlike the ESP8266_IO_Receive(), the ESP8266_IO_Send() is using a blocking call
     to ensure that the AT commands were correctly sent. */
	if (HAL_UART_Transmit(&huart1, (uint8_t*)pData, Length, DEFAULT_TIME_OUT) != HAL_OK)
	{
		return -1;
	}

	return 0;
}

/**
 * @brief  Receive Data from the ESP8266 module over the UART interface.
 *         This function receives data from the  ESP8266 WiFi module, the
 *         data is fetched from a ring buffer that is asynchonously and continuously
            filled with the received data.
 * @param  Buffer: a buffer inside which the data will be read.
 * @param  Length: the Maximum size of the data to receive.
 * @retval int32_t: the actual data size that has been received.
 */
int32_t ESP8266_IO_Receive(uint8_t* Buffer, uint32_t Length)
{
	uint32_t ReadData = 0;

	/* Loop until data received */
	while (Length--)
	{
		uint32_t tickStart = HAL_GetTick();
		do
		{
			if(WiFiRxBuffer.head != WiFiRxBuffer.tail)
			{
				/* serial data available, so return data to user */
				*Buffer++ = WiFiRxBuffer.data[WiFiRxBuffer.head++];
				ReadData++;

				/* check for ring buffer wrap */
				if (WiFiRxBuffer.head >= RING_BUFFER_SIZE)
				{
					/* Ring buffer wrap, so reset head pointer to start of buffer */
					WiFiRxBuffer.head = 0;
				}
				break;
			}
		}while((HAL_GetTick() - tickStart ) < DEFAULT_TIME_OUT);
	}

	return ReadData;
}

/**
 * @brief  Rx Callback when new data is received on the UART.
 * @param  UartHandle: Uart handle receiving the data.
 * @retval None.
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if (UartHandle->Instance == USART1)
	{
		/* If ring buffer end is reached reset tail pointer to start of buffer */
		if(++WiFiRxBuffer.tail >= RING_BUFFER_SIZE)
		{
			WiFiRxBuffer.tail = 0;
		}

		HAL_UART_Receive_IT(UartHandle, (uint8_t *)&WiFiRxBuffer.data[WiFiRxBuffer.tail], 1);
	}
	else if (UartHandle->Instance == USART2)
	{
		RxComplete = true;
	}
}

/**
 * @brief  Function called when error happens on the UART.
 * @param  UartHandle: Uart handle receiving the data.
 * @retval None.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	/* Call  the WIFI_Handler() to deinitialize the UART Interface. */
	WIFI_Handler();
}

/**
 * @brief  Handler to deinialize the ESP8266 UART interface in case of errors.
 * @param  None
 * @retval None.
 */
static void WIFI_Handler(void)
{
	HAL_UART_DeInit(&huart1);

	while(1)
	{
	}
}


