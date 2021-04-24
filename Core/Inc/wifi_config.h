/*
 * wifi_config.h
 *
 *  Created on: 19 janv. 2017
 *      Author: romain reicher
 */

#ifndef WIFI_CONFIG_H_
#define WIFI_CONFIG_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* WiFi connection Setting ---------------------------------------------------*/

/* Access point connection parametres */
#define WIFI_SSID       	"SFR_8CF0"
#define WIFI_PASSWORD   	"ttyriolakeemanc8aeup"
#define HOST_ADDRESS    	"50.23.124.68"												// Ubidots IP Adress
#define HOST_PORT       	80																		// Port number
#define UBIDOTS_TOKEN   	"BBFF-pIhLief2uvovnVStnKOQn7obqJuLrI"	// Default Token
#define UBIDOTS_DEVICE  	"nucleol4_demo"												// Device API Label

/* Maximum number of trials for WiFi connection */
#define MAX_NUM_TRIAL   10

///* This section can be used to tailor USARTx instance used and associated
//   resources */
//#define USARTx                      		USART1
//#define USARTx_CLK_ENABLE()         		__USART1_CLK_ENABLE()
//#define USARTx_RX_GPIO_CLK_ENABLE() 		__GPIOA_CLK_ENABLE()
//#define USARTx_TX_GPIO_CLK_ENABLE() 		__GPIOA_CLK_ENABLE()
//
//#define USARTx_FORCE_RESET()        		__USART1_FORCE_RESET()
//#define USARTx_RELEASE_RESET()      		__USART1_RELEASE_RESET()
//
///* Definition for USARTx Pins */
//#define USARTx_TX_PIN               		GPIO_PIN_9
//#define USARTx_TX_GPIO_PORT         		GPIOA
//#define USARTx_TX_AF                		GPIO_AF7_USART1
//#define USARTx_RX_PIN               		GPIO_PIN_10
//#define USARTx_RX_GPIO_PORT         		GPIOA
//#define USARTx_RX_AF                		GPIO_AF7_USART1
//
///* Definition for USARTx's NVIC IRQ and IRQ Handlers */
//#define USARTx_IRQn                 		USART1_IRQn
//#define USARTx_IRQHandler           		USART1_IRQHandler
//
///* WiFi module Reset pin definitions */
//#define ESP8266_RST_GPIO_PORT           GPIOA
//#define ESP8266_RST_PIN                 GPIO_PIN_1
//#define ESP8266_RST_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
//
///* Wifi module CH_PD pin definitions */
//#define ESP8266_CHPD_GPIO_PORT					GPIOA
//#define ESP8266_CHPD_PIN								GPIO_PIN_8
//#define ESP8266_CHPD_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()


/* Exported macro ------------------------------------------------------------*/
#define ARRAY_SIZE(array) ((int)(sizeof(array) / sizeof((array)[0])))

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* WIFI_CONFIG_H_ */
