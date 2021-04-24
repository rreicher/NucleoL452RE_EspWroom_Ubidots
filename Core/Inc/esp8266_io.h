/*
 * esp8266_io.h
 *
 *  Created on: 19 janv. 2017
 *      Author: romain reicher
 */

#ifndef ESP8266_IO_H_
#define ESP8266_IO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define DEFAULT_TIME_OUT                 3000 /* in ms */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t ESP8266_IO_Init(void);
void ESP8266_IO_DeInit(void);

int8_t ESP8266_IO_Send(uint8_t* Buffer, uint32_t Length);
int32_t ESP8266_IO_Receive(uint8_t* Buffer, uint32_t Length);

#ifdef __cplusplus
}
#endif



#endif /* ESP8266_IO_H_ */

