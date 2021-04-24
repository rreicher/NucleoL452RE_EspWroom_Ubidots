/*
 * uart_term.h
 *
 *  Created on: 6 juil. 2018
 *      Author: Romain REICHER
 */

#ifndef UART_TERM_H_
#define UART_TERM_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "stm32l4xx_hal.h"

/* Defines */
#define UART_PRINT   Report
#define DBG_PRINT    Report
#define ERR_PRINT(x) Report("Error [%d] at line [%d] in function [%s] \n\r",\
																									 x, __LINE__, __FUNCTION__)
#define UART_GET     GetCmd
#define IS_SPACE(x)	 (x == 32 ? 1 : 0)
#define CHAR_NUM		 1U

/* API */
int16_t InitTerm(uint32_t baudrate);
int16_t Report(const char *pcFormat, ...);
int16_t TrimSpace(char *pcInput);
int16_t GetCmd(char *pcBuffer, uint16_t uiBufLen);
void Message(const char *str);
void ClearTerm(void);
char getch(void);
void putch(char ch);

#ifdef __cplusplus
}
#endif

#endif /* UART_TERM_H_ */
