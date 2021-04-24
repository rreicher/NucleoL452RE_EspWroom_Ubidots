/*
 * uart_term.c
 *
 *  Created on: 6 juil. 2018
 *      Author: Romain REICHER
 */

#include "uart_term.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

extern int vsnprintf(char *s, size_t n, const char * format, va_list arg);

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

__IO bool RxComplete = false;
__IO bool TxComplete = false;

extern void MX_USART2_UART_Init(void);

//*****************************************************************************
//
//! Initialization
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
int16_t InitTerm(uint32_t baudrate)
{
	MX_USART2_UART_Init();

	return 0;	// return 0 is success, -1 if not
}

//*****************************************************************************
//
//! prints the formatted string on to the console
//!
//! \param[in]  format  - is a pointer to the character string specifying the
//!                       format in the following arguments need to be
//!                       interpreted.
//! \param[in]  [variable number of] arguments according to the format in the
//!             first parameters
//!
//! \return count of characters printed
//
//*****************************************************************************
int16_t Report(const char *pcFormat, ...)
{
	int16_t iRet = 0;
	char *pcBuff;
	char *pcTemp;
	int16_t iSize = 256;
	va_list list;

	/* Alocate some memory to buffer */
	pcBuff = (char *)malloc(iSize);

	if(pcBuff == NULL)
	{
		return(-1);
	}

	while(1)
	{
		va_start(list, pcFormat);
		iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
		va_end(list);

		if((iRet > -1) && (iRet < iSize))
		{
			break;
		}
		else
		{
			iSize *= 2;
			if((pcTemp = realloc(pcBuff, iSize)) == NULL)
			{
				Message("Could not reallocate memory\n\r");
				iRet = -1;
				break;
			}
			else
			{
				pcBuff = pcTemp;
			}
		}
	}
	Message(pcBuff);
	free(pcBuff);

	return(iRet);
}

//*****************************************************************************
//
//! Trim the spaces from left and right end of given string
//!
//! \param  pcInput - string on which trimming happens
//!
//! \return length of trimmed string
//
//*****************************************************************************
int16_t TrimSpace(char * pcInput)
{
	size_t size;
	char *endStr;
	char *strData = pcInput;
	char index = 0;

	/* Get lenght of incoming sting */
	size = strlen(strData);

	/* Exit now if doesn't contains any character */
	if(!size)
	{
		return(0);
	}

	endStr = strData + size - 1;
	while((endStr >= strData) && (IS_SPACE(*endStr)))
	{
		endStr--;
	}

	*(endStr + 1) = '\0';

	while(*strData && IS_SPACE(*strData))
	{
		strData++;
		index++;
	}
	memmove(pcInput, strData, strlen(strData) + 1);

	return(strlen(pcInput));
}

//*****************************************************************************
//
//! Get the Command string from UART
//!
//! \param[in]  pucBuffer   - is the command store to which command will be
//!                           populated
//! \param[in]  ucBufLen    - is the length of buffer store available
//!
//! \return Length of the bytes received. -1 if buffer length exceeded.
//!
//*****************************************************************************
int16_t GetCmd(char *pcBuffer, uint16_t uiBufLen)
{
	char cChar = 0;
	int16_t iLen = 0;
	iLen = 0;

	/* Checking the end of Command */
	while(1)
	{
		/* Only One Char at a time */
		cChar = getch();

		/* Handling overflow of buffer */
    if(iLen >= uiBufLen)
    	return(-1);

		/* Copying Data from UART into a buffer */
		if((cChar == '\r') || (cChar == '\n'))
		{
			putch(cChar);
			break;
		}
		else if(cChar == '\b')
		{
			/* Deleting last character when you hit backspace */
			char ch = 0;
			putch(cChar);

			ch = ' ';
			putch(ch);

			if(iLen)
			{
				putch(cChar);
				iLen--;
			}
			else
			{
				ch = '\a';
				putch(ch);
			}
		}
		else
		{
			/* Echo the received character and format it */
			putch(cChar);
			*(pcBuffer + iLen) = cChar;
			iLen++;
		}
	}
	*(pcBuffer + iLen) = '\0';

	return iLen;
}

//*****************************************************************************
//
//! Outputs a character string to the console
//!
//! This function
//!        1. prints the input string character by character on to the console.
//!
//! \param[in]  str - is the pointer to the string to be printed
//!
//! \return none
//!
//! \note If UART_NONPOLLING defined in than Message or UART write should be
//!       called in task/thread context only.
//
//*****************************************************************************
void Message(const char *str)
{
	/* Reset UART Interrupt flag */
	TxComplete = false;
	/* Transmit buffer with UARt in non-blocking mode */
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)str, strlen(str));
	/* Stop Systick Timer Interrupt during Sleep mode */
	HAL_SuspendTick();
	/* Wait for UART interrupt */
	while (!TxComplete){__WFE();}
	HAL_ResumeTick();
}

//*****************************************************************************
//
//! Clear the console window
//!
//! This function
//!        1. clears the console window.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void ClearTerm(void)
{
	Message("\33[2J\r");
}

//*****************************************************************************
//
//! Read a character from the console
//!
//! \param none
//!
//! \return Character
//
//*****************************************************************************
char getch(void)
{
	char ch;

	/* Reset UART Interrupt flag */
	RxComplete = false;
	/* Receive single char with UART in non-blocking mode */
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)&ch, CHAR_NUM);
	/* Stop Systick Timer Interrupt during Sleep mode */
	HAL_SuspendTick();
	/* Wait for UART interrupt */
	while (!RxComplete){__WFE();}
	HAL_ResumeTick();
	return(ch);
}

//*****************************************************************************
//
//! Outputs a character to the console
//!
//! \param[in]  char    - A character to be printed
//!
//! \return none
//
//*****************************************************************************
void putch(char ch)
{
	/* Reset UART Interrupt flag */
	TxComplete = false;
	/* Transmit single char with UART in non-blocking mode */
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, CHAR_NUM);
	/* Stop Systick Timer Interrupt during Sleep mode */
	HAL_SuspendTick();
	/* Wait for UART interrupt */
	while (!TxComplete){__WFE();}
	HAL_ResumeTick();
}


///**
//  * @brief Rx Transfer completed callback.
//  * @param huart UART handle.
//  * @retval None
//  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	RxComplete = true;
//}

/**
  * @brief Tx Transfer completed callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	TxComplete = true;
}


