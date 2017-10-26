/**
  *****************************************************************************
  * @title   Main.c
  * @author  Willian
  * @date    12 Out 2017
  * @brief   MainFile
  *******************************************************************************
  */

/* Includes */
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_usart.h"
#include <stdio.h>

// INTERRUPT REQUESTS
// *************************************************************************************

/* Handler of the UART2 module */
void USART2_IRQHandler(void)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char

	USART_SendData(USART2,USART_ReceiveData(USART2));
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
//	uint16_t Data;
//
//			while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
//
//			Data = USART_ReceiveData(USART2); // Collect Char
//
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
//
//			USART_SendData(USART2, Data); // Echo Char



//	 /* RXNE handler */
//	    if(USART_GetITStatus(USART2, USART_IT_RXNE ) != RESET)
//	    {
//	        /* If received 't', toggle LED and transmit 'T' */
//	        if((char)USART_ReceiveData(USART2) == 't')
//	        {
//	            //led_toggle();
//	            USART_SendData(USART2, 'T');
//	            /* Wait until Tx data register is empty, not really
//	             * required for this example but put in here anyway.
//	             */
//	            /*
//	            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
//	            {
//	            }*/
//	        }
//	    }
}


// PERIPHERALS CONFIGURATION
// *************************************************************************************

void RCCconfig()
{
	/* Enable GPIO Port A clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* VEEER */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

/* Configuration of the GPIO*/
void GPIOconfig()
{
	// Board LED Configuration -> PA5
	// USART TX -> PA2; USART RX -> PA3

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure alternate function for the PA2 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	/* Configure alternate function for the PA3 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	/* Configure USART TX and RX*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2  | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Configure the LED of the Board*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

/*Configuration of the UART2 on the board -> TX: PA2; RX: PA3*/
void UARTconfig()
{
	USART_InitTypeDef USART_InitStructure;

	/* USART2 configured as follow:
			- BaudRate = 9600 baud
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Hardware flow control disabled (RTS and CTS signals)
			- Receive and transmit enabled
			*/

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Initialize the UART2 module */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART2 */
	USART_Cmd(USART2, ENABLE);

	/* PARTE NOVA ***********************************/
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	//USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	/* ***********************************************/

	NVIC_EnableIRQ(USART2_IRQn);
}

// FUNCTIONS
// *************************************************************************************

/* Toggle the LED PA5 */
void blink()
{
	long c;
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,SET);
	for (c=0; c<=1000000; c++);
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,RESET);
	for (c=0; c<=1000000; c++);

}


// MAIN
// *************************************************************************************

void main(void)
{

	/*Configure the RCC */
	RCCconfig();

	/*Configure the GPIO */
	GPIOconfig();

	/* Configure the UART */
	UARTconfig();

	USART_SendData(USART2,'T');
	/* Loop until transmit data register is empty */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2,'E');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2,'S');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2,'T');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2,'E');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2,'!');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2,'\n');
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2,'\r');
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);

	while(1)
	{
		blink();
	}

}

/*
 *  ***************************************************************************
 * 	EOF - End Of File
 */
