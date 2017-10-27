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

uint8_t VAL = 0;

// INTERRUPT REQUESTS
// *************************************************************************************

/* Handler of the UART2 module */
void USART2_IRQHandler(void)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
	uint16_t character = USART_ReceiveData(USART2);
	USART_SendData(USART2,character);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
}

void USART1_IRQHandler(void)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_IDLE) == RESET); // Wait for Char
	uint16_t character = USART_ReceiveData(USART1);
	USART_SendData(USART2,character);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
}


// PERIPHERALS CONFIGURATION
// *************************************************************************************

void RCCconfig()
{
	/* Enable GPIO Port A clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable GPIO Port B clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* Enable USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	/* Enable USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

}

/* Configuration of the GPIO*/
void GPIOconfig()
{
	// Board LED Configuration -> PA5
	// UART2 TX -> PA2; UART2 RX -> PA3
	// UART1 TX -> PA9; UART1 RX -> PA10

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure alternate function for the PA2 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	/* Configure alternate function for the PA3 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	/* Configure alternate function for the PB6*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);

	/* Configure alternate function for the PB7 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

	/* Configure USART1 TX and RX*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6  | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART2 TX and RX (Emulated in PC) */
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

	USART_DeInit(USART1);
	USART_DeInit(USART2);

	/* USART1 configured as follow:
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
	USART_Init(USART1, &USART_InitStructure);

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
	USART_Init(USART2, &USART_InitStructure);

//	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStruct);

	/* Configure Interruption on the USARTx module */
	//USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	/* Enable interrupt of the USARTx*/
	NVIC_EnableIRQ(USART2_IRQn);
	//NVIC_EnableIRQ(USART1_IRQn);

	/* Enable USARTx */
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
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

/* Print the data via UART */
void print(char *string)
{
	while(*string)
	{
		USART_SendData(USART2,*string++);
		/* Loop until transmit data register is empty */
		//while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}
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

	blink();
	blink();
	blink();
	blink();

	print("Teste!\n\r");

	while(1)
	{
		if(VAL)
		{
			VAL = 0;
			blink();
		}
		USART_SendData(USART2,'a');
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USART1,'a');
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}

}

/*
 *  ***************************************************************************
 * 	EOF - End Of File
 */
