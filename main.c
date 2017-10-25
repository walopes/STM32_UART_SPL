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

void USART2_IRQHandler(void)
{
	 /* RXNE handler */
	    if(USART_GetITStatus(USART2, USART_IT_RXNE ) != RESET)
	    {
	        /* If received 't', toggle LED and transmit 'T' */
	        if((char)USART_ReceiveData(USART2) == 't')
	        {
	            //led_toggle();
	            USART_SendData(USART2, 'T');
	            /* Wait until Tx data register is empty, not really
	             * required for this example but put in here anyway.
	             */
	            /*
	            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	            {
	            }*/
	        }
	    }
}

void main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	    /* USARTx configured as follow:
	    - BaudRate = 115200 baud
	    - Word Length = 8 Bits
	    - One Stop Bit
	    - No parity
	    - Hardware flow control disabled (RTS and CTS signals)
	    - Receive and transmit enabled
	    */

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	/* Configure USART Tx*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Configure USART RX*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);

	/* PARTE NOVA ***********************************/
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	/* ***********************************************/

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

	NVIC_EnableIRQ(USART2_IRQn);

//
//	long c;
	USART_SendData(USART2,'A');
//			  /* Loop until the end of transmission */
//			  /* The software must wait until TC=1. The TC flag remains cleared during all data
//			     transfers and it is set by hardware at the last frame’s end of transmission*/
//			  //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//
//			  for(c=0;c<1000000;c++);
//
    USART_SendData(USART2,'\n');
    USART_SendData(USART2,'\r');

	while(1)
	{
		/* Output a message on Hyperterminal using printf function */
		  //printf("\n\rUSART Printf Example\n\r");



	}
}

/*
 *  ***************************************************************************
 * 	EOF - End Of File
 */
