/**
// *************************************************************************************
  * @title   Main.c
  * @author  Willian
  * @date    12 Out 2017
  * @brief   MainFile
// *************************************************************************************
 */


/* INCLUDES */
// *************************************************************************************
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "DrvGps_NEO_6M.h"

/* DEFINES */
// *************************************************************************************
#define SENTENCE_SIZE 80
#define NMEA_SENT_SIZE 5

/* VARIABLES */
// *************************************************************************************
char BUFFER[NMEA_SENT_SIZE];
uint8_t count; // No main -> count=0
char newCharacter;

uint8_t sentCome;
	// Se 0 -> ainda nao foi recebido caracter novo
	// Se 1 -> novo char recebido

uint8_t sentHold;
	// Se 0-> uma nova escrita eh permitida para BUFFER
	// Se 1-> nao sera escrito mais nada em BUFFER, pois essa sentenca deve ir para o parse()

uint8_t sentenceStatus;
	// Se 0-> disponivel para escrever normalmente em BUFFER
 	// Se 1-> atualmente escrevendo em BUFFER
	// Se 2-> A sentenca atual esta sendo tratada pelo GPSHandler(), assim, BUFFER nao pode ser escrito
	// Se 3-> Foi detectado que BUFFER contem informacoes sobre GPRMC - continua escrevendo normal
	// Se 4-> Foi finalizada a escrita em BUFFER - Agora a etapa e tratar em GPSHandler()
uint8_t comma;
uint8_t Hour, Minutes, Seconds;

// INTERRUPT REQUESTS
// *************************************************************************************

/* Handler of the UART2 module */
void USART2_IRQHandler(void)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
//	uint16_t character = USART_ReceiveData(USART2);
//	USART_SendData(USART2,character);
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty
//
	newCharacter = USART_ReceiveData(USART2);
	sentCome=1;
}

void USART1_IRQHandler(void)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); // Wait for Char
	newCharacter = USART_ReceiveData(USART1);
	sentCome=1;

//	char character = USART_ReceiveData(USART1);
//	USART_SendData(USART2,character);
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty

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

	/* Configure alternate function for the USART1 TX*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

	/* Configure alternate function for the USART1 RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	/* Configure USART1 TX and RX*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9  | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

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

/*Configuration of the UART1 on the board -> TX: PA9; RX: PA10*/
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
	/* Enable USARTx */
	USART_Cmd(USART1, ENABLE);

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
	/* Enable USARTx */
	USART_Cmd(USART2, ENABLE);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

	/* Enable interrupt of the USARTx*/
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
}

// FUNCTIONS
// *************************************************************************************


/* Toggle the LED of the board - PA5 */
void blink()
{
	long c;
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,SET);
	for (c=0; c<=1000000; c++);
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,RESET);
	for (c=0; c<=1000000; c++);

}

///* Print the data via UART */
//void print(char *string)
//{
//	while(*string)
//	{
//		USART_SendData(USART2,*string++);
//		/* Loop until transmit data register is empty */
//		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//	}
//}
//
//
//void verifySentence()
//{
//
//	if(sentenceStatus==1) // The sentence comming needs to be verified if it is of GPRMC type
//	{
//
//		if(count==5)
//		{
//			if(newCharacter == ',') comma++; // To get the first comma
//
//			//if(!strcmp(BUFFER,"GPTXT")) // A GPRMC sentence is comming -> Not working
//			if(BUFFER[0]=='G' && BUFFER[1]=='P' && BUFFER[2]=='R' && BUFFER[3]=='M' && BUFFER[4]=='C')
//			{
//				sentenceStatus = 2;
//				count=0;
//				blink();
//
//				//print("GPRMC");
//				print(BUFFER);
//				//print("END\n\r");
//			}else sentenceStatus=0;
//		}else // count < 5
//		{
//			BUFFER[count++] = newCharacter;
//			blink();
//		}
//
//	}else if(sentenceStatus==2) // The current sentence is of type GPRMC
//	{
//
//		if(comma==1) // Verification of the time
//		{
//			//checkTime();
//			if(newCharacter >= 48 && newCharacter <= 57) //Valid number
//			{
//				if(count == 1) Hour = newCharacter*10;
//				else if(count==2) Hour += newCharacter;
//				else if(count==3) Minutes = newCharacter*10;
//				else if(count==4) Minutes += newCharacter;
//				else if(count==5) Seconds = newCharacter*10;
//				else if(count==6) Seconds += newCharacter;
//			}
//
//			count++;
//
//		}else if(comma==2) //Verification of the validity of the sentence received
//		{
//			// A: Active
//			// V: Void
//			if(!(newCharacter == 'A'))
//			{
//				print("Invalido!\n\r");
//				sentenceStatus=0;
//			}
//
//		}else if(comma==3 || comma==4) // Latitude
//		{
//			;
//		}else if(comma==5 || comma==6) // Longitude
//		{
//			;
//		}
//
//		// Now that the sentence is GPRMC type, the position of the commas will
//		// indicate the desired information
//		if(newCharacter == ',')
//			comma++;
//
//
//
//	}
//
//	// !sentenceStatus -> The sentence is useless or is a new sentence
//	if(newCharacter == '$') // A new sentence is beginning
//	{
//		sentenceStatus = 1;
//		count=0;
//		comma=0;
//		blink();
//	}
//
//}
//
//
//	/*
//		//print("False\n\r");
//		if(!sentenceStatus){// and the buffer is ready to save data
//			sentenceStatus++;
//			count=0;
//			//print(BUFFER);
//		}else if(sentenceStatus==3)
//		{
//			sentenceStatus=4; // The sentence is ready to be parsed!
//			print("True\n\r");
//			USART_SendData(USART2,count+64);
//			//* Loop until transmit data register is empty * /
//			while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//			print(BUFFER);
//			blink();
//		}else
//			sentenceStatus=2; // The previous sentece is being parsed - Cannot write in BUFFER[]
//	}
//	if(sentenceStatus==1)
//	{
//		BUFFER[count++] = newCharacter;
//		if(count>=6) // Verify if this String is $GPRMC
//		{
//			if(BUFFER[1] == 'G' && BUFFER[2] == 'P' && BUFFER[3] == 'R' && BUFFER[4] == 'M' && BUFFER[5] == 'C')
//			{
//				sentenceStatus=3;
//				print("Verificado\n\r");
//			}else
//				sentenceStatus=0;
//		}
//	}
//	else if(sentenceStatus==3)
//	{
//		BUFFER[count++] = newCharacter;
//	}
//
//	//return sentenceStatus;
//	//print("Oi!\n\r");
//	//USART_SendData(USART2,newCharacter);
//	//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
//
//	*/
//
//
//
///* Used to parse the NMEA sentences in data */
////void GPSHandler()
////{
////  checkST=0;
////  uint8_t substr[6] = memcpy();//substring para pegar o $GPRMC
////  if(strcmp(substr,"$GPRMC"))
////  {
////     substr= memcpy()  //substring do 7 ao 12
////     // Agora tem o tempo
////
////     // Verificar se conexao ativa
////  }
////  //... Fazer para o resto
////
////  else // Nao bateu com nada
////    return;
////
////}
//




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

	/* Blink 4 times the LED */
	blink();
	blink();
	blink();
	blink();

	sentenceStatus=0;
	count=0;
	//checkST=0;

	/* Print a test phrase via UART2 */
	print("Teste!\n\r");

	while(1)
	{
//		if(sentCome){
//			sentCome=0;
//			verifySentence();
//		}



		// Do something

	}

}

/*
 *  ***************************************************************************
 * 	EOF - End Of File
 */
