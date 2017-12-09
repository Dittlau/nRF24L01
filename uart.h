/*
 * uart.h
 *
 * Created: 23-05-2017 17:22:51
 *  Author: Daniel
 */ 

#if USART_YES

#ifndef UART_H_
#define UART_H_

#ifndef F_CPU
#define F_CPU 8000000UL // 8MHz Clock
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BAUD 500000
#define BAUDRATE (((F_CPU/(BAUD*16UL)))-1)

#define MAX_CHAR_IN_STRING 50

extern volatile uint8_t stringReceived;
extern uint8_t stringBuffer[MAX_CHAR_IN_STRING];
extern volatile uint8_t currentStringIndex;

ISR(USART_RX_vect);
void USART_init(void);
unsigned char USART_receive(void);
void USART_sendChar(char);
void USART_sendString(char string[]);

#endif /* UART_H_ */

#endif