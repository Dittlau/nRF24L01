/*
 * uart.c
 *
 * Created: 23-05-2017 17:23:04
 *  Author: Daniel
 */ 
//#ifdef UART_H_

#include "uart.h"

#if USART_YES

void USART_init(void){
	
	// Set BAUD rate
	UBRR0H = (uint8_t) (BAUDRATE>>8);
	UBRR0L = (uint8_t) BAUDRATE;
	
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
	// Enable interrupts
	UCSR0B |= (1<<RXCIE0);
	sei();
}

ISR(USART_RX_vect){
	
	uint8_t receivedChar = UDR0;
	if ((receivedChar != '\r')){
		stringBuffer[currentStringIndex] = UDR0;
		currentStringIndex++;
		UDR0 = receivedChar;
	}
	else{
		stringBuffer[currentStringIndex] = UDR0;
		currentStringIndex++;
		stringReceived = 1;
	}
}

unsigned char USART_receive(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void USART_sendChar(char data){
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void USART_sendString(char string[]){
	
	int i = 0;
	
	while(string[i] != 0x00){
		USART_sendChar(string[i]);
		i++;
	}
}

#endif