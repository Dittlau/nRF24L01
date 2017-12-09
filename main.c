/*
 * Attiny_RF.c
 *
 * Created: 05-06-2017 14:42:01
 * Author : Daniel
 */ 

#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 8000000UL // 1MHz Clock
#endif

#include <util/delay.h>
#include <stdlib.h>
#include "nrf24_lib.h"

#if defined(__AVR_ATtiny44A__) 

#define LED_PORT PORTA0
#define DEVICEID 0X00000003
#define USART_YES 0

#elif defined(__AVR_ATmega328P__)

#define LED_PORT PORTC0
#define DEVICEID 0X00000002
#define USART_YES 1

#endif

#if USART_YES
	#include "uart.h"
#endif

#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

void processPackage(void);

uint8_t volatile currentStringIndex = 0;
uint8_t volatile stringReceived = 0;
uint8_t volatile packet_received = 0;
uint8_t *received_data = 0;

#if USART_YES
	uint8_t stringBuffer[MAX_CHAR_IN_STRING];
	char buffer[16];
#endif

///////////////////////////
uint8_t secondsPassed = 0;
uint8_t lastReceived = 0;
uint16_t totalPackets = 0;
uint16_t totalPacketsLost = 0;
///////////////////////////


int main(void)
{
	#if defined(__AVR_ATtiny44A__)
		set_output(DDRA, LED_PORT);
	#elif defined(__AVR_ATmega328P__)
		set_output(DDRC, LED_PORT);
	#endif
	
	#if USART_YES
		USART_init();
	#endif
	SPIinit();
	nrfInit(1);
	INT0_interrupt_init();
	reset();
	
	//  MAIN LOOP //
	while (1)
	{	
		
		_delay_ms(100); // something with this delay??? or power??
		if(packet_received){
			
			
			processPackage();	
		
		}
		
	}
}


void processPackage(void){
	
	uint8_t receivedData[PACKET_SIZE]; // container to hold the received data
	uint8_t i; // Counter variable
	uint8_t Packet_ID;
	uint32_t Receiver_ID;
	
	received_data = accessRegister(R,R_RX_PAYLOAD,received_data,PACKET_SIZE);

	packet_received = 0; // reset flag
	
	totalPackets++;

	Packet_ID = received_data[0];
	Receiver_ID = (((uint32_t)received_data[1] << 24) | ((uint32_t)received_data[2] << 16) | ((uint32_t)received_data[3]<<8) | (uint32_t)received_data[4]);
	
	#if USART_YES
	//USART_sendString("Packet ID = ");
	//itoa(Packet_ID,buffer,10);
	//USART_sendString(buffer);
	//
	//USART_sendString(", Receiver ID = ");
	//itoa(Receiver_ID,buffer,10);
	//USART_sendString(buffer);
	//USART_sendString(":");
	//
	//USART_sendString("  ");
	#endif
	
	if (Receiver_ID == DEVICEID){
		
		#if USART_YES
		  USART_sendString("Matching the packet, sending back: ");
		#endif
		
		// Make a local copy of the received data
		for(i=0;i<PACKET_SIZE;i++){
			receivedData[i] = received_data[i];
		}
		
		
		
		// Send acknowledge package (same as received)
		transmitterStart();
		transmit_payload(receivedData);
		transmitterStop();
		
		#if USART_YES
		//for (uint8_t i=0;i<PACKET_SIZE;i++){
			itoa(receivedData[0],buffer,10);
			USART_sendString(buffer);
		//}
		#endif
	}
	else{
		accessRegister(R, FLUSH_RX, received_data, 0);
	}
	
	#if USART_YES
	//USART_sendString(" Received: ");
	//
	//for (i=0;i<PACKET_SIZE;i++){
		//itoa(receivedData[i],buffer,10);
		//USART_sendString(buffer);
	//}	
	
	/////////////////
	if(receivedData[0]-1 > lastReceived){
		
		USART_sendString(" skipped! ");
		
		totalPacketsLost++;
	}
	else{
		
		USART_sendString(" Total lost: ");
		itoa(totalPacketsLost,buffer,10);
		USART_sendString(buffer);
		USART_sendString(" of ");
		itoa(totalPackets,buffer,10);
		USART_sendString(buffer);
	}
	//////////////////
	
	USART_sendChar('\n');
	USART_sendChar('\r');
	
	lastReceived = receivedData[0];
	secondsPassed = 0;
	#endif	
}