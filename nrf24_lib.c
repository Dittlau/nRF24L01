/*
 * nrf24_lib.c
 *
 * Created: 25-05-2017 23:31:05
 *  Author: Daniel
 */ 
#include "nrf24_lib.h"
#include <AVR/interrupt.h>



#if defined(__AVR_ATtiny44A__)

void INT0_interrupt_init(void){
	DDRB &= ~(1<<DDB2);		// Set B2 to input

	MCUCR |= (1<<ISC01);	// INT0 falling edge
	MCUCR &= ~(1<<ISC00);	// INT0 falling edge

	GIMSK |= (1<<INT0);		// Enable int0
	sei();
}

ISR(EXT_INT0_vect){
	packet_received = 1;
	reset();
}


void SPIinit(void){
	DDRA |= (1<<DDA2) | (1<<DDA3) | (1<<DDA4) | (1<<DDA5);	// define CE, CSN, CLK and DO as outputs
	USICR |= (1<<USIWM0)|(1<<USICS1)|(1<<USICLK);
}

char writeSPI(uint8_t cData){
	
	USIDR = cData; // load byte to Data register
	USISR |= (1<<USIOIF); // Clear flag to be able to receive new data
	
	/* Wait for transmission complete */
	while ( (USISR & (1<<USIOIF)) == 0){
		USICR |=(1<<USITC);
	}
	return USIDR;
}

#elif defined(__AVR_ATmega328P__)

void INT0_interrupt_init(void){
	DDRD &= ~(1<<DDD2);		// Set D2 to input

	EICRA |= (1<<ISC01);	// INT0 falling edge
	EICRA &= ~(1<<ISC00);	// INT0 falling edge

	EIMSK |= (1<<INT0);		// Enable int0
	sei();
}

ISR(INT0_vect){
	packet_received = 1;
	reset();
}

void SPIinit(void){
	DDRB |= (1<<DDB2) | (1<<DDB3) | (1<<DDB5);	// define CSN, CLK and MOSI as outputs
	DDRC |= (1<<PORTC2);						// define CE as output
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
}

char writeSPI(uint8_t cData){
	SPDR = cData;	// Write to Data register
	while(!(SPSR & (1<<SPIF)));	// wait for transmission is complete
	return SPDR;
}

#endif


void nrfInit(uint8_t RX_TX){
	// RX_TX: 0 = Transmitter, 1 = Receiver
	
	uint8_t value[PACKET_SIZE];
	int i;
	
	SET_CSN;
	CLEAR_CE;
	IRQ_PULLUP;
	_delay_ms(100); //delay 100 milliseconds for startup
	value[0] = 0x01;
	accessRegister(W, EN_RXADDR, value, 1); // enable data pipe 0
	value[0] = 0x03;
	accessRegister(W, SETUP_AW, value, 1); // Setup address width to 5 bytes
	value[0] = 0x01;
	accessRegister(W, RF_CH, value, 1); // set frequency to 2,401 GHz
	value[0] = 0x07;
	accessRegister(W, RF_SETUP, value, 1); // 00000111 bit 3="0" 1Mbps=longer range, bit 2-1 power mode (""11" = -0dB ; "00"= -18dB)
	value[0] = 0x10;
	accessRegister(W, RX_ADDR_P0, value, 1);

	// Set Receiver address on Pipe 0 to 5 x 0x07
	for (i=0; i<5; i++){
		value[i] = 0x07;
	}
	accessRegister(W, RX_ADDR_P0, value, 5);

	// Set Transmitter address on Pipe 0 to 5 x 0x07
	for (i=0; i<5; i++){
		value[i] = 0x07;
	}
	accessRegister(W, TX_ADDR, value, 5);
	
	value[0] = PACKET_SIZE;
	accessRegister(W, RX_PW_P0, value, 1); // Send x bytes per package

	if (RX_TX == 1){
		value[0] = 0x33; //0x13;
	}
	else{
		value[0] = 0x32; //0x12;
	}
	accessRegister(W, CONFIG, value, 1); //bit "1":1=power up,  bit "0":0=transmitter,bit "0":1=Receiver, bit "4":1=>mask_Max_RT IRQ, bit "5": mask TX IRQ
	SET_CE;
	_delay_ms(100);
}

uint8_t GetReg(uint8_t reg){
	_delay_us(10);
	CLEAR_CSN;
	_delay_us(10);
	writeSPI(READ_REGISTER + reg);
	_delay_us(10);
	reg = writeSPI(NOP);
	_delay_us(10);
	SET_CSN;
	return reg;
}

uint8_t *accessRegister(uint8_t readWrite, uint8_t reg, uint8_t *value, uint8_t arraySize){
	
	int i;
	static uint8_t ret[PACKET_SIZE];

	if (readWrite == W){
		reg = WRITE_REGISTER + reg;
	}
	
	_delay_us(10);
	CLEAR_CSN;
	_delay_us(10);
	writeSPI(reg);
	_delay_us(10);

	for (i=0; i<arraySize; i++){
		if (readWrite == R && reg != W_TX_PAYLOAD){
			ret[i] = writeSPI(0);
			_delay_us(10);
		}
		else{
			writeSPI(value[i]);
			_delay_us(10);
		}
	}

	SET_CSN;
	return ret;
}

void transmit_payload(uint8_t *w_buffer){
	accessRegister(R, FLUSH_TX, w_buffer, 0);
	accessRegister(R, W_TX_PAYLOAD, w_buffer, PACKET_SIZE);
	_delay_ms(19);
	SET_CE;
	_delay_us(20);
	CLEAR_CE;
}

void receive_payload(void){
	SET_CE;
	_delay_ms(1000);
	CLEAR_CE;
	_delay_ms(50);
}

void reset(void){
	_delay_us(50);
	CLEAR_CSN;
	_delay_us(50);
	writeSPI(WRITE_REGISTER + STATUS);
	_delay_us(10);
	writeSPI(0x70);
	_delay_us(50);
	SET_CSN;
}

void transmitterStart(void) {

	uint8_t value[PACKET_SIZE];
	uint8_t *data = 0;

	//_delay_ms(100); // for debugging!!!
	
	_delay_ms(19);
	CLEAR_CE; // goto Standby mode
	_delay_us(20);
	data = accessRegister(R, CONFIG, data, 1);
	value[0] = data[0] & ~(1 << 0); // set PRIMRX=0 - Transmitter mode
	accessRegister(W, CONFIG, value, 1);
	SET_CE; // goto Transmitter mode
	_delay_us(250);
	accessRegister(R, FLUSH_TX, data, 0);
}

void transmitterStop(void) {

	uint8_t value[PACKET_SIZE];
	uint8_t *data = 0;
	
	//_delay_ms(100); // for debugging!!!
	
	_delay_ms(19);
	CLEAR_CE; // goto Standby mode
	_delay_us(20);
	data = accessRegister(R, CONFIG, data, 1);
	value[0] = data[0] | (1 << 0); // set PRIMRX=0 - Transmitter mode
	accessRegister(W, CONFIG, value, 1);
	SET_CE;
	_delay_us(250);
	
	accessRegister(R, FLUSH_RX, data, 0);
	
	//_delay_ms(100); // for debugging!!!
}

