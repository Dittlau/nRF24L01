/*
 * nrf24_lib.h
 *
 * Created: 25-05-2017 23:30:49
 *  Author: Daniel
 */ 

#ifndef NRF24_LIB_H_
#define NRF24_LIB_H_

#ifndef F_CPU
#define F_CPU 8000000UL // 1MHz Clock
#endif

#include <avr/io.h>
#include <util/delay.h>

void INT0_interrupt_init(void);
void SPIinit(void);
uint8_t GetReg(uint8_t);
uint8_t *accessRegister(uint8_t, uint8_t, uint8_t *, uint8_t);
void nrfInit(uint8_t);
void transmit_payload(uint8_t *);
void receive_payload(void);
void reset(void);
void transmitterStart(void);
void transmitterStop(void);

#define PACKET_SIZE 8
#define NUMBER_RETRIES 5

extern uint8_t volatile packet_received;
extern uint8_t *received_data;

#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))


#if defined(__AVR_ATtiny44A__)

	#define CE 2	// A2
	#define CSN 3	// A3
	#define CLK 4	// A4
	#define MISO 6	// DI = A6
	#define MOSI 5	// DO = A5
	#define IRQ	2	// B2
	
	
	#define SET_CSN SETBIT(PORTA,CSN)
	#define CLEAR_CSN CLEARBIT(PORTA,CSN)
	#define SET_CE SETBIT(PORTA,CE)
	#define CLEAR_CE CLEARBIT(PORTA,CE)
	#define IRQ_PULLUP SETBIT(PORTB,IRQ)
	#define CLEAR_IRQ_PULLUP CLEARBIT(PORTB,IRQ)
	
	#define LED_ON SETBIT(PORTA,0)
	#define LED_OFF CLEARBIT(PORTA,0)
	
#elif defined(__AVR_ATmega328P__)

	#define CLK 13	// B5
	#define MISO 12 // B4
	#define MOSI 11 // B3
	#define CSN 2	// B2
	#define CE 2	// C2
	#define IRQ 2	// D2
	
	#define SET_CSN SETBIT(PORTB,CSN)
	#define CLEAR_CSN CLEARBIT(PORTB,CSN)
	#define SET_CE SETBIT(PORTC,CE)
	#define CLEAR_CE CLEARBIT(PORTC,CE)
	#define IRQ_PULLUP SETBIT(PORTD,IRQ)
	#define CLEAR_IRQ_PULLUP CLEARBIT(PORTD,IRQ)
	
	#define LED_ON SETBIT(PORTC,0)
	#define LED_OFF CLEARBIT(PORTC,0)

#endif

#define W 1
#define R 0


/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD      0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define READ_REGISTER    0x00
#define WRITE_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2



#endif /* NRF24_LIB_H_ */