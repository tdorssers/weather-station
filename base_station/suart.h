/*
 * Software UART using Timer1
 *
 * Created: 2-5-2021 13:51:36
 *  Author: Tim Dorssers
 */ 


#ifndef SUART_H_
#define SUART_H_

#include <avr/io.h>
#include <stdint.h>

#define BAUD		9600
#define STX_SIZE	2		// between 2 and 256
#define	SRX_SIZE	256

#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega48P__) || defined(__AVR_ATmega88P__) || \
	defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || \
	defined(__AVR_ATtiny48__) || defined(__AVR_ATtiny88__)
	#define SRX			PB0     // ICP
	#define SRXPIN		PINB
	#define STX			PB1     // OC1A
	#define STXDDR		DDRB
#elif defined(__AVR_ATmega16__) || defined(__AVR_ATmega163__) || defined(__AVR_ATmega32__) || \
      defined(__AVR_ATmega323__) || defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || \
	  defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
	#define SRX			PD6     // ICP
	#define SRXPIN		PIND
	#define STX			PD5     // OC1A
	#define STXDDR		DDRD
#elif defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega64__) || \
      defined(__AVR_ATmega128__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || \
	  defined(__AVR_ATmega640__)
	#define SRX			PD4     // ICP
	#define SRXPIN		PIND
	#define STX			PB5     // OC1A
	#define STXDDR		DDRB
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega32U2__)
	#define SRX			PC7     // ICP
	#define SRXPIN		PINC
	#define STX			PC6     // OC1A
	#define STXDDR		DDRC
#elif defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__) || defined(__AVR_ATmega162__)
	#define SRX			PE0     // ICP
	#define SRXPIN		PINE
	#define STX			PD5     // OC1A
	#define STXDDR		DDRD
#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)
	#define SRX			PD6     // ICP
	#define SRXPIN		PIND
	#define STX			PD3     // OC1A
	#define STXDDR		DDRD
#elif defined(__AVR_ATmega169__) || defined(__AVR_ATmega329__) || defined(__AVR_ATmega649__) || \
      defined(__AVR_ATmega325__) ||defined(__AVR_ATmega3250__) || defined(__AVR_ATmega645__) || \
	  defined(__AVR_ATmega6450__) || defined(__AVR_ATmega3290__) || defined(__AVR_ATmega6490__)
	#define SRX			PD0     // ICP
	#define SRXPIN		PIND
	#define STX			PD5     // OC1A
	#define STXDDR		DDRD
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	#define SRX			PA7     // ICP
	#define SRXPIN		PINA
	#define STX			PA6     // OC0A
	#define STXDDR		DDRA
#else
	#error "no pin definition for MCU available"
#endif

#define BIT_TIME	(uint16_t)(F_CPU * 1.0 / BAUD + 0.5)
#define STX_MASK	(STX_SIZE - 1)
#define SRX_MASK	(SRX_SIZE - 1)

void suart_init(void);
uint8_t suart_available(void);
uint8_t suart_getc(void);
void suart_putc(uint8_t c);
void suart_puts(const char *s);
void suart_puts_p(const char *progmem_s);

#define suart_puts_P(__s) suart_puts_p(PSTR(__s))

#endif /* SUART_H_ */