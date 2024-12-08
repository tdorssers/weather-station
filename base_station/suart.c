/*
 * Software UART using Timer1
 *
 * Created: 2-5-2021 13:51:23
 *  Author: Tim Dorssers
 */ 

#ifndef F_CPU
#define F_CPU 12000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "suart.h"

static volatile uint8_t stx_buff[STX_SIZE];	// circular buffer
static volatile uint8_t stx_in;				// head
static volatile uint8_t stx_out;			// tail
static volatile uint8_t stx_data;			// byte
static volatile uint8_t stx_count;			// current bit

static volatile uint8_t srx_buff[SRX_SIZE];
static volatile uint8_t srx_in;
static volatile uint8_t srx_out;
static volatile uint8_t srx_data;
static volatile uint8_t srx_count;

void suart_init(void) {
	OCR1A = BIT_TIME - 1;
	TCCR1A = _BV(COM1A1) | _BV(COM1A0);				// set OC1A high on compare match
	TCCR1B = _BV(ICNC1) | _BV(WGM12) | _BV(CS10);	// noise canceler, falling edge, CLK/1, T1 mode 4 (CTC)
	TCCR1C = _BV(FOC1A);							// force output compare
	stx_count = 0;
	stx_in = stx_out = 0;
	srx_in = srx_out = 0;
	STXDDR |= _BV(STX);					// output enable
	TIFR1 = _BV(ICF1);					// clear pending interrupt
	TIMSK1 = _BV(ICIE1) | _BV(OCIE1A);	// enable TX and wait for start
}

// check, if RX buffer not empty
uint8_t suart_available(void) {
	return srx_out ^ srx_in;
}

// wait until byte received
uint8_t suart_getc(void) {
	uint8_t data;

	while (!suart_available());			// until at least one byte in
	data = srx_buff[srx_out];			// get byte
	srx_out = (srx_out + 1) & SRX_MASK;	// increment and wrap around
	return data;
}

// transmit byte
void suart_putc(uint8_t c) {
	uint8_t i = (stx_in + 1) & STX_MASK;
	stx_buff[stx_in] = ~c;				// complement for stop bit after data
	while (i == stx_out);				// until at least one byte free
	stx_in = i;
}

// transmit string from SRAM
void suart_puts(const char *s) {
	while (*s)
		suart_putc(*s++);
}

// transmit string from flash
void suart_puts_p(const char *progmem_s) {
	register char c;

	while ((c = pgm_read_byte(progmem_s++)))
		suart_putc(c);
}

// start detection
ISR(TIMER1_CAPT_vect) {
	int16_t i = ICR1 - BIT_TIME / 2;		// scan at 0.5 bit time
	if (i < 0)
		i += BIT_TIME;						// wrap around
	OCR1B = i;
	srx_count = 10;
	TIFR1 = _BV(OCF1B);						// clear pending interrupt
	if (bit_is_clear(SRXPIN, SRX))			// still low
		TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);	// wait for first bit
}

// receive data bits
ISR(TIMER1_COMPB_vect) {
	uint8_t i;

	switch (--srx_count) {
		case 9:
			if (bit_is_clear(SRXPIN, SRX))	// start bit valid
				return;
			break;
		default:
			i = srx_data >> 1;	      		// LSB first
			if (bit_is_set(SRXPIN, SRX))
				i |= 0x80;					// data bit = 1
			srx_data = i;
			return;
		case 0:
			if (bit_is_set(SRXPIN, SRX)) {	// stop bit valid
				i = (srx_in + 1) & SRX_MASK;
				if (i != srx_out) {			// no buffer overflow
					srx_buff[srx_in] = srx_data;
					srx_in = i;				// advance in pointer
				}
			}
			TIFR1 = _BV(ICF1);				// clear pending interrupt
	}
	TIMSK1 = _BV(ICIE1) | _BV(OCIE1A);		// enable next start
}

// transmit data bits
ISR(TIMER1_COMPA_vect) {
	if (stx_count) {
		stx_count--;
		TCCR1A = _BV(COM1A1) | _BV(COM1A0);	// high on compare match
		if (stx_data & 1)					// LSB first
			TCCR1A = _BV(COM1A1);			// low on compare match
		stx_data >>= 1;
		return;
	}
	if (stx_in != stx_out) {				// next byte to sent
		stx_data = stx_buff[stx_out];
		stx_out = (stx_out + 1) & STX_MASK;	// increment and wrap around
		stx_count = 9;						// 8 data bits + stop
		TCCR1A = _BV(COM1A1);				// start bit (low)
	}
}
