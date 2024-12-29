/*
 * Title   : Remote temperature sensor module
 * Hardware: ATtiny25/45/85 @ 1 MHz, RFM85 433MHz ASK/OOK transmitter module,
 *           AHT20/AM2320/SHT30 digital temperature and humidity sensor or
 *           DS18B20 digital temperature sensor, 4 DIP switches
 * Created : 12-12-2020 12:56:25
 * Author  : Tim Dorssers
 *
 * PB0 -> AHT20/AM2320 SDA or DS18B20 DQ
 * PB1 -> transmitter data
 * PB2 -> AHT20/AM2320 SCL or N/C
 * PB3 -> resistor ladder
 * PB4 -> transmitter Vcc
 */ 

#define F_CPU 1000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <stdlib.h>
#include "am2320.h"
#include "ds18b20.h"
#include "aht20.h"
#include "sht30.h"

#undef DEBUG

#define BAUD 1200
#define STOPBITS 1
#define PRESCALE ((F_CPU) / (BAUD) > 255 ? 8 : 1)
#define FULL_BIT_TICKS ((F_CPU) / (BAUD) / (PRESCALE))

typedef struct {
	uint8_t unit;   // Bits 0-4 = id, bits 5-6 = result, bits 7-8 = type
	uint16_t humid;
	int16_t temp;
} packet_t;

const uint8_t lookup[] PROGMEM = {116, 112, 107, 102, 97, 91, 85, 77, 68, 60, 50, 42, 32, 20, 6};

static uint8_t reverse_byte(uint8_t x) {
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}

static void usi_uart_putc(char data) {
	data = reverse_byte(data);
	// Configure Timer0 in CTC mode to trigger every full bit width and reset
	TCCR0A = _BV(WGM01);
	#if PRESCALE == 8
	TCCR0B = _BV(CS01);
	#else
	TCCR0B = _BV(CS00);
	#endif
	OCR0A = FULL_BIT_TICKS;
	TCNT0 = 0;
	// Start bit (low) followed by first 7 bits of data
	USIDR = 0x00 | data >> 1;
	// Enable three wire mode using Timer0 as clock source
	USICR  = _BV(USIWM0) | _BV(USICS0);
	DDRB |= _BV(PB1);  // Configure USI_DO as output
	// Clear USI overflow interrupt flag and set USI counter to 8
	USISR = _BV(USIOIF) | (16 - 8);
	loop_until_bit_is_set(USISR, USIOIF);  // Wait until bits are transmitted
	USIDR = data << 7 | 0x7F; // Send last 1 bit of data and stop bits (high)
	// Clear USI overflow flag and set counter to send last bit and stop bits
	USISR = _BV(USIOIF) | (16 - (1 + (STOPBITS)));
	loop_until_bit_is_set(USISR, USIOIF);  // Wait until bits are transmitted
	PORTB |= _BV(PB1);  // Ensure output is high
	USICR = 0;  // Disable USI
}

#ifdef DEBUG
static void usi_uart_puts(char *buffer) {
	while (*buffer)
		usi_uart_putc(*buffer++);
}

void usi_uart_puts_p(const char *progmem_s) {
	char c;
	while ((c = pgm_read_byte(progmem_s++))) {
		usi_uart_putc(c);
	}
}

void dump_int(const char *progmem_s, int val) {
	char buffer[10];
	usi_uart_puts_p(progmem_s);
	itoa(val, buffer, 10);
	usi_uart_puts(buffer);
	usi_uart_puts_p(PSTR("\r\n"));
}
#endif

EMPTY_INTERRUPT(WDT_vect);

static void wdt_on(void) {
	wdt_reset();
	MCUSR = 0x00;
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0);    // 8192ms
}

static void wdt_off(void) {
	wdt_reset();
	MCUSR = 0x00;
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = 0x00;
}

int main(void) {
	packet_t txData;

	sei();
	// Vcc reference, channel ADC3, left adjust result, prescaler /8, enable ADC
	ADMUX = _BV(ADLAR) | _BV(MUX1) | _BV(MUX0);
	ADCSRA = _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN);
	// Enter main loop
	while (1) {
		txData.humid = 0xaaaa;
		uint8_t type = 0, id;
		// Read out sensor
		uint8_t result = ds18b20_read_temperature(&txData.temp);
		if (result == 1) {
			result = am2320_get(&txData.humid, &txData.temp);
			type = 1;
		}
		if (result == 1) {
			result = aht20_get(&txData.humid, &txData.temp);
			type = 2;
		}
		#ifndef __AVR_ATtiny25__
		if (result == 1) {
			result = sht30_readTempHum(&txData.temp, &txData.humid);
			type = 3;
		}
		#endif
		// Read ID
		ADCSRA |= _BV(ADSC);
		loop_until_bit_is_clear(ADCSRA, ADSC);
		for (id = 0; id < 15; id++)
			if (pgm_read_byte(lookup + id) < ADCH) break;
		txData.unit = type << 6 | result << 4 | id;
		#ifdef DEBUG
		if (result == 1)
			usi_uart_puts_p(PSTR("No response\r\n"));
		else if (result == 2)
			usi_uart_puts_p(PSTR("CRC error\r\n"));
		else {
			dump_int(PSTR("\r\nid="), id);
			if (type) dump_int(PSTR("humid="), txData.humid);
			dump_int(PSTR("temp="), txData.temp);
		}
		#endif
		// Turn on transmitter
		DDRB |= _BV(PB4);
		PORTB |= _BV(PB4);
		_delay_ms(20);
		// Send preamble
		usi_uart_putc(0x55);
		usi_uart_putc(0x55);
		// Send packet and calculate CRC
		uint16_t crc = 0xFFFF;
		for (uint8_t i=0; i<sizeof(txData); i++) {
			uint8_t c = ((uint8_t *)&txData)[i];
			usi_uart_putc(c);
			crc = _crc16_update(crc, c);
		}
		// Send CRC
		usi_uart_putc(crc);
		usi_uart_putc(crc >> 8);
		// Turn transmitter off
		_delay_ms(20);
		PORTB &= ~_BV(PB4);
		// Enter sleep mode for 8 seconds
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		wdt_on();
		sleep_cpu();
		sleep_disable();
		wdt_off();
	}
}
