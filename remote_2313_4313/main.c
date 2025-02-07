/*
 * Title   : Remote temperature sensor module
 * Hardware: ATtiny2313/2313A/4313 @ 1 MHz, RFM85 433MHz ASK/OOK transmitter
 *           module, AHT20/AM2320/SHT30 digital temperature and humidity sensor
 *           or DS18B20 digital temperature sensor, 4 DIP switches
 * Created: 26-12-2020 13:34:28
 * Author : Tim Dorssers
 *
 * PB0 -> DIP switch 1
 * PB1 -> DIP switch 2
 * PB2 -> DIP switch 3
 * PB3 -> DIP switch 4
 * PB4 -> transmitter Vcc
 * PB5 -> AHT20/AM2320 SDA or DS18B20 DQ
 * PB7 -> AHT20/AM2320 SCL or N/C
 * PD1 -> transmitter data
 */ 

#define F_CPU 1000000
#define BAUD 1200

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/cpufunc.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdlib.h>
#include "am2320.h"
#include "ds18b20.h"
#include "aht20.h"
#include "sht30.h"

#undef DEBUG

typedef struct {
	uint8_t unit;   // Bits 0-4 = id, bits 5-6 = result, bits 7-8 = type
	uint16_t humid;
	int16_t temp;
} packet_t;

static void uart_init(void) {
	UCSRB = _BV(TXEN);
	UCSRC = _BV(UCSZ0) | _BV(UCSZ1); // 8N1
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
}

static void uart_putc(char c) {
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = c;
}

#ifdef DEBUG
static void uart_puts(const char *string) {
	while(*string)
		uart_putc(*string++);
}

static void uart_puts_p(const char *string) {
	char c;
	while ((c = pgm_read_byte(string++)))
		uart_putc(c);
}

void dump_int(const char *progmem_s, int val) {
	char buffer[10];
	uart_puts_p(progmem_s);
	itoa(val, buffer, 10);
	uart_puts(buffer);
	uart_puts_p(PSTR("\r\n"));
}
#endif

EMPTY_INTERRUPT(WDT_OVERFLOW_vect);

static void wdt_on(void) {
	wdt_reset();
	MCUSR = 0x00;
	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0);    // 8192ms
}

static void wdt_off(void) {
	wdt_reset();
	MCUSR = 0x00;
	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0x00;
}

static uint8_t get_id(void) {
	PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
	_NOP();
	return ~PINB & 0xf;
}

int main(void) {
	packet_t txData;
	
	uart_init();
	sei();
	// Enter main loop
    while (1) {
		txData.humid = 0xaaaa;
		uint8_t type = 0;
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
		#ifdef __AVR_ATtiny4313__
		if (result == 1) {
			result = sht30_readTempHum(&txData.temp, &txData.humid);
			type = 3;
		}
		#endif
		txData.unit = type << 6 | result << 4 | get_id();
		#ifdef DEBUG
		if (result == 1)
			uart_puts_p(PSTR("No response\r\n"));
		else if (result == 2)
			uart_puts_p(PSTR("CRC error\r\n"));
		else {
			dump_int(PSTR("\r\nid="), get_id());
			if (type) dump_int(PSTR("humid="), txData.humid);
			dump_int(PSTR("temp="), txData.temp);
		}
		#endif
		// Turn on transmitter
		DDRB |= _BV(PB4);
		PORTB |= _BV(PB4);
		_delay_ms(20);
		// Send preamble
		uart_putc(0x55);
		uart_putc(0x55);
		// Send packet and calculate CRC
		uint16_t crc = 0xFFFF;
		for (uint8_t i=0; i<sizeof(txData); i++) {
			uint8_t c = ((uint8_t *)&txData)[i];
			uart_putc(c);
			crc = _crc16_update(crc, c);
		}
		// Send CRC
		uart_putc(crc);
		uart_putc(crc >> 8);
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
