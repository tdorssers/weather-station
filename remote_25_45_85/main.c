/*
 * Title   : Remote temperature sensor module
 * Hardware: ATtiny45/85 @ 1 MHz, RFM85 433MHz ASK/OOK transmitter module,
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
#include "usi_uart.h"
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

packet_t txData;

const uint8_t lookup[] PROGMEM = {116, 112, 107, 102, 97, 91, 85, 77, 68, 60, 50, 42, 32, 20, 6};

#ifdef DEBUG
static void dump(char *buffer) {
	while (*buffer)
		USI_UART_Transmit_Byte(*buffer++);
}

void dump_p(const char *progmem_s) {
	register char c;
	while ((c = pgm_read_byte(progmem_s++))) {
		USI_UART_Transmit_Byte(c);
	}
}

void dump_int(const char *progmem_s, int val) {
	char buffer[10];
	dump_p(progmem_s);
	itoa(val, buffer, 10);
	dump(buffer);
	dump_p(PSTR("\r\n"));
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
		if (result == 1) {
			result = sht30_readTempHum(&txData.temp, &txData.humid);
			type = 3;
		}
		// Read ID
		ADCSRA |= _BV(ADSC);
		loop_until_bit_is_clear(ADCSRA, ADSC);
		for (id = 0; id < 15; id++)
			if (pgm_read_byte(lookup + id) < ADCH) break;
		txData.unit = type << 6 | result << 4 | id;
		#ifdef DEBUG
		if (result == 1)
			dump_p(PSTR("No response\r\n"));
		else if (result == 2)
			dump_p(PSTR("CRC error\r\n"));
		else {
			dump_int(PSTR("\r\nid="), id);
			if (type) dump_int(PSTR("humid="), txData.humid);
			dump_int(PSTR("temp="), txData.temp);
		}
		_delay_ms(240);
		#endif
		// Turn on transmitter
		DDRB |= _BV(PB4);
		PORTB |= _BV(PB4);
		// Send preamble
		USI_UART_Transmit_Byte(0x55);
		USI_UART_Transmit_Byte(0x55);
		// Send packet and calculate CRC
		uint16_t crc = 0xFFFF;
		for (uint8_t i=0; i<sizeof(txData); i++) {
			uint8_t c = ((uint8_t *)&txData)[i];
			USI_UART_Transmit_Byte(c);
			crc = _crc16_update(crc, c);
		}
		// Send CRC
		USI_UART_Transmit_Byte(crc);
		USI_UART_Transmit_Byte(crc >> 8);
		// Wait until transmission is finished
		_delay_ms(120);
		// Turn transmitter off
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
