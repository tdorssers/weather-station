/*
 * DS18B20 Driver
 *
 * Created: 26-12-2020 16:27:42
 *  Author: Tim Dorssers
 */ 

#define F_CPU 1000000

#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <util/crc16.h>
#include "ds18b20.h"

// Return the value read from the presence pulse (0=OK, 1=WRONG)
uint8_t ds18b20_reset() {
	uint8_t i;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Pull line low and wait for 480uS
		DS18B20_LOW();
		DS18B20_OUTPUT_MODE();
		_delay_us(480);
		// Release line and wait for 60uS
		DS18B20_INPUT_MODE();
		_delay_us(60);
		// Store line value and wait until the completion of 480uS period
		i = (DS18B20_PIN & (1 << DS18B20_DQ));
		_delay_us(420);
	}
	return i;
}
void ds18b20_write_bit(uint8_t bit) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Pull line low for 1uS
		DS18B20_LOW();
		DS18B20_OUTPUT_MODE();
		_delay_us(1);
		// If we want to write 1, release the line (if not will keep low)
		if (bit) DS18B20_INPUT_MODE();
		// Wait for 60uS and release the line
		_delay_us(60);
		DS18B20_INPUT_MODE();
	}
}
uint8_t ds18b20_read_bit(void) {
	uint8_t bit=0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Pull line low for 1uS
		DS18B20_LOW();
		DS18B20_OUTPUT_MODE();
		_delay_us(1);
		// Release line and wait for 14uS
		DS18B20_INPUT_MODE();
		_delay_us(14);
		// Read line value
		if (DS18B20_PIN & (1 << DS18B20_DQ)) bit=1;
		// Wait for 45uS to end and return read value
		_delay_us(45);
	}
	return bit;
}
uint8_t ds18b20_read_byte(void) {
	uint8_t i=8, n=0;
	while (i--){
		// Shift one position right and store read value
		n >>= 1;
		n |= (ds18b20_read_bit() << 7);
	}
	return n;
}
void ds18b20_write_byte(uint8_t byte) {
	uint8_t i=8;
	while(i--){
		// Write actual bit and shift one position right to make the next bit ready
		ds18b20_write_bit(byte & 1);
		byte >>= 1;
	}
}
// Returns 0 for success, 1 for no response, 2 for crc error
uint8_t ds18b20_read_temperature(int16_t *temp) {
	uint8_t scratchpad[9], crc=0;
	int8_t digit;
	uint16_t decimal;
	// Reset, skip ROM and start temperature conversion
	if (ds18b20_reset()) return 1;
	ds18b20_write_byte(DS18B20_CMD_SKIPROM);
	ds18b20_write_byte(DS18B20_CMD_CONVERTTEMP);
	// Wait until conversion is complete
	while(!ds18b20_read_bit());
	// Reset, skip ROM and send command to read Scratch pad
	ds18b20_reset();
	ds18b20_write_byte(DS18B20_CMD_SKIPROM);
	ds18b20_write_byte(DS18B20_CMD_RSCRATCHPAD);
	// Read Scratch pad
	for (uint8_t i=0; i<9; i++) {
		scratchpad[i] = ds18b20_read_byte();
		if (i < 8) crc = _crc_ibutton_update(crc, scratchpad[i]);
	}
	// Check CRC
	if (crc != scratchpad[8]) return 2;
	// Store temperature integer digits and decimal digits
	digit = scratchpad[0] >> 4;
	digit |= (scratchpad[1] & 0x7) << 4;
	decimal = scratchpad[0] & 0xf;
	decimal *= DS18B20_DECIMAL_STEPS_12BIT;
	// Calculate temperature in 0.1 degC resolution, "213" equals 21.3 degrees degC
	*temp = (digit * 10) + decimal / 1000;
	return 0;
}