/*
 * AM2320 I2C driver
 *
 * Created: 12-12-2020 15:35:57
 *  Author: Tim Dorssers
 */ 

#define F_CPU 1000000

#include <util/delay.h>
#include <util/crc16.h>
#include "am2320.h"
#include "i2cmaster.h"

// Returns 0 for success, 1 for no response, 2 for crc error
uint8_t am2320_get(uint16_t *humid, int16_t *temp) {
	uint8_t buffer[8];
	i2c_init();
	// Sensor wake up
	i2c_start(AM2320_ADDR + I2C_WRITE);
	_delay_us(800);
	i2c_stop();
	// Read temperature and humidity
	if (i2c_start(AM2320_ADDR + I2C_WRITE)) return 1;
	i2c_write(AM2320_CMD_READREG);
	i2c_write(AM2320_REG_HUMID_H);
	i2c_write(0x04);
	i2c_stop();
	// Host read back
	_delay_us(1600);
	i2c_start(AM2320_ADDR + I2C_READ);
	uint16_t crc = 0xFFFF;
	for (uint8_t s = 0; s < 7; s++) {
		buffer[s] = i2c_readAck();
		if (s < 6) crc = _crc16_update(crc, buffer[s]);
	}
	buffer[7] = i2c_readNak();
	i2c_stop();
	// Check CRC
	if (((buffer[7] << 8) | buffer[6]) != crc) return 2;
	*humid = (buffer[2] << 8) | buffer[3];
	*temp = ((buffer[4] & 0x7F) << 8) | buffer[5];
	// Check sign bit
	if (buffer[4] & 0x80)
		*temp *= -1;
	return 0;
}
