/*
 * SHT31 Digital Humidity & Temp Sensor
 *
 * Created: 21/12/2024 20:01:13
 *  Author: Tim Dorssers
 */ 

#define F_CPU 1000000

#include <util/delay.h>
#include "sht30.h"
#include "i2cmaster.h"
#include "crc8.h"

static uint8_t writeCommand(uint16_t command) {
	if (i2c_start(SHT31_DEFAULT_ADDR << 1 | I2C_WRITE)) return 1;
	i2c_write(command >> 8);
	i2c_write(command & 0xFF);
	i2c_stop();
	return 0;
}

uint8_t sht30_readTempHum(int16_t *temp, uint16_t *humidity) {
	uint8_t msb, lsb, crc;
	i2c_init();
	if (writeCommand(SHT31_MEAS_HIGHREP)) return 1;
	_delay_ms(20);
	i2c_start(SHT31_DEFAULT_ADDR << 1 | I2C_READ);
	msb = i2c_readAck();
	crc = _crc8_update(0xFF, msb);
	lsb = i2c_readAck();
	if (i2c_readAck() != _crc8_update(crc, lsb)) return 2;
	int32_t stemp = (int32_t)(((uint32_t)msb << 8) | lsb);
	stemp = (875 * stemp) >> 15;
	*temp = (int16_t)stemp - 450;
	msb = i2c_readAck();
	crc = _crc8_update(0xFF, msb);
	lsb = i2c_readAck();
	if (i2c_readNak() != _crc8_update(crc, lsb)) return 2;
	uint32_t shum = ((uint32_t)msb << 8) | lsb;
	shum = (125 * shum) >> 13;
	*humidity = (uint16_t)shum;
	i2c_stop();
	return 0;
}