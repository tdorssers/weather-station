/*
 * AHT20 Driver
 *
 * Created: 25-8-2022 20:50:04
 *  Author: Tim Dorssers
 */ 

#define F_CPU 1000000

#include <util/delay.h>
#include "aht20.h"
#include "i2cmaster.h"
#include "crc8.h"

uint8_t aht20_get(uint16_t *humid, int16_t *temperature) {
	uint8_t data[7];
	i2c_init();
	/* send status command */
	if (i2c_start(AHTXX_ADDRESS << 1 | I2C_WRITE)) return 1;
	i2c_write(AHTXX_STATUS_REG);
	/* check calibration bit */
	i2c_rep_start(AHTXX_ADDRESS << 1 | I2C_READ);
	uint8_t status = i2c_readNak();
	i2c_stop();
	if (!(status & 0x08)) {
		/* Set initialization register */
		i2c_start(AHTXX_ADDRESS << 1 | I2C_WRITE);
		i2c_write(AHT2X_INIT_REG);
		i2c_write(0x08);
		i2c_write(0x00);
		i2c_stop();
		_delay_ms(10);
	}
	/* send measurement command */
	i2c_start(AHTXX_ADDRESS << 1 | I2C_WRITE);
	i2c_write(AHTXX_START_MEASUREMENT_REG);
	i2c_write(0x33);
	i2c_write(0x00);
	i2c_stop();
	_delay_ms(80);
	/* read data from sensor */
	i2c_start(AHTXX_ADDRESS << 1 | I2C_READ);
	uint8_t crc = 0xFF;
	for (uint8_t i = 0; i < 6; i++ ) {
		data[i] = i2c_readAck();
		crc = _crc8_update(crc, data[i]);
	}
	data[6] = i2c_readNak();
	i2c_stop();
	if (crc != data[6]) return 2;
	/*  Calculate the temperature and humidity value */
	uint32_t hum = ((uint32_t)data[1] << 12) | ((uint16_t)data[2] << 4) | (data[3] >> 4);
	hum = (hum * 250) >> 18;
	*humid = (uint16_t)hum;
	uint32_t temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint16_t)data[4] << 8) | data[5];
	temp = (temp * 250) >> 17;
	*temperature = (int16_t)temp - 500;
	return 0;
}
