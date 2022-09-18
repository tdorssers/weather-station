/*
 * BME280 I2C/SPI Driver
 *
 * Created: 29-11-2020 16:00:04
 *  Author: Tim Dorssers
 *
 * PB2 -> CSB
 * PB3 -> SDA/MOSI
 * PB4 -> SDO/MISO
 * PB5 -> SCL/SCK
 */ 

#define USE_SPI // Don't use I2C

#include "bme280.h"
#ifdef USE_SPI
#include <avr/io.h>
#else
#include "i2cmaster.h"
#endif

uint8_t m_dig[32];
int32_t t_fine;

#define spi_begin() PORTB &= ~_BV(PB2) // CS low
#define spi_end() PORTB |= _BV(PB2) // CS high

#ifdef USE_SPI
inline uint8_t spi(uint8_t data) {
	SPDR = data;
	while (!(SPSR & _BV(SPIF))) ; // wait
	return SPDR;
}
#endif

void write8(uint8_t reg, uint8_t value) {
#ifdef USE_SPI
	spi_begin();
	spi(reg & BME280_SPI_WRITE);
	spi(value);
	spi_end();
#else
	i2c_start(BME280_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
#endif
}

uint8_t read8(uint8_t reg) {
	uint8_t value;
#ifdef USE_SPI
	spi_begin();
	spi(reg | BME280_SPI_READ);
	value = spi(0x0);
	spi_end();
#else
	if (i2c_start(BME280_ADDRESS+I2C_WRITE)) return 0;
	i2c_write(reg);
	i2c_rep_start(BME280_ADDRESS+I2C_READ);
	value = i2c_readNak();
	i2c_stop();
#endif
	return value;
}

void readMulti(uint8_t reg, uint8_t *data, uint8_t len) {
#ifdef USE_SPI
	spi_begin();
	spi(reg | BME280_SPI_READ);
	while (len--)
		*data++ =spi(0x0);
	spi_end();
#else
	if (i2c_start(BME280_ADDRESS+I2C_WRITE)) return;
	i2c_write(reg);
	i2c_rep_start(BME280_ADDRESS+I2C_READ);
	while (len) {
		*data++ = i2c_read(len > 1);
		len--;
	}
	i2c_stop();
#endif
}

// Initialize sensor. Returns 0 on success and 1 otherwise.
uint8_t bme280_init() {
#ifdef USE_SPI
	DDRB |= _BV(PB2); // CS output
	PORTB |= _BV(PB2); // CS high
	DDRB |= _BV(PB3) | _BV(PB5); // MOSI and SCK as output
	SPCR = _BV(SPE) | _BV(MSTR); // Mode 0, fOsc/4
	SPSR |= _BV(SPI2X); // Double clock rate
#else
	i2c_init();
#endif
	// Check if sensor, i.e. the chip ID is correct
	if (read8(BME280_REGISTER_CHIPID) != 0x60) return 1;
	// Reset the device using soft-reset. This makes sure the IIR is off, etc.
	write8(BME280_REGISTER_SOFTRESET, 0xB6);
	// Wait for chip to read calibration
	while (read8(BME280_REGISTER_STATUS) & 0x01);
	// Read trimming parameters
	readMulti(BME280_REGISTER_DIG_T1, m_dig, 24);
	readMulti(BME280_REGISTER_DIG_H1, &m_dig[24], 1);
	readMulti(BME280_REGISTER_DIG_H2, &m_dig[25], 7);
	// Set humidity oversampling to 1
	write8(BME280_REGISTER_CONTROLHUMID, BME280_OSS_1);
	// Set temperature and pressure oversampling to 1
	write8(BME280_REGISTER_CONTROL, (BME280_OSS_1 << 5) | (BME280_OSS_1 << 2) | BME280_SLEEP_MODE);
	return 0;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC
// Code based on calibration algorithms provided by Bosch
static int16_t compensate_temperature(int32_t adc_T) {
	int32_t var1, var2;
	uint16_t dig_T1 = (m_dig[1] << 8) | m_dig[0];
	int16_t dig_T2 = (m_dig[3] << 8) | m_dig[2];
	int16_t dig_T3 = (m_dig[5] << 8) | m_dig[4];
	var1 = (int32_t)((adc_T >> 3) - ((int32_t)dig_T1 << 1));
    var1 = (var1 * ((int32_t)dig_T2)) >> 11;
    var2 = (int32_t)((dig_T1 >> 4) - ((int32_t)dig_T1));
    var2 = (((var2 * var2) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	return (int16_t)((t_fine * 5 + 128) >> 8);
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
// Code based on calibration algorithms provided by Bosch
static uint32_t compensate_pressure(int32_t adc_P) {
	int32_t var1, var2;
	uint32_t p;
	uint16_t dig_P1 = (m_dig[7] << 8) | m_dig[6];
	int16_t dig_P2 = (m_dig[9] << 8) | m_dig[8];
	int16_t dig_P3 = (m_dig[11] << 8) | m_dig[10];
	int16_t dig_P4 = (m_dig[13] << 8) | m_dig[12];
	int16_t dig_P5 = (m_dig[15] << 8) | m_dig[14];
	int16_t dig_P6 = (m_dig[17] << 8) | m_dig[16];
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (var1 == 0) return 0;  // avoid exception caused by division by zero
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / ((uint32_t)var1)) * 2;
    int16_t dig_P7 = (m_dig[19] << 8) | m_dig[18];
    int16_t dig_P8 = (m_dig[21] << 8) | m_dig[20];
    int16_t dig_P9 = (m_dig[23] << 8) | m_dig[22];
    var1 = (((int32_t)dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
    return (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format. Output value of “47445” represents 47445/1024 = 46.333 %RH
// Code based on calibration algorithms provided by Bosch
static uint32_t compensate_humidity(int32_t adc_H) {
	int32_t var1, var2, var3, var4, var5;
	uint8_t dig_H1 = m_dig[24];
	int16_t dig_H2 = (m_dig[26] << 8) | m_dig[25];
	uint8_t dig_H3 = m_dig[27];
	int16_t dig_H4 = ((int8_t)m_dig[28] * 16) | (0x0F & m_dig[29]);
	int16_t dig_H5 = ((int8_t)m_dig[30] * 16) | ((m_dig[29] >> 4) & 0x0F);
	int8_t dig_H6 = m_dig[31];
	var1 = (int32_t)t_fine - (int32_t)76800;
	var2 = adc_H << 14;
	var3 = ((int32_t)dig_H4) << 20;
	var4 = ((int32_t)dig_H5) * var1;
	var5 = ((var2 - var3 - var4 + (int32_t)16384) >> 15);
	var2 = (var1 * ((int32_t)dig_H6)) >> 10;
	var3 = (var1 * ((int32_t)dig_H3)) >> 11;
	var4 = ((var2 * (var3 + (int32_t)32768)) >> 10) + (int32_t)2097152;
	var2 = (var4 * ((int32_t)dig_H2) + 8192) >> 14;
	var3 = var5 * var2;
	var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
	var5 = var3 - ((var4 * ((int32_t)dig_H1)) >> 4);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	return (uint32_t)(var5 >> 12);
}

void bme280_get_sensor_data(int16_t *temperature, uint32_t *pressure, uint32_t *humidity) {
	uint8_t data[8];
	// Set to forced mode, i.e. "take next measurement"
	write8(BME280_REGISTER_CONTROL, (BME280_OSS_1 << 5) | (BME280_OSS_1 << 2) | BME280_FORCED_MODE);
	// Wait until measurement has been completed
	while (read8(BME280_REGISTER_STATUS) & 0x08);
	// Perform burst read
	readMulti(BME280_REGISTER_PRESSUREDATA, data, sizeof(data));
	*temperature = compensate_temperature(((uint32_t)data[3] << 12) | ((uint16_t)data[4] << 4) | (data[5] >> 4));
	*pressure = compensate_pressure(((uint32_t)data[0] << 12) | ((uint16_t)data[1] << 4) | (data[2] >> 4));
	*humidity = compensate_humidity(((uint16_t)data[6] << 8) | data[7]);
}
