/*
 * BME280 I2C Driver
 *
 * Created: 29-11-2020 16:00:04
 *  Author: Tim Dorssers
 */ 

#include "bme280.h"
#include "i2cmaster.h"

bme280_calib_data calib_data;

void write8(uint8_t reg, uint8_t value) {
	i2c_start(BME280_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
}

uint8_t read8(uint8_t reg) {
	uint8_t value;
	i2c_start(BME280_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(BME280_ADDRESS+I2C_READ);
	value = i2c_readNak();
	i2c_stop();
	return value;
}

uint16_t read16(uint8_t reg) {
	uint16_t value;
	i2c_start(BME280_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(BME280_ADDRESS+I2C_READ);
	value = (uint16_t)i2c_readAck() << 8;
	value |= i2c_readNak();
	i2c_stop();
	return value;
}

uint16_t read16_LE(uint8_t reg) {
	uint16_t temp = read16(reg);
	return (temp >> 8) | (temp << 8);
}

uint32_t read24(uint8_t reg) {
	uint32_t value;
	i2c_start(BME280_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(BME280_ADDRESS+I2C_READ);
	value = (uint32_t)i2c_readAck() << 12; // MSB
	value |= (uint32_t)i2c_readAck() << 4; // LSB
	value |= i2c_readNak() >> 4 & 0x0f;    // XLSB
	i2c_stop();
	return value;
}

static void bme280_readCoefficients(void) {
	calib_data.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
	calib_data.dig_T2 = (int16_t)read16_LE(BME280_REGISTER_DIG_T2);
	calib_data.dig_T3 = (int16_t)read16_LE(BME280_REGISTER_DIG_T3);
	calib_data.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
	calib_data.dig_P2 = (int16_t)read16_LE(BME280_REGISTER_DIG_P2);
	calib_data.dig_P3 = (int16_t)read16_LE(BME280_REGISTER_DIG_P3);
	calib_data.dig_P4 = (int16_t)read16_LE(BME280_REGISTER_DIG_P4);
	calib_data.dig_P5 = (int16_t)read16_LE(BME280_REGISTER_DIG_P5);
	calib_data.dig_P6 = (int16_t)read16_LE(BME280_REGISTER_DIG_P6);
	calib_data.dig_P7 = (int16_t)read16_LE(BME280_REGISTER_DIG_P7);
	calib_data.dig_P8 = (int16_t)read16_LE(BME280_REGISTER_DIG_P8);
	calib_data.dig_P9 = (int16_t)read16_LE(BME280_REGISTER_DIG_P9);
	calib_data.dig_H1 = read8(BME280_REGISTER_DIG_H1);
	calib_data.dig_H2 = (int16_t)read16_LE(BME280_REGISTER_DIG_H2);
	calib_data.dig_H3 = read8(BME280_REGISTER_DIG_H3);
	calib_data.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
	calib_data.dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
	calib_data.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

void bme280_init() {
	i2c_init();
	if (read8(BME280_REGISTER_CHIPID) != 0x60) return;
	bme280_readCoefficients();
	// Set humidity oversampling to 1
	write8(BME280_REGISTER_CONTROLHUMID, BME280_OSS_1);
	// Set temperature and pressure oversampling to 1
	write8(BME280_REGISTER_CONTROL, (BME280_OSS_1 << 5) | (BME280_OSS_1 << 2) | BME280_SLEEP_MODE);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC
static int16_t compensate_temperature(uint32_t adc_T) {
	int32_t var1, var2;
	var1 = (int32_t)((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1));
    var1 = (var1 * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (int32_t)((calib_data.dig_T1 >> 4) - ((int32_t)calib_data.dig_T1));
    var2 = (((var2 * var2) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
	calib_data.t_fine = var1 + var2;
	return (int16_t)((calib_data.t_fine * 5 + 128) >> 8);
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
static uint32_t compensate_pressure(int32_t adc_P) {
	int32_t var1, var2;
	uint32_t p;
    var1 = (((int32_t)calib_data.t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_data.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_data.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_data.dig_P4) << 16);
    var1 = (((calib_data.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_data.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib_data.dig_P1)) >> 15);
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (var1 == 0) return 0;  // avoid exception caused by division by zero
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / ((uint32_t)var1)) * 2;
    var1 = (((int32_t)calib_data.dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_data.dig_P8)) >> 13;
    return (uint32_t)((int32_t)p + ((var1 + var2 + calib_data.dig_P7) >> 4));
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format. Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t compensate_humidity(int32_t adc_H) {
	int32_t var1, var2, var3, var4, var5;
	var1 = (int32_t)calib_data.t_fine - (int32_t)76800;
	var2 = adc_H << 14;
	var3 = ((int32_t)calib_data.dig_H4) << 20;
	var4 = ((int32_t)calib_data.dig_H5) * var1;
	var5 = ((var2 - var3 - var4 + (int32_t)16384) >> 15);
	var2 = (var1 * ((int32_t)calib_data.dig_H6)) >> 10;
	var3 = (var1 * ((int32_t)calib_data.dig_H3)) >> 11;
	var4 = ((var2 * (var3 + (int32_t)32768)) >> 10) + (int32_t)2097152;
	var2 = (var4 * ((int32_t)calib_data.dig_H2) + 8192) >> 14;
	var3 = var5 * var2;
	var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
	var5 = var3 - ((var4 * ((int32_t)calib_data.dig_H1)) >> 4);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	return (uint32_t)(var5 >> 12);
}

void bme280_get_sensor_data(int16_t *temperature, uint32_t *pressure, uint32_t *humidity) {
	write8(BME280_REGISTER_CONTROL, (BME280_OSS_1 << 5) | (BME280_OSS_1 << 2) | BME280_FORCED_MODE);
	*temperature = compensate_temperature(read24(BME280_REGISTER_TEMPDATA));
	*pressure = compensate_pressure(read24(BME280_REGISTER_PRESSUREDATA));
	*humidity = compensate_humidity(read16(BME280_REGISTER_HUMIDDATA));
}
