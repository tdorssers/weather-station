/*
 * BME280 I2C/SPI Driver
 *
 * Created: 29-11-2020 16:00:30
 *  Author: Tim Dorssers
 */ 


#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>

// If SDO pin is connected to VCC
//#define BME280_ADDRESS (0x77 << 1)

// If SDO pin is connected to GND
#define BME280_ADDRESS (0x76 << 1)

#define BME280_SPI_WRITE 0x7F
#define BME280_SPI_READ  0x80

#define    BME280_REGISTER_DIG_T1   0x88
#define    BME280_REGISTER_DIG_T2   0x8A
#define    BME280_REGISTER_DIG_T3   0x8C
#define    BME280_REGISTER_DIG_P1   0x8E
#define    BME280_REGISTER_DIG_P2   0x90
#define    BME280_REGISTER_DIG_P3   0x92
#define    BME280_REGISTER_DIG_P4   0x94
#define    BME280_REGISTER_DIG_P5   0x96
#define    BME280_REGISTER_DIG_P6   0x98
#define    BME280_REGISTER_DIG_P7   0x9A
#define    BME280_REGISTER_DIG_P8   0x9C
#define    BME280_REGISTER_DIG_P9   0x9E
#define    BME280_REGISTER_DIG_H1   0xA1
#define    BME280_REGISTER_DIG_H2   0xE1
#define    BME280_REGISTER_DIG_H3   0xE3
#define    BME280_REGISTER_DIG_H4   0xE4
#define    BME280_REGISTER_DIG_H5   0xE5
#define    BME280_REGISTER_DIG_H6   0xE7

#define    BME280_REGISTER_CHIPID       0xD0
#define    BME280_REGISTER_VERSION      0xD1
#define    BME280_REGISTER_SOFTRESET    0xE0
#define    BME280_REGISTER_CAL26        0xE1
#define    BME280_REGISTER_CONTROLHUMID 0xF2
#define    BME280_REGISTER_STATUS       0xF3
#define    BME280_REGISTER_CONTROL      0xF4
#define    BME280_REGISTER_CONFIG       0xF5
#define    BME280_REGISTER_PRESSUREDATA 0xF7
#define    BME280_REGISTER_TEMPDATA     0xFA
#define    BME280_REGISTER_HUMIDDATA    0xFD

// Oversampling settings
#define BME280_OSS_DISABLED 0x00
#define BME280_OSS_1        0x01
#define BME280_OSS_2        0x02
#define BME280_OSS_4        0x03
#define BME280_OSS_8        0x04
#define BME280_OSS_16       0x05

// Sensor power modes
#define BME280_SLEEP_MODE   0x00
#define BME280_FORCED_MODE  0x01
#define BME280_NORMAL_MODE  0x03

// IIR filter coefficient
#define BME280_IIR_DISABLED 0x00
#define BME280_IIR_2        0x01
#define BME280_IIR_4        0x02
#define BME280_IIR_8        0x03
#define BME280_IIR_16       0x04

// Standby duration in milliseconds
#define BME280_T_SB_MS_0_5  0x00
#define BME280_T_SB_MS_10   0x06
#define BME280_T_SB_MS_20   0x07
#define BME280_T_SB_MS_62_5 0x01
#define BME280_T_SB_MS_125  0x02
#define BME280_T_SB_MS_250  0x03
#define BME280_T_SB_MS_500  0x04
#define BME280_T_SB_MS_1000 0x05

uint8_t bme280_init(void);
void bme280_get_sensor_data(int16_t *temperature, uint32_t *pressure, uint32_t *humidity);

#endif /* BME280_H_ */