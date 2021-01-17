/*
 * AM2320 I2C driver
 *
 * Created: 12-12-2020 15:36:24
 *  Author: Tim Dorssers
 */ 


#ifndef AM2320_H_
#define AM2320_H_

#define AM2320_ADDR         0xB8
#define AM2320_CMD_READREG  0x03 // reading register data
#define AM2320_CMD_WRITEREG 0x10 // write multiple registers
#define AM2320_REG_TEMP_H   0x02 // temperature register address
#define AM2320_REG_HUMID_H  0x00 // humidity register address

uint8_t am2320_get(uint16_t *humid, int16_t *temp);

#endif /* AM2320_H_ */