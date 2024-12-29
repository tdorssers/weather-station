/*
 * AHT20 Driver
 *
 * Created: 25-8-2022 20:50:15
 *  Author: Tim Dorssers
 */ 


#ifndef AHT20_H_
#define AHT20_H_

#define AHTXX_ADDRESS                     0x38  //AHT15/AHT20/AHT21/AHT25 I2C address
#define AHT2X_INIT_REG                    0xBE  //initialization register
#define AHTXX_STATUS_REG                  0x71  //read status byte register
#define AHTXX_START_MEASUREMENT_REG       0xAC  //start measurement register
#define AHTXX_SOFT_RESET_REG              0xBA  //soft reset register

uint8_t aht20_get(uint16_t *humid, int16_t *temperature);

#endif /* AHT20_H_ */