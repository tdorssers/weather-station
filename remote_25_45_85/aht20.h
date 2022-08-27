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

// Non-reversed CRC-8 algorithm with 1 + x^4 + x^5 + x^8 (0x31) polynomial
static __inline__ uint8_t _crc8_update(uint8_t __crc, uint8_t __data) {
	uint8_t __i, __pattern;
	__asm__ __volatile__ (
	"    eor    %0, %4" "\n\t"
	"    ldi    %1, 8" "\n\t"
	"    ldi    %2, 0x31" "\n\t"
	"1:  lsl    %0" "\n\t"
	"    brcc   2f" "\n\t"
	"    eor    %0, %2" "\n\t"
	"2:  dec    %1" "\n\t"
	"    brne   1b" "\n\t"
	: "=r" (__crc), "=d" (__i), "=d" (__pattern)
	: "0" (__crc), "r" (__data));
	return __crc;
}

void aht20_init();
uint8_t aht20_get(uint16_t *humid, int16_t *temperature);

#endif /* AHT20_H_ */