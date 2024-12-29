/*
 * Non-reversed CRC-8 algorithm with 1 + x^4 + x^5 + x^8 (0x31) polynomial
 *
 * Created: 21/12/2024 22:16:39
 *  Author: Tim Dorssers
 */ 


#ifndef CRC8_H_
#define CRC8_H_

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

#endif /* CRC8_H_ */