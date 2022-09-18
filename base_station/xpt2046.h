/*
 * XPT2046 / ADS7843 Driver
 *
 * Created: 3-9-2022 15:37:40
 *  Author: Tim Dorssers
 */ 


#ifndef XPT2046_H_
#define XPT2046_H_

#include <stdint.h>

#define CTRL_LO_DFR 0b0011
#define CTRL_LO_SER 0b0100
#define CTRL_HI_X (0b1001 << 4)
#define CTRL_HI_Y (0b1101 << 4)
#define LCD_WIDTH 320
#define LCD_HEIGHT 240

extern uint16_t ts_x, ts_y;
extern uint16_t ts_xMin, ts_xMax;
extern uint16_t ts_yMin, ts_yMax;

#define xpt2046_isTouching() bit_is_clear(PIND, PD2)

void xpt2046_init(void);
void xpt2046_getPosition(uint8_t max_samples, uint8_t rotation);

#endif /* XPT2046_H_ */