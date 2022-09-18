/*
 * XPT2046 / ADS7843 Driver
 *
 * Created: 3-9-2022 15:37:15
 *  Author: Tim Dorssers
 *
 * PB3 -> T_DIN/MOSI
 * PB4 -> T_DO/MISO
 * PB5 -> T_CLK/SCK
 * PD2 -> T_IRQ
 * PD3 -> T_CS
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include "xpt2046.h"

uint16_t ts_x, ts_y;
uint16_t ts_xMin=0, ts_xMax=0;
uint16_t ts_yMin=0, ts_yMax=0;

#define map(x,in_min,in_max,out_min,out_max) (((x)-(in_min))*((out_max)-(out_min))/((in_max)-(in_min))+(out_min))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define spi_begin() PORTD &= ~_BV(PD3) // CS low
#define spi_end() PORTD |= _BV(PD3) // CS high

// Write to the SPI bus (MOSI pin) and also receive (MISO pin)
uint8_t spi_transfer(uint8_t data) {
	SPDR = data;
	while (!(SPSR & _BV(SPIF))) ; // wait
	return SPDR;
}

inline uint16_t spi_transfer16(uint16_t data) {
	union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;
	t.val = data;
	t.msb = spi_transfer(t.msb);
	t.lsb = spi_transfer(t.lsb);
	return t.val;
}

void xpt2046_init(void) {
	DDRD |= _BV(PD3); // CS output
	DDRB |= _BV(PB3) | _BV(PB5); // MOSI and SCK as output
	SPCR = _BV(SPE) | _BV(MSTR); // Mode 0, fOsc/4
	SPSR |= _BV(SPI2X); // Double clock rate
	spi_begin();
	// Issue a throw-away read, with power-down enabled (PD{1,0} == 0b00)
	// Otherwise, ADC is disabled
	spi_transfer(CTRL_HI_Y | CTRL_LO_SER);
	spi_transfer16(0);  // Flush, just to be sure
	spi_end();
}

uint16_t _readLoop(uint8_t ctrl, uint8_t max_samples) {
	uint16_t prev, cur = 0xffff;
	do {
		prev = cur;
		cur = spi_transfer16(ctrl);  // 16 clocks -> 12-bits (zero-padded at end)
	} while (prev != cur && max_samples--);
	return cur;
}

// Implementation based on TI Technical Note http://www.ti.com/lit/an/sbaa036/sbaa036.pdf
static void xpt2046_getRaw(uint16_t *vi, uint16_t *vj, uint8_t max_samples) {
	spi_begin();
	spi_transfer(CTRL_HI_X | CTRL_LO_DFR);  // Send first control byte
	*vi = _readLoop(CTRL_HI_X | CTRL_LO_DFR, max_samples);
	*vj = _readLoop(CTRL_HI_Y | CTRL_LO_DFR, max_samples);
	// Turn off ADC by issuing one more read (throwaway)
	// This needs to be done, because PD=0b11 (needed for MODE_DFR) will disable PENIRQ
	spi_transfer(0);  // Maintain 16-clocks/conversion; _readLoop always ends after issuing a control byte
	spi_transfer(CTRL_HI_Y | CTRL_LO_SER);
	spi_transfer16(0);  // Flush last read, just to be sure
	spi_end();
}

#define swap(a,b) {uint16_t t = a; a = b; b = t;}

void xpt2046_getPosition(uint8_t max_samples, uint8_t rotation) {
	if (!xpt2046_isTouching()) {
		ts_x = ts_y = 0xffff;
		return;
	}

	uint16_t vi, vj;
	xpt2046_getRaw(&vi, &vj, max_samples);

	if (ts_xMin == 0 && ts_xMax == 0)
		ts_xMin = ts_xMax = vi;
	if (ts_yMin == 0 && ts_yMax == 0)
		ts_yMin = ts_yMax = vj;
	ts_xMin = min(vi, ts_xMin);
	ts_xMax = max(vi, ts_xMax);
	ts_yMin = min(vj, ts_yMin);
	ts_yMax = max(vj, ts_yMax);

	ts_x = map((int32_t)vi, ts_xMin, ts_xMax, 0, LCD_WIDTH - 1);
	ts_y = map((int32_t)vj, ts_yMin, ts_yMax, 0, LCD_HEIGHT - 1);

	switch (rotation) {
		case 0:
			swap(ts_x, ts_y);
			ts_y = LCD_WIDTH - 1 - ts_y;
			break;
		case 1:
			ts_x = LCD_WIDTH - 1 - ts_x;
			ts_y = LCD_HEIGHT - 1 - ts_y;
			break;
		case 2:
			swap(ts_x, ts_y);
			ts_x = LCD_HEIGHT - 1 - ts_x;
			break;
		case 3:
			break;
	}
}
