/*
 * ILI9341 Driver
 *
 * Created: 14-12-2019 20:03:53
 *  Author: Tim Dorssers
 *
 * Speed optimized ILI9341 driver for AVR
 * Implements graphics drawing primitives and openGLCD library fonts
 *
 * PB3 -> MOSI
 * PB4 -> MISO
 * PB5 -> SCK
 * PD6 -> DC
 * PD7 -> CS
 * PD4 -> RST
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdlib.h>	// abs()
#include "ili9341.h"

const uint8_t *font;
uint8_t textsize = 1;
uint16_t cursor_x = 0, cursor_y = 0, font_size;
uint16_t textcolor = ILI9341_WHITE, textbgcolor = ILI9341_WHITE;
uint16_t _width = ILI9341_TFTWIDTH, _height = ILI9341_TFTHEIGHT;

static const uint8_t init_commands[] PROGMEM = {
	4, 0xEF, 0x03, 0x80, 0x02,
	4, 0xCF, 0x00, 0XC1, 0X30, // Power Control B
	5, 0xED, 0x64, 0x03, 0X12, 0X81, // Power On Sequence Control
	4, 0xE8, 0x85, 0x00, 0x78, // Driver Timing Control A
	6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02, // Power Control A
	2, 0xF7, 0x20, // Pump Ratio Control
	3, 0xEA, 0x00, 0x00, // Driver Timing Control B
	2, ILI9341_PWCTR1, 0x23, // Power control
	2, ILI9341_PWCTR2, 0x10, // Power control
	3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
	2, ILI9341_VMCTR2, 0x86, // VCM control 2
	2, ILI9341_MADCTL, 0x48, // Memory Access Control
	2, ILI9341_PIXFMT, 0x55, // Pixel Format
	3, ILI9341_FRMCTR1, 0x00, 0x18, // Frame Rate Control
	4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
	2, 0xF2, 0x00, // 3Gamma Function Disable
	2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
	16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
	0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Positive Gamma
	16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
	0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Negative Gamma
	0
};

#define FAST_SPI // Enable fast SPI writing optimizations for Fck/2 double speed mode

#define spi_begin() PORTD &= ~_BV(PD7) // CS low
#define spi_end() PORTD |= _BV(PD7) // CS high

// Send 8 bit value
inline void spiwrite(uint8_t data) {
#ifdef FAST_SPI
	SPDR = data;
	asm volatile(
		"rjmp .+0\n\t"				// wait 8*2+1 = 17 cycles
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"nop\n\t"
	);
#else
	SPDR = data;
	while (!(SPSR & _BV(SPIF)));
#endif
}

// Send 16 bit value multiple times
inline void spiwrite16(uint16_t data, uint16_t count) {
#ifdef FAST_SPI
	asm volatile (
		"sbiw %[count],0\n\t"		// test count
		"breq 2f\n\t"				// if == 0 then done
		"1: out %[spdr],%[hi]\n\t"	// write SPI data (18 cycles until next write)
		"rjmp .+0\n\t"				// wait 8*2+1 = 17 cycles
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"nop\n\t"
		"out %[spdr],%[lo]\n\t"		// write SPI data (18 cycles until next write)
		"rjmp .+0\n\t"				// wait 6*2+1 = 13 cycles + sbiw + brne
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"nop\n\t"
		"sbiw %[count],1\n\t"		// decrement count
		"brne 1b\n\t"				// if != 0 then loop
		"2:\n\t"
		: [count] "+w" (count)
		: [spdr] "I" (_SFR_IO_ADDR(SPDR)), [lo] "r" ((uint8_t)data), [hi] "r" ((uint8_t)(data>>8))
	);
#else
	while (count--) {
		spiwrite(data >> 8);
		spiwrite(data);
	}
#endif
}

// Send buffer of 16 bit values
inline void spicopy16(uint8_t *data, uint16_t count)
{
#ifdef FAST_SPI
	uint8_t lo = 0, hi = 0;
	asm volatile (
		"1: ld %[hi],%a[data]+\n\t"
		"ld %[lo],%a[data]+\n\t"
		"out %[spdr],%[lo]\n\t"
		"rjmp .+0\n\t"              // wait 8*2+1 = 17 cycles
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"nop\n\t"
		"out %[spdr],%[hi]\n\t"
		"rjmp .+0\n\t"              // wait 4*2+1 = 9 cycles + sbiw + brne + ld + ld
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"rjmp .+0\n\t"
		"nop\n\t"
		"sbiw %[count],1\n\t"
		"brne 1b\n\t"
		: [count] "+w" (count)
		: [spdr] "I" (_SFR_IO_ADDR(SPDR)), [data] "e" (data), [lo] "r" (lo), [hi] "r" (hi)
	);
#else
	while (count--) {
		spiwrite(*(data + 1));
		spiwrite(*data);
		data += 2;
	}
#endif
}

// Retrieve 8 bit value
inline uint8_t spiread(void) {
	while (!(SPSR & _BV(SPIF)));
	return SPDR;
}

// Send command and deselect
void writecommand(uint8_t com) {
	PORTD &= ~_BV(PD6); // set DC low to send command
	spi_begin();
	spiwrite(com);
	PORTD |= _BV(PD6); // set DC high for data
	spi_end();
}

// Send command
inline void writecommand_cont(uint8_t com) {
	PORTD &= ~_BV(PD6); // set DC low to send command
	spi_begin();
	spiwrite(com);
	PORTD |= _BV(PD6); // set DC high for data
}

// Send 8 bit data and deselect
void writedata8(uint8_t data) {
	spi_begin();
	spiwrite(data);
	spi_end();
}

// Send 16 bit data and deselect
void writedata16(uint16_t data) {
	spi_begin();
	spiwrite(data >> 8);
	spiwrite(data);
	spi_end();
}

// Send 16 bit data
inline void writedata16_cont(uint16_t data) {
	spiwrite(data >> 8);
	spiwrite(data);
}

// Retrieve 8 bit data
static uint8_t read8_cont(void) {
	spiwrite(ILI9341_NOP);
	return spiread();
}

// Initialize
void ili9341_init(void) {
	PORTD |= _BV(PD4); // set RST high for normal operation
	DDRD |= _BV(PD4) | _BV(PD6) | _BV(PD7); // RST, DC and CS as output
	// Initialize SPI
	DDRB |= _BV(PB2) | _BV(PB3) | _BV(PB5); // SS, MOSI and SCK as output
	SPCR = _BV(SPE) | _BV(MSTR); // Mode 0, fOsc/4
	SPSR |= _BV(SPI2X); // Double clock rate
	PORTD |= _BV(PD7); // CS low during startup
	// Hardware reset
	PORTD &= ~_BV(PD4);
	_delay_ms(5);
	PORTD |= _BV(PD4);
	_delay_ms(120);
	// Soft reset
	writecommand(ILI9341_SWRESET);
	_delay_ms(5);
	// Initialize display
	const uint8_t *addr = init_commands;
	while (1) {
		uint8_t count = pgm_read_byte(addr++);
		if (count-- == 0) break;
		writecommand(pgm_read_byte(addr++));
		while (count-- > 0) {
			writedata8(pgm_read_byte(addr++));
		}
	}
	writecommand(ILI9341_SLPOUT);    // Exit Sleep
	_delay_ms(5);
	writecommand(ILI9341_DISPON);    // Display on
}

// 8-bit read mode for read ID or register commands, 24 and 32 bit is not supported
uint8_t ili9341_readcommand8(uint8_t com) {
	if (com < ILI9341_RDMODE || (com > ILI9341_RDSELFDIAG && com < ILI9341_RDID1) || com > ILI9341_RDID4) return 0;
	writecommand_cont(com);
	uint8_t result = read8_cont();
	spi_end();
	return result;
}

//set coordinate for print or other function
void ili9341_setaddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	writecommand_cont(ILI9341_CASET);
	writedata16_cont(x1);
	writedata16_cont(x2);
	writecommand_cont(ILI9341_PASET);
	writedata16_cont(y1);
	writedata16_cont(y2);
	writecommand_cont(ILI9341_RAMWR); // memory write
	spi_end();
}

// Reads one pixel/color from the TFT's GRAM
uint16_t ili9341_readPixel(uint16_t x, uint16_t y) {
	ili9341_setaddress(x,y,x+1,y+1);
	writecommand_cont(ILI9341_RAMRD);
	read8_cont(); // dummy byte
	uint8_t red = read8_cont();
	uint8_t green = read8_cont();
	uint8_t blue = read8_cont();
	spi_end();
	return color565(red, green, blue);
}

// Read multiple pixels
void ili9341_readRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *pcolors) {
	ili9341_setaddress(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMRD);
	read8_cont(); // dummy byte
	uint16_t c = w * h;
	while (c--) {
		uint8_t red = read8_cont();
		uint8_t green = read8_cont();
		uint8_t blue = read8_cont();
		*pcolors++ = color565(red, green, blue);
	}
	spi_end();
}

// Write multiple pixels
void ili9341_writeRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pcolors) {
	ili9341_setaddress(x, y, x+w-1, y+h-1);
	spi_begin();
	spicopy16((uint8_t *)pcolors, w * h);
	spi_end();
}

//clear LCD and fill with color
void ili9341_fillScreen(uint16_t color) {
	ili9341_fillrect(0,0,_width,_height,color);
}

//draw pixel
void ili9341_drawpixel(uint16_t x,uint16_t y,uint16_t color) {
	if ((x >= _width) || (y >= _height)) return;
	ili9341_setaddress(x,y,x+1,y+1);
	spi_begin();
	writedata16_cont(color);
	spi_end();
}

//draw vertical line
void ili9341_drawvline(uint16_t x,uint16_t y,uint16_t h,uint16_t color) {
	if ((x >= _width) || (y >= _height)) return;
	if ((y+h-1) >= _height)
		h = _height-y;
	ili9341_setaddress(x,y,x,y+h-1);
	spi_begin();
	spiwrite16(color, h);
	spi_end();
}

//draw horizontal line
void ili9341_drawhline(uint16_t x,uint16_t y,uint16_t w,uint16_t color) {
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)
		w = _width-x;
	ili9341_setaddress(x,y,x+w-1,y);
	spi_begin();	
	spiwrite16(color, w);
	spi_end();
}

//draw color filled rectangle
void ili9341_fillrect(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t color) {
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)
		w = _width-x;
	if ((y+h-1) >= _height)
		h = _height-y;
	ili9341_setaddress(x, y, x+w-1, y+h-1);
	spi_begin();
	while (h--)
		spiwrite16(color, w);
	spi_end();
}

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

//rotate screen at desired orientation
void ili9341_setRotation(uint8_t m) {
	writecommand(ILI9341_MADCTL);
	switch (m % 4) {
		case 0:
			writedata8(MADCTL_MX | MADCTL_BGR);
			_width = ILI9341_TFTWIDTH;
			_height = ILI9341_TFTHEIGHT;
			break;
		case 1:
			writedata8(MADCTL_MV | MADCTL_BGR);
			_width = ILI9341_TFTHEIGHT;
			_height = ILI9341_TFTWIDTH;
			break;
		case 2:
			writedata8(MADCTL_MY | MADCTL_BGR);
			_width = ILI9341_TFTWIDTH;
			_height = ILI9341_TFTHEIGHT;
			break;
		case 3:
			writedata8(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
			_width = ILI9341_TFTHEIGHT;
			_height = ILI9341_TFTWIDTH;
			break;
	}
}

void ili9341_invertDisplay(bool i) {
	writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}

void ili9341_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	int16_t x = x1-x0, y = y1-y0;
	int16_t dx = abs(x), dy = -abs(y); 
	int8_t sx = x0<x1 ? 1 : -1;
	int8_t sy = y0<y1 ? 1 : -1;
	int16_t err = dx+dy, e2;			// error value e_xy
	for (;;) {
		ili9341_drawpixel(x0,y0,color);
		e2 = 2*err;
		if (e2 >= dy) {			// e_xy+e_x > 0
			if (x0 == x1) break;
			err += dy;
			x0 += sx;
		}
		if (e2 <= dx) {	 		// e_xy+e_y < 0
			if (y0 == y1) break;
			err += dx;
			y0 += sy;
		}
	}
}

void ili9341_drawRect(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t color) {
	ili9341_drawhline(x0, y0, width, color);
	ili9341_drawhline(x0, y0+height-1, width, color);
	ili9341_drawvline(x0, y0, height, color);
	ili9341_drawvline(x0+width-1, y0, height, color);
}

void ili9341_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
	//ili9341_drawRoundRect(x0-r, y0-r, 2*r+1, 2*r+1, r, color);
	int16_t x = -r, y = 0, err = 2-2*r, e2;
	do {
		ili9341_drawpixel(x0-x, y0+y,color);
		ili9341_drawpixel(x0+x, y0+y,color);
		ili9341_drawpixel(x0+x, y0-y,color);
		ili9341_drawpixel(x0-x, y0-y,color);
		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x)
			err += ++x*2+1;
	} while (x <= 0);
}

void ili9341_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
	int16_t x = -r, y = 0, err = 2-2*r, e2;
	do {
		ili9341_drawvline(x0-x, y0-y, 2*y, color);
		ili9341_drawvline(x0+x, y0-y, 2*y, color);
		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x)
			err += ++x*2+1;
	} while (x <= 0);
}

// if true, TFT will be blank (white), displays frame buffer is unaffected
// (you can write to it without showing content on the screen)
void ili9341_display(bool d) {
	writecommand(d ? ILI9341_DISPON : ILI9341_DISPOFF);
}

// puts display in/out of sleep mode
void ili9341_sleep(bool s) {
	writecommand(s ? ILI9341_SLPIN : ILI9341_SLPOUT);
	_delay_ms(120);
}

void ili9341_idle(bool i) {
	writecommand(i ? ILI9341_IDMON : ILI9341_IDMOFF);
}

void ili9341_setupScrollArea(uint16_t tfa, uint16_t bfa) {
	writecommand(ILI9341_VSCRDEF); // Vertical scroll definition
	writedata16(tfa);              // Top Fixed Area line count
	writedata16(_height-tfa-bfa);  // Vertical Scrolling Area line count
	writedata16(bfa);              // Bottom Fixed Area line count
}

void ili9341_scrollAddress(uint16_t vsp) {
	writecommand(ILI9341_VSCRSADD); // Vertical scrolling pointer
	writedata16(vsp);
}

void ili9341_drawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color) {
	if (rx<2 || ry<2) return;
	int16_t x, y;
	int32_t rx2 = rx * rx;
	int32_t ry2 = ry * ry;
	int32_t fx2 = 4 * rx2;
	int32_t fy2 = 4 * ry2;
	int32_t s;
	for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++) {
		ili9341_drawpixel(x0 + x, y0 + y, color);
		ili9341_drawpixel(x0 - x, y0 + y, color);
		ili9341_drawpixel(x0 - x, y0 - y, color);
		ili9341_drawpixel(x0 + x, y0 - y, color);
		if (s >= 0) {
			s += fx2 * (1 - y);
			y--;
		}
		s += ry2 * ((4 * x) + 6);
	}
	for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++) {
		ili9341_drawpixel(x0 + x, y0 + y, color);
		ili9341_drawpixel(x0 - x, y0 + y, color);
		ili9341_drawpixel(x0 - x, y0 - y, color);
		ili9341_drawpixel(x0 + x, y0 - y, color);
		if (s >= 0) {
			s += fy2 * (1 - x);
			x--;
		}
		s += rx2 * ((4 * y) + 6);
	}
}

void ili9341_fillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color) {
	if (rx<2 || ry<2) return;
	int16_t x, y;
	int32_t rx2 = rx * rx;
	int32_t ry2 = ry * ry;
	int32_t fx2 = 4 * rx2;
	int32_t fy2 = 4 * ry2;
	int32_t s;
	for (x = 0, y = ry, s = 2*ry2+rx2*(1-2*ry); ry2*x <= rx2*y; x++) {
		ili9341_drawhline(x0 - x, y0 - y, x + x + 1, color);
		ili9341_drawhline(x0 - x, y0 + y, x + x + 1, color);
		if (s >= 0) {
			s += fx2 * (1 - y);
			y--;
		}
		s += ry2 * ((4 * x) + 6);
	}
	for (x = rx, y = 0, s = 2*rx2+ry2*(1-2*rx); rx2*y <= ry2*x; y++) {
		ili9341_drawhline(x0 - x, y0 - y, x + x + 1, color);
		ili9341_drawhline(x0 - x, y0 + y, x + x + 1, color);
		if (s >= 0) {
			s += fy2 * (1 - x);
			x--;
		}
		s += rx2 * ((4 * y) + 6);
	}
}

// Draw a triangle
void ili9341_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	ili9341_drawLine(x0, y0, x1, y1, color);
	ili9341_drawLine(x1, y1, x2, y2, color);
	ili9341_drawLine(x2, y2, x0, y0, color);
}

#define swap(a,b) {int16_t t = a; a = b; b = t;}
	
void ili9341_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	int16_t a, b, y, last;
	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1) {
		swap(y0, y1); swap(x0, x1);
	}
	if (y1 > y2) {
		swap(y2, y1); swap(x2, x1);
	}
	if (y0 > y1) {
		swap(y0, y1); swap(x0, x1);
	}
	if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if (x1 < a)      a = x1;
		else if (x1 > b) b = x1;
		if (x2 < a)      a = x2;
		else if (x2 > b) b = x2;
		ili9341_drawhline(a, y0, b - a + 1, color);
		return;
	}
	int16_t dx01 = x1 - x0,	dy01 = y1 - y0;
	int16_t dx02 = x2 - x0,	dy02 = y2 - y0;
	int16_t	dx12 = x2 - x1,	dy12 = y2 - y1;
	int16_t	sa = 0,	sb = 0;
	// For upper part of triangle, find line crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the line y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise line y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if (y1 == y2) last = y1;  // Include y1 line
	else          last = y1 - 1; // Skip it
	for (y = y0; y <= last; y++) {
		a   = x0 + sa / dy01;
		b   = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		if (a > b) swap(a, b);
		ili9341_drawhline(a, y, b - a + 1, color);
	}
	// For lower part of triangle, find line crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for (; y <= y2; y++) {
		a   = x1 + sa / dy12;
		b   = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if (a > b) swap(a, b);
		ili9341_drawhline(a, y, b - a + 1, color);
	}
}

#define MAXSIN 255

const uint8_t sinTab[91] PROGMEM = {
	0,4,8,13,17,22,26,31,35,39,44,48,53,57,61,65,70,74,78,83,87,91,95,99,103,
	107,111,115,119,123,127,131,135,138,142,146,149,153,156,160,163,167,170,173,
	177,180,183,186,189,192,195,198,200,203,206,208,211,213,216,218,220,223,225,
	227,229,231,232,234,236,238,239,241,242,243,245,246,247,248,249,250,251,251,
	252,253,253,254,254,254,254,254,255
};

int16_t fastSin(int16_t i) {
	while (i < 0) i += 360;
	while (i >= 360) i -= 360;
	if (i < 90)  return pgm_read_byte(&sinTab[i]);
	if (i < 180) return pgm_read_byte(&sinTab[180-i]);
	if (i < 270) return -pgm_read_byte(&sinTab[i-180]);
	return -pgm_read_byte(&sinTab[360-i]);
}

static int16_t fastCos(int16_t i) {
	return fastSin(i+90);
}

void ili9341_drawLineByAngle(uint16_t x, uint16_t y, int16_t angle, int16_t length, uint16_t color) {
	ili9341_drawLine(x, y, x + length * fastCos(angle - 90) / MAXSIN, y + length * fastSin(angle - 90) / MAXSIN, color);
}

// Draw an elliptical arc with given thickness (w) between start and end angle
void ili9341_fillArc(uint16_t cx, uint16_t cy, int16_t start, int16_t end, int16_t rx, int16_t ry, int16_t w, uint16_t color) {
	uint8_t seg = 5; // Segment size in degrees
	for(int16_t i=start; i<end; i+=seg) {
		int16_t sx = fastCos(i-90);
		int16_t sy = fastSin(i-90);
		int16_t xs0 = cx+sx*(rx-w)/MAXSIN;
		int16_t ys0 = cy+sy*(ry-w)/MAXSIN;
		int16_t xe0 = cx+sx*rx/MAXSIN;
		int16_t ye0 = cy+sy*ry/MAXSIN;
		sx = fastCos(i-90+seg);
		sy = fastSin(i-90+seg);
		int16_t xs1 = cx+sx*(rx-w)/MAXSIN;
		int16_t ys1 = cy+sy*(ry-w)/MAXSIN;
		int16_t xe1 = cx+sx*rx/MAXSIN;
		int16_t ye1 = cy+sy*ry/MAXSIN;
		ili9341_fillTriangle(xs0,ys0,xe0,ye0,xe1,ye1, color);
		ili9341_fillTriangle(xs1,ys1,xe1,ye1,xs0,ys0, color);
	}
}

// Draw a dashed circular arc with given thickness (w) between start and end angle
void ili9341_fillArcDashed(uint16_t cx, uint16_t cy, int16_t start, int16_t end, int16_t r, int16_t w, uint16_t color) {
	uint8_t seg = 5; // Segment size in degrees
	for(int16_t i=start; i<end; i+=2*seg) {
		int16_t sx = fastCos(i-90);
		int16_t sy = fastSin(i-90);
		int16_t xs0 = cx+sx*(r-w)/MAXSIN;
		int16_t ys0 = cy+sy*(r-w)/MAXSIN;
		int16_t xe0 = cx+sx*r/MAXSIN;
		int16_t ye0 = cy+sy*r/MAXSIN;
		sx = fastCos(i-90+seg);
		sy = fastSin(i-90+seg);
		int16_t xs1 = cx+sx*(r-w)/MAXSIN;
		int16_t ys1 = cy+sy*(r-w)/MAXSIN;
		int16_t xe1 = cx+sx*r/MAXSIN;
		int16_t ye1 = cy+sy*r/MAXSIN;
		ili9341_fillTriangle(xs0,ys0,xe0,ye0,xe1,ye1, color);
		ili9341_fillTriangle(xs1,ys1,xe1,ye1,xs0,ys0, color);
	}
}

void ili9341_drawRoundRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t radius, uint16_t color) {
	int16_t tSwitch = 3 - 2 * radius;
	uint16_t x1 = 0, y1 = radius;
	while (x1 <= y1) {
		// upper left corner
		ili9341_drawpixel(x+radius - x1, y+radius - y1, color);
		ili9341_drawpixel(x+radius - y1, y+radius - x1, color);
		// upper right corner
		ili9341_drawpixel(x+width-radius-1 + x1, y+radius - y1, color);
		ili9341_drawpixel(x+width-radius-1 + y1, y+radius - x1, color);
		// lower right corner
		ili9341_drawpixel(x+width-radius-1 + x1, y+height-radius-1 + y1, color);
		ili9341_drawpixel(x+width-radius-1 + y1, y+height-radius-1 + x1, color);
		// lower left corner
		ili9341_drawpixel(x+radius - x1, y+height-radius-1 + y1, color);
		ili9341_drawpixel(x+radius - y1, y+height-radius-1 + x1, color);
		if (tSwitch < 0) {
			tSwitch += (4 * x1 + 6);
		} else {
			tSwitch += (4 * (x1 - y1) + 10);
			y1--;
		}
		x1++;
	}
	ili9341_drawhline(x+radius, y, width-(2*radius), color);			// top
	ili9341_drawhline(x+radius, y+height-1, width-(2*radius), color);	// bottom
	ili9341_drawvline(x, y+radius, height-(2*radius), color);			// left
	ili9341_drawvline(x+width-1, y+radius, height-(2*radius), color);	// right
}

void ili9341_fillRoundRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t radius, uint16_t color) {
	int16_t tSwitch = 3 - 2 * radius;
	uint8_t x1 = 0, y1 = radius;
	uint16_t delta = height - 2*radius-1;
	// center block
	ili9341_fillrect(x+radius, y, width-2*radius, height, color);
	while (x1 <= y1) {
		// left side
		ili9341_drawvline(x+radius - x1, y+radius - y1, delta + 2*y1+1, color);
		ili9341_drawvline(x+radius - y1, y+radius - x1, delta + 2*x1+1, color);
		// right side
		ili9341_drawvline(x+width-radius-1 + x1, y+radius - y1, delta + 2*y1+1, color);
		ili9341_drawvline(x+width-radius-1 + y1, y+radius - x1, delta + 2*x1+1, color);
		if (tSwitch < 0) {
			tSwitch += (4 * x1 + 6);
		} else {
			tSwitch += (4 * (x1 - y1) + 10);
			y1--;
		}
		x1++;
	}
}

void ili9341_setFont(const uint8_t *f) {
	font = f;
	font_size = (readFontByte(&font[FONT_LENGTH]) << 8) | readFontByte(&font[FONT_LENGTH+1]);
}

// Draw openGLCD font character
void ili9341_drawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t fgcolor, uint16_t bgcolor, uint8_t size) {
	uint8_t font_width = readFontByte(&font[FONT_WIDTH]);
	uint8_t font_height = readFontByte(&font[FONT_HEIGHT]);
	uint8_t font_first = readFontByte(&font[FONT_FIRST_CHAR]);
	uint8_t font_count = readFontByte(&font[FONT_CHAR_COUNT]);
	const uint8_t *base = &font[FONT_WIDTH_TABLE];
	uint8_t padding = (font_size == 1) ? 0 : 1;
	uint8_t bytes = (font_height + 7) / 8; // calculates height in rounded up bytes
	uint8_t shift = 0;
	c -= font_first;
	if (font_size < 2) {
		// Fixed width font
		base += c * bytes * font_width;
	} else {
		// Proportional font, read width data, to get the index
		font_width = readFontByte(base + c);
		if (font_height & 7)
			shift = 8 - (font_height & 7); // FontCreator shifts residual font bits the wrong direction
		uint16_t index = 0;
		for (uint8_t i = 0; i < c; i++)
			index += readFontByte(base + i);
		base += bytes*index + font_count;
	}
	if ((x+font_width*size > _width) || (y+font_height*size > _height)) return;
	if (fgcolor == bgcolor) {
		uint16_t xoff, yoff, count;
		uint8_t mask;
		for (uint8_t byte = bytes; byte > 0; byte--, base += font_width) {
			if (shift && byte == 1)
				mask = 0x01 << shift;
			else
				mask = 0x01;
			for (yoff=0; yoff < 8; yoff++) {
				xoff = 0;
				while (xoff < font_width) {
					// skip transparent bg
					while (xoff < font_width && !(readFontByte(&base[xoff]) & mask))
						xoff++;
					// foreground
					count = 0;
					while (xoff < font_width && readFontByte(&base[xoff]) & mask) {
						count++;
						xoff++;
					}
					if (count) {
						if (size == 1)
							ili9341_drawhline(x+xoff-count, y + yoff, count, fgcolor);
						else
							ili9341_fillrect(x + (xoff-count) * size, y + yoff * size, count * size, size, fgcolor);
					}
				}
				mask <<= 1;
			}
			y += 8 * size;
		}
	} else {
		ili9341_setaddress(x, y, x + (font_width + padding) * size - 1, y + (font_height+padding) * size - 1);
		spi_begin();
		uint8_t mask = 0x01;
		for (uint8_t h=font_height; h > 0; h--) {
			for (uint8_t yr=0; yr < size; yr++) {
				for (uint8_t w=0; w < font_width; w++)
					spiwrite16((readFontByte(&base[w]) & mask) ? fgcolor : bgcolor, size);
				// extra pixels on right for spacing
				spiwrite16(bgcolor, padding * size);
			}
			mask <<= 1;
			if (mask == 0) {
				base += font_width;
				if (shift && --bytes == 1)
					mask = 0x01 << shift;
				else
					mask = 0x01;
			}
		}
		if (padding)
			spiwrite16(bgcolor, (font_width+padding) * size); // extra pixels below for spacing
		spi_end();
	}
}

void ili9341_drawXBitmapTrans(uint16_t x, uint16_t y, const char bitmap[], uint16_t w, uint16_t h, uint16_t color) {
	uint8_t byteWidth = (w + 7) / 8; // Bitmap scan line pad = whole byte
	uint8_t byte = 0;

	for (uint16_t j = 0; j < h; j++, y++) {
		for (uint16_t i = 0; i < w; i++) {
			if (i & 7)
				byte >>= 1;
			else
				byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
			// Bit order left-to-right = LSB to MSB:
			if (byte & 0x01)
				ili9341_drawpixel(x + i, y, color);
		}
	}
}

void ili9341_drawXBitmap(uint16_t x, uint16_t y, const char bitmap[], uint16_t w, uint16_t h, uint16_t color, uint16_t bg) {
	uint8_t byteWidth = (w + 7) / 8; // Bitmap scan line pad = whole byte
	uint8_t byte = 0;

	for (uint16_t j = 0; j < h; j++, y++) {
		ili9341_setaddress(x,y,x+w,y);
		spi_begin();
		for (uint16_t i = 0; i < w; i++) {
			if (i & 7)
				byte >>= 1;
			else
				byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
			// Bit order left-to-right = LSB to MSB:
			writedata16_cont((byte & 0x1) ? color : bg);
		}
		spi_end();
	}
}

void ili9341_setCursor(uint16_t x, uint16_t y) {
	if (x >= _width) x = _width - 1;
	cursor_x = x;
	if (y >= _height) y = _height - 1;
	cursor_y = y;
}

void ili9341_setTextColor(uint16_t color, uint16_t bg) {
	textcolor = color;
	textbgcolor = bg;
}

void ili9341_setTextSize(uint8_t size) {
	textsize = size;
}

void ili9341_write(char c) {
	uint8_t font_height = readFontByte(&font[FONT_HEIGHT]);
	uint8_t font_width, font_first = readFontByte(&font[FONT_FIRST_CHAR]);
	if (font_size > 1)
		font_width = readFontByte(&font[FONT_WIDTH_TABLE + c - font_first]);
	else
		font_width = readFontByte(&font[FONT_WIDTH]);
	if (font_size != 1) {
		font_height++;
		font_width++;
	}
	if (c == '\n') {
		cursor_x = 0;
		cursor_y += font_height * textsize;
	} else {
		uint8_t font_count = readFontByte(&font[FONT_CHAR_COUNT]);
		if (c < font_first || c >= (font_first + font_count)) return;
		ili9341_drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
		cursor_x += font_width * textsize;
	}
}

uint8_t ili9341_charWidth(char c) {
	uint8_t font_width, font_first = readFontByte(&font[FONT_FIRST_CHAR]);
	uint8_t font_count = readFontByte(&font[FONT_CHAR_COUNT]);
	if (c < font_first || c >= (font_first + font_count)) return 0;
	if (font_size > 1)
		font_width = readFontByte(&font[FONT_WIDTH_TABLE + c - font_first]);
	else
		font_width = readFontByte(&font[FONT_WIDTH]);
	return font_width * textsize;
}

size_t ili9341_strWidth(const char *string) {
	size_t sw = 0;
	while (*string) {
		uint8_t cw = ili9341_charWidth(*string++);
		if (cw == 0) continue;
		sw += cw;
		if (font_size != 1) sw++;
	}
	return sw;
}

void ili9341_puts(const char *string) {
	while(*string)
		ili9341_write(*string++);
}

void ili9341_puts_p(const char *string) {
	register char c;
	while ((c = pgm_read_byte(string++)))
		ili9341_write(c);
}

#define min(a,b) ((a)<(b)?(a):(b))

// Clears area from x to cursor
void ili9341_clearTextArea(uint16_t x) {
	uint8_t font_height = readFontByte(&font[FONT_HEIGHT]);
	if (font_size != 1) font_height++;
	ili9341_fillrect(min(x, cursor_x), cursor_y, abs((int16_t)x - cursor_x), font_height, textbgcolor);
}
