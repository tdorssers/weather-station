/*
 * ILI9341 Driver
 *
 * Created: 14-12-2019 20:04:13
 *  Author: Tim Dorssers
 */ 


#ifndef ILI9341_H_
#define ILI9341_H_

#include <stdint.h>
#include <stdbool.h>
#include "fonts/allFonts.h"

#define ILI9341_TFTWIDTH   240      // ILI9341 max TFT width
#define ILI9341_TFTHEIGHT  320      // ILI9341 max TFT height

#define ILI9341_NOP        0x00     // No-op register
#define ILI9341_SWRESET    0x01     // Software reset register
#define ILI9341_RDDID      0x04     // Read display identification information
#define ILI9341_RDDST      0x09     // Read Display Status

#define ILI9341_SLPIN      0x10     // Enter Sleep Mode
#define ILI9341_SLPOUT     0x11     // Sleep Out
#define ILI9341_PTLON      0x12     // Partial Mode ON
#define ILI9341_NORON      0x13     // Normal Display Mode ON

#define ILI9341_RDMODE     0x0A     // Read Display Power Mode
#define ILI9341_RDMADCTL   0x0B     // Read Display MADCTL
#define ILI9341_RDPIXFMT   0x0C     // Read Display Pixel Format
#define ILI9341_RDIMGFMT   0x0D     // Read Display Image Format
#define ILI9341_RDDSPSGNMODE 0x0E   // Read Display Signal Mode
#define ILI9341_RDSELFDIAG 0x0F     // Read Display Self-Diagnostic Result

#define ILI9341_INVOFF     0x20     // Display Inversion OFF
#define ILI9341_INVON      0x21     // Display Inversion ON
#define ILI9341_GAMMASET   0x26     // Gamma Set
#define ILI9341_DISPOFF    0x28     // Display OFF
#define ILI9341_DISPON     0x29     // Display ON

#define ILI9341_CASET      0x2A     // Column Address Set
#define ILI9341_PASET      0x2B     // Page Address Set
#define ILI9341_RAMWR      0x2C     // Memory Write
#define ILI9341_RAMRD      0x2E     // Memory Read

#define ILI9341_PTLAR      0x30     // Partial Area
#define ILI9341_VSCRDEF    0x33     // Vertical Scrolling Definition
#define ILI9341_MADCTL     0x36     // Memory Access Control
#define ILI9341_VSCRSADD   0x37     // Vertical Scrolling Start Address
#define ILI9341_IDMOFF     0x38     // Idle Mode OFF
#define ILI9341_IDMON      0x39     // Idle Mode ON
#define ILI9341_PIXFMT     0x3A     // COLMOD: Pixel Format Set

#define ILI9341_FRMCTR1    0xB1     // Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2    0xB2     // Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3    0xB3     // Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR     0xB4     // Display Inversion Control
#define ILI9341_DFUNCTR    0xB6     // Display Function Control

#define ILI9341_PWCTR1     0xC0     // Power Control 1
#define ILI9341_PWCTR2     0xC1     // Power Control 2
#define ILI9341_PWCTR3     0xC2     // Power Control 3
#define ILI9341_PWCTR4     0xC3     // Power Control 4
#define ILI9341_PWCTR5     0xC4     // Power Control 5
#define ILI9341_VMCTR1     0xC5     // VCOM Control 1
#define ILI9341_VMCTR2     0xC7     // VCOM Control 2

#define ILI9341_RDID1      0xDA     // Read ID 1
#define ILI9341_RDID2      0xDB     // Read ID 2
#define ILI9341_RDID3      0xDC     // Read ID 3
#define ILI9341_RDID4      0xDD     // Read ID 4

#define ILI9341_GMCTRP1    0xE0     // Positive Gamma Correction
#define ILI9341_GMCTRN1    0xE1     // Negative Gamma Correction

// Color definitions
#define ILI9341_ALICEBLUE 0xF7DF
#define ILI9341_ANTIQUEWHITE 0xFF5A
#define ILI9341_AQUA 0x07FF
#define ILI9341_AQUAMARINE 0x7FFA
#define ILI9341_AZURE 0xF7FF
#define ILI9341_BEIGE 0xF7BB
#define ILI9341_BISQUE 0xFF38
#define ILI9341_BLACK 0x0000
#define ILI9341_BLANCHEDALMOND 0xFF59
#define ILI9341_BLUE 0x001F
#define ILI9341_BLUEVIOLET 0x895C
#define ILI9341_BROWN 0xA145
#define ILI9341_BURLYWOOD 0xDDD0
#define ILI9341_CADETBLUE 0x5CF4
#define ILI9341_CHARTREUSE 0x7FE0
#define ILI9341_CHOCOLATE 0xD343
#define ILI9341_CORAL 0xFBEA
#define ILI9341_CORNFLOWERBLUE 0x64BD
#define ILI9341_CORNSILK 0xFFDB
#define ILI9341_CRIMSON 0xD8A7
#define ILI9341_CYAN 0x07FF
#define ILI9341_DARKBLUE 0x0011
#define ILI9341_DARKCYAN 0x0451
#define ILI9341_DARKGOLDENROD 0xBC21
#define ILI9341_DARKGREY 0xAD55
#define ILI9341_DARKGREEN 0x0320
#define ILI9341_DARKKHAKI 0xBDAD
#define ILI9341_DARKMAGENTA 0x8811
#define ILI9341_DARKOLIVEGREEN 0x5345
#define ILI9341_DARKORANGE 0xFC60
#define ILI9341_DARKORCHID 0x9999
#define ILI9341_DARKRED 0x8800
#define ILI9341_DARKSALMON 0xECAF
#define ILI9341_DARKSEAGREEN 0x8DF1
#define ILI9341_DARKSLATEBLUE 0x49F1
#define ILI9341_DARKSLATEGRAY 0x2A69
#define ILI9341_DARKTURQUOISE 0x067A
#define ILI9341_DARKVIOLET 0x901A
#define ILI9341_DEEPPINK 0xF8B2
#define ILI9341_DEEPSKYBLUE 0x05FF
#define ILI9341_DIMGRAY 0x6B4D
#define ILI9341_DODGERBLUE 0x1C9F
#define ILI9341_FIREBRICK 0xB104
#define ILI9341_FLORALWHITE 0xFFDE
#define ILI9341_FORESTGREEN 0x2444
#define ILI9341_FUCHSIA 0xF81F
#define ILI9341_GAINSBORO 0xDEFB
#define ILI9341_GHOSTWHITE 0xFFDF
#define ILI9341_GOLD 0xFEA0
#define ILI9341_GOLDENROD 0xDD24
#define ILI9341_GRAY 0x8410
#define ILI9341_GREEN 0x0400
#define ILI9341_GREENYELLOW 0xAFE5
#define ILI9341_HONEYDEW 0xF7FE
#define ILI9341_HOTPINK 0xFB56
#define ILI9341_INDIANRED 0xCAEB
#define ILI9341_INDIGO 0x4810
#define ILI9341_IVORY 0xFFFE
#define ILI9341_KHAKI 0xF731
#define ILI9341_LAVENDER 0xE73F
#define ILI9341_LAVENDERBLUSH 0xFF9E
#define ILI9341_LAWNGREEN 0x7FE0
#define ILI9341_LEMONCHIFFON 0xFFD9
#define ILI9341_LIGHTBLUE 0xAEDC
#define ILI9341_LIGHTCORAL 0xF410
#define ILI9341_LIGHTCYAN 0xE7FF
#define ILI9341_LIGHTGOLDENRODYELLOW 0xFFDA
#define ILI9341_LIGHTGREEN 0x9772
#define ILI9341_LIGHTGREY 0xD69A
#define ILI9341_LIGHTPINK 0xFDB8
#define ILI9341_LIGHTSALMON 0xFD0F
#define ILI9341_LIGHTSEAGREEN 0x2595
#define ILI9341_LIGHTSKYBLUE 0x867F
#define ILI9341_LIGHTSLATEGRAY 0x7453
#define ILI9341_LIGHTSTEELBLUE 0xB63B
#define ILI9341_LIGHTYELLOW 0xFFFC
#define ILI9341_LIME 0x07E0
#define ILI9341_LIMEGREEN 0x3666
#define ILI9341_LINEN 0xFF9C
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_MAROON 0x8000
#define ILI9341_MEDIUMAQUAMARINE 0x6675
#define ILI9341_MEDIUMBLUE 0x0019
#define ILI9341_MEDIUMORCHID 0xBABA
#define ILI9341_MEDIUMPURPLE 0x939B
#define ILI9341_MEDIUMSEAGREEN 0x3D8E
#define ILI9341_MEDIUMSLATEBLUE 0x7B5D
#define ILI9341_MEDIUMSPRINGGREEN 0x07D3
#define ILI9341_MEDIUMTURQUOISE 0x4E99
#define ILI9341_MEDIUMVIOLETRED 0xC0B0
#define ILI9341_MIDNIGHTBLUE 0x18CE
#define ILI9341_MINTCREAM 0xF7FF
#define ILI9341_MISTYROSE 0xFF3C
#define ILI9341_MOCCASIN 0xFF36
#define ILI9341_NAVAJOWHITE 0xFEF5
#define ILI9341_NAVY 0x0010
#define ILI9341_OLDLACE 0xFFBC
#define ILI9341_OLIVE 0x8400
#define ILI9341_OLIVEDRAB 0x6C64
#define ILI9341_ORANGE 0xFD20
#define ILI9341_ORANGERED 0xFA20
#define ILI9341_ORCHID 0xDB9A
#define ILI9341_PALEGOLDENROD 0xEF55
#define ILI9341_PALEGREEN 0x9FD3
#define ILI9341_PALETURQUOISE 0xAF7D
#define ILI9341_PALEVIOLETRED 0xDB92
#define ILI9341_PAPAYAWHIP 0xFF7A
#define ILI9341_PEACHPUFF 0xFED7
#define ILI9341_PERU 0xCC27
#define ILI9341_PINK 0xFE19
#define ILI9341_PLUM 0xDD1B
#define ILI9341_POWDERBLUE 0xB71C
#define ILI9341_PURPLE 0x8010
#define ILI9341_RED 0xF800
#define ILI9341_ROSYBROWN 0xBC71
#define ILI9341_ROYALBLUE 0x435C
#define ILI9341_SADDLEBROWN 0x8A22
#define ILI9341_SALMON 0xFC0E
#define ILI9341_SANDYBROWN 0xF52C
#define ILI9341_SEAGREEN 0x2C4A
#define ILI9341_SEASHELL 0xFFBD
#define ILI9341_SIENNA 0xA285
#define ILI9341_SILVER 0xC618
#define ILI9341_SKYBLUE 0x867D
#define ILI9341_SLATEBLUE 0x6AD9
#define ILI9341_SLATEGRAY 0x7412
#define ILI9341_SNOW 0xFFDF
#define ILI9341_SPRINGGREEN 0x07EF
#define ILI9341_STEELBLUE 0x4416
#define ILI9341_TAN 0xD5B1
#define ILI9341_TEAL 0x0410
#define ILI9341_THISTLE 0xDDFB
#define ILI9341_TOMATO 0xFB08
#define ILI9341_TURQUOISE 0x471A
#define ILI9341_VIOLET 0xEC1D
#define ILI9341_WHEAT 0xF6F6
#define ILI9341_WHITE 0xFFFF
#define ILI9341_WHITESMOKE 0xF7BE
#define ILI9341_YELLOW 0xFFE0
#define ILI9341_YELLOWGREEN 0x9E66

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
#define color565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

void ili9341_init();
uint8_t ili9341_readcommand8(uint8_t com);
uint16_t ili9341_readPixel(uint16_t x, uint16_t y);
void ili9341_readRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *pcolors);
void ili9341_writeRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pcolors);
void ili9341_fillScreen(uint16_t color);
void ili9341_drawpixel(uint16_t x,uint16_t y,uint16_t color);
void ili9341_drawvline(uint16_t x,uint16_t y,uint16_t h,uint16_t color);
void ili9341_drawhline(uint16_t x,uint16_t y,uint16_t w,uint16_t color);
void ili9341_fillrect(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t color);
void ili9341_setRotation(uint8_t m);
void ili9341_invertDisplay(bool i);
void ili9341_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ili9341_drawLineByAngle(uint16_t x, uint16_t y, int16_t angle, int16_t length, uint16_t color);
void ili9341_drawRect(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t color);
void ili9341_drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void ili9341_fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void ili9341_display(bool d);
void ili9341_sleep(bool s);
void ili9341_idle(bool i);
void ili9341_setupScrollArea(uint16_t tfa, uint16_t bfa);
void ili9341_scrollAddress(uint16_t vsp);
void ili9341_drawEllipse(uint16_t x0, uint16_t y0, int16_t rx, int16_t ry, uint16_t color);
void ili9341_fillEllipse(uint16_t x0, uint16_t y0, int16_t rx, int16_t ry, uint16_t color);
void ili9341_drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ili9341_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ili9341_fillArc(uint16_t x, uint16_t y, int16_t start, int16_t end, int16_t rx, int16_t ry, int16_t w, uint16_t color);
void ili9341_fillArcDashed(uint16_t x, uint16_t y, int16_t start, int16_t end, int16_t r, int16_t w, uint16_t color);
void ili9341_drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
void ili9341_fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
void ili9341_drawXBitmapTrans(uint16_t x, uint16_t y, const char bitmap[], uint16_t w, uint16_t h, uint16_t color);
void ili9341_drawXBitmap(uint16_t x, uint16_t y, const char bitmap[], uint16_t w, uint16_t h, uint16_t color, uint16_t bg);
void ili9341_drawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t fgcolor, uint16_t bgcolor, uint8_t size);
void ili9341_setFont(const uint8_t *f);
void ili9341_setCursor(uint16_t x, uint16_t y);
void ili9341_getCursor(uint16_t *x, uint16_t *y);
void ili9341_setTextColor(uint16_t color, uint16_t bg);
void ili9341_setTextSize(uint8_t size);
void ili9341_write(char c);
uint8_t ili9341_fontHeight(void);
uint8_t ili9341_charWidth(char c);
size_t ili9341_strWidth(const char *string);
size_t ili9341_strWidth_p(const char *string);
void ili9341_puts(const char *string);
void ili9341_puts_p(const char *string);
void ili9341_clearTextArea(uint16_t x);

#endif /* ILI9341_H_ */