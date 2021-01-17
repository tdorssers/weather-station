/*
 * allFonts.h font header for GLCD library
 * The fonts listed below will be available in a sketch if this file is included
 *
 * If you create your own fonts you can add the header to this file
 *
 * Note that the build environment only holds a font in Flash if its selected
 * so there is no penalty to including a font file here if its not used
 */

#ifndef _allFonts_h_
#define _allFonts_h_

#ifdef __AVR__
#include <avr/pgmspace.h>
/** declare a font for AVR. */
#define GLCDFONTDECL(_n) static const uint8_t __attribute__ ((progmem))_n[]
#define readFontByte(addr) pgm_read_byte(addr)
#else  // __AVR__
/** declare a font. */
#define GLCDFONTDECL(_n) static const uint8_t _n[]
/** Fake read from flash. */
#define readFontByte(addr) (*(const unsigned char *)(addr))
#endif  // __AVR__
//------------------------------------------------------------------------------
// Font Indices
/** No longer used Big Endian length field. Now indicates font type.
 *
 * 00 00 (fixed width font with 1 padding pixel on right and below)
 * 
 * 00 01 (fixed width font with no padding pixels)
 */
#define FONT_LENGTH      0
/** Maximum character width. */
#define FONT_WIDTH       2
/** Font hight in pixels */ 
#define FONT_HEIGHT      3
/** Ascii value of first character */
#define FONT_FIRST_CHAR  4
/** count of characters in font. */
#define FONT_CHAR_COUNT  5
/** Offset to width table. */
#define FONT_WIDTH_TABLE 6
//
// FONT_LENGTH is a 16 bit Big Endian length field.
// Unfortunately, FontCreator2 screwed up the value it put in the field
// so it is pretty much meaningless. However it still is used to indicate
// some special things.
// 00 00 (fixed width font with 1 padding pixel on right and below)
// 00 01 (fixed width font with no padding pixels)
// any other value means variable width font in FontCreator2 (Thiele)
// format with pixel padding

#include "System5x7.h" 	        // system font (fixed width)
#include "SystemFont5x7.h"      // backward compatibility System5x7 header
#include "Iain5x7.h"		// similar to system5x7 but proportional
#include "Arial14.h"            // proportional font
#include "Arial_bold_14.h"      // Bold proportional font
#include "Corsiva_12.h"
#include "Verdana_digits_24.h"  // large proportional font contains [0-9] and :

#include "fixednums7x15.h"       // fixed width font - + , - . / [0-9] and :
#include "fixednums8x16.h"       // fixed width font - + , - . / [0-9] and :
#include "fixednums15x31.h"      // fixed width font - + , - . / [0-9] and :

#include "CalBlk36.h"
#include "CalLite24.h"
#include "lcdnums12x16.h"	// font that looks like LCD digits
#include "lcdnums14x24.h"	// font that looks like LCD digits
#include "fixed_bold10x15.h"
#include "Wendy3x5.h"
#include "newbasic3x5.h"

/*
 * These fonts require no-pad rendering code
 */
#include "font8x8.h"		// fixed wider font but similar to system5x7 font
#include "cp437font8x8.h"       // fixed Font from 80's IBM PC

#endif
