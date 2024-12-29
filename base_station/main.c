/*
 * Title   : Weather station
 * Hardware: ATmega328P @ 12 MHz, RFM210LCF 433MHz ASK/OOK receiver module,
 *           BME280 digital humidity, pressure and temperature sensor,
 *           ILI9341 TFT LCD driver, XPT2046 touch screen controller, 
 *           GL5528 LDR, NEO-6M GPS module
 *
 * Created: 14-12-2019 19:43:50
 * Author : Tim Dorssers
 *
 * A BME280 barometric sensor is used to implement a simple Zambretti weather
 * forecasting algorithm by calculating the Pa/h trend. Up to 16 wireless
 * DS18B20, AM2320, AHT20 or SHT30 sensors transmit NRZ encoded packets with
 * CRC data using a 433MHz ASK/OOK module connected to the hardware UART at
 * 1200 baud. Up to 6 remote sensors are shown on the LCD. Minimum and maximum
 * temperatures and humidity values are stored in 4 six hour periods. A
 * software UART implementation using Timer1 is interfacing with the GPS module
 * at 9600 baud. Libc timekeeping uses the GPS position to determine day and
 * night display mode, which can be overridden from the menu. Time zone and
 * daylight saving time can be set from the menu which is shown by touching the
 * main screen. Different temperature and pressure units can be selected. The
 * LCD back light level is auto adjusted using a 5528 LDR and can be set
 * manually. Screen orientation can be adjusted and the touch screen requires
 * calibration which is done by touching the screen edges. The remote sensors
 * can be named with 3 characters from the menu. Calibration and names are
 * stored in EEPROM.
 *
 * PB0/ICP1=GPS RX	PC0/ADC0=NC		PD0/RXD=DATA
 * PB1/OC1A=GPS TX	PC1/ADC1=NC		PD1/TXD=RS232
 * PB2/SS=CSB		PC2/ADC2=LDR	PD2/INT0=T_IRQ
 * PB3/MOSI=SPI		PC3/ADC3=NC		PD3/INT1=T_CS
 * PB4/MISO=SPI		PC4/SDA=NC		PD4/T0=RESET
 * PB5/SCK=SPI		PC5/SCL=NC		PD5/OC0B=LED
 * PB6/XTAL1=NC		PC6/RESET=ISP	PD6/OC0A=DC
 * PB7/XTAL2=NC						PD7/AIN1=CS
 */

#define F_CPU 12000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "ili9341.h"
#include "bme280.h"
#include "uart.h"
#include "weatherIcons.h"
#include "suart.h"
#include "gps.h"
#include "xpt2046.h"

char buffer[26];

typedef enum {OK, NO_RESPONSE, CRC_ERROR} result_t;
typedef enum {DS18B20, AM2320, AHT20, SHT30} type_t;

typedef union {
	struct {
		uint8_t id:4;
		result_t result:2;
		type_t type:2;
	};
	uint8_t raw;
} unit_t;

typedef struct {
	unit_t unit;
	uint16_t humid;
	int16_t temp;
	uint16_t crc;
} packet_t;

typedef struct {
	uint16_t min_humid, max_humid;
	int16_t min_temp, max_temp;
} history_t;

typedef struct {
	bool enabled;
	uint16_t age;
	unit_t unit;
	uint16_t humid;
	int16_t temp;
	history_t hist[4];
	char name[4];
} sensor_t;

#define SENSOR_COUNT 16 // times 44 bytes = 704 bytes
packet_t rxData;
sensor_t local, remote[SENSOR_COUNT];
uint8_t period = 0, max_period = 1;
char EEMEM nv_names[SENSOR_COUNT][4];

typedef struct {
	bool update_screen;
	bool take_sample;
	bool advance_period;
	bool blink;
} action_t;

volatile action_t action;
volatile uint16_t millis=0;

typedef struct {
	uint8_t value;
	uint8_t touch;
} button_t;

typedef enum {SCREEN, MENU, CALIBRATE} view_t;

view_t view = SCREEN;
enum {TAB_CONFIG = 1, TAB_LCD, TAB_ETC, TAB_NAMES};
enum {THEME_LIGHT = 1, THEME_DARK, THEME_AUTO};
enum {DEG_CELSIUS = 1, DEG_FAHRENHEIT};
enum {PRESSURE_HPA = 1, PRESSURE_MMHG, PRESSURE_INHG, PRESSURE_PSI};
bool dst = true, old_auto_led, is_day = false, refresh, north, old_rainbow;
button_t b_dst = {.value = true}, b_auto_led = {.value = true};
button_t b_theme = {.value = THEME_AUTO}, b_pressure = {.value = PRESSURE_HPA};
button_t b_degrees = {.value = DEG_CELSIUS}, b_rainbow = {.value = true};
int8_t tz = 1, new_tz = 1;
uint8_t old_ocr0b, rotation = 3, old_theme, old_pressure, old_degrees;
int16_t altitude;
uint16_t fgcolor = ILI9341_WHITE, bgcolor = ILI9341_BLACK;

const char keys1[] PROGMEM = "1234567890qwertyuiopasdfghjkl\x1Ezxcvbnm\x19\x1B";
const char keys2[] PROGMEM = "!@#$%^&*()QWERTYUIOPASDFGHJKL\x1EZXCVBNM\x18\x1B";

const char no_gps_icon[] PROGMEM = {
	0x90, 0x63, 0x90, 0x77, 0x91, 0x1f, 0x91, 0x07, 0x8f, 0x1f, 0x7c, 0x88,
	0x77, 0x7c, 0x07, 0x18, 0x76, 0x7d, 0x8d, 0x07, 0x22, 0x89, 0x07, 0x78,
	0x88, 0x1f, 0x7c, 0x03, 0x78, 0x79, 0x0f, 0x40, 0x1f, 0x70, 0x88, 0x3f,
	0x60, 0x04, 0x70, 0x07, 0x72, 0x00, 0x7f, 0x10, 0x06, 0x78, 0x0f, 0x71,
	0x00, 0xc7, 0x40, 0x03, 0x78, 0x1f, 0x18, 0x40, 0xc9, 0x18, 0x00, 0xce,
	0x8a, 0xcb, 0x8c, 0xc9, 0x90, 0x0f, 0x00
};

const char gps_icon[] PROGMEM = {
	0x8c, 0x03, 0x90, 0x1f, 0x90, 0x7f, 0x94, 0x0f, 0x8c, 0x67, 0x03, 0x1f,
	0x78, 0x1d, 0x7c, 0x07, 0x5e, 0x73, 0x7d, 0x40, 0x19, 0x07, 0x22, 0x5c,
	0x39, 0x40, 0x47, 0x7d, 0x07, 0x7f, 0x6c, 0x3f, 0x7e, 0x03, 0x70, 0x07,
	0x1c, 0x00, 0x3f, 0x60, 0x04, 0x70, 0x07, 0x72, 0x00, 0x7f, 0x10, 0x06,
	0x78, 0x0f, 0x71, 0x00, 0xc7, 0x40, 0x03, 0x78, 0x1f, 0x18, 0x40, 0xc9,
	0x18, 0x00, 0xce, 0x8a, 0xcb, 0x8c, 0xc9, 0x90, 0x0f, 0x00
};

#define LAST_SAMPLES_COUNT  5
int32_t dP_dt;

// Null character as string separator
const char str_A[] PROGMEM = "Settled fine\0";
const char str_B[] PROGMEM = "Fine weather\0";
const char str_C[] PROGMEM = "Becoming fine\0";
const char str_D[] PROGMEM = "Fine becoming\0less settled";
const char str_E[] PROGMEM = "Fine possible\0showers";
const char str_F[] PROGMEM = "Fairly fine\0improving";
const char str_G[] PROGMEM = "Fairly fine\0maybe showers";
const char str_H[] PROGMEM = "Fairly fine\0showery later";
const char str_I[] PROGMEM = "Showery early\0improving";
const char str_J[] PROGMEM = "Changeable\0mending";
const char str_K[] PROGMEM = "Fairly fine\0showers likely";
const char str_L[] PROGMEM = "Quite unsettled\0clearing";
const char str_M[] PROGMEM = "Unsettled\0likely improving";
const char str_N[] PROGMEM = "Showery bright\0intervals";
const char str_O[] PROGMEM = "Showery less\0settled";
const char str_P[] PROGMEM = "Changeable\0some rain";
const char str_Q[] PROGMEM = "Unsettled\0fine intervals";
const char str_R[] PROGMEM = "Unsettled\0rain later";
const char str_S[] PROGMEM = "Unsettled\0some rain";
const char str_T[] PROGMEM = "Mostly very\0unsettled";
const char str_U[] PROGMEM = "Occasional rain\0worsening";
const char str_V[] PROGMEM = "Some rain\0very unsettled";
const char str_W[] PROGMEM = "Rain at\0regular interval";
const char str_X[] PROGMEM = "Rain\0very unsettled";
const char str_Y[] PROGMEM = "Stormy\0may improve";
const char str_Z[] PROGMEM = "Stormy\0much rain";
PGM_P const forecast[] PROGMEM = {
	str_A, str_B, str_C, str_D, str_E, str_F, str_G, str_H, str_I, str_J, str_K, str_L, str_M, 
	str_N, str_O, str_P, str_Q, str_R, str_S, str_T, str_U, str_V, str_W, str_X, str_Y, str_Z
};
const char rising[] PROGMEM = {
	'A', 'B', 'B', 'C', 'F', 'G', 'I', 'J', 'L', 'M', 'M', 'Q', 'T', 'Y'
};
const char falling[] PROGMEM = {
	'B', 'D', 'H', 'O', 'R', 'U', 'V', 'X', 'X', 'Z'
};
const char steady[] PROGMEM = {
	'A', 'B', 'B', 'B', 'E', 'K', 'N', 'N', 'P', 'P', 'S', 'W', 'W', 'X', 'X', 'X', 'Z'
};

#define map(x,in_min,in_max,out_min,out_max) (((x)-(in_min))*((out_max)-(out_min))/((in_max)-(in_min))+(out_min))
#define min(a,b) ((a)<(b)?(a):(b))

// Time based event triggers
ISR(TIMER2_COMPA_vect) {
	static uint8_t sec=0;
	static uint16_t msec=0, mins=0;

	millis++;
	if (++msec == 1000) {
		msec = 0;
		system_tick();
		for (uint8_t i = 0; i < SENSOR_COUNT; i++)
			remote[i].age++;
		action.blink = !action.blink;
		action.update_screen = true;
		if (++sec == 60) {
			sec = 0;
			mins++;
			action.take_sample = true;
			if (mins == 360) {
				mins = 0;
				action.advance_period = true;
			}
		}
	}
}

// Initialize 1 millisecond timer
static void timer2_init() {
	TCCR2A = _BV(WGM21);
	TCCR2B = _BV(CS22);
	OCR2A = ((F_CPU / 1000) / 64) - 1;
	TIMSK2 = _BV(OCIE2A);
}

// Initialize LED PWM
static void timer0_init(void) {
	TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); /* Enable non inverting 8-Bit PWM */
	TCCR0B = _BV(CS01); /* Timer clock = I/O clock/8 */
	OCR0B = 0x7F; /* 50% duty cycle */
	DDRD |= _BV(PD5); /* Set OC0B pin as output */
}

// Initialize ADC
static void init_adc(void) {
	// AVCC with external capacitor at AREF pin and ADC Left Adjust Result
	ADMUX = _BV(ADLAR) | _BV(REFS0);
	// ADC prescaler of 128 and enable ADC
	ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN);
}

// Read single conversion from given ADC channel
static uint8_t read_adc(uint8_t ch) {
	ADMUX = (ADMUX & 0xf0) | (ch & 0x0f);	// select channel
	ADCSRA |= _BV(ADSC);					// start conversion
	while (ADCSRA & _BV(ADSC));				// wait until conversion complete
	return ADCH;
}

// Daylight Saving function for the European Union
// From http://savannah.nongnu.org/bugs/?44327
static int eu_dst(const time_t *timer, int32_t *z)
{
	uint32_t t = *timer;
	if ((uint8_t)(t >> 24) >= 194) t -= 3029443200U;
	t = (t + 655513200) / 604800 * 28;
	if ((uint16_t)(t % 1461) < 856) return 3600;
	else return 0;
}

// Convert (scaled) integer to (zero filled) string
static char *itostr(int16_t num, char *str, uint8_t decimal, uint8_t padding) {
	uint8_t i = 0;
	uint16_t sum = abs(num);
	if (decimal)
		padding++;
	do {
		str[i++] = '0' + sum % 10;
		if (i == decimal)
			str[i++] = '.';
	} while ((sum /= 10) || i < decimal);
	while (i < padding)
		str[i++] = '0';
	if (num < 0)
		str[i++] = '-';
	str[i] = '\0';
	return strrev(str);
}

// Based on https://www.nxp.com/docs/en/application-note/AN3914.pdf
// Pressure in Pa -->  forecast done by calculating Pa/h
static void sample(uint32_t pressure) {
	static uint32_t lastPressureSamples[LAST_SAMPLES_COUNT];
	static uint8_t minuteCount = 0;
	static bool firstPass = true;
	static uint32_t pressureAvg;
	static uint32_t pressureAvg2;

	// Calculate the average of the last n minutes.
	uint8_t index = minuteCount % LAST_SAMPLES_COUNT;
	lastPressureSamples[index] = pressure;
	uint32_t lastPressureAvg = 0;
	for (uint8_t i = 0; i < LAST_SAMPLES_COUNT; i++) {
		lastPressureAvg += lastPressureSamples[i];
	}
	lastPressureAvg /= LAST_SAMPLES_COUNT;
	
	minuteCount++;
	if (minuteCount > 180) {
		minuteCount = 6;
	}
	if (minuteCount == 5) {
		pressureAvg = lastPressureAvg;
	} else if (minuteCount == 35) {
		int32_t change = (lastPressureAvg - pressureAvg);
		if (firstPass) { // first time initial 3 hour
			dP_dt = change * 2; // note this is for t = 0.5hour
		} else {
			dP_dt = (change * 2) / 3; // divide by 1.5 as this is the difference in time from 0 value.
		}
	} else if (minuteCount == 60) {
		int32_t change = (lastPressureAvg - pressureAvg);
		if (firstPass) { //first time initial 3 hour
			dP_dt = change; //note this is for t = 1 hour
		} else {
			dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
		}
	} else if (minuteCount == 95) {
		int32_t change = (lastPressureAvg - pressureAvg);
		if (firstPass) { // first time initial 3 hour
			dP_dt = (change * 2) / 3; // note this is for t = 1.5 hour
		} else {
			dP_dt = (change * 2) / 5; // divide by 2.5 as this is the difference in time from 0 value
		}
	} else if (minuteCount == 120) {
		pressureAvg2 = lastPressureAvg; // store for later use.
		int32_t change = (lastPressureAvg - pressureAvg);
		if (firstPass) { // first time initial 3 hour
			dP_dt = change / 2; // note this is for t = 2 hour
		} else {
			dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
		}
	} else if (minuteCount == 155) {
		int32_t change = (lastPressureAvg - pressureAvg);
		if (firstPass) { // first time initial 3 hour
			dP_dt = (change * 2) / 5; // note this is for t = 2.5 hour
		} else {
			dP_dt = (change * 2) / 7; // divide by 3.5 as this is the difference in time from 0 value
		}
	} else if (minuteCount == 180) {
		int32_t change = (lastPressureAvg - pressureAvg);
		if (firstPass) { // first time initial 3 hour
			dP_dt = change / 3; // note this is for t = 3 hour
		} else {
			dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
		}
		pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		firstPass = false; // flag to let you know that this is on the past 3 hour mark.
	}
}

// Simple implementation of Zambretti forecaster algorithm
static char zambretti(int32_t pressure, int8_t month) {
	if (dP_dt > 25) {
		// rising pressure
		if (north == (month >= 4 && month <= 9)) pressure += 320;
		int8_t index = (103140 - pressure) / 574;
		if (index < 0) index = 0;
		if (index > 13) index = 13;
		return pgm_read_byte(&rising[index]);
	} else if (dP_dt < -25) {
		// falling pressure
		if (north == (month >= 4 && month <= 9)) pressure -= 320;
		int8_t index = (102995 - pressure) / 652;
		if (index < 0) index = 0;
		if (index > 9) index = 9;
		return pgm_read_byte(&falling[index]);
	} else {
		// steady
		int8_t index = (103081 - pressure) / 432;
		if (index < 0) index = 0;
		if (index > 16) index = 16;
		return pgm_read_byte(&steady[index]);
	}
}

// draw a scaled integer at cursor
static void drawScaled(int16_t value) {
	ili9341_puts(itostr(value, buffer, 1, 2));
}

// draw a scaled integer right aligned in an area
static void drawScaledRight(uint16_t x, uint16_t y, uint16_t w, int16_t value) {
	itostr(value, buffer, 1, 2);
	ili9341_setCursor(x + w - ili9341_strWidth(buffer), y);
	ili9341_clearTextArea(x);
	ili9341_puts(buffer);
}

// convert and draw scaled pressure. align right when w is set
static void drawPressure(uint16_t x, uint16_t y, uint16_t w, int32_t value) {
	uint8_t dec = 1, pad = 2;
	int16_t val = value / 10;
	if (b_pressure.value == PRESSURE_MMHG) {
		val = value / 13.3322;
	}
	if (b_pressure.value == PRESSURE_INHG) {
		dec = 2; pad = 3;
		val = value / 33.8639;
	}
	if (b_pressure.value == PRESSURE_PSI) {
		dec = 2; pad = 3;
		val = value / 68.9655;
	}
	itostr(val, buffer, dec, pad);
	uint16_t pos = (w) ? w - ili9341_strWidth(buffer) : 0;
	ili9341_setCursor(x + pos, y);
	if (w) ili9341_clearTextArea(x);
	ili9341_puts(buffer);
}

// draw integer at cursor
static void drawInt(int16_t value) {
	ili9341_puts(itostr(value, buffer, 0, 0));
}

// draw a code page 437 ascii character
static void drawSymbol(uint8_t c) {
	uint16_t x, y;
	uint8_t h = ili9341_fontHeight();
	ili9341_getCursor(&x, &y);
	ili9341_setFont(cp437font8x8);
	ili9341_write(c);
	ili9341_fillrect(x, y+8, 8, h-8, bgcolor);
	ili9341_setFont(Arial_bold_14);
}

// initialize array for current period
static void init_period(void) {
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		remote[i].hist[period].max_humid = 0;
		remote[i].hist[period].max_temp = -400;
		remote[i].hist[period].min_humid = 999;
		remote[i].hist[period].min_temp = 1250;
	}
	local.hist[period].max_humid = 0;
	local.hist[period].max_temp = -4000;
	local.hist[period].min_humid = 1000;
	local.hist[period].min_temp = 8500;
}

// format and draw time
static void drawTime(const time_t *timer) {
	struct tm *timeptr;
	
	timeptr = localtime(timer);
	itostr(timeptr->tm_hour, buffer, 0, 2);
	buffer[2] = ':';
	itostr(timeptr->tm_min, &buffer[3], 0, 2);
	buffer[5] = ':';
	itostr(timeptr->tm_sec, &buffer[6], 0, 2);
	ili9341_puts(buffer);
}

// draw degrees symbol with temperature unit and optional space
static void drawTempUnit(bool space) {
	drawSymbol(9);
	ili9341_write((b_degrees.value == 1) ? 'C' : 'F');
	if (space) ili9341_write(' ');
}

// initialize main screen
static void drawScreen(void) {
	view = SCREEN;
	ili9341_fillScreen(bgcolor);
	ili9341_setFont(Arial_bold_14);
	ili9341_drawRect(0,0,210,224,ILI9341_GRAY);
	ili9341_setCursor(1,0);
	ili9341_setTextColor(fgcolor, ILI9341_GRAY);
	ili9341_puts_p(PSTR("Remote stations"));
	ili9341_clearTextArea(209);
	ili9341_drawRect(210,0,110,224,ILI9341_GRAY);
	ili9341_setCursor(211,0);
	ili9341_puts_p(PSTR("Base station"));
	ili9341_clearTextArea(319);
	ili9341_setTextColor(fgcolor, bgcolor);
	ili9341_setCursor(295,15);
	drawTempUnit(false);
	ili9341_setCursor(295,40);
	ili9341_write('%');
	ili9341_setCursor(295,65);
	if (b_pressure.value == PRESSURE_MMHG) {
		ili9341_puts_p(PSTR("mm"));
		ili9341_setCursor(295,80);
	}
	ili9341_puts_p((b_pressure.value == 2) ? PSTR("Hg") : (b_pressure.value == 3) ? PSTR("\"Hg") : (b_pressure.value == 4) ? PSTR("psi") : PSTR("hPa"));
	refresh = true;
}

// returns true when dark mode is enabled or disabled
static bool changeMode(void) {
	if ((b_theme.value == THEME_AUTO && !is_day) || b_theme.value == THEME_DARK) {
		if (fgcolor != ILI9341_WHITE) {
			fgcolor = ILI9341_WHITE;
			bgcolor = ILI9341_BLACK;
			return true;
		}
	} else {
		if (fgcolor != ILI9341_BLACK) {
			fgcolor = ILI9341_BLACK;
			bgcolor = ILI9341_WHITE;
			return true;
		}
	}
	return false;
}

// convert scaled temperature in DegC to Fahrenheit
static int16_t convertTemp(int16_t temp) {
	return (b_degrees.value == DEG_CELSIUS) ? temp : temp * 36 / 20 + 320;
}

// convert to spectrum color from 0=green to 63=red
static uint16_t green_red(uint8_t value) {
	if (value > 63) value = 63;
	uint8_t green = (value > 31) ? 63 - 2 * (value - 32) : 63;
	uint8_t red = min(value, 31);
	return (red << 11) + (green << 5);
}

// convert to spectrum color from 0=blue to 63=red
static uint16_t blue_red(uint8_t value) {
	if (value > 63) value = 63;
	uint8_t blue = (value > 31) ? 31 - value : 31;
	uint8_t red = min(value, 31);
	return (red << 11) + blue;
}

// convert station pressure to sea level pressure
// Babinet's formula is accurate up to 1000 meters and within 1% to considerably greater heights.
static int32_t convertSeaLevel(int32_t pressure, int8_t temperature) {
	// altitude in whole meters and temperature in whole degrees (not scaled)
	return -pressure * (altitude + 16000 + 64 * temperature) / (altitude - 16000 - 64 * temperature);
}

// update main screen
static void updateScreen(void) {
	int16_t temp = 0;
	uint32_t pres, humid;
	history_t local_day, remote_day;
	time_t now;
	PGM_P ptr;
	
	// every 6 hours
	if (action.advance_period) {
		action.advance_period = false;
		period = (period + 1) % 4;
		if (max_period < 4) max_period++;
		init_period();
	}
	// get base station sensor readings
	do {
		bme280_get_sensor_data(&temp, &pres, &humid);
		humid = humid * 10 / 1024;
	} while (temp < -4000);
	// calculate minimum and maximum values
	if (humid > local.hist[period].max_humid) local.hist[period].max_humid = humid;
	if (humid < local.hist[period].min_humid) local.hist[period].min_humid = humid;
	if (temp > local.hist[period].max_temp) local.hist[period].max_temp = temp;
	if (temp < local.hist[period].min_temp) local.hist[period].min_temp = temp;
	memcpy(&local_day, &local.hist[0], sizeof(history_t));
	for (uint8_t j = 1; j < max_period; j++) {
		if (local.hist[j].max_humid > local_day.max_humid) local_day.max_humid = local.hist[j].max_humid;
		if (local.hist[j].min_humid < local_day.min_humid) local_day.min_humid = local.hist[j].min_humid;
		if (local.hist[j].max_temp > local_day.max_temp) local_day.max_temp = local.hist[j].max_temp;
		if (local.hist[j].min_temp < local_day.min_temp) local_day.min_temp = local.hist[j].min_temp;
	}
	// forecast
	if (action.take_sample) {
		action.take_sample = false;
		sample(pres);
		refresh = true;
	}
	time(&now);
	if (refresh) {
		refresh = false;
		// show forecast
		struct tm *timeptr;
		timeptr = localtime(&now);
		char z = zambretti(convertSeaLevel(pres, temp / 100), timeptr->tm_mon);
		if (z < 'C')
			ptr = (is_day) ? clear_icon : nt_clear_icon;
		else if (z < 'E')
			ptr = (is_day) ? mostlyclear_icon : nt_mostlyclear_icon;
		else if (z < 'O')
			ptr = (is_day) ? showers_icon : nt_showers_icon;
		else if (z < 'Y')
			ptr = rain_icon;
		else
			ptr = tstorms_icon;
		ili9341_drawRLEBitmap(218,90,ptr,64,54,fgcolor, bgcolor);
		memcpy_P(&ptr, &forecast[z - 'A'], sizeof(PGM_P));
		// draw first line
		ili9341_setCursor(211,144);
		ili9341_puts_p(ptr);
		ili9341_clearTextArea(319);
		// draw second line
		ili9341_setCursor(211,159);
		const char *split = strchr_P(ptr, 0);
		ili9341_puts_p(++split);
		ili9341_clearTextArea(319);
		// show pressure trend
		drawPressure(211,204,0,dP_dt);
		ili9341_puts_p((b_pressure.value == 2) ? PSTR(" mmHg/hr") : (b_pressure.value == 3) ? PSTR(" \"Hg/hr") : (b_pressure.value == 4) ? PSTR(" psi/hr") : PSTR(" hPa/hr"));
		ili9341_clearTextArea(319);
		// determine day/night mode
		time_t noon = solar_noon(&now);
		int32_t half = daylight_seconds(&now) / 2;
		time_t sunset = noon + half;
		time_t sunrise = noon - half;
		is_day = (now >= sunrise && now < sunset);
		if (changeMode()) drawScreen();
		// show sun rise/set time
		ili9341_setCursor(193,225);
		drawSymbol(15);
		if (is_day) {
			drawSymbol(25);
			drawTime(&sunset);
		} else {
			drawSymbol(24);
			drawTime(&sunrise);
		}
		ili9341_clearTextArea(265);
	}
	// calculate global highest and lowest values
	int16_t h_temp = temp / 10, l_temp = temp / 10;
	uint16_t h_humid = humid, l_humid = humid;
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		if (remote[i].enabled) {
			if (remote[i].temp > h_temp) h_temp = remote[i].temp;
			if (remote[i].temp < l_temp) l_temp = remote[i].temp;
			if (remote[i].humid > h_humid) h_humid = remote[i].humid;
			if (remote[i].humid < l_humid) l_humid = remote[i].humid;
		}
	}
	// show base station sensor readings
	ili9341_setFont(lcdnums14x24);
	uint16_t scale = green_red(map(temp/10,l_temp,h_temp,0,63));
	ili9341_setTextColor(b_rainbow.value ? scale : ILI9341_RED,bgcolor);
	drawScaledRight(211,15,84,convertTemp(temp/10));
	scale = blue_red(map(humid,l_humid,h_humid,0,63));
	ili9341_setTextColor(b_rainbow.value ? scale : ILI9341_BLUE,bgcolor);
	drawScaledRight(211,40,84,humid);
	ili9341_setTextColor(fgcolor,bgcolor);
	drawPressure(211,65,84,pres);
	ili9341_setFont(Arial_bold_14);
	// show minimum
	ili9341_setCursor(211,174);
	drawSymbol(25);
	drawScaled(convertTemp(local_day.min_temp/10));
	drawTempUnit(true);
	drawScaled(local_day.min_humid);
	ili9341_write('%');
	ili9341_clearTextArea(319);
	// show maximum
	ili9341_setCursor(211,190);
	drawSymbol(24);
	drawScaled(convertTemp(local_day.max_temp/10));
	drawTempUnit(true);
	drawScaled(local_day.max_humid);
	ili9341_write('%');
	ili9341_clearTextArea(319);
	// show time and date
	ili9341_setCursor(1,225);
	ctime_r(&now, buffer);
	ili9341_puts(buffer);
	ili9341_clearTextArea(192);
	// show remote sensor readings
	uint16_t y = 15;
	bool clearToBottom = false;
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		if (!remote[i].enabled) continue;
		if (remote[i].age > 900) {
			remote[i].enabled = false;
			clearToBottom = true;
			continue;
		}
		// calculate minimum and maximum values
		if (remote[i].humid > remote[i].hist[period].max_humid) remote[i].hist[period].max_humid = remote[i].humid;
		if (remote[i].humid < remote[i].hist[period].min_humid) remote[i].hist[period].min_humid = remote[i].humid;
		if (remote[i].temp > remote[i].hist[period].max_temp) remote[i].hist[period].max_temp = remote[i].temp;
		if (remote[i].temp < remote[i].hist[period].min_temp) remote[i].hist[period].min_temp = remote[i].temp;
		memcpy(&remote_day, &remote[i].hist[0], sizeof(history_t));
		for (uint8_t j = 1; j < max_period; j++) {
			if (remote[i].hist[j].max_humid > remote_day.max_humid) remote_day.max_humid = remote[i].hist[j].max_humid;
			if (remote[i].hist[j].min_humid < remote_day.min_humid) remote_day.min_humid = remote[i].hist[j].min_humid;
			if (remote[i].hist[j].max_temp > remote_day.max_temp) remote_day.max_temp = remote[i].hist[j].max_temp;
			if (remote[i].hist[j].min_temp < remote_day.min_temp) remote_day.min_temp = remote[i].hist[j].min_temp;
		}
		// show unit name
		ili9341_fillCircle(6,y+22,5,green_red(min(remote[i].age, 63)));
		ili9341_setFont(Arial_bold_14);
		ili9341_setCursor(1,y);
		ili9341_puts(remote[i].name);
		if (remote[i].unit.result) {
			// show status
			ili9341_puts_p((remote[i].unit.result == NO_RESPONSE) ? PSTR(" No response") : PSTR(" CRC error"));
			ili9341_clearTextArea(209);
			ili9341_fillrect(12,y+15,197,14,bgcolor);
			y += 17;
		} else {
			// show current temperature
			ili9341_clearTextArea(29);
			ili9341_setFont(lcdnums12x16);
			if (b_rainbow.value) ili9341_setTextColor(green_red(map(remote[i].temp,l_temp,h_temp,0,63)),bgcolor);
			drawScaledRight(30,y,60,convertTemp(remote[i].temp));
			ili9341_setTextColor(fgcolor,bgcolor);
			drawTempUnit(false);
			ili9341_setCursor(109,y);
			// show minimum
			drawSymbol(25);
			drawScaled(convertTemp(remote_day.min_temp));
			drawTempUnit(true);
			if (remote[i].unit.type != DS18B20) {
				drawScaled(remote_day.min_humid);
				ili9341_write('%');
			}
			ili9341_clearTextArea(209);
			// show maximum
			ili9341_setCursor(109,y+15);
			drawSymbol(24);
			drawScaled(convertTemp(remote_day.max_temp));
			drawTempUnit(true);
			if (remote[i].unit.type != DS18B20) {
				drawScaled(remote_day.max_humid);
				ili9341_write('%');
			}
			ili9341_clearTextArea(209);
			y += 17;
			// show current humidity
			if (remote[i].unit.type != DS18B20) {
				ili9341_setFont(lcdnums12x16);
				if (b_rainbow.value) ili9341_setTextColor(blue_red(map(remote[i].humid,l_humid,h_humid,0,63)),bgcolor);
				drawScaledRight(30,y,60,remote[i].humid);
				ili9341_setTextColor(fgcolor,bgcolor);
				ili9341_setFont(Arial_bold_14);
				ili9341_write('%');
			} else
				ili9341_fillrect(30,y,68,16,bgcolor);
		}
		y += 17;
		if (y >= 223) break;
		// show separator
		ili9341_drawhline(0,y++,209,ILI9341_GRAY);
	}
	if (clearToBottom && y < 223) ili9341_fillrect(1,y,208,223-y,bgcolor);
}

// draw and handle touchscreen button with label string/char. use non zero id for radio button group
static button_t handleButton(uint16_t x, uint16_t y, const char *str, char c, uint8_t id, button_t button) {
	uint8_t h = ili9341_fontHeight() + 2;
	uint16_t w = ili9341_strWidth_p(str) + 10;
	if (c) w += ili9341_charWidth(c);
	uint8_t set = (id) ? id : true;
	if (ts_x >= x && ts_y >= y && ts_x < x + w && ts_y < y + h) {
		button.touch = set;
	} else if (button.touch == set) {
		button.touch = false; // button released
		if (id) button.value = false;
		button.value ^= set;
	}
	// invert color when button touched or has value set
	if (button.touch == set || button.value == set) {
		ili9341_setTextColor(bgcolor, (button.touch == set) ? ILI9341_GRAY : fgcolor);
	}
	ili9341_drawRect(x, y, w, h, fgcolor);
	ili9341_setCursor(x+6, y+1);
	ili9341_clearTextArea(x+1); // padding left
	ili9341_puts_p(str);
	if (c) ili9341_write(c);
	ili9341_clearTextArea(x+w-1); // padding right
	ili9341_setTextColor(fgcolor,bgcolor);
	return button;
}

// draw and handle touchscreen slider
static int16_t handleSlider(uint16_t x, uint16_t y, int16_t w, int16_t min_val, int16_t max_val, int16_t value) {
	uint8_t h = ili9341_fontHeight() + 2;
	uint16_t color = bgcolor;
	if (ts_x >= x && ts_y >= y && ts_x < x + w && ts_y < y + h) {
		value = map((int32_t)ts_x, x, x + w, min_val, max_val + 1);
		color = ILI9341_GRAY;
	}
	ili9341_drawRect(x, y, w, h, fgcolor);
	int16_t pos = map((int32_t)value, min_val, max_val, 0, w-22);
	ili9341_fillrect(x+1, y+1, pos, h-2, color);
	ili9341_fillrect(x+pos+1, y+1, 20, h-2, fgcolor);
	ili9341_fillrect(x+pos+21, y+1, w-pos-22, h-2, color);
	return value;
}

// initialize menu screen
static void drawMenu(void) {
	view = MENU;
	ili9341_fillScreen(bgcolor);
	ili9341_drawRect(0,32,320,168,ILI9341_GRAY);
}

// initialize calibrate screen
static void drawCalibrate(void) {
	view = CALIBRATE;
	ili9341_fillScreen(bgcolor);
	ili9341_setCursor(0,80);
	ili9341_puts_p(PSTR("Use pen to touch corners of screen to calibrate"));
	ili9341_fillTriangle(0,0,20,0,0,20,ILI9341_GRAY);
	ili9341_fillTriangle(319,219,299,239,319,239,ILI9341_GRAY);
}

// clear tab
static void fillTab(void) {
	ili9341_fillrect(1,33,318,166,bgcolor);
}

// update config tab
static void updateConfig(void) {
	static button_t b_plus, b_minus;
	
	// draw time zone
	ili9341_setTextSize(1);
	ili9341_setCursor(10,37);
	ili9341_puts_p(PSTR("Time zone"));
	ili9341_setCursor(10,62);
	if (new_tz >= 0) ili9341_write('+');
	drawInt(new_tz);
	ili9341_clearTextArea(39);
	ili9341_setCursor(10,91);
	ili9341_puts_p(PSTR("Temperature"));
	ili9341_setCursor(10,145);
	ili9341_puts_p(PSTR("Pressure"));
	ili9341_setTextSize(2);
	b_minus = handleButton(40, 54, PSTR("-"), 0, 0, b_minus);
	if (b_minus.value) {
		if (new_tz > -12) new_tz--;
		b_minus.value = false;
	}
	b_plus = handleButton(59, 54, PSTR("+"), 0, 0, b_plus);
	if (b_plus.value) {
		if (new_tz < 12) new_tz++;
		b_plus.value = false;
	}
	b_dst = handleButton(95,54,PSTR("DST"),0,0,b_dst);
	b_degrees = handleButton(10,108,PSTR("Celsius"),0,1,b_degrees);
	b_degrees = handleButton(118,108,PSTR("Fahrenheit"),0,2,b_degrees);
	b_pressure = handleButton(10,162,PSTR("hPa"),0,1,b_pressure);
	b_pressure = handleButton(76,162,PSTR("mmHg"),0,2,b_pressure);
	b_pressure = handleButton(172,162,PSTR("\"Hg"),0,3,b_pressure);
	b_pressure = handleButton(236,162,PSTR("psi"),0,4,b_pressure);
}

// update LCD tab
static void updateLCD(void) {
	static button_t b_flip = {.value = true};

	// draw LED percentage
	ili9341_setTextSize(1);
	ili9341_setCursor(10,37);
	ili9341_puts_p(PSTR("Backlight "));
	drawInt(map((uint16_t)OCR0B, 0, 255, 0, 100));
	ili9341_write('%');
	ili9341_clearTextArea(119);
	// draw and handle buttons
	ili9341_setCursor(10,91);
	ili9341_puts_p(PSTR("Theme"));
	ili9341_setCursor(10,145);
	ili9341_puts_p(PSTR("Touchscreen"));
	ili9341_setTextSize(2);
	OCR0B = handleSlider(10,54,200,0,255,OCR0B);
	b_auto_led = handleButton(220,54,PSTR("Auto"),0,0,b_auto_led);
	b_theme = handleButton(10,108,PSTR("Light"),0,1,b_theme);
	b_theme = handleButton(95,108,PSTR("Dark"),0,2,b_theme);
	b_theme = handleButton(175,108,PSTR("Auto"),0,3,b_theme);
	if (changeMode()) drawMenu();
	b_flip = handleButton(10,162,PSTR("Flip"),0,0,b_flip);
	if (b_flip.value && rotation == 1) {
		rotation = 3;
		ili9341_setRotation(rotation);
		drawMenu();
	}
	if (!b_flip.value && rotation == 3) {
		rotation = 1;
		ili9341_setRotation(rotation);
		drawMenu();
	}
}

// draw coordinate in DMS
static void drawPosition(char pos, char neg, int32_t coord) {
	ili9341_write((coord < 0) ? neg : pos);
	drawInt(abs(coord / 360000));
	drawSymbol(9);
	int32_t seconds = coord % 360000;
	drawInt(abs(seconds / 6000));
	ili9341_write('\'');
	ili9341_puts(itostr(abs(seconds % 6000), buffer, 2, 3));
	ili9341_write('"');
}

// update etc tab
static void updateEtc(void) {
	ili9341_setTextSize(1);
	ili9341_setCursor(10,37);
	ili9341_puts_p(PSTR("Digits"));
	ili9341_setCursor(10,91);
	ili9341_puts_p(PSTR("GPS"));
	if (gps_valid) {
		ili9341_setCursor(10,108);
		ili9341_puts_p(PSTR("Altitude "));
		drawInt(altitude);
		ili9341_write('m');
		ili9341_clearTextArea(119);
		ili9341_setCursor(10,123);
		drawPosition('N', 'S', gps_latitude);
		ili9341_clearTextArea(119);
		ili9341_setCursor(10,138);
		drawPosition('E', 'W', gps_longitude);
		ili9341_clearTextArea(119);
		ili9341_setCursor(10,153);
		time_t now = mk_gmtime(&gps_time);
		ctime_r(&now, buffer);
		ili9341_puts(buffer);
		ili9341_clearTextArea(192);
		ili9341_setCursor(10,168);
		ili9341_puts_p(PSTR("Satellites "));
		drawInt(gps_numsats);
		ili9341_clearTextArea(119);
	}
	ili9341_setTextSize(2);
	b_rainbow = handleButton(10,54,PSTR("Rainbow"),0,0,b_rainbow);
}

// update names tab
static void updateNames(void) {
	static button_t b_key[39];
	static uint8_t j=0;
	uint8_t i=0, c, y=80;
	uint16_t x=3;

	// draw remote name with cursor
	ili9341_setTextSize(1);
	ili9341_setCursor(10,47);
	ili9341_puts_p(PSTR("Remote station "));
	drawInt(j);
	ili9341_puts_p(PSTR(": "));
	ili9341_puts(remote[j].name);
	if (action.blink) ili9341_write('_');
	ili9341_clearTextArea(180);
	// draw and handle buttons
	ili9341_setFont(cp437font8x8);
	ili9341_setTextSize(2);
	uint8_t k = strlen(remote[j].name);
	while ((c = pgm_read_byte((b_key[29].value) ? &keys2[i] : &keys1[i]))) {
		b_key[i] = handleButton(x, y, PSTR(""), c, 0, b_key[i]);
		if (b_key[i].value && c != 0x1E) {
			b_key[i].value = false;
			b_key[29].value = false;
			if (c == 0x18 && j) j--;
			if (c == 0x19 && j < SENSOR_COUNT-1) j++;
			if (c == 0x1B) {
				remote[j].name[k-1] = 0; // backspace
			} else if (k < 3) {
				remote[j].name[k] = c;
				remote[j].name[k+1] = 0;
				eeprom_update_block(remote[j].name, &nv_names[j], sizeof(remote[j].name));
			}
		}
		x += 32;
		if (x > 303) {
			x = 3;
			y += 22;
		}
		i++;
		if (i == 20) x += 16;
	}
	ili9341_setFont(Arial_bold_14);
}

// update menu screen
static void updateMenu(void) {
	static button_t b_ok, b_cancel, b_calibrate, b_tab = {.value = 1};
	uint8_t old_tab = b_tab.value;
	
	ili9341_setTextSize(2);
	b_tab = handleButton(0,0,PSTR("Config"),0,TAB_CONFIG,b_tab);
	b_tab = handleButton(96,0,PSTR("LCD"),0,TAB_LCD,b_tab);
	b_tab = handleButton(160,0,PSTR("Etc"),0,TAB_ETC,b_tab);
	b_tab = handleButton(214,0,PSTR("Names"),0,TAB_NAMES,b_tab);
	if (b_tab.value != old_tab) fillTab();
	if (b_tab.value == TAB_CONFIG) {
		updateConfig();
	} else if (b_tab.value == TAB_LCD) {
		updateLCD();
		b_calibrate = handleButton(75,162,PSTR("Calibrate"),0,0,b_calibrate);
	} else if (b_tab.value == TAB_ETC) {
		updateEtc();
	} else {
		updateNames();
	}
	b_ok = handleButton(80,204,PSTR("Ok"),0,0,b_ok);
	b_cancel = handleButton(150,204,PSTR("Cancel"),0,0,b_cancel);
	ili9341_setTextSize(1);
	if (b_calibrate.value) {
		b_calibrate.value = false;
		drawCalibrate();
	}
	if (b_ok.value) {
		b_ok.value = false;
		tz = new_tz;
		dst = b_dst.value;
		set_dst(dst ? eu_dst : NULL); // daylight saving time
		set_zone(tz * ONE_HOUR); // adjust time zone
		drawScreen();
	}
	if (b_cancel.value) {
		b_cancel.value = false;
		new_tz = tz;
		b_dst.value = dst;
		OCR0B = old_ocr0b;
		b_auto_led.value = old_auto_led;
		b_degrees.value = old_degrees;
		b_pressure.value = old_pressure;
		b_theme.value = old_theme;
		b_rainbow.value = old_rainbow;
		changeMode();
		drawScreen();
	}
}

// update calibrate screen
static void updateCalibrate(void) {
	static button_t b_done, b_reset;
	
	ili9341_fillCircle(ts_x,ts_y,3,ILI9341_BLUE);
	ili9341_setTextSize(2);
	b_reset = handleButton(200,136,PSTR("Reset"),0,0,b_reset);
	b_done = handleButton(60,136,PSTR("Done"),0,0,b_done);
	ili9341_setTextSize(1);
	if (b_reset.value) {
		b_reset.value = false;
		// reset bounds
		ts_xMax = ts_xMin = ts_yMax = ts_yMin = 0xffff;
		drawCalibrate();
	}
	if (b_done.value) {
		b_done.value = false;
		drawMenu();
	}
}

// read names from EEPROM if set or use default values
static void init_eeprom(void) {
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		eeprom_read_block(remote[i].name, &nv_names[i], sizeof(remote[i].name));
		if (remote[i].name[0] == 0xFF) {
			remote[i].name[0] = '#';
			remote[i].name[1] = i<0xA ? '0'+i : 'A'+i-0xA;
			remote[i].name[2] = 0;
		}
	}
}

int main(void) {
	uint8_t length = 0xff, prev, c = 0;
	uint16_t crc = 0;
	time_t last = 0;
	
	timer0_init();
	timer2_init();
	uart_init(UART_BAUD_SELECT(1200, F_CPU));
	uart_puts_P("\r\nILI9341");
	ili9341_init();
	if (ili9341_readcommand8(ILI9341_RDSELFDIAG) != 0xc0)
		uart_puts_P(" fail");
	ili9341_setRotation(rotation);
	xpt2046_init();
	drawScreen();
	uart_puts_P("\r\nBME280");
	if (bme280_init())
		uart_puts_P(" fail");
	suart_init();
	init_adc();
	init_period();
	set_zone(1 * ONE_HOUR); // adjust time zone
	set_dst(eu_dst);        // enable daylight saving time
	init_eeprom();

	// Main loop
	while (1) {
		uint16_t start = millis;
		if (xpt2046_isTouching() && view == SCREEN) {
			old_auto_led = b_auto_led.value;
			old_theme = b_theme.value;
			old_degrees = b_degrees.value;
			old_pressure = b_pressure.value;
			old_rainbow = b_rainbow.value;
			old_ocr0b = OCR0B;
			if (ts_xMax != ts_xMin)
				drawMenu();
			else
				drawCalibrate();
		}
		xpt2046_getPosition(0xff, rotation);
		if (b_auto_led.value)
			OCR0B = ~read_adc(2); // adjust back light
		if (view == MENU) {
			updateMenu();
		} else if (view == CALIBRATE) {
			updateCalibrate();
		} else if (action.update_screen) {
			action.update_screen = false;
			updateScreen();
			ili9341_drawRLEBitmap(294,109,(gps_fix) ? gps_icon : no_gps_icon,24,24,fgcolor, bgcolor);
			ili9341_setCursor(272,225);
			drawInt(millis - start);
			ili9341_puts_p(PSTR("ms"));
			ili9341_clearTextArea(319);
		}
		if (view) {
			ili9341_setCursor(1,225);
			drawInt(millis - start);
			ili9341_puts_p(PSTR("ms"));
			ili9341_clearTextArea(49);
		}
		while (suart_available()) {
			if (gps_decode(suart_getc())) {
				altitude = gps_altitude / 100;
				north = gps_latitude > 0;
				set_position(gps_latitude / 100, gps_longitude / 100);
				time_t timestamp = mk_gmtime(&gps_time);
				if (difftime(timestamp, last) == 1) set_system_time(timestamp);
				last = timestamp;
			}
		}
		while (uart_available()) {
			prev = c;
			c = uart_getc();
			if (prev == 0x55 && c == 0x55) {
				// Preamble received
				length = 0;
				crc = 0xffff;
			} else if (length < sizeof(rxData)) {
				// Receive packet data
				if (length < sizeof(rxData) - sizeof(crc))
					crc = _crc16_update(crc, c);
				((uint8_t *)&rxData)[length++] = c;
			}
			if (length == sizeof(rxData)) {
				// Validate received packet
				length = 0xff;
				if (rxData.crc == crc) {
					remote[rxData.unit.id].enabled = true;
					remote[rxData.unit.id].age = 0;
					remote[rxData.unit.id].unit = rxData.unit;
					remote[rxData.unit.id].temp = rxData.temp;
					remote[rxData.unit.id].humid = (rxData.humid > 999) ? 999 : rxData.humid;
				}
			}
		}
	}
}

