/*
 * Title   : Weather station
 * Hardware: ATmega328P @ 8 MHz, RFM210LCF 433MHz ASK/OOK receiver module,
 *           BME280 digital humidity, pressure and temperature sensor,
 *           ILI9341 TFT LCD driver, XPT2046 touch screen controller, 
 *           GL5528 LDR, NEO-6M GPS module
 *
 * Created: 14-12-2019 19:43:50
 * Author : Tim Dorssers
 *
 * A BME280 barometric sensor is used to implement a simple weather forecasting
 * algorithm by calculating Pa/h. Up to 16 wireless AHT20, AM2320 or DS18B20
 * sensors transmit NRZ encoded packets with CRC data using a 433MHz ASK/OOK
 * module connected to the hardware UART at 1200 baud. Up to 6 remote sensors
 * are shown on the LCD. Minimum and maximum temperatures and humidity values
 * are stored in 4 six hour periods. The LCD back light level is auto adjusted
 * using a 5528 LDR. A software UART implementation using Timer1 is interfacing
 * with the GPS module at 9600 baud. Libc timekeeping uses the GPS position to
 * determine day and night display mode. Time zone and daylight saving time can
 * be set using the menu which is shown by touching the main screen. The touch
 * screen requires calibration which is done by touching the screen edges. The
 * remote sensors can be named with 3 characters from the menu. Calibration and
 * names are stored in EEPROM.
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

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <util/delay.h>
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
typedef enum {DS18B20, AM2320, AHT20} type_t;
typedef enum {DISABLED, ENABLED, IDLE} status_t;

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
	status_t status;
	uint16_t timestamp;
	unit_t unit;
	uint16_t humid;
	int16_t temp;
	history_t hist[4];
	char name[4];
} sensor_t;

typedef struct {
	int16_t min_temp, max_temp;
	uint32_t min_humid, max_humid;
} bmehist_t;

typedef struct {
	int16_t temp;
	uint32_t humid;
	bmehist_t hist[4];
} bmesensor_t;

#define SENSOR_COUNT 16 // times 44 bytes = 704 bytes
packet_t rxData;
sensor_t remote[SENSOR_COUNT];
bmesensor_t local;
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
bool dst = true, old_auto_led, is_day = false, refresh;
button_t b_dst = {.value = true}, b_auto_led = {.value = true};
button_t b_dark_mode = {.value = true}, b_auto_mode = {.value = true};
button_t b_fahrenheit, b_inhg;
int8_t tz = 1, new_tz = 1;
uint8_t old_ocr0b, rotation = 3;
uint16_t fgcolor = ILI9341_WHITE, bgcolor = ILI9341_BLACK;

const char keys1[] PROGMEM = "1234567890qwertyuiopasdfghjkl\x1Ezxcvbnm\x19\x1B";
const char keys2[] PROGMEM = "!@#$%^&*()QWERTYUIOPASDFGHJKL\x1EZXCVBNM\x18\x1B";

const char no_gps_icon[] PROGMEM = {
	0x00, 0x00, 0xC6, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x38,
	0x00, 0x00, 0x7C, 0xF8, 0x00, 0xEE, 0xFC, 0x03, 0xC6, 0xDE, 0x07, 0x00,
	0x0E, 0x22, 0x00, 0x0E, 0x78, 0x00, 0x1F, 0xFE, 0x00, 0x9F, 0x7F, 0x00,
	0x3F, 0x70, 0x00, 0x3F, 0x30, 0x01, 0x7E, 0x90, 0x03, 0xFE, 0x10, 0x03,
	0xFE, 0x11, 0x07, 0xFC, 0x03, 0x07, 0xF8, 0x0F, 0x06, 0xF8, 0x3F, 0x06,
	0xF0, 0xFF, 0x07, 0xC0, 0xFF, 0x03, 0x80, 0xFF, 0x01, 0x00, 0x3C, 0x00,
};

const char gps_icon[] PROGMEM = {
	0x00, 0x60, 0x00, 0x00, 0xE0, 0x03, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0x1E,
	0x00, 0xE0, 0x3C, 0xF8, 0xE0, 0x3B, 0xFC, 0x83, 0x77, 0xDE, 0x07, 0x66,
	0x0E, 0x22, 0x6E, 0x0E, 0x78, 0xEC, 0x1F, 0xFE, 0xEC, 0x9F, 0x7F, 0x00,
	0x3F, 0x70, 0x00, 0x3F, 0x30, 0x01, 0x7E, 0x90, 0x03, 0xFE, 0x10, 0x03,
	0xFE, 0x11, 0x07, 0xFC, 0x03, 0x07, 0xF8, 0x0F, 0x06, 0xF8, 0x3F, 0x06,
	0xF0, 0xFF, 0x07, 0xC0, 0xFF, 0x03, 0x80, 0xFF, 0x01, 0x00, 0x3C, 0x00,
};

PGM_P const icons[6] PROGMEM = {cloudy_icon, clear_icon, rain_icon, mostlyclear_icon, tstorms_icon, unknown_icon};

#define LAST_SAMPLES_COUNT  5
int32_t dP_dt;

const char weather0[] PROGMEM = "stable";
const char weather1[] PROGMEM = "sunny";
const char weather2[] PROGMEM = "cloudy";
const char weather3[] PROGMEM = "unstable";
const char weather4[] PROGMEM = "thunderstorm";
const char weather5[] PROGMEM = "unknown";
PGM_P const forecast[6] PROGMEM = {weather0, weather1, weather2, weather3, weather4, weather5};

#define map(x,in_min,in_max,out_min,out_max) (((x)-(in_min))*((out_max)-(out_min))/((in_max)-(in_min))+(out_min))

// Time based event triggers
ISR(TIMER2_COMPA_vect) {
	static uint8_t sec=0;
	static uint16_t msec=0, min=0;

	millis++;
	msec++;
	if (msec == 1000) {
		msec = 0;
		system_tick();
		sec++;
		action.blink = !action.blink;
		action.update_screen = true;
		if (sec == 60) {
			sec = 0;
			min++;
			action.take_sample = true;
			if (min == 360) {
				min = 0;
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
static void i_to_a(int16_t num, char *str, uint8_t decimal, uint8_t padding) {
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
	strrev(str);
}

// Forecast algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in Pa -->  forecast done by calculating Pa/h
static uint8_t sample(uint32_t pressure) {
	static uint32_t lastPressureSamples[LAST_SAMPLES_COUNT];
	static uint8_t minuteCount = 0;
	static bool firstPass = true;
	static uint32_t pressureAvg;
	static uint32_t pressureAvg2;
	uint8_t weatherStatus = 5;

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

	if (minuteCount < 35 && firstPass) { //if time is less than 35 min on the first 3 hour interval.
		weatherStatus = 5; // Unknown
	} else if (dP_dt < -250) {
		weatherStatus = 4; // Quickly falling LP, Thunderstorm, not stable
	} else if (dP_dt > 250) {
		weatherStatus = 3; // Quickly rising HP, not stable weather
	} else if ((dP_dt > -250) && (dP_dt < -50)) {
		weatherStatus = 2; // Slowly falling Low Pressure System, stable rainy weather
	} else if ((dP_dt > 50) && (dP_dt < 250)) {
		weatherStatus = 1; // Slowly rising HP stable good weather
	} else if ((dP_dt > -50) && (dP_dt < 50)) {
		weatherStatus = 0; // Stable weather
	}

	return weatherStatus;
}

// draw a scaled integer at cursor
static void drawScaled(int16_t value) {
	i_to_a(value, buffer, 1, 2);
	ili9341_puts(buffer);
}

// draw a scaled integer left aligned in an area
static void drawScaledLeft(uint16_t x, uint16_t y, uint16_t w, int16_t value) {
	i_to_a(value, buffer, 1, 2);
	ili9341_setCursor(x + w - ili9341_strWidth(buffer), y);
	ili9341_clearTextArea(x);
	ili9341_puts(buffer);
}

// convert to inHg and draw scaled pressure. align left when w is set
static void drawPressure(uint16_t x, uint16_t y, uint16_t w, int32_t value) {
	uint8_t dec = 1, pad = 2;
	int16_t val = value / 10;
	if (b_inhg.value) {
		dec = 2; pad = 3;
		val = value / 33.8639;
	}
	i_to_a(val, buffer, dec, pad);
	uint16_t pos = (w) ? w - ili9341_strWidth(buffer) : 0;
	ili9341_setCursor(x + pos, y);
	if (w) ili9341_clearTextArea(x);
	ili9341_puts(buffer);
}

// draw integer at cursor
static void drawInt(int16_t value) {
	i_to_a(value, buffer, 0, 0);
	ili9341_puts(buffer);
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
	local.hist[period].min_humid = 102400;
	local.hist[period].min_temp = 8500;
}

// format and draw time
static void drawTime(const time_t *timer) {
	struct tm *timeptr;
	
	timeptr = localtime(timer);
	i_to_a(timeptr->tm_hour, buffer, 0, 2);
	buffer[2] = ':';
	i_to_a(timeptr->tm_min, &buffer[3], 0, 2);
	buffer[5] = ':';
	i_to_a(timeptr->tm_sec, &buffer[6], 0, 2);
	ili9341_puts(buffer);
}

// draw degrees symbol with temperature unit and optional space
static void drawTempUnit(bool space) {
	drawSymbol(9);
	ili9341_write(b_fahrenheit.value ? 'F' : 'C');
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
	ili9341_puts_p(b_inhg.value ? PSTR("\"Hg") : PSTR("hPa"));
	refresh = true;
}

// returns true when dark mode is enabled or disabled
static bool changeMode(void) {
	if (b_dark_mode.value) {
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
	return (b_fahrenheit.value) ? temp * 36 / 20 + 320 : temp;
}

// update main screen
static void updateScreen(void) {
	static uint8_t weatherStatus = 5;
	int16_t temp=0;
	uint32_t pres, humid;
	history_t remote_day;
	bmehist_t local_day;
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
	} while (temp < -4000);
	// calculate minimum and maximum values
	if (humid > local.hist[period].max_humid) local.hist[period].max_humid = humid;
	if (humid < local.hist[period].min_humid) local.hist[period].min_humid = humid;
	if (temp > local.hist[period].max_temp) local.hist[period].max_temp = temp;
	if (temp < local.hist[period].min_temp) local.hist[period].min_temp = temp;
	memcpy(&local_day, &local.hist[0], sizeof(bmehist_t));
	for (uint8_t j = 1; j < max_period; j++) {
		if (local.hist[j].max_humid > local_day.max_humid) local_day.max_humid = local.hist[j].max_humid;
		if (local.hist[j].min_humid < local_day.min_humid) local_day.min_humid = local.hist[j].min_humid;
		if (local.hist[j].max_temp > local_day.max_temp) local_day.max_temp = local.hist[j].max_temp;
		if (local.hist[j].min_temp < local_day.min_temp) local_day.min_temp = local.hist[j].min_temp;
	}
	// forecast
	time(&now);
	if (action.take_sample) {
		action.take_sample = false;
		weatherStatus = sample(pres);
		refresh = true;
	}
	if (refresh) {
		refresh = false;
		// show forecast
		memcpy_P(&ptr, &icons[weatherStatus], sizeof(PGM_P));
		ili9341_drawXBitmap(218,90,ptr,64,64,fgcolor, bgcolor);
		memcpy_P(&ptr, &forecast[weatherStatus], sizeof(PGM_P));
		ili9341_setCursor(211,154);
		ili9341_puts_p(ptr);
		ili9341_clearTextArea(319);
		drawPressure(211,200,0,dP_dt);
		ili9341_puts_p(b_inhg.value ? PSTR(" \"Hg/hr") : PSTR(" hPa/hr"));
		ili9341_clearTextArea(319);
		// determine day/night mode
		time_t noon = solar_noon(&now);
		int32_t half = daylight_seconds(&now) / 2;
		time_t sunset = noon + half;
		time_t sunrise = noon - half;
		is_day = (now >= sunrise && now < sunset);
		if (b_auto_mode.value) b_dark_mode.value = !is_day;
		if (changeMode()) drawScreen();
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
	// show base station sensor readings
	ili9341_setFont(lcdnums14x24);
	ili9341_setTextColor(ILI9341_RED,bgcolor);
	drawScaledLeft(211,15,84,convertTemp(temp/10));
	ili9341_setTextColor(ILI9341_BLUE,bgcolor);
	drawScaledLeft(211,40,84,humid*10/1024);
	ili9341_setTextColor(fgcolor,bgcolor);
	drawPressure(211,65,84,pres);
	ili9341_setFont(Arial_bold_14);
	// show minimum
	ili9341_setCursor(211,170);
	drawSymbol(25);
	drawScaled(convertTemp(local_day.min_temp/10));
	drawTempUnit(true);
	drawScaled(local_day.min_humid*10/1024);
	ili9341_write('%');
	ili9341_clearTextArea(319);
	// show maximum
	ili9341_setCursor(211,186);
	drawSymbol(24);
	drawScaled(convertTemp(local_day.max_temp/10));
	drawTempUnit(true);
	drawScaled(local_day.max_humid*10/1024);
	ili9341_write('%');
	ili9341_clearTextArea(319);
	// show time and date
	ili9341_setCursor(1,225);
	ctime_r(&now, buffer);
	ili9341_puts(buffer);
	ili9341_clearTextArea(192);
	// show remote sensor readings
	uint16_t y = 15;
	for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
		if (remote[i].status == DISABLED) continue;
		if (millis - remote[i].timestamp > 8960) remote[i].status = IDLE;
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
		ili9341_fillCircle(6,y+22,5,(remote[i].status == IDLE) ? ILI9341_RED : ILI9341_LIME);
		ili9341_setCursor(1,y);
		ili9341_setFont(Arial_bold_14);
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
			drawScaledLeft(30,y,60,convertTemp(remote[i].temp));
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
				drawScaledLeft(30,y,60,remote[i].humid);
				ili9341_setFont(Arial_bold_14);
				ili9341_write('%');
			}
		}
		y += 17;
		// show separator
		ili9341_drawhline(0,y,209,ILI9341_GRAY);
		y++;
		if (y >= 224) break;
	}
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

// update settings tab
static void updateSettings(void) {
	// draw time zone
	ili9341_setTextSize(1);
	ili9341_setCursor(10,40);
	ili9341_puts_p(PSTR("GMT "));
	if (new_tz >= 0) ili9341_write('+');
	drawInt(new_tz);
	ili9341_clearTextArea(100);
	ili9341_setCursor(10,108);
	ili9341_puts_p(PSTR("Degrees"));
	ili9341_setCursor(10,148);
	ili9341_puts_p(PSTR("Pressure"));
	ili9341_setTextSize(2);
	new_tz = handleSlider(10,60,200,-12,12,new_tz);
	b_dst = handleButton(220,60,PSTR("DST"),0,0,b_dst);
	b_fahrenheit = handleButton(130,100,PSTR("Fahrenheit"),0,0,b_fahrenheit);
	b_inhg = handleButton(226,140,PSTR("\"Hg"),0,0,b_inhg);
}

// update display tab
static void updateDisplay(void) {
	static button_t b_flip = {.value = true};

	// draw LED percentage
	ili9341_setTextSize(1);
	ili9341_setCursor(10,40);
	ili9341_puts_p(PSTR("Backlight "));
	drawInt(map((uint16_t)OCR0B, 0, 255, 0, 100));
	ili9341_write('%');
	ili9341_clearTextArea(119);
	// draw and handle buttons
	ili9341_setCursor(10,108);
	ili9341_puts_p(PSTR("Dark mode"));
	ili9341_setCursor(10,148);
	ili9341_puts_p(PSTR("Touchscreen"));
	ili9341_setTextSize(2);
	OCR0B = handleSlider(10,60,200,0,255,OCR0B);
	b_auto_led = handleButton(220,60,PSTR("Auto"),0,0,b_auto_led);
	b_dark_mode = handleButton(114,100,PSTR("Enable"),0,0,b_dark_mode);
	b_auto_mode = handleButton(220,100,PSTR("Auto"),0,0,b_auto_mode);
	if (b_auto_mode.value) b_dark_mode.value = !is_day;
	if (changeMode()) drawMenu();
	b_flip = handleButton(103,140,PSTR("Flip"),0,0,b_flip);
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
	b_tab = handleButton(0,0,PSTR("Settings"),0,1,b_tab);
	b_tab = handleButton(120,0,PSTR("Display"),0,2,b_tab);
	b_tab = handleButton(224,0,PSTR("Names"),0,3,b_tab);
	if (b_tab.value != old_tab) fillTab();
	if (b_tab.value == 1) {
		updateSettings();
	} else if (b_tab.value == 2) {
		updateDisplay();
		b_calibrate = handleButton(166,140,PSTR("Calibrate"),0,0,b_calibrate);
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
	bool gps_valid = false;
	
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
			ili9341_drawXBitmap(294,114,(gps_valid) ? gps_icon : no_gps_icon,24,24,fgcolor, bgcolor);
			gps_valid = false;
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
			if (decode(suart_getc())) {
				set_system_time(mk_gmtime(&gps_time));
				set_position(gps_latitude / 100, gps_longitude / 100);
				gps_valid = true;
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
					remote[rxData.unit.id].status = ENABLED;
					remote[rxData.unit.id].timestamp = millis;
					remote[rxData.unit.id].unit = rxData.unit;
					remote[rxData.unit.id].temp = rxData.temp;
					remote[rxData.unit.id].humid = rxData.humid;
				}
			}
		}
	}
}

