/*
 * Title   : Weather station
 * Hardware: ATmega328P @ 8 MHz, RFM210LCF 433MHz ASK/OOK receiver module,
 *           BME280 digital humidity, pressure and temperature sensor,
 *           ILI9341 driver for 240x320 TFT LCD module, GL5528 LDR,
 *           NEO-6M GPS module
 *
 * Created: 14-12-2019 19:43:50
 * Author : Tim Dorssers
 *
 * A barometric sensor is used to implement a simple weather forecasting
 * algorithm by calculating Pa/h. Up to 16 remote AHT20, AM2320 or DS18B20
 * sensors can transmit NRZ encoded packets with CRC data using 433MHz ASK/OOK
 * modules connected to the hardware UART at 1200 baud. Up to 6 remote sensors
 * are shown on the LCD. Minimum and maximum temperatures and humidity values
 * are stored in 4 six hour periods. The LCD back light level is auto adjusted
 * using the LDR. A software UART implementation using Timer1 is interfacing
 * with the GPS module at 9600 baud. Libc timekeeping uses the position to
 * determine day and night display mode.
 */

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
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
	uint8_t timestamp;
	unit_t unit;
	uint16_t humid;
	int16_t temp;
	history_t hist[4];
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

packet_t rxData;
sensor_t remote[16];
bmesensor_t local;
uint8_t period = 0, max_period = 1;

typedef union {
	struct {
		uint8_t update_screen:1;
		uint8_t take_sample:1;
		uint8_t advance_period:1;
	};
	uint8_t all;
} action_t;

volatile action_t action;
volatile uint8_t sec=0;
volatile uint16_t msec=0, min=0;

uint16_t fgcolor = ILI9341_WHITE, bgcolor = ILI9341_BLACK;

const char signal_icon[] PROGMEM = {
	0x80, 0x1F, 0x00, 0xF0, 0xFF, 0x01, 0x7C, 0xC0, 0x03, 0x1E, 0x00, 0x0F,
	0x87, 0x3F, 0x0E, 0xE3, 0x7F, 0x08, 0xF0, 0xF0, 0x01, 0x38, 0x80, 0x03,
	0x1C, 0x00, 0x03, 0x80, 0x1F, 0x00, 0xC0, 0x7F, 0x00, 0xC0, 0x70, 0x00,
	0x40, 0x40, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x1F, 0x00,
};

PGM_P const PROGMEM icons[6]={cloudy_icon,clear_icon,rain_icon,mostlyclear_icon,tstorms_icon,unknown_icon};

#define LAST_SAMPLES_COUNT  5
int32_t dP_dt;

const char PROGMEM weather0[]="stable";
const char PROGMEM weather1[]="sunny";
const char PROGMEM weather2[]="cloudy";
const char PROGMEM weather3[]="unstable";
const char PROGMEM weather4[]="thunderstorm";
const char PROGMEM weather5[]="unknown";
PGM_P const PROGMEM forecast[6]={weather0,weather1,weather2,weather3,weather4,weather5};

// Time based event triggers
ISR(TIMER2_COMPA_vect) {
	msec++;
	if (msec == 1000) {
		msec = 0;
		system_tick();
		sec++;
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
	//OCR0B = 0xAB; /* 66% duty cycle */
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
	do {
		str[i++] = '0' + sum % 10;
		if (i == decimal) 
			str[i++] = '.';
	} while ((sum /= 10));
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
uint8_t sample(uint32_t pressure) {
	static uint32_t lastPressureSamples[LAST_SAMPLES_COUNT];
	static uint8_t minuteCount = 0;
	static bool firstPass = true;
	static uint32_t pressureAvg;
	static uint32_t pressureAvg2;
	//static int32_t dP_dt;
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
	} else if (dP_dt < (-250)) {
		weatherStatus = 4; // Quickly falling LP, Thunderstorm, not stable
	} else if (dP_dt > 250) {
		weatherStatus = 3; // Quickly rising HP, not stable weather
	} else if ((dP_dt > (-250)) && (dP_dt < (-50))) {
		weatherStatus = 2; // Slowly falling Low Pressure System, stable rainy weather
	} else if ((dP_dt > 50) && (dP_dt < 250)) {
		weatherStatus = 1; // Slowly rising HP stable good weather
	} else if ((dP_dt > (-50)) && (dP_dt < 50)) {
		weatherStatus = 0; // Stable weather
	}

	return weatherStatus;
}

// draw a scaled integer at cursor
static void drawScaled(int16_t value) {
	i_to_a(value, buffer, 1, 3);
	ili9341_puts(buffer);
}

// draw a scaled integer left aligned in an area
static void drawScaledLeft(uint16_t x, uint16_t y, uint16_t w, int16_t value) {
	i_to_a(value, buffer, 1, 3);
	uint8_t sw = ili9341_strWidth(buffer);
	ili9341_setCursor(x + w - sw, y);
	ili9341_clearTextArea(x);
	ili9341_puts(buffer);
}

static void drawSymbol(uint8_t c) {
	ili9341_setFont(cp437font8x8);
	ili9341_write(c);
	ili9341_setFont(Arial_bold_14);
}

// initialize array for current period
static void init_period(void) {
	for (uint8_t i = 0; i < 15; i++) {
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

static void drawScreen(void) {
	ili9341_fillScreen(bgcolor);
	ili9341_setFont(Arial_bold_14);
	ili9341_drawRect(0,0,190,224,ILI9341_GRAY);
	ili9341_setCursor(1,0);
	ili9341_setTextColor(fgcolor, ILI9341_GRAY);
	ili9341_puts_p(PSTR("Remote stations"));
	ili9341_clearTextArea(189);
	ili9341_drawRect(200,0,120,224,ILI9341_GRAY);
	ili9341_setCursor(201,0);
	ili9341_puts_p(PSTR("Base station"));
	ili9341_clearTextArea(319);
	ili9341_setTextColor(fgcolor, bgcolor);
	ili9341_setCursor(285,15);
	drawSymbol(9);
	ili9341_write('C');
	ili9341_setCursor(285,40);
	ili9341_write('%');
	ili9341_setCursor(285,65);
	ili9341_puts_p(PSTR("hPa"));
}

static void updateScreen(void) {
	int16_t temp;
	uint32_t pres, humid;
	history_t remote_day;
	bmehist_t local_day;
	time_t now;
	
	// every 6 hours
	if (action.advance_period) {
		period = (period + 1) % 4;
		if (max_period < 4) max_period++;
		init_period();
	}
	bme280_get_sensor_data(&temp, &pres, &humid);
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
		uint8_t ws = sample(pres);
		// determine day/night mode
		time_t noon = solar_noon(&now);
		int32_t half = daylight_seconds(&now) / 2;
		time_t sunset = noon + half;
		time_t sunrise = noon - half;
		if (now >= sunrise && now < sunset) {
			if (fgcolor != ILI9341_BLACK) {
				fgcolor = ILI9341_BLACK;
				bgcolor = ILI9341_WHITE;
				drawScreen();
			}
			ili9341_setCursor(193,225);
			drawSymbol(15);
			drawSymbol(25);
			drawTime(&sunset);
		} else {
			if (fgcolor != ILI9341_WHITE) {
				fgcolor = ILI9341_WHITE;
				bgcolor = ILI9341_BLACK;
				drawScreen();
			}
			ili9341_setCursor(193,225);
			drawSymbol(15);
			drawSymbol(24);
			drawTime(&sunrise);
		}
		ili9341_clearTextArea(265);
		// show forecast
		PGM_P ptr;
		memcpy_P(&ptr, &icons[ws], sizeof(PGM_P));
		ili9341_drawXBitmap(208,90,ptr,64,64,fgcolor, bgcolor);
		memcpy_P(&ptr, &forecast[ws], sizeof(PGM_P));
		ili9341_setCursor(201,154);
		ili9341_puts_p(ptr);
		ili9341_clearTextArea(319);
		ili9341_setCursor(201,200);
		i_to_a(dP_dt/10, buffer, 1, 3);
		ili9341_puts(buffer);
		ili9341_puts_p(PSTR(" hPa/hr"));
		ili9341_clearTextArea(319);
	}
	// show base station sensor readings
	ili9341_setFont(lcdnums14x24);
	ili9341_setTextColor(ILI9341_RED,bgcolor);
	drawScaledLeft(201,15,84,temp/10);
	ili9341_setTextColor(ILI9341_BLUE,bgcolor);
	drawScaledLeft(201,40,84,humid*10/1024);
	ili9341_setTextColor(fgcolor,bgcolor);
	drawScaledLeft(201,65,84,pres/10);
	ili9341_setFont(Arial_bold_14);
	// show minimum
	ili9341_setCursor(201,170);
	drawSymbol(25);
	drawScaled(local_day.min_temp/10);
	ili9341_puts_p(PSTR("C "));
	drawScaled(local_day.min_humid*10/1024);
	ili9341_write('%');
	ili9341_clearTextArea(319);
	// show maximum
	ili9341_setCursor(201,186);
	drawSymbol(24);
	drawScaled(local_day.max_temp/10);
	ili9341_puts_p(PSTR("C "));
	drawScaled(local_day.max_humid*10/1024);
	ili9341_write('%');
	ili9341_clearTextArea(319);
	// show clock
	ili9341_setCursor(1,225);
	ctime_r(&now, buffer);
	ili9341_puts(buffer);
	ili9341_clearTextArea(192);
	// show remote sensor readings
	uint16_t y = 15;
	for (uint8_t i=0; i<15; i++) {
		if (remote[i].status == DISABLED) continue;
		if ((sec + ((remote[i].timestamp > sec) ? 60 : 0)) - remote[i].timestamp > 9) remote[i].status = IDLE;
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
		// show unit number and status
		ili9341_fillCircle(6,y+22,5,(remote[i].status == IDLE) ? ILI9341_RED : ILI9341_LIME);
		ili9341_setCursor(1,y);
		ili9341_setFont(Arial_bold_14);
		ili9341_write('#');
		ili9341_write(i<0xA ? '0'+i : 'A'+i-0xA);
		ili9341_write(' ');
		if (remote[i].unit.result == NO_RESPONSE) {
			ili9341_puts_p(PSTR("No response"));
			ili9341_clearTextArea(189);
			ili9341_fillrect(12,y+15,177,14,bgcolor);
			y += 17;
		} else if (remote[i].unit.result == CRC_ERROR) {
			ili9341_puts_p(PSTR("CRC error"));
			ili9341_clearTextArea(189);
			ili9341_fillrect(12,y+15,177,14,bgcolor);
			y += 17;
		} else {
			// show current temperature
			ili9341_setFont(lcdnums12x16);
			drawScaledLeft(18,y,60,remote[i].temp);
			drawSymbol(9);
			ili9341_write('C');
			ili9341_setCursor(99,y);
			// show minimum
			drawSymbol(25);
			drawScaled(remote_day.min_temp);
			ili9341_puts_p(PSTR("C "));
			if (remote[i].unit.type != DS18B20) {
				drawScaled(remote_day.min_humid);
				ili9341_write('%');
			}
			ili9341_clearTextArea(189);
			// show maximum
			ili9341_setCursor(99,y+15);
			drawSymbol(24);
			drawScaled(remote_day.max_temp);
			ili9341_puts_p(PSTR("C "));
			if (remote[i].unit.type != DS18B20) {
				drawScaled(remote_day.max_humid);
				ili9341_write('%');
			}
			ili9341_clearTextArea(189);
			y += 17;
			// show current humidity
			if (remote[i].unit.type != DS18B20) {
				ili9341_setFont(lcdnums12x16);
				drawScaledLeft(18,y,60,remote[i].humid);
				ili9341_setFont(Arial_bold_14);
				ili9341_write('%');
			}
		}
		y += 17;
		// show separator
		ili9341_drawhline(0,y,189,ILI9341_GRAY);
		y++;
		if (y >= 224) break;
	}
}

int main(void) {
	uint8_t length = 0xff, prev, c = 0;
	uint16_t crc = 0, signal_color = ILI9341_DARKGREY;
	
	timer0_init();
	timer2_init();
	ili9341_init();
	uart_init(UART_BAUD_SELECT(1200, F_CPU));
	if (ili9341_readcommand8(ILI9341_RDSELFDIAG) != 0xc0)
		uart_puts_P("diag fail\r\n");
	ili9341_setRotation(3);
	drawScreen();
	bme280_init();
	suart_init();
	init_adc();
	init_period();
	set_zone(1 * ONE_HOUR); // adjust time zone
	set_dst(eu_dst);        // enable daylight saving time
	
	// Main loop
	while (1) {
		if (action.update_screen) {
			OCR0B = ~read_adc(2); // adjust back light
			updateScreen();
			action.all = false;
			ili9341_drawXBitmap(288,114,signal_icon,20,16,signal_color, bgcolor);
			signal_color = ILI9341_DARKGREY;
			ili9341_setCursor(272,225);
			i_to_a(msec, buffer, 0, 0);
			ili9341_puts(buffer);
			ili9341_puts_p(PSTR("ms"));
			ili9341_clearTextArea(319);
		}
		if (suart_available()) {
			if (decode(suart_getc())) {
				set_system_time(mk_gmtime(&gps_time));
				set_position(gps_latitude / 100, gps_longitude / 100);
			}
		}
		if (uart_available()) {
			prev = c;
			c = uart_getc();
			if (prev == 0x55 && c == 0x55) {
				// Preamble received
				length = 0;
				crc = 0xffff;
				signal_color = ILI9341_ORANGE;
			} else if (length < sizeof(rxData)) {
				// Receive packet data
				if (length < sizeof(rxData) - sizeof(crc))
					crc = _crc16_update(crc, c);
				((uint8_t *)&rxData)[length++] = c;
			}
		}
		if (length == sizeof(rxData)) {
			// Validate received packet
			length = 0xff;
			if (rxData.crc == crc) {
				signal_color = ILI9341_LIME;
				remote[rxData.unit.id].status = ENABLED;
				remote[rxData.unit.id].timestamp = sec;
				remote[rxData.unit.id].unit = rxData.unit;
				remote[rxData.unit.id].temp = rxData.temp;
				remote[rxData.unit.id].humid = rxData.humid;
			} else {
				signal_color = ILI9341_RED;
			}
		}
	}
}

