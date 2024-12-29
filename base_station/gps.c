/*
 * NMEA 0183 Decoder
 *
 * Created: 30-1-2021 20:53:03
 *  Author: Tim Dorssers
 */ 

#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "gps.h"

uint8_t checksum = 0, message = 0, field = 0, offset = 0;
char buffer[15];

int32_t gps_latitude, gps_longitude, gps_altitude;
int16_t gps_speed, gps_course, gps_hdop;
uint8_t gps_numsats;
struct tm gps_time;
bool gps_fix, gps_valid;

// convert one hex digit to integer
static uint8_t parse_hex(char c) {
	c |= 0x20; // make it lowercase
	return (c >= 'a') ? c - 'a' + 0xA : c - '0';
}

// convert string of two digits to integer
static uint8_t parse_two(char *s) {
	return ((s[0] - '0') * 10 + (s[1] - '0'));
}

// convert to integer scaled by 1/100
static int32_t parse_decimal() {
	char *end = buffer;
	int32_t num = atol(buffer) * 100;
	if (num < 0) ++end;
	while (isdigit(*end)) ++end;
	uint8_t frac = 0;
	if (*end == '.') {
		uint8_t mult = 10;
		while (isdigit(*++end)) {
			frac += mult * (*end - '0');
			mult /= 10;
		}
	}
	return (num < 0) ? num - frac : num + frac;
}

// convert to seconds scaled by 1/100 or decimal degrees scaled by 1/100000
static uint32_t parse_degrees() {
	char *end = buffer;
	int32_t deg_min = atol(buffer);
	while (isdigit(*end)) ++end;
	uint16_t frac = 0;
	if (*end == '.') {
		uint16_t mult = 1000;
		while (isdigit(*++end)) {
			frac += mult * (*end - '0');
			mult /= 10;
		}
	}
#ifdef DECIMAL_DEGREES
	return (deg_min / 100) * 100000 + ((deg_min % 100) * 10000 + frac + 3) / 6;
#else
	return (deg_min / 100) * 360000 + ((deg_min % 100) * 6000) + (frac * 6) / 10;
#endif
}

#define GGA 0x20 // Global positioning system fix data
#define GLL 0x40 // Latitude and longitude, with time of position fix and status
#define RMC 0x60 // Recommended minimum data

// Returns true if new sentence has just passed checksum test
static bool parse_term() {
	if (buffer[0] == 0) return false;
	// the first term determines the sentence type
	if (field == 0) {
		if (strcmp_P(buffer, PSTR("GPGGA")) == 0) message = GGA;
		//if (strcmp_P(buffer, PSTR("GPGLL")) == 0) message = GLL;
		if (strcmp_P(buffer, PSTR("GPRMC")) == 0) message = RMC;
	}
	// checksum term
	if (message == 0xFF) {
		gps_valid = true;
		if ((parse_hex(buffer[0]) << 4) + parse_hex(buffer[1]) != checksum) gps_valid = false;
		return gps_valid;
	}
	if (message == 0) return false;
	// parse sentence term
	switch (message + field) {
		//case GLL + 5:
		case RMC + 1:
		//case GGA + 1:
			gps_time.tm_hour = parse_two(buffer);
			gps_time.tm_min = parse_two(&buffer[2]);
			gps_time.tm_sec = parse_two(&buffer[4]);
			break;
		//case GLL + 6:
		case RMC + 2:
			gps_fix = buffer[0] == 'A';
			break;
		case GGA + 6:
			gps_fix = buffer[0] > '0';
			break;
		//case GLL + 1:
		//case RMC + 3:
		case GGA + 2:
			gps_latitude = parse_degrees();
			break;
		//case GLL + 2:
		//case RMC + 4:
		case GGA + 3:
			if (buffer[0] == 'S') gps_latitude = -gps_latitude;
			break;
		//case GLL + 3:
		//case RMC + 5:
		case GGA + 4:
			gps_longitude = parse_degrees();
			break;
		//case GLL + 4:
		//case RMC + 6:
		case GGA + 5:
			if (buffer[0] == 'W') gps_longitude = -gps_longitude;
			break;
		case RMC + 7:
			gps_speed = parse_decimal();
			break;
		case GGA + 7:
			gps_numsats = atol(buffer);
			break;
		case RMC + 8:
			gps_course = parse_decimal();
			break;
		case GGA + 8:
			gps_hdop = parse_decimal();
			break;
		case RMC + 9:
			gps_time.tm_mday = parse_two(buffer);
			gps_time.tm_mon = parse_two(&buffer[2]) - 1;
			gps_time.tm_year = parse_two(&buffer[4]) + 100;
			break;
		case GGA + 9:
			gps_altitude = parse_decimal();
			break;
	}
	return false;
}

// Returns true when sentence is complete and valid
bool gps_decode(char c) {
	bool completed = false;
	switch (c) {
		case '$':  // sentence begin
			checksum = message = field = offset = 0;
			break;
		case ',':  // term terminators
			checksum ^= c;
		case '\r':
		case '\n':
		case '*':
			buffer[offset] = 0;
			completed = parse_term();
			if (c == '*') message = 0xFF;
			offset = 0;
			field++;
			break;
		default:  // ordinary characters
			if (offset < sizeof(buffer) - 1) buffer[offset++] = c;
			if (message != 0xFF) checksum ^= c;
	}
	return completed;
}
