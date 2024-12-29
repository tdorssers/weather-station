/*
 * NMEA 0183 Decoder
 *
 * Created: 30-1-2021 20:53:48
 *  Author: Tim Dorssers
 */ 


#ifndef GPS_H_
#define GPS_H_

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

extern int32_t gps_latitude, gps_longitude, gps_altitude;
extern int16_t gps_speed, gps_course, gps_hdop;
extern uint8_t gps_numsats;
extern struct tm gps_time;
extern bool gps_fix, gps_valid;

bool gps_decode(char c);

#endif /* GPS_H_ */