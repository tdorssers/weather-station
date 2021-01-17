/*
 * DS18B20 Driver
 *
 * Created: 26-12-2020 16:27:56
 *  Author: Tim Dorssers
 */ 


#ifndef DS18B20_H_
#define DS18B20_H_

#define DS18B20_PORT PORTB
#define DS18B20_DDR  DDRB
#define DS18B20_PIN  PINB
#define DS18B20_DQ   PB5

#define DS18B20_INPUT_MODE()  DS18B20_DDR  &= ~(1 << DS18B20_DQ)
#define DS18B20_OUTPUT_MODE() DS18B20_DDR  |=  (1 << DS18B20_DQ)
#define DS18B20_LOW()         DS18B20_PORT &= ~(1 << DS18B20_DQ)
#define DS18B20_HIGH()        DS18B20_PORT |=  (1 << DS18B20_DQ)
#define DS18B20_CMD_CONVERTTEMP   0x44
#define DS18B20_CMD_RSCRATCHPAD   0xbe
#define DS18B20_CMD_WSCRATCHPAD   0x4e
#define DS18B20_CMD_CPYSCRATCHPAD 0x48
#define DS18B20_CMD_RECEEPROM     0xb8
#define DS18B20_CMD_RPWRSUPPLY    0xb4
#define DS18B20_CMD_SEARCHROM     0xf0
#define DS18B20_CMD_READROM       0x33
#define DS18B20_CMD_MATCHROM      0x55
#define DS18B20_CMD_SKIPROM       0xcc
#define DS18B20_CMD_ALARMSEARCH   0xec
#define DS18B20_DECIMAL_STEPS_12BIT 625 //.0625
uint8_t ds18b20_read_temperature(int16_t *temp);

#endif /* DS18B20_H_ */