# weather-station

## Base

A barometric sensor is used to implement a simple weather forecasting algorithm by calculating Pa/h. Up to 16 remote AM2320 or DS18B20 sensors can transmit NRZ encoded packets with CRC data using 433MHz ASK/OOK modules connected to the hardware USART at 1200 baud. Minimum and maximum temperatures and humidities are stored over a 24 hour period.

It uses a speed optimized ILI9341 driver for AVR which implements graphics drawing primitives and uses openGLCD library fonts.

### Hardware

* ATmega328P @ 8 MHz
* RFM210LCF 433MHz ASK/OOK receiver module
* BME280 digital humidity, pressure and temperature sensor
* ILI9341 driver for 240x320 TFT LCD module

### Prototype

![](media/base_station.jpg)

## Remote ATtiny25/45/85

Remote temperature sensor module with drivers for AM2320 and DS18B20 sensor. The tactile button is used to set the unit address. Every 8 seconds a NRZ encoded packet with CRC data is transmitted using a 433MHz ASK/OOK module connected to the USI UART at 1200 baud.

### Hardware

* ATtiny25/45/85 @ 1 MHz
* RFM85 433MHz ASK/OOK transmitter module
* AM2320 digital temperature and humidity sensor
* DS18B20 digital temperature sensor
* 1 tactile button

### Schematic

AM2320
![](schematic/remote_25_45_85.png)

DS18B20
![](schematic/remote_25_45_85-1wire.png)

### Prototype

![](media/remote_25_45_85.jpg)

## Remote ATtiny2313/2313A/4313

Remote temperature sensor module with drivers for AM2320 and DS18B20 sensor. The DIP switches are used to set the unit address. Every 8 seconds a NRZ encoded packet with CRC data is transmitted using a 433MHz ASK/OOK module connected to the hardware USART at 1200 baud.

### Hardware

* ATtiny2313/2313A/4313 @ 1 MHz
* RFM85 433MHz ASK/OOK transmitter module
* AM2320 digital temperature and humidity sensor
* DS18B20 digital temperature sensor
* 4 DIP switches

### Schematic

AM2320
![](schematic/remote_2313_4313.png)

DS18B20
![](schematic/remote_2313_4313-1wire.png)
 
### Prototype

![](media/remote_2313_4313.jpg)

## Firmware
The firmware has been developed in Atmel Studio 7 using GCC C and can be uploaded to the ATmega328P using the ISP connector and an ISP programmer such as [USBasp tool](http://www.fischl.de/usbasp/) using [avrdude](http://www.nongnu.org/avrdude/).
