/*
 * HDC2010.h
 *
 *  Created on: Sep 13, 2022
 *      Author: saffaria
 */

#ifndef APP_HDC2010_H_
#define APP_HDC2010_H_
#include "wisp-base.h"


#define TEMP_ADDR 0x40
//Define Register Map
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define INTERRUPT_DRDY 0x04
#define TEMP_MAX 0x05
#define HUMID_MAX 0x06
#define INTERRUPT_CONFIG 0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST 0x09
#define TEMP_THR_L 0x0A
#define TEMP_THR_H 0x0B
#define HUMID_THR_L 0x0C
#define HUMID_THR_H 0x0D
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L 0xFC
#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF

//  Constants for setting measurement resolution
#define FOURTEEN_BIT 0
#define ELEVEN_BIT 1
#define NINE_BIT  2

//  Constants for setting sensor mode
#define TEMP_AND_HUMID 0
#define TEMP_ONLY      1
#define HUMID_ONLY     2
#define ACTIVE_LOW     0
#define ACTIVE_HIGH    1
#define LEVEL_MODE      0
#define COMPARATOR_MODE 1

//  Constants for setting sample rate
#define MANUAL          0
#define TWO_MINS        1
#define ONE_MINS        2
#define TEN_SECONDS     3
#define FIVE_SECONDS    4
#define ONE_HZ          5
#define TWO_HZ          6
#define FIVE_HZ         7


void initI2C(void);
void hdc2010_triggerMeasurement(void);          // Triggers a manual temperature/humidity reading
void hdc2010_read(uint8_t* data);



#endif /* APP_HDC2010_H_ */
