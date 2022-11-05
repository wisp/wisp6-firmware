/*
 * HDC2010.c
 *
 *  Created on: Sep 13, 2022
 *      Author: saffaria
 */



#include "HDC2010.h"





void hdc2010_triggerMeasurement(void)
{

    uint8_t stat = 0;
    i2c_write(MEASUREMENT_CONFIG, 0x01);
    DELAY_CYCLES(100);
    stat = i2c_read(INTERRUPT_DRDY);

    while(!(stat & 0x80))
    {
        stat = i2c_read(INTERRUPT_DRDY);
        DELAY_CYCLES(10);
    }


}







