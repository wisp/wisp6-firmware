/*
 * i2c.c
 *
 *  Created on: Jun 26, 2014
 *      Author: saman
 */

//#include <msp430.h>
#include "i2c.h"

uint8_t TxBuff;



void i2c_init(){


    UCB0CTLW0 |= UCSWRST;                       // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC | UCSSEL__SMCLK | UCTR;    // I2C mode, Master mode, sync, SMCLK.
    UCB0BRW = 0x000A;                           //1MHz/10
    UCB0I2CSA = 0x0040;
    UCB0CTL1 &= ~UCSWRST;


    BITCLR(PI2C_SDASEL0 , PIN_I2C_SDA);
    BITCLR(PI2C_SCLSEL0 , PIN_I2C_SCL);
    BITSET(PI2C_SDASEL1 , PIN_I2C_SDA);
    BITSET(PI2C_SCLSEL1 , PIN_I2C_SCL);


}


void i2c_write(uint8_t addr, uint8_t wrData){

    uint16_t i=0;
    UCB0CTLW0 |= UCSWRST;
    for (i=0xFFFF; i>0; i--);
    UCB0CTL1 &= ~UCSWRST;
    //Load some data, then do start bit
    uint8_t TxBuff = (addr);
    while (UCB0CTLW0 & UCTXSTP);            // Ensure stop condition got sent
    UCB0CTLW0 |= UCTR | UCTXSTT;            // I2C TX, start condition
    while( !(UCB0IFG & UCTXIFG0)) ;


    UCB0TXBUF = TxBuff;
    while( !(UCB0IFG & UCTXIFG0));


    //Do the next byte
    TxBuff = wrData;
    UCB0TXBUF = TxBuff;
    while( !(UCB0IFG & UCTXIFG0) ) ;
    UCB0CTLW0 |= UCTXSTP;


    return;
}


uint8_t i2c_read(uint8_t addr){

    uint16_t i=0;
    UCB0CTLW0 |= UCSWRST;
    for (i=0xFFFF; i>0; i--);
    UCB0CTL1 &= ~UCSWRST;
    uint8_t TxBuff = (addr);




    UCB0CTLW0 |= UCTR | UCTXSTT;            // I2C TX, start condition
    while( !(UCB0IFG & UCTXIFG0) ) ;



    UCB0TXBUF = TxBuff;
    while( !(UCB0IFG & UCTXIFG0) );

    UCB0CTLW0 &= ~UCTR;



    uint8_t readData = 0;
    UCB0CTL1 |= UCTXSTT;
    while( (UCB0CTL1 & UCTXSTT) ) {

    }

    UCB0CTL1 |= UCTXSTP;                                                    /* set the I2C stop condition                           */
    while (UCB0CTL1 & UCTXSTP) {                                            /* stop bit is cleared right after stop is generated    */

    }

    while( !(UCB0IFG & UCRXIFG) ) {

    }

    readData = UCB0RXBUF;
    return readData;

}


