/*
 * Microphone.c
 *
 *  Created on: Feb 27, 2023
 *      Author: saffaria
 */


#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "Microphone.h"
#include "mic.h"

void init_mic()
{


    // Enable Mic board
    P3DIR |= BIT6;
    P3OUT |= BIT6;

    //Define ADC channel 15 (P3.3) pin
    P3SEL0 |= BIT3;
    P3SEL1 |= BIT3;
    __delay_cycles(8000);

    TBCCR0 = 125-1;                               // 8KHz sampling rate to keep an accurate sampling rate for the ADC
    TBCCR1 = 25;
    TBCCTL1 = OUTMOD_3;                           // CCR1 set/reset mode
    TBCTL |= TBSSEL_2 | MC_1 |TBCLR;              // SMCLK, Up-Mode

    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL2 &= 0xFFCF;                                    //8 bit resolution
    ADC12CTL0 |= ADC12SHT0_10 | ADC12MSC | ADC12ON;         // Sampling time, MSC, ADC12 on
    ADC12CTL1 |= ADC12SHS_3 | ADC12CONSEQ_2|ADC12SSEL_3;     // Use sampling timer; ADC12MEM0
                                                             // Sample-and-hold source = CCI0B =
//                                                           // TBCCR1 output
//                                                           // Repeated-single-channel
    ADC12MCTL0 |= ADC12VRSEL_0 | ADC12INCH_15;               // V+=AVcc V-=AVss, A5 channel
    ADC12CTL0 |= ADC12ENC;

                                                              // Setup DMA0
    DMACTL0 |= DMA0TSEL_26;                                                      // ADC12IFGx triggered
    __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC12MEM0);    //ADC12MEM0 buffer address as the source
    __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) AUDIO_START_ADD);// memory address for saving the image as the destinati
}

void record_audio()
{
    DMA0CTL &= ~DMAIFG;
    DMA0SZ = 32000;
    DMA0CTL |= DMADT_4|DMAEN|DMADSTINCR_3|DMAIE|DMADSTBYTE | DMASRCBYTE; // Rpt single tranfer, inc dst, Int
    __bis_SR_register(LPM0_bits + GIE);       // LPM0 w/ interrupts
}

#pragma vector=DMA_VECTOR
__interrupt void DMA(void)
{
                                              // data is received by the MCU

    P3OUT &= ~BIT6;                            // Turn off the mic
    DMA0CTL &= ~DMAIFG;
    DMA0CTL &= ~DMAEN;
    DMA0CTL &= ~DMAIE;
    __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0


}

