/*
 * mic.c
 *
 *  Created on: Sep 14, 2022
 *      Author: Rohan
 */

 // https://users.wpi.edu/~ndemarinis/ece2049/e16/lecture10.html
#include "mic.h"
#include "wisp-base.h"
#include "adpcm.h"

uint8_t* dst = (uint8_t*)AUDIO_START_ADDRESS;
uint8_t start_chunk = TRUE;

void MIC_initialize()
{
    BITSET(PMIC_AMP_DIR, PIN_MIC_AMP_EN); // Set the mic amp enable pin to output

    BITSET(PMIC_AUDIO_SEL1, PIN_MIC_AUDIO); // Set the audio pin to adc input
    BITSET(PMIC_AUDIO_SEL0, PIN_MIC_AUDIO);

    // Set up voltage reference
    while (REFCTL0 & REFGENBUSY) {};                    // Wait for reference generator to be ready
    REFCTL0 = REFVSEL_2 | REFTCOFF | REFON;             // Set reference voltage to 2.5V

    // Set up ADC12
    ADC12CTL0 = ADC12SHT0_0 | ADC12ON;                 // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP | ADC12SHS_6 | ADC12CONSEQ_2; // Use sampling timer, SHS source = TA3 out, repeat-single-channel
    ADC12CTL2 = ADC12RES__12BIT;                       // 12-bit conversion results
    ADC12MCTL0 |= AUDIO_PIN | ADC12VRSEL_1;            // A15 ADC input select; Vref=2.5V
    ADC12IFGR0 &= ~ADC12IFG0;                          // Clear interrupt flag

    // Set up Timer A3
    TA3CCR0 = 6;                             // Set period to 6
    TA3CCR1 = 3;                             // Set duty cycle to 50%
    TA3CCTL1 = OUTMOD_3;                     // Reset/set mode
    TA3CTL = MC__STOP;                       // Gets set in when we capture
}

void MIC_capture()
{
    // Reset compressor constants
    ADPCM_Init();

    // Set up clock
    CSCTL0_H = CSKEY >> 8;                                // Unlock CS registers
    CSCTL1 = DCOFSEL_6;                                   // Set DCO to 8MHz
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; // Set ACLK = LFXT; MCLK = DCO
    CSCTL3 = DIVA__1 | DIVS__4 | DIVM__4;                 // Set SMCLK = MCLK/4 = 250kHz
    CSCTL0_H = 0;                                         // Lock CS registers

    BITSET(PMIC_AMP_OUT, PIN_MIC_AMP_EN); // Enable the mic power

    // Wait for the mic to power up (~170ms)
    // Should enter sleep mode for this delay
    Timer_LooseDelay(5400);

    // Enable timer
    TA3CTL = TASSEL__SMCLK | MC__UP | TACLR;

    // Enable ADC
    ADC12CTL0 |= ADC12ENC | ADC12SC;
    ADC12IER0 |= ADC12IE0;

    // Enter LPM3, enable interrupts
    __bis_SR_register(LPM3_bits | GIE);
    __no_operation();
}

void _end_capture()
{
    // Disable mic power
    BITCLR(PMIC_AMP_OUT, PIN_MIC_AMP_EN);

    // Disable ADC
    ADC12CTL0 &= ~(ADC12ENC | ADC12SC);
    ADC12IER0 &= ~ADC12IE0;

    // Disabe TA3
    TA3CTL = MC__STOP;

    // Reset capture variables
    dst = (uint8_t*)AUDIO_START_ADDRESS;
    start_chunk = TRUE;
}

// #pragma vector = TIMER3_A0_VECTOR
// __interrupt void TIMER3_A0_ISR(void)
// {
// }

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{

    if (dst >= (uint8_t*)(AUDIO_END_ADDRESS))
    {
        _end_capture();

        // Exit LMP3
        __bic_SR_register_on_exit(LPM3_bits | GIE);
    }

    // Encodes the 12-bit ADC value into just 4 using ADPCM compression
    // The encoder expects a 16-bit value, so we shift the 12-bit ADC result
    uint8_t encoded_4b = ADPCM_Encoder(ADC12MEM0);

    // Alternates between writing the 4-bit value to the upper and lower nibble
    if (start_chunk)
    {
        *dst = encoded_4b << 4;
        start_chunk = FALSE;
    }
    else
    {
        *dst |= encoded_4b;
        dst++;
        start_chunk = TRUE;
    }

//    P3OUT ^= BIT0;

}
