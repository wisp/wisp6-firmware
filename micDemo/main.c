/**
 * @file       main.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Rohan Menon
 *
 */

#include "wisp-base.h"
#include "./mic/mic.h"

WISP_dataStructInterface_t wispData;

// Variables to store sending state (0x1980 in memory)
uint8_t* audio_capture_done = (uint8_t*)AUDIO_CAPTURE_DONE;
uint16_t* audio_address_toSend = (uint16_t*)AUDIO_ADDRESS_TOSEND;
uint8_t* audio_sequence_counter = (uint8_t*)AUDIO_SEQUENCE_COUNTER;
uint8_t* dstPtr, * srcPtr, * srcEnd;
uint8_t loop_counter = 0;
uint16_t vmeas = 0;

/**
 * This function is called by WISP FW after a successful ACK reply
 *
 */
void my_ackCallback(void)
{
    asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful read command
 *  reception
 *
 */
void my_readCallback(void)
{
    asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful write command
 *  reception
 *
 */
void my_writeCallback(void)
{
    *audio_address_toSend = ((10 * (*(wispData.writeBufPtr))) + AUDIO_START_ADDRESS);
    asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful BlockWrite
 *  command decode
 *
 */
void my_blockWriteCallback(void)
{
    asm(" NOP");
}

/**
 * This implements the user application and should never return
 *
 * Must call WISP_init() in the first line of main()
 * Must call WISP_doRFID() at some point to start interacting with a reader
 */

uint8_t vMeasIsEnough() {
    BITSET(PMEASSEL0, PIN_MEAS);
    BITSET(PMEASSEL1, PIN_MEAS);
    ADC_initCustom(ADC_reference_2_5V, ADC_precision_12bit, PMEASADC_INPUT);
    BITSET(PMEAS_ENOUT, PIN_MEAS_EN);
    __delay_cycles(1000);
    vmeas = ADC_critRead();
    BITCLR(PMEAS_ENOUT, PIN_MEAS_EN);

    if (vmeas >= 1800) {
        return TRUE;
    }
    return FALSE;
}

void main(void)
{
    WISP_init();
    FRAM_init();

    // FOR TESTING
    BITSET(P3DIR, BIT0); // Set the led pin to output

    // Register callback functions with WISP comm routines
    WISP_registerCallback_ACK(&my_ackCallback);
    WISP_registerCallback_READ(&my_readCallback);
    WISP_registerCallback_WRITE(&my_writeCallback);
    WISP_registerCallback_BLOCKWRITE(&my_blockWriteCallback);

    // Get access to EPC, READ, and WRITE data buffers
    WISP_getDataBuffers(&wispData);

    // Set up operating parameters for WISP comm routines
    WISP_setMode(MODE_STD);
    WISP_setAbortConditions(CMD_ID_ACK);

    // Set up EPC
    wispData.epcBuf[0] = 0xAD; // Tag type (Audio)

    *audio_capture_done = CAPTURE_REQ;
    *audio_address_toSend = AUDIO_START_ADDRESS;

    while (FOREVER)
    {
        //        if (!vMeasIsEnough())
        //         {
        //             // VBOOST is not high enough, so go to sleep
        //
        //             BITSET(TA1CCTL0, CCIE);
        //             TA1CCR0 = 2000;
        //             BITSET(TA1CTL, (TASSEL_1 | MC_1 | TACLR));
        //             __bis_SR_register(LPM4_bits | GIE);       // Enter LPM4 w/ interrupt
        //             BITCLR(TA1CCTL0, CCIE);                  // Disable the timer and its interrupt.
        //             BITCLR(TA1CTL, (TASSEL_1 | MC_1 | TACLR));
        //
        //             __bis_SR_register(LPM4_bits | GIE);       // Enter LPM4 w/ interrupt
        //         }

        // BITSET(PMEASSEL0, PIN_MEAS);
        // BITSET(PMEASSEL1, PIN_MEAS);
        // ADC_initCustom(ADC_reference_2_5V, ADC_precision_12bit, PMEASADC_INPUT);
        // BITSET(PMEAS_ENOUT, PIN_MEAS_EN);
        // __delay_cycles(1000);
        // vmeas = ADC_critRead();
        // BITCLR(PMEAS_ENOUT, PIN_MEAS_EN);

        // wispData.epcBuf[1] = 0xFE;
        // wispData.epcBuf[2] = (vmeas >> 8) & 0xFF;
        // wispData.epcBuf[3] = vmeas & 0xFF;

        // WISP_doRFID();



        // If an audio capture is required
        if (*audio_capture_done == CAPTURE_DONE)
        {
            // Audio already captured, send it
            while (*audio_address_toSend < AUDIO_END_ADDRESS)
            {
                // Copy audio data to send buffer
                srcPtr = (uint8_t*)(*audio_address_toSend);
                srcEnd = srcPtr + 10;
                dstPtr = (wispData.epcBuf + 2);
                do
                {
                    *(dstPtr++) = *(srcPtr++);
                } while (srcPtr < srcEnd);
                wispData.epcBuf[1] = *audio_sequence_counter;

                *audio_sequence_counter += 1;
                *audio_address_toSend += 10;

                // Send audio data
                WISP_doRFID();

                // Reset the sequence counter
                if (*audio_sequence_counter >= 200)
                    *audio_sequence_counter = 0;

//                __delay_cycles(10);
                 __delay_cycles(100);
            }

            __delay_cycles(100);

            // Now send tags indicating that the audio is done
            for (loop_counter = 0; loop_counter < SEQUENCE_END_REPEAT; loop_counter++)
            {
                wispData.epcBuf[1] = AUDIO_DONE_FLAG; // Audio done flag
                WISP_doRFID();
                __delay_cycles(100);
            }

            *audio_capture_done = CAPTURE_REQ;
        }
        else
        {
            // Capture audio if it's required

            // Setup the timer interrupt on TA1 and go to sleep for some time. If woke up from sleep means the
            // VBOOST has reached its higher voltage, so it is required to start image capturing.

            // if (!vMeasIsEnough())
            // {
            //     // VBOOST is not high enough, so go to sleep

            //     BITSET(TA1CCTL0, CCIE);
            //     TA1CCR0 = 2000;
            //     BITSET(TA1CTL, (TASSEL_1 | MC_1 | TACLR));
            //     __bis_SR_register(LPM4_bits | GIE);       // Enter LPM4 w/ interrupt
            //     BITCLR(TA1CCTL0, CCIE);                  // Disable the timer and its interrupt.
            //     BITCLR(TA1CTL, (TASSEL_1 | MC_1 | TACLR));

            //     __bis_SR_register(LPM4_bits | GIE);       // Enter LPM4 w/ interrupt
            // }
            MIC_initialize();

            MIC_capture();
            //            ledBlink();
            *audio_capture_done = CAPTURE_DONE;
            *audio_address_toSend = AUDIO_START_ADDRESS;
            *audio_sequence_counter = 0;
        }
    }
}


#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
{
    if (vMeasIsEnough())
    {
        BITCLR(TA1CTL, TAIFG);
        __bic_SR_register_on_exit(LPM4_bits | GIE);
        P3OUT ^= BIT0;
        __delay_cycles(100);
    }
    P3OUT ^= BIT0;
}
