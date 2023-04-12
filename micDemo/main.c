/**
 * @file       usr.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Aaron Parks, UW Sensor Systems Lab
 *
 */

#include "wisp-base.h"
#include "Microphone/mic.h"
#include "Microphone/Microphone.h"

WISP_dataStructInterface_t wispData;

// The variable to save if the image is captured fully.
uint8_t *audio_capture_done = (uint8_t *)AUDIO_CAPTURE_DONE;
uint16_t *audio_address_toSend = (uint16_t *)AUDIO_ADDRESS_TOSEND;
uint16_t *audio_length_sent = (uint16_t *)AUDIO_LENGTH_SENT;
uint8_t *audio_sequence_counter = (uint8_t *)AUDIO_SEQUENCE_COUNTER;
uint8_t *dstPtr, *srcPtr, *srcEnd;
uint8_t loop_counter = 0;


/** 
 * This function is called by WISP FW after a successful ACK reply
 *
 */
void my_ackCallback (void) {
//	BITSET(PDIR_LED1 , PIN_LED1);
//	BITTOG(PLED1OUT , PIN_LED1);
    asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful read command
 *  reception
 *
 */
void my_readCallback (void) {
  asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful write command
 *  reception
 *
 */
void my_writeCallback (void) {
	*audio_address_toSend = ((10*(*(wispData.writeBufPtr))) + AUDIO_START_ADD);
  asm(" NOP");
}

/** 
 * This function is called by WISP FW after a successful BlockWrite
 *  command decode

 */
void my_blockWriteCallback  (void) {
  asm(" NOP");
}


/**
 * This implements the user application and should never return
 *
 * Must call WISP_init() in the first line of main()
 * Must call WISP_doRFID() at some point to start interacting with a reader
 */
void main(void) {

   WISP_init();




   P4DIR |= BIT3;
   P4OUT |= BIT3;  //RF Switch enabled



   // Register callback functions with WISP comm routines
   WISP_registerCallback_ACK(&my_ackCallback);
   WISP_registerCallback_READ(&my_readCallback);
   WISP_registerCallback_WRITE(&my_writeCallback);
   WISP_registerCallback_BLOCKWRITE(&my_blockWriteCallback);

   // Get access to EPC, READ, and WRITE data buffers
   WISP_getDataBuffers(&wispData);

   // Set up operating parameters for WISP comm routines
   WISP_setMode( MODE_READ | MODE_WRITE | MODE_USES_SEL);
   WISP_setAbortConditions(CMD_ID_READ | CMD_ID_WRITE | CMD_ID_ACK);



   // Set up EPC
   wispData.epcBuf[0] = AUDIO_TAG_ID;        // Tag type
   wispData.epcBuf[1] = 0xFE;        // Unused data field
   wispData.epcBuf[2] = 0;           // Unused data field
   wispData.epcBuf[3] = 0;           // Unused data field


//   WISP_doRFID();

   BITCLR(PRXIFG, PIN_RX);                    // clear interrupt flag for RX pin
   BITCLR(PRXIE, PIN_RX);


//                                            Check if the audio is captured and stored previousely
  if(*audio_capture_done != CAPTURE_DONE)
  {

      init_mic();
      record_audio();

	  *audio_capture_done = CAPTURE_DONE;
	  *audio_address_toSend = AUDIO_START_ADD;
	  *audio_length_sent = 0;
	  *audio_sequence_counter = 0;
  }


  // Set up static EPC
 	wispData.epcBuf[0] = AUDIO_TAG_ID; // WISP version for audio. In this case GUI will recognize that this is an audio tag.


 	while (*audio_length_sent < AUDIO_DATA_TOBESENT)
 	{

     	srcPtr = (uint8_t*)(*audio_address_toSend);
     	srcEnd = srcPtr + 10;
     	dstPtr = (wispData.epcBuf + 2);
//
     	do
     	{
     		*(dstPtr++) = *(srcPtr++);
     	} while (srcPtr < srcEnd);

     	wispData.epcBuf[0] = AUDIO_TAG_ID;
     	wispData.epcBuf[1] = *audio_sequence_counter;

     	*audio_address_toSend = *audio_address_toSend + 10;
     	*audio_length_sent = *audio_length_sent + 10;
     	*audio_sequence_counter = *audio_sequence_counter + 1;

     	WISP_doRFID();
     	if(*audio_sequence_counter >= 200)
     		*audio_sequence_counter = 0;
     	__delay_cycles(10);
    }

     loop_counter = 0;
     do
     {
         wispData.epcBuf[0] = AUDIO_TAG_ID;
         wispData.epcBuf[1] = 0xFF;
     	WISP_doRFID();
     	__delay_cycles(100);
     	loop_counter++;
     } while(loop_counter < SEQUENCE_END_REPEAT);
     *audio_capture_done = CAPTURE_REQ;

     wispData.epcBuf[1] = 0xFD;
     while(1)
    	 WISP_doRFID();




}
