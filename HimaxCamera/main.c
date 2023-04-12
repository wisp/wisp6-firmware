/**
 * @file       usr.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Aaron Parks, UW Sensor Systems Lab
 *
 */

#include "wisp-base.h"
#include "Camera/HM.h"
#include "Camera/HMRegs.h"
#include "cam.h"
WISP_dataStructInterface_t wispData;

// The variable to save if the image is captured fully.
uint8_t *image_capture_done = (uint8_t *)IMAGE_CAPTURE_DONE;
uint16_t *image_address_toSend = (uint16_t *)IMAGE_ADDRESS_TOSEND;
uint16_t *image_length_sent = (uint16_t *)IMAGE_LENGTH_SENT;
uint8_t *image_sequence_counter = (uint8_t *)IMAGE_SEQUENCE_COUNTER;
uint8_t *dstPtr, *srcPtr, *srcEnd;
uint8_t loop_counter = 0;
uint8_t stat = 0;

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
	*image_address_toSend = ((10*(*(wispData.writeBufPtr))) + IMAGE_START_ADD);
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



   P3DIR |= BIT3;  // Camera EN PIN
   P3OUT &= ~BIT3; // Put Camera in sleep mode

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
   wispData.epcBuf[0] = CAMERA_TAG_ID;        // Tag type
   wispData.epcBuf[1] = 0xFE;        // Unused data field
   wispData.epcBuf[2] = 0;           // Unused data field
   wispData.epcBuf[3] = 0;           // Unused data field



//   WISP_doRFID();                             // commented on 1/18/23
   BITCLR(PRXIFG , PIN_RX);                    // clear interrupt flag for RX pin
   BITCLR(PRXIE , PIN_RX);


                                           // Check if the image is captured and stored previousely
  if(*image_capture_done != CAPTURE_DONE)
  {


	   P3OUT |= BIT3;                       //Enable Camera
	   P3DIR &= ~BIT6;                      // Enable interrupt high-low edge for FVLD camera pin
	   P3REN &= ~BIT6;
	   P3IES |= BIT6;

	   initI2C();                           // I2C for camera initialization
	   __delay_cycles(10000);

	   uint8_t i = 5;                       // initialization the camera 5 times to make sure it is operating. Not sure why sometimes we need more than 1.
	  for (i=5; i>0; i--)
	  {
	      stat = init_Himax();               // Camera in QQVGA mode
	      __delay_cycles(10000);
	  }

	   while(stat == 0)
	   {
	       stat = init_Himax();                // make sure I2C is working

	       if(stat == 1)
	           break;

	                                           //restart the camera if I2C is not working.
	       P3OUT &= ~BIT3;
	       __delay_cycles(1000);
	       P3OUT |= BIT3;
	       __delay_cycles(10000);

	   }

	   init_SPI();                             // read camera data through SPI and DMA.


	   *image_capture_done = CAPTURE_DONE;
	   *image_address_toSend = IMAGE_START_ADD;
	   *image_length_sent = 0;
	   *image_sequence_counter = 0;
  }

  
  // Set up static EPC
 	wispData.epcBuf[0] = CAMERA_TAG_ID; // WISP version for camera. In this case GUI will recognize that this is a camera tag.


 	while (*image_length_sent < IMAGE_DATA_TOBESENT)
 	{

     	srcPtr = (uint8_t*)(*image_address_toSend);
     	srcEnd = srcPtr + 10;
     	dstPtr = (wispData.epcBuf + 2);

     	do
     	{
     		*(dstPtr++) = *(srcPtr++);
     	} while (srcPtr < srcEnd);

     	wispData.epcBuf[0] = CAMERA_TAG_ID;
     	wispData.epcBuf[1] = *image_sequence_counter;

     	*image_address_toSend = *image_address_toSend + 10;
     	*image_length_sent = *image_length_sent + 10;
     	*image_sequence_counter = *image_sequence_counter + 1;

     	WISP_doRFID();
     	if(*image_sequence_counter >= 200)
     		*image_sequence_counter = 0;
     	__delay_cycles(10);
    }

     loop_counter = 0;
     do
     {
         wispData.epcBuf[0] = CAMERA_TAG_ID;
         wispData.epcBuf[1] = 0xFF;
     	WISP_doRFID();
     	__delay_cycles(100);
     	loop_counter++;
     } while(loop_counter < SEQUENCE_END_REPEAT);
     *image_capture_done = CAPTURE_REQ;

     wispData.epcBuf[1] = 0xFD;
     while(1)
    	 WISP_doRFID();




}



