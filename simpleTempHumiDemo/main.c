/**
 * @file       usr.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Aaron Parks, UW Sensor Systems Lab
 *
 */

#include "wisp-base.h"
#include "App/i2c.h"
#include "App/HDC2010.h"
WISP_dataStructInterface_t wispData;
uint8_t temp_lsb =0;
uint8_t temp_msb =0;
uint8_t humidity_lsb =0;
uint8_t humidity_msb =0;

/**
 * This function is called by WISP FW after a successful ACK reply
 *
 */
void my_ackCallback (void) {
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


  //hdc2010 power up sequence


  BITSET(PDIRSENSORPWR , PIN_SENSORPWR);
  BITCLR(PSENSORPWROUT , PIN_SENSORPWR);
  P1OUT &= ~BIT1;
  CSCTL0_H = 0xA5;
  CSCTL1 = DCOFSEL_0; //1MHz
  CSCTL2 = SELA__VLOCLK + SELS_3 + SELM_3;
  CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;
  BITCLR(CSCTL6 , (MODCLKREQEN|SMCLKREQEN|MCLKREQEN));
  BITSET(CSCTL6 , ACLKREQEN);

  __delay_cycles(100);

  BITSET(PSENSORPWROUT , PIN_SENSORPWR);
  __delay_cycles(2000);
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

  i2c_init();
  hdc2010_triggerMeasurement();
  temp_lsb = i2c_read (TEMP_LOW);
  temp_msb = i2c_read(TEMP_HIGH);
  humidity_lsb = i2c_read (HUMID_LOW);
  humidity_msb = i2c_read(HUMID_HIGH);




  // Set up EPC
  wispData.epcBuf[0] = 0x0D;        // Tag type
  wispData.epcBuf[1] = temp_lsb;           // temp_LSB
  wispData.epcBuf[2] = temp_msb;           // temp_MSB
  wispData.epcBuf[3] = humidity_lsb;       // humidity_LSB
  wispData.epcBuf[4] = humidity_msb;       // humidity_MSB
  wispData.epcBuf[5] = 0;           // Unused data field
  wispData.epcBuf[6] = 0;           // Unused data field
  wispData.epcBuf[7] = 0x00;        // Unused data field
  wispData.epcBuf[8] = 0x00;        // Unused data field
  wispData.epcBuf[9] = 0x60;        // Tag hardware revision (6.0)
  wispData.epcBuf[10] = *((uint8_t*)INFO_WISP_TAGID+1); // WISP ID MSB: Pull from INFO seg
  wispData.epcBuf[11] = *((uint8_t*)INFO_WISP_TAGID); // WISP ID LSB: Pull from INFO seg

  // Talk to the RFID reader.
  while (FOREVER) {
    WISP_doRFID();

    CSCTL0_H = 0xA5;
    CSCTL1 = DCOFSEL_0; //1MHz
    CSCTL2 = SELA__VLOCLK + SELS_3 + SELM_3;
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;
    BITCLR(CSCTL6 , (MODCLKREQEN|SMCLKREQEN|MCLKREQEN));
    BITSET(CSCTL6 , ACLKREQEN);

    hdc2010_triggerMeasurement();

    temp_lsb = i2c_read (TEMP_LOW);
    temp_msb = i2c_read(TEMP_HIGH);
    humidity_lsb = i2c_read (HUMID_LOW);
    humidity_msb = i2c_read(HUMID_HIGH);


    wispData.epcBuf[1] = temp_lsb;               // temp_LSB
    wispData.epcBuf[2] = temp_msb;               // temp_MSB
    wispData.epcBuf[3] = humidity_lsb;           // humidity_LSB
    wispData.epcBuf[4] = humidity_msb;           // humidity_MSB

  }
}
