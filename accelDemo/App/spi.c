/**
 * sensor_spi.c
 *
 * @author Aaron Parks
 * @date Aug 2013
 *
 * @updated by Rohan Menon
 * @date Sep 2022
 *
 */


#include <msp430.h>
#include "globals.h"
#include "spi.h"
#include "accel-pin-assign.h"

uint8_t gpRxBuf[SPI_GP_RXBUF_SIZE];

/**
 * Description of state of the SPI module.
 */
static struct {
    BOOL bPortInUse;
    BOOL bNewDataReceived;
    unsigned int uiCurRx;
    unsigned int uiCurTx;
    unsigned int uiBytesToSend;
    uint8_t *pcRxBuffer;
    uint8_t *pcTxBuffer;
} spiSM;


/**
 *
 * @return success or failure
 *
 * @todo Implement this function
 */
BOOL SPI_initialize() {


    // Hardware peripheral initialization
    BITSET(UCA0CTL1, UCSWRST);

//  UCA1CTL0 = UCMST | UCSYNC | UCCKPH | UCMSB;      // This instruction seem to work wrong since UA1CTL0 is an 8-bit register.
    UCA0CTL0 = (UCMST>>8) | (UCSYNC>>8) | (UCCKPH>>8) | (UCMSB>>8);
    UCA0CTL1 = UCSSEL_3 | UCSWRST;
    UCA0BR0 = 0; // 1MHz for 1MHz clock
    UCA0BR1 = 0;
    UCA0IFG = 0;
    UCA0MCTLW = 0;  // No modulation, I don't think it is vital to write this command since the default should be like that.

    BITSET(PSPI_SCKSEL1 , PIN_SPI_SCK);
    BITCLR(PSPI_SCKSEL0 , PIN_SPI_SCK);

    BITSET(PSPI_MOSISEL1 , PIN_SPI_MISO | PIN_SPI_MOSI);
    BITCLR(PSPI_MOSISEL0 , PIN_SPI_MISO | PIN_SPI_MOSI);

    BITCLR(UCA0CTL1, UCSWRST);

    // State variable initialization
    spiSM.bPortInUse = FALSE;
    spiSM.bNewDataReceived = FALSE;
    spiSM.uiCurRx = 0;
    spiSM.uiCurTx = 0;
    spiSM.uiBytesToSend = 0;


    return SUCCESS;
}

/**
 *
 * @return Success - you were able to get the port. Fail - you don't have the port, so don't use it.
 */
BOOL SPI_acquirePort() {

    if(spiSM.bPortInUse) {
        return FAIL;
    } else {
        spiSM.bPortInUse=TRUE;
        return SUCCESS;
    }

}

/**
 *
 * @return success or fail
 * @todo Make this more robust (don't allow release of port if we don't have it)
 */
BOOL SPI_releasePort() {
    if(spiSM.bPortInUse) {
        spiSM.bPortInUse = FALSE;
        return SUCCESS;
    }
    return FAIL;
}


/**
 * Engage in a synchronous serial transaction of the specified length.
 * This function blocks until transaction is complete.
 *
 * @param txBuf
 * @param size
 * @return success or fail
 */
BOOL SPI_transaction(uint8_t* rxBuf, uint8_t* txBuf, uint16_t size) {

    if(!spiSM.bPortInUse)
        return FAIL; // If the port is not acquired, fail!

    if(size==0)
        return FAIL; // If we aren't sending anything, fail!

    spiSM.bNewDataReceived = FALSE;
    spiSM.uiCurRx = 0;
    spiSM.uiCurTx = 0;
    spiSM.uiBytesToSend = size;

    spiSM.pcRxBuffer = rxBuf;
    spiSM.pcTxBuffer = txBuf;

    //BITSET(UCA1IE, UCTXIE | UCRXIE);

    do {
        // Reset receive flag
        spiSM.bNewDataReceived = FALSE;

        // Start transmission
        UCA0TXBUF = spiSM.pcTxBuffer[spiSM.uiCurTx];

        // Sleep until receive occurs
        while(!(UCA0IFG & UCRXIFG));
        spiSM.pcRxBuffer[spiSM.uiCurRx] = UCA0RXBUF;
        UCA0IFG &= ~UCRXIFG;
        UCA0IFG &= ~UCTXIFG;
        // Move to next TX and RX index
        spiSM.uiCurTx++;
        spiSM.uiCurRx++;
        spiSM.uiBytesToSend--;

    } while(spiSM.uiBytesToSend);


    return SUCCESS;
}


/// Be careful that FR5969 has 16 bits UCA1RXBUF and UCA1TXBUF other than F5310 that had 8 bits.
//#pragma vector=USCI_A1_VECTOR
//__interrupt void SPI_ISR(void) {
//
//    if(UCRXIFG){
//        spiSM.pcRxBuffer[spiSM.uiCurRx] = UCA1RXBUF;
//        spiSM.bNewDataReceived = TRUE;
//        UCA1IFG &= ~UCRXIFG;    // Not sure it is vital to write this command or not but just as being punctilious
//        __bic_SR_register_on_exit(LPM4_bits);
//    }
//
//    if(UCTXIFG){
//        UCA1IFG &= ~UCTXIFG;
//    }



    /// another way to handle the interrupt.
    //  if((UCA1IFG & UCRXIFG) == 1){
    //      spiSM.pcRxBuffer[spiSM.uiCurRx] = UCA1RXBUF;
    //      spiSM.bNewDataReceived = TRUE;
    //      UCA1IFG &= ~UCRXIFG;
    //      __bic_SR_register_on_exit(LPM4_bits);
    //
    //  }
    //
    //  else if((UCA1IFG & UCTXIFG) == 2){
    //      UCA1IFG &= ~UCTXIFG;
    //      __bic_SR_register_on_exit(LPM4_bits);
    //
    //  }

//}
