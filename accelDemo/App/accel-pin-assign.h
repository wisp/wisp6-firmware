/**
 * @file       pin-assign.h
 *
 * This file specifies pin assignments for the particular hardware platform used.
 *  currently this file targets the WISP5-LRG platform.
 *
 * @author     Aaron Parks, Justin Reina, Ali Saffari UW Sensor Systems Lab
 *
 * @todo       The pin definitions in this file are incomplete! Use script to autogenerate these.
 *
 */

#ifndef ACCEL_PIN_ASSIGN_H_
#define ACCEL_PIN_ASSIGN_H_
//#include "wispGuts.h"

/** @section    IO CONFIGURATION
 *  @brief      This represents the default IO configuration for the WISP 5.0 rev 0.1 hardware
 *  @details    Pay very close attention to your IO direction and connections if you are modifying any of this!
 *
 *  @note   PIN_TX Must be BIT7 of a port register, as the register is used as a mini-FIFO in the transmit operation. BIT0 may also be
 *          used with some modification of the transmit routine. Do NOT attempt to use other pins on PTXOUT as outputs.
 */
/************************************************************************************************************************************/

/*
 * Port 1
 */


// P1.4 - ACCEL_CS - OUTPUT
#define PIN_ACCEL_CS        BIT4
#define POUT_ACCEL_CS       P1OUT
#define PDIR_ACCEL_CS       P1DIR



// P1.5 - SPI SCK
#define     PIN_SPI_SCK             (BIT5)
#define     PDIR_SPI_SCK            (P1DIR)
#define     PSPI_SCKSEL0            (P1SEL0)
#define     PSPI_SCKSEL1            (P1SEL1)




/*
 * Port 2
 */

// P2.0 - SPI_MOSI
#define     PIN_SPI_MOSI          (BIT0)
#define     PSPI_MOSIIN           (P2IN)
#define     PSPI_MOSIOUT          (P2OUT)
#define     PDIR_SPI_MOSI         (P2DIR)
#define     PSPI_MOSISEL0         (P2SEL0)
#define     PSPI_MOSISEL1         (P2SEL1)

// P2.1 - SPI_MISO
#define     PIN_SPI_MISO          (BIT1)
#define     PSPI_MISOIN           (P2IN)
#define     PSPI_MISOOUT          (P2OUT)
#define     PDIR_SPI_MISO         (P2DIR)
#define     PSPI_MISOSEL0         (P2SEL0)
#define     PSPI_MISOSEL1         (P2SEL1)





/*
 * Port 3
 */

// P3.3 - ACCEL Power - OUTPUT
#define     PIN_AccelPWR          (BIT3)
#define     PAccelPWROUT          (P3OUT)
#define     PAccelPWRDIR          (P3DIR)
/*
 * Port 4
 */



/*
 * Port 5
 */

/*
 * Port 6
 */

/*
 * Port J
 */




#endif /* PIN_ASSIGN_H */
