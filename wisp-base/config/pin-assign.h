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

#ifndef PIN_ASSIGN_H_
#define PIN_ASSIGN_H_
#include "wispGuts.h"

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

// P1.0 - RX_BITLINE INPUT
#define PIN_RX_BITLINE		(BIT0)
#define PRX_BITLINEOUT 		(P1OUT)

//P1.1 LED
#define   PDIR_LED           (P1DIR)
#define   PLEDOUT            (P1OUT)
#define   PIN_LED            (BIT1)



// P1.6 - I2C_SDA -  INPUT/OUTPUT
#define		PIN_I2C_SDA				(BIT6)
#define 	PI2C_SDAIN				(P1IN)
#define 	PDIR_I2C_SDA			(P1DIR)
#define		PI2C_SDASEL0			(P1SEL0)
#define		PI2C_SDASEL1			(P1SEL1)

// P1.7 - I2C_SCL -  INPUT/OUTPUT
#define		PIN_I2C_SCL				(BIT7)
#define 	PDIR_I2C_SCL			(P1DIR)
#define		PI2C_SCLSEL0			(P1SEL0)
#define		PI2C_SCLSEL1			(P1SEL1)


/*
 * Port 2
 */




// P2.3 - RECEIVE - INPUT
#define		PIN_RX			(BIT3)
#define 	PRXIN			(P2IN)
#define 	PDIR_RX			(P2DIR)
#define		PRXIES			(P2IES)
#define		PRXIE			(P2IE)
#define		PRXIFG			(P2IFG)
#define		PRXSEL0			(P2SEL0)
#define		PRXSEL1			(P2SEL1)
#define 	PRX_VECTOR_DEF	(PORT2_VECTOR)

// P2.5 - UART TX - OUTPUT
#define     PIN_UART_TX             (BIT5)
#define     PUART_TXSEL0            (P2SEL0)
#define     PUART_TXSEL1            (P2SEL1)

// P2.6 - UART RX - INPUT
#define     PIN_UART_RX             (BIT6)
#define     PUART_RXSEL0            (P2SEL0)
#define     PUART_RXSEL1            (P2SEL1)


// P2.7 - TRANSMIT - OUTPUT
#define		PIN_TX			(BIT7)
#define 	PTXOUT			(P2OUT)
#define		PTXDIR			(P2DIR)

/*
 * Port 3
 */


/*
 * Port 4
 */

// P4.0 - Sensor INT
#define     PIN_SEN_INT          (BIT0)
#define     PSEN_INTIN           (P4IN)
#define     PDIR_SEN_INT         (P4DIR)
#define     PSEN_INTIES          (P4IES)
#define     PSEN_INTIE           (P4IE)
#define     PSEN_INTIFG          (P4IFG)
#define     PSEN_INTSEL0         (P4SEL0)
#define     PSEN_INTSEL1         (P4SEL1)
#define     PSEN_INT_VECTOR_DEF  (PORT4_VECTOR)


// P4.1 MEAS INPUT
#define 	PIN_MEAS			(BIT1)
#define		PMEASOUT			(P4OUT)
#define		PMEASDIR			(P4DIR)
#define		PMEASSEL0			(P4SEL0)
#define		PMEASSEL1			(P4SEL1)


// P4.3 - TXEN - OUTPUT
#define     PIN_TXEN          (BIT3)
#define     PTXENOUT          (P4OUT)
#define     PTXENDIR          (P4DIR)


// P4.5 - RECEIVE ENABLE - OUTPUT
#define     PIN_RX_EN       (BIT5)
#define     PRXENOUT        (P4OUT)
#define     PDIR_RX_EN      (P4DIR)


// P4.6 - DEBUG LINE - OUTPUT
#define     PIN_DBG0        (BIT6)
#define     PDBGOUT         (P4OUT)

/*
 * Port 5
 */

/*
 * Port 6
 */

/*
 * Port J
 */


// PJ.2 - MEASURE ENABLE - OUTPUT
#define     PIN_MEAS_EN         (BIT2)
#define     PMEAS_ENOUT         (PJOUT)
#define     PDIR_MEAS_EN        (PJDIR)


// PJ.3 - SENSOR PWR - OUTPUT
#define     PIN_SENSORPWR         (BIT3)
#define     PSENSORPWROUT         (PJOUT)
#define     PDIRSENSORPWR         (PJDIR)



/*
 * ADC Channel definitions
 */

/**
 * Default IO setup
 */
/** @todo: Default for unused pins should be output, not tristate.  */
/** @todo:  Make sure the Tx port pin should be tristate not output and unused pin to be output*/
// Set as many as possible pins to output and drive them low
#ifndef __ASSEMBLER__
#define setupDflt_IO() \
    P2OUT = 0x00;\
    P3OUT = 0x00;\
    P4OUT = 0x00;\
    PJOUT = 0x00;\
    P1DIR = ~PIN_RX_BITLINE;\
    PJDIR = 0xFF;\
    P2DIR = ~PIN_RX;\
    P3DIR = 0xFF;\
    P4DIR = 0xFF;\

#endif /* ~__ASSEMBLER__ */

#endif /* PIN_ASSIGN_H */
