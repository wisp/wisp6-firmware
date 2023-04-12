/*
 * HDC2010.c
 *
 *  Created on: Sep 13, 2022
 *      Author: saffaria
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "HDC2010.h"

#define SLAVE_ADDR  0x40
//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;


uint8_t TransmitRegAddr = 0;
uint8_t ReceiveBuffer = 0;
uint8_t RXByteCtr = 0;
uint8_t TransmitBuffer = 0;
uint8_t TXByteCtr = 0;
uint32_t sensor_data = 0;

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data);
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr);

I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    TXByteCtr = 0;
    RXByteCtr = 1;


    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;

}

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    TransmitBuffer = reg_data;

    TXByteCtr = 1;
    RXByteCtr = 0;


    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

void initI2C()
{

    P1SEL1 |= BIT6 | BIT7;                    // I2C pins

    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 0x0A;                            // fSCL = SMCLK/10 = ~100kHz
    UCB0I2CSA = SLAVE_ADDR;                   // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;
}



void hdc2010_triggerMeasurement(void)
{

    I2C_Master_WriteReg(SLAVE_ADDR, MEASUREMENT_CONFIG, 0x01);
    DELAY_CYCLES(1000);
    I2C_Master_ReadReg(SLAVE_ADDR, INTERRUPT_DRDY);

    while(!(ReceiveBuffer & 0x80))
    {
        I2C_Master_ReadReg(SLAVE_ADDR, INTERRUPT_DRDY);
        DELAY_CYCLES(10);
    }


}

void hdc2010_read(uint8_t* data)
{

    I2C_Master_ReadReg(SLAVE_ADDR, TEMP_LOW);
    data[0]= ReceiveBuffer;
    I2C_Master_ReadReg(SLAVE_ADDR, TEMP_HIGH);
    data[1] = ReceiveBuffer;
    I2C_Master_ReadReg(SLAVE_ADDR, HUMID_LOW);
    data[2] = ReceiveBuffer;
    I2C_Master_ReadReg(SLAVE_ADDR, HUMID_HIGH);
    data[3] = ReceiveBuffer;
}
// I2C Interrupt Routine

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB0RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG: break;         // Vector 4: NACKIFG
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
          ReceiveBuffer = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB0CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB0IE &= ~UCRXIE;
          MasterMode = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (MasterMode)
        {

          case TX_REG_ADDRESS_MODE:
              UCB0TXBUF = TransmitRegAddr;
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              UCB0IE |= UCRXIE;              // Enable RX interrupt
              UCB0IE &= ~UCTXIE;             // Disable TX interrupt
              UCB0CTLW0 &= ~UCTR;            // Switch to receiver
              MasterMode = RX_DATA_MODE;    // State state is to receive data
              UCB0CTLW0 |= UCTXSTT;          // Send repeated start
              if (RXByteCtr == 1)
              {
                  //Must send stop since this is the N-1 byte
                  while((UCB0CTLW0 & UCTXSTT));
                  UCB0CTLW0 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB0TXBUF = TransmitBuffer;
                  TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTLW0 |= UCTXSTP;                   // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB0IE &= ~UCTXIE;                      // disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
              }
              break;

          default:
              break;
        }
        break;
    default: break;
  }
}








