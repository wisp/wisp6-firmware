/*
 * HM.c
 *
 *  Created on: Sep 12, 2022
 *      Author: saffaria
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "HM.h"

#define SLAVE_ADDR  0x24

char image_transferred = 0;
//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS1_MODE,
    TX_REG_ADDRESS2_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;


uint16_t TransmitRegAddr = 0;
uint8_t ReceiveBuffer = 0;
uint8_t RXByteCtr = 0;
uint8_t TransmitBuffer = 0;
uint8_t TXByteCtr = 0;


I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t reg_data);
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr);

I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS1_MODE;
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
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t reg_data)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS1_MODE;
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
    UCB0BRW = 0x0A;                            // fSCL = SMCLK/10 = ~400kHz
    UCB0I2CSA = SLAVE_ADDR;                   // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;
}

uint8_t init_Himax()
{
    uint8_t Himax_status = 0;
    FRCTL0 = FRCTLPW | NWAITS_1;                // 16MHZ operation so adding the wait state

                                                // Clock System Setup
    CSCTL0 = CSKEY;                             // unlock CS module registers
    CSCTL1 = DCORSEL | DCOFSEL_4;               // Set DCO to 16Mhz
    CSCTL2 = SELA_0 | SELM_3;
    CSCTL2 |= SELS_3;
    CSCTL3 = DIVA_0|DIVS_2|DIVM_0;              // MCLK= 16 MHz, SMCLK = 4 MHz
    CSCTL0_H = 0;                               // Lock CS registers

    PJDIR |= BIT0;                              // Output SMCLK on Pin J.0 to provide the clock for camera
    PJSEL1 &= ~BIT0;
    PJSEL0 |= BIT0;


    I2C_Master_WriteReg(SLAVE_ADDR, REG_MODE_SELECT, 0x00);

    I2C_Master_WriteReg(SLAVE_ADDR,  REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    I2C_Master_WriteReg(SLAVE_ADDR, REG_MODE_SELECT, 0x00);//go to stand by mode
    I2C_Master_WriteReg(SLAVE_ADDR, REG_ANA_REGISTER_17, 0x00);//register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default


//    I2C_Master_WriteReg(SLAVE_ADDR, REG_TEST_PATTERN_MODE, TEST_PATTERN_WALKING_1);    //Enable the test pattern, set it to walking 1


    //Image size settings. Enable QQVGA.
    I2C_Master_WriteReg(SLAVE_ADDR, REG_QVGA_WIN_EN, 0x01);//Enable QVGA Window
    I2C_Master_WriteReg(SLAVE_ADDR, REG_BIN_RDOUT_X, 0x03);//Horizontal Binning enable
    I2C_Master_WriteReg(SLAVE_ADDR, REG_BIN_RDOUT_Y, 0x03);//Vertical Binning enable
    I2C_Master_WriteReg(SLAVE_ADDR, REG_BINNING_MODE, 0x03);//VERTICAL BIN MODE: Horizontal & Vertical


    I2C_Master_WriteReg(SLAVE_ADDR,0x3044,0x0A);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3045,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3047,0x0A);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3050,0xC0);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3051,0x42);
//    hm_i2c_write(hi2c,0x3052,0x50);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3053,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3054,0x03);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3055,0xF7);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3056,0xF8);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3057,0x29);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3058,0x1F);
//    hm_i2c_write(hi2c,0x3059,0x1E);//bit control
    I2C_Master_WriteReg(SLAVE_ADDR,0x3064,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x3065,0x04);

    //black level control
    I2C_Master_WriteReg(SLAVE_ADDR,0x1000,0x43);
    I2C_Master_WriteReg(SLAVE_ADDR,0x1001,0x40);
    I2C_Master_WriteReg(SLAVE_ADDR,0x1002,0x32);
    I2C_Master_WriteReg(SLAVE_ADDR,0x1003,0x08);//default from lattice 0x08
    I2C_Master_WriteReg(SLAVE_ADDR,0x1006,0x01);
    I2C_Master_WriteReg(SLAVE_ADDR,0x1007,0x08);//default from lattice 0x08

    I2C_Master_WriteReg(SLAVE_ADDR,0x0350,0x7F);


    //Sensor reserved
    I2C_Master_WriteReg(SLAVE_ADDR,0x1008,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x1009,0xA0);
    I2C_Master_WriteReg(SLAVE_ADDR,0x100A,0x60);
    I2C_Master_WriteReg(SLAVE_ADDR,0x100B,0x90);//default from lattice 0x90
    I2C_Master_WriteReg(SLAVE_ADDR,0x100C,0x40);//default from lattice 0x40

    //Vsync, hsync and pixel shift register
    I2C_Master_WriteReg(SLAVE_ADDR, REG_VSYNC_HSYNC_PIXEL_SHIFT_EN, 0x00); //lattice value

    //Statistic control and read only
    I2C_Master_WriteReg(SLAVE_ADDR,0x2000,0x07);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2003,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2004,0x1C);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2007,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2008,0x58);
    I2C_Master_WriteReg(SLAVE_ADDR,0x200B,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x200C,0x7A);
    I2C_Master_WriteReg(SLAVE_ADDR,0x200F,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2010,0xB8);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2013,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2014,0x58);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2017,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2018,0x9B);

    //Automatic exposure gain control
    I2C_Master_WriteReg(SLAVE_ADDR,0x2100,0x01);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2101,0x15);//0x70);//lattice 0xA0//changed on Nov 8 2022
    I2C_Master_WriteReg(SLAVE_ADDR,0x2102,0x01);//lattice 0x06
    I2C_Master_WriteReg(SLAVE_ADDR,0x2104,0x07);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2105,0x02); //from 3 to 2 on Nov 8 2022
    I2C_Master_WriteReg(SLAVE_ADDR,0x2106,0xA4);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2108,0x33);
    I2C_Master_WriteReg(SLAVE_ADDR,0x210A,0x00);
    //hm_i2c_write(hi2c,0x210C,0x04);
    I2C_Master_WriteReg(SLAVE_ADDR,0x210B,0x80);
    I2C_Master_WriteReg(SLAVE_ADDR,0x210F,0x00);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2110,0xE9);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2111,0x01);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2112,0x17);
    I2C_Master_WriteReg(SLAVE_ADDR,0x2150,0x03);

    //Sensor exposure gain
    I2C_Master_WriteReg(SLAVE_ADDR,0x0205,0x05);//Vikram
    I2C_Master_WriteReg(SLAVE_ADDR,0x020E,0x01);//Vikram
    I2C_Master_WriteReg(SLAVE_ADDR,0x020F,0x00);//Vikram
    I2C_Master_WriteReg(SLAVE_ADDR,0x0202,0x01);//Vikram
    I2C_Master_WriteReg(SLAVE_ADDR,0x0203,0x08);//Vikram

    //frame timing control
    I2C_Master_WriteReg(SLAVE_ADDR, REG_FRAME_LENGTH_LINES_H, 0x00);
    I2C_Master_WriteReg(SLAVE_ADDR, REG_FRAME_LENGTH_LINES_L, 0xCC);
    I2C_Master_WriteReg(SLAVE_ADDR, REG_FRAME_LENGTH_PCK_H, 0x00);
    I2C_Master_WriteReg(SLAVE_ADDR, REG_FRAME_LENGTH_PCK_L, 0xEE);

    I2C_Master_WriteReg(SLAVE_ADDR, REG_OSC_CLK_DIV, 0x38);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    I2C_Master_WriteReg(SLAVE_ADDR, REG_BIT_CONTROL, 0x20);//Set the output to send 1 bit serial

    I2C_Master_WriteReg(SLAVE_ADDR, REG_MODE_SELECT, MODE_STREAMING);

    I2C_Master_ReadReg(SLAVE_ADDR, REG_MODEL_ID_L);

    if (ReceiveBuffer ==  0xB0)                         // check if I2C returns the correct ID
    {
        Himax_status = 1;
    }

    return Himax_status;
}


void init_SPI(void)
{
    P1SEL1 |= BIT5;                                    // Configure SPI pins
    P2SEL1 |= BIT0 | BIT1;


    DMACTL0 |= DMA0TSEL_14;                           // DMA0 trigger source: UCA0RXIFG


    DMA0CTL |= DMADT_4 | DMADSTINCR_3 | DMASRCINCR_0  // single repeat, increment destination address
            | DMADSTBYTE | DMASRCBYTE | DMALEVEL;     // same source address, byte data size, level trigger

    DMA0SZ = 19764;                                    //162*122 BYTES data size


    __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) 0x05CC); //SPI A0 RX buffer address as the source
    __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) 0x7000);// memory address for saving the image as the destination

    UCA0CTLW0 = UCSWRST;                      // **Put state machine in reset**
    UCA0CTLW0 |= UCSYNC | UCMSB;              // 3-pin, 8-bit SPI slave
                                              // Clock polarity low, MSB
    UCA0CTLW0 |= UCSSEL__SMCLK;               // MCLK
    UCA0BR0 = 0x02;                           // /2
    UCA0BR1 = 0;                              //
    UCA0MCTLW = 0;                            // No modulation
    UCA0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**\

                                              // Enable FVLD interrupt to find the frame start
    P3IFG &= ~BIT6;                           // Clear all P3.6 interrupt flag
    P3IE |= BIT6;                             // P3.6 interrupt enabled
    __bis_SR_register(LPM0_bits|GIE);       // Enter LPM0 w/ interrupt

}


// DMA interrupt routine
#pragma vector=DMA_VECTOR
__interrupt void DMA(void)
{
                                              // data is received by the MCC

    image_transferred = 1;                            // Disable the camera and clock
    P3OUT &= ~BIT3;
    PJSEL1 &= ~BIT0;
    PJSEL0 &= ~BIT0;
    PJOUT &= ~BIT0;
    DMA0CTL &= ~DMAIFG;
    DMA0CTL &= ~DMAEN;
    DMA0CTL &= ~DMAIE;
    __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0


}

// P3 Interrupt routine service

#pragma vector=PORT3_VECTOR
__interrupt void Port_3(void)
{
    if (P3IFG & BIT6)
    {
        P3IE &= ~BIT6;
        P3IFG &= ~BIT6;
        P3OUT |= BIT5;
        DMA0CTL |= DMAEN |DMAIE;            // Enable DMA and interrupt since we have found the frame start

    }

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
          case TX_REG_ADDRESS1_MODE:
              UCB0TXBUF = (uint8_t)(TransmitRegAddr >>8);
              MasterMode = TX_REG_ADDRESS2_MODE;
              break;

          case TX_REG_ADDRESS2_MODE:
              UCB0TXBUF = (uint8_t)(TransmitRegAddr);
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




