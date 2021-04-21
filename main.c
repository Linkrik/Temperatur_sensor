
//******************************************************************************
#include "driverlib.h"

//*****************************************************************************
//
//Set the address for slave module. This is a 7-bit address sent in the
//following format:
//[A6:A5:A4:A3:A2:A1:A0:RS]
//
//A zero in the "RS" position of the first byte means that the master
//transmits (sends) data to the selected slave, and a one in this position
//means that the master receives data from the slave.
//
//*****************************************************************************
#define SLAVE_ADDRESS 0x48

//*****************************************************************************
//
//Target frequency for SMCLK in kHz
//
//*****************************************************************************


//*****************************************************************************
//
//SMCLK/FLLRef Ratio
//
//*****************************************************************************


uint8_t transmitData=0;
uint8_t RXData1=0;
uint8_t RXData2=0;
uint8_t RXDataUart=0;

void initClockTo8MHz();
void initI2C();
unsigned char uart_receive_byte();
void uart_transmit_byte(unsigned char byte);
void initUART();


void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    initClockTo8MHz();
    initUART();
    initI2C();

    P3OUT &= ~(BIT0|BIT1|BIT2); //Clear P1.0 output latch
    P3DIR |= BIT0|BIT1|BIT2; //For LED
    unsigned char counter=0;


     while (1)
    {
      __delay_cycles(2000);

      EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE,
                  transmitData
                  );
      if(counter>8)
      {
          counter=0;
      }

      P3OUT = counter;
      counter++;

      while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE)) ;

     // I2C start condition
      RXData1 = EUSCI_B_I2C_masterReceiveSingleByte(EUSCI_B0_BASE);
      RXData2 = EUSCI_B_I2C_masterReceiveSingleByte(EUSCI_B0_BASE);

      RXDataUart=uart_receive_byte();

      uart_transmit_byte(RXData1);
      uart_transmit_byte(RXData2);


    }


}



void initI2C()
{
    P1OUT &= ~BIT0; //Clear P1.0 output latch
    P1DIR |= BIT0; //For LED
    // Configure Pins for I2C
    P1SEL0 |= BIT2 | BIT3;                                // I2C pins


    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure USCI_B0 for I2C mode
    UCB0CTLW0 |= UCSWRST;            //Software reset enabled
    UCB0CTLW0 |= UCMODE_3 |          // I2C
                 UCMST|              // master mode
                 UCSSEL__SMCLK |     // CLK source: SMCLK (8 MHz)
                 UCSYNC;             // Synchronous mode enable;
    UCB0CTLW1 |= UCASTP_2;            // Automatic stop generated
    UCB0TBCNT = 0x0002;

    UCB0BRW = 80;                   //  SCL = SMCLK / 80 = ~100 KHz
    UCB0I2CSA = 0x48;                  //Slave Address initialization

    UCB0CTLW0 &=~UCSWRST;                                 //clear reset register

    UCB0IE &= ~(UCRXIE0 | UCTXIE0 | UCNACKIE);
    UCB0IE |=  UCRXIE0 | UCTXIE0 | UCNACKIE;//UCRXIE1| UCRXIE2 | UCRXIE3 | UCTXIE0 |UCTXIE1 |UCTXIE2 | UCTXIE3;      //receive interrupt enable
}

void initClockTo8MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);    // disable FLL
    CSCTL3 |= SELREF__REFOCLK;  // Set REFO as FLL reference source
    CSCTL0 = 0;                 // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);     // Clear DCO frequency select bits first
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
    CSCTL2 = FLLD_0 + 243;      // set to fDCOCLKDIV = (FLLN + 1)*(fFLLREFCLK/n)
                                //                   = (243 + 1)*(32.768 kHz/1)
                                //                   = 8 MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                        // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));      // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;// set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
    // default DCODIV as MCLK and SMCLK source
}


void initUART()
{
    // Configure UART pins
    //P1SEL0 |= BIT4 | BIT5;                    // set 2-UART pin as second function
    P2SEL0 |= BIT6 | BIT5;                    // set 2-UART pin as second function

    // Configure USCI_A0 for UART mode
    UCA1CTLW0 |= UCSWRST | UCSSEL__SMCLK;     //Четность(отключена);
                                              //Направление сдвига(сначала младший бит)
                                              //Колличество битов в посылке(8 бит)
                                              //Кол-во стоп битов (1)
                                              //UART
                                              //Синхронный режим
                                              //Источник часов (SMCLK)
                                              // Put eUSCI in reset

        // Baud Rate calculation
        // 8000000/(16*115200) = 4.34
        // Fractional portion = 0.34
        // User's Guide Table 22-4: UCBRSx = 0x49
        // UCBRFx = int ( (4.34-4)*16) = 5
    UCA1BR0 = 4;                             // 8000000/16/115200
    UCA1BR1 = 0x00;
    UCA1MCTLW = 0x4900 | UCOS16 | UCBRF_5 ;

    UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    UCA1IE |= UCRXIE|UCTXIE;                         // Enable USCI_A0 RX interrupt
}


void uart_transmit_byte(unsigned char byte)
{
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = byte;                         /** transmit the same character on terminal*/
}

unsigned char uart_receive_byte()
{
     while(!(UCA1IFG & UCRXIFG));
     return UCA1RXBUF;                        /** receive character from user */
}
