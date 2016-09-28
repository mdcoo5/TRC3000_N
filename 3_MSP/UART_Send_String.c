#include <msp430.h>
#include <msp430g2553.h>

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

const char string[] = { "Hello World\r\n" };
unsigned int i; // counter

unsigned int I2C_flag = 0;
unsigned int Rx = 0, Tx = 0;
unsigned int TxByteCtr = 0, RxByteCtr = 0;

unsigned char TxData[2];
volatile unsigned char HeaderRxData[4];

void Transmit(void);
void Receive(void);

void main(void) {
    /*
     * ----------------------------------------------
     * Setup of initial clock values
     * 1MHz SMCLK/MCLK etc.
     * ----------------------------------------------
     */
	WDTCTL = WDTPW + WDTHOLD;
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    /*
     * ----------------------------------------------
     * Pin setup here
     * sets all P2 to outputs, output 0
     * sets P1.1 and P1.2 to RXD and TXD respectively
     * P1.6 & P1.7 SCL & SDA
     * (and sets P1.0 and P1.6 to RXD & TXD leds)
     * ----------------------------------------------
     */
    P2DIR = 0xFF;
    P2OUT &= 0x00;
    P1SEL |= RXD + TXD;
    P1SEL2 |= RXD + TXD;
    P1DIR |= RXLED + TXLED;
    P1OUT &= 0x00;

    /*
     * ----------------------------------------------
     * Setup of USCI Module here
     * UCA0 uses SMCLK
     * UCA0BR0 & UCA0BR1 set up to use 115200 baud
     * Modulation = 5 (p424 of user guide)
     * (Enable RX Interrupt)
     * ----------------------------------------------
     */
    UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = 0x08;
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS2 + UCBRS0;
    UCA0CTL1 &= ~UCSWRST;
    UC0IE |= UCA0RXIE;

    /*
     * Here we turn off the CPU and wait for the Interrupt on RX byte recieved
     */
	__bis_SR_register(CPUOFF + GIE);
    for(;;)
    {

    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	P1OUT |= RXLED;
	if (UCA0RXBUF == 'a') // wait for a
	{
		i = 0;
		UC0IE |= UCA0TXIE;
		UCA0TXBUF = string[i++];
	}
	P1OUT &= ~RXLED;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	P1OUT |= TXLED;
	UCA0TXBUF = string[i++];
	if (i == sizeof string - 1)
	{
		UC0IE &= ~UCA0TXIE; //disable USCI_A0 TX interrupt
	}
	P1OUT &= ~TXLED;
}
