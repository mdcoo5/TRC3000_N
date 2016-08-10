#include <msp430g2553.h>

/*
 * main.c
 */

unsigned int PWM_Flag = 0;

void main(void) {
    WDTCTL = WDTPW + WDTHOLD; // Stop the watchdog

    BCSCTL1 = CALBC1_16MHZ; // Calibrated value for 16MHz SMCLK
    DCOCTL = CALDCO_16MHZ;

    P1OUT = 0x00; // Set port 1 to 0

    P1DIR |= 0xF7; // All output except P1.3
    P1SEL |= 0x40; // Sets P1.6 to TA0.1
    P2DIR |= 0xFF; // Sets port 2 to outputs
    P1OUT |= 0x08; // sets PU resistor for 1.3
    P1IE |= BIT3; // Interrupt Enable on button press 1.3
    P1IES |= BIT3; // Edgemode for Interrupt
    P1REN |= BIT3; // enables PU res on 1.3 

    TA0CCR0 = 640-1; // 16Mhz Clk with 640 steps = 25khz PWM
    TA0CCTL1 = OUTMOD_7; // Timer reset/set
    TA0CCR1 = 16; // initial duty cycle
    TA0CTL = TASSEL_2 + MC_1; //SMCLK, up mode

    P1IFG &= ~0x08; // clear edgecapture reg
    __enable_interrupt();

    _BIS_SR(LPM0_bits + GIE); // enter LPM0

    for(;;)
    {

    }
}

//Port 1 ISR
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) // Changes PWM duty cycle on TA0CCR1 on button press then returns to LMP0
{
	if(PWM_Flag)
		{
			TA0CCR1 = 16;
			PWM_Flag = 0;
		}
	else
		{
			TA0CCR1 = 350;
			PWM_Flag = 1;
		}


	P1IFG &= ~0x08; //clears interrupt flag
}
