#include <msp430g2553.h>

/*
 * main.c
 */

unsigned int PWM_Flag = 0;

void main(void) {
    WDTCTL = WDTPW + WDTHOLD;

    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;

    P1OUT = 0x00;

    P1DIR |= 0xF7;
    P1SEL |= 0x40;
    P2DIR |= 0xFF;
    P1OUT |= 0x08; // PU resistor for 1.3
    P1IE |= BIT3; // IE on button press 1.3
    P1IES |= BIT3;
    P1REN |= BIT3;

    TA0CCR0 = 400-1; // 16Mhz Clk with 400 steps = 40khz PWM
    TA0CCTL1 = OUTMOD_7;
    TA0CCR1 = 16;
    TA0CTL = TASSEL_2 + MC_1;

    P1IFG &= ~0x08;
    __enable_interrupt();

    _BIS_SR(LPM0_bits + GIE);

    for(;;)
    {

    }
}

//Port 1 ISR
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
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
