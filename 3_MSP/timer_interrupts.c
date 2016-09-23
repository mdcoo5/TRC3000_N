/*
 * Periodic (timer) interrupts
 * Toggle between red LED and green LED at 2Hz
 */

#include <msp430.h>

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	/* Timer configuration */
	TA1CTL = TASSEL_2 + MC_1 + ID_3;	// Source select: SMCLK (TASSEL_2)
										// Mode control: up mode (MC_1)
										// Input divider: /8 (ID_3)
	TA1CCR0 = 62500;		// 2Hz overflow frequency
	TA1CCTL0 = CCIE;		// Enable interrupt request for TA1CCR0 CCIFG

	/* Pin configuration */
	P1OUT &= 0x00;			// Clear port 1 registers
	P1DIR &= 0x00;
	P1DIR |= (BIT0 | BIT6);	// Set P1.0 (LED1) and P1.6 (LED2) as outputs
	P1OUT |= BIT0;

	// Enable general interrupts and enter LPM4
	_BIS_SR(LPM0_bits + GIE);

}

/* TimerA0 ISR */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Port_1(void)
{
	P1OUT ^= (BIT0 | BIT6);
	//TACCR0 CCIFG flag is automatically reset when TACCR0 interrupt request serviced
}
