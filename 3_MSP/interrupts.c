/*
 * interrupts.c
 *
 *  Created on: 10 Aug 2016
 *      Author: Sam
 */

#include <msp430.h>

int PWMduty = 0;

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;		// Set DCO for 16MHz

	//P1DIR |= BIT0;	// Set P1.0 to output direction
	P1REN |= BIT3;	// Enable pull up resistor on S2

	/* PWM configuration */
	P1DIR |= BIT6;	// Set P1.6 to output direction
	P2DIR |= BIT1;	// Set P2.1 to output direction
	P1SEL |= BIT6;	// Set P1.6 to TA0.1
	P2SEL |= BIT1;	// Set P2.1 to TA1.1
	TA0CCR0 = 640-1;				// PWM period
	TA1CCR0 = 640-1;
	TA0CCTL1 = OUTMOD_7;			// Output mode 7 (reset/set)
	TA1CCTL1 = OUTMOD_7;
	TA0CCR1 = 0;					// PWM initial duty cycle
	TA1CCR1 = 0;

	TA0CTL = TASSEL_2 + MC_1;	// SMCLK (TASSEL_2), up mode (MC_1)
	TA1CTL = TASSEL_2 + MC_1;

	/* Interrupt configuration */
	P1IE |= BIT3;	// Set P1.3 IE bit
	P1IES |= BIT3;	// Configure P1.3 IES bit (falling edge trigger)
	P1IFG &= ~BIT3;	// Clear P1.3 IFG bit

	// Enable general interrupts and enter LPM4
	_BIS_SR(LPM0_bits + GIE);
}

/* P1 ISR */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
	PWMduty = (PWMduty + 32)%(640 + 32);
	TA0CCR1 = (PWMduty == 0 ? 0 : PWMduty - 1);
	TA1CCR1 = (PWMduty == 640 ? 0 : (640 - PWMduty) - 1);
	P1IFG &= ~BIT3;                           // Clear P1.3 IFG
}
