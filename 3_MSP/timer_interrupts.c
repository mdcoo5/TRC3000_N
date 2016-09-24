/*
 * Periodic (timer) interrupts with sleep-wake servicing in main()
 * -> Toggle between red LED and green LED at 8Hz
 */

#include <msp430.h>

unsigned char ifg1 = 0;

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	/* Timer configuration */
	TA1CTL = TASSEL_2 + MC_1 + ID_3;	// Source select: SMCLK (TASSEL_2)
										// Mode control: up mode (MC_1)
										// Input divider: /8 (ID_3)
	TA1CCR0 = 15625 - 1;	// 8Hz overflow frequency
	TA1CCTL0 = CCIE;		// Enable interrupt request for TA1CCR0 CCIFG

	/* Pin configuration */
	P1OUT &= 0x00;			// Clear port 1 registers
	P1DIR &= 0x00;
	P1DIR |= (BIT0 | BIT6);	// Set P1.0 (LED1) and P1.6 (LED2) as outputs
	P1OUT |= BIT0;			// Initialise P1.0 output to 1

	for (;;)
	{
		// Enable general interrupts and enter LPM0
		__bis_SR_register(LPM0_bits + GIE);

		/* Interrupt servicing */
		if (ifg1){
			P1OUT ^= (BIT0 | BIT6);
			ifg1 = 0;
			// TACCR0 CCIFG flag is automatically reset when TACCR0 interrupt request serviced
		//}
	
		//elseif (ifg2){
			//...
		}
	}
}

/* TimerA1 ISR */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Port_1(void)
{
	ifg1 = 1;
	// Modify SR bits (clear CPUOFF) to keep CPU awake after return
	__bic_SR_register_on_exit(LPM0_bits + GIE);
}

