/*
 *
 *
 */

#include  "msp430g2553.h"

#define PWM_PERIOD		127
#define MOTOR_FWD		0x80
#define MOTOR_PWM_DUTY	0x7F

/* UART variables */
unsigned char *UART_PRxData;
unsigned char UART_RXByteCtr;
volatile unsigned char UART_RxBuffer[6];
unsigned char UART_char;
const char string[] = {0x6C, 0x72, 0x0D, 0};
unsigned int i = 0;

/* UART function prototypes */
void UART_Setup(void);
void UART_Receive(unsigned char num_RX_Bytes);
void UART_Transmit(void);

/* Other function prototypes */
void driveMotors(unsigned char L_CTRL, unsigned char R_CTRL);

int fastDecay = 0;

/* main() */
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 	// Stop WDT
	BCSCTL1 = CALBC1_1MHZ;                    	// Configure DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;

	/* GPIO set up */
	P1SEL |= BIT1 + BIT2;                     	// UART (P1.1 = RXD, P1.2 = TXD)
	P1SEL2 |= BIT1 + BIT2;                    	// UART (P1.1 = RXD, P1.2 = TXD)
	P1DIR |= BIT0;								// Red/green LED
	P1OUT &= ~BIT0;								// Red/green LED
	P2DIR |= BIT1 + BIT2 + BIT4 + BIT5;			// Motor drive signal pins

	/* TimerA1 (PWM) configuration */
	TA1CTL = TASSEL_2 + MC_1 + ID_2; 			// Source select: SMCLK (TASSEL_2)
												// Mode control: up mode (MC_1)
												// Input divider: /4 (ID_2)
	TA1CCR0 = PWM_PERIOD;						// PWM period
	TA1CCTL1 = OUTMOD_7;    					// Output mode 7 (reset/set)
	TA1CCTL2 = OUTMOD_7;
	TA1CCR1 = 0;								// Initialise 0% duty cycle
	TA1CCR2 = 0;

	/* UART setup */
	UART_Setup();								// Setup UART

	while(1){
		UART_Receive(1);						// Receive 1 bytes of UART data
		driveMotors(UART_RxBuffer[0], UART_RxBuffer[0]);
	}
}

/* UART setup function */
void UART_Setup(void)
{
	UCA0CTL1 |= UCSSEL_2; 							// Use SMCLK
	UCA0BR0 = 0x08;        							// Set baud rate to 115200 with 1MHz clock
	UCA0BR1 = 0x00;          						// Set baud rate to 115200 with 1MHz clock
	UCA0MCTL = UCBRS2 + UCBRS0;    					// Modulation UCBRSx = 5
	UCA0CTL1 &= ~UCSWRST; 							// Initialize USCI state machine
	IFG2 &= ~UCA0TXIFG;
}

/* UART receive data function */
void UART_Receive(unsigned char num_RX_Bytes)
{
	IE2 |= UCA0RXIE; 								// Enable UART RX interrupt
	UART_PRxData = (unsigned char*)UART_RxBuffer;
	UART_RXByteCtr = num_RX_Bytes;					// Load number of bytes to be received
	__bis_SR_register(CPUOFF + GIE);    			// Enter LPM0 w/ interrupts
													// Remain in LPM0 until all data is received*/
}

///* UART transmit data function */
//void UART_Transmit()
//{
//	i = 0;
//	UCA0TXBUF = string[i++];
//	UC0IE |= UCA0TXIE;								// Enable UART TX interrupt
//	__bis_SR_register(CPUOFF + GIE);    			// Enter LPM0 w/ interrupts
//													// Remain in LPM0 until all data is received*/
//}

/* UART RX interrupt servicing */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	UART_RXByteCtr--;								// Decrement RX byte counter
	if(UART_RXByteCtr)								// Check RX byte counter
	{
		*UART_PRxData++ = UCA0RXBUF;				// Load byte from UART RX buffer
	}else{
		IE2 &= ~UCA0RXIE;							// Disable UART RX interrupt
		*UART_PRxData++ = UCA0RXBUF;				// Load final byte from UART RX buffer
		__bic_SR_register_on_exit(CPUOFF);			// Exit LPM0 (all data received)
	}

}

/* UART TX interrupt servicing */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	if(IFG2&UCA0TXIFG){
		//------------------------- UART Transmit data -----------------------------
		// Move data from MSP430 memory to the UART buffer
		//	__ points to the next byte to be transmitted,
		//	__ keeps track of the number of bytes transmitted
		//--------------------------------------------------------------------------
		UCA0TXBUF = string[i++];						// Load byte onto UART TX buffer
		if(i == (sizeof(string) - 1))					// All data has been TX'd
		{
			UC0IE &= ~UCA0TXIE;							// Disable UART TX interrupt
			__bic_SR_register_on_exit(CPUOFF);
		}

	}
}


void driveMotors(unsigned char L_CTRL, unsigned char R_CTRL)
{
	/* Motor direction and decay mode */
	//	  Fast decay:
	//		  PWM	  0			Forward
	//		  0	  	  PWM		Reverse
	if(fastDecay){
		if(L_CTRL&MOTOR_FWD){				// LEFT motor forward:
			P2SEL &= ~BIT2;					//   Clear PWM for LIN2
			P2OUT &= ~BIT2;					//   Clear BIT2 out (LIN2) for fast decay
			P2SEL |= BIT1; 					//   PWM for LIN1
		}else{								// LEFT motor reverse:
			P2SEL &= ~BIT1;					//   Clear PWM for LIN1
			P2OUT &= ~BIT1;					//   Clear BIT1 out (LIN1) for fast decay
			P2SEL |= BIT2; 					//   PWM for LIN2
		}

		if(R_CTRL&MOTOR_FWD){				// RIGHT motor forward:
			P2SEL &= ~BIT5;					//   Clear PWM for RIN2
			P2OUT &= ~BIT5;					//   Clear BIT5 out (RIN2) for fast decay
			P2SEL |= BIT4; 					//   PWM for RIN1
		}else{								// RIGHT motor reverse:
			P2SEL &= ~BIT4;					//   Clear PWM for RIN1
			P2OUT &= ~BIT4;					//   Clear BIT4 out (RIN1) for fast decay
			P2SEL |= BIT5; 					//   PWM for RIN2
		}
	}else{
	//	  Fast decay:
	//		  1		  	PWM		Forward
	//		  PWM	  	1		Reverse
		if(L_CTRL&MOTOR_FWD){				// LEFT motor forward:
			P2SEL &= ~BIT1;					//   Clear PWM for LIN1
			P2OUT |= BIT1;					//   Set BIT1 out (LIN1) for slow decay
			if(L_CTRL > 0) P2OUT |= BIT1;
			else P2OUT &= ~BIT1;            // 	 Clear BIT1 so motors do not see fast (PWM=0, BIT1=1)
			P2SEL |= BIT2; 					//   PWM for LIN2
		}else if(L_CTRL){								// LEFT motor reverse:
			P2SEL &= ~BIT2;					//   Clear PWM for LIN2
			if(L_CTRL > 0) P2OUT |= BIT2;		//   Set BIT2 out (LIN2) for slow decay
			else P2OUT &= ~BIT2;			//   Clear BIT2 so motors do not see fast (PWM=0, BIT2=1)
			P2SEL |= BIT1; 					//   PWM for LIN1
		}else{
			P2SEL &= ~BIT1 + ~BIT2;			// Clear to 0 at pwm = 0
			P2OUT &= ~BIT1 + ~BIT2;
		}

		if(R_CTRL&MOTOR_FWD){				// RIGHT motor forward:
			P2SEL &= ~BIT4;					//   Clear PWM for RIN1
			if(R_CTRL > 0) P2OUT |= BIT4;		//   Set BIT4 out (RIN1) for slow decay
			else P2OUT &= ~BIT4;			//   Clear BIT4 so motors do not see fast (PWM=0, BIT4=1)
			P2SEL |= BIT5; 					//   PWM for RIN2
		}else if(R_CTRL){								// RIGHT motor reverse:
			P2SEL &= ~BIT5;					//   Clear PWM for RIN2
			if(R_CTRL > 0) P2OUT |= BIT5;		//   Set BIT5 out (RIN2) for slow decay
			else P2OUT &= ~BIT5;			//   Clear BIT5 so motors do not see fast (PWM=0, BIT5=1)
			P2SEL |= BIT4; 					//   PWM for RIN1
		}else{
			P2SEL &= ~BIT4 + ~BIT5;			// Clear to 0 at pwm = 0
			P2OUT &= ~BIT4 + ~BIT5;
		}
	}


	/* Motor PWM */
	TA1CCR1 = (L_CTRL&MOTOR_PWM_DUTY);	// LEFT motor duty cycle
	TA1CCR2 = (R_CTRL&MOTOR_PWM_DUTY);  // RIGHT motor duty cycle
}
