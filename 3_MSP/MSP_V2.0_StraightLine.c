/*	UART implementation w/ start byte & error checking (but no header)
 *		Receives start byte, receives message w/checksum, checks message checksum
 *		If message checksum OK, performs required actions and waits for next start byte
 *
 *		Requires transmitter to transmit packets of {START_BYTE, L_CTRL, R_CTRL, CHECKSUM}
 *			Where CHECKSUM is computed based on {L_CTRL, R_CTRL}.  (See checkSum function)
 *
 *		Notes: PWM_PERIOD is 126 (0x7E)
 */

#include "msp430g2553.h"
#include <stdint.h>

#define START_BYTE 		0x7F
#define MESSAGE_LEN		4							// Two data bytes (L_CTRL, R_CTRL) and one checksum byte

#define PWM_PERIOD		126
#define MOTOR_DIR_MASK	0x80
#define MOTOR_PWM_MASK	0x7F

const int fastDecay = 0;									// 0 for slow decay, 1 for fast decay

/* UART variables */
unsigned char UART_RXByteCtr;						// Stores number of bytes to be received
unsigned char *UART_PRxData;						// Pointer to destination for received data
unsigned char UART_message[MESSAGE_LEN];			// Array to store message bytes
unsigned char UART_rxbyte = 0x00;

/* UART function prototypes */
void UART_Setup(void);
void UART_Receive(uint8_t num_RX_Bytes, unsigned char* destinationPtr);
void UART_Transmit(void);

/* Other function prototypes */
void driveMotors(unsigned char L_CTRL, unsigned char R_CTRL);
uint8_t checkSum(uint8_t const message[], int nBytes);

/* main() */
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 	// Stop WDT
	BCSCTL1 = CALBC1_1MHZ;                    	// Configure DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;

	/* GPIO configuration */
	P1SEL |= BIT1 + BIT2;                     	// UART (P1.1 = RXD, P1.2 = TXD)
	P1SEL2 |= BIT1 + BIT2;                    	// UART (P1.1 = RXD, P1.2 = TXD)
	P1DIR |= BIT0 + BIT3;						// Red LED / test pin (P1.3)
	P1OUT &= ~(BIT0 + BIT3);					// Red LED / test pin (P1.3)
	P2DIR |= BIT1 + BIT2 + BIT4 + BIT5;			// Motor drive signal pins

	/* TimerA1 (PWM) configuration */
	TA1CTL = TASSEL_2 + MC_1 + ID_2; 			// Source select: SMCLK (TASSEL_2)
	//											// Mode control: up mode (MC_1)
	//											// Input divider: /4 (ID_2)
	TA1CCR0 = PWM_PERIOD;						// PWM period
	TA1CCTL1 = OUTMOD_7;    					// Output mode 7 (reset/set)
	TA1CCTL2 = OUTMOD_7;
	TA1CCR1 = 0;								// Initialise 0% duty cycle
	TA1CCR2 = 0;

	/* UART setup */
	UART_Setup();

	while(1)
	{
		//P1OUT &= ~BIT3;
		//unsigned char UART_rxbyte = 0x00;
		while(UART_rxbyte != START_BYTE){UART_Receive(1, &UART_rxbyte);}	// Wait for a start byte

		P1OUT ^= BIT0;
		UART_Receive(MESSAGE_LEN, (unsigned char*)UART_message);			// Receive next 3 bytes, corresponding to message length


		/*  UART_message[0] = L_CTRL				*
		 *  UART_message[1] = R_CTRL				*
		 *  UART_message[2] = message checksum    	*/

		if(checkSum(UART_message, MESSAGE_LEN-1) == UART_message[MESSAGE_LEN-1])	// Compute & check message checksum
		{
			driveMotors(UART_message[1], UART_message[2]);
		} // else message checksum error
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
void UART_Receive(uint8_t num_RX_Bytes, unsigned char* destinationPtr)
{
	IE2 |= UCA0RXIE; 								// Enable UART RX interrupt
	UART_PRxData = destinationPtr;					// Set destination for received data
	UART_RXByteCtr = num_RX_Bytes;					// Store # bytes to be received in counter
	__bis_SR_register(CPUOFF + GIE);    			// Enter LPM0 w/ interrupts
													// Remain in LPM0 until all data is received*/
}

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

///* UART transmit data function */
//void UART_Transmit()
//{
//	i = 0;
//	UCA0TXBUF = string[i++];						// Load first byte onto TX buffer
//	UC0IE |= UCA0TXIE;								// Enable UART TX interrupt
//	__bis_SR_register(CPUOFF + GIE);    			// Enter LPM0 w/ interrupts
//													// Remain in LPM0 until all data is transmitted*/
//}
//
///* UART TX interrupt servicing */
//#pragma vector=USCIAB0TX_VECTOR
//__interrupt void USCI0TX_ISR(void)
//{
//	if(IFG2&UCA0TXIFG){
//		UCA0TXBUF = string[i++];						// Load byte onto UART TX buffer
//		if(i == (sizeof(string) - 1))
//		{
//			UC0IE &= ~UCA0TXIE;							// Disable UART TX interrupt
//			__bic_SR_register_on_exit(CPUOFF);			// Exit LPM0 (all data transmitted)
//		}
//	}
//}

/* Checksum calculation function */
uint8_t checkSum(uint8_t const message[], int nBytes)
{
	uint8_t sum = 0;
	while (nBytes-- > 0) sum += *(message++);
	return(~sum);
}

/* Drive motors function */
void driveMotors(unsigned char L_CTRL, unsigned char R_CTRL)
{
	/* Motor direction and decay mode */
	//	Fast decay:				IN1:	IN2:
	//				Forward:	PWM	  	0
	//		  		Reverse:	0	  	PWM
	//
	//	Slow decay:				IN1:	IN2:
	//				Forward:	1		PWM
	//				Reverse:	PWM	  	1
	if(fastDecay){
		if(L_CTRL&MOTOR_DIR_MASK){			// LEFT motor forward:
			P2SEL &= ~BIT2;					//   Clear PWM for LIN2
			P2OUT &= ~BIT2;					//   Clear BIT2 out (LIN2) for fast decay
			P2SEL |= BIT1; 					//   PWM for LIN1
		}else{								// LEFT motor reverse:
			P2SEL &= ~BIT1;					//   Clear PWM for LIN1
			P2OUT &= ~BIT1;					//   Clear BIT1 out (LIN1) for fast decay
			P2SEL |= BIT2; 					//   PWM for LIN2
		}

		if(R_CTRL&MOTOR_DIR_MASK){			// RIGHT motor forward:
			P2SEL &= ~BIT5;					//   Clear PWM for RIN2
			P2OUT &= ~BIT5;					//   Clear BIT5 out (RIN2) for fast decay
			P2SEL |= BIT4; 					//   PWM for RIN1
		}else{								// RIGHT motor reverse:
			P2SEL &= ~BIT4;					//   Clear PWM for RIN1
			P2OUT &= ~BIT4;					//   Clear BIT4 out (RIN1) for fast decay
			P2SEL |= BIT5; 					//   PWM for RIN2
		}
	}else{
		if(L_CTRL&MOTOR_DIR_MASK){			// LEFT motor forward:
			P2SEL &= ~BIT1;					//   Clear PWM for LIN1
			P2OUT |= BIT1;					//   Set BIT1 out (LIN1) for slow decay
			P2SEL |= BIT2; 					//   PWM for LIN2
		}else{								// LEFT motor reverse:
			P2SEL &= ~BIT2;					//   Clear PWM for LIN2
			P2OUT |= BIT2;					//   Set BIT2 out (LIN2) for slow decay
			P2SEL |= BIT1; 					//   PWM for LIN1
		}

		if(R_CTRL&MOTOR_DIR_MASK){			// RIGHT motor forward:
			P2SEL &= ~BIT4;					//   Clear PWM for RIN1
			P2OUT |= BIT4;					//   Set BIT4 out (RIN1) for slow decay
			P2SEL |= BIT5; 					//   PWM for RIN2
		}else{								// RIGHT motor reverse:
			P2SEL &= ~BIT5;					//   Clear PWM for RIN2
			P2OUT |= BIT5;					//   Set BIT5 out (RIN2) for slow decay
			P2SEL |= BIT4; 					//   PWM for RIN1
		}
	}

	/* Motor PWM */
	TA1CCR1 = (L_CTRL&MOTOR_PWM_MASK);	// LEFT motor duty cycle
	TA1CCR2 = (R_CTRL&MOTOR_PWM_MASK);  // RIGHT motor duty cycle
}
