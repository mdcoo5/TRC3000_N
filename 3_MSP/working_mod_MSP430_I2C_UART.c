/* Modified I2C/UART code:
 *	Reworked transmit and receive functions
 *		-  Allowed for variable # bytes, thereby combining transmit() and transmitOne()
 *  Moved set up function code to main to prevent constant re-execution
 *  Added more comments, definitions
 */
 
 // TODO - fixed-point maths library
 // 	 - combine with timer interrupt system
 
#include <msp430g2553.h>
#include <math.h>

#define IMU_ADDR		0x6B
#define ACCEL_ADDR		0x10
#define ACCEL_HIGHPWR	0x80
#define ACCEL_DATA		0x28

/* Angle estimation variables */
int PI = 3.141592653589;
float accel[3];
float angle1,angle0;
float tilt, roll;

/* I2C variables */
unsigned char RX = 0;
unsigned char *PRxData;
unsigned char *PTxData;
unsigned char RXByteCtr;
unsigned char TXByteCtr;
volatile unsigned char RxBuffer[6];		// Allocate 6 bytes of RAM for received I2C data
unsigned char TxData[2];				// Allocate 2 bytes of RAM for I2C data to be transmitted

/* UART variables */
unsigned char chr = 1;
unsigned char data = 0;

/* I2C function prototypes */
void I2C_Transmit(unsigned char* dataPtr, unsigned char num_TX_Bytes);
void I2C_Receive(unsigned char num_RX_Bytes);

/* UART function prototypes */
unsigned char UART_getChar(void);
//void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength);
//void UARTSendInt(unsigned int x);

/* main() */
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;				// Stop WDT
	BCSCTL1 = CALBC1_1MHZ;					// Configure DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;					// Configure DCO to 1MHz
	
	/* GPIO set up */
	P1DIR |= BIT0; 							// Red LED (P1.0)
	P1OUT |= BIT0; 							// Red LED (P1.0)
	P1SEL  |= BIT6 + BIT7;  				// I2C (P1.6 = SCL, P1.7 = SDA)
	P1SEL2 |= BIT6 + BIT7;  				// I2C (P1.6 = SCL, P1.7 = SDA)
	P1SEL |= BIT1 + BIT2;  					// Hardware UART (P1.1 = RXD, P1.2 = TXD)
	P1SEL2 |= BIT1 + BIT2; 					// Hardware UART (P1.1 = RXD, P1.2 = TXD)
	P2DIR |= BIT1 + BIT2 + BIT4 + BIT5;		// Motor drive signal pins
	
	/* I2C set up */
	UCB0CTL1 |= UCSWRST;                    // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;   // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;          // Use SMCLK, keep SW reset
	UCB0BR0 = 12;                           // fSCL = SMCLK/12 = ~100kHz
	UCB0BR1 = 0;
	UCB0I2CSA = IMU_ADDR;                   // Slave Address is 0x6B
	UCB0CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation
	IE2 |= UCB0RXIE + UCB0TXIE;             // Enable I2C RX/TX interrupts
	
	/* UART set up */
	UCA0CTL1 |= UCSSEL_2; 					// Use SMCLK
	UCA0BR0 = 104;        					// Set baud rate to 9600 with 1MHz clock (Data Sheet 15.3.13)
	UCA0BR1 = 0;          					// Set baud rate to 9600 with 1MHz clock
	UCA0MCTL = UCBRS0;    					// Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; 					// Initialize USCI state machine
	IE2 |= UCA0RXIE; 						// Enable UART RX interrupt

	/* TimerA1 (PWM) configuration */
	TA1CTL = TASSEL_2 + MC_1; 				// Source select: SMCLK (TASSEL_2)
											// Mode control: up mode (MC_1)
	TA1CCR0 = 500-1;						// PWM period
	TA1CCTL1 = OUTMOD_7;    				// Output mode 7 (reset/set)
	TA1CCTL2 = OUTMOD_7;
	TA1CCR1 = 0;							// Initialise 0% duty cycle
	TA1CCR2 = 0;

	/* TimerA0 (periodic interrupts) configuration */
	/*
	TA0CTL = TASSEL_2 + MC_1 + ID_3;		// Source select: SMCLK (TASSEL_2)
											// Mode control: up mode (MC_1)
											// Input divider: /8 (ID_3)
	TA0CCR0 = 15625 - 1;					// 8Hz overflow frequency
	TA0CCTL0 = CCIE;						// Enable interrupt request for TA0CCR0 CCIFG
	*/
	
	// Set up accelerometer for high power mode
	TxData[0] = ACCEL_ADDR;
	TxData[1] = ACCEL_HIGHPWR;
	I2C_Transmit((unsigned char*)TxData, 2);
	
	while(1)
	{
		// Request data from accelerometer in 2g range, 10Bit resolution
		TxData[0] = ACCEL_DATA;
		I2C_Transmit((unsigned char*)TxData, 1);
		while (UCB0CTL1 & UCTXSTP);         		// Ensure stop condition got sent (?)
		
		// Receive data from accelerometer (6 bytes)
		I2C_Receive(6);
		while (UCB0CTL1 & UCTXSTP);         		// Ensure stop condition got sent (?)
	
		/* Angle estimation calculations */
		/*
		int x = (((int)RxBuffer[1]) << 8) | RxBuffer[0];
		int y = (((int)RxBuffer[3]) << 8) | RxBuffer[2];
		int z = (((int)RxBuffer[5]) << 8) | RxBuffer[4];

		accel[0] = ((2.0*x)/32767.0); //x
		accel[1] = ((2.0*y)/32767.0); //y
		accel[2] = ((2.0*z)/32767.0); //z

		angle0 = -atan2(accel[1], sqrt(pow(-accel[2],2) + pow(-accel[0],2)));
		angle1 = -atan2(-accel[2], sqrt(pow(accel[1],2) + pow(-accel[0],2)));

		tilt = (angle0*180.0)/PI;
		roll = (angle1*180.0)/PI;
		*/
		
		// Retrieve UART character
		// data = UART_getChar();		// Call UART_getChar?
		data = chr;
		
		/* Motor control (PWM) */
		TA1CCR1 = 250;	// P2.1/P2.2 (TA1.1) duty cycle
		TA1CCR2 = 250;  // P2.4/P2.5 (TA1.2) duty cycle
		
		if(tilt >= 0 && !data){ 		// Both forward, Fast Decay
			P2SEL &= ~BIT2;					// Clear PWM for LIN2
           	P2OUT &= ~BIT2;					// Clear BIT2 out (LIN2) for fast decay
           	P2SEL |= BIT1; 					// PWM for LIN1

   			P2SEL &= ~BIT5;					// Clear PWM for RIN2
           	P2OUT &= ~BIT5;					// Clear BIT5 out (RIN2) for fast decay
   			P2SEL |= BIT4; 					// PWM for RIN1
		}else if(tilt <= 0 && !data){	// Both reverse, Fast Decay
          	P2SEL &= ~BIT1;					// Clear PWM for LIN1
            P2OUT &= ~BIT1;					// Clear BIT1 out (LIN1) for fast decay
            P2SEL |= BIT2; 					// PWM for LIN2

            P2SEL &= ~BIT4;					// Clear PWM for RIN1
   			P2OUT &= ~BIT4;					// Clear BIT4 out (RIN1) for fast decay
   			P2SEL |= BIT5; 					// PWM for RIN2
		}else if(data == 0x6C){ 		//Left Forward Motor, Right Reverse Motor
           	P2SEL &= ~BIT2;					// Clear PWM for LIN2
   			P2OUT &= ~BIT2;					// Clear BIT2 out (LIN2) for fast decay
   			P2SEL |= BIT1; 					// PWM for LIN1

   			P2SEL &= ~BIT4;					// Clear PWM for RIN1
   			P2OUT &= ~BIT4;					// Clear BIT4 out (RIN1) for fast decay
   			P2SEL |= BIT5; 					// PWM for RIN2
        }else if(data == 0x72){ 		// Left Reverse Motor, Right Forward Motor
           	P2SEL &= ~BIT1;					// Clear PWM for LIN1
   			P2OUT &= ~BIT1;					// Clear BIT1 out (LIN1) for fast decay
   			P2SEL |= BIT2; 					// PWM for LIN2

   			P2SEL &= ~BIT5;					// Clear PWM for RIN2
   			P2OUT &= ~BIT5;					// Clear BIT5 out (RIN2) for fast decay
   			P2SEL |= BIT4; 					// PWM for RIN1
		}
		//...	
	}
}

/* I2C transmit function */
void I2C_Transmit(unsigned char* dataPtr, unsigned char num_TX_Bytes)
{
	_DINT();
	RX = 0;								// Clear RX flag
	IE2 &= ~UCA0RXIE; 					// Disable UART RX interrupt (USCI_A0)
	IE2 &= ~UCB0RXIE; 					// Disable I2C RX interrupt (USCI_B0)
	IE2 |= UCB0TXIE;					// Enable I2C TX interrupt
	
	PTxData = dataPtr;					// Set PTxData to point to data to be transmitted
	TXByteCtr = num_TX_Bytes;			// Load TX byte counter w/ number of bytes to be transmitted
	while (UCB0CTL1 & UCTXSTP); 		// Ensure stop condition got sent
	UCB0CTL1 |= UCTR + UCTXSTT;      	// I2C TX, start condition
	__bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupts
										// Remain in LPM0 until all data is transmitted
}

/* I2C receive function -  */
void I2C_Receive(unsigned char num_RX_Bytes)
{
	_DINT();
	RX = 1;								// Set RX flag
	IE2 &= ~UCA0RXIE; 					// Disable UART RX interrupt (USCI_A0)
	IE2 &= ~UCB0TXIE; 					// Disable I2C TX interrupt (USCI_B0)
	IE2 |= UCB0RXIE;          			// Enable I2C RX interrupt
	
	PRxData = (unsigned char*)RxBuffer;	// Set PRxData to point to start of RxBuffer
	RXByteCtr = num_RX_Bytes;			// Load RX byte counter w/ number of bytes to be received
	while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
	UCB0CTL1 |= UCTXSTT;                // I2C start condition
	__bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupts
										// Remain in LPM0 until all data is received
}

/* I2C transmit/receive ISR */
#pragma vector = USCIAB0TX_VECTOR
// "I2C transmit and receive interrupt flags (UCBxTXIFG and UCBxRXIFG) from USCI_Bx share an interrupt vector"
__interrupt void USCIAB0TX_ISR(void)
{
	//------------------------------ Receive data -----------------------------
	// Move data from the I2C slave to MSP430 memory.
	//   PRxData points to the next byte to be received,
	//   RXByteCtr keeps track of the number of bytes received.
	//--------------------------------------------------------------------------	  
	if(RX){
		if (RXByteCtr){
			*PRxData++ = UCB0RXBUF;             // Move RX data byte to address PRxData, then increment pointer
			RXByteCtr--;						// Decrement RX byte counter
		}else{
			UCB0CTL1 |= UCTXSTP;                // I2C stop condition
			__bic_SR_register_on_exit(CPUOFF);  // Exit LPM0 (all data received)
		}
	  
	//------------------------------ Transmit data ------------------------------
	// Move data from MSP430 memory to the I2C master.
	//   PTxData points to the next byte to be transmitted,
	//   TXByteCtr keeps track of the number of bytes transmitted.
	//--------------------------------------------------------------------------
	}else{
		if (TXByteCtr)                          // Check TX byte counter
		{
			UCB0TXBUF = *PTxData++;      		// Load data byte at PTxData to TX buffer, then increment pointer
			TXByteCtr--;                        // Decrement TX byte counter
		}else{
			UCB0CTL1 |= UCTXSTP;                // I2C stop condition
			IFG2 &= ~UCB0TXIFG;                 // Clear USCI_B0 TX int flag
			__bic_SR_register_on_exit(CPUOFF);  // Exit LPM0 (all data transmitted)
		}
	}
	
	// UART (??)
	if (UCA0RXIFG){
		chr = UCA0RXBUF;
    }
}

/* UART getChar function */
unsigned char UART_getChar(void)
{
	_DINT();
	IE2 &= ~(UCB0RXIE + UCB0TXIE); 		// Disable I2C RX/TX interrupts (USCI_B0)
	IE2 |= UCA0RXIE; 					// Enable UART RX interrupt (USCI_A0)
	if (IFG2 & UCA0RXIFG){
        return UCA0RXBUF;
    }else{
    	return (unsigned char) 3;
    }
}

