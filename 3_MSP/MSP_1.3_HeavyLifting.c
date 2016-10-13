/* 	MSP does the heavy lifting (no BBB)
 *	  Reads from I2C
 *	  Computes angle, filtered angle, PID, etc
 *	  Drives motors in same direction
 */


/* --COPYRIGHT--,
 * Copyright (c) 2013, Bariscan Kayaoglu
 * All rights reserved.
 *
 * --COPYRIGHT--*/
//******************************************************************************
//  MSP430G2xx3 I2C Demo - USCI_B0 I2C Master TX/RX single byte from ITG-3200
//  gyroscope.
//
//  Description: This demo connects a MSP430Gxx3 launchpad to ITG-3200 via
//  USCI_B0 I2C module. The launchpad act as a master and itg-300 act as a
//  slave. Master reads 6 byte consecutively and send it to the vitual COM
//  Port on Usb with USCI_A0 Uart module. DCO = 1MHz
//
//                                /|\  /|\
//                  ITG-3200      10k  10k     MSP430G2xx3
//                   slave         |    |        master
//             -----------------   |    |     -----------------
//            |              SDA|<-|----+---->|P1.7/UCB0SDA     |
//            |              CLK|<-|-.GND     |                 |
//            |              GND|<-|-'        |                 |
//            |              SCL|<-+--------->|P1.6/UCB0SCL     |
//            |              VCC|<--.         |      P1.1/UCA0TX|-------> COM Port (9600 baud)
//            |              VIO|<--'3.3V     |                 |
//            |              INT|             |                 |
//  Bariscan Kayaoglu
//  May 2013
//  Built with CCS Version 5.3.0.00090
//******************************************************************************
#include <msp430g2553.h>
#include <msp430.h>
#include <math.h>

#define	M_PI		3.14159265358979323846
#define PWM_PERIOD		127
#define ALPHA 0.98

#define MOTOR_FWD 0x80

#define KP 10.0
#define KI 0.0
#define KD 1.0
#define KV 0.0

volatile int TXByteCtr; // This counter used for counting bytes before the stop condition
volatile char xAcc_L;  // Variable to hold X_Low register value
volatile char xAcc_H;  // Variable to hold X_High register value
volatile char yAcc_L;  // Variable to hold Y_Low register value
volatile char yAcc_H;  // Variable to hold Y_High register value
volatile char zAcc_L;  // Variable to hold Z_Low register value
volatile char zAcc_H;  // Variable to hold Z_High register value
volatile char xGyr_L;  // Variable to hold X_Low register value
volatile char xGyr_H;  // Variable to hold X_High register value
volatile char yGyr_L;  // Variable to hold Y_Low register value
volatile char yGyr_H;  // Variable to hold Y_High register value
volatile char zGyr_L;  // Variable to hold Z_Low register value
volatile char zGyr_H;  // Variable to hold Z_High register value
volatile int i2c_State; // This counter held the state machine condition of I2C
volatile int Rx = 0;    // Flag for the interrupt vector. 1 for Receive 0 for Transmit
volatile int init = 0;  // Flag for the interrupt vector. 1 for Writing to gyro for config 0 for reading

volatile int L_CTRL, R_CTRL;
volatile int PWM;

//Accel/Gyro configuration addresses
const char CTRL1_XL = 0x10;   // Accel config register
const char XL_HP = 0x80; // Byte to write to accelerometer to config for 1.66kHz HP mode
const char CTRL2_G = 0x11;  // Gyro config register
const char G_HP = 0x80; // Byte to write for 1.66kHz HP mode gyro
const char CTRL7_G = 0x16; // Address
const char G_HPF = 0x68; // Byte to achieve 6 - 2.07 hz hpf band, 8 - hpf reset

//AccelGyro Memory addresses;
const char ACC_XOUT_H = 0x29;
const char ACC_XOUT_L = 0x28;
const char ACC_YOUT_H = 0x2B;
const char ACC_YOUT_L = 0x2A;
const char ACC_ZOUT_H = 0x2D;
const char ACC_ZOUT_L = 0x2C;
const char GYRO_XOUT_H = 0x23;
const char GYRO_XOUT_L = 0x22;
const char GYRO_YOUT_H = 0x25;
const char GYRO_YOUT_L = 0x24;
const char GYRO_ZOUT_H = 0x27;
const char GYRO_ZOUT_L = 0x26;
const char accAddress = 0x6B;

void init_PWM(void);
void init_I2C(void);
void Transmit(void);
void Receive(void);
void initUart(void);
void initGyro(void);

volatile float x_accel;
volatile float y_accel;
volatile float gyro_z;
volatile float cf;
volatile float tilt, tilt_old;
volatile float dt = 0.01;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1Mhz
  DCOCTL = CALDCO_1MHZ;
  //__delay_cycles(15000);                  // Wait gyro to wake up
  P1SEL |= BIT1 + BIT2 + BIT6 + BIT7;       // Assign I2C pins to USCI_B0 // Assign Uart pins to USCI_A0
  P1SEL2 |= BIT1 + BIT2 + BIT6 + BIT7;       // Assign I2C pins to USCI_B0 // Assign Uart pins to USCI_A0
  P1DIR |= BIT3;
  init_PWM();
  init_I2C();
  //initUart();
  initGyro();
  init = 0;                                 // Flag to reading from configuration writing

  while(1) {
      switch(i2c_State) {
          case 0: // Wait state
            //__delay_cycles(8000); // 10 ms
        	P1OUT &= ~BIT3;
            i2c_State++;
            break;
          case 1: // Read X_High state
            Rx = 0; // Receive flag = transmit
            TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
            Transmit();  // I2C start condition with Write operation

            Rx = 1; // Receive flag = receive
            Receive();  // I2C start condition with Read operation
            i2c_State++;
            break;
          case 2: // Read X_Low state
            Rx = 0; // Receive flag = transmit
            TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
            Transmit();  // I2C start condition with Write operation

            Rx = 1; // Receive flag = receive
            Receive();  // I2C start condition with Read operation
            i2c_State++;
            break;
          case 3: // Read Y_High state
            Rx = 0; // Receive flag = transmit
            TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
            Transmit();  // I2C start condition with Write operation

            Rx = 1; // Receive flag = receive
            Receive();  // I2C start condition with Read operation
            i2c_State++;
            break;
          case 4: // Read Y_Low state
            Rx = 0; // Receive flag = transmit
            TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
            Transmit();  // I2C start condition with Write operation

            Rx = 1; // Receive flag = receive
            Receive();  // I2C start condition with Read operation
            i2c_State++;
            break;
          case 5: // Read Z_High state
        	Rx = 0; // Receive flag = transmit
            TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
            Transmit();  // I2C start condition with Write operation

            Rx = 1; // Receive flag = receive
            Receive();  // I2C start condition with Read operation
            i2c_State++;
            break;
          case 6: // Read Z_Low state
            Rx = 0; // Receive flag = transmit
            TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
            Transmit();  // I2C start condition with Write operation

            Rx = 1; // Receive flag = receive
            Receive();  // I2C start condition with Read operation
            i2c_State++;
            break;
			case 7: // G_Read Z_High state
			Rx = 0; // Receive flag = transmit
			TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
			Transmit();  // I2C start condition with Write operation

			Rx = 1; // Receive flag = receive
			Receive();  // I2C start condition with Read operation
			i2c_State++;
			break;
			case 8: // G_Read Z_Low state
			Rx = 0; // Receive flag = transmit
			TXByteCtr = 1; // TXByteCtr 1 for sending register address and stop
			Transmit();  // I2C start condition with Write operation

			Rx = 1; // Receive flag = receive
			Receive();  // I2C start condition with Read operation
			i2c_State++;
		    break;
          case 9:
        	y_accel = ((2.0*(yAcc_H << 8 | yAcc_L))/32767);
        	x_accel = ((2.0*(xAcc_H << 8 | xAcc_L))/32767);
        	gyro_z  = ((245.0*(zGyr_H << 8| zGyr_L))/32767);

        	tilt = (-atan2f(y_accel, x_accel)*180.0)/M_PI;

        	cf = (ALPHA*(tilt_old + gyro_z*dt)) + (1 - ALPHA)*tilt;

        	PWM = (int) -(-KP*cf - KI*0 - KD*gyro_z - KV*0);
        	if(PWM >126)  PWM = 126;
        	if(PWM <-126) PWM = -126;

        	if(PWM >= 0) PWM = (127-PWM);
        	if(PWM < 0) PWM = -(127+PWM);

        	if(PWM >= 0) PWM |= 0x80; // FORWARD MOTORS
        	if(PWM < 0)  {PWM = PWM*-1; PWM &= ~0x80;} // REVERSE MOTORS

        	tilt_old = tilt;
            i2c_State++;
            break;
          case 10:
        	  L_CTRL = PWM;
        	  R_CTRL = PWM;
        	  if(L_CTRL&MOTOR_FWD){				// LEFT motor forward:
        	  			P2SEL &= ~BIT1;					//   Clear PWM for LIN1
        	  			P2OUT |= BIT1;					//   Set BIT1 out (LIN1) for slow decay
        	  			if(L_CTRL > 0) P2OUT |= BIT1;
        	  			else P2OUT &= ~BIT1;            // 	 Clear BIT1 so motors do not see fast (PWM=0, BIT1=1)
        	  			P2SEL |= BIT2; 					//   PWM for LIN2
        	  		}else if(L_CTRL){		// LEFT motor reverse:
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
        	  		TA1CCR1 = PWM&0x7F;	// LEFT motor duty cycle
        	  		TA1CCR2 = PWM&0x7F;  // RIGHT motor duty cycle
        	  		P1OUT |= BIT3;
        	i2c_State = 0;
        	__delay_cycles(100);
        	break;

      } // End of state machine
  }
} // End of Main


 void readX_H() {
      if(Rx == 1){                              // Master Recieve
          xAcc_H = UCB0RXBUF;                  // Get RXBuffer value to xAcc_H
      }

      else {                                     // Master Transmit
          if (TXByteCtr) {                       // TX byte counter = 1 : send gyro address
              UCB0TXBUF = ACC_XOUT_H;           // Load TX buffer // request from X_High register
              TXByteCtr--;                       // Decrement TX byte counter
          }
          else {                                      // Tx byte counter = 0 : register address sent
              UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
              IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
          }
      }
  }

  void readX_L() {
      if(Rx == 1){                              // Master Recieve
          xAcc_L = UCB0RXBUF;                  // Get RXBuffer value to xAcc_L
      }

      else {                                     // Master Transmit
          if (TXByteCtr) {                       // TX byte counter = 1 : send gyro address
              UCB0TXBUF = ACC_XOUT_L;           // Load TX buffer // request from X_Low register
              TXByteCtr--;                       // Decrement TX byte counter
          }
          else {                                      // Tx byte counter = 0 : register address sent
              UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
              IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
          }
      }
  }

  void readY_H() {
      if(Rx == 1) {                              // Master Recieve
          yAcc_H = UCB0RXBUF;                   // Get RXBuffer value to yAcc_H
      }

      else {                                         // Master Transmit
          if (TXByteCtr) {                           // TX byte counter = 1 : send gyro address
              UCB0TXBUF = ACC_YOUT_H;               // Load TX buffer // request from Y_High register
              TXByteCtr--;                           // Decrement TX byte counter
          }
          else {                                        // Tx byte counter = 0 : register address sent
              UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
              IFG2 &= ~UCB0TXIFG;                       // Clear USCI_B0 TX interrupt flag
          }
      }
  }

  void readY_L() {
      if(Rx == 1){                              // Master Recieve
          yAcc_L = UCB0RXBUF;                  // Get RXBuffer value to yAcc_L
      }

      else {                                     // Master Transmit
          if (TXByteCtr) {                       // TX byte counter = 1 : send gyro address
              UCB0TXBUF = ACC_YOUT_L;           // Load TX buffer // request from Y_Low register
              TXByteCtr--;                       // Decrement TX byte counter
          }
          else {                                      // Tx byte counter = 0 : register address sent
              UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
              IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
          }
      }
  }


  void readZ_H() {
      if(Rx == 1){                              // Master Recieve
          zAcc_H = UCB0RXBUF;                  // Get RXBuffer value to zAcc_H
      }

      else {                                     // Master Transmit
          if (TXByteCtr) {                       // TX byte counter = 1 : send gyro address
              UCB0TXBUF = ACC_ZOUT_H;           // Load TX buffer // request from Z_High register
              TXByteCtr--;                       // Decrement TX byte counter
          }
          else {                                      // Tx byte counter = 0 : register address sent
              UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
              IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
          }
      }
  }

  void readZ_L() {
      if(Rx == 1){                              // Master Recieve
          zAcc_L = UCB0RXBUF;                   // Get RXBuffer value to zAcc_L
      }

      else {                                     // Master Transmit
         if (TXByteCtr)  {                       // TX byte counter = 1 : send gyro address
             UCB0TXBUF = ACC_ZOUT_L;            // Load TX buffer // request from Z_Low register
             TXByteCtr--;                        // Decrement TX byte counter
         }
         else {                                       // Tx byte counter = 0 : register address sent
              UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
              IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
          }
      }
  }
   void GreadZ_H() {
       if(Rx == 1){                              // Master Recieve
           zGyr_H = UCB0RXBUF;                  // Get RXBuffer value to zAcc_H
       }

       else {                                     // Master Transmit
           if (TXByteCtr) {                       // TX byte counter = 1 : send gyro address
               UCB0TXBUF = GYRO_ZOUT_H;           // Load TX buffer // request from Z_High register
               TXByteCtr--;                       // Decrement TX byte counter
           }
           else {                                      // Tx byte counter = 0 : register address sent
               UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
               IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
           }
       }
   }

   void GreadZ_L() {
       if(Rx == 1){                              // Master Recieve
           zGyr_L = UCB0RXBUF;                   // Get RXBuffer value to zAcc_L
       }

       else {                                     // Master Transmit
          if (TXByteCtr)  {                       // TX byte counter = 1 : send gyro address
              UCB0TXBUF = GYRO_ZOUT_L;            // Load TX buffer // request from Z_Low register
              TXByteCtr--;                        // Decrement TX byte counter
          }
          else {                                       // Tx byte counter = 0 : register address sent
               UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
               IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX interrupt flag
           }
       }
   }

void initGyro(void) {
    TXByteCtr = 2;  // 2 bytes will occur. First register address send, Second Data send
    init = 1;       // config flag to 1 : Sending XL_Init
    Transmit();     // Start condition with Writing mode
    init = 2;       // config flag to 2 : Sending G_Init
    TXByteCtr = 2;  // Restart byte counter
    Transmit();		// Start condition with Writing mode
    init = 3;		// config flag to 3 : Sending more G_init
    TXByteCtr = 2; 	// Restart byte counter
    Transmit();		// Start condition with Writing mode
  }

void init_I2C(void) {
      UCB0CTL1 |= UCSWRST;                      // Enable SW reset
      UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
      UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
      UCB0BR0 = 12;                             // fSCL = 1Mhz/10 = ~100kHz
      UCB0BR1 = 0;
      UCB0I2CSA = accAddress;                   // Slave Address is 06Bh
      UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      IE2 |= UCB0RXIE + UCB0TXIE;               // Enable RX and TX interrupt
  }
void initUart(void) {
      UCA0CTL1 |= UCSSEL_2;                     // Use SMCLK
      UCA0BR0 = 104;                            // 1MHz 9600
      UCA0BR1 = 0;                              // 1MHz 9600
      UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
      UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  }
void Transmit(void){
      while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
      UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX with start condition
      __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
  }
void Receive(void){
        while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
        UCB0CTL1 &= ~UCTR ;                     // Clear I2C TX flag
        UCB0CTL1 |= UCTXSTT;                    // I2C start condition
        while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
        UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
        __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
  }

  void twosCompToDecimal() {

  }

#pragma vector = USCIAB0TX_VECTOR       // USCI TX interrupt vector. I2C module share RX and TX interrupts.
__interrupt void USCIAB0TX_ISR(void)    // This will occur either when RXBuffer is full or TXBuffer is sent.
{
    if(init == 1) {                 // Gyro writing for configuration. Sending LPF register address
        if (TXByteCtr == 2) {       // byte counter = 2 : sending register address
            UCB0TXBUF = CTRL1_XL;    // XL CTRL1 register address to TXBuffer
            TXByteCtr--;            // Decrement TX byte counter
        }
        else if(TXByteCtr == 1) {                                       // byte counter = 1 : sending config value
              UCB0TXBUF = XL_HP;     // config value to TXBuffer
              TXByteCtr--;                                              // Decrement TX byte counter
        }
        else {                                      // byte counter = 0 : stop condition
            UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
            IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
    }
    else if(init == 2) {                // Gyro writing for configuration. Sending Sample Rate Divider register address
        if (TXByteCtr == 2) {           // byte counter = 1 : sending config value
            UCB0TXBUF = CTRL2_G;     // Sample Rate Divider value to TXBuffer
            TXByteCtr--;                // Decrement TX byte counter
        }
        else if(TXByteCtr == 1) {           // byte counter = 1 : sending config value
            UCB0TXBUF = G_HP;                  // config value to TXBuffer
            TXByteCtr--;                    // Decrement TX byte counter
        }
        else {                                      // byte counter = 0 : stop condition
            UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
            IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
    }
    else if(init == 3) {                // Gyro writing for configuration. Sending Sample Rate Divider register address
            if (TXByteCtr == 2) {           // byte counter = 1 : sending config value
                UCB0TXBUF = CTRL7_G;     // Sample Rate Divider value to TXBuffer
                TXByteCtr--;                // Decrement TX byte counter
            }
            else if(TXByteCtr == 1) {           // byte counter = 1 : sending config value

                UCB0TXBUF = G_HPF;                  // config value to TXBuffer
                TXByteCtr--;                    // Decrement TX byte counter
            }
            else {                                      // byte counter = 0 : stop condition
                UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
                IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
                __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
            }
        }
    else {  // Gyro Reading operation.
      switch(i2c_State) {
          case 1: // Read X High
        	readX_H();
            __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
              break;
          case 2: // Read X Low
            readX_L();
            __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
              break;
          case 3: // Read Y High
            readY_H();
            __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
              break;
          case 4: // Read Y Low
            readY_L();
            __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
              break;
         case 5: // Read Z High
            readZ_H();
            __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
              break;
         case 6: // Read Z Low
            readZ_L();
            __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
              break;
		  case 7: // Read Z High
			 GreadZ_H();
			 __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
			   break;
		  case 8: // Read Z Low
			 GreadZ_L();
			 __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
			   break;
      }
  }

}

void init_PWM(void){
	P2DIR |= BIT1 + BIT2 + BIT4 + BIT5;			// Motor drive pins
	TA1CTL = TASSEL_2 + MC_1 + ID_2; 			// Source select: SMCLK (TASSEL_2)
													// Mode control: up mode (MC_1)
													// Input divider: /1 (ID_0)
	TA1CCR0 = PWM_PERIOD;						// PWM period
	TA1CCTL1 = OUTMOD_7;    					// Output mode 7 (reset/set)
	TA1CCTL2 = OUTMOD_7;
	TA1CCR1 = 0;								// Initialise 0% duty cycle
	TA1CCR2 = 0;
}
