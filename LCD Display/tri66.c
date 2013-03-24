/*
 * Tritium 3.5 digit multifunction CAN bus LCD display firmware
 * Copyright (c) 2007, Tritium Pty Ltd.  All rights reserved.
 *  
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products 
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * Last modified: J.Kennedy, Tritium Pty Ltd, 2 October 2007
 *
 *	- For TRI66v1 PCB with MSP430F413 microcontroller & MCP2515 CAN controller
 *	- Normal state is CPU idle waiting for interrupts
 *	- Interrupt from switch (Port 2) triggers:
 *		- Decide what value we want to display now
 *		- Indicate value by moving to next LED on front panel
 *		- Pull stored value from memory and:
 *			- Scale it as appropriate
 *			- Convert from 32-bit IEEE float to 16-bit integer
 *			- Choose where to put the decimal places
 *			- Update the LCD
 *	- Interrupt from MCP2515 (Port 1) triggers:
 *		- We've received a packet with one of the four selectable data types
 *		- Save it to memory
 *		- If it's the one we're currently displaying:
 *			- Scale it as appropriate
 *			- Convert from 32-bit IEEE float to 16-bit integer
 *			- Choose where to put the decimal places
 *			- Update the LCD
 *
 *	- Version 02
 *		- Added ability to select average or multiplication for both values in a CAN packet
 *			- AVERAGE both Phase A and Phase B motor Current values together
 *			- MULTIPLY Bus Current and Bus Voltage to give Bus Power readout
 *		- Added explanatory text in tri66.h for packet data types
 *
 *	- Version 03
 *		- Added ability to select conversion to degrees F for values in a CAN packet
 */

// Include files
#include <msp430x41x.h>
#include <signal.h>
#include "tri66.h"
#include "spi.h"
#include "can.h"

// Function prototypes
void clock_init( void );
void io_init( void );
void lcd_init( void );
void timerA_init( void );
void display( int value );
void display_dp( unsigned char position );

// Global variables
volatile unsigned char button_flag = FALSE;
volatile unsigned int debounce_timer = 0;
const unsigned char digit[44] = {
	0x10, 0x11,		// "0" Lower half LCD segments: E + DP, C + D
	0x11, 0x01,		// "0" Upper half LCD segments: A + B, G + F
	0x00, 0x10,		// "1"
	0x01, 0x00,		// "1"
	0x10, 0x01,		// "2"
	0x11, 0x10,		// "2"
	0x00, 0x11,		// "3"
	0x11, 0x10,		// "3"
	0x00, 0x10,		// "4"
	0x01, 0x11,		// "4"
	0x00, 0x11,		// "5"
	0x10, 0x11,		// "5"
	0x10, 0x11,		// "6"
	0x10, 0x11,		// "6"
	0x00, 0x10,		// "7"
	0x11, 0x01,		// "7"
	0x10, 0x11,		// "8"
	0x11, 0x11,		// "8"
	0x00, 0x11,		// "9"
	0x11, 0x11,		// "9"
	0x00, 0x00,		// "-"
	0x00, 0x10,		// "-"
};

/*
 * Main routine
 *
 */
int main( void )
{ 
	// Local variables
	unsigned char state = 0;
	unsigned char update_flag = FALSE;
	long delay;
	float can_a_value = 0.0, can_b_value = 0.0, can_c_value = 0.0, can_d_value = 0.0;

	// Stop watchdog timer
	WDTCTL = WDTPW + WDTHOLD;
	
	// Initialise I/O ports
	io_init();
	
	// Initialise clock
	clock_init();
	
	// Initialise SPI port to CAN controller
	spi_init(0);
	
	// Wait for CAN controller powerup
	for(delay=0; delay<100000; delay++);
	
	// Initialise CAN controller
	can_init();

	// Initialise LCD display
	lcd_init();
	
	// Initialise Timer A (timing tick generation)
	timerA_init();

	// Set up switch interrupt
	P2IES = nSWITCH;						// Set High -> Low edge trip on nSWITCH pin
	P2IFG &= ~nSWITCH;						// Make sure IRQ flag is clear
	P2IE = nSWITCH;							// Enable pin interrupt

	// Enable global interrupts
	eint();
	
	// Wait in main loop (idle, if possible)
	while( TRUE ){
		// Check for pushbutton events
		if(button_flag == TRUE){
			// Clear flag
			button_flag = FALSE;
			// Begin pushbutton debounce time
			debounce_timer = DEBOUNCE_TIME;
			// Move to next display variable
			state++;
			if( state == 4) state = 0;
			// Schedule a display update
			update_flag = TRUE;
		}
		
		// Check for CAN receive events
		if((P1IN & CAN_nINT) == 0x00){
			// Get the CAN packet
			can_receive();
			// Check the status
			if(can.status == CAN_OK){
				// Update the internal display values with new data
				if(can.address == CAN_A){
					switch(CAN_A_TYPE){
						case LOW:
							can_a_value = can.data.data_fp[0];
							break;
						case HIGH:
							can_a_value = can.data.data_fp[1];
							break;
						case MULTIPLY:
							can_a_value = (can.data.data_fp[0] * can.data.data_fp[1]);
							break;
						case AVERAGE:
							can_a_value = (can.data.data_fp[0] + can.data.data_fp[1]) / 2.0;
							break;
						case LOW_TO_F:
							can_a_value = (can.data.data_fp[0] * 1.8) + 32.0;
							break;
						case HIGH_TO_F:
							can_a_value = (can.data.data_fp[1] * 1.8) + 32.0;
							break;
					}
				}
				if(can.address == CAN_B){
					switch(CAN_B_TYPE){
						case LOW:
							can_b_value = can.data.data_fp[0];
							break;
						case HIGH:
							can_b_value = can.data.data_fp[1];
							break;
						case MULTIPLY:
							can_b_value = (can.data.data_fp[0] * can.data.data_fp[1]);
							break;
						case AVERAGE:
							can_b_value = (can.data.data_fp[0] + can.data.data_fp[1]) / 2.0;
							break;
						case LOW_TO_F:
							can_b_value = (can.data.data_fp[0] * 1.8) + 32.0;
							break;
						case HIGH_TO_F:
							can_b_value = (can.data.data_fp[1] * 1.8) + 32.0;
							break;
					}
				}
				if(can.address == CAN_C){
					switch(CAN_C_TYPE){
						case LOW:
							can_c_value = can.data.data_fp[0];
							break;
						case HIGH:
							can_c_value = can.data.data_fp[1];
							break;
						case MULTIPLY:
							can_c_value = (can.data.data_fp[0] * can.data.data_fp[1]);
							break;
						case AVERAGE:
							can_c_value = (can.data.data_fp[0] + can.data.data_fp[1]) / 2.0;
							break;
						case LOW_TO_F:
							can_c_value = (can.data.data_fp[0] * 1.8) + 32.0;
							break;
						case HIGH_TO_F:
							can_c_value = (can.data.data_fp[1] * 1.8) + 32.0;
							break;
					}
				}
				if(can.address == CAN_D){
					switch(CAN_D_TYPE){
						case LOW:
							can_d_value = can.data.data_fp[0];
							break;
						case HIGH:
							can_d_value = can.data.data_fp[1];
							break;
						case MULTIPLY:
							can_d_value = (can.data.data_fp[0] * can.data.data_fp[1]);
							break;
						case AVERAGE:
							can_d_value = (can.data.data_fp[0] + can.data.data_fp[1]) / 2.0;
							break;
						case LOW_TO_F:
							can_d_value = (can.data.data_fp[0] * 1.8) + 32.0;
							break;
						case HIGH_TO_F:
							can_d_value = (can.data.data_fp[1] * 1.8) + 32.0;
							break;
					}
				}
				// Schedule a display update
				update_flag = TRUE;	
			}
			if(can.status == CAN_RTR){
				// Do nothing...
			}
			if(can.status == CAN_ERROR){
				// Do nothing...
			}
		}

		// Display a value
		if( update_flag == TRUE ){
			// Clear flag
			update_flag = FALSE;
			// Show current state & update LCD
			P1OUT |= nLED1 | nLED2 | nLED3 | nLED4;
			switch( state ){
				case 0:
					display( (int)(can_a_value * CAN_A_SCALE) );
					display_dp( CAN_A_DP );
					P1OUT &= ~nLED1;
					break;
				case 1:
					display( (int)(can_b_value * CAN_B_SCALE) );
					display_dp( CAN_B_DP );
					P1OUT &= ~nLED2;
					break;
				case 2:
					display( (int)(can_c_value * CAN_C_SCALE) );
					display_dp( CAN_C_DP );
					P1OUT &= ~nLED3;
					break;
				case 3:
					display( (int)(can_d_value * CAN_D_SCALE) );
					display_dp( CAN_D_DP );
					P1OUT &= ~nLED4;
					break;
				default:
					break;
			}
		}
	}		
	
	// Will never get here, keeps the compiler happy
	return(1);
}

/*
 * Port 2 interrupt service routine
 *	- Triggers on falling edge of nSWITCH pin
 *	- Signals the main loop that a button push has occurred
 *	- Locks itself out of further interrupts to avoid switch bounce issues
 */
interrupt(PORT2_VECTOR) port2_isr(void)
{
	// Clear the flag
	P2IFG &= ~nSWITCH;
	// Disable further interrupts on this pin
	P2IE &= ~nSWITCH;
	// Set the semaphore for the main loop
	button_flag = TRUE;
}

/*
 * Timer A interrupt service routine
 *	- Triggers on CCR0 match at approx 100 Hz
 *	- Counts down debounce time for pushbutton
 *		- If debounced, re-enable pushbutton pin edge IRQ
 */
interrupt(TIMERA0_VECTOR) timera_isr(void)
{
	if(debounce_timer > 0){
		debounce_timer--;
		if(debounce_timer == 0){
			P2IFG &= ~nSWITCH;					// Make sure IRQ flag is clear
			P2IE = nSWITCH;						// Enable pin interrupt		
		}
	}
}

/*
 * Initialise DCO & FLL circuits
 *	- Sets internal DCO running at 4.91 MHz with auto-calibration by FLL+
 *	- ACLK: 32768 Hz
 *	- MCLK = SMCLK = DCO = (74+1)*2*ACLK = 4.915200 MHz
 *
 */
void clock_init( void )
{
	SCFI0 |= FN_3;								// FLL+ loop divider = /2, DCO range
	SCFQCTL = 74;								// FLL+ multiplier = 74+1
	FLL_CTL0 = DCOPLUS | XCAP18PF;				// Enable FLL+ loop divider, set XT load caps
}

/*
 * Initialise I/O port directions and states
 *	- Drive unused pins as outputs to avoid floating inputs
 *
 */
void io_init( void )
{
	P1OUT = nLED1 | nLED2 | nLED3 | nLED4 | CAN_nCS;
	P1DIR = nLED1 | nLED2 | nLED3 | nLED4 | CAN_MOSI | CAN_SCLK | CAN_nCS | P1_UNUSED;
	
	P2OUT = 0x00;
	P2DIR = G3 | F3 | A3 | B3 | G2 | F2 | P2_UNUSED;
	
	P3OUT = 0x00;
	P3DIR = A2 | B2 | G1 | F1 | A1 | B1 | C1 | D1 | P3_UNUSED;
	
	P4OUT = 0x00;
	P4DIR = E1 | DP2 | C2 | D2 | E2 | DP3 | C3 | D3 | P4_UNUSED;
	
	P5OUT = 0x00;
	P5DIR = E3 | BC4 | P5_UNUSED;
	
	P6OUT = 0x00;
	P6DIR = COLON | PLUS | MINUS | DP4 | P6_UNUSED;
}

/*
 * Initialise LCD segment controller
 *	- All segments on
 */
void lcd_init( void )
{
	// Init segments
	LCDM1  = 0xFF;
	LCDM2  = 0xFF;
	LCDM3  = 0xFF;
	LCDM4  = 0xFF;
	LCDM5  = 0xFF;
	LCDM6  = 0xFF;
	LCDM7  = 0xFF;
	LCDM8  = 0xFF;
	LCDM9  = 0xFF;
	LCDM10 = 0xFF;
	LCDM11 = 0xFF;
	LCDM12 = 0xFF;

	// Initialise LCD driver & basic timer peripheral
	LCDCTL = 0x65;								// Static mode, segments 0-23
	BTCTL = BTFRFQ1 + BTFRFQ0;					// fLCD = ACLK / 256;
}

/*
 * Initialise Timer A for regular timing tick interrupts
 *	- Interrupt at approximately 100Hz (10ms)
 *
 */
void timerA_init( void )
{
	CCTL0 = CCIE;								// Enable CCR0 interrupt
	CCR0 = (INPUT_CLOCK / TICK_RATE);			// Set timebase
	TACTL = TASSEL_1 | MC_1;					// ACLK/1, up count mode
}

/*
 * display subroutine
 *
 * Displays a signed integer on the 3.5 digit 7 segment LCD screen
 * Numbers higher than 1999 and lower than -1999 are truncated to these values
 *
 */
void display( int value )
{
	// Local variables
	unsigned int dig_ptr;
	unsigned int hundreds;

	// Check range of value
	if( value > 1999 ) value = 1999;
	if( value < -1999 ) value = -1999;
	
	// Display 'negative' - converts value to absolute integer
	if( value < 0 ){
		P6OUT |= MINUS;
		value = 0 - value;
	}
	else{
		P6OUT &= ~MINUS;
	}

	// Display 'ones' - digit 1
	dig_ptr = 4 * ( value % 10 );
	value /= 10;
	LCDM5  = digit[dig_ptr++];	
	LCDM6  = digit[dig_ptr++];	
	LCDM7  = digit[dig_ptr++];	
	LCDM8  = digit[dig_ptr];	
	
	// Display 'tens' - digit 2
	dig_ptr = 4 * ( value % 10 );
	value /= 10;
	LCDM3  = digit[dig_ptr++];	
	LCDM4  = digit[dig_ptr++];	
	LCDM9  = digit[dig_ptr++];	
	LCDM10 = digit[dig_ptr];	
	
	// Display 'hundreds' - digit 3
	hundreds = value % 10;
	dig_ptr = 4 * hundreds;
	value /= 10;
	LCDM1  = digit[dig_ptr++];	
	LCDM2  = digit[dig_ptr++];	
	LCDM11 = digit[dig_ptr++];	
	LCDM12 = digit[dig_ptr];	
	
	// Display 'thousands' - digit 4
	if( value == 1)	LCDM1 |= 0x01;
	else{
		LCDM1 &= ~0x01;
		// Blank out 'hundreds' if zero also
		if( hundreds == 0 ){
			LCDM1 = 0;
			LCDM2 = 0;
			LCDM11 = 0;
			LCDM12 = 0;
		}		
	}
}

/*
 * display_dp
 *	- Turns on the dot point as specified by the position
 *	- Position 0 = All off
 *	- Position 1 = RH dot point ( between 1's and 10's )
 *	- Position 2 = Middle dot point ( between 10's and 100's )
 *	- Position 3 = LH dot point ( between 100's and 1000's )
 *	- Position 4 = Colon ( between 10's and 100's )
 *	- Others: All off
 */
void display_dp( unsigned char position )
{
	LCDM5 &= ~0x01;
	LCDM3 &= ~0x01;
	P6OUT &= ~DP4;
	P6OUT &= ~COLON;
	switch( position ){
		case 1:
			LCDM5 |= 0x01;
			break;
		case 2:
			LCDM3 |= 0x01;
			break;
		case 3:
			P6OUT |= DP4;
			break;
		case 4:
			P6OUT |= COLON;
			break;
		default:
			break;
	}
}
