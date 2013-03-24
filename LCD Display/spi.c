/*
 * Tritium MSP430 SPI interface - bit-bashed hardware interface
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
 * Last Modified: J.Kennedy, Tritium Pty Ltd, 17 January 2007
 *
 * - Implements the following SPI interface functions
 *	- init
 *	- transmit
 *	- exchange
 *
 */

// Include files
#include <msp430x41x.h>
#include "tri66.h"
#include "spi.h"

/*
 * Initialise SPI port
 *	- Grab I/O if necessary
 */
void spi_init( unsigned char clock )
{
	_NOP();
}

/*
 * Transmits data on SPI connection
 *	- Busy waits until entire shift is complete
 *	- On devices with hardware SPI support, this function is identical to spi_exchange,
 *	  with the execption of not returning a value
 *	- On devices with software (bit-bashed) SPI support, this function can run faster
 *	  because it does not require data reception code
 */
void spi_transmit( unsigned char data )
{
	unsigned char i;

	for(i = 0; i < 8; i++){
		// Output data bit
		if(data & 0x80) P1OUT |= CAN_MOSI;
		else P1OUT &= ~CAN_MOSI;
		data = data << 1;
		// Clock
		P1OUT |= CAN_SCLK;
		// Clock
		P1OUT &= ~CAN_SCLK;
	}
}

/*
 * Exchanges data on SPI connection
 *	- Busy waits until entire shift is complete
 *	- This function is safe to use to control hardware lines that rely on shifting being finalised
 */
unsigned char spi_exchange( unsigned char data )
{
	unsigned char i;
	unsigned char receive = 0x00;

	for(i = 0; i < 8; i++){
		// Output data bit
		if(data & 0x80) P1OUT |= CAN_MOSI;
		else P1OUT &= ~CAN_MOSI;
		data = data << 1;
		// Clock
		P1OUT |= CAN_SCLK;
		// Input data bit
		receive = receive << 1;
		if(P2IN & CAN_MISO) receive |= 0x01;
		// Clock
		P1OUT &= ~CAN_SCLK;
	}
	return(receive);
}
