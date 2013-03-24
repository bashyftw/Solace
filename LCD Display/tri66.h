/*
 * TRI66 3.5 digit multifunction CAN bus LCD display 
 * (c)2007 Tritium Pty Ltd.  All rights reserved.
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
 * Last Modified: J.Kennedy, Tritium Pty Ltd, 2 October 2007
 *
 */

// LCD segments
//
//    AAAA
//   F    B
//   F    B
//    GGGG
//   E    C
//   E    C
//    DDDD   DP
//
//
// Pin Definitions
// Port 1
#define nLED1	 		0x01
#define nLED2	 		0x02
#define nLED3	 		0x04
#define nLED4	 		0x08
#define CAN_nCS 		0x10
#define CAN_nINT 		0x20
#define CAN_SCLK 		0x40
#define CAN_MOSI		0x80
#define P1_UNUSED		0x00

// Port 2
#define CAN_MISO		0x01
#define nSWITCH			0x02
#define G3				0x04
#define F3				0x08
#define A3				0x10
#define B3				0x20
#define G2				0x40
#define F2				0x80
#define P2_UNUSED		0x00

// Port 3
#define A2				0x01
#define B2				0x02
#define G1				0x04
#define F1				0x08
#define A1				0x10
#define B1				0x20
#define C1				0x40
#define D1				0x80
#define P3_UNUSED		0x00

// Port 4
#define E1				0x01
#define DP2				0x02
#define C2				0x04
#define D2				0x08
#define E2				0x10
#define DP3				0x20
#define C3				0x40
#define D3				0x80
#define P4_UNUSED		0x00

// Port 5
#define E3				0x01
#define BC4				0x02
#define P5_UNUSED		0x04 | 0x08 | 0x10 | 0x20 | 0x40 | 0x80

// Port 6
#define COLON			0x10
#define PLUS			0x20
#define MINUS			0x40
#define DP4				0x80
#define P6_UNUSED		0x01 | 0x02 | 0x04 | 0x08

// Constant Definitions
#define	TRUE			1
#define FALSE			0
#define PUSHED			1
#define RELEASED		0
#define LOW				0
#define HIGH			1
#define MULTIPLY		2
#define AVERAGE			3
#define LOW_TO_F		4
#define HIGH_TO_F		5

// Event timing
#define INPUT_CLOCK		32768				// Hz
#define TICK_RATE		100					// Hz
#define DEBOUNCE_TIME	5					// Ticks

// Motor controller CAN base address and packet offsets
#define	MC_CAN_BASE		0x400		// High = Serial Number             Low = "TRIa" string
#define MC_LIMITS		0x01		// High = Active Motor              Low = Error & Limit flags
#define	MC_BUS			0x02		// High = Bus Current               Low = Bus Voltage
#define MC_VELOCITY		0x03		// High = Velocity (m/s)            Low = Velocity (rpm)
#define MC_PHASE		0x04		// High = Phase A Current           Low = Phase B Current
#define MC_V_VECTOR		0x05		// High = Vd vector                 Low = Vq vector
#define MC_I_VECTOR		0x06		// High = Id vector                 Low = Iq vector
#define MC_BEMF_VECTOR	0x07		// High = BEMFd vector              Low = BEMFq vector
#define MC_RAIL1		0x08		// High = 15V                       Low = 1.65V
#define MC_RAIL2		0x09		// High = 2.5V                      Low = 1.2V
#define MC_FAN			0x0A		// High = Fan speed (rpm)           Low = Fan drive (%)
#define MC_TEMP1		0x0B		// High = Heatsink Temp             Low = Motor Temp
#define MC_TEMP2		0x0C		// High = Inlet Temp                Low = CPU Temp
#define MC_TEMP3		0x0D		// High = Outlet Temp               Low = Capacitor Temp
#define MC_CUMULATIVE	0x0E		// High = DC Bus AmpHours           Low = Odometer

// Driver controls CAN base address and packet offsets
#define DC_CAN_BASE		0x500		// High = Serial Number             Low = "TRIb" string
#define DC_DRIVE		0x01		// High = Motor Current Setpoint    Low = Motor Velocity Setpoint
#define DC_POWER		0x02		// High = Bus Current Setpoint      Low = Unused
#define DC_RESET		0x03		// High = Unused                    Low = Unused
#define DC_SWITCH		0x04		// High = Switch position           Low = Switch state change

// Define the four packet types we wish to observe with this display.  Set the following:
//  - CAN packet address
//	- The position of the data within that packet (high word/low word/average/multiply/to °F)
//	- The scale factor to apply to the data (floating-point)
//	- The decimal point position to use on the LCD

// CAN A - Motor current setpoint (percent)
#define CAN_A			(DC_CAN_BASE + DC_DRIVE)
#define CAN_A_TYPE		HIGH
#define CAN_A_SCALE		1000.0
#define CAN_A_DP		1

// CAN B - Bus Current (Amps)
#define CAN_B			(MC_CAN_BASE + MC_BUS)
#define CAN_B_TYPE		HIGH
#define CAN_B_SCALE		10.0
#define CAN_B_DP		1

// CAN C - Bus Voltage (Volts)
#define CAN_C			(MC_CAN_BASE + MC_BUS)
#define CAN_C_TYPE		LOW
#define CAN_C_SCALE		10.0
#define CAN_C_DP		1

// CAN D - Vehicle Velocity (km/h)
#define CAN_D			(MC_CAN_BASE + MC_VELOCITY)
#define CAN_D_TYPE		HIGH
#define CAN_D_SCALE		36.0
#define CAN_D_DP		1

// Typedefs for quickly joining multiple bytes/ints/etc into larger values
// These rely on byte ordering in CPU & memory - i.e. they're not portable across architectures
typedef union _group_64 {
	float data_fp[2];
	unsigned char data_u8[8];
	unsigned int data_u16[4];
	unsigned long data_u32[2];
} group_64;

typedef union _group_32 {
	float data_fp;
	unsigned char data_u8[4];
	unsigned int data_u16[2];
	unsigned long data_u32;
} group_32;

typedef union _group_16 {
	unsigned char data_u8[2];
	unsigned int data_u16;
} group_16;
