/*
 * "Winamp" 8-bit driver module for Hitachi HD44780 based LCD displays.
 * The LCD is operated in it's 8 bit-mode to be connected to a single
 * PC parallel port.
 *
 * Copyright (c)  1999 Andrew McMeikan <andrewm@engineer.com>
 *		  modular driver 1999-2000 Benjamin Tse <blt@Comports.com>
 *
 *		  Modified July 2000 by Charles Steinkuehler for enhanced
 *		  performance and reduced CPU usage
 *		    - provided required setup time for RS valid to E high
 *		    - 1 uS call to uPause removed as it is unnecessary
 *
 *                2001 Joris Robijn <joris@robijn.net>
 *                  - Keypad support
 *
 * The connections are:
 * printer port   LCD
 * D0 (2)	  D0 (7)
 * D1 (3)	  D1 (8)
 * D2 (4)	  D2 (9)
 * D3 (5)	  D3 (10)
 * D4 (6)	  D4 (11)
 * D5 (7)	  D5 (12)
 * D6 (8)	  D6 (13)
 * D7 (9)	  D7 (14)
 * nSTRB (1)      EN (6)
 * nLF   (14)     nRW (5) (EN3 6 - LCD 3) (optional)
 * INIT  (16)     RS (4)
 * nSEL  (17)     EN2 (6 - LCD 2) (optional)
 *
 * Backlight
 * SEL  (17)	  backlight (optional, see documentation)
 *
 * Keypad connection (optional):
 * Some diodes and resistors are needed, see further documentation.
 * printer port   keypad
 * D0 (2)	  Y0
 * D1 (3)	  Y1
 * D2 (4)	  Y2
 * D3 (5)	  Y3
 * D4 (6)	  Y4
 * D5 (7)	  Y5
 * D6 (7)	  Y6
 * D7 (7)	  Y7
 * nLF    (14)    Y8
 * INIT   (16)    Y9
 * nSEL   (17)    Y10
 * nACK   (10)    X0
 * BUSY   (11)    X1
 * PAPEREND (12)  X2
 * SELIN  (13)    X3
 * nFAULT (15)    X4
 *
 * Created modular driver Dec 1999, Benjamin Tse <blt@Comports.com>
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

#include "hd44780-winamp.h"
#include "hd44780-low.h"
#include "lpt-port.h"

#include "port.h"
#include "shared/str.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>

// Generally, any function that accesses the LCD control lines needs to be
// implemented separately for each HW design. This is typically (but not
// restricted to):
// HD44780_senddata
// HD44780_readkeypad

void lcdwinamp_HD44780_senddata (PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch);
void lcdwinamp_HD44780_backlight (PrivateData *p, unsigned char state);
unsigned char lcdwinamp_HD44780_readkeypad (PrivateData *p, unsigned int YData);

#define EN1	STRB
#define EN2	SEL
#define EN3	LF
#define RW	LF
#define RS	INIT
#define BL	SEL

static const unsigned char EnMask[] = { EN1, EN2, EN3 };

// initialise the driver
int
hd_init_winamp (Driver *drvthis)
{
	PrivateData *p = (PrivateData*) drvthis->private_data;
	HD44780_functions *hd44780_functions = p->hd44780_functions;

	// Reserve the port registers
	port_access(p->port);
	port_access(p->port+1);
	port_access(p->port+2);

	hd44780_functions->senddata = lcdwinamp_HD44780_senddata;
	hd44780_functions->backlight = lcdwinamp_HD44780_backlight;
	hd44780_functions->readkeypad = lcdwinamp_HD44780_readkeypad;

	// setup the lcd in 8 bit mode
	hd44780_functions->senddata (p, 0, RS_INSTR, FUNCSET | IF_8BIT);
	hd44780_functions->uPause (p, 4100);
	hd44780_functions->senddata (p, 0, RS_INSTR, FUNCSET | IF_8BIT);
	hd44780_functions->uPause (p, 100);
	hd44780_functions->senddata (p, 0, RS_INSTR, FUNCSET | IF_8BIT | TWOLINE | SMALLCHAR);
	hd44780_functions->uPause (p, 40);

	common_init (p);

	if (p->have_keypad) {
		// Remember which input lines are stuck
		p->stuckinputs = lcdwinamp_HD44780_readkeypad (p, 0);
	}

	return 0;
}

// lcdwinamp_HD44780_senddata
void
lcdwinamp_HD44780_senddata (PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch)
{
	unsigned char enableLines = 0, portControl;

	if (flags == RS_DATA)
		portControl = RS;
	else
		portControl = 0;

	portControl |= p->backlight_bit;

	if (displayID == 0)
		enableLines = EnMask[0] | EnMask[1] | ((p->extIF) ? EnMask[2] : 0);
	else
		enableLines = EnMask[displayID - 1];

	// 40 nS setup time for RS valid to EN high, so set RS
	port_out (p->port + 2, portControl ^ OUTMASK);

	// Output the actual data
	port_out (p->port, ch);

	if( p->delayBus ) p->hd44780_functions->uPause (p, 1);

	// then set EN high
	port_out (p->port + 2, (enableLines|portControl) ^ OUTMASK);

	if( p->delayBus ) p->hd44780_functions->uPause (p, 1);

	// 80 nS setup from valid data to EN low will be met without any delay
	// unless you are running a REALLY FAST ISA bus (like 75 MHZ!)
	// 230 nS minimum E high time provided by ISA bus delays as well...

	// ABOVE TEXT ignored now, using delays if delayBus is specified

	// Set EN low and we're done...
	port_out (p->port + 2, portControl ^ OUTMASK);

	// 10 nS data hold time provided by the length of ISA write for EN
}

void lcdwinamp_HD44780_backlight (PrivateData *p, unsigned char state)
{
	p->backlight_bit = (state?0:nSEL);

	port_out (p->port + 2, p->backlight_bit ^ OUTMASK);
}

unsigned char lcdwinamp_HD44780_readkeypad (PrivateData *p, unsigned int YData)
{
	unsigned char readval;

	// 8 bits output
	// Convert the positive logic to the negative logic on the LPT port
	port_out (p->port, ~YData & 0x00FF );

	if( p->delayBus ) p->hd44780_functions->uPause (p, 1);

	// Read inputs
	readval = ~ port_in (p->port + 1) ^ INMASK;

	// Set output back to idle state for backlight
	port_out (p->port + 2, p->backlight_bit ^ OUTMASK );

	// And convert value back.
	return ( (readval >> 4 & 0x03) | (readval >> 5 & 0x04) | (readval >> 3 & 0x08) | (readval << 1 & 0x10) ) & ~p->stuckinputs;
}
