/*  This is the LCDproc driver for Matrix Orbital devices (http://www.matrixorbital.com)

    For the Matrix Orbital LCD* LKD* VFD* and VKD* displays

    NOTE: GLK displays have a different driver.

    Copyright (C) 1999, William Ferrell and Scott Scriven
		  2001, André Breiler
		  2001, Philip Pokorny
		  2001, David Douthitt
		  2001, David Glaude
		  2001, Joris Robijn
		  2001, Rene Wagner

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <ctype.h>
#include <sys/poll.h>

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include "render.h"
#include "lcd.h"
#include "lcd_lib.h"
#include "MtxOrb.h"
#include "drv_base.h"
#include "input.h"

#include "shared/report.h"
#include "shared/str.h"
#include "configfile.h"

#define IS_LCD_DISPLAY	(MtxOrb_type == MTXORB_LCD)
#define IS_LKD_DISPLAY	(MtxOrb_type == MTXORB_LKD)
#define IS_VFD_DISPLAY	(MtxOrb_type == MTXORB_VFD)
#define IS_VKD_DISPLAY	(MtxOrb_type == MTXORB_VKD)

#define NUM_CUSTOM_CHARS 8
#define CUSTOM_NOT_FOUND (NUM_CUSTOM_CHARS)

#define NotEnoughArgs (i + 1 > argc)

/* NOTE: This does not appear to make use of the
 *       hbar and vbar functions present in the LKD202-25.
 *       Why I do not know.
 * RESP: Because software emulated hbar/vbar permit simultaneous use.
 */

typedef enum {
	normal,
	hbar,
	vbar,
	bign
} custom_type;
#define KNOWN_CHARS 26

#define DIRTY_CHAR 254
#define START_ICON 22
typedef enum {
	baru1 = 0,
	baru2 = 1,
	baru3 = 2,
	baru4 = 3,
	baru5 = 4,
	baru6 = 5,
	baru7 = 6,
	bard1 = 7,
	bard2 = 8,
	bard3 = 9,
	bard4 = 10,
	bard5 = 11,
	bard6 = 12,
	bard7 = 13,
	barr1 = 14,
	barr2 = 15,
	barr3 = 16,
	barr4 = 17,
	barl1 = 18,
	barl2 = 19,
	barl3 = 20,
	barl4 = 21,
	empty_heart = 22,
	filled_heart = 23,
	ellipsis = 24,
	barb = 25,
	barw = 32
} bar_type;

#define IS_BARU(x) (((x) == baru1) || ((x) == baru2) || ((x) == baru3) || \
		            ((x) == baru4) || ((x) == baru5) || ((x) == baru6) || \
					((x) == baru7) || ((x) == barb))
#define IS_BARD(x) (((x) == bard1) || ((x) == bard2) || ((x) == bard3) || \
		            ((x) == bard4) || ((x) == bard5) || ((x) == bard6) || \
					((x) == bard7) || ((x) == barb))
#define IS_BARL(x) (((x) == barl1) || ((x) == barl2) || ((x) == barl3) || \
					((x) == barl4))
#define IS_BARR(x) (((x) == barr1) || ((x) == barr2) || ((x) == barr3) || \
					((x) == barr4))


static unsigned char *lcd_contains; // what we think is actually on the LCD
static int custom = normal;
static enum {MTXORB_LCD, MTXORB_LKD, MTXORB_VFD, MTXORB_VKD} MtxOrb_type;
static int fd;
static int backlightenabled = MTXORB_DEF_BACKLIGHT;
static int uselinewrap = 1;
static int linewrapenabled = MTXORB_DEF_LINEWRAP;

static char pause_key = MTXORB_DEF_PAUSE_KEY, back_key = MTXORB_DEF_BACK_KEY;
static char forward_key = MTXORB_DEF_FORWARD_KEY, main_menu_key = MTXORB_DEF_MAIN_MENU_KEY;
static int keypad_test_mode = 0;
static int barb_is_255 = 1;

static int def[NUM_CUSTOM_CHARS] = { -1, -1, -1, -1, -1, -1, -1, -1 };
static int use[NUM_CUSTOM_CHARS];
static int sent[NUM_CUSTOM_CHARS];
static int defCount[KNOWN_CHARS];
static void sendOutCustomChars(void);

static void MtxOrb_linewrap (int on);
static void MtxOrb_autoscroll (int on);
static void MtxOrb_cursorblink (int on);
static void MtxOrb_string (int x, int y, char *string);
static void reassign(int pos);
static int approx_match(int type);

/* 
 * This does not belong to MtxOrb.h unless used externally
 * Having these defines here reduces the number of warnings.
 */ 
static void MtxOrb_clear ();
static void MtxOrb_close ();
static void MtxOrb_flush ();
static void MtxOrb_flush_box (int lft, int top, int rgt, int bot);
static void MtxOrb_chr (int x, int y, char c);
static int MtxOrb_contrast (int contrast);
static void MtxOrb_backlight (int on);
static void MtxOrb_output (int on);
static void MtxOrb_init_vbar ();
static void MtxOrb_init_hbar ();
static void MtxOrb_vbar (int x, int len);
static void MtxOrb_hbar (int x, int y, int len);
static void MtxOrb_init_num ();
static void MtxOrb_num (int x, int num);
static void MtxOrb_set_char (int n, char *dat);
static void MtxOrb_icon (int which, char dest);
static void MtxOrb_draw_frame (char *dat);
static char MtxOrb_getkey ();
static char * MtxOrb_getinfo ();
static void MtxOrb_heartbeat (int type);
static int MtxOrb_ask_bar (int type, int priority);
static void MtxOrb_set_known_char (int car, int type);
/*
 * End of what was in MtxOrb.h
 */

/* Parse one key from the configfile */
static char MtxOrb_parse_keypad_setting (char * sectionname, char * keyname, char default_value)
{
	char return_val = 0;
	char * s;

	s = config_get_string ( sectionname, keyname, 0, NULL);
	if (s != NULL){
		return_val = *s;
	} else {
		return_val=default_value;
	}
	return return_val;
}

/* Very private function that clear internal definition.*/
static void
MtxOrb_clear_custom ()
{
	int pos;

	/* The whole of the custom chars are being used (i.e. by bignum).
	 * so reset custom char settings.
	 */

	for (pos = 0; pos < NUM_CUSTOM_CHARS; pos++) 
	{
		if (use[pos])
		{
			// If a character is in use since the last clear
			// then reassign it (i.e. use an approx char for it)
			reassign(pos);
		}
		def[pos] = -1;		/* Character is not defined.*/
		use[pos] = 0;		/* Character is not in use.*/
		sent[pos]=0;        
	}

}

lcd_logical_driver *MtxOrb;	/* set by MtxOrb_init() */

/*********************************************************************
 * init() should set up any device-specific stuff, and
 * point all the function pointers.
 */
int
MtxOrb_init (lcd_logical_driver * driver, char *args)
{
	struct termios portset;

	int contrast = MTXORB_DEF_CONTRAST;
	char device[256] = MTXORB_DEF_DEVICE;
	int speed = MTXORB_DEF_SPEED;
	char *size;
	char *type;
	int tmp, w, h;

	MtxOrb_type = MTXORB_LKD;  // Assume it's an LCD w/keypad

	MtxOrb = driver;

	debug( RPT_INFO, "MtxOrb: init(%p,%s)", driver, args );

	/* TODO: replace DriverName with driver->name when that field exists. */
	#define DriverName "MtxOrb"


	/* READ CONFIG FILE */

	/* Get serial device to use */
	strncpy(device, config_get_string ( DriverName , "device" , 0 , MTXORB_DEF_DEVICE),sizeof(device));
	device[sizeof(device)-1]=0;
	report (RPT_INFO,"MtxOrb: Using device: %s", device);

	/* Get display size */
	size=config_get_string ( DriverName , "size" , 0 , MTXORB_DEF_SIZE);
	if( sscanf(size , "%dx%d", &w, &h ) != 2
	|| (w <= 0) || (w > LCD_MAX_WIDTH)
	|| (h <= 0) || (h > LCD_MAX_HEIGHT)) {
		report (RPT_WARNING, "MtxOrb: Cannot read size: %s. Using default value %s.", size, MTXORB_DEF_SIZE);
		sscanf( MTXORB_DEF_SIZE , "%dx%d", &w, &h );
	}
	driver->wid = w;
	driver->hgt = h;

	/* Get contrast */
	if (0<=config_get_int ( DriverName , "Contrast" , 0 , MTXORB_DEF_CONTRAST) && config_get_int ( DriverName , "Contrast" , 0 , MTXORB_DEF_CONTRAST) <= 255) {
		contrast = config_get_int ( DriverName , "Contrast" , 0 , MTXORB_DEF_CONTRAST);
	} else {
		report (RPT_WARNING, "MtxOrb: Contrast must be between 0 and 255. Using default value.");
	}

	/* Get speed */
	tmp = config_get_int ( DriverName , "Speed" , 0 , MTXORB_DEF_SPEED);

	switch (tmp) {
		case 1200:
			speed = B1200;
			break;
		case 2400:
			speed = B2400;
			break;
		case 9600:
			speed = B9600;
			break;
		case 19200:
			speed = B19200;
			break;
		default:
			{
				int baud=0;
				speed = MTXORB_DEF_SPEED;
				switch (speed) {
					case B1200:
						baud=1200;
						break;
					case B2400:
						baud=2400;
						break;
					case B9600:
						baud=9600;
						break;
					case B19200:
						baud=19200;
						break;
				}
				report (RPT_WARNING , 
						"MtxOrb: Speed must be 1200, 2400, 9600 or "
							"19200. Using default value of %u baud!", baud);
			}
	}


	/* Get backlight setting*/
	if(config_get_bool( DriverName , "enablebacklight" , 0 , MTXORB_DEF_BACKLIGHT)) {
		backlightenabled = 1;
	}

	barb_is_255 = config_get_bool( DriverName , "hasfullblock" , 0 , 1);


	/* Get uselinewrap setting
	 * This setting is used when the display doesn't support linewrap
	 * or when the display size being used is smaller than the real
	 * LCD size.
	 */
	uselinewrap = config_get_bool( DriverName , "uselinewrap" , 0 , 1);

	/* Get display type */
	type=config_get_string ( DriverName , "type" , 0 , MTXORB_DEF_TYPE);

	if (strncasecmp(type, "lcd", 3) == 0) {
		MtxOrb_type = MTXORB_LCD;
	} else if (strncasecmp(type, "lkd", 3) == 0) {
		MtxOrb_type = MTXORB_LKD;
	} else if (strncasecmp (type, "vfd", 3) == 0) {
		MtxOrb_type = MTXORB_VFD;
	} else if (strncasecmp (type, "vkd", 3) == 0) {
		MtxOrb_type = MTXORB_VKD;
	} else {
		report (RPT_ERR, "MtxOrb: unknown display type %s; "
							"must be one of lcd, lkd, vfd, or vkd", type);
		return (-1);
		}

	/* Get keypad settings*/

	/* keypad test mode? */
	if (config_get_bool( DriverName , "keypad_test_mode" , 0 , 0)) {
		report (RPT_INFO, "MtxOrb: Entering keypad test mode...\n");
		keypad_test_mode = 1;
	}

	if (!keypad_test_mode) {
		/* We don't send any chars to the server in keypad test mode.
		 * So there's no need to get them from the configfile in keypad test mode.
		 */
		/* pause_key */
		pause_key = MtxOrb_parse_keypad_setting (DriverName, "PauseKey", MTXORB_DEF_PAUSE_KEY);
		report (RPT_DEBUG, "MtxOrb: Using \"%c\" as pause_key.", pause_key);

		/* back_key */
		back_key = MtxOrb_parse_keypad_setting (DriverName, "BackKey", MTXORB_DEF_BACK_KEY);
		report (RPT_DEBUG, "MtxOrb: Using \"%c\" as back_key", back_key);

		/* forward_key */
		forward_key = MtxOrb_parse_keypad_setting (DriverName, "ForwardKey", MTXORB_DEF_FORWARD_KEY);
		report (RPT_DEBUG, "MtxOrb: Using \"%c\" as forward_key", forward_key);

		/* main_menu_key */
		main_menu_key = MtxOrb_parse_keypad_setting (DriverName, "MainMenuKey", MTXORB_DEF_MAIN_MENU_KEY);
		report (RPT_DEBUG, "MtxOrb: Using \"%c\" as main_menu_key", main_menu_key);
	}

	/* End of config file parsing*/


	/* Allocate framebuffer memory*/
	/* You must use driver->framebuf here, but may use lcd.framebuf later.*/
	if (!driver->framebuf) {
		driver->framebuf = malloc (driver->wid * driver->hgt);
	}

	if (!driver->framebuf) {
	        report(RPT_ERR, "MtxOrb: Error: unable to create framebuffer.");
		return -1;
	}
	
	// Initialize to be empty, just to be safe.
	memset (MtxOrb->framebuf, ' ', (MtxOrb->wid * MtxOrb->hgt));

    lcd_contains = malloc(MtxOrb->wid * MtxOrb->hgt);
    if (!lcd_contains) {
        report(RPT_ERR, "MtxOrb: Error: unable to create lcd_contains.");
        return -1;
    }
	// Fill it with the dirty char so the first LCD screen will all be sent.
    memset(lcd_contains, DIRTY_CHAR, MtxOrb->wid * MtxOrb->hgt);


	/* Set up io port correctly, and open it... */
	fd = open (device, O_RDWR | O_NOCTTY);
	if (fd == -1) {
		switch (errno) {
			case ENOENT:
				report (RPT_ERR, "MtxOrb: %s device file missing!", device);
				break;
			case EACCES: report (RPT_ERR, "MtxOrb: %s device could not be opened...", device);
				report (RPT_ERR, "MtxOrb: Make sure you have rw access to %s", device);
				break;
			default: report (RPT_ERR, "MtxOrb: opening %s failed (%s)", device, strerror (errno));
				break;
		}
  		return -1;
	} else
		report (RPT_INFO, "MtxOrb: opened display on %s", device);

	tcgetattr (fd, &portset);

	portset.c_iflag &= ~( IGNBRK | BRKINT | PARMRK | ISTRIP
	                      | INLCR | IGNCR | ICRNL | IXON );
	portset.c_oflag &= ~(OPOST | ONLCR);
	portset.c_lflag &= ~( ECHO | ECHONL | ICANON | ISIG | IEXTEN );
	portset.c_cflag &= ~( CSIZE | PARENB | CRTSCTS );
	portset.c_cflag |= CS8 | CREAD | CLOCAL ;
	portset.c_cc[VMIN] = 1;
	portset.c_cc[VTIME] = 3;

	// Set port speed
	cfsetospeed (&portset, speed);
	cfsetispeed (&portset, B0);

	// Do it...
	tcsetattr (fd, TCSANOW, &portset);

	/*
	 * Configure display
	 */

	MtxOrb_linewrap (MTXORB_DEF_LINEWRAP);
	MtxOrb_autoscroll (MTXORB_DEF_AUTOSCROLL);
	MtxOrb_cursorblink (MTXORB_DEF_CURSORBLINK);
	MtxOrb_contrast (contrast);
	/* Set autotransmit to on - so keys will be sent to
	 * us automatically (usually the default but let's not make
	 * any assumptions).
	 */
	write(fd, "\x0FE" "A", 2);

	/*
	 * Configure the display functions
	 */

	if (!keypad_test_mode) {
		/* make the server daemonize after initialization*/
		driver->daemonize = 1;
	} else {
		/* We're running in keypad_test_mode now, so don't daemonize */
		driver->daemonize = 0;
		fprintf(stdout, "MtxOrb: Press a key of your device.\n");
	}

	driver->clear = MtxOrb_clear;
	driver->string = MtxOrb_string;
	driver->chr = MtxOrb_chr;
	driver->vbar = MtxOrb_vbar;
	driver->init_vbar = MtxOrb_init_vbar;
	driver->hbar = MtxOrb_hbar;
	driver->init_hbar = MtxOrb_init_hbar;
	driver->num = MtxOrb_num;
	driver->init_num = MtxOrb_init_num;

	driver->init = MtxOrb_init;
	driver->close = MtxOrb_close;
	driver->flush = MtxOrb_flush;
	driver->flush_box = MtxOrb_flush_box;
	driver->contrast = MtxOrb_contrast;
	driver->backlight = MtxOrb_backlight;
	driver->output = MtxOrb_output;
	driver->set_char = MtxOrb_set_char;
	driver->icon = MtxOrb_icon;
	driver->draw_frame = MtxOrb_draw_frame;

	driver->getkey = MtxOrb_getkey;
	driver->getinfo = MtxOrb_getinfo;
	driver->heartbeat = MtxOrb_heartbeat;

	return fd;
}

#define ValidX(x) if ((x) > MtxOrb->wid) { (x) = MtxOrb->wid; } else (x) = (x) < 1 ? 1 : (x);
#define ValidY(y) if ((y) > MtxOrb->hgt) { (y) = MtxOrb->hgt; } else (y) = (y) < 1 ? 1 : (y);

/***************************************************************************
 * Clear: catch up when the screen get clear to be able to
 *  forget bar caracter not in use anymore and reuse the
 *  slot for another bar caracter.
 */
static void
MtxOrb_clear ()
{
	static int called=0;
	int i;

	if (MtxOrb->framebuf != NULL)
		memset (MtxOrb->framebuf, ' ', (MtxOrb->wid * MtxOrb->hgt));

	for (i=0; i<NUM_CUSTOM_CHARS; i++)
	{
		use[i]=0;
	}

	custom=normal; /* assume normal custom characters until bignum/vbar/hbar is
					* used.
					*/

	/* Not the perfect place for this - but...
	 * To avoid the defCount[]s overflowing and messing up the custom char
	 * allocation scheme - we will clear them every now and then...
	 */
	called++;
	if (called>0x000FFFFF)
	{
		int i;
		called=0;

		for (i=0; i<NUM_CUSTOM_CHARS; i++)
		{
			defCount[i]=1;
		}
	}

	debug(RPT_DEBUG, "MtxOrb: asked to clear screen");
}

/*****************************************************************************
 * Clean-up
 */
static void
MtxOrb_close ()
{
	close (fd);

	if (MtxOrb->framebuf)
		free (MtxOrb->framebuf);

	MtxOrb->framebuf = NULL;

    if (lcd_contains)
        free (lcd_contains);
    lcd_contains=NULL;

	report(RPT_INFO, "MtxOrb: closed");
}

static void
MtxOrb_string (int x, int y, char *string)
{
	int offset, siz;

	ValidX(x);
	ValidY(y);

	x--; y--; /* Convert 1-based coords to 0-based... */
	offset = (y * MtxOrb->wid) + x;
	siz = (MtxOrb->wid * MtxOrb->hgt) - offset;
	siz = siz > strlen(string) ? strlen(string) : siz;

	memcpy(MtxOrb->framebuf + offset, string, siz);

	debug(RPT_DEBUG, "MtxOrb: printed string at (%d,%d)", x, y);
}

static void
MtxOrb_flush ()
{
	MtxOrb_draw_frame (MtxOrb->framebuf);

	debug(RPT_DEBUG, "MtxOrb: frame buffer flushed");
}

static void
MtxOrb_flush_box (int lft, int top, int rgt, int bot)
{
    int y;
    int x;
    int mv=0;
    unsigned char *p, *q;
    
    debug(RPT_DEBUG, "MtxOrb: flush_box(%d,%d,%d,%d)", lft, top, rgt, bot);

	sendOutCustomChars();

    // We can't just write it out because it could contain
    // the dirty character.
    for (y = top; y <= bot; y++) {
        p=(unsigned char *)MtxOrb->framebuf + (y * MtxOrb->wid) + lft;
        q = lcd_contains + (y*MtxOrb->wid) + lft;
        mv = 1;

        for (x = lft; x <= rgt; x++) {

            if ((*p == *q) || (*p == DIRTY_CHAR))
                mv = 1;
            else {

                if (mv == 1)
                {
                    char out[5];
                    snprintf(out, sizeof(out), "\x0FEG%c%c", x, y);
                    write (fd, out, 4);
                    mv = 0;
                }
                write (fd, p, 1);
				*q = *p;
            }
            p++;
            q++;
        }
    }

	debug(RPT_DEBUG, "MtxOrb: frame buffer box flushed");
}

/*************************************************************************
 * Prints a character on the lcd display, at position (x,y).  The
 * upper-left is (1,1), and the lower right should be (20,4).
 */
static void
MtxOrb_chr (int x, int y, char c)
{
	int offset;

	/* Characters may or may NOT be alphabetic; it appears
	 * that characters 0..4 (or similar) are graphic fonts
	 */

	ValidX(x);
	ValidY(y);

	/* write immediately to screen... this code was taken
	 * from the LK202-25; should work for others, yes?
	 * snprintf(out, sizeof(out), "\x0FEG%c%c%c", x, y, c);
	 * write (fd, out, 4);
	 */

	/* write to frame buffer */
	y--; x--; /* translate to 0-index */
	offset = (y * MtxOrb->wid) + x;
	MtxOrb->framebuf[offset] = c;

	debug(RPT_DEBUG, "MtxOrb: written character %02X to position (%d,%d)", c, x, y);
}

/**************************************************************************
 * Changes screen contrast (0-255; 140 seems good)
 * note: works only for LCD displays
 * Is it better to use the brightness for VFD/VKD displays ?
 */
static int
MtxOrb_contrast (int contrast)
{
	static int mtxorb_contrast_state = -1;

	char out[4];

	if (contrast==-1 || contrast==mtxorb_contrast_state) {
	    return mtxorb_contrast_state;
	}

	/* validate contrast value */
	if (contrast > 255)
		contrast = 255;
	if (contrast < 0)
		contrast = 0;

	if (IS_LCD_DISPLAY || IS_LKD_DISPLAY) {
		snprintf (out, sizeof(out), "\x0FEP%c", contrast);
		write (fd, out, 3);
		mtxorb_contrast_state=contrast;

		report(RPT_DEBUG, "MtxOrb: contrast set to %d", contrast);
	} else {
		report(RPT_DEBUG, "MtxOrb: contrast not set to %d - not LCD or LKD display", contrast);
		mtxorb_contrast_state = -1;
	}

	return mtxorb_contrast_state;
}

/********************************************************************
 * Sets the backlight on or off -- can be done quickly for
 * an intermediate brightness...
 *
 * WARNING: off switches vfd/vkd displays off entirely
 *	    so maybe it is best to start LCDd with -b on
 *
 * WARNING: there seems to be a movement afoot to add more
 *          functions than just on/off to this..
 */

static void
MtxOrb_backlight (int on)
{
	static int mtxorb_backlight_state = -1;

	if (mtxorb_backlight_state == on)
		return;
	mtxorb_backlight_state = on;

	if(backlightenabled) {
		if (on){
			write (fd, "\x0FE" "B" "\x000", 3);
			report(RPT_DEBUG, "MtxOrb: backlight turned on");
		} else {
			if (IS_VKD_DISPLAY || IS_VFD_DISPLAY) {
				/* turns display off entirely (whoops!) */
				report(RPT_DEBUG, "MtxOrb: backlight ignored - not LCD or LKD display");
			} else {
				write (fd, "\x0FE" "F", 2);
				report(RPT_DEBUG, "MtxOrb: backlight turned off");
			}
		}
	}
}

/*************************************************************************
 * Sets output port on or off
 * displays with keypad have 6 outputs but the one without keypad
 * have only one output
 * NOTE: length of command are different
 */
static void
MtxOrb_output (int on)
{
	static int mtxorb_output_state = -1;

	char out[5];

	on = on & 077;	// strip to six bits

	if (mtxorb_output_state == on)
		return;

	mtxorb_output_state = on;

	report(RPT_DEBUG, "MtxOrb: output pins set: %04X", on);

	if (IS_LCD_DISPLAY || IS_VFD_DISPLAY) {
		/* LCD and VFD displays only have one output port */
		if (on)
			write (fd, "\x0FEW", 2);
		else
			write (fd, "\x0FEV", 2);
	} else {
		int i;

		/* Other displays have six output ports;
		 * the value "on" is a binary value determining which
		 * ports are turned on (1) and off (0).
		 */

		for(i = 0; i < 6; i++) {
			if (on & (1 << i))
				snprintf (out, sizeof(out), "\x0FEW%c", i + 1);
			else
				snprintf (out, sizeof(out), "\x0FEV%c", i + 1);
			write (fd, out, 3);
		}
	}
}

/***********************************************************************************
 * Toggle the built-in linewrapping feature
 */
static void
MtxOrb_linewrap (int on)
{
	if (on) {
		write (fd, "\x0FE" "C", 2);
		debug(RPT_DEBUG, "MtxOrb: linewrap turned on");
	} else {
		write (fd, "\x0FE" "D", 2);
		debug(RPT_DEBUG, "MtxOrb: linewrap turned off");
	}
	linewrapenabled = on;
}

/************************************************************************************
 * Toggle the built-in automatic scrolling feature
 */
static void
MtxOrb_autoscroll (int on)
{
	if (on) {
		write (fd, "\x0FEQ", 2);
		debug(RPT_DEBUG, "MtxOrb: autoscroll turned on");
	} else {
		write (fd, "\x0FER", 2);
		debug(RPT_DEBUG, "MtxOrb: autoscroll turned off");
	}
}

/* TODO: make sure this doesn't mess up non-VFD displays*/

/***********************************************************************************
 * Toggle cursor blink on/off
 */
static void
MtxOrb_cursorblink (int on)
{
	if (on) {
		write (fd, "\x0FES", 2);
		debug(RPT_DEBUG, "MtxOrb: cursorblink turned on");
	} else {
		write (fd, "\x0FET", 2);
		debug(RPT_DEBUG, "MtxOrb: cursorblink turned off");
	}
}

/************************************************************************************
 * Sets up for vertical bars.  Call before lcd.vbar()
 */
static void
MtxOrb_init_vbar ()
{
	custom = vbar;
}

/************************************************************************************
 * Inits horizontal bars...
 */
static void
MtxOrb_init_hbar ()
{
	custom = hbar;
}

/***********************************************************************************
 * Returns string with general information about the display
 */
static char *
MtxOrb_getinfo (void)
{
	char in = 0;
	static char info[255];
	char tmp[255], buf[64];
	/* int i = 0; */
	fd_set rfds;

	struct timeval tv;
	int retval;

	debug(RPT_DEBUG, "MtxOrb: getinfo()");

	memset(info, '\0', sizeof(info));
	strncpy(info, "Matrix Orbital Driver ", sizeof(info));

	/*
	 * Read type of display
	 */

	write(fd, "\x0FE" "7", 2);

	/* Watch fd to see when it has input. */
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	/* Wait the specified amount of time. */
	tv.tv_sec = 0;		/* seconds */
	tv.tv_usec = 500;	/* microseconds */

	retval = select(1, &rfds, NULL, NULL, &tv);

	if (retval) {
		if (read (fd, &in, 1) < 0) {
			report (RPT_WARNING, "MtxOrb: getinfo(): unable to read data");
		} else {
			switch (in) {
				case '\x01': strcat(info, "LCD0821 "); break;
				case '\x03': strcat(info, "LCD2021 "); break;
				case '\x04': strcat(info, "LCD1641 "); break;
				case '\x05': strcat(info, "LCD2041 "); break;
				case '\x06': strcat(info, "LCD4021 "); break;
				case '\x07': strcat(info, "LCD4041 "); break;
				case '\x08': strcat(info, "LK202-25 "); break;
				case '\x09': strcat(info, "LK204-25 "); break;
				case '\x0A': strcat(info, "LK404-55 "); break;
				case '\x0B': strcat(info, "VFD2021 "); break;
				case '\x0C': strcat(info, "VFD2041 "); break;
				case '\x0D': strcat(info, "VFD4021 "); break;
				case '\x0E': strcat(info, "VK202-25 "); break;
				case '\x0F': strcat(info, "VK204-25 "); break;
				case '\x10': strcat(info, "GLC12232 "); break;
				case '\x11': strcat(info, "GLC12864 "); break;
				case '\x12': strcat(info, "GLC128128 "); break;
				case '\x13': strcat(info, "GLC24064 "); break;
				case '\x14': strcat(info, "GLK12864-25 "); break;
				case '\x15': strcat(info, "GLK24064-25 "); break;
				case '\x21': strcat(info, "GLK128128-25 "); break;
				case '\x22': strcat(info, "GLK12232-25 "); break;
				case '\x31': strcat(info, "LK404-AT "); break;
				case '\x32': strcat(info, "VFD1621 "); break;
				case '\x33': strcat(info, "LK402-12 "); break;
				case '\x34': strcat(info, "LK162-12 "); break;
				case '\x35': strcat(info, "LK204-25PC "); break;
				case '\x36': strcat(info, "LK202-24-USB "); break;
				default:
					report(RPT_WARNING, "MtxOrb: getinfo(): Unknown device");
					break;
			}
		}
	} else
		report(RPT_WARNING, "MtxOrb: getinfo(): unable to read device type");

	/*
	 * Read serial number of display
	 */

	memset(tmp, '\0', sizeof(tmp));
	write(fd, "\x0FE" "5", 2);

	/* Wait the specified amount of time. */
	tv.tv_sec = 0;		// seconds
	tv.tv_usec = 500;	// microseconds

	retval = select(1, &rfds, NULL, NULL, &tv);

	if (retval) {
		if (read (fd, &tmp, 2) < 0) {
			report (RPT_WARNING, "MtxOrb: getinfo(): unable to read data");
		} else {
			snprintf(buf, sizeof(buf), "Serial No: %ld ", (long int) tmp);
			strcat(info, buf);
		}
	} else
		syslog(LOG_WARNING, "MtxOrb: getinfo(): unable to read device serial number");

	/*
	 * Read firmware revision number
	 */

	memset(tmp, '\0', sizeof(tmp));
	write(fd, "\x0FE" "6", 2);

	/* Wait the specified amount of time. */
	tv.tv_sec = 0;		// seconds
	tv.tv_usec = 500;	// microseconds

	retval = select(1, &rfds, NULL, NULL, &tv);

	if (retval) {
		if (read (fd, &tmp, 2) < 0) {
			report (RPT_WARNING, "MtxOrb: getinfo(): unable to read data");
		} else {
			snprintf(buf, sizeof(buf), "Firmware Rev. %ld ", (long int) tmp);
			strcat(info, buf);
		}
	} else
		report (RPT_WARNING, "MtxOrb: getinfo(): unable to read device firmware revision");

	return info;
}

/* TODO: Finish the support for bar growing reverse way.*/
/* TODO: Need a "y" as input also !!! */
/**************************************************************************************************
 * Draws a vertical bar...
 * This is the new version ussing dynamic icon alocation
 */
static void
MtxOrb_vbar (int x, int len)
{
	unsigned char mapu[9] = { barw, baru1, baru2, baru3, baru4, baru5, baru6, baru7, barb };
	unsigned char mapd[9] = { barw, bard1, bard2, bard3, bard4, bard5, bard6, bard7, barb };

	int y;

	debug(RPT_DEBUG, "MtxOrb: vertical bar at %d set to %d", x, len);

	custom=vbar; 	// this is reset on MtxOrb_clear - so we might not get a
   					// init_vbar if we're still on the same screen.

/* REMOVE THE NEXT LINE FOR TESTING ONLY...*/
/*  len=-len;*/
/* REMOVE THE PREVIOUS LINE FOR TESTING ONLY...*/

	if (len > 0) {
		for (y = MtxOrb->hgt; y > 0 && len > 0; y--) {
			if (len >= MtxOrb->cellhgt)
				MtxOrb_chr (x, y, MtxOrb_ask_bar(barb, 1));
			else
				MtxOrb_chr (x, y, MtxOrb_ask_bar (mapu[len], 1));

			len -= MtxOrb->cellhgt;
		}
	} else {
		len = -len;
		for (y = 2; y <= MtxOrb->hgt && len > 0; y++) {
			if (len >= MtxOrb->cellhgt)
				MtxOrb_chr (x, y, MtxOrb_ask_bar(barb, 1));
			else
				MtxOrb_chr (x, y, MtxOrb_ask_bar (mapd[len], 1));

			len -= MtxOrb->cellhgt;
		}
	}

}

/* TODO: Finish the support for bar growing reverse way. */
/*******************************************************************************
 * Draws a horizontal bar to the right.
 * This is the new version ussing dynamic icon alocation
 */
static void
MtxOrb_hbar (int x, int y, int len)
{
	unsigned char mapr[6] = { barw, barr1, barr2, barr3, barr4, barb };
	unsigned char mapl[6] = { barw, barl1, barl2, barl3, barl4, barb };

	ValidX(x);
	ValidY(y);

	debug(RPT_DEBUG, "MtxOrb: horizontal bar at %d set to %d", x, len);

	custom=hbar; 	// this is reset on MtxOrb_clear - so we might not get a
   					// init_hbar if we're still on the same screen.

	if (len > 0) {
		for (; x <= MtxOrb->wid && len > 0; x++) {
			if (len >= MtxOrb->cellwid)
				MtxOrb_chr (x, y, MtxOrb_ask_bar(barb, 1));
			else
				MtxOrb_chr (x, y, MtxOrb_ask_bar (mapr[len], 1));

			len -= MtxOrb->cellwid;

		}
	} else {
		len = -len;
		for (; x > 0 && len > 0; x--) {
			if (len >= MtxOrb->cellwid)
				MtxOrb_chr (x, y, MtxOrb_ask_bar(barb, 1));
			else
				MtxOrb_chr (x, y, MtxOrb_ask_bar (mapl[len], 1));

			len -= MtxOrb->cellwid;

		}
	}

}

/* TODO: Might not work, bignum is untested... an untested with dynamic bar.
 */
/* TODO: Rather than to use the hardware BigNum we should use software
 * emulation, this will make it work simultaniously as hbar/vbar. GLU
 */

/**************************************************************************************
 * Sets up for big numbers.
 */
static void
MtxOrb_init_num ()
{
	debug(RPT_DEBUG, "MtxOrb: init for big numbers");

	if (custom != bign) {
		write (fd, "\x0FEn", 2);
		custom = bign;
		MtxOrb_clear_custom ();
	}

}

/* TODO: Rather than to use the hardware BigNum we should use software
 * emulation, this will make it work simultaniously as hbar/vbar. GLU
 */
/* TODO: Before the desinitive solution we need to make the caracter
 * hiden behind the hardware bignum dirty so that they get cleaned
 * when draw_frame is called. There is no dirty char so I will use 254
 * hoping nobody is using that char. GLU
 *
 * Nobody is going to be using 254 - as that's the command escape character! CJL
 *
 * TODO: No support for num==10 - this is meant to be a colon - CJL
 *
 */

/*************************************************************************
 * Writes a big number.
 */
static void
MtxOrb_num (int x, int num)
{
	int y, dx;
	char out[5];

	if (custom!=bign)
	{	// custom might not be set correctly because
		// we reset it in MtxOrb_clear() - init won't be called again if
		// we're still on the same screen.
		
		custom = bign;
		MtxOrb_clear_custom ();
	}

	debug(RPT_DEBUG, "MtxOrb: write big number %d at %d", num, x);

	snprintf (out, sizeof(out), "\x0FE#%c%c", x, num);
	write (fd, out, 4);

/* Make this space dirty as far as frame buffer knows.*/
	for (y = 1; y < 5; y++)
		for (dx = 0; dx < 3; dx++)
			MtxOrb_chr (x + dx, y, DIRTY_CHAR);

}

/* TODO: Every time we define a custom char within the LCD,
 * we have to compute the binary value we are going to use.
 * It is easy to keep the bitmap in this source file,
 * but we compute that once rather than every time. GLU
 */
/* TODO: We make one 3 bytes write folowed by MtxOrb->cellhgt one byte
 * write. This should be done in one single write. GLU
 */


/***********************************************************************************
 * Sets a custom character from 0-7...
 *
 * The input is just an array of characters...
 *
 * This function appears to be used internally only,
 * other code in this file will break if that assumption is not true.
 */
static void
MtxOrb_set_char (int n, char *dat)
{
	char out[4];
	int row, col;
	int letter;

	if (n < 0 || n >= NUM_CUSTOM_CHARS)
		return;
	if (!dat)
		return;

	snprintf (out, sizeof(out), "\x0FEN%c", n);
	write (fd, out, 3);

	for (row = 0; row < MtxOrb->cellhgt; row++) {
		letter = 0;
		for (col = 0; col < MtxOrb->cellwid; col++) {
			/* shift to make room for new scan line data */
			letter <<= 1;
			/* Now read a single bit of data*/
			/* -- one entry in dat[] --*/
			/* and add it to the binary data in "letter"*/
			letter |= (dat[(row * MtxOrb->cellwid) + col] > 0);
		}
		write (fd, &letter, 1); /* write one character for each row*/
	}
}

/************************************************************************************
 * Set an icon
 * This function doesn't seem to be called.
 * Don't use internally.
 */
static void
MtxOrb_icon (int which, char dest)
{

	if ((which>=0) && (which<KNOWN_CHARS))
	{
		reassign(which); // reassign which if it's on screen.
		sent[(int)dest]=0;
		use[(int)dest]=1;
		def[(int)dest]=which;
		defCount[which]++;
	}
}

/************************************************************************************
 * Blasts a single frame onscreen, to the lcd...
 *
 * Input is a character array, sized lcd.wid*lcd.hgt
 */
static void
MtxOrb_draw_frame (char *dat)
{
	char out[12];
	int i,j,mv = 1;
	unsigned char *p, *q;
	static int called=0;

	if (!dat)
		return;

	/* What ever you do - serial communications are not perfect and
	 * transmissions errors do occur.  So we could end up with what
	 * we think is on the lcd and what is actually on the lcd as being
	 * different.  If it's in an area of the lcd that doesn't change
	 * offern then an error could be on the lcd for a while.
	 *
	 * It's best to sync up every now and then - so I'm going to
	 * really draw the whole lcd every 1000 times this function is called.
	 */
	called++;
	if (called>=1000)
	{  // time to sync our view of the world and the lcd.
		// Make our view dirty so the full screen is outputted.
		memset(lcd_contains, DIRTY_CHAR, MtxOrb->wid * MtxOrb->hgt);

		/* We could extend this idea further and reset the lcd
		 * settings (contrast etc) - it's possible, because of a
		 * transmission error that lcd was told to set things we
		 * didn't want done! But I won't code that at the moment.
		 */

		called=0;
	}

	sendOutCustomChars();

    p = (unsigned char*)dat;
    q = lcd_contains;

	for (i = 1; i <= MtxOrb->hgt; i++) {
		/* if linewrap isn't enabled or we aren't allowed
		 * to use linewrap then move to the start on each
		 * line.
		 */
		if (!uselinewrap || !linewrapenabled)
			mv =1;

		for (j = 1; j <= MtxOrb->wid; j++) {

			if ((*p == *q) || (*p == DIRTY_CHAR))
				mv = 1;
			else {
				/* Draw characters that have changed.
				 */

				if (mv == 1) {
					snprintf(out, sizeof(out), "\x0FEG%c%c", j, i);
					write (fd, out, 4);
					mv = 0;
				}
				write (fd, p, 1);
			}
			p++;
			q++;
		}
	}

	memcpy(lcd_contains, dat, MtxOrb->wid * MtxOrb->hgt);
}

/* TODO: Recover the code for I2C connectivity to MtxOrb
 * and don't query the LCD if it does not support keypad.
 * Otherwise crash of the LCD and/or I2C bus.
 */
/******************************************************************************
 * returns one character from the keypad...
 * (A-Z) on success, 0 on failure...
 */
static char
MtxOrb_getkey ()
{
	char in = 0;

	// POLL For data or return
	struct pollfd fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;
	fds[0].revents = 0;
	poll (fds,1,0);
	if (fds[0].revents == 0) { return in; }

	read (fd, &in, 1);
	report(RPT_INFO, "MtxOrb: getkey: key X %i", in);

	if (in!=0) {
		if (!keypad_test_mode) {
			if (in==pause_key) {
				in = INPUT_PAUSE_KEY;
			} else if (in==back_key) {
				in = INPUT_BACK_KEY;
			} else if (in==forward_key){
				in = INPUT_FORWARD_KEY;
			} else if (in==main_menu_key) {
				in = INPUT_MAIN_MENU_KEY;
			}
			/*TODO: add more translations here (anything flexible that can be done from LCDd.conf ...)*/
			  else {
			  	/*in = 0;*/ /*Enable this if your display sends garbage*/
			}
			report(RPT_DEBUG, "MtxOrb: getkey(): returning %c", in);
		} else {
			/* We're running in keypad_test_mode.
			 * So only output the character that has been received.
			 * The output goes directly to stdout. Otherwise it might
			 * not be visible because of the report level
			 */
			fprintf (stdout, "MtxOrb: Received character %c\n", in);
			in = 0;
			fprintf (stdout, "MtxOrb: Press another key of your device.\n");
		}
	}

	return in;
}

/****************************************************************************
 * Ask for dynamic allocation of a custom caracter to be
 *  a well none bar graphic. The function is suppose to
 *  return a value between 0 and 7 but 0 is reserver for
 *  heart beat.
 *  This function manadge a cache of graphical caracter in use.
 *  I really hope it is working and bug-less because it is not
 *  completely tested, just a quick hack.
 */
static int
MtxOrb_ask_bar (int type, int priority)
{
	int i;
	int pos;			/* 0 to 7 are custom chars, 8 is not found.*/

	// If the current LCD's font has a full block at character 255, then
	// use that rather than define a custom character.
	if ((type==barb) && (barb_is_255))
		return(255);

	if ((type >= KNOWN_CHARS) || (type<0))
		return type;

	if (custom==bign)
		return approx_match(type);

	/* Check if it's already defined. */
	pos = CUSTOM_NOT_FOUND;				/* Not found.*/
	for (i = 0; i < NUM_CUSTOM_CHARS; i++) 
	{		
		if (def[i] == type)
		{ /* Match found */
			pos = i;		
			break;
		}
	}

	if (pos==CUSTOM_NOT_FOUND)
	{	// Still not found -- search for a slot that's unused.
		// Choose the least used type.
		// And try to avoid vbar chars if in vbar mode (same for hbar).
		unsigned int minCount_own=0xFFFFFFFF;
		unsigned int minCount_not_own=0xFFFFFFFF;
		int pos_own=CUSTOM_NOT_FOUND;
		int pos_not_own=CUSTOM_NOT_FOUND;

		for (i = 0; i < NUM_CUSTOM_CHARS; i++) 
		{		
			if (!use[i])
			{
				int deftype=def[i];
				int i_count=0;
				int not_own_char=0;

				// Try to avoid vbar chars if in vbar mode (same for hbar)
				if (((custom==hbar) && (!IS_BARL(deftype)) 
							        && (!IS_BARR(deftype))) ||
					((custom==vbar) && (!IS_BARU(deftype)) 
					 				&& (!IS_BARD(deftype))) ||
					(custom==normal))
				{
					not_own_char=1;
				}

				// lookup how much this character has been used - ever.
				if ((deftype>=0) && (deftype<KNOWN_CHARS))
					i_count=defCount[deftype];

				if ((not_own_char) && (i_count<minCount_not_own))
				{
					minCount_not_own = i_count;
					pos_not_own = i;
				}
				else if ((!not_own_char) && (i_count<minCount_own))
				{
					minCount_own = i_count;
					pos_own=i;
				}
			}
		}

		// Prefer using not one of our own... (vbar chars if in vbar mode etc)
		if (pos_not_own != CUSTOM_NOT_FOUND)
			pos = pos_not_own;
		else
			pos = pos_own;
	}


	/* We have found a slot to use */
	if (pos != CUSTOM_NOT_FOUND) 
	{	
		if (def[pos] != type) 
		{
			def[pos] = type; /* Remember that now the character is available. */
			sent[pos] = 0;
			defCount[type]++; // count the number of time this has been defined
		}
		use[pos]=1;	  /* Remember it is in use. */
	}


	/* There are no free slots left...
	 *
	 * If priority is set (i.e. we're in vbar or hbar mode) then
	 * take the slot from a non-bar character.
	 */
	if ((pos==CUSTOM_NOT_FOUND) && priority)
	{
		int i;
		unsigned int minCount=0xFFFFFFFF;

		for (i=0; i<NUM_CUSTOM_CHARS;i++)
		{
			int deftype=def[i];

			/* don't steal from our own chars */
			if (((custom==hbar) && (!IS_BARL(deftype)) 
								&& (!IS_BARR(deftype))) ||
				((custom==vbar) && (!IS_BARU(deftype)) 
								&& (!IS_BARD(deftype))) ||
				(custom==normal))
			{
				if (defCount[deftype]<minCount)
				{
					minCount=defCount[deftype];
					pos=i;
				}
			}
		}
		if (pos!=CUSTOM_NOT_FOUND)
		{
			/* We are going to steal slot 'pos' */

			reassign(pos);

			/* now steal it */
			def[pos]=type;
			use[pos]=1;
			defCount[type]++;
			sent[pos]=0;
		}
	}

	// Still no slot found - give up and give them an ascii character...
	if (pos == CUSTOM_NOT_FOUND) 
	{
		pos = approx_match(type);
	}

	return (pos);
}

/**********************************************************************
 * Does the heartbeat...
 */
static void
MtxOrb_heartbeat (int type)
{
	int the_icon=255;
	static int timer = 0;
	int whichIcon;
	static int saved_type = HEARTBEAT_ON;

	if (type)
		saved_type = type;

	if (type == HEARTBEAT_ON) {
		/* Set this to pulsate like a real heart beat... */
		whichIcon = (! ((timer + 4) & 5));

		the_icon=MtxOrb_ask_bar (whichIcon+START_ICON, 0);

		/* Put character on screen...*/
		MtxOrb_chr (MtxOrb->wid, 1, the_icon);
	}

	timer++;
	timer &= 0x0f;
}

/***************************************************************************
 * Sets up a well known character for use.
 */
static void
MtxOrb_set_known_char (int car, int type)
{
	char all_bar[KNOWN_CHARS][5 * 8] = {
		{
		0, 0, 0, 0, 0,	//  char u1[] =
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	//  char u2[] =
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	//  char u3[] =
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	//  char u4[] =
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	//  char u5[] =
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	//  char u6[] =
		0, 0, 0, 0, 0,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	//  char u7[] =
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		1, 1, 1, 1, 1,	//  char d1[] =
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		}, {
		1, 1, 1, 1, 1,	//  char d2[] =
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		}, {
		1, 1, 1, 1, 1,	//  char d3[] =
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		}, {
		1, 1, 1, 1, 1,	//  char d4[] =
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		}, {
		1, 1, 1, 1, 1,	//  char d5[] =
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		}, {
		1, 1, 1, 1, 1,	//  char d6[] =
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		}, {
		1, 1, 1, 1, 1,	//  char d7[] =
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 0,
		}, {
		1, 0, 0, 0, 0,	//  char r1[] =
		1, 0, 0, 0, 0,
		1, 0, 0, 0, 0,
		1, 0, 0, 0, 0,
		1, 0, 0, 0, 0,
		1, 0, 0, 0, 0,
		1, 0, 0, 0, 0,
		1, 0, 0, 0, 0,
		}, {
		1, 1, 0, 0, 0,	//  char r2[] =
		1, 1, 0, 0, 0,
		1, 1, 0, 0, 0,
		1, 1, 0, 0, 0,
		1, 1, 0, 0, 0,
		1, 1, 0, 0, 0,
		1, 1, 0, 0, 0,
		1, 1, 0, 0, 0,
		}, {
		1, 1, 1, 0, 0,	//  char r3[] =
		1, 1, 1, 0, 0,
		1, 1, 1, 0, 0,
		1, 1, 1, 0, 0,
		1, 1, 1, 0, 0,
		1, 1, 1, 0, 0,
		1, 1, 1, 0, 0,
		1, 1, 1, 0, 0,
		}, {
		1, 1, 1, 1, 0,	//  char r4[] =
		1, 1, 1, 1, 0,
		1, 1, 1, 1, 0,
		1, 1, 1, 1, 0,
		1, 1, 1, 1, 0,
		1, 1, 1, 1, 0,
		1, 1, 1, 1, 0,
		1, 1, 1, 1, 0,
		}, {
		0, 0, 0, 0, 1,	//  char l1[] =
		0, 0, 0, 0, 1,
		0, 0, 0, 0, 1,
		0, 0, 0, 0, 1,
		0, 0, 0, 0, 1,
		0, 0, 0, 0, 1,
		0, 0, 0, 0, 1,
		0, 0, 0, 0, 1,
		}, {
		0, 0, 0, 1, 1,	//  char l2[] =
		0, 0, 0, 1, 1,
		0, 0, 0, 1, 1,
		0, 0, 0, 1, 1,
		0, 0, 0, 1, 1,
		0, 0, 0, 1, 1,
		0, 0, 0, 1, 1,
		0, 0, 0, 1, 1,
		}, {
		0, 0, 1, 1, 1,	//  char l3[] =
		0, 0, 1, 1, 1,
		0, 0, 1, 1, 1,
		0, 0, 1, 1, 1,
		0, 0, 1, 1, 1,
		0, 0, 1, 1, 1,
		0, 0, 1, 1, 1,
		0, 0, 1, 1, 1,
		}, {
		0, 1, 1, 1, 1,	//  char l4[] =
		0, 1, 1, 1, 1,
		0, 1, 1, 1, 1,
		0, 1, 1, 1, 1,
		0, 1, 1, 1, 1,
		0, 1, 1, 1, 1,
		0, 1, 1, 1, 1,
		0, 1, 1, 1, 1,
		}, {
		1, 1, 1, 1, 1,	// Empty Heart
		1, 0, 1, 0, 1,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 0, 0, 0, 1,
		1, 1, 0, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		1, 1, 1, 1, 1,	// Filled Heart
		1, 0, 1, 0, 1,
		0, 1, 0, 1, 0,
		0, 1, 1, 1, 0,
		0, 1, 1, 1, 0,
		1, 0, 1, 0, 1,
		1, 1, 0, 1, 1,
		1, 1, 1, 1, 1,
		}, {
		0, 0, 0, 0, 0,	// Ellipsis
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,
		1, 0, 1, 0, 1,
		}, {
		1, 1, 1, 1, 1,	// barb
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		1, 1, 1, 1, 1,
		}
	};

	if ((car < KNOWN_CHARS) && (car>=0))
	{
		MtxOrb_set_char (car, &all_bar[type][0]);
	}
}

static void 
sendOutCustomChars(void)
{
	int i;

	for (i=0; i<NUM_CUSTOM_CHARS; i++)
	{
		if ((use[i]) && (!sent[i]))
		{
			MtxOrb_set_known_char (i, def[i]);
			sent[i]=1;
		}
	}
}

/* TODO: Choose a character to approximate the graph*/
static int
approx_match(int type)
{
	int pos;

	switch (type) {
		case baru1:
			pos = '_';
			break;
		case baru2:
			pos = '.';
			break;
		case baru3:
			pos = ',';
			break;
		case baru4:
			pos = 'o';
			break;
		case baru5:
			pos = 'o';
			break;
		case baru6:
			pos = 'O';
			break;
		case baru7:
			pos = '8';
			break;

		case bard1:
			pos = '\'';
			break;
		case bard2:
			pos = '"';
			break;
		case bard3:
			pos = '^';
			break;
		case bard4:
			pos = '^';
			break;
		case bard5:
			pos = '*';
			break;
		case bard6:
			pos = 'O';
			break;
		case bard7:
			pos = '8';
			break;

		case barr1:
			pos = '-';
			break;
		case barr2:
			pos = '-';
			break;
		case barr3:
			pos = '=';
			break;
		case barr4:
			pos = '=';
			break;

		case barl1:
			pos = '-';
			break;
		case barl2:
			pos = '-';
			break;
		case barl3:
			pos = '=';
			break;
		case barl4:
			pos = '=';
			break;

		case barw:
			pos = ' ';
			break;

		case barb:
			pos = 255;
			break;

		case empty_heart:
			pos = 'O';
			break;

		case filled_heart:
			pos = '@';
			break;

		default:
			pos = '?';
			break;
	}

	return(pos);
}

/*
 * Reassign uses of custom char 'pos' so it is freed up.
 * Currently we just give it an approx character rather than
 * checking if it could have a different slot in the cache.
 */
static void
reassign(int pos)
{
	unsigned char *frame;
	unsigned char newchar=0;
	int i;

	if ((pos<0) || (pos>=KNOWN_CHARS))
		return;

	/* We need to reassign the old character
	 * [If any are there...]
	 */
	frame= (unsigned char *)MtxOrb->framebuf;
	for (i = 1; i <= MtxOrb->hgt*MtxOrb->wid; i++) 
	{
		if (*frame == def[pos])
		{
			if (newchar==0)
				newchar = approx_match(def[pos]);
			*frame = newchar;
		}
		frame++;
	}
}
