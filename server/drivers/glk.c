/*  This is the LCDproc driver for MatrixOrbital GLK Graphic Displays
                                         http://www.matrixorbital.com

    Copyright (C) 2001, Philip Pokorny
		  2001, David Douthitt
		  2001, David Glaude
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
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <sys/errno.h>

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif
#if TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h>
#else
# if HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif
#endif

#include "lcd.h"
#include "shared/str.h"
#include "shared/report.h"
#include "configfile.h"
#include "input.h"

#include "glk.h"
#include "glkproto.h"

static GLKDisplay *  PortFD ;
/*  Initialize pseudo-CGRAM to empty */
static unsigned char  CGRAM[8] = { '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0' };

static void glk_close();
static void glk_clear();
static void glk_flush();
static void glk_string(int x, int y, char string[]);
static void glk_chr(int x, int y, char c);
static int glk_contrast(int contrast);
static void glk_backlight(int on);
static void glk_output(int on);
static void glk_vbar(int x, int len);
static void glk_hbar(int x, int y, int len);
static void glk_num(int x, int num);
static void glk_init_num();
static void glk_set_char(int n, char *dat);
static void glk_icon(int which, char dest);
static void glk_flush_box(int lft, int top, int rgt, int bot);
static void glk_draw_frame(char *dat);
static char glk_getkey();

lcd_logical_driver *glk;

static unsigned char *  screen_contents = NULL ;
int  fontselected = 0 ;
int  gpo_count = 0 ;

// TODO: Get lcd.framebuf to properly work as whatever driver is running...


////////////////////////////////////////////////////////////
// init() should set up any device-specific stuff, and
// point all the function pointers.
int glk_init(struct lcd_logical_driver *driver, char *args) 
{
	char device[256] = GLK_DEF_DEVICE ;
	int  contrast = GLK_DEF_CONTRAST ;
	speed_t  speed = GLK_DEF_SPEED ;
	int i, tmp ;
	char buf[256] = "";

	int  width ;
	int  height ;

	debug (RPT_INFO, "glk_init()\n");

	glk = driver;

	/* TODO: replace DriverName with driver->name when that field exists. */
	#define DriverName "glk"


	/* READ THE CONFIG FILE */

	/* Get serial device to use */
	strncpy(device, config_get_string ( DriverName , "Device" , 0 , GLK_DEF_DEVICE),sizeof(device));
	device[sizeof(device)-1]=0;
	report (RPT_INFO,"glk: Using device: %s", device);

	/* Get contrast */
	if (0<=config_get_int ( DriverName , "contrast" , 0 , GLK_DEF_CONTRAST) && config_get_int ( DriverName , "contrast" , 0 , GLK_DEF_CONTRAST) <= 255) {
		contrast = config_get_int ( DriverName , "contrast" , 0 , GLK_DEF_CONTRAST);
	} else {
		report (RPT_WARNING, "glk: Contrast must be between 0 and 255. Using default value.\n");
	}

	/* Get serial speed*/
	tmp = config_get_int ( DriverName , "speed" , 0 , GLK_DEF_SPEED);

	switch (tmp) {
		case 9600:
			speed = B9600;
			break;
		case 19200:
			speed = B19200;
			break;
		case 38400:
			speed = B38400;
			break;
		default:
			speed = GLK_DEF_SPEED;
			switch (speed) {
				case B9600:
					strncpy(buf,"9600", sizeof(buf));
					break;
				case B19200:
					strncpy(buf,"19200", sizeof(buf));
					break;
				case B38400:
					strncpy(buf,"38400", sizeof(buf));
					break;
			}
			report (RPT_WARNING , "glk: Speed must be 9600, 19200 or 38400. Using default value of %s baud!", buf);
			strncpy(buf,"", sizeof(buf));
	}


  PortFD = glkopen( device, speed );
  if( PortFD == NULL ) {
    return -1 ;
  };

  // Query the module for a device type
  glkputl( PortFD, GLKCommand, 0x37, EOF );
  i = glkget( PortFD );
  if( i < 0 ) {
    report( RPT_ERR, "glk: GLK did not respond to READ MODULE TYPE.\n" );
    return -1 ;
  } else {
    switch( i ) {
    case 0x10 :  // GLC12232
      width = 20 ; height = 4 ; break ;
    case 0x11 :  // GLC12864
      width = 20 ; height = 8 ; break ;
    case 0x12 :  // GLC128128
      width = 20 ; height = 16 ; break ;
    case 0x13 :  // GLC24064
      width = 40 ; height = 8 ; gpo_count = 1 ; break ;
    case 0x14 :  // GLK12864-25
      width = 20 ; height = 8 ; break ;
    case 0x15 :  // GLK24064-25
      width = 40 ; height = 8 ; gpo_count = 1 ; break ;
    case 0x21 :  // GLK128128-25
      width = 20 ; height = 16 ; break ;
    case 0x22 :  // GLK12232-25
      width = 20 ; height = 4 ; gpo_count = 2 ; break ;
    case 0x23 :  // GLK12232-25SM
      width = 20 ; height = 4 ; gpo_count = 2 ; break ;
    case 0x24 :  // GLK12232-25SM-Penguin
      width = 20 ; height = 4 ; gpo_count = 2 ; break ;
    default :
      report (RPT_ERR, "glk: Unrecognized module type: 0x%02x\n", i );
      return( -1 );
    };
  };
  driver->wid = width ;
  driver->hgt = height ;

  // You must use driver->framebuf here, but may use lcd.framebuf later.
  if(!driver->framebuf) {
    driver->framebuf = malloc(driver->wid * driver->hgt);
  };
  screen_contents = malloc( driver->wid * driver->hgt );

  if(driver->framebuf == NULL || screen_contents == NULL ) {
    report( RPT_ERR, "glk: Unable to allocate memory for screen buffers\n" );
    glk_close();
    return -1;
  }

  memset(driver->framebuf, ' ', driver->wid*driver->hgt);

/*  glk_clear();*/
/*  glkputl( PortFD, GLKCommand, 0x58, EOF );*/

  /* No font selected */
  fontselected = -1 ;

  /* Enable flow control*/
  glkflow( PortFD, 40, 2 );

  /* Set read character timeout to 0*/
  glktimeout( PortFD, 0 );

  /* Enable auto-transmit of up/down key events
   * This allows us to generate REPEAT keys distinct from
   *   normal keys using timeouts.  (see glk_getkey)
   */
  glkputl( PortFD, GLKCommand, 0x7e, 1, GLKCommand, 0x41, EOF );

  /* Set contrast */
  glk_contrast( contrast );

  driver->cellwid = 5;
  driver->cellhgt = 8;

  driver->clear = glk_clear;
  driver->string = glk_string;
  driver->chr = glk_chr;
  /* init_hbar and init_vbar don't seem to be needed by glk */
  driver->vbar = glk_vbar;
  driver->hbar = glk_hbar;
  driver->num = glk_num ;
  driver->init_num = glk_init_num ;

  driver->init = glk_init;
  driver->close = glk_close;
  driver->flush = glk_flush;
  driver->flush_box = glk_flush_box;
  driver->contrast = glk_contrast;
  driver->backlight = glk_backlight;
  driver->output = glk_output;
  driver->set_char = glk_set_char;
  driver->icon = glk_icon;
  driver->draw_frame = glk_draw_frame;

  driver->getkey = glk_getkey;


  return 200;  /* 200 is arbitrary.  (must be 1 or more)*/
}


/* Below here, you may use either lcd.framebuf or driver->framebuf..
 * lcd.framebuf will be set to the appropriate buffer before calling
 * your driver.
 */

static void glk_close()
{
  if(glk->framebuf != NULL) free(glk->framebuf);

  glk->framebuf = NULL;

  glkclose( PortFD ) ;

  report (RPT_INFO, "glk: closed");
}

/**************************************************************
 * Clears the LCD screen
 */
#define CLEARCOUNT  (1000000)
static int  clearcount = 0 ;
static void glk_clear_forced()
{
  debug(RPT_DEBUG, "REALLY CLEARING the display" );
  clearcount = CLEARCOUNT ;
  glkputl( PortFD, GLKCommand, 0x58, EOF );
  memset(screen_contents, ' ', glk->wid*glk->hgt);
}
static void glk_clear()
{
  debug(RPT_DEBUG, "glk_clear( )" );
  memset(glk->framebuf, ' ', glk->wid*glk->hgt);
  if( --clearcount < 0 ) {
    glk_clear_forced( );
  };
}


/***************************************************************
 * Flushes all output to the lcd...
 */
static void glk_flush()
{
   debug(RPT_DEBUG, "glk_flush( )" );
   glk->draw_frame(glk->framebuf);
}


/****************************************************************
 * Prints a string on the lcd display, at position (x,y).  The
 * upper-left is (1,1), and the lower right should be (20,4).
 */
static void glk_string(int x, int y, char string[])
{
  char *  p ;

  debug(RPT_DEBUG, "glk_string( %d, %d, \"%s\" )", x, y, string );

  if( x > glk->wid || y > glk->hgt ) {
     return ;
  };

  for( p = string ; *p && x <= glk->wid ; ++x, ++p ) {
    glk_chr( x, y, *p );
  };

}

/******************************************************************
 * Prints a character on the lcd display, at position (x,y).  The
 * upper-left is (1,1), and the lower right should be (20,4).
 */
static void glk_chr(int x, int y, char c)
{
  int  myc = (unsigned char) c ;
  x -= 1;  /* Convert 1-based coords to 0-based...*/
  y -= 1;

  if( fontselected != 2 ) {
    debug(RPT_DEBUG, "Switching to font 2" );
    /* Select font 2 */
    glkputl( PortFD, GLKCommand, 0x31, 2, EOF );
    fontselected = 2 ;
    /* Set font metrics */
    glkputl( PortFD, GLKCommand, 0x32, 1, 0, 1, 1, 32, EOF );
    /* Clear the screen */
    glk_clear_forced( );
  };

  if( myc >= 0 && myc <= 15 ) {
    /* CGRAM */
/*    if( myc != 0 ) {
 *      printf( "CGRAM changing %d => %d\n", myc, CGRAM[myc&7] );
 *    };
 */
    myc = CGRAM[myc&7] ;
  } else if( myc == 255 || myc == -1 ) {
    /* Solid block */
    myc = 133 ;
  } else if( (myc > 15 && myc < 32) || myc > 143 ) {
    debug(RPT_DEBUG, "Attempt to write %d to (%d,%d)\n", myc, x, y );
    myc = 133 ;
  };

  glk->framebuf[(y*glk->wid) + x] = myc;

}


/***********************************************************************
 * Sets the contrast of the display.  Value is 0-255, where 140 is
 * what I consider "just right".
 */
static int glk_contrast(int contrast)
{
  static int  saved_contrast = 140 ;

  if( contrast > 0 && contrast < 256 ) {
    saved_contrast = contrast ;
    report(RPT_DEBUG, "Contrast: %i\n", contrast);
    glkputl( PortFD, GLKCommand, 0x50, contrast, EOF );
  };
  return( saved_contrast );
}

/************************************************************************
 * Turns the lcd backlight on or off...
 */
static void glk_backlight(int on)
{
  if(on) {
    report(RPT_DEBUG, "Backlight ON\n");
    glkputl( PortFD, GLKCommand, 0x42, 0, EOF );
  } else {
    report(RPT_DEBUG, "Backlight OFF\n");
    glkputl( PortFD, GLKCommand, 0x46, EOF );
  }
}

/************************************************************************
 * Sets general purpose outputs on or off
 */
static void
glk_output(int on)
{
  if( gpo_count < 2 ) {
    if( on ) {  glkputl( PortFD, GLKCommand, 'W', EOF );
    } else {    glkputl( PortFD, GLKCommand, 'V', EOF );
    };
  } else {
    int  i;
    for( i = 1 ; i <= gpo_count ; ++i, on >>= 1 ) {
      if( on & 1 ) {
        glkputl( PortFD, GLKCommand, 'W', i, EOF );
      } else {
        glkputl( PortFD, GLKCommand, 'V', i, EOF );
      };
    };
  };
}

/*********************************************************************
 * Tells the driver to get ready for big numbers, if possible.
 */
static void glk_init_num()
{
  report(RPT_DEBUG, "Big Numbers.\n");
  if( fontselected != 3 ) {
    /* Select Big Numbers font */
    glkputl( PortFD, GLKCommand, 0x31, 3, EOF );
    fontselected = 3 ;
    /* Set font metrics */
    glkputl( PortFD, GLKCommand, 0x32, 1, 0, 1, 1, 32, EOF );
    /* Clear the screen */
    glk_clear_forced( );
  };
}

/*********************************************************************
 * Draws a big (4-row) number.
 */
static void glk_num(int x, int num)
{
  debug(RPT_DEBUG, "BigNum(%i, %i)\n", x, num);
  glk->framebuf[x-1] = num + '0' ;
}

/*********************************************************************
 * Changes the font data of character n.
 */
static void glk_set_char(int n, char *dat)
{
  debug(RPT_DEBUG, "Set Character %i\n", n);
}

/********************************************************************
 * Draws a vertical bar, from the bottom of the screen up.
 */
static void glk_vbar(int x, int len)
{
  int  y = glk->hgt ;

  debug(RPT_DEBUG, "glk_vbar( %d, %d )\n", x, len );
  while( len > glk->cellhgt ) {
    glk_chr( x, y, 255 );
    --y ;
    len -= glk->cellhgt ;
  };

  if( y >= 0 ) {
    int  lastc ;
    switch( len ) {
    case 0 :  return ; break ;  /* Don't output a char */
    case 1 :  lastc = 138 ; break ;  /* One bar */
    case 2 :  lastc = 139 ; break ;
    case 3 :  lastc = 140 ; break ;
    case 4 :  lastc = 141 ; break ;
    case 5 :  lastc = 142 ; break ;
    case 6 :  lastc = 143 ; break ;
    default:  lastc = 133 ; break ;
    };
    glk_chr( x, y, lastc );
  };
}

/************************************************************
 * Draws a horizontal bar to the right.
 */
static void glk_hbar(int x, int y, int len)
{
  debug(RPT_DEBUG, "glk_hbar( %d, %d, %d )\n", x, y, len );
  while( len > glk->cellwid ) {
    glk_chr( x, y, 255 );
    ++x ;
    len -= glk->cellwid ;
  };

  if( x <= glk->wid ) {
    int  lastc ;
    switch( len ) {
    case 0 :  lastc = ' ' ; break ;
    case 1 :  lastc = 134 ; break ;  /* One bar */
    case 2 :  lastc = 135 ; break ;
    case 3 :  lastc = 136 ; break ;
    case 4 :  lastc = 137 ; break ;
    default:  lastc = 133 ; break ;
    };
    glk_chr( x, y, lastc );
  };
}


/*****************************************************************
 * Sets character 0 to an icon...
 */
static void glk_icon(int which, char dest)
{
  unsigned char  old, new ;
  unsigned char *  p ;
  unsigned char *  q ;
  int  count ;

  debug(RPT_DEBUG, "Char %i set to icon %i\n", dest, which);

  if( dest < 0 || dest > 7 ) {
    /* Illegal custom character */
    return ;
  };

  /* which == 0  => empty heart   => 131
   * which == 1  => filled heart  => 132
   * which == 2  => ellipsis      => 128
   */
  switch( which ) {
  case 0:  new = 131 ; break ;
  case 1:  new = 132 ; break ;
  case 2:  new = 128 ; break ;
  default:  return ;  /* ERROR */
  };

  old = CGRAM[(int)dest] ;
  CGRAM[(int)dest] = new ;
  p = glk->framebuf ;
  q = screen_contents ;

  /* Replace all old icons with new icon in new frame */
  for( count = glk->wid * glk->hgt ; count ; --count ) {
    if( *q == old ) {
      debug(RPT_DEBUG, "icon %d to %d at %d\n", old, new, q - screen_contents );
      *p = new ;
    };
    ++q ; ++p ;
  };

  return ;
}


/****************************************************************************
 * Send a rectangular area to the display.
 *
 * I've just called glk_flush() because there's not much point yet
 * in flushing less than the entire framebuffer.
 */
static void glk_flush_box(int lft, int top, int rgt, int bot)
{
   debug(RPT_DEBUG,"glk_flush_box( %d, %d, %d, %d )\n", lft, top, rgt, bot );
   glk_flush( );
}

/*****************************************************************************
 * Draws the framebuffer on the display.
 *
 * The commented-out code is from the text driver.
 */
static void glk_draw_frame(char *dat)
{
  char *  p ;
  char *  q ;
  int  x, y ;
  int  xs ;
  char *  ps = NULL ;

  debug(RPT_DEBUG, "glk_draw_frame( %p )  glk->framebuf = %p\n", dat, glk->framebuf );

  p = glk->framebuf ;
  q = screen_contents ;

  for( y = 0 ; y < glk->hgt ; ++y ) {
    xs = -1 ;  /* XStart not set */
    for( x = 0 ; x < glk->wid ; ++x ) {
      if( *q == *p && xs >= 0 ) {
        /* Write accumulated string */
        glkputl( PortFD, GLKCommand, 0x79, xs*6+1, y*8, EOF );
        glkputa( PortFD, x - xs, ps );
        debug(RPT_DEBUG, "draw_frame: Writing at (%d,%d) for %d\n", xs, y, x-xs );
        xs = -1 ;
      } else if( *q != *p && xs < 0 ) {
        /* Start new string of changes */
        ps = p ;
        xs = x ;
      };
      *q++ = *p++ ;  /* Update screen_contents from glk->framebuf */
    };
    if( xs >= 0 ) {
      /* Write accumulated line */
      glkputl( PortFD, GLKCommand, 0x79, xs*6+1, y*8, EOF );
      glkputa( PortFD, glk->wid - xs, ps );
      debug(RPT_DEBUG, "draw_frame: Writing at (%d,%d) for %d\n", xs, y, glk->wid-xs );
    };

  };  /* For y */

  return ;
}


/**************************************************************************************
 * Tries to read a character from an input device...
 *
 * Return 0 for "nothing available".
 */
static char glk_getkey()
{
  int  c ;
  static int  key = -1 ;
  static struct timeval  lastkey ;
  struct timeval  now ;

  debug(RPT_DEBUG, "glk_getkey( )" );

  c = glkgetc( PortFD );

  if( c >= 'A' && c <= 'Z' ) {
    /* Key down event */
    key = c ;
    gettimeofday( &lastkey, NULL );
    debug(RPT_DEBUG, "KEY %c at %ld.%06ld\n", c, lastkey.tv_sec, lastkey.tv_usec );
  } else if( c >= 'a' && c <= 'z' ) {
    /* Key up event */
    debug(RPT_DEBUG, "KEY %c UP\n", c );
    key = -1 ;
    c = 0 ;
  } else {
    /* Assume timeout */
    c = 0 ;
    if( key > 0 ) {
      int  msec_diff ;
      /* A key is down */
      gettimeofday( &now, NULL );
      msec_diff  = (now.tv_sec - lastkey.tv_sec) * 1000 ;
      msec_diff += (now.tv_usec - lastkey.tv_usec) / 1000 ;
      debug(RPT_DEBUG, "KEY %c down for %d msec\n", key, msec_diff );
      if( msec_diff > 1000 ) {
        /* Generate repeat event */
        c = key | 0x20 ;  /* Upper case to lower case */
        ++lastkey.tv_sec ;  /* HACK HACK. repeat at 1 sec intervals */
        debug(RPT_DEBUG, "KEY %c REPEAT\n", c );
      };
    };
  };

  /* Remap keys according to what LCDproc expects */
  switch( c ) {
  default :  break ;
  case 'V' : c = INPUT_PAUSE_KEY ; break ; /* Hold/Select */
  case 'P' : c = INPUT_BACK_KEY ; break ; /* Left  -- Minus */
  case 'Q' : c = INPUT_FORWARD_KEY ; break ; /* Right -- Plus */
  case 'L' : c = INPUT_MAIN_MENU_KEY ; break ; /* Menu/Exit */

  /* NOTE: The following keys are IGNORED by the server! */

  case 'U' : c = 'E' ; break ; /* Up */
  case 'K' : c = 'F' ; break ; /* Down */

  case 'v' : c = 'N' ; break ;
  case 'p' : c = 'O' ; break ;
  case 'q' : c = 'P' ; break ;
  case 'l' : c = 'Q' ; break ;
  case 'u' : c = 'R' ; break ;
  case 'k' : c = 'S' ; break ;
  };

  if( c ) {
    debug(RPT_DEBUG, "KEY %c\n", c );
  };

  return( c );
}
