/** \file server/drivers/debug.c
 * \c debug driver for LCDd.
 * This driver does does nothing more than writing a debug message when
 * any one of the driver's functions is called.
 */

/*  Copyright (C) ?????? ?????????
                  2008, Peter Marschall

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
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 */

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

#include "lcd.h"
#include "debug.h"
#include "report.h"


#define DEFAULT_WIDTH		LCD_DEFAULT_WIDTH
#define DEFAULT_HEIGHT		LCD_DEFAULT_HEIGHT
#define DEFAULT_CELLWIDTH	LCD_DEFAULT_CELLWIDTH
#define DEFAULT_CELLHEIGHT	LCD_DEFAULT_CELLHEIGHT
#define DEFAULT_CONTRAST	500
#define DEFAULT_BRIGHTNESS	750
#define DEFAULT_OFFBRIGHTNESS	250


typedef struct debug_private_data {
	char *framebuf;
	int width;
	int height;
	int cellwidth;
	int cellheight;
	int contrast;
	int brightness;
	int offbrightness;
} PrivateData;


// Vars for the server core
MODULE_EXPORT char *api_version = API_VERSION;
MODULE_EXPORT int stay_in_foreground = 1;
MODULE_EXPORT int supports_multiple = 0;
MODULE_EXPORT char *symbol_prefix = "debug_";



/**
 * Initialize the driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0   Success.
 * \retval <0  Error.
 */
MODULE_EXPORT int
debug_init (Driver *drvthis)
{
	PrivateData *p;

	report(RPT_INFO, "%s()", __FUNCTION__);

        /* Allocate and store private data */
	p = (PrivateData *) calloc(1, sizeof(PrivateData));
	if (p == NULL)
		return -1;
	if (drvthis->store_private_ptr(drvthis, p))
		return -1;

	p->width = DEFAULT_WIDTH;
	p->height = DEFAULT_HEIGHT;
	p->cellwidth = DEFAULT_CELLWIDTH;
	p->cellheight = DEFAULT_CELLHEIGHT;
	p->contrast = DEFAULT_CONTRAST;
	p->brightness = DEFAULT_BRIGHTNESS;
	p->offbrightness = DEFAULT_OFFBRIGHTNESS;

	p->framebuf = malloc(p->width * p->height);
	if (p->framebuf == NULL) {
		report(RPT_INFO, "%s: unable to allocate framebuffer", drvthis->name);
		return -1;
	}	

	debug_clear(drvthis);

	return 0;
}


/**
 * Close the driver (do necessary clean-up).
 * \param drvthis  Pointer to driver structure.
 */
MODULE_EXPORT void
debug_close (Driver *drvthis)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_INFO, "%s()", __FUNCTION__);

	if (p != NULL) {
		if (p->framebuf != NULL)
			free(p->framebuf);
		p->framebuf = NULL;

		free(p);
	}
	drvthis->store_private_ptr(drvthis, NULL);
}


/**
 * Return the display width in characters.
 * \param drvthis  Pointer to driver structure.
 * \return  Number of characters the display is wide.
 */
MODULE_EXPORT int
debug_width (Driver *drvthis)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_INFO, "%s()", __FUNCTION__);

	return p->width;
}


/**
 * Return the display height in characters.
 * \param drvthis  Pointer to driver structure.
 * \return  Number of characters the display is high.
 */
MODULE_EXPORT int
debug_height (Driver *drvthis)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_INFO, "%s()", __FUNCTION__);

	return p->height;
}


/**
 * Return the width of a character in pixels.
 * \param drvthis  Pointer to driver structure.
 * \return  Number of pixel columns a character cell is wide.
 */
MODULE_EXPORT int
debug_cellwidth (Driver *drvthis)
{
        PrivateData *p = drvthis->private_data;

        return p->cellwidth;
}


/**
 * Return the height of a character in pixels.
 * \param drvthis  Pointer to driver structure.
 * \return  Number of pixel lines a character cell is high.
 */
MODULE_EXPORT int
debug_cellheight (Driver *drvthis)
{
        PrivateData *p = drvthis->private_data;

        return p->cellheight;
}


/**
 * Clear the screen.
 * \param drvthis  Pointer to driver structure.
 */
MODULE_EXPORT void
debug_clear (Driver *drvthis)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_INFO, "%s()", __FUNCTION__);

	memset (p->framebuf, ' ', p->width * p->height);

}


/**
 * Flush data on screen to the display.
 * \param drvthis  Pointer to driver structure.
 */
MODULE_EXPORT void
debug_flush (Driver *drvthis)
{
	PrivateData *p = drvthis->private_data;
	int i, j;
	char out[LCD_MAX_WIDTH];

	report(RPT_INFO, "%s()", __FUNCTION__);

	for (i = 0; i < p->width; i++) {
		out[i] = '-';
	}
	out[p->width] = 0;
	//report(RPT_DEBUG, "+%s+", out);

	for (i = 0; i < p->height; i++) {
		for (j = 0; j < p->width; j++) {
			out[j] = p->framebuf[j + (i * p->width)];
		}
		out[p->width] = 0;
		//report(RPT_DEBUG, "|%s|", out);

	}

	for (i = 0; i < p->width; i++) {
		out[i] = '-';
	}
	out[p->width] = 0;
	//report(RPT_DEBUG, "+%s+", out);
}


/**
 * Print a string on the screen at position (x,y).
 * The upper-left corner is (1,1), the lower-right corner is (p->width, p->height).
 * \param drvthis  Pointer to driver structure.
 * \param x        Horizontal character position (column).
 * \param y        Vertical character position (row).
 * \param string   String that gets written.
 */
MODULE_EXPORT void
debug_string (Driver *drvthis, int x, int y, const char string[])
{
	PrivateData *p = drvthis->private_data;
	int i;

	report(RPT_INFO, "%s(%i,%i,%.40s)", __FUNCTION__, x, y, string);

	y --; x --;  // Convert 1-based coords to 0-based...

	if ((y < 0) || (y >= p->height))
		return;

	for (i = 0; (string[i] != '\0') && (x < p->width); i++, x++) {
		if (x >= 0)	// no write left of left border
			p->framebuf[(y * p->width) + x] = string[i];
	}
}


/**
 * Print a character on the screen at position (x,y).
 * The upper-left corner is (1,1), the lower-right corner is (p->width, p->height).
 * \param drvthis  Pointer to driver structure.
 * \param x        Horizontal character position (column).
 * \param y        Vertical character position (row).
 * \param c        Character that gets written.
 */
MODULE_EXPORT void
debug_chr (Driver *drvthis, int x, int y, char c)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_DEBUG, "%s(%i,%i,%c)", __FUNCTION__, x, y, c);

	x--; y--;

	if ((x >= 0) && (y >= 0) && (x < p->width) && (y < p->height))
		p->framebuf[(y * p->width) + x] = c;
}


/**
 * Draw a vertical bar bottom-up.
 * \param drvthis  Pointer to driver structure.
 * \param x        Horizontal character position (column) of the starting point.
 * \param y        Vertical character position (row) of the starting point.
 * \param len      Number of characters that the bar is high at 100%
 * \param promille Current height level of the bar in promille.
 * \param options  Options (currently unused).
 */
MODULE_EXPORT void
debug_vbar (Driver *drvthis, int x, int y, int len, int promille, int options)
{
	//PrivateData *p = drvthis->private_data;
	int pos;

	report(RPT_INFO, "%s(%i,%i,%i,%i,%i)", __FUNCTION__, x, y, len, promille, options);

	for (pos = 0; pos < len; pos++) {
		if (2 * pos < ((long) promille * len / 500 + 1)) {
			debug_chr(drvthis, x, y-pos, '|');
		} else {
			; /* print nothing */
		}
	}
}


/**
 * Draw a horizontal bar to the right.
 * \param drvthis  Pointer to driver structure.
 * \param x        Horizontal character position (column) of the starting point.
 * \param y        Vertical character position (row) of the starting point.
 * \param len      Number of characters that the bar is long at 100%
 * \param promille Current length level of the bar in promille.
 * \param options  Options (currently unused).
 */
MODULE_EXPORT void
debug_hbar (Driver *drvthis, int x, int y, int len, int promille, int options)
{
	//PrivateData *p = drvthis->private_data;
	int pos;

	report(RPT_INFO, "%s(%i,%i,%i,%i,%i)", __FUNCTION__, x, y, len, promille, options);

	for (pos = 0; pos < len; pos++) {
		if (2 * pos < ((long) promille * len / 500 + 1)) {
			debug_chr (drvthis, x+pos, y, '-');
		} else {
			; /* print nothing */
		}
	}
}


/**
 * Write a big number to the screen.
 * \param drvthis  Pointer to driver structure.
 * \param x        Horizontal character position (column).
 * \param num      Character to write (0 - 10 with 10 representing ':')
 */
MODULE_EXPORT void
debug_num (Driver *drvthis, int x, int num)
{
	//PrivateData *p = drvthis->private_data;
	report(RPT_INFO, "%s(%i,%i)", __FUNCTION__, x, num);
}


/**
 * Place an icon on the screen.
 * \param drvthis  Pointer to driver structure.
 * \param x        Horizontal character position (column).
 * \param y        Vertical character position (row).
 * \param icon     synbolic value representing the icon.
 * \return  Information whether the icon is handled here or needs to be handled by the server core.
 */
MODULE_EXPORT int
debug_icon (Driver *drvthis, int x, int y, int icon)
{
	//PrivateData *p = drvthis->private_data;
	report(RPT_INFO, "%s(%i,%i,%i)", __FUNCTION__, x, y, icon);

	return -1;	/* let the core do all the icon stuff */
}


/**
 * Define a custom character and write it to the LCD.
 * \param drvthis  Pointer to driver structure.
 * \param n        Custom character to define [0 - (NUM_CCs-1)].
 * \param dat      Array of 8 (=cellheight) bytes, each representing a pixel row
 *                 starting from the top to bottom.
 *                 The bits in each byte represent the pixels where the LSB
 *                 (least significant bit) is the rightmost pixel in each pixel row.
 */
MODULE_EXPORT void
debug_set_char (Driver *drvthis, int n, char *dat)
{
	//PrivateData *p = drvthis->private_data;
	report(RPT_INFO, "%s(%i,data)", __FUNCTION__, n);
}


/**
 * Get current LCD contrast.
 * This is only the locally stored contrast, the contrast value
 * cannot be retrieved from the LCD.
 * \param drvthis  Pointer to driver structure.
 * \return  Stored contrast in promille.
 */
MODULE_EXPORT int
debug_get_contrast (Driver *drvthis)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_INFO, "%s()", __FUNCTION__);

	return p->contrast;
}


/**
 * Change LCD contrast.
 * \param drvthis  Pointer to driver structure.
 * \param promille New contrast value in promille.
 */
MODULE_EXPORT void
debug_set_contrast (Driver *drvthis, int promille)
{
	PrivateData *p = drvthis->private_data;

	report(RPT_INFO, "%s(%i)", __FUNCTION__, promille);

	p->contrast = promille;
}


/**
 * Retrieve brightness.
 * \param drvthis  Pointer to driver structure.
 * \param state    Brightness state (on/off) for which we want the value.
 * \return Stored brightness in promille.
 */
MODULE_EXPORT int
debug_get_brightness(Driver *drvthis, int state)
{
        PrivateData *p = drvthis->private_data;

        return (state == BACKLIGHT_ON) ? p->brightness : p->offbrightness;
}


/**
 * Set on/off brightness.
 * \param drvthis  Pointer to driver structure.
 * \param state    Brightness state (on/off) for which we want to store the value.
 * \param promille New brightness in promille.
 */
MODULE_EXPORT void
debug_set_brightness(Driver *drvthis, int state, int promille)
{
        PrivateData *p = drvthis->private_data;

        /* Check it */
        if (promille < 0 || promille > 1000)
                return;

        /* store the software value since there is not get */
        if (state == BACKLIGHT_ON) {
                p->brightness = promille;
        }
        else {
                p->offbrightness = promille;
        }
        debug_backlight(drvthis, state);
}

/**
 * Turn the backlight on or off.
 * \param drvthis  Pointer to driver structure.
 * \param on       New backlight status.
 */
MODULE_EXPORT void
debug_backlight (Driver *drvthis, int on)
{
	//PrivateData *p = drvthis->private_data;
	report(RPT_INFO, "%s(%i)", __FUNCTION__, on);
}


/**
 * Handle input from keyboard.
 * \param drvthis  Pointer to driver structure.
 * \return  String representation of the key.
 */
MODULE_EXPORT const char *
debug_get_key (Driver *drvthis)
{
	//PrivateData *p = drvthis->private_data;
	report(RPT_INFO, "%s()", __FUNCTION__);

	return NULL;
}


/**
 * Provide some information about this driver.
 * \param drvthis  Pointer to driver structure.
 * \return  Constant string with information.
 */
MODULE_EXPORT const char *
debug_get_info(Driver *drvthis)
{
	//PrivateData *p = drvthis->private_data;
	static char *info_string = "debug driver";

	report(RPT_INFO, "%s()", __FUNCTION__);

        return info_string;
}

