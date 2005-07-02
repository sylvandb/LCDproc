/*  This is the LCDproc driver for LIRC infrared devices (http://www.lirc.org)

    Copyright (C) 2000, Harald Klein
		  2002, Rene Wagner

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
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
*/

#ifndef LCD_LIRCIN_H
#define LCD_LIRCIN_H

MODULE_EXPORT int lircin_init (Driver *drvthis);
MODULE_EXPORT void lircin_close (Driver *drvthis);
MODULE_EXPORT char * lircin_get_key (Driver *drvthis);

#define LIRCIN_VERBOSELY 0

#define LIRCIN_DEF_PROG "lcdd"

#endif
