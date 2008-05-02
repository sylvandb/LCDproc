/** \file hd44780-ftdi.c
 * \c ftdi connection type of \c hd44780 driver for Hitachi HD44780 based LCD displays.
 */

/*
 * FTDI/USB driver module for Hitachi HD44780 based LCD displays
 * connected to a FT2232C/D chip in 8 bit mode.
 *
 * Copyright (c) 2007, Intra2net AG <opensource@intra2net.com>
 * 
 * Modified February 2008 by Ismail Ibrahim <whirleyes@gmail.com> to
 * add support for single channel FTDI chip with 4 bit wiring mode.
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 *
 */

/* info for 4bit mote (ftdi_mode == 4):
  wiring example:
  FTDI chip: D0 D1 D2 D3 D4 D5 D6 D7
             |  |  |  |  |  |  |  |
  HD44780:   D4 D5 D6 D7 EN RS RW n/c

 in LCDd.conf we then need to define
  ftdi_line_RS=0x40
  ftdi_line_RW=0x20
  ftdi_line_EN=0x10
*/

#include "hd44780-ftdi.h"

#include "report.h"

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <usb.h>

#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

#include <errno.h>

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif


// connection type specific functions to be exposed using pointers in init()
void ftdi_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch);
void ftdi_HD44780_backlight(PrivateData *p, unsigned char state);
void ftdi_HD44780_close(PrivateData *p);


/**
 * Initialize the driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0   Success.
 * \retval -1  Error.
 */
int
hd_init_ftdi(Driver *drvthis)
{
    int vendor_id, product_id;
    int f;

    PrivateData *p = (PrivateData*) drvthis->private_data;

    p->hd44780_functions->senddata = ftdi_HD44780_senddata;
    p->hd44780_functions->backlight = ftdi_HD44780_backlight;
    p->hd44780_functions->close = ftdi_HD44780_close;

    // Load config
    vendor_id = drvthis->config_get_int(drvthis->name, "VendorID", 0, 0x0403);
    product_id = drvthis->config_get_int(drvthis->name, "ProductID", 0, 0x6001);

    // these config settings are not documented intentionally
    p->ftdi_mode = drvthis->config_get_int(drvthis->name, "ftdi_mode", 0, 8);
    p->ftdi_line_RS = drvthis->config_get_int(drvthis->name, "ftdi_line_RS", 0, 0x01);
    p->ftdi_line_RW = drvthis->config_get_int(drvthis->name, "ftdi_line_RW", 0, 0x02);
    p->ftdi_line_EN = drvthis->config_get_int(drvthis->name, "ftdi_line_EN", 0, 0x04);
    p->ftdi_line_backlight = drvthis->config_get_int(drvthis->name, "ftdi_line_backlight", 0, 0x08);

    //some foolproof check
    if (p->ftdi_mode != 4 && p->ftdi_mode != 8) {
	report(RPT_ERR, "invalid ftdi_mode: %d", p->ftdi_mode);
	return -1;
    }

    // Init 1. channel: data
    ftdi_init(&p->ftdic);
    ftdi_set_interface(&p->ftdic, INTERFACE_A);
    f = ftdi_usb_open(&p->ftdic, vendor_id, product_id);
    if (f < 0 && f != -5) {
	report(RPT_ERR, "unable to open ftdi device: %d (%s)", f, ftdi_get_error_string(&p->ftdic));
	return -1;
    }
    debug(RPT_DEBUG, "ftdi open succeeded(channel 1): %d", f);

    debug(RPT_DEBUG, "enabling bitbang mode(channel 1)\n");

    if (p->ftdi_mode == 4) {
	//set fast baudrate for 4 bit wiring to eliminate garbage on lcd
	f = ftdi_set_baudrate(&p->ftdic, 921600);
	if (f < 0) {
	    report(RPT_ERR, "unable to open ftdi device: %d (%s)", f, ftdi_get_error_string(&p->ftdic));
	    return -1;
	}
    }

    ftdi_enable_bitbang(&p->ftdic, 0xFF);

    if (p->ftdi_mode == 8) {
	// Init 2. channel: control
	ftdi_init(&p->ftdic2);
	ftdi_set_interface(&p->ftdic2, INTERFACE_B);
	f = ftdi_usb_open(&p->ftdic2, vendor_id, product_id);
	if (f < 0 && f != -5) {
	    report(RPT_ERR, "unable to open second ftdi device: %d (%s)", f, ftdi_get_error_string(&p->ftdic2));
	    return -2;
	}
	debug(RPT_DEBUG, "ftdi open succeeded(channel 2): %d", f);

	debug(RPT_DEBUG, "enabling bitbang mode (channel 2)");
	ftdi_enable_bitbang(&p->ftdic2, 0xFF);

	// FTDI bug: Sometimes first write gets lost on kernel 2.6, needs investigation.
	ftdi_HD44780_senddata (p, 0, RS_INSTR, FUNCSET | IF_8BIT);
	usleep (4100);

	common_init(p, IF_8BIT);
    }
    else if (p->ftdi_mode == 4) {
	ftdi_HD44780_senddata (p, 0, RS_INSTR, FUNCSET | IF_4BIT );
	ftdi_HD44780_senddata (p, 0, RS_INSTR, FUNCSET | IF_4BIT );
	ftdi_HD44780_senddata (p, 0, RS_INSTR, FUNCSET | IF_4BIT );

	common_init(p, IF_4BIT);
    }


    return 0;
}


// ftdi_HD44780_senddata
void
ftdi_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch)
{
    if (p->ftdi_mode == 8) {
	// Output data on first channel
	int f = ftdi_write_data(&p->ftdic, &ch, 1);
	if (f < 0) {
	    p->hd44780_functions->drv_report(RPT_ERR, "failed to write: %d (%s). Exiting",
    					f, ftdi_get_error_string(&p->ftdic));
	    exit (-1);
	}

	// Setup RS and R/W and EN
	ch = p->ftdi_line_EN | p->backlight_bit;
	if (flags == RS_DATA) {
	    ch |= p->ftdi_line_RS;
	}
	f = ftdi_write_data(&p->ftdic2, &ch, 1);
	if (f < 0) {
	    p->hd44780_functions->drv_report(RPT_ERR, "failed to write: %d (%s). Exiting",
    					f, ftdi_get_error_string(&p->ftdic2));
	    exit(-1);
	}

	// Disable E
	ch = 0x00 | p->backlight_bit;
	if (flags == RS_DATA) {
	    ch |= p->ftdi_line_RS;
	}
	f = ftdi_write_data(&p->ftdic2, &ch, 1);
	if (f < 0) {
	    p->hd44780_functions->drv_report(RPT_ERR, "failed to write: %d (%s). Exiting",
    					f, ftdi_get_error_string(&p->ftdic2));
	    exit(-1);
	}
    }
    else if (p->ftdi_mode == 4) {
	unsigned char buf[4];
	unsigned char portControl = 0;

	if (flags == RS_DATA) {
	    portControl = p->ftdi_line_RS;
	}

	buf[0] = (ch>>4) & 0x0F | portControl | p->ftdi_line_EN;
	buf[1] = (ch>>4) & 0x0F | portControl;
	buf[2] = (ch & 0x0F) | portControl | p->ftdi_line_EN;
	buf[3] = (ch & 0x0F) | portControl;
	int f = ftdi_write_data(&p->ftdic, &buf, 4);

	if (f < 0) {
	     p->hd44780_functions->drv_report(RPT_ERR, "failed to write: %d (%s). Exiting",
	                                     f, ftdi_get_error_string(&p->ftdic));
	     exit(-1);
	}

	if (flags == RS_INSTR) {
 	    usleep (4100);
	}
    }
}


void
ftdi_HD44780_backlight(PrivateData *p, unsigned char state)
{
    if (p->ftdi_mode == 8) {
	int f;

	p->backlight_bit = state ? p->ftdi_line_backlight : 0;

	f = ftdi_write_data(&p->ftdic2, &state, 1);
	if (f < 0) {
	    p->hd44780_functions->drv_report(RPT_ERR, "failed to write: %d (%s). Exiting",
					    f, ftdi_get_error_string(&p->ftdic2));
	    exit(-1);
	}
    }
}


void
ftdi_HD44780_close(PrivateData *p)
{
    ftdi_disable_bitbang(&p->ftdic);
    ftdi_usb_close(&p->ftdic);
    ftdi_deinit(&p->ftdic);

    if (p->ftdi_mode == 8) {
	ftdi_disable_bitbang(&p->ftdic2);
	ftdi_usb_close(&p->ftdic2);
	ftdi_deinit(&p->ftdic2);
    }
}

/* EOF */
