/*
 * menu.c
 * This file is part of LCDd, the lcdproc server.
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 *
 * Copyright (c) 1999, William Ferrell, Scott Scriven
 *
 *
 * Handles server-supplied menus defined by a table.  Read menu.h for
 * more information.
 *
 * Menus are similar to "pull-down" menus, but have some extra features.
 * They can contain "normal" menu items, checkboxes, sliders, "movers",
 * etc..
 *
 * I should probably find a more elegant way of doing this in order
 * to handle dynamically-changing menus such as the client list.  Tcl/Tk
 * has neat ways to do it.  Hmm...
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "parse.h"
#include "sock.h"
#include "render.h"
#include "main.h"

#include "drivers/lcd.h"
#include "menu.h"

// FIXME: Implement this where it is supposed to be...
void
framedelay ()
{
	sock_poll_clients ();
	parse_all_client_messages ();

	usleep (TIME_UNIT);
}

static void
draw_heartbeat ()
{
	static int timer = 0;

	if (heartbeat) {
		// Set this to pulsate like a real heart beat...
		// (binary is fun...  :)
		lcd_ptr->icon (!((timer + 4) & 5), 0);
		lcd_ptr->chr (lcd_ptr->wid, 1, 0);
	}
	lcd_ptr->flush ();

	timer++;
	timer &= 0x0f;
}

static int PAD = 255;

typedef struct menu_info {
	int selected;
	int length;
} menu_info;

static int draw_menu (Menu menu, menu_info * info);
static int fill_menu_info (Menu menu, menu_info * info);
static int menu_handle_action (menu_item * item);

static int slid_func (menu_item * item);

int
do_menu (Menu menu)
{
	menu_info info;
	int key = 0;
	int status = MENU_OK;
	int done = 0;

	int (*func) ();
	int (*readfunc) (int);

	if (!menu)
		return MENU_ERROR;

	fill_menu_info (menu, &info);

	while (!done) {
		// Keep the cursor off titles... (?)
		while (menu[info.selected].type == TYPE_TITL) {
			info.selected++;
			// If the title is the last thing in the menu...
			if (!menu[info.selected].text)
				info.selected -= 2;
		}

		draw_menu (menu, &info);

		// FIXME: This should use a better keypress interface, which
		// FIXME: handles things according to keybindings...

		for (key = lcd_ptr->getkey (); key == 0; key = lcd_ptr->getkey ()) {
			// sleep for 1/8th second...
			framedelay ();
			// do the heartbeat...
			draw_heartbeat ();
			// Check for client input...
		}

		// Handle the key according to the keybindings...
		switch (key) {
		case 'D':
			done = 1;
			break;
		case 'B':
			if (info.selected > 0)
				info.selected--;
			while (menu[info.selected].type == TYPE_TITL) {
				if (info.selected > 0)
					info.selected--;
				else
					break;
			}
			break;
		case 'C':
			if (menu[info.selected + 1].text)
				info.selected++;
			break;
		case 'A':
			switch (menu[info.selected].type) {
			case TYPE_MENU:
				status = do_menu (menu[info.selected].data);
				break;
			case TYPE_FUNC:
				func = menu[info.selected].data;
				if (func)
					status = func ();
				break;
			case TYPE_CHEK:
				readfunc = menu[info.selected].data;
				if (readfunc)
					status = readfunc (MENU_CHECK);
				status &= 0xffff0000;
				break;
			case TYPE_SLID:
				func = menu[info.selected].data;
				if (func)
					status = slid_func (&menu[info.selected]);
				break;
			default:
				break;
			}

			switch (status) {
			case MENU_OK:
				break;
			case MENU_CLOSE:
				return MENU_OK;
			case MENU_QUIT:
				return MENU_QUIT;
//        case MENU_KILL:
//          return MENU_KILL;
			case MENU_ERROR:
				return MENU_ERROR;
			}

			// status = menu_handle_action(&menu[info.selected]);
			// TODO: It should now do special stuff for "mover" widgets,
			// TODO: and handle the return code appropriately.
			break;
		default:
			break;
		}

	}

	return status;

}

static int
draw_menu (Menu menu, menu_info * info)
{
	int i;
	int x = 1, y = 1;
	int top = 0, bottom = 0;

	int (*readfunc) (int);

	// these should maybe be removed:
	int wid = lcd_ptr->wid, hgt = lcd_ptr->hgt;

	if (!menu)
		return MENU_ERROR;

	lcd_ptr->clear ();

	// Scroll down until the selected item is centered, if possible...
	top = info->selected - (hgt / 2);
	if (top < 0)
		top = 0;
	bottom = top + hgt;
	if (bottom > info->length)
		bottom = info->length;
	top = bottom - hgt;
	if (top < 0)
		top = 0;

	// Draw all visible items...
	for (i = top; i < bottom; i++, y++) {
		if (i == info->selected)
			lcd_ptr->chr (2, y, '>');

		switch (menu[i].type) {
		case TYPE_TITL:
			lcd_ptr->chr (1, y, PAD);
			lcd_ptr->chr (2, y, PAD);
			lcd_ptr->string (4, y, menu[i].text);
			for (x = strlen (menu[i].text) + 5; x <= wid; x++)
				lcd_ptr->chr (x, y, PAD);
			break;
		case TYPE_MENU:
			lcd_ptr->string (3, y, menu[i].text);
			lcd_ptr->chr (wid, y, '>');
			break;
		case TYPE_FUNC:
			lcd_ptr->string (3, y, menu[i].text);
			break;
		case TYPE_CHEK:
			if (menu[i].data) {
				readfunc = menu[i].data;
				if (readfunc (MENU_READ))
					lcd_ptr->chr (wid, y, 'Y');
				else
					lcd_ptr->chr (wid, y, 'N');
			}
			lcd_ptr->string (3, y, menu[i].text);
			break;
		case TYPE_SLID:
			lcd_ptr->string (3, y, menu[i].text);
			break;
		case TYPE_MOVE:
			break;
		default:
			break;
		}
	}

	if (top != 0)
		lcd_ptr->chr (1, 1, '^');
	if (bottom < info->length)
		lcd_ptr->chr (1, hgt, 'v');

	draw_heartbeat ();
	//lcd_ptr->flush();

	return 0;
}

static int
fill_menu_info (Menu menu, menu_info * info)
{
	int i;

	info->selected = 0;

	// count the entries in the menu
	for (i = 0; menu[i].text; i++);

	info->length = i;

	return 0;

}

static int
menu_handle_action (menu_item * item)
{
	return MENU_OK;
}

static int
slid_func (menu_item * item)
{
	char str[16];
	int key = 0;
	int value = 0;
	int x, y = 1;
	int (*readfunc) (int);

	readfunc = item->data;

	lcd_ptr->init_hbar ();

	while (key != 'A' && key != 'D') {
		// Draw the title...
		lcd_ptr->clear ();
		lcd_ptr->chr (1, y, PAD);
		lcd_ptr->chr (2, y, PAD);
		lcd_ptr->string (4, y, item->text);
		for (x = strlen (item->text) + 5; x <= lcd_ptr->wid; x++)
			lcd_ptr->chr (x, y, PAD);

		// Draw the slider now...
		value = readfunc (MENU_READ);
		if (value < 0 || value >= MENU_CLOSE)
			return value;
		snprintf (str, sizeof(str), "%i", value);
		if (lcd_ptr->hgt >= 4) {
			lcd_ptr->string (8, 4, str);
			value = (lcd_ptr->wid * lcd_ptr->cellwid * value / 256);
			lcd_ptr->hbar (1, 3, value);
		} else {
			lcd_ptr->string (17, 2, str);
			value = ((lcd_ptr->wid - 4) * lcd_ptr->cellwid * value / 256);
			lcd_ptr->hbar (1, 2, value);
		}
		//lcd_ptr->flush();

		for (key = lcd_ptr->getkey (); key == 0; key = lcd_ptr->getkey ()) {
			// do the heartbeat...
			draw_heartbeat ();
			// sleep for 1/8th second...
			framedelay ();
			// Check for client input...
		}

		switch (key) {
		case 'B':
			value = readfunc (MENU_MINUS);
			break;
		case 'C':
			value = readfunc (MENU_PLUS);
			break;
		}

		if (value >= MENU_CLOSE || value < 0 || key == 'A' || key == 'D')
			return value;
	}

	return MENU_OK;
}
