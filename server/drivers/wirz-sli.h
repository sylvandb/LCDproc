/*	wirz-sli.h -- Header file for LCDproc Wirz SLI driver
	Copyright (C) 1999 Horizon Technologies-http://horizon.pair.com/
	Written by Bryan Rittmeyer <bryanr@pair.com> - Released under GPL

        LCD info: http://www.wirz.com/sli/                               */

#ifndef SLI_H
#define SLI_H

#include "lcd.h"

MODULE_EXPORT int  sli_init (Driver *drvthis);
MODULE_EXPORT void sli_close (Driver *drvthis);
MODULE_EXPORT int  sli_width (Driver *drvthis);
MODULE_EXPORT int  sli_height (Driver *drvthis);
MODULE_EXPORT void sli_clear (Driver *drvthis);
MODULE_EXPORT void sli_flush (Driver *drvthis);
MODULE_EXPORT void sli_string (Driver *drvthis, int x, int y, char *string);
MODULE_EXPORT void sli_chr (Driver *drvthis, int x, int y, char c);

MODULE_EXPORT void sli_old_vbar (Driver *drvthis, int x, int len);
MODULE_EXPORT void sli_old_hbar (Driver *drvthis, int x, int y, int len);
MODULE_EXPORT void sli_num (Driver *drvthis, int x, int num);
MODULE_EXPORT void sli_old_icon (Driver *drvthis, int which, char dest);

MODULE_EXPORT void sli_set_char (Driver *drvthis, int n, char *dat);

#endif
