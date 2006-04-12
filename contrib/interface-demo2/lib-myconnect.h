/* lib-myconnect.h

 used by interfacedemo for "pyramid" LCD device.
 */

/*
 Copyright (C) 2005 Silvan Marco Fin <silvan@kernelconcepts.de>

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

#ifndef LIB_MYCONNECT
#define LIB_MYCONNECT

/*
 lib-myconnect.c:

 used by interfacedemo to connect to a TCP socket.
 */


/* TCP connection to given host/port
 */
int get_connection(char *peername, int peerport);

#endif

