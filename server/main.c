/*
 * main.c
 * This file is part of LCDd, the lcdproc server.
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 *
 * Copyright (c) 1999, William Ferrell, Scott Scriven
 *		 2001, Joris Robijn
 *		 2001, Rene Wagner
 *		 2002, Mike Patnode
 *		 2002, Guillaume Filion
 *		 2002, Bernhard Tittelbach
 *
 *
 * Contains main(), plus signal callback functions and a help screen.
 *
 * Program init, command-line handling, and the main loop are
 * implemented here.  Also, minimal data about the program such as
 * the revision number.
 *
 * Some of this stuff should probably be move elsewhere eventually,
 * such as command-line handling and the main loop.  main() is supposed
 * to be "dumb".
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pwd.h>
#include <errno.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

extern char *optarg;
extern int optind, optopt, opterr;

#include "shared/report.h"

#include "drivers.h"
#include "sock.h"
#include "clients.h"
#include "screenlist.h"
#include "screen.h"
#include "parse.h"
#include "render.h"
#include "serverscreens.h"
#include "input.h"
#include "configfile.h"
#include "drivers.h"
#include "input.h"
#include "main.h"

#define MAX_TIMER 0x10000

/*#define DEFAULT_DEBUG_LEVEL 1*/
#define DEFAULT_LCD_PORT LCDPORT
#define DEFAULT_BIND_ADDR "127.0.0.1"
#define DEFAULT_CONFIGFILE "/etc/LCDd.conf"
#define DEFAULT_USER "nobody"
#define DEFAULT_DRIVER "curses"
#define DEFAULT_WAITTIME 8
#define MAX_DRIVERS 8
#define DEFAULT_DAEMON_MODE 1
#define DEFAULT_ENABLE_SERVER_SCREEN SERVER_SCREEN_NOSCREEN
#define DEFAULT_REPORTTOSYSLOG 0
#define DEFAULT_REPORTLEVEL RPT_WARNING


/* Store some standard defines into vars... */
char *version = VERSION;
char *protocol_version = PROTOCOL_VERSION;
char *api_version = API_VERSION;
char *build_date = __DATE__;


/* Socket to bind to...

   Using loopback is much more secure; it means that this port is
   accessible only to programs running locally on the same host as LCDd.

   Using variables for these means that (later) we can select which port
   and which address to bind to at run time. */


/**** Configuration variables ****/

/* All variables are set to 'unset' values*/
#define UNSET_INT -1
#define UNSET_STR "\01"

int debug_level; /* for compatibility with MtxOrb and joy drivers.
 * I was about to remove the comment in front of this.
 * Now how do we become compatible WITHOUT this debug_level ?*/

int lcd_port = UNSET_INT;
char bind_addr[64] = UNSET_STR;
char configfile[256] = UNSET_STR;
char user[64] = UNSET_STR;

int daemon_mode = UNSET_INT;
int enable_server_screen = UNSET_INT;

static int reportLevel = UNSET_INT;
static int reportToSyslog = UNSET_INT;
static int serverStarted = 0;

/* The drivers and their driver parameters*/
char *drivernames[MAX_DRIVERS];
char *driverfilenames[MAX_DRIVERS];
char *driverargs[MAX_DRIVERS];
int num_drivers = 0;


/* The parameter structure and args[] should
 * be removed when getopt(3) is implemented,
 * as there won't be any need for them then.
 * typedef struct parameter {
 *	char *sh, *lg;	*/ /* short and long versions*/
 /*} parameter;
 +
 * This is currently only a list of available arguments, but doesn't
 * really *do* anything.  It just helps to figure out which parameters
 * go to the server, and which ones go to individual drivers...
 *static parameter args[] = {
 *	{"-h", "--help"},
 *	{"-d", "--driver"},
 *	{"-t", "--type"},
 *	{"-f", "--foreground"},
 *	{"-b", "--backlight"},
 *	{"-i", "--serverinfo"},
 *	{"-w", "--waittime"},
 *	{NULL, NULL},
 *};
 */


/**** Local functions ****/
void exit_program (int val);
void HelpScreen ();

void clear_settings();
int process_command_line (int argc, char **argv);
int process_configfile (char *cfgfile);
void set_default_settings();
void output_GPL_notice();
int daemonize();
int init_sockets();
int drop_privs(char *user);
int init_drivers();
int init_screens();
void do_mainloop();
void lcd_list_drivers();

#if defined(__FILE__)
#define ESSENTIAL(f) {int r; if( ( r=(f) )<0 ) { report( RPT_CRIT,"Critical error: %s:%d, abort", __FILE__, __LINE__); return r;}}
#else
#define ESSENTIAL(f) {int r; if( ( r=(f) )<0 ) { report( RPT_CRIT,"Critical error, abort"); return r;}}
#endif

int
main (int argc, char **argv)
{
	/* FIXME: s is getting clobbered - in MANY places!!!
	 *screen *s = NULL;
	 *char buf[64];
	 */

	signal (SIGINT, exit_program);		/* Ctrl-C will cause a clean exit...*/
	signal (SIGTERM, exit_program);		/* and "kill"...*/
	signal (SIGHUP, exit_program);		/* and "kill -HUP" (hangup)...*/
	signal (SIGKILL, exit_program);		/* and just in case, "kill -KILL" (which cannot be trapped; but oh well)*/
	signal (SIGPIPE, SIG_IGN);		    /* ignore sigpipe.  Who cares if
	the client is gone... */

	/* If no paramaters given, give the help screen.
	 *if (argc == 1)
	 *	HelpScreen ();
	 */

	/*
	 * Settings in order of preference:
	 *
	 * 1: Settings specified in command line options...
	 * 2: Settings specified in configuration file...
	 * 3: Default settings
	 *
	 * Because of this, and because one option (-c) specifies where
	 * the configuration file is, things are done in this order:
	 *
	 * 1. Read and set options.
	 * 2. Read configuration file; if option is read in configuration
	 *    file and not already set, then set it.
	 * 3. Having read configuration file, if parameter is not set,
	 *    set it to the default value.
	 *
	 * It is for this reason that the default values are **NOT** set
	 * in the variable declaration...
	 */

	/* Set the initial reporting parameters*/
	report(RPT_NOTICE, "LCDd version %s starting", version );
	report(RPT_INFO, "Built on %s, protocol version %s, API version %s",
		build_date, protocol_version, api_version );

	clear_settings();

	/* Read command line*/
	ESSENTIAL( process_command_line (argc, argv) );

	/* Read config file
	 * Set configfile to default value first unless changed before
	 */
	if (strcmp(configfile, UNSET_STR)==0)
		strncpy (configfile, DEFAULT_CONFIGFILE, sizeof(configfile));
	ESSENTIAL( process_configfile (configfile) );

	/* Has the default setting for the driver in the config file (driver=none) been changed? */
	if (drivernames[0]&&(strcmp(drivernames[0], "none")==0)) {
		fprintf(stderr, "No driver specified. Make sure to edit the config file: %s\n", configfile);
		exit (-1);
	}

	/* Set default values*/
	set_default_settings();

	/* Set reporting values*/
	debug_level = reportLevel;
	ESSENTIAL( set_reporting( reportLevel, (reportToSyslog?RPT_DEST_SYSLOG:RPT_DEST_STDERR) ) );
 	report( RPT_NOTICE, "Set report level to %d, output to %s", reportLevel, (reportToSyslog?"syslog":"stderr") );

	/* Startup the server*/
	ESSENTIAL( init_drivers() );
	ESSENTIAL( init_sockets() );
	ESSENTIAL( init_screens() );
	ESSENTIAL( drop_privs(user) );

	/* Store it for exit_program()*/
	serverStarted = 1;

#ifndef DEBUG
	/* Now, go into daemon mode...*/
	if (daemon_mode) {
		report(RPT_NOTICE, "Server forking to background");
		ESSENTIAL( daemonize() );
	} else {
		output_GPL_notice();
		report(RPT_NOTICE, "Server running in foreground");
	}
#endif

	do_mainloop();
	/* This loop never stops; we'll get out only with a signal...*/

	return 0;
}


void
clear_settings ()
{
	int i;

	/*report( RPT_INFO, "clear_settings()" );*/

	lcd_port = UNSET_INT;
	strncpy( bind_addr, UNSET_STR, sizeof(bind_addr) );
	strncpy( configfile, UNSET_STR, sizeof(configfile) );
	strncpy( user, UNSET_STR, sizeof(user) );
	daemon_mode = UNSET_INT;
	enable_server_screen = UNSET_INT;
	backlight = UNSET_INT;
	backlight_state = UNSET_INT;

	backlight_brightness = UNSET_INT;
	backlight_off_brightness = UNSET_INT;

	default_duration = UNSET_INT;
	reportToSyslog = UNSET_INT;
	reportLevel = UNSET_INT;

	for( i=0; i < num_drivers; i++ ) {
		free( drivernames[i] );
		free( driverfilenames[i] );
		free( driverargs[i] );
		drivernames[i] = NULL;
		driverfilenames[i] = NULL;
		driverargs[i] = NULL;
	}
	num_drivers = 0;

	freepausekey = UNSET_INT;
	freebackkey = UNSET_INT;
	freeforwardkey = UNSET_INT;
	freemainmenukey = UNSET_INT;
}


/* parses arguments given on command line */
int
process_command_line (int argc, char **argv)
{
	char  c;

	/*report( RPT_INFO, "process_command_line()" );*/

	/* analyze options here..*/
	while ((c = getopt(argc, argv, "a:p:d:hfib:w:c:u:sr:")) > 0) {
		/* FIXME: Setting of c in this loop clobbers s!
		 * s is set equivalent to c.
		  */
		switch(c) {
	 		case 'd':
				/* Add to a list of drivers to be initialized later...*/
				if (num_drivers < MAX_DRIVERS) {
					drivernames[num_drivers]	= malloc( strlen(optarg)+1 );
					driverfilenames[num_drivers]	= malloc( strlen(optarg)+1 );
					driverargs[num_drivers]		= malloc(1);

					strcpy( drivernames[num_drivers], optarg );
					strcpy( driverfilenames[num_drivers], optarg );
					strcpy( driverargs[num_drivers], "" );
					num_drivers ++;
				} else
					report( RPT_ERR, "Too many drivers!" );
				break;
			case 'p':
				lcd_port = atoi(optarg);
				break;
			case 'u':
				strncpy(user, optarg, sizeof(user));
				break;
			case 'a':
				strncpy(bind_addr, optarg, sizeof(bind_addr));
				break;
			case 'h':
				HelpScreen ();
				break;
			case 'f':
				daemon_mode = 0;
				break;
			case 'c':
				strncpy(configfile, optarg, sizeof(configfile));
				break;
			case 'i':
				enable_server_screen = 0;
				break;
			case 'b':
				if( strcmp( optarg, "on" ) == 0 ) {
					backlight = BACKLIGHT_ON;
					backlight_state = backlight;
				}
				else if( strcmp( optarg, "off" ) == 0 ) {
					backlight = BACKLIGHT_OFF;
					backlight_state = backlight;
				}
				else if( strcmp( optarg, "open" ) == 0 ) {
					backlight = BACKLIGHT_OPEN;
					/* NOTE: The initial backlight (AKA backlight_state)
					 *       can be set from the configfile only
					 *       It defaults to BACKLIGHT_OFF
					 */
				}
				else if( strcmp( optarg, "" ) != 0 ) {
					report( RPT_ERR, "Backlight state should be on, off or open" );
					HelpScreen();
				}
				break;
			case 'w':
				default_duration = (int) (atof(optarg) * 1e6 / TIME_UNIT);
				if ( default_duration * TIME_UNIT < 2e6 ) {
					report( RPT_ERR, "Waittime should be at least 2 (seconds), not %.8s", optarg );
					HelpScreen ();
				};
				break;
			case 's':
				reportToSyslog = 1;
				break;
			case 'r':
				reportLevel = atoi(optarg);
				break;
			case '?':
				report( RPT_ERR, "Unknown option: '%c'", optopt );
				HelpScreen();
				break;
			case ':':
				report( RPT_ERR, "Missing option argument!" );
				HelpScreen();
				break;
		}
	}

	if (optind < argc)
		report( RPT_ERR, "Non-option arguments on the command line !");

	return 0;
}


/* reads and parses configuration file */
int
process_configfile ( char *configfile )
{
	int i;
	char * s;
	/*char buf[64];*/

	/*report( RPT_INFO, "process_configfile()" );*/

	/* Read server settings*/

	config_read_file( configfile );

/*	if( debug_level == UNSET_INT )
 *		debug_level = config_get_int( "server", "debug", 0, UNSET_INT );
 */

	if( lcd_port == UNSET_INT )
		lcd_port = config_get_int( "server", "port", 0, UNSET_INT );

	if( strcmp( bind_addr, UNSET_STR ) == 0 )
		strncpy( bind_addr, config_get_string( "server", "bind", 0, UNSET_STR ), sizeof(bind_addr));

	if( strcmp( user, UNSET_STR ) == 0 )
		strncpy( user, config_get_string( "server", "user", 0, UNSET_STR ), sizeof(user));

	if( default_duration == UNSET_INT ) {
		default_duration = (config_get_float( "server", "waittime", 0, 0 ) * 1e6 / TIME_UNIT );
		if( default_duration == 0 )
			default_duration = UNSET_INT;
		else if( default_duration * TIME_UNIT < 2e6 ) {
			report( RPT_ERR, "Waittime should be at least 2 (seconds)" );
			return -1;
		}
	}

	if( daemon_mode == UNSET_INT ) {
		int fg;
		fg = config_get_bool( "server", "foreground", 0, UNSET_INT );
		if( fg != UNSET_INT )
			daemon_mode = !fg;
		/*It makes no sense to report to stdout while running in background*/
		if(fg == 0)
			reportToSyslog=1;
	}

	if( enable_server_screen == UNSET_INT ) {
		s = config_get_string( "server", "serverscreen", 0, UNSET_STR );
		if( strcmp( s, "no" ) == 0 ) {
			enable_server_screen = SERVER_SCREEN_NOSCREEN;
		}
		else if( strcmp( s, "yes" ) == 0 ) {
			enable_server_screen = SERVER_SCREEN_ALWAYS;
		}
		else if( strcmp( s, UNSET_STR ) != 0 ) {
			report( RPT_ERR, "Serverscreen can only be no or yes" );
		}
	}

	if( backlight == UNSET_INT ) {
		s = config_get_string( "server", "backlight", 0, UNSET_STR );
		if( strcmp( s, "on" ) == 0 ) {
			backlight = BACKLIGHT_ON;
			backlight_state = backlight;
		}
		else if( strcmp( s, "off" ) == 0 ) {
			backlight = BACKLIGHT_OFF;
			backlight_state = backlight;
		}
		else if( strcmp( s, "open" ) == 0 ) {
			backlight = BACKLIGHT_OPEN;
		}
		else if( strcmp( s, UNSET_STR ) != 0 ) {
			report( RPT_ERR, "Backlight should be on, off or open" );
		}
	}

	if( backlight == BACKLIGHT_OPEN && backlight_state == UNSET_INT ) {
		s = config_get_string( "server", "initialbacklight", 0, UNSET_STR );
		if( strcmp( s, "on" ) == 0 ) {
			backlight_state = BACKLIGHT_ON;
		}
		else if( strcmp( s, "off" ) == 0 ) {
			backlight_state = BACKLIGHT_OFF;
		}
		else if( strcmp( s, UNSET_STR ) != 0 ) {
			report( RPT_ERR, "Initial backlight should be on or off" );
		}
	}

	if( backlight_brightness == UNSET_INT ) {
		i = config_get_int( "server", "backlightbrightness", 0, UNSET_INT );
		if( (0 <= i) && (i <= 255) ) {
			backlight_brightness = i;
		}
		else if (i != UNSET_INT){
			report( RPT_ERR, "BacklightBrightness should be an integer value between 0 and 255" );
		}
	}

	if( backlight_off_brightness == UNSET_INT ) {
		i = config_get_int( "server", "backlightoffbrightness", 0, UNSET_INT );
		if( (0 <= i) && (i <= 255) ) {
			backlight_off_brightness = i;
		}
		else if (i != UNSET_INT){
			report( RPT_ERR, "BacklightOffBrightness should be an integer value between 0 and 255" );
		}
	}

	if( reportToSyslog == UNSET_INT ) {
		/* Is the value set in the config file anyway ?*/
		if( strcmp( config_get_string( "server", "reportToSyslog", 0, "" ), "" ) != 0 ) {
			reportToSyslog = config_get_bool( "server", "reportToSyslog", 0, 0 );
		}
	}
	if( reportLevel == UNSET_INT ) {
		reportLevel = config_get_int( "server", "reportLevel", 0, UNSET_INT );
	}


	/* Read drivers*/

	 /* If drivers have been specified on the command line, then do not
	 * use the driver list from the config file.
	 */
	if( num_drivers == 0 ) {
		/* read the drivernames*/

		while( 1 ) {
			s = config_get_string( "server", "driver", num_drivers, "" );
			if( !s || s[0] == 0 )
				break;

			drivernames[num_drivers] = malloc(strlen(s)+1);
			driverfilenames[num_drivers] = malloc(1);
			driverargs[num_drivers] = malloc(1);

			strcpy( drivernames[num_drivers], s );
			strcpy( driverfilenames[num_drivers], "" );
			strcpy( driverargs[num_drivers], "" );

			num_drivers++;
		}
	}

	/* Now read the driver options that the server needs
	 * Drivers can read their own options later...
	 */
	for( i=0; i<num_drivers; i ++ ) {
		s = config_get_string( drivernames[i], "file", 0, "" );
		driverfilenames[i] = realloc( driverfilenames[i], strlen(s)+1 );
		strcpy (driverfilenames[i], s );

		s = config_get_string( drivernames[i], "arguments", 0, "" );
		driverargs[i] = realloc( driverargs[i], strlen(s)+1 );
		strcpy (driverargs[i], s );
	}

	/* Read the settings from the [input] section
	 */

	if( freepausekey == UNSET_INT ) {
		/* Is the value set in the config file anyway ?*/
		if( strcmp( config_get_string( "input", "FreePauseKey", 0, "" ), "" ) != 0 ) {
			freepausekey = config_get_bool( "input", "FreePauseKey", 0, UNSET_INT );
		}
	}

	if( freebackkey == UNSET_INT ) {
		/* Is the value set in the config file anyway ?*/
		if( strcmp( config_get_string( "input", "FreeBackKey", 0, "" ), "" ) != 0 ) {
			freebackkey = config_get_bool( "input", "FreeBackKey", 0, UNSET_INT );
		}
	}

	if( freeforwardkey == UNSET_INT ) {
		/* Is the value set in the config file anyway ?*/
		if( strcmp( config_get_string( "input", "FreeForwardKey", 0, "" ), "" ) != 0 ) {
			freeforwardkey = config_get_bool( "input", "FreeForwardKey", 0, UNSET_INT );
		}
	}

	if( freemainmenukey == UNSET_INT ) {
		/* Is the value set in the config file anyway ?*/
		if( strcmp( config_get_string( "input", "FreeMainMenuKey", 0, "" ), "" ) != 0 ) {
			freemainmenukey = config_get_bool( "input", "FreeMainMenuKey", 0, UNSET_INT );
		}
	}

	return 0;
}


void
set_default_settings()
{
	/*report( RPT_INFO, "set_default_settings()" );*/

	/* Set defaults into unfilled variables....*/

/*	if (debug_level == UNSET_INT)
 *		debug_level = DEFAULT_DEBUG_LEVEL;
 */
	if (lcd_port == UNSET_INT)
		lcd_port = DEFAULT_LCD_PORT;
	if (strcmp( bind_addr, UNSET_STR ) == 0)
		strncpy (bind_addr, DEFAULT_BIND_ADDR, sizeof(bind_addr));
	if (strcmp( user, UNSET_STR ) == 0)
		strncpy(user, DEFAULT_USER, sizeof(user));

	if (daemon_mode == UNSET_INT)
		daemon_mode = DEFAULT_DAEMON_MODE;
		
	if (enable_server_screen == UNSET_INT)
		enable_server_screen = DEFAULT_ENABLE_SERVER_SCREEN;

	if (default_duration == UNSET_INT)
		default_duration = DEFAULT_SCREEN_DURATION;

	if (backlight == UNSET_INT)
		backlight = BACKLIGHT_OPEN;
	if (backlight_state == UNSET_INT)
		backlight_state = BACKLIGHT_OFF;

	if (backlight_brightness == UNSET_INT)
		backlight_brightness = DEF_BACKLIGHT_BRIGHTNESS;
	if (backlight_off_brightness == UNSET_INT)
		backlight_off_brightness = DEF_BACKLIGHT_OFF_BRIGHTNESS;


	if (reportToSyslog == UNSET_INT )
		reportToSyslog = DEFAULT_REPORTTOSYSLOG;
	if( reportLevel == UNSET_INT )
		reportLevel = DEFAULT_REPORTLEVEL;


	/* Use default driver*/
	if( num_drivers == 0 ) {
		drivernames[0] = malloc(strlen(DEFAULT_DRIVER)+1);
		driverfilenames[0] = malloc(1);
		driverargs[0] = malloc(1);
		strcpy(drivernames[0], DEFAULT_DRIVER);
		strcpy(driverfilenames[0], "");
		strcpy(driverargs[0], "");

		num_drivers++;
	}

	if( freepausekey == UNSET_INT ) {
		freepausekey = DEFAULT_FREEPAUSEKEY;
	}

	if( freebackkey == UNSET_INT ) {
		freebackkey = DEFAULT_FREEBACKKEY;
	}

	if( freeforwardkey == UNSET_INT ) {
		freeforwardkey = DEFAULT_FREEFORWARDKEY;
	}

	if( freemainmenukey == UNSET_INT ) {
		freemainmenukey = DEFAULT_FREEMAINMENUKEY;
	}
}

void
output_GPL_notice()
{
	/* This will only be invoked when running in foreground
	 * So, directly output to stderr
	 */
	fprintf (stderr, "LCDd %s, LCDproc Protocol %s\n", VERSION, PROTOCOL_VERSION);
	fprintf (stderr, "Part of LCDproc\n");
	fprintf (stderr, "Copyright (C) 1998-2002 William Ferrell, Scott Scriven\n");
	fprintf (stderr, "                        and many other contributors\n\n");
	fprintf (stderr, "This program is free software; you can redistribute it and/or\n");
	fprintf (stderr, "modify it under the terms of the GNU General Public License\n");
	fprintf (stderr, "as published by the Free Software Foundation; either version 2\n");
	fprintf (stderr, "of the License, or (at your option) any later version.\n\n");

	fprintf (stderr, "This program is distributed in the hope that it will be useful,\n");
	fprintf (stderr, "but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
	fprintf (stderr, "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
	fprintf (stderr, "GNU General Public License for more details.\n\n");

	fprintf (stderr, "The file COPYING contains the GNU General Public License.\n");
	fprintf (stderr, "You should have received a copy of the GNU General Public License\n");
	fprintf (stderr, "along with this program; if not, write to the Free Software\n");
	fprintf (stderr, "Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.\n\n");
}


int
daemonize()
{
	int child;

	report( RPT_INFO, "daemonize()" );

	switch ((child = fork ()) ) {
	  case -1:
		report(RPT_ERR, "Could not fork");
		return -1;
	  case 0: /* We are the child*/
		break;
	  default: /* We are the parent*/
		usleep (1500000);		  /* Wait for child to initialize*/
		exit (0);				  /* PARENT EXITS */
	}
	/* This line removed because it eats error messages...
	 * setsid();*/                                       /* RELEASE TTY */
	/*
	 * After this point, as a daemon, no error messages should
	 * go to the console unless drastic; rather, they should go to syslog
	 *
	 * However, option processing is not yet done, nor is any initialization (!)
	 * So we must wait until the main loop.
	 */
	return 0;
}


int
init_sockets ()
{
	report( RPT_INFO, "init_sockets()" );

	if (sock_create_server (&bind_addr, lcd_port) <= 0) {
		report(RPT_ERR, "Error opening socket");
		return -1;
	}

	/* Now init a bunch of required stuff...*/

	if (client_init () < 0) {
		report(RPT_ERR, "Error initializing client list");
		return -1;
	}
	return 0;
}


int
init_drivers()
{
	int i, res;

	int output_loaded = 0;

	report( RPT_INFO, "init_drivers()" );

	/* FIXME: This sets s equal to a value related to i
	 * (bitshifted left?)  FIX FIX FIX ARGH....
	 *
	 * Go thru all drivers and initialize all of them
	 */
	for (i = 0; i < num_drivers; i++) {

		res = load_driver (drivernames[i], driverfilenames[i], driverargs[i]);
		if (res >= 0) {
			/* Load went OK*/
			switch( res ) {
			  case 0: /* Driver does input only*/
			  	break;
			  case 1: /* Driver does output*/
			  	output_loaded = 1;
			  	break;
			  case 2: /* Driver does output in foreground (don't daemonize)*/
			  	if ( !output_loaded ) {
			  		daemon_mode = 0;
			  	}
			  	output_loaded = 1;
			  	break;
			}
		} else {
			report( RPT_ERR, "Could not load driver %.40s", drivernames[i] );
		}
	}

	/* Do we have a running output driver ?*/
	if ( output_loaded ) {
		return 0;
	} else {
		report( RPT_ERR, "There is no output driver" );
		return -1;
	}
}


int drop_privs(char *user)
{
	struct passwd *pwent;

	report( RPT_INFO, "drop_privs()" );

	if (getuid() == 0 || geteuid() == 0) {
		if ((pwent = getpwnam(user)) == NULL) {
			report( RPT_ERR, "User %.40s not a valid user!", user );
			return -1;
		} else {
			if (setuid(pwent->pw_uid) < 0) {
				report( RPT_ERR, "Unable to switch to user %.40s", user );
				return -1;
			}
		}
	}
	return 0;
}


int
init_screens ()
{
	report( RPT_INFO, "init_screens()" );

	if (screenlist_init () < 0) {
		report(RPT_ERR, "Error initializing screen list");
		return -1;
	}
	
	if (enable_server_screen > SERVER_SCREEN_NEVER) {
		/* Add the server screen */
		if (server_screen_init () < 0) {
			report(RPT_ERR, "Error initializing server screens");
			return -1;
		}
		
		if (enable_server_screen == SERVER_SCREEN_NOSCREEN) {
			/* Display the server screen only when there are no other screens */
			server_screen->priority = 256;
		}
	}
	return 0;
}


void
do_mainloop ()
{
	screen *s = NULL;
	char *message=NULL;

	report( RPT_INFO, "do_mainloop()" );

	/*char buf[64];*/

	/* FIXME: s should still be null from initialization.... what's happening here?!*/
	while (1) {
		sock_poll_clients ();		/* poll clients for input*/
		parse_all_client_messages ();	/* analyze input from network clients*/
		handle_input ();		/* handle key input from devices*/

		/* TODO:  Move this code to screenlist.c...
		 * ... it should just say "handle_screens();"
		 * Timer gets reset by screenlist_next()
		 */

		timer++;

		/*if (s == NULL)
		 *	s = screenlist_current();

		 * this is here because s is getting overwritten...
		 *if (s != screenlist_current()) {
		 *	report(RPT_DEBUG, "internal error! s was found overwritten at main.c:637");
		 *	s = screenlist_current();
		 *}
		 */

		/*TODO: THIS MUST BE FIXED..... WHY is s getting overwritten?
		 * s is a local, it is never passed or assigned to anywhere.
		 * So SOMETHING is going haywire and clobbering memory....
		 */

		if (s && (timer >= s->duration))
			screenlist_next ();

		/* Just in case it gets out of hand...*/
		if (timer >= MAX_TIMER)
			timer = 0;

		/* Update server screen with the right number
		 * of clients and screens...
		 */

		/* TODO: Move this call to every client connection
		 *       and every screen add...
		 */

		update_server_screen (timer);

		/* draw the current scren*/

		if ((s = screenlist_current ()) != NULL)
			draw_screen (s, timer);

		usleep (TIME_UNIT);

		/* Check to see if the screen has a timeout value, if it does
		 * decrese it and then check to see if it has excpired.
		 * Remove if expired.
		 */
		if((message = malloc(256)) == NULL)
			report(RPT_ERR, "Error allocating message string");
		else if (s && s->name) {
			snprintf(message, 256, "Screen->%s has timeout->%d", s->name, s->timeout);
			report(RPT_DEBUG, message);
			free(message);
		} else {
			report(RPT_DEBUG, "Error: Received NULL pointer");
			free(message); /*fixes memory leak*/
		}
		if (s && s->timeout != -1) {

			--(s->timeout);
			if((message = malloc(256)) == NULL)
				report(RPT_ERR, "Error allocating message string");
			else {
				snprintf(message, 256, "Timeout matches check, screen %s has timeout->%d", s->name, s->timeout);
				report(RPT_DEBUG, message);
				free(message);
			}
			if (s->timeout <= 0) {
				screen_remove (s->parent, s->id);
				if((message = malloc(256)) == NULL)
					report(RPT_ERR, "Error allocating message string");
				else {
					snprintf(message, 256, "Removing screen %s which has timeout->%d", s->name, s->timeout);
					report(RPT_DEBUG, message);
					free(message);
				}
			}
		}
	}

	/* Quit! */
	exit_program (0);
}

void
exit_program (int val)
{
	char buf[64];

	report( RPT_INFO, "exit_program()" );

	/* TODO: These things shouldn't be so interdependent.  The order
	 * things are shut down in shouldn't matter...
	 */

	strncpy(buf, "Server shutting down on ", sizeof (buf) );
	switch(val) {
		case 1: strcat(buf, "SIGHUP"); break;
		case 2: strcat(buf, "SIGINT"); break;
		case 15: strcat(buf, "SIGTERM"); break;
		default: snprintf(buf, sizeof(buf), "Server shutting down on signal %d", val); break;
			 /* Other values should not be seen, but just in case..*/
	}

	report(RPT_NOTICE, buf);	/* report it*/

	/* Set emergency reporting and flush all messages if not done already.*/
	if( reportLevel == UNSET_INT )
		reportLevel = DEFAULT_REPORTLEVEL;
	if( reportToSyslog == UNSET_INT )
		reportLevel = DEFAULT_REPORTLEVEL;
	set_reporting( reportLevel, (reportToSyslog?RPT_DEST_SYSLOG:RPT_DEST_STDERR) );

	/* Shutdown things if server start was complete*/
	if( serverStarted ) {
		/* Get rid of all clients*/
		client_shutdown ();

		/* Last thing to display:
		 * display goodbye screen on LCD display*/
		goodbye_screen ();

		/* screenlist no longer needed - close*/
		screenlist_shutdown ();

		/* release driver memory and file descriptors*/
		unload_all_drivers ();

		/* open sockets can't be cloed earlier as sock_close_all()
		   closes ALL filedescriptors including the device file of the drivers,
		   stdout, and stderr
		   FIXME: Get sock_close_all to close SOCKETS ONLY*/
		sock_close_all ();
	}

	exit (0);
}


void
HelpScreen ()
{
	/* Help screen is printed to stdout on purpose. No reason to have
	 * this in syslog...
	 */
	report( RPT_INFO, "HelpScreen()" );

	fprintf (stdout, "\nLCDd Server Daemon (part of lcdproc), %s\n", version);
	fprintf (stdout, "Copyright (c) 1999 Scott Scriven, William Ferrell, and misc contributors\n");
	fprintf (stdout, "This program is freely redistributable under the terms of the GNU Public License\n\n");
	fprintf (stdout, "Usage: LCDd [ -hfiws ] [ -c <config> ] [ -d <driver> ] [ -a <addr> ] \\\n\t[ -p <port> ] [ -u <user> ] [ -w <time> ] [ -r <level> ]\n\n");
	fprintf (stdout, "Available options are:\n");

	fprintf (stdout, "\t-h\t\tDisplay this help screen\n");
	fprintf (stdout, "\t-c <config>\tUse a configuration file other than %s\n", DEFAULT_CONFIGFILE);
	/*fprintf (stdout, "\t-t\t\tSelect an LCD size (20x4, 16x2, etc...)\n");*/
	fprintf (stdout, "\t-d <driver>\tAdd a driver to use (output only to first)\n");
	/*fprintf (stdout, "\t\t\tCFontz, curses, HD44780, irmanin, joy,\n\t\t\tMtxOrb, LB216, text\n");*/
	/*fprintf (stdout, "\t\t\t(args will be passed to the driver for init)\n");*/
	fprintf (stdout, "\t-f\t\tRun in the foreground\n");
	/*fprintf (stdout, "\t-b\t--backlight <mode>\n\t\t\tSet backlight mode (on, off, open)\n");*/
	fprintf (stdout, "\t-i\t\tDisable showing of the main LCDproc server screen\n");
	fprintf (stdout, "\t-w <waittime>\tTime to pause at each screen (in seconds)\n");
	fprintf (stdout, "\t-a <addr>\tNetwork (IP) address to bind to\n");
	fprintf (stdout, "\t-p <port>\tNetwork port to listen for connections on\n");
	fprintf (stdout, "\t-u <user>\tUser to run as\n");
	fprintf (stdout, "\t-s\t\tOutput messages to syslog\n");
	fprintf (stdout, "\t-r <level>\tReport level (default=2)\n");

	fprintf (stdout, "\nCurrently available drivers:\n");
	lcd_list_drivers();

	/*fprintf (stdout, "\tHelp on each driver's parameters are obtained upon request:\n\t\t\"LCDd -d driver --help\"\n");*/
	/*fprintf (stdout, "Example:\n");*/
	/*fprintf (stdout, "\tLCDd -d MtxOrb \"--device /dev/lcd --contrast 200\" -d joy\n");*/
	fprintf (stdout, "\n");

	exit (0);
}
