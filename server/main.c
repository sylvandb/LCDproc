/*
  main.c for LCDd

  Contains main(), plus signal callback functions and a help screen.

  Program init, command-line handling, and the main loop are
  implemented here.  Also, minimal data about the program such as
  the revision number.

  Some of this stuff should probably be move elsewhere eventually,
  such as command-line handling and the main loop.  main() is supposed
  to be "dumb".
  
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pwd.h>
#include <errno.h>
#include <sys/types.h>
#include <syslog.h>

#include "shared/debug.h"

#include "drivers/lcd.h"
#include "sock.h"
#include "clients.h"
#include "screenlist.h"
#include "screen.h"
#include "parse.h"
#include "render.h"
#include "serverscreens.h"
#include "input.h"
#include "main.h"

#define MAX_TIMER 0x10000
#define DEFAULT_USER "nobody"
#define CONFIG_FILE "/etc/LCDd.conf"

int debug_level = 3;
char *version = VERSION;
char *protocol_version = PROTOCOL_VERSION;
char *build_date = __DATE__;

/* Socket to bind to...

   Using loopback is much more secure; it means that this port is
   accessible only to programs running locally on the same host as LCDd.

   Using variables for these means that (later) we can select which port
   and which address to bind to at run time. */

char bind_addr[64] = "127.0.0.1";
int lcd_port = LCDPORT;

// The parameter structure and args[] should
// be removed when getopt(3) is implemented,
// as there won't be any need for them then.
typedef struct parameter {
	char *sh, *lg;	// short and long versions
} parameter;

// This is currently only a list of available arguments, but doesn't
// really *do* anything.  It just helps to figure out which parameters
// go to the server, and which ones go to individual drivers...
static parameter args[] = {
	{"-h", "--help"},
	{"-d", "--driver"},
	{"-t", "--type"},
	{"-f", "--foreground"},
	{"-b", "--backlight"},
	{"-i", "--serverinfo"},
	{"-w", "--waittime"},
	{NULL, NULL},
};

void exit_program (int val);
void HelpScreen ();

// At this point, this function will only succeed;
// all other options will stop the program.
// However, it may not always be thus; so we prepared
// for this possibility by making it return an int.
//
int drop_privs(char *user) {
	struct passwd *pwent;

	if (getuid() == 0 || geteuid() == 0) {
		if ((pwent = getpwnam(user)) == NULL) {
			if (errno) {
				perror("LCDd: getpwnam");
				exit(1);
			} else {
				fprintf(stderr, "user %s not a valid user!", user);
				exit(1);
			}
		} else {
			if (setuid(pwent->pw_uid) < 0) {
				fprintf(stderr, "unable to switch to user %s\n", user);
				perror("LCDd: setuid");
				exit(1);
			}
		}
	}
	return 0;
}

int
main (int argc, char **argv)
{
	// TODO:  Use a config file!
	char cfgfile[256] = CONFIG_FILE;
	int i, err;
	int daemon_mode = 1;
	int disable_server_screen = 1;
	screen *s = NULL;
	char *str, *ing;				  // strings for commandline handling
	char *user = DEFAULT_USER;

	signal (SIGINT, exit_program);		// Ctrl-C will cause a clean exit...
	signal (SIGTERM, exit_program);		// and "kill"...
	signal (SIGHUP, exit_program);		// and "kill -HUP" (hangup)...
	signal (SIGKILL, exit_program);		// and just in case, "kill -KILL" (which cannot be trapped; but oh well)

	// If no paramaters given, give the help screen.
	if (argc == 1)
		HelpScreen ();

	// Don't spawn a child if we're using the curses driver
	for (i = 1; i < argc; i++) {
		if (0 == strcmp (argv[i], "-f") || 0 == strcmp (argv[i], "--foreground") || 0 == strcmp (argv[i], "curses"))
			daemon_mode = 0;
	}

	// Now, go into daemon mode...
#ifndef DEBUG
	openlog("LCDd", LOG_PID, LOG_DAEMON);
	syslog(LOG_NOTICE, "server version %s starting up (protocol version %s)",
		version, protocol_version);
	syslog(LOG_NOTICE, "server built on %s",
		build_date);

	if (daemon_mode) {
		int child;

		syslog(LOG_NOTICE, "server forking to background");

		if ((child = fork ()) != 0) {
			usleep (1500000);		  // Wait for child to initialize
			exit (0);				  /* PARENT EXITS */
		}
		// This line removed because it eats error messages...
		//setsid();                                       /* RELEASE TTY */
		//
		// After this point, as a daemon, no error messages should
		// go to the console unless drastic; rather, they should go to syslog
		//
		// However, option processing is not yet done, nor is any initialization (!)
		// So we must wait until the main loop.
	} else {
		syslog(LOG_NOTICE, "server running in foreground");
	}
#endif

	// Set up lcd driver base system
	lcd_init ("");

	// Parse the command line now...
	// TODO:  Move this to a separate function?
	for (i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {

			/*
			 * -d <driver> [ <driver_opts> ... ]
			 * --driver <driver> [ <driver_opts> ... ]
			 * 
			 * This option does not transfer to a getopt() version;
			 * will have to change the option format...
			 */

			if (0 == strcmp (argv[i], "-d") || 0 == strcmp (argv[i], "--driver")) {
				if (argc <= i + 1)
					HelpScreen ();
				str = argv[++i];
				ing = NULL;

				// Check to see if the next parameter is intended for LCDd,
				// or if it should be passed to the driver...
				if (argc > i + 1) {
					int j, skip = 0;
					for (j = 1; args[j].lg; j++)	// check each option except "help"
						if (!strcmp (argv[i + 1], args[j].sh) || !strcmp (argv[i + 1], args[j].lg))
							skip = 1;

					if (!skip) {
						ing = argv[++i];
					}
					//else i++;
				}
				err = lcd_add_driver (str, ing);
				if (err <= 0) {
					printf ("Error loading driver %s.  Continuing anyway...\n", str);
				}
				if ((err > 0) && (0 == strcmp (str, "curses")))
					daemon_mode = 0;

			/*
			 * -h
			 * --help
			 *
			 * Help function
			 */

			} else if (0 == strcmp (argv[i], "-h") || 0 == strcmp (argv[i], "--help")) {
				HelpScreen ();
			}

			/*
			 * -f
			 * --foreground
			 *
			 * Keep program in foreground; use for debugging, such
			 * as with gdb
			 */

			// Redundant, but it prevents errors...
			else if (0 == strcmp (argv[i], "-f") || 0 == strcmp (argv[i], "--foreground")) {
				daemon_mode = 0;

			/*
			 * -b [ "on" | "off" | "open" ]
			 * --backlight [ "on" | "off" | "open" ]
			 *
			 * This should be a driver option, likely within the config file.
			 * What IS "open" anyway?
			 */

			} else if (0 == strcmp (argv[i], "-b") || 0 == strcmp (argv[i], "--backlight")) {
				if (argc <= i + 1)
					HelpScreen ();
				str = argv[++i];
				if (0 == strcmp (str, "off"))
					backlight_state = backlight = BACKLIGHT_OFF;
				else if (0 == strcmp (str, "on"))
					backlight_state = backlight = BACKLIGHT_ON;
				else if (0 == strcmp (str, "open"))
					backlight = BACKLIGHT_OPEN;
				else
					HelpScreen ();

			/*
			 * -t
			 * --type
			 *
			 * Type of LCD; this *DEFINITELY* should only be
			 * a driver option.
			 */

			} else if (0 == strcmp (argv[i], "-t") || 0 == strcmp (argv[i], "--type")) {
				if (i + 1 > argc)
					HelpScreen ();
				else {
					int wid, hgt;

					i++;
					sscanf (argv[i], "%ix%i", &wid, &hgt);
					if (wid > 80 || wid < 16 || hgt > 25 || hgt < 2) {
						fprintf (stderr, "LCDd: Invalid lcd size \"%s\".  Using 20x4.\n", argv[i]);
						lcd.wid = 20;
						lcd.hgt = 4;
					} else {
						lcd.wid = wid;
						lcd.hgt = hgt;
					}

				}

			/*
			 * -i
			 * --serverinfo
			 *
			 * Display server information during normal screen
			 * updates when windows are active; server information
			 * is always shown if there are no windows active.
			 */

			} else if (0 == strcmp (argv[i], "-i") || 0 == strcmp (argv[i], "--serverinfo")) {
				if (i + 1 > argc)
					HelpScreen ();
				else {
					i++;
					if (0 == strcmp (argv[i], "off"))
						disable_server_screen = 1;
					if (0 == strcmp (argv[i], "on"))
						disable_server_screen = 0;
				}

			/*
			 * -w <1/8 secs>
			 * --waittime <1/8 secs>
			 *
			 * Sets the default duration that a screen
			 * is present.
			 */

			} else if (0 == strcmp (argv[i], "-w") || 0 == strcmp (argv[i], "--waittime")) {
				if (i + 1 > argc)
					HelpScreen ();
				else {
					int  tmp ;
					i++;
					tmp = atoi( argv[i] );
					if ( tmp < 16 || tmp > 10000 ) {
						printf ("Wait time should be between 16 and 10000 (1/8ths of second), not %s\n", argv[i]);
						HelpScreen ();
					} else {
						default_duration = tmp ;
					};
				}
			} else {

			/*
			 * End of valid options
			 */

				printf ("Invalid parameter: %s\n", argv[i]);
				HelpScreen ();
			}
		} else {
			printf("non-option: %s", argv[i]);
			HelpScreen ();
		}
	}

	// switch to a different user for the real work...
	if (lcd_port >= 1024)	// unpriviledged port
		drop_privs(user);

	if (sock_create_server (&bind_addr, lcd_port) <= 0) {
		syslog(LOG_ERR, "error opening socket");
		return 1;
	}

	if (lcd_port < 1024)	// priviledged port
		drop_privs(user);

	// Now init a bunch of required stuff...

	if (client_init () < 0) {
		syslog(LOG_ERR, "error initializing client list");
		return 1;
	}

	if (screenlist_init () < 0) {
		syslog(LOG_ERR, "error initializing screen list");
		return 1;
	}
	// Make sure the server screen shows up every once in a while..
	if (server_screen_init () < 0) {
		syslog(LOG_ERR, "error initializing server screens");
		return 1;
	} else if (disable_server_screen) {
		server_screen->priority = 256;
	}

	// Probably a better place to fork
	//
	// Main loop...

	syslog(LOG_NOTICE, "using %dx%d LCD with cells %dx%d",
		lcd.wid, lcd.hgt, lcd.cellwid, lcd.cellhgt);

	while (1) {
		sock_poll_clients ();		// poll clients for input
		parse_all_client_messages ();	// analyze input from network clients
		handle_input ();		// handle key input from devices

		// TODO:  Move this code to screenlist.c...
		// ... it should just say "handle_screens();"
		// Timer gets reset by screenlist_next()

		timer++;

		// this line's here because s was getting overwritten at one time...
		//s = screenlist_current();

		if (s && (timer >= s->duration)) {
			screenlist_next ();
		}

		// Just in case it gets out of hand...
		if (timer >= MAX_TIMER)
			timer = 0;

		// Update server screen with the right number
		// of clients and screens...
		//
		// TODO: Move this call to every client connection
		//       and every screen add...

		update_server_screen (timer);

		// draw the current scren

		if ((s = screenlist_current ()) != NULL)

			draw_screen (s, timer);
		else
			no_screen_screen (timer);

		usleep (TIME_UNIT);
	}

	// Quit!
	exit_program (0);

	return 0;
}

void
exit_program (int val)
{
	char buf[64];

	// TODO: These things shouldn't be so interdependent.  The order
	// things are shut down in shouldn't matter...

	strcpy(buf, "server shutting down on ");
	switch(val) {
		case 1: strcat(buf, "SIGHUP"); break;
		case 2: strcat(buf, "SIGINT"); break;
		case 15: strcat(buf, "SIGTERM"); break;
		default: snprintf(buf, sizeof(buf), "server shutting down on signal %d", val); break;
			 // Other values should not be seen, but just in case..
	}

	syslog(LOG_NOTICE, buf);	// send message to syslog
	goodbye_screen ();		// display goodbye screen on LCD display
	lcd_shutdown ();		// release driver memory and file descriptors

	client_shutdown ();		// shutdown clients (must come first)

	screenlist_shutdown ();		// shutdown screens (must come after client_shutdown)
	sock_close_all ();		// close all open sockets (must come after client_shutdown)

	exit (0);
}

void
HelpScreen ()
{
	printf ("LCDproc server daemon, %s\n", version);
	printf ("Copyright (c) 1999 Scott Scriven, William Ferrell, and misc contributors\n");
	printf ("This program is freely redistributable under the terms of the GNU Public License\n\n");
	printf ("Usage: LCDd [options]\n");
	printf ("\tOptions in []'s are optional.  Available options are:\n");
	printf ("\t-h\t--help\n\t\t\tDisplay this help screen\n");
	printf ("\t-t\t--type <size>\n\t\t\tSelect an LCD size (20x4, 16x2, etc...)\n");
	printf ("\t-d\t--driver <driver> [args]\n\t\t\tAdd a driver to use:\n");
	printf ("\t\t\tCFontz, curses, HD44780, irmanin, joy,\n\t\t\tMtxOrb, LB216, text\n");
	printf ("\t\t\t(args will be passed to the driver for init)\n");
	printf ("\t-f\t--foreground\n\t\t\tRun in the foreground (no daemon)\n");
	printf ("\t-b\t--backlight <mode>\n\t\t\tSet backlight mode (on, off, open)\n");
	printf ("\t-i\t--serverinfo off\n\t\t\tSet the server screen to low priority\n");
	printf ("\n");
	printf ("\tUse \"man LCDd\" for more info.\n");
	printf ("\tHelp on each driver's parameters are obtained upon request:\n\t\t\"LCDd -d driver --help\"\n");
	printf ("Example:\n");
	printf ("\tLCDd -d MtxOrb \"--device /dev/lcd --contrast 200\" -d joy\n");
	printf ("\n");
	exit (0);
}
