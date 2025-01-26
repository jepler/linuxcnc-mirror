/** This file, 'scope_files.c', handles file I/O for halscope.
    It includes code to save and restore front panel setups,
    and a clunky way to save captured scope data.
*/

/** Copyright (C) 2003 John Kasunich
                       <jmkasunich AT users DOT sourceforge DOT net>
*/

/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to https://linuxcnc.org.
*/

#include <locale.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ctype.h>
#include <string.h>

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "hal.h"		/* HAL public API decls */

#include <gtk/gtk.h>
#include "miscgtk.h"		/* generic GTK stuff */
#include "scope_usr.h"		/* scope related declarations */

/***********************************************************************
*                         DOCUMENTATION                                *
************************************************************************/

/* Scope setup is stored in the form of a script containing commands
   that set various parameters.

   Each command consists of a keyword followed by one or more values.
   Keywords are not case sensitive.
*/

/*
   THREAD <string>	name of thread to sample in
   MAXCHAN <int>	1,2,4,8,16, maximum channel count
   HMULT <int>		multiplier, sample every N runs of thread
   HZOOM <int>		1-9, horizontal zoom setting
   HPOS <float>		0.0-1.0, horizontal position setting
   CHAN <int>		sets channel for subsequent commands
   PIN <string>		named pin becomes source for channel
   PARAM <string>	named parameter becomes source for channel
   SIG <string>		named signal becomes source for channel
   CHOFF		disables selected channel
   VSCALE <int>		vertical scaling
   VPOS <float>		0.0-1.0, vertical position setting
   VOFF <float>		vertical offset
   VAC <float>	        vertical offset with AC coupling
   TSOURCE <int>	channel number for trigger source
   TLEVEL <float>	0.0-1.0, trigger level setting
   TPOS <float>		0.0-1.0, trigger position setting
   TPOLAR <enum>	trigger polarity, RISE or FALL
   TMODE <int>		0 = normal trigger, 1 = auto trigger
   RMODE <int>		0 = stop, 1 = norm, 2 = single, 3 = roll

*/


/***********************************************************************
*                         TYPEDEFS AND DEFINES                         *
************************************************************************/

typedef enum {
  INT,
  FLOAT,
  STRING
} arg_type_t;

typedef struct {
  const char* name;
  arg_type_t arg_type;
  char * (*handler)(void *arg);
} cmd_lut_entry_t;


/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/



/***********************************************************************
*                     LOCAL FUNCTION PROTOTYPES                        *
************************************************************************/

static void write_sample(FILE *fp, scope_data_t *dptr, hal_type_t type);
static int parse_command(char *in);
/* the following functions implement halscope config items
   each is called with a pointer to a single argument (the parser
   used here allows only one arg per command) and returns NULL if
   the command succeeded, or an error message if it failed.
*/
static char *dummy_cmd(void * arg);
static char *thread_cmd(void * arg);
static char *maxchan_cmd(void * arg);
static char *hzoom_cmd(void * arg);
static char *hpos_cmd(void * arg);
static char *hmult_cmd(void * arg);
static char *chan_cmd(void * arg);
static char *choff_cmd(void * arg);
static char *pin_cmd(void * arg);
static char *sig_cmd(void * arg);
static char *param_cmd(void * arg);
static char *vscale_cmd(void * arg);
static char *vpos_cmd(void * arg);
static char *voff_cmd(void * arg);
static char *voff_ac_cmd(void * arg);
static char *tsource_cmd(void * arg);
static char *tlevel_cmd(void * arg);
static char *tpos_cmd(void * arg);
static char *tpolar_cmd(void * arg);
static char *tmode_cmd(void * arg);
static char *rmode_cmd(void * arg);

/***********************************************************************
*                         LOCAL VARIABLES                              *
************************************************************************/

static const cmd_lut_entry_t cmd_lut[25] =
{
  { "thread",	STRING,	thread_cmd },
  { "maxchan",	INT,	maxchan_cmd },
  { "hmult",	INT,	hmult_cmd },
  { "hzoom",	INT,	hzoom_cmd },
  { "hpos",	FLOAT,	hpos_cmd },
  { "chan",	INT,	chan_cmd },
  { "choff",	INT,	choff_cmd },
  { "pin",	STRING,	pin_cmd },
  { "sig",	STRING,	sig_cmd },
  { "param",	STRING,	param_cmd },
  { "vscale",	INT,	vscale_cmd },
  { "vpos",	FLOAT,	vpos_cmd },
  { "vac",	FLOAT,	voff_ac_cmd },
  { "voff",	FLOAT,	voff_cmd },
  { "tsource",	INT,	tsource_cmd },
  { "tlevel",	FLOAT,	tlevel_cmd },
  { "tpos",	FLOAT,	tpos_cmd },
  { "tpolar",	INT,	tpolar_cmd },
  { "tmode",	INT,	tmode_cmd },
  { "rmode",	INT,	rmode_cmd },
  { "", 0, dummy_cmd }
};

static int deferred_channel;

/***********************************************************************
*                        PUBLIC FUNCTION CODE                          *
************************************************************************/

int read_config_file (char *filename)
{
    FILE *fp;
    char cmd_buf[100];
    char *cp;
    int retval;

    deferred_channel = 0;
    fp = fopen(filename, "r");
    if ( fp == NULL ) {
	fprintf(stderr, "halscope: config file '%s' could not be opened\n", filename );
	return -1;
    }
    retval = 0;
    while ( fgets(cmd_buf, 99, fp) != NULL ) {
	/* remove trailing newline if present */
	cp = cmd_buf;
	while (( *cp != '\n' ) && ( *cp != '\r' ) && ( *cp != '\0' )) {
	    cp++;
	}
	*cp = '\0';
	/* parse and execute the command */
	retval += parse_command(cmd_buf);
    }
    fclose(fp);
    if ( retval < 0 ) {
	fprintf(stderr, "halscope: config file '%s' caused %d warnings\n", filename, -retval );
	return -1;
    }
    return 0;
}


void write_config_file (char *filename)
{
    FILE *fp;

    fp = fopen(filename, "w");
    if ( fp == NULL ) {
	fprintf(stderr, "halscope: config file '%s' could not be created\n", filename );
	return;
    }
    write_horiz_config(fp);
    write_vert_config(fp);
    write_trig_config(fp);
    /* write run mode */
    if (ctrl_usr->run_mode == NORMAL ) {
	fprintf(fp, "RMODE 1\n" );
    } else if ( ctrl_usr->run_mode == SINGLE ) {
	fprintf(fp, "RMODE 2\n" );
#if 0 /* FIXME - role mode not implemented yet */
    } else if ( ctrl_usr->run_mode == ROLL ) {
	fprintf(fp, "RMODE 3\n" );
#endif
    } else {
	/* stop mode */
	fprintf(fp, "RMODE 0\n" );
    }
    fclose(fp);
}

/* writes captured data to disk */

void write_log_file(char *filename)
{
    scope_chan_t *chan;
    scope_data_t *dptr, *start;
    scope_horiz_t *horiz;
    scope_vert_t *vert;
    hal_type_t type[16];

    char *label[16];
    char *old_locale, *saved_locale;
    int sample_len, chan_active, chan_num, sample_period_ns, samples, n;
    FILE *fp;

    fp = fopen(filename, "w");
    if (fp == NULL) {
        fprintf(stderr, "ERROR: log file '%s' could not be created\n", filename);
        return;
    }

    /* Get name and type of active channels. */
    chan_active = 0;
    vert = &(ctrl_usr->vert);
    for (chan_num = 0; chan_num < 16; chan_num++) {
        if (vert->chan_enabled[chan_num] == 1) {
            chan = &(ctrl_usr->chan[chan_num]);
            label[chan_active] = chan->name;
            type[chan_active] = chan->data_type;
            chan_active++;
        }
    }

    /* sample_len is really the number of channels, don't let it fool you */
    sample_len = ctrl_shm->sample_len;
    samples = ctrl_usr->samples * sample_len;

    horiz = &(ctrl_usr->horiz);
    sample_period_ns = horiz->thread_period_ns * ctrl_shm->mult;

    /* write data */
    fprintf(fp, "# Sampling period is %i ns\n", sample_period_ns);

    /* point to the first sample in the display buffer */
    start = ctrl_usr->disp_buf;

    /* write header to csv file */
    for (chan_num = 0; chan_num < chan_active; chan_num++) {
        fprintf(fp, "%s", label[chan_num]);
        if (chan_num < chan_active - 1) {
            fprintf(fp, ";");
        }
    }
    fprintf(fp, "\n");

    /*
     * Specify LC_NUMERIC, makes the number format consistent, regardless
     * which locale previously in use. Necessary since the number format changes
     * with different locales.
     */
    old_locale = setlocale(LC_NUMERIC, NULL);
    if (old_locale == NULL) {
        fprintf(stderr, "ERROR: Could not read locale.");
        return;
    }
    saved_locale = strdup(old_locale);
    if (saved_locale == NULL) {
        fprintf(stderr, "ERROR: Could not copy old locale.");
        return;
    }
    setlocale(LC_NUMERIC, "C");

    n = 0;
    while (n < samples) {
        for (chan_num = 0; chan_num < sample_len; chan_num++) {
            dptr = start + n;
            /* Skip values for inactive channels. */
            if (chan_num >= chan_active) {
                n++;
                continue;
            }
            write_sample(fp, dptr, type[chan_num]);
            if (chan_num < chan_active - 1) {
                fprintf(fp, ";");
            }
            if ((chan_num == chan_active - 1) || (chan_num == 15 && chan_active == 16)) {
                fprintf(fp, "\n");
            }
            /* point to next sample */
            n++;
        }
    }

    fclose(fp);
    setlocale(LC_NUMERIC, saved_locale);
    free(saved_locale);
    printf("Log file '%s' written.\n", filename);
}

/* format the data and print it */
static void write_sample(FILE *fp, scope_data_t *dptr, hal_type_t type)
{
	double data_value;
	switch (type) {
		case HAL_BIT:
			if (dptr->d_u8) {
			data_value = 1.0;
			} else {
			data_value = 0.0;
			};
			break;
		case HAL_FLOAT:
			data_value = dptr->d_real;
			break;
		case HAL_S32:
			data_value = dptr->d_s32;
			break;
		case HAL_U32:
			data_value = dptr->d_u32;
			break;
		default:
			data_value = 0.0;
			break;
		}
	/* actually write the data to disk */
	fprintf(fp, "%.14f", data_value);
}


/***********************************************************************
*                         LOCAL FUNCTION CODE                          *
************************************************************************/

static int parse_command(char *in)
{
    int n;
    char *cp1, *rv;
    const char *cp2;
    int arg_int;
    double arg_float;
    char *arg_string;

    n = -1;
    do {
	cp1 = in;
	cp2 = cmd_lut[++n].name;
	/* skip all matching chars */
	while (( *cp2 != '\0') && (tolower(*cp1) == *cp2 )) {
	    cp1++;
	    cp2++;
	}
    } while ( *cp2 != '\0' );
    /* either a match, or zero length name (last entry) */
    if ( cp1 == in ) {
	/* zero length name, last entry, no match */
	if ( *in != '#' ) {
	    /* not a comment, must be a mistake */
	    fprintf (stderr, "halscope: unknown config command: '%s'\n", in );
	    return -1;
	}
    }
    switch ( cmd_lut[n].arg_type ) {
    case STRING:
	while ( isspace(*cp1) ) {
	    cp1++;
	}
	arg_string = cp1;
	/* find and replace newline at end */
	while (( *cp1 != '\n' ) && ( *cp1 != '\r' ) && ( *cp1 != '\0')) {
	    cp1++;
	}
	*cp1 = '\0';
	/* call command handler, it returns NULL on success,
	   or an error message on failure */
	rv = cmd_lut[n].handler(arg_string);
	break;
    case FLOAT:
	arg_float = strtod(cp1, &cp1);
	rv = cmd_lut[n].handler(&arg_float);
	break;
    case INT:
	arg_int = strtol(cp1, &cp1, 10);
	rv = cmd_lut[n].handler(&arg_int);
	break;
    default:
	return -1;
	break;
    }
    /* commands return NULL on success, an error msg on fail */
    if ( rv != NULL ) {
	fprintf(stderr, "halscope: %s: '%s'\n", rv, in );
	return -1;
    }
    return 0;
}

static char *dummy_cmd(void * arg)
{
    (void)arg;
    return "command not implemented";
}

static char *thread_cmd(void * arg)
{
    char *name;
    int rv;

    name = (char *)(arg);
    rv = set_sample_thread(name);
    if ( rv < 0 ) {
	return "could not find thread";
    }
    return NULL;
}

static char *maxchan_cmd(void * arg)
{
    int *argp, rv;

    argp = (int *)(arg);
    rv = set_rec_len(*argp);
    if ( rv < 0 ) {
	return "could not set record length";
    }
    return NULL;
}

static char *hzoom_cmd(void * arg)
{
    int *argp, rv;

    argp = (int *)(arg);
    rv = set_horiz_zoom(*argp);
    if ( rv < 0 ) {
	return "could not set horizontal zoom";
    }
    return NULL;
}

static char *hpos_cmd(void * arg)
{
    double *argp;
    int rv;

    argp = (double *)(arg);
    rv = set_horiz_pos(*argp);
    if ( rv < 0 ) {
	return "could not set horizontal position";
    }
    return NULL;
}

static char *hmult_cmd(void * arg)
{
    int *argp, rv;

    argp = (int *)(arg);
    rv = set_horiz_mult(*argp);
    if ( rv < 0 ) {
	return "could not set horizontal multiplier";
    }
    return NULL;
}

static char *chan_cmd(void * arg)
{
    int *argp, chan_num, rv;

    argp = (int *)(arg);
    chan_num = *argp;
    deferred_channel = 0;
    rv = set_active_channel(chan_num);
    switch (rv) {
    case 0:
	// successful return
	return NULL;
    case -1:
	return "illegal channel number";
    case -2:
	return "too many active channels";
    case -3:
	// no source for channel, OK as long as we get
	// a subsequent command that specifies a source
	deferred_channel = chan_num;
	return NULL;
    default:
	return "unknown result";
    }
}

static char *choff_cmd(void * arg)
{
    (void)arg;
    int chan_num;

    if ( deferred_channel != 0 ) {
	deferred_channel = 0;
	return NULL;
    }
    chan_num = ctrl_usr->vert.selected;
    set_channel_off(chan_num);
    return NULL;
}

static char *chan_src_cmd(int src_type, char *src_name)
{
    int chan_num, rv;

    if ( deferred_channel == 0 ) {
	// changing currently active channel
	chan_num = ctrl_usr->vert.selected;
	rv = set_channel_source(chan_num, src_type, src_name);
    } else {
	// setting source for previously empty channel
	chan_num = deferred_channel;
	rv = set_channel_source(chan_num, src_type, src_name);
	if ( rv == 0 ) {
	    // got a source now, select the channel
	    return chan_cmd(&chan_num);
	}
    }
    if ( rv < 0 ) {
	return "object not found";
    }
    return NULL;
}


static char *pin_cmd(void * arg)
{
    return chan_src_cmd(0, (char *)(arg));
}

static char *sig_cmd(void * arg)
{
    return chan_src_cmd(1, (char *)(arg));
}

static char *param_cmd(void * arg)
{
    return chan_src_cmd(2, (char *)(arg));
}

static char *vscale_cmd(void * arg)
{
    int *argp, rv;

    argp = (int *)(arg);
    rv = set_vert_scale(*argp);
    if ( rv < 0 ) {
	return "could not set vertical scale";
    }
    return NULL;
}

static char *vpos_cmd(void * arg)
{
    double *argp;
    int rv;

    argp = (double *)(arg);
    rv = set_vert_pos(*argp);
    if ( rv < 0 ) {
	return "could not set vertical position";
    }
    return NULL;
}

static char *voff_cmd(void * arg)
{
    double *argp;
    int rv;

    argp = (double *)(arg);
    rv = set_vert_offset(*argp, 0);
    if ( rv < 0 ) {
	return "could not set vertical offset";
    }
    return NULL;
}


static char *voff_ac_cmd(void * arg)
{
    double *argp;
    int rv;

    argp = (double *)(arg);
    rv = set_vert_offset(*argp, 1);
    if ( rv < 0 ) {
	return "could not set vertical offset";
    }
    return NULL;
}

static char *tsource_cmd(void * arg)
{
    int *argp, rv;

    argp = (int *)(arg);
    rv = set_trigger_source(*argp);
    if ( rv < 0 ) {
	return "could not set trigger source";
    }
    return NULL;
}

static char *tlevel_cmd(void * arg)
{
    double *argp;
    int rv;

    argp = (double *)(arg);
    rv = set_trigger_level(*argp);
    if ( rv < 0 ) {
	return "could not set trigger level";
    }
    return NULL;
}

static char *tpos_cmd(void * arg)
{
    double *argp;
    int rv;

    argp = (double *)(arg);
    rv = set_trigger_pos(*argp);
    if ( rv < 0 ) {
	return "could not set trigger position";
    }
    return NULL;
}

static char *tpolar_cmd(void * arg)
{
    int *argp;
    int rv;

    argp = (int *)(arg);
    rv = set_trigger_polarity(*argp);
    if ( rv < 0 ) {
	return "could not set trigger polarity";
    }
    return NULL;
}

static char *tmode_cmd(void * arg)
{
    int *argp;
    int rv;

    argp = (int *)(arg);
    rv = set_trigger_mode(*argp);
    if ( rv < 0 ) {
	return "could not set trigger mode";
    }
    return NULL;
}

static char *rmode_cmd(void * arg)
{
    int *argp;
    int rv;

    argp = (int *)(arg);
    rv = set_run_mode(*argp);
    if ( rv < 0 ) {
	return "could not set run mode";
    }
    return NULL;
}

