/********************************************************************
* Description:  sampler_usr.c
*               User space part of "sampler", a HAL component that
*		can be used to sample data from HAL pins and write
*		it to a file at a specific realtime sample rate.
*
* Author: John Kasunich <jmkasunich at sourceforge dot net>
* License: GPL Version 2
*    
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/
/** This file, 'sampler_usr.c', is the user part of a HAL component
    that allows values to be sampled from HAL pins at a uniform
    realtime sample rate, and writes them to a stdout (from which
    they can be redirected to a file).  When the realtime module
    is loaded, it creates a stream in shared memory and begins capturing
    samples to the stream.  Then, the user space program 'hal_sampler'
    is invoked to read from the stream and print to stdout.

    Invoking:

    halsampler [-c chan_num] [-n num_samples] [-t]

    'chan_num', if present, specifies the sampler channel to use.
    The default is channel zero.

    'num_samples', if present, specifies the number of samples
    to be printed, after which the program will exit.  If omitted
    it will print continuously until killed.

    '-t' tells sampler to print the sample number at the start
    of each line.

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
    information, go to www.linuxcnc.org.
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "hal.h"                /* HAL public API decls */
#include "streamer.h"

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

int comp_id = -1;	/* -1 means hal_init() not called yet */
int shmem_id = -1;
int exitval = 1;	/* program return code - 1 means error */
int ignore_sig = 0;	/* used to flag critical regions */
char comp_name[HAL_NAME_LEN+1];	/* name for this instance of sampler */

/***********************************************************************
*                            MAIN PROGRAM                              *
************************************************************************/

/* signal handler */
static sig_atomic_t stop;
static void quit(int sig)
{
    (void)sig;
    if ( ignore_sig ) {
	return;
    }
    stop = 1;
}

#define BUF_SIZE 4000

int main(int argc, char **argv)
{
    int n, channel, tag;
    long int samples;
    unsigned this_sample, last_sample=0;
    char *cp, *cp2;
    hal_stream_t stream;

    /* set return code to "fail", clear it later if all goes well */
    exitval = 1;
    channel = 0;
    tag = 0;
    samples = -1;  /* -1 means run forever */
    /* FIXME - if I wasn't so lazy I'd learn how to use getopt() here */
    for ( n = 1 ; n < argc ; n++ ) {
	cp = argv[n];
	if ( *cp != '-' ) {
	    break;
	}
	switch ( *(++cp) ) {
	case 'c':
	    if (( *(++cp) == '\0' ) && ( ++n < argc )) { 
		cp = argv[n];
	    }
	    channel = strtol(cp, &cp2, 10);
	    if (( *cp2 ) || ( channel < 0 ) || ( channel >= MAX_SAMPLERS )) {
		fprintf(stderr,"ERROR: invalid channel number '%s'\n", cp );
		exit(1);
	    }
	    break;
	case 'n':
	    if (( *(++cp) == '\0' ) && ( ++n < argc )) { 
		cp = argv[n];
	    }
	    samples = strtol(cp, &cp2, 10);
	    if (( *cp2 ) || ( samples < 0 )) {
		fprintf(stderr, "ERROR: invalid sample count '%s'\n", cp );
		exit(1);
	    }
	    break;
	case 't':
	    tag = 1;
	    break;
	default:
	    fprintf(stderr,"ERROR: unknown option '%s'\n", cp );
	    exit(1);
	    break;
	}
    }
    if(n < argc) {
	int fd;
	if(argc > n+1) {
	    fprintf(stderr, "ERROR: At most one filename may be specified\n");
	    exit(1);
	}
	// make stdout be the named file
	fd = open(argv[n], O_WRONLY | O_CREAT | O_TRUNC, 0666);
	close(1);
	dup2(fd, 1);
    }
    /* register signal handlers - if the process is killed
       we need to call hal_exit() to free the shared memory */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGPIPE, quit);
    /* connect to HAL */
    /* create module name for specified channel */
    snprintf(comp_name, sizeof(comp_name), "halsampler%d", channel);
    /* connect to the HAL */
    ignore_sig = 1;
    comp_id = hal_init(comp_name);
    ignore_sig = 0;
    /* check result */
    if (comp_id < 0) {
	fprintf(stderr, "ERROR: hal_init() failed: %d\n", comp_id );
	goto out;
    }
    hal_ready(comp_id);
    int res = hal_stream_attach(&stream, comp_id, SAMPLER_SHMEM_KEY+channel, 0);
    if (res < 0) {
	errno = -res;
	perror("hal_stream_attach");
	goto out;
    }
    int num_pins = hal_stream_element_count(&stream);
    while ( samples != 0 ) {
	union hal_stream_data buf[num_pins];
	hal_stream_wait_readable(&stream, &stop);
	if(stop) break;
	int res = hal_stream_read(&stream, buf, &this_sample);
	if (res < 0) {
	    errno = -res;
	    perror("hal_stream_read");
	    goto out;
	}
	++last_sample;
	if ( this_sample != last_sample ) {
	    printf ( "overrun\n");
	    last_sample = this_sample;
	}
	if ( tag ) {
	    printf ( "%d ", this_sample-1 );
	}
	for ( n = 0 ; n < num_pins; n++ ) {
	    switch ( hal_stream_element_type(&stream, n) ) {
	    case HAL_FLOAT:
		printf ( "%f ", buf[n].f);
		break;
	    case HAL_BIT:
		if ( buf[n].b ) {
		    printf ( "1 " );
		} else {
		    printf ( "0 " );
		}
		break;
	    case HAL_U32:
		printf ( "%lu ", (unsigned long)buf[n].u);
		break;
	    case HAL_S32:
		printf ( "%ld ", (long)buf[n].s);
		break;
	    default:
		/* better not happen */
		goto out;
	    }
	}
	printf ( "\n" );
	if ( samples > 0 ) {
	    samples--;
	}
    }
    /* run was successful */
    exitval = 0;

out:
    ignore_sig = 1;
    hal_stream_detach(&stream);
    if ( comp_id >= 0 ) {
	hal_exit(comp_id);
    }
    return exitval;
}
