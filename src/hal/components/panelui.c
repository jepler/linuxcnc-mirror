/********************************************************************
* Description:  panelui.c
              A User space program to connect to "sampler", a HAL component that
*       can be used to sample data from HAL pins.
*       It decodes keyboard codes then update pins/linuxcnc commands.
*       The key codes are patterned after the MESA 7i73 card output.
*       There is also a realtime program sim_matrix_kb that outputs keycodes.
*       Users can add a handler.py file to include their own python routines.
*
* based on sampler_usr.c written by
* John Kasunich <jmkasunich at sourceforge dot net>
* Author: Chris Morley <chrisinnanaimo at hotmail dot com>
* License: GPL Version 2
*    
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/
/** This file, 'sampler_usr.c', is the user part of a HAL component
    that allows values to be sampled from HAL pins at a uniform 
    realtime sample rate.  When the realtime module
    is loaded, it creates a fifo in shared memory and begins capturing
    samples to the fifo.  Then, the user space program 'sampler_py'
    is invoked to read from the fifo and decode the data to keycodes.
    It then calls a python program to update pins or send linuxcnc commands.
    The python part uses INI style configs to define 'buttons', that either
    update hal pins, call linuxcnc commands or optionally call a user function.
    adding a handler.py file to a config's folder will be parsed and its functions
    added to available commands.

    Invoking:

    panelui [-c chan_num] [-n num_samples] [-t] [-d] [-v]

    'chan_num', if present, specifies the sampler channel to use.
    The default is channel zero.

    'num_samples', if present, specifies the number of samples
    to be printed, after which the program will exit.  If omitted
    it will print continuously until killed.

    '-t' tells sampler to print the sample number at the start
    of each line.

   '-d' tells sampler to print debug info such as button and command text.

   '-v' tells sampler to print lots of debug info .
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

    This code was written as part of the LinuxCNC HAL project.  For more
    information, go to www.linuxcnc.org.
*/
#include <Python.h>
#include <pyconfig.h>
#include <time.h>

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

#include "rtapi.h"      /* RTAPI realtime OS API */
#include "hal.h"                /* HAL public API decls */
#include "streamer.h"

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

int comp_id = -1;   /* -1 means hal_init() not called yet */
int shmem_id = -1;
int exitval = 1;    /* program return code - 1 means error */
int ignore_sig = 0; /* used to flag critical regions */
char comp_name[HAL_NAME_LEN+1]; /* name for this instance of psnelui */
int dbg = 0;
int vdbg = 0;
int keydown = 0xC0;
int keyup = 0x80;
int nochange = 0x40;
int rowshift;
int ncols = 8;
int nrows = 8;
int rollover = 2; //maximum number of simultaneous keys presses recognised
int s = 0; //key press state argument to pass to python
int r; //row argument passed to python
int c; //column argument passed to python
int num_keys; //count of the number of keys currently pressed
long int raw; //the raw data passed from shared memory realtime component 
long int keycode; // calculated from raw by masking keyup and keydown codes
long int start_time;
long int last_heartbeat;
long int heartbeat_difference;
struct timespec gettime_now;
PyObject *pExit,*pValue;
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
    case 'd':
        dbg = 1;
        break;
    case 'v':
        vdbg = 1;
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
    fd = open(argv[n], O_WRONLY | O_CREAT, 0666);
    close(1);
    dup2(fd, 1);
    }

    /* import the python module and get references for needed function */
    PyObject *pModule, *pFunc, *pPeriodicFunc, *pClass;
    PyConfig config;
    PyConfig_InitPythonConfig(&config);
    char name[] = "panelui"; 
    wchar_t *wname = Py_DecodeLocale(name, NULL);
    PyConfig_SetString(&config, &config.program_name, wname);
    Py_Initialize();
    PyRun_SimpleString("import pyui\n"
                     "pyui.instance = pyui.master.keyboard()\n"
                     "pyui.exit_funct = pyui.instance.exit\n"
                     "pyui.update_funct =  pyui.master.keyboard.update\n"
                     "pyui.periodic_funct =  pyui.master.keyboard.periodic\n");
    if (vdbg == 1){
        PyRun_SimpleString("pyui.intilize = pyui.instance.build(2)\n");
    }else if (dbg == 1){
        PyRun_SimpleString("pyui.intilize = pyui.instance.build(1)\n");
    }else{
        PyRun_SimpleString("pyui.intilize = pyui.instance.build(0)\n");
    }
    pModule = PyImport_ImportModule("pyui");

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, "update_funct");
        pPeriodicFunc = PyObject_GetAttrString(pModule, "periodic_funct");
        pClass = PyObject_GetAttrString(pModule, "instance");
        pExit = PyObject_GetAttrString(pModule, "exit_funct");
        if (pFunc == NULL){
            fprintf(stderr, "Panelui: Failed to find update function in python section\n");
            exit(1);
        }
        /* pFunc is a new reference */
        /* preset update function's arguments*/
        /* class instance, raw read code, row, column, state */
        if (pFunc && PyCallable_Check(pFunc)) {
            pValue = PyObject_CallFunction(pFunc, "Olllh", pClass, 0, 0, 0, 0);
            if (pValue == NULL){
                fprintf(stderr, "Panelui: update function failed: returned NULL\n");
            }
        }else{
            if (PyErr_Occurred()){
                PyErr_Print();
            }
                fprintf(stderr, "Panelui: Failed python function");
                exit(1);
        }
    }else {
        PyErr_Print();
        fprintf(stderr, "Panelui: Failed to load \"%s\"\n", "pyui");
        exit(1);
    }

    /* rowshoft is calculated from number of columns*/
    for (rowshift = 1; ncols > (1 << rowshift); rowshift++);




    /* register signal handlers - if the process is killed
       we need to call hal_exit() to free the shared memory */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGPIPE, quit);
    /* connect to HAL */
    /* create a unique module name, to allow for multiple samplers */
    snprintf(comp_name, sizeof(comp_name), "panelui");
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
    int res = hal_stream_attach(&stream, comp_id, SAMPLER_SHMEM_KEY+channel, "u");
    if (res < 0) {
        errno = -res;
        perror("hal_stream_attach");
        goto out;
    }

    /***********************************************************************
    *                            Sample Loop                               *
    ************************************************************************/
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    last_heartbeat = gettime_now.tv_nsec;
    while ( samples != 0 ) {
        union hal_stream_data buf[1];
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
        /* get raw value, mask keyup and keydown codes */
        /* compute row and column */
        raw = (unsigned long)buf[0].u;
        keycode = (raw & ~(keydown | keyup));
        r = keycode >> rowshift;
        c = keycode & ~(0xFFFFFFFF << rowshift);
        //printf ( "raw= %lu row= %d col = %d\n", (unsigned long)buf[n].u,r,c);
        /* error checking of row and columns */
        if  (r < 0 
            || c < 0
            || r >= nrows 
            || c >= ncols){
             goto skip;
        }
        /* skip 'no change' key code */
        if (raw == nochange) goto skip;
        /* KEY_DOWN: skip if too many keys down,
        add to rollover count and set state: True */
        if ((raw & keydown) == keydown){
            if (num_keys >= rollover) goto skip;
            num_keys++;
            s = 1;
        /* KEY_UP: subtract a key from rollover count and set state: False */
        }else if ((raw & keyup) == keyup){
            if (num_keys > 0) num_keys--;
            s = 0;
        /* zero is the only other valid keycode: it means all keys up*/
        }else if (raw  != 0) goto skip;
        //printf ( "output raw:%ld row: %ld col %ld state %d \n", raw, r, c, s );
        /* call python function with: class raw, row, and column arguments */
        pValue = PyObject_CallFunction(pFunc, "Olllh",pClass, raw, r, c, s);
        if (PyErr_Occurred()) PyErr_Print();
        if (pValue == NULL){
            fprintf(stderr, "Panelui's python update function failed: returned NULL\n");
        }
skip:
        if ( samples > 0 ) samples--;
        //do python periodic function every 100 ms
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        heartbeat_difference = gettime_now.tv_nsec - last_heartbeat;//Get nS value
        if (heartbeat_difference < 0)
            heartbeat_difference += 1000000000;//(Rolls over every 1 second)
        if (heartbeat_difference > 100000000){//<<< Heartbeat every 100mS
            last_heartbeat += 100000000;
            if (last_heartbeat > 1000000000)//(Rolls over every 1 second)
                last_heartbeat -= 1000000000;
            pValue = PyObject_CallFunction(pPeriodicFunc, "O",pClass);
            if (PyErr_Occurred()) PyErr_Print();
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
    PyRun_SimpleString("print ('''Exiting panelui's python module ''')");
    pValue = PyObject_CallObject(pExit, NULL);
    if (PyErr_Occurred()) {
        PyErr_Print();
    }
    Py_Finalize();
    return exitval;
}
