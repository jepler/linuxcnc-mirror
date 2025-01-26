/********************************************************************
* Description:  uspace_ulapi.c
*               This file, 'uspace_ulapi.c', implements the user-level
*               API functions for machines without RT (simultated
*               processes)
*
* Author: John Kasunich, Paul Corner
* License: LGPL Version 2
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#define _GNU_SOURCE

#include <stddef.h>		/* NULL */
#include <stdio.h>		/* printf */
#include <stdlib.h>		/* malloc(), free() */
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include "rtapi.h"
#include <unistd.h>
#include <rtapi_errno.h>
#include "rtapi/uspace_common.h"


/* FIXME - no support for fifos */

int rtapi_fifo_new(int key, int module_id, unsigned long int size, char mode)

{
  (void)key;
  (void)module_id;
  (void)size;
  (void)mode;
  return -ENOSYS;
}

int rtapi_fifo_delete(int fifo_id, int module_id)
{
  (void)fifo_id;
  (void)module_id;
  return -ENOSYS;
}

int rtapi_fifo_read(int fifo_id, char *buf, unsigned long size)
{
  (void)fifo_id;
  (void)buf;
  (void)size;
  return -ENOSYS;
}

int rtapi_fifo_write(int fifo_id, char *buf, unsigned long int size)
{
  (void)fifo_id;
  (void)buf;
  (void)size;
  return -ENOSYS;
}

long long rtapi_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

long int rtapi_delay_max() { return 999999999; }

void rtapi_delay(long ns) {
    if(ns > rtapi_delay_max()) ns = rtapi_delay_max();
    struct timespec ts = {0, ns};
    rtapi_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL, NULL);
}

void default_rtapi_msg_handler(msg_level_t level, const char *fmt, va_list ap) {
    if(level == RTAPI_MSG_ALL) {
	vfprintf(stdout, fmt, ap);
        fflush(stdout);
    } else {
	vfprintf(stderr, fmt, ap);
        fflush(stderr);
    }
}
