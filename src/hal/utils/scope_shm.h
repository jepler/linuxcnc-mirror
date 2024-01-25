#ifndef HALSC_SHM_H
#define HALSC_SHM_H
/** This file, 'halsc_shm.h', contains declarations used by both
    'halscope.c' and 'halscope_rt.c' to implement an oscilloscope.
    The declarations in this file are used by both the realtime
    and user space components of the scope.  Those used only in
    realtime are in 'halsc_rt.h', and those used only in user
    space are in 'halsc_usr.h'.
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
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

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

/***********************************************************************
*                         TYPEDEFS AND DEFINES                         *
************************************************************************/

#define SCOPE_SHM_KEY  0x130CF406
#define SCOPE_NUM_SAMPLES_DEFAULT 128000

typedef enum {
    IDLE = 0,			/* waiting for run command */
    INIT,			/* run command received */
    PRE_TRIG,			/* acquiring pre-trigger data */
    TRIG_WAIT,			/* waiting for trigger */
    POST_TRIG,			/* acquiring post-trigger data */
    DONE,			/* data acquisition complete */
    RESET			/* data acquisition interrupted */
} scope_state_t;

/* this struct holds a single value - one sample of one channel */

typedef union {
    unsigned char d_u8;		/* variable for bit */
    rtapi_u32 d_u32;		/* variable for u32 */
    rtapi_s32 d_s32;		/* variable for s32 */
    real_t d_real;		/* variable for float */
    ireal_t d_ireal;		/* intlike variable for float */
} scope_data_t;

/** This struct holds control data needed by both realtime and GUI code.
    It lives in shared memory.  The codes for each field identify which
    module(s) set the field.  "I" set at init only, "R" set by realtime
    code, "U" set by user code, "RU" set/modified by both.
*/

typedef struct {
    unsigned long shm_size;	/* Actual size of SHM area */
    int buf_len;		/* I length of buffer */
    int watchdog;		/* RU rt sets to zero, user incs */
    char thread_name[HAL_NAME_LEN + 1];	/* U thread used for sampling */
    int mult;			/* U sample period multiplier */
    int rec_len;		/* U total samples in record */
    int sample_len;		/* U max channels in each sample */
    int pre_trig;		/* U number of samples before trigger */
    int trig_chan;		/* U trigger channel number */
    scope_data_t trig_level;	/* U trigger level */
    int trig_edge;		/* U 0 = falling edge, 1 = rising edge */
    int force_trig;		/* RU U sets non-zero to force trigger */
    int auto_trig;		/* U enables auto triggering */
    int start;			/* R first sample in record */
    int curr;			/* R next sample to be acquired */
    int samples;		/* R number of valid samples */
    scope_state_t state;	/* RU current state */
    int data_offset[16];	/* U data addr in shmem for each channel */
    hal_type_t data_type[16];	/* U data type for each channel */
    char data_len[16];		/* U data size, 0 if not to be acquired */
} scope_shm_control_t;

#endif /* HALSC_SHM_H */
