/********************************************************************
* Description: usrmotintf.cc
*   Defs for interface functions (init, exit, read, write) for user
*   processes which communicate with the real-time motion controller
*   in emcmot.c
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include "config.h"     	/* LINELEN definition */
#include <stdlib.h>		/* exit() */
#include <sys/stat.h>
#include <string.h>		/* memcpy() */
#include <float.h>		/* DBL_MIN */
#include "motion.h"		/* emcmot_status_t,CMD */
#include "motion_debug.h"       /* emcmot_debug_t */
#include "motion_struct.h"      /* emcmot_struct_t */
#include "emcmotcfg.h"		/* EMCMOT_ERROR_NUM,LEN */
#include "emcmotglb.h"		/* SHMEM_KEY */
#include "usrmotintf.h"		/* these decls */
#include "_timer.h"
#include "rcs_print.hh"

#include "inifile.hh"

#define READ_TIMEOUT_SEC 0	/* seconds for timeout */
#define READ_TIMEOUT_USEC 100000	/* microseconds for timeout */

#include "rtapi.h"

#include "dbuf.h"
#include "stashf.h"

static int inited = 0;		/* flag if inited */

static emcmot_command_t *emcmotCommand = 0;
static emcmot_status_t *emcmotStatus = 0;
static emcmot_config_t *emcmotConfig = 0;
static emcmot_debug_t *emcmotDebug = 0;
static emcmot_error_t *emcmotError = 0;
static emcmot_struct_t *emcmotStruct = 0;

/* usrmotIniLoad() loads params (SHMEM_KEY, COMM_TIMEOUT, COMM_WAIT)
   from named ini file */
int usrmotIniLoad(const char *filename)
{
    IniFile inifile(IniFile::ERR_CONVERSION);   // Enable exception.
    
    /* open it */
    if (!inifile.Open(filename)) {
	rtapi_print("can't find emcmot ini file %s\n", filename);
	return -1;
    }

    try {
        inifile.Find((int *)&SHMEM_KEY, "SHMEM_KEY", "EMCMOT");
        inifile.Find(&EMCMOT_COMM_TIMEOUT, "COMM_TIMEOUT", "EMCMOT");
        inifile.Find(&EMCMOT_COMM_WAIT, "COMM_WAIT", "EMCMOT");
    }

    catch(IniFile::Exception &e){
        e.Print();
	return -1;
    }

    return 0;
}

/* writes command from c */
int usrmotWriteEmcmotCommand(emcmot_command_t * c)
{
    emcmot_status_t s;
    static int commandNum = 0;
    static unsigned char headCount = 0;
    double end;

    if (!MOTION_ID_VALID(c->id)) {
        rcs_print("USRMOT: ERROR: invalid motion id: %d\n",c->id);
	return EMCMOT_COMM_INVALID_MOTION_ID;
    }
    c->head = ++headCount;
    c->tail = c->head;
    c->commandNum = ++commandNum;

    /* check for mapped mem still around */
    if (0 == emcmotCommand) {
        rcs_print("USRMOT: ERROR: can't connect to shared memory\n");
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    /* copy entire command structure to shared memory */
    *emcmotCommand = *c;
    /* poll for receipt of command */
    /* set timeout for comm failure, now + timeout */
    end = etime() + EMCMOT_COMM_TIMEOUT;
    /* now check to see if it got it */
    while (etime() < end) {
	/* update status */
	if (( usrmotReadEmcmotStatus(&s) == 0 ) && ( s.commandNumEcho == commandNum )) {
	    /* now check emcmot status flag */
	    if (s.commandStatus == EMCMOT_COMMAND_OK) {
		return EMCMOT_COMM_OK;
	    } else {
                rcs_print("USRMOT: ERROR: invalid command\n");
		return EMCMOT_COMM_ERROR_COMMAND;
	    }
	}
	esleep(25e-6);
    }
    rcs_print("USRMOT: ERROR: command timeout\n");
    return EMCMOT_COMM_ERROR_TIMEOUT;
}

/* copies status to s */
int usrmotReadEmcmotStatus(emcmot_status_t * s)
{
    int split_read_count;
    
    /* check for shmem still around */
    if (0 == emcmotStatus) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	/* copy status struct from shmem to local memory */
	memcpy(s, emcmotStatus, sizeof(emcmot_status_t));
	/* got it, now check head-tail matche */
	if (s->head == s->tail) {
	    /* head and tail match, done */
	    return EMCMOT_COMM_OK;
	}
	/* inc counter and try again, max three times */
    } while ( ++split_read_count < 3 );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

/* copies config to s */
int usrmotReadEmcmotConfig(emcmot_config_t * s)
{
    int split_read_count;
    
    /* check for shmem still around */
    if (0 == emcmotConfig) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	/* copy config struct from shmem to local memory */
	memcpy(s, emcmotConfig, sizeof(emcmot_config_t));
	/* got it, now check head-tail matches */
	if (s->head == s->tail) {
	    /* head and tail match, done */
	    return EMCMOT_COMM_OK;
	}
	/* inc counter and try again, max three times */
    } while ( ++split_read_count < 3 );
printf("ReadEmcmotConfig COMM_SPLIT_READ_TIMEOUT\n" );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

/* copies debug to s */
int usrmotReadEmcmotDebug(emcmot_debug_t * s)
{
    int split_read_count;
    
    /* check for shmem still around */
    if (0 == emcmotDebug) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	/* copy debug struct from shmem to local memory */
	memcpy(s, emcmotDebug, sizeof(emcmot_debug_t));
	/* got it, now check head-tail matches */
	if (s->head == s->tail) {
	    /* head and tail match, done */
	    return EMCMOT_COMM_OK;
	}
	/* inc counter and try again, max three times */
    } while ( ++split_read_count < 3 );
printf("ReadEmcmotDebug COMM_SPLIT_READ_TIMEOUT\n" );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

/* copies error to s */
int usrmotReadEmcmotError(char *e)
{
    /* check to see if ptr still around */
    if (emcmotError == 0) {
	return -1;
    }

    char data[EMCMOT_ERROR_LEN];
    struct dbuf d;
    dbuf_init(&d, (unsigned char *)data, EMCMOT_ERROR_LEN);

    /* returns 0 if something, -1 if not */
    int result = emcmotErrorGet(emcmotError, data);
    if(result < 0) return result;

    struct dbuf_iter di;
    dbuf_iter_init(&di, &d);

    result =  snprintdbuf(e, EMCMOT_ERROR_LEN, &di);
    if(result < 0) return result;
    return 0;
}

/*
 htostr()

 converts short int to 0-1 style string, in s. Assumes a short is 2 bytes.
*/
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - don't know if this is still needed 
				 */

static int htostr(char *s, unsigned short h)
{
    int t;

    for (t = 15; t >= 0; t--) {
	s[t] = h % 2 ? '1' : '0';
	h >>= 1;
    }
    s[16] = 0;			/* null terminate */

    return 0;
}
#endif

void printEmcPose(EmcPose * pose)
{
    printf("x=%f\ty=%f\tz=%f\tu=%f\tv=%f\tw=%f\ta=%f\tb=%f\tc=%f",
           pose->tran.x, pose->tran.y, pose->tran.z, 
           pose->u, pose->v, pose->w,
           pose->a, pose->b, pose->c);
}

void printTPstruct(TP_STRUCT * tp)
{
    printf("queueSize=%d\n", tp->queueSize);
    printf("cycleTime=%f\n", tp->cycleTime);
    printf("vMax=%f\n", tp->vMax);
    printf("vLimit=%f\n", tp->vLimit);
    printf("wMax=%f\n", tp->wMax);
    printf("wDotMax=%f\n", tp->wDotMax);
    printf("nextId=%d\n", tp->nextId);
    printf("execId=%d\n", tp->execId);
    printf("termCond=%d\n", tp->termCond);
    printf("currentPos :");
    printEmcPose(&tp->currentPos);
    printf("\n");
    printf("goalPos :");
    printEmcPose(&tp->goalPos);
    printf("\n");
    printf("done=%d\n", tp->done);
    printf("depth=%d\n", tp->depth);
    printf("activeDepth=%d\n", tp->activeDepth);
    printf("aborting=%d\n", tp->aborting);
    printf("pausing=%d\n", tp->pausing);
}

void usrmotPrintEmcmotDebug(emcmot_debug_t *d, int which)
{
//    int t;

    printf("running time: \t%f\n", d->running_time);
    switch (which) {
    case 0:
	printf("split:        \t%d\n", d->split);
	printf("teleop desiredVel: \t%f\t%f\t%f\t%f\t%f\t%f\n",
	    d->teleop_data.desiredVel.tran.x,
	    d->teleop_data.desiredVel.tran.y,
	    d->teleop_data.desiredVel.tran.z,
	    d->teleop_data.desiredVel.a,
	    d->teleop_data.desiredVel.b, d->teleop_data.desiredVel.c);
	printf("teleop currentVel: \t%f\t%f\t%f\t%f\t%f\t%f\n",
	    d->teleop_data.currentVel.tran.x,
	    d->teleop_data.currentVel.tran.y,
	    d->teleop_data.currentVel.tran.z,
	    d->teleop_data.currentVel.a,
	    d->teleop_data.currentVel.b, d->teleop_data.currentVel.c);
	printf("teleop desiredAccel: \t%f\t%f\t%f\t%f\t%f\t%f\n",
	    d->teleop_data.desiredAccel.tran.x,
	    d->teleop_data.desiredAccel.tran.y,
	    d->teleop_data.desiredAccel.tran.z,
	    d->teleop_data.desiredAccel.a,
	    d->teleop_data.desiredAccel.b, d->teleop_data.desiredAccel.c);
	printf("teleop currentAccel: \t%f\t%f\t%f\t%f\t%f\t%f\n",
	    d->teleop_data.currentAccel.tran.x,
	    d->teleop_data.currentAccel.tran.y,
	    d->teleop_data.currentAccel.tran.z,
	    d->teleop_data.currentAccel.a,
	    d->teleop_data.currentAccel.b, d->teleop_data.currentAccel.c);
	break;
/*! \todo Another #if 0 */
#if 0
	printf("\nferror:        ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", d->ferrorCurrent[t]);
	}
	printf("\n");

	printf("\nferror High:        ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", d->ferrorHighMark[t]);
	}
	printf("\n");
	break;
#endif
    case 5:
	printf("traj  m/m/a:\t%f\t%f\t%f\n", d->tMin, d->tMax, d->tAvg);
	printf("\n");
	printf("servo m/m/a:\t%f\t%f\t%f\n", d->sMin, d->sMax, d->sAvg);
	printf("\n");
	printf("(off) m/m/a:\t%f\t%f\t%f\n", d->nMin, d->nMax, d->nAvg);
	printf("\n");
	printf("(cycle to cycle  time) m/m/a:\t%f\t%f\t%f\n", d->yMin, d->yMax,
	    d->yAvg);
	printf("\n");
	printf("(frequency compute  time) m/m/a:\t%f\t%f\t%f\n", d->fMin,
	    d->fMax, d->fAvg);
	printf("\n");
	printf("(frequecy cycle to cycle  time) m/m/a:\t%f\t%f\t%f\n",
	    d->fyMin, d->fyMax, d->fyAvg);
	printf("\n");
	break;

    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
//      printf("jointPos[%d]: %f\n", which - 6, d->jointPos[(which - 6)]);
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	printf("coarseJointPos[%d]: %f\n",
	    which - 6, d->coarseJointPos[(which - 6)]);
	printf("jointVel[%d]: %f\n", which - 6, d->jointVel[(which - 6)]);
	printf("rawInput[%d]: %f\n", which - 6, d->rawInput[(which - 6)]);
	printf("rawOutput[%d]: %f\n", which - 6, d->rawOutput[(which - 6)]);
#endif
//      printf("bcompincr[%d]: %f\n", which - 6, d->bcompincr[(which - 6)]);
	break;

    case 12:
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	printf("\noldInput:  ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", d->oldInput[t]);
	}
	printf("\nrawInput:  ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", d->rawInput[t]);
	}
	printf("\ninverseInputScale:  ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", d->inverseInputScale[t]);
	}
#endif
	printf("\n");

    default:
	break;
    }

}

void usrmotPrintEmcmotConfig(emcmot_config_t c, int which)
{
//    int t;
//    char m[32];

    switch (which) {
    case 0:
	printf("debug level   \t%d\n", c.debug);
	printf("traj time:    \t%f\n", c.trajCycleTime);
	printf("servo time:   \t%f\n", c.servoCycleTime);
	printf("interp rate:  \t%d\n", c.interpolationRate);
	printf("v limit:      \t%f\n", c.limitVel);
	printf("axis vlimit:  \t");
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - waiting for new structs */
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%f ", c.axisLimitVel[t]);
	}
	printf("\n");
	printf("axis acc: \t");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%f ", c.axisLimitAcc[t]);
	}
	printf("\n");
#endif
	printf("\n");
	break;

    case 1:
	printf("pid stuff is obsolete\n");
/*! \todo Another #if 0 */
#if 0
	printf
	    ("pid:\tP\tI\tD\tFF0\tFF1\tFF2\tBCKLSH\tBIAS\tMAXI\tDEADBAND\tCYCLE TIME\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf
		("\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%f\t%f\n",
		c.pid[t].p, c.pid[t].i, c.pid[t].d, c.pid[t].ff0,
		c.pid[t].ff1, c.pid[t].ff2, c.pid[t].backlash, c.pid[t].bias,
		c.pid[t].maxError, c.pid[t].deadband, c.pid[t].cycleTime);
	}
	printf("\n");
#endif
	break;

    case 3:
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - waiting for new structs */
	printf("pos limits:   ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", c.maxLimit[t]);
	}

	printf("\nneg limits:   ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", c.minLimit[t]);
	}

	printf("\nmax ferror:   ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", c.maxFerror[t]);
	}
	printf("\n");

	printf("\nmin ferror:   ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", c.minFerror[t]);
	}
	printf("\n");

	printf("\nhome offsets:  ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", c.homeOffset[t]);
	}
	printf("\n");
#endif
	break;

    default:
	break;
    }

}

/* status printing function */
void usrmotPrintEmcmotStatus(emcmot_status_t *s, int which)
{
//    int t;
//    char m[32];

    switch (which) {
    case 0:
	printf("mode:         \t%s\n",
	    s->motionFlag & EMCMOT_MOTION_TELEOP_BIT ? "teleop" :
	    (s->motionFlag & EMCMOT_MOTION_COORD_BIT ? "coord" : "free")
	    );
	printf("cmd:          \t%d\n", s->commandEcho);
	printf("cmd num:      \t%d\n", s->commandNumEcho);
	printf("heartbeat:    \t%u\n", s->heartbeat);
	printf("compute time: \t%f\n", s->computeTime);
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	printf("axes enabled: \t");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d", s->axisFlag[t] & EMCMOT_JOINT_ENABLE_BIT ? 1 : 0);
	}
	printf("\n");
#endif
	printf("cmd pos:      \t%f\t%f\t%f\t%f\t%f\t%f\n",
	    s->carte_pos_cmd.tran.x, s->carte_pos_cmd.tran.y,
	    s->carte_pos_cmd.tran.z, s->carte_pos_cmd.a, s->carte_pos_cmd.b,
	    s->carte_pos_cmd.c);
	printf("act pos:      \t%f\t%f\t%f\t%f\t%f\t%f\n",
	    s->carte_pos_fb.tran.x, s->carte_pos_fb.tran.y,
	    s->carte_pos_fb.tran.z, s->carte_pos_fb.a, s->carte_pos_fb.b,
	    s->carte_pos_fb.c);
	printf("joint data:\n");
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	printf(" cmd: ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->joint_pos_cmd[t]);
	}
	printf("\n");
	printf(" fb:  ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->joint_pos_fb[t]);
	}
	printf("\n");
	printf(" vel: ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->joint_vel_cmd[t]);
	}
	printf("\n");
	printf(" ferr:");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->ferrorCurrent[t]);
	}
	printf("\n");
	printf(" lim:");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->ferrorLimit[t]);
	}
	printf("\n");
	printf(" max:");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->ferrorHighMark[t]);
	}
	printf("\n");
#endif
	printf("velocity:     \t%f\n", s->vel);
	printf("accel:        \t%f\n", s->acc);
	printf("id:           \t%d\n", s->id);
	printf("depth:        \t%d\n", s->depth);
	printf("active depth: \t%d\n", s->activeDepth);
	printf("inpos:        \t%d\n",
	    s->motionFlag & EMCMOT_MOTION_INPOS_BIT ? 1 : 0);
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	printf("vscales:      \tQ: %.2f", s->qVscale);
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%d: %.2f", t, s->axVscale[t]);
	}
	printf("\n");
#endif
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	printf("homing:       \t");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d", s->axisFlag[0] & EMCMOT_JOINT_HOMING_BIT ? 1 : 0);
	}
	printf("\n");
#endif
	printf("enabled:     \t%s\n",
	    s->motionFlag & EMCMOT_MOTION_ENABLE_BIT ? "ENABLED" : "DISABLED");
	printf("probe value: %d\n", s->probeVal);
	printf("probe Tripped: %d\n", s->probeTripped);
	printf("probing: %d\n", s->probing);
	printf("probed pos:      \t%f\t%f\t%f\n",
	    s->probedPos.tran.x, s->probedPos.tran.y, s->probedPos.tran.z);
	break;

    case 2:
	/* print motion and axis flags */
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - change to work with joint
				   structures */
	htostr(m, s->motionFlag);
	printf("motion:   %s\n", m);
	printf("axes:     ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    htostr(m, s->axisFlag[t]);
	    printf("%s ", m);
	}
	printf("\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_ENABLE_BIT) != 0));
	}
	printf("enable\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_ACTIVE_BIT) != 0));
	}
	printf("active\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_INPOS_BIT) != 0));
	}
	printf("inpos\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_ERROR_BIT) != 0));
	}
	printf("error\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t",
		((s->axisFlag[t] & EMCMOT_JOINT_MAX_SOFT_LIMIT_BIT) != 0));
	}
	printf("max_soft_limit\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t",
		((s->axisFlag[t] & EMCMOT_JOINT_MIN_SOFT_LIMIT_BIT) != 0));
	}
	printf("min_soft_limit\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t",
		((s->axisFlag[t] & EMCMOT_JOINT_MAX_HARD_LIMIT_BIT) != 0));
	}
	printf("max_hard_limit\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t",
		((s->axisFlag[t] & EMCMOT_JOINT_MIN_HARD_LIMIT_BIT) != 0));
	}
	printf("min_hard_limit\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t",
		((s->axisFlag[t] & EMCMOT_JOINT_HOME_SWITCH_BIT) != 0));
	}
	printf("home_switch\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_HOMING_BIT) != 0));
	}
	printf("homing\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_HOMED_BIT) != 0));
	}
	printf("homed\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_FERROR_BIT) != 0));
	}
	printf("ferror\n");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("%d\t", ((s->axisFlag[t] & EMCMOT_JOINT_FAULT_BIT) != 0));
	}
#endif
	printf("fault\n");
	printf("\npolarity: ");
	printf("limit override mask: %08x\n", s->overrideLimitMask);
	break;

    case 4:
	printf("scales handled in HAL now!\n");
/*! \todo Another #if 0 */
#if 0
	printf("output scales: ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->outputScale[t]);
	}

	printf("\noutput offsets:");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->outputOffset[t]);
	}

	printf("\ninput scales:  ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->inputScale[t]);
	}

	printf("\ninput offsets: ");
	for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	    printf("\t%f", s->inputOffset[t]);
	}

	printf("\n");
#endif
	break;

    default:
	break;
    }
}

static int module_id;
static int shmem_id;

int usrmotInit(const char *modname)
{
    int retval;

    module_id = rtapi_init(modname);
    if (module_id < 0) {
	fprintf(stderr,
	    "usrmotintf: ERROR: rtapi init failed\n");
	return -1;
    }
    /* get shared memory block from RTAPI */
    shmem_id = rtapi_shmem_new(SHMEM_KEY, module_id, sizeof(emcmot_struct_t));
    if (shmem_id < 0) {
	fprintf(stderr,
	    "usrmotintf: ERROR: could not open shared memory\n");
	rtapi_exit(module_id);
	return -1;
    }
    /* get address of shared memory area */
    retval = rtapi_shmem_getptr(shmem_id, (void **) &emcmotStruct);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "usrmotintf: ERROR: could not access shared memory\n");
	rtapi_exit(module_id);
	return -1;
    }
    /* got it */
    emcmotCommand = &(emcmotStruct->command);
    emcmotStatus = &(emcmotStruct->status);
    emcmotDebug = &(emcmotStruct->debug);
    emcmotConfig = &(emcmotStruct->config);
    emcmotError = &(emcmotStruct->error);

    inited = 1;

    return 0;
}

int usrmotExit(void)
{
    if (NULL != emcmotStruct) {
	rtapi_shmem_delete(shmem_id, module_id);
	rtapi_exit(module_id);
    }

    emcmotStruct = 0;
    emcmotCommand = 0;
    emcmotStatus = 0;
    emcmotError = 0;
/*! \todo Another #if 0 */
#if 0
/*! \todo FIXME - comp structs no longer in shmem */
    for (axis = 0; axis < EMCMOT_MAX_JOINTS; axis++) {
	emcmotComp[axis] = 0;
    }
#endif

    inited = 0;
    return 0;
}

/* Loads pairs of comp from the compensation file.
   The default way is to specify nominal, forward & reverse triplets in the file
   However if type != 0, it expects nominal, forward_trim & reverse_trim 
	(where forward_trim = nominal - forward
	       reverse_trim = nominal - reverse)
*/
int usrmotLoadComp(int joint, const char *file, int type)
{
    FILE *fp;
    char buffer[LINELEN];
    double nom, fwd, rev;
    int ret = 0;
    emcmot_command_t emcmotCommand;

    /* check axis range */
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	fprintf(stderr, "joint out of range for compensation\n");
	return -1;
    }

    /* open input comp file */
    if (NULL == (fp = fopen(file, "r"))) {
	fprintf(stderr, "can't open compensation file %s\n", file);
	return -1;
    }

    while (!feof(fp)) {
	if (NULL == fgets(buffer, LINELEN, fp)) {
	    break;
	}
	if (3 != sscanf(buffer, "%lf %lf %lf", &nom, &fwd, &rev)) {
	    break;
	} else {
	    // got a triplet
	    if (type == 0) {
		/* expecting nominal-forward-reverse triplets, e.g., 
		    0.000000 0.000000 -0.001279 
		    0.100000 0.098742  0.051632 
		    0.200000 0.171529  0.194216 */
    		emcmotCommand.comp_nominal = nom;
    		emcmotCommand.comp_forward = nom - fwd; //convert to diffs
    		emcmotCommand.comp_reverse = nom - rev; //convert to diffs
	    } else {
		/* expecting nominal-forw_trim-rev_trim triplets */
    		emcmotCommand.comp_nominal = nom;
    		emcmotCommand.comp_forward = fwd;
    		emcmotCommand.comp_reverse = rev;		
	    }
	    emcmotCommand.axis = joint;
	    emcmotCommand.command = EMCMOT_SET_JOINT_COMP;
	    ret |= usrmotWriteEmcmotCommand(&emcmotCommand);
	}
    }
    fclose(fp);

    return ret;
}


int usrmotPrintComp(int joint)
{
/* FIXME-AJ: comp isn't in shmem atm
  it's in the joint struct, which is only in shmem when STRUCTS_IN_SHM is defined,
  currently only usrmot uses usrmotPrintComp - might go away */
return -1;
/* currently disabled */
#if 0
    int t;

    /* check axis range */
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	fprintf(stderr, "joint out of range for compensation\n");
	return -1;
    }

    /* first check if comp pointer is valid */
    if (emcmotComp[joint] == 0) {
	fprintf(stderr, "compensation data structure not present\n");
	return -1;
    }

    printf("total:  %d\n", emcmotComp[joint]->total);
    printf("avgint: %f\n", emcmotComp[joint]->avgint);
    for (t = 0; t < emcmotComp[joint]->total; t++) {
	printf("%f\t%f\t%f\n",
	    emcmotComp[joint]->nominal[t],
	    emcmotComp[joint]->forward[t], emcmotComp[joint]->reverse[t]);
    }

    return 0;
#endif
}
