/********************************************************************
* Description: tp.h
*   Trajectory planner based on TC elements
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef TP_H
#define TP_H

#include "posemath.h"
#include "tc_types.h"
#include "tp_types.h"
#include "tcq.h"

int tpCreate(TP_STRUCT * const tp, int _queueSize, TC_STRUCT * const tcSpace);
int tpClear(TP_STRUCT * const tp);
int tpInit(TP_STRUCT * const tp);
int tpClearDIOs(TP_STRUCT * const tp);
int tpSetCycleTime(TP_STRUCT * const tp, double secs);
int tpSetVmax(TP_STRUCT * const tp, double vmax, double ini_maxvel);
int tpSetVlimit(TP_STRUCT * const tp, double vLimit);
int tpSetId(TP_STRUCT * const tp, int id);
int tpGetExecId(TP_STRUCT * const tp);
int tpSetTermCond(TP_STRUCT * const tp, tc_term_cond_t cond, double tolerance);
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos);
int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp);
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos);

int tpAddRigidTap(TP_STRUCT * const tp,
        EmcPose end,
        double vel,
        double ini_maxvel,
        double acc,
        unsigned char enables);

int tpAddLine(TP_STRUCT * const tp,
        EmcPose end,
        int canon_motion_type,
        double vel,
        double ini_maxvel,
        double acc,
        unsigned char enables,
        char atspeed,
        int indexrotary,
        int pure_angular);

int tpAddCircle(TP_STRUCT * const tp,
        EmcPose end,
        PmCartesian center,
        PmCartesian normal,
        int turn,
        int canon_motion_type,
        double vel,
        double ini_maxvel,
        double acc,
        double acc_normal,
        unsigned char enables,
        char atspeed);

int tpRunCycle(TP_STRUCT * const tp, long period);
int tpPause(TP_STRUCT * const tp);
int tpResume(TP_STRUCT * const tp);
int tpAbort(TP_STRUCT * const tp);
int tpGetPos(TP_STRUCT const  * const tp, EmcPose * const pos);
int tpIsDone(TP_STRUCT * const tp);
int tpQueueDepth(TP_STRUCT * const tp);
int tpActiveDepth(TP_STRUCT * const tp);
int tpGetMotionType(TP_STRUCT * const tp);
int tpSetSpindleSync(TP_STRUCT * const tp, double sync, int wait);
void tpToggleDIOs(TC_STRUCT * const tc); //gets called when a new tc is taken from the queue. it checks and toggles all needed DIO's

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end);
int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end); //gets called to place DIO toggles on the TC queue

#endif				/* TP_H */
