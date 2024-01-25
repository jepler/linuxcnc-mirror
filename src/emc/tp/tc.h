/********************************************************************
* Description: tc.h
*   Discriminate-based trajectory planning
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
#ifndef TC_H
#define TC_H

#include "spherical_arc.h"
#include "posemath.h"
#include "emcpos.h"
#include "emcmotcfg.h"
#include "tc_types.h"
#include "tp_types.h"

double tcGetMaxTargetVel(TC_STRUCT const * const tc,
        double max_scale);

double tcGetAccelScale(TC_STRUCT const * tc);
double tcGetOverallMaxAccel(TC_STRUCT const * tc);
double tcGetTangentialMaxAccel(TC_STRUCT const * const tc);

int tcSetKinkProperties(TC_STRUCT *prev_tc, TC_STRUCT *tc, double kink_vel, double accel_reduction);
int tcInitKinkProperties(TC_STRUCT *tc);
int tcRemoveKinkProperties(TC_STRUCT *prev_tc, TC_STRUCT *tc);
int tcGetEndpoint(TC_STRUCT const * const tc, EmcPose * const out);
int tcGetStartpoint(TC_STRUCT const * const tc, EmcPose * const out);
int tcGetPos(TC_STRUCT const * const tc,  EmcPose * const out);
int tcGetPosReal(TC_STRUCT const * const tc, int of_endpoint,  EmcPose * const out);
int tcGetEndAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out);
int tcGetStartAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out);
int tcGetEndTangentUnitVector(TC_STRUCT const * const tc, PmCartesian * const out);
int tcGetStartTangentUnitVector(TC_STRUCT const * const tc, PmCartesian * const out);

int tcGetIntersectionPoint(TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc, PmCartesian * const point);

int tcCanConsume(TC_STRUCT const * const tc);

int tcSetTermCond(TC_STRUCT * tc, TC_STRUCT * nexttc, tc_term_cond_t term_cond);

int tcConnectBlendArc(TC_STRUCT * const prev_tc, TC_STRUCT * const tc,
        PmCartesian const * const circ_start,
        PmCartesian const * const circ_end);

int tcIsBlending(TC_STRUCT * const tc);


int tcFindBlendTolerance(TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc, double * const T_blend, double * const nominal_tolerance);

int pmCircleTangentVector(PmCircle const * const circle,
        double angle_in, PmCartesian * const out);

int tcFlagEarlyStop(TC_STRUCT * const tc,
        TC_STRUCT * const nexttc);

double pmLine9Target(PmLine9 * const line9, int pure_angular);

int pmLine9Init(PmLine9 * const line9,
        EmcPose const * const start,
        EmcPose const * const end);

double pmCircle9Target(PmCircle9 const * const circ9);

int pmCircle9Init(PmCircle9 * const circ9,
        EmcPose const * const start,
        EmcPose const * const end,
        PmCartesian const * const center,
        PmCartesian const * const normal,
        int turn);

int pmRigidTapInit(PmRigidTap * const tap,
        EmcPose const * const start,
        EmcPose const * const end);

double pmRigidTapTarget(PmRigidTap * const tap, double uu_per_rev);

int tcInit(TC_STRUCT * const tc,
        tc_motion_type_t motion_type,
        int canon_motion_type,
        double cycle_time,
        unsigned char enables,
        char atspeed);

int tcSetupFromTP(TC_STRUCT * const tc, TP_STRUCT const * const tp);

int tcSetupMotion(TC_STRUCT * const tc,
        double vel,
        double ini_maxvel,
        double acc);

int tcSetupState(TC_STRUCT * const tc, TP_STRUCT const * const tp);

int tcUpdateCircleAccRatio(TC_STRUCT * tc);

int tcFinalizeLength(TC_STRUCT * const tc);

int tcClampVelocityByLength(TC_STRUCT * const tc);

int tcPureRotaryCheck(TC_STRUCT const * const tc);

int tcSetCircleXYZ(TC_STRUCT * const tc, PmCircle const * const circ);

int tcClearFlags(TC_STRUCT * const tc);

const char *tcTermCondAsString(tc_term_cond_t c);
const char *tcMotionTypeAsString(tc_motion_type_t c);
const char *tcSyncModeAsString(tc_spindle_sync_t c);

#endif				/* TC_H */
