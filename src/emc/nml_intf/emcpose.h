/********************************************************************
* Description: emcpose.h
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef EMCPOSE_H
#define EMCPOSE_H

#include "emcpos.h"
#include "posemath.h"

typedef enum {
    EMCPOSE_ERR_OK = 0,
    EMCPOSE_ERR_FAIL = -1,
    EMCPOSE_ERR_INPUT_MISSING = -2,
    EMCPOSE_ERR_OUTPUT_MISSING = -3,
    EMCPOSE_ERR_ALL
} EmcPoseErr;

void emcPoseZero(EmcPose * const pos);

int emcPoseAdd(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out);
int emcPoseSub(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out);

int emcPoseMultScalar(EmcPose * const p1, double m);

int emcPoseToPmCartesian(EmcPose const * const pose,
        PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw);
int pmCartesianToEmcPose(PmCartesian const * const xyz,
        PmCartesian const * const abc, PmCartesian const * const uvw, EmcPose * const pose);

int emcPoseSelfAdd(EmcPose * const self, EmcPose const * const p2);
int emcPoseSelfSub(EmcPose * const self, EmcPose const * const p2);

int emcPoseSetXYZ(PmCartesian const * const xyz, EmcPose * const pose);
int emcPoseSetABC(PmCartesian const * const abc, EmcPose * const pose);
int emcPoseSetUVW(PmCartesian const * const uvw, EmcPose * const pose);

int emcPoseGetXYZ(EmcPose const * const pose, PmCartesian * const xyz);
int emcPoseGetABC(EmcPose const * const pose, PmCartesian * const abc);
int emcPoseGetUVW(EmcPose const * const pose, PmCartesian * const uvw);

int emcPoseMagnitude(EmcPose const * const pose, double * const out);

unsigned int findAbsThresholdViolations(EmcPose vec, double threshold);

int emcPoseValid(EmcPose const * const pose);

#endif
