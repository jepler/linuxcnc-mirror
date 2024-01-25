/********************************************************************
* Description: emcpose.c
*
*   Miscellaneous functions to handle EmcPose operations
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2014 All rights reserved.
*
********************************************************************/

#include "emcpose.h"
#include "posemath.h"
#include "rtapi_math.h"
#include "spherical_arc.h"
#include "blendmath.h"

#include "tp_debug.h"

//#define EMCPOSE_PEDANTIC

void emcPoseZero(EmcPose * const pos) {
#ifdef EMCPOSE_PEDANTIC
    if(!pos) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif

    pos->tran.x = 0.0;               
    pos->tran.y = 0.0;               
    pos->tran.z = 0.0;               
    pos->a = 0.0;                    
    pos->b = 0.0;                    
    pos->c = 0.0;                    
    pos->u = 0.0;                    
    pos->v = 0.0;                    
    pos->w = 0.0;
}


int emcPoseAdd(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out)
{
#ifdef EMCPOSE_PEDANTIC
    if (!p1 || !p2) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif

    pmCartCartAdd(&p1->tran, &p2->tran, &out->tran);
    out->a = p1->a + p2->a;
    out->b = p1->b + p2->b;
    out->c = p1->c + p2->c;
    out->u = p1->u + p2->u;
    out->v = p1->v + p2->v;
    out->w = p1->w + p2->w;
    return EMCPOSE_ERR_OK;
}

int emcPoseSub(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out)
{
#ifdef EMCPOSE_PEDANTIC
    if (!p1 || !p2) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif

    pmCartCartSub(&p1->tran, &p2->tran, &out->tran);
    out->a = p1->a - p2->a;
    out->b = p1->b - p2->b;
    out->c = p1->c - p2->c;
    out->u = p1->u - p2->u;
    out->v = p1->v - p2->v;
    out->w = p1->w - p2->w;
    return EMCPOSE_ERR_OK;

}

int emcPoseMultScalar(EmcPose * const p1, double m)
{
#ifdef EMCPOSE_PEDANTIC
    if (!p1 || !p2) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif

    pmCartScalMultEq(&p1->tran, m);
    p1->a *= m;
    p1->b *= m;
    p1->c *= m;
    p1->u *= m;
    p1->v *= m;
    p1->w *= m;
    return EMCPOSE_ERR_OK;
}


int emcPoseSelfAdd(EmcPose * const self, EmcPose const * const p2)
{
    return emcPoseAdd(self, p2, self);
}

int emcPoseSelfSub(EmcPose * const self, EmcPose const * const p2)
{
    return emcPoseSub(self, p2, self);
}

int emcPoseToPmCartesian(EmcPose const * const pose,
        PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw)
{

#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_INPUT_MISSING;
    } 
    if (!xyz | !abc || !uvw) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
#endif

    //Direct copy of translation struct for xyz
    *xyz = pose->tran;

    //Convert ABCUVW axes into 2 pairs of 3D lines
    abc->x = pose->a;
    abc->y = pose->b;
    abc->z = pose->c;

    uvw->x = pose->u;
    uvw->y = pose->v;
    uvw->z = pose->w;
    return EMCPOSE_ERR_OK;
}


/**
 * Collect PmCartesian elements into 9D EmcPose structure.
 */
int pmCartesianToEmcPose(PmCartesian const * const xyz,
        PmCartesian const * const abc, PmCartesian const * const uvw, EmcPose * const pose)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz || !abc || !uvw) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif
    //Direct copy of translation struct for xyz
    pose->tran = *xyz;

    pose->a = abc->x;
    pose->b = abc->y;
    pose->c = abc->z;

    pose->u = uvw->x;
    pose->v = uvw->y;
    pose->w = uvw->z;
    return EMCPOSE_ERR_OK;
}


int emcPoseSetXYZ(PmCartesian const * const xyz, EmcPose * const pose)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif

    pose->tran.x = xyz->x;
    pose->tran.y = xyz->y;
    pose->tran.z = xyz->z;
    return EMCPOSE_ERR_OK;
}


int emcPoseSetABC(PmCartesian const * const abc, EmcPose * const pose)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!abc) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif

    pose->a = abc->x;
    pose->b = abc->y;
    pose->c = abc->z;
    return EMCPOSE_ERR_OK;
}


int emcPoseSetUVW(PmCartesian const * const uvw, EmcPose * const pose)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!uvw) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif

    pose->u = uvw->x;
    pose->v = uvw->y;
    pose->w = uvw->z;

    return EMCPOSE_ERR_OK;
}


int emcPoseGetXYZ(EmcPose const * const pose, PmCartesian * const xyz)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif

    xyz->x = pose->tran.x;
    xyz->y = pose->tran.y;
    xyz->z = pose->tran.z;
    return EMCPOSE_ERR_OK;
}


int emcPoseGetABC(EmcPose const * const pose, PmCartesian * const abc)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!abc) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif

    abc->x = pose->a;
    abc->y = pose->b;
    abc->z = pose->c;
    return EMCPOSE_ERR_OK;
}


int emcPoseGetUVW(EmcPose const * const pose, PmCartesian * const uvw)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!uvw) {
        return EMCPOSE_ERR_INPUT_MISSING;
   }
#endif

    uvw->x = pose->u;
    uvw->y = pose->v;
    uvw->z = pose->w;

    return EMCPOSE_ERR_OK;
}


/**
 * Find the magnitude of an EmcPose position, treating it like a single vector.
 */
int emcPoseMagnitude(EmcPose const * const pose, double * const out) {

#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
    if (!out) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
#endif

    double mag = 0.0;
    mag += pmSq(pose->tran.x);
    mag += pmSq(pose->tran.y);
    mag += pmSq(pose->tran.z);
    mag += pmSq(pose->a);
    mag += pmSq(pose->b);
    mag += pmSq(pose->c);
    mag += pmSq(pose->u);
    mag += pmSq(pose->v);
    mag += pmSq(pose->w);
    mag = pmSqrt(mag);

    *out = mag;
    return EMCPOSE_ERR_OK;
}


/**
 * Return true for a numerically valid pose, or false for an invalid pose (or null pointer).
 */
int emcPoseValid(EmcPose const * const pose)
{

    if (!pose || 
            isnan(pose->tran.x) ||
            isnan(pose->tran.y) ||
            isnan(pose->tran.z) ||
            isnan(pose->a) ||
            isnan(pose->b) ||
            isnan(pose->c) ||
            isnan(pose->u) ||
            isnan(pose->v) ||
            isnan(pose->w)) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * Checks all axis values in an EmcPose to see if they exceed a magnitude threshold.
 * @return a bitmask that is 0 if all axes are within the threshold. Any
 * out-of-limit axes set their corresponding bit to 1 in the returned value (X
 * is 0th bit, Y is 1st, etc.).
 */
unsigned int findAbsThresholdViolations(EmcPose vec, double threshold)
{
    threshold = fabs(threshold);
    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0
            | (fabs(vec.tran.x) > threshold) << 0
            | (fabs(vec.tran.y) > threshold) << 1
            | (fabs(vec.tran.z) > threshold) << 2
            | (fabs(vec.a) > threshold) << 3
            | (fabs(vec.b) > threshold) << 4
            | (fabs(vec.c) > threshold) << 5
            | (fabs(vec.u) > threshold) << 6
            | (fabs(vec.v) > threshold) << 7
            | (fabs(vec.w) > threshold) << 8);
    return fail_bits;
}
