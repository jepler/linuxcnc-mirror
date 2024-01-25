/********************************************************************
* Description: blendmath.h
*   Circular arc blend math functions
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2014 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef BLENDMATH_H
#define BLENDMATH_H

#include "posemath.h"
#include "tc_types.h"

#define BLEND_ACC_RATIO_TANGENTIAL 0.5
#define BLEND_ACC_RATIO_NORMAL (pmSqrt(1.0 - pmSq(BLEND_ACC_RATIO_TANGENTIAL)))
#define BLEND_KINK_FACTOR 0.25

typedef enum {
    BLEND_NONE,
    BLEND_LINE_LINE,
    BLEND_LINE_ARC,
    BLEND_ARC_LINE,
    BLEND_ARC_ARC,
} blend_type_t;

/**
 * 3D Input geometry for a spherical blend arc. 
 * This structure contains all of the basic geometry in 3D for a blend arc.
 */
typedef struct {
    PmCartesian u1;         /* unit vector along line 1 */
    PmCartesian u2;         /* unit vector along line 2 */
    PmCartesian P;          /* Intersection point */
    PmCartesian normal;     /* normal unit vector to plane containing lines */
    PmCartesian binormal;   /* binormal unit vector to plane containing lines */
    PmCartesian u_tan1;     /* Actual tangent vector to 1 (used for arcs only) */
    PmCartesian u_tan2;     /* Actual tangent vector to 2 (used for arcs only) */
    PmCartesian center1;    /* Local approximation of center for arc 1 */
    PmCartesian center2;    /* Local approximation of center for arc 2 */
    double radius1;         /* Local approximation of radius */
    double radius2;
    double theta_tan;
    double v_max1;          /* maximum velocity in direction u_tan1 */
    double v_max2;          /* maximum velocity in direction u_tan2 */

} BlendGeom3;

/**
 * 9D Input geometry for a spherical blend arc.
 */
#ifdef BLEND_9D
typedef struct {
//Not implemented yet
} BlendGeom9;
#endif 


/**
 * Blend arc parameters (abstracted).
 * This structure holds blend arc parameters that have been abstracted from the
 * physical geometry. This data is used to find the maximum radius given the
 * constraints on the blend. By abstracting the parameters from the geometry,
 * the same calculations can be used with any input geometry (lines, arcs, 6 or
 * 9 dimensional lines). 
 */
typedef struct {
    double tolerance;   /* Net blend tolerance (min of line 1 and 2) */
    double L1;          /* Available part of line 1 to blend over */
    double L2;          /* Available part of line 2 to blend over */
    double v_req;       /* requsted velocity for the blend arc */
    double a_max;       /* max acceleration allowed for blend */

    /* These fields are considered "output", and may be refactored into a
     * separate structure in the future */
    
    double theta;       /* Intersection angle, half of angle between -u1 and u2 */
    double phi;         /* supplement of intersection angle, angle between u1 and u2 */
    double a_n_max;     /* max normal acceleration allowed */

    double R_plan;      /* planned radius for blend arc */
    double d_plan;      /* distance along each line to arc endpoints */

    double v_goal;      /* desired velocity at max feed override */
    double v_plan;      /* planned max velocity at max feed override */
    double v_actual;    /* velocity at feedscale = 1.0 */
    double s_arc;       /* arc length */
    int consume;        /* Consume the previous segment */
    double line_length;
    //Arc specific stuff
    int convex1;
    int convex2;
    double phi1_max;
    double phi2_max;
    
} BlendParameters;


/**
 * Output geometry in 3D.
 * Stores the three points representing a simple 3D spherical arc.
 */
typedef struct {
    PmCartesian arc_start;      /* start point for blend arc */
    PmCartesian arc_end;        /* end point for blend arc */
    PmCartesian arc_center;     /* center point for blend arc */
    double trim1;               /* length (line) or angle (arc) to cut from prev_tc */
    double trim2;               /* length (line) or angle (arc) to cut from tc */
} BlendPoints3;



#ifdef BLEND_9D
typedef struct {
//Not implemented yet
} BlendPoints9;
#endif

double findMaxTangentAngle(double v, double acc, double cycle_time);

double findKinkAccel(double kink_angle, double v_plan, double cycle_time);

double fsign(double f);

int clip_min(double * const x, double min);

int clip_max(double * const x, double max);

double saturate(double x, double max);

int sat_inplace(double * const x, double max);

int checkTangentAngle(PmCircle const * const circ, SphericalArc const * const arc, BlendGeom3 const * const geom, BlendParameters const * const param, double cycle_time, int at_end);

int findIntersectionAngle(PmCartesian const * const u1,
        PmCartesian const * const u2, double * const theta);

double pmCartMin(PmCartesian const * const in);

int calculateInscribedRadius(PmCartesian const * const normal,
        PmCartesian const * const bounds, double * const radius);

int findAccelScale(PmCartesian const * const acc,
        PmCartesian const * const bounds,
        PmCartesian * const scale);

double findTrapezoidalDesiredVel(double a_max,
                              double dx,
                              double v_final,
                              double currentvel,
                              double cycle_time);

int pmUnitCartsColinear(PmCartesian const * const u1,
        PmCartesian const * const u2);

int pmCartCartParallel(PmCartesian const * const u1,
        PmCartesian const * const u2,
        double tol);

int pmCartCartAntiParallel(PmCartesian const * const u1,
        PmCartesian const * const u2,
        double tol);

int pmCircLineCoplanar(PmCircle const * const circ,
        PmCartLine const * const line, double tol);

int blendCoplanarCheck(PmCartesian const * const normal,
        PmCartesian const * const u1_tan,
        PmCartesian const * const u2_tan,
        double tol);

int blendCalculateNormals3(BlendGeom3 * const geom);

int blendComputeParameters(BlendParameters * const param);

int blendCheckConsume(BlendParameters * const param,
        BlendPoints3 const * const points,
        TC_STRUCT const * const prev_tc, int gap_cycles);

int blendFindPoints3(BlendPoints3 * const points, BlendGeom3 const * const geom,
        BlendParameters const * const param);

int blendGeom3Init(BlendGeom3 * const geom,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc);

int blendParamKinematics(BlendGeom3 * const geom,
        BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale);

int blendInit3FromLineLine(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale);

int blendInit3FromLineArc(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale);

int blendInit3FromArcLine(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale);

int blendInit3FromArcArc(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale);

int blendArcArcPostProcess(BlendPoints3 * const points, BlendPoints3 const * const points_in,
        BlendParameters * const param, BlendGeom3 const * const geom,
        PmCircle const * const circ1, PmCircle const * const circ2);

int blendLineArcPostProcess(BlendPoints3 * const points, BlendPoints3 const * const points_in,
        BlendParameters * const param, BlendGeom3 const * const geom,
        PmCartLine const * const line1, PmCircle const * const circ2);

int blendArcLinePostProcess(BlendPoints3 * const points, BlendPoints3 const * const points_in,
        BlendParameters * const param, BlendGeom3 const * const geom,
        PmCircle const * const circ1, PmCartLine const * const line2);

int arcFromBlendPoints3(SphericalArc * const arc, BlendPoints3 const * const points,
        BlendGeom3 const * const geom, BlendParameters const * const param);

//Not implemented yet
int blendGeom3Print(BlendGeom3 const * const geom);
int blendParamPrint(BlendParameters const * const param);
int blendPoints3Print(BlendPoints3 const * const points);

double pmCartAbsMax(PmCartesian const * const v);

typedef struct {
    double v_max;
    double acc_ratio;
} PmCircleLimits;

PmCircleLimits pmCircleActualMaxVel(const PmCircle *circle,
        double v_max_nominal,
        double a_max_nominal);

int findSpiralArcLengthFit(PmCircle const * const circle,
        SpiralArcLengthFit * const fit);
int pmCircleAngleFromProgress(PmCircle const * const circle,
        SpiralArcLengthFit const * const fit,
        double progress,
        double * const angle);
double pmCircleEffectiveMinRadius(const PmCircle *circle);
double spiralEffectiveRadius(PmCircle const * circle);

static inline double findVPeak(double a_t_max, double distance)
{
    return pmSqrt(a_t_max * distance);
}

// Start by mimicing the current structure for minimal changes
typedef struct {
    double v_f;
    double dt;
} EndCondition;

EndCondition checkEndCondition(double cycleTime,
                               double progress,
                               double target,
                               double currentvel,
                               double v_f,
                               double a_max);

#endif
