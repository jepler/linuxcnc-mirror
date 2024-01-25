/********************************************************************
 * Description: spherical_arc.h
 *
 * A simple spherical linear interpolation library and related functions.
 *
 * Author: Robert W. Ellenberg
 * License: GPL Version 2
 * System: Linux
 *    
 * Copyright (c) 2014 All rights reserved.
 *
 ********************************************************************/
#ifndef SPHERICAL_ARC_H
#define SPHERICAL_ARC_H

#include "posemath.h"

#define ARC_POS_EPSILON 1e-12
#define ARC_MIN_RADIUS 1e-12
#define ARC_MIN_ANGLE 1e-6
//FIXME relate this to cornering acceleration?
#define ARC_ABS_ERR 5e-4
#define ARC_REL_ERR 5e-4

typedef struct {
    // Three defining points for the arc
    PmCartesian start;
    PmCartesian end;
    PmCartesian center;
    // Relative vectors from center to start and center to end
    // These are cached here since they'll be reused during SLERP
    PmCartesian rStart;
    PmCartesian rEnd;
    PmCartesian uTan;   /* Tangent vector at start of arc (copied from
                           prev. tangent line)*/
    PmCartesian binormal;
    double radius;
    double spiral;
    // Angle that the arc encloses
    double angle;
    double Sangle;
    double line_length;
} SphericalArc;


int arcInitFromPoints(SphericalArc * const arc, PmCartesian const * const start,
        PmCartesian const * const end, PmCartesian const * const center);

int arcInitFromVectors(SphericalArc * const arc, PmCartesian const * const vec0,
        PmCartesian const * const vec1,
        PmCartesian const * const center);

int arcPoint(SphericalArc const * const arc, double angle_in, PmCartesian * const out);

int arcNormalizedSlerp(SphericalArc const * const arc, double t, PmCartesian * const out);

int arcLength(SphericalArc const * const arc, double * const length);

int arcFromLines(SphericalArc * const arc, PmCartLine const * const line1,
        PmCartLine const * const line2, double radius,
        double blend_dist, double center_dist, PmCartesian * const start, PmCartesian * const end, int consume);

int checkRayIntersectsArc(PmCartesian const * const center,
        PmCartesian const * const P, PmCartesian const * const u_ray, int reverse_dir);

int arcTangent(SphericalArc const * const arc, PmCartesian * const tan, int at_end);
#endif
