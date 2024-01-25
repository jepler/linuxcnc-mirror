#ifndef JOINT_UTIL_H
#define JOINT_UTIL_H

#include <posemath.h>
#include <emcpos.h>

PmCartesian getXYZAccelBounds();

PmCartesian getXYZVelBounds();

unsigned findAccelViolations(EmcPose axis_accel);
unsigned findVelocityViolations(EmcPose axis_vel);
unsigned findPositionLimitViolations(EmcPose axis_pos);

/**
 * Finds the smallest non-zero component in a non-negative "bounds" vector.
 * Used to identify the "slowest" axis, but ignore axes
 */
double findMinNonZero(PmCartesian const * const bounds);

#endif // JOINT_UTIL_H
