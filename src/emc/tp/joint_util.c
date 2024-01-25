#include "joint_util.h"
#include "motion_debug.h"
#include "rtapi_math.h"

extern emcmot_debug_t *emcmotDebug;

PmCartesian getXYZAccelBounds() {
    PmCartesian acc_bound = {
        emcmotDebug->joints[0].acc_limit,
        emcmotDebug->joints[1].acc_limit,
        emcmotDebug->joints[2].acc_limit,
    };
    return acc_bound;
}

PmCartesian getXYZVelBounds() {
    PmCartesian vel_bound = {
        emcmotDebug->joints[0].vel_limit,
        emcmotDebug->joints[1].vel_limit,
        emcmotDebug->joints[2].vel_limit,
    };
    return vel_bound;
}

unsigned jointAccelViolation(int joint_idx, double acc)
{
    // Allow some numerical tolerance here since max acceleration is a bit
    // fuzzy anyway. Torque is the true limiting factor, so there has to be
    // some slack here anyway due to variations in table load, friction, etc.
    static const double ABS_TOL = 1e-4;
    const double REL_TOL = 1.0 + ABS_TOL;
    const double a_max_nominal = emcmotDebug->joints[joint_idx].acc_limit;
    const double a_limit = fmax(a_max_nominal * REL_TOL, a_max_nominal + ABS_TOL);
    return (unsigned)(fabs(acc) > a_limit) << joint_idx;
}

unsigned jointVelocityViolation(int joint_idx, double v_actual)
{
    // Allow some numerical tolerance here since max acceleration is a bit
    // fuzzy anyway. Torque is the true limiting factor, so there has to be
    // some slack here anyway due to variations in table load, friction, etc.
    static const double ABS_TOL = 1e-2;
    const double REL_ACC_TOL = 1.0 + ABS_TOL;
    const double v_max_nominal = emcmotDebug->joints[joint_idx].vel_limit;
    const double v_limit = fmax(v_max_nominal * REL_ACC_TOL, v_max_nominal + ABS_TOL);
    return (unsigned)(fabs(v_actual) > v_limit) << joint_idx;
}

unsigned jointPositionViolation(int joint_idx, double position)
{
    static const double ABS_TOL = 1e-6;
    return (unsigned)(position < emcmotDebug->joints[joint_idx].min_pos_limit - ABS_TOL &&
           position > emcmotDebug->joints[joint_idx].max_pos_limit + ABS_TOL) << joint_idx;
}

/**
 * Checks for any acceleration violations based on axis limits.
 *
 * @return 0 if all acceleration bounds are respected, or a bit-set of failed axes (in XYZABCUVW bit order).
 */
unsigned findAccelViolations(EmcPose axis_accel)
{
    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0
        | jointAccelViolation(0, axis_accel.tran.x)
        | jointAccelViolation(1, axis_accel.tran.y)
        | jointAccelViolation(2, axis_accel.tran.z)
        | jointAccelViolation(3, axis_accel.a)
        | jointAccelViolation(4, axis_accel.b)
        | jointAccelViolation(5, axis_accel.c)
        | jointAccelViolation(6, axis_accel.u)
        | jointAccelViolation(7, axis_accel.v)
        | jointAccelViolation(8, axis_accel.w));
    return fail_bits;
}

unsigned findVelocityViolations(EmcPose axis_vel)
{
    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0
        | jointVelocityViolation(0, axis_vel.tran.x)
        | jointVelocityViolation(1, axis_vel.tran.y)
        | jointVelocityViolation(2, axis_vel.tran.z)
        | jointVelocityViolation(3, axis_vel.a)
        | jointVelocityViolation(4, axis_vel.b)
        | jointVelocityViolation(5, axis_vel.c)
        | jointVelocityViolation(6, axis_vel.u)
        | jointVelocityViolation(7, axis_vel.v)
        | jointVelocityViolation(8, axis_vel.w));
    return fail_bits;
}

unsigned findPositionLimitViolations(EmcPose axis_pos)
{
    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0
        | jointPositionViolation(0, axis_pos.tran.x)
        | jointPositionViolation(1, axis_pos.tran.y)
        | jointPositionViolation(2, axis_pos.tran.z)
        | jointPositionViolation(3, axis_pos.a)
        | jointPositionViolation(4, axis_pos.b)
        | jointPositionViolation(5, axis_pos.c)
        | jointPositionViolation(6, axis_pos.u)
        | jointPositionViolation(7, axis_pos.v)
        | jointPositionViolation(8, axis_pos.w));
    return fail_bits;
}


double findMinNonZero(const PmCartesian * const bounds) {
    //Start with max accel value
    double act_limit = fmax(fmax(bounds->x, bounds->y), bounds->z);

    // Compare only with active axes
    if (bounds->x > 0) {
        act_limit = fmin(act_limit, bounds->x);
    }
    if (bounds->y > 0) {
        act_limit = fmin(act_limit, bounds->y);
    }
    if (bounds->z > 0) {
        act_limit = fmin(act_limit, bounds->z);
    }
    return act_limit;
}
