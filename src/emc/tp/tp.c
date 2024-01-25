/********************************************************************
* Description: tp.c
*   Trajectory planner based on TC elements
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/
#include "rtapi.h"              /* rtapi_print_msg */
#include "posemath.h"           /* Geometry types & functions */
#include "tc.h"
#include "tp.h"
#include "emcpose.h"
#include "rtapi_math.h"
#include "mot_priv.h"
#include "motion_debug.h"
#include "motion_types.h"
#include "spherical_arc.h"
#include "blendmath.h"
#include "math_util.h"
#include "joint_util.h"
#include "string.h"

//KLUDGE Don't include all of emc.hh here, just hand-copy the TERM COND
//definitions until we can break the emc constants out into a separate file.
//#include "emc.hh"
#define EMC_TRAJ_TERM_COND_STOP  0
#define EMC_TRAJ_TERM_COND_EXACT 1
#define EMC_TRAJ_TERM_COND_BLEND 2

/**
 * @section tpdebugflags TP debugging flags
 * Enable / disable various debugging functions here.
 * These flags control debug printing from RTAPI. These functions are
 * admittedly kludged on top of the existing rtapi_print framework. As written,
 * though, it's an easy way to selectively compile functions as static or not,
 * and selectively compile in assertions and debug printing.
 */

#include "tp_debug.h"

// FIXME: turn off this feature, which causes blends between rapids to
// use the feed override instead of the rapid override
#undef TP_SHOW_BLENDS

#define TP_OPTIMIZATION_LAZY

extern emcmot_status_t *emcmotStatus;
extern emcmot_debug_t *emcmotDebug;
extern emcmot_config_t *emcmotConfig;

/** static function primitives (ugly but less of a pain than moving code around)*/
STATIC int tpComputeBlendVelocity(
        TC_STRUCT const *tc,
        TC_STRUCT const *nexttc,
        double v_target_this,
        double v_target_next,
        double *v_blend_this,
        double *v_blend_next,
        double *v_blend_net);

STATIC double estimateParabolicBlendPerformance(
        TP_STRUCT const *tp,
        TC_STRUCT const *tc,
        TC_STRUCT const *nexttc);

STATIC int tpCheckEndCondition(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc);

typedef enum {
    UPDATE_NORMAL,
    UPDATE_PARABOLIC_BLEND,
    UPDATE_SPLIT
} UpdateCycleMode;

STATIC int tpUpdateCycle(TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT const * const nexttc,
        UpdateCycleMode cycle_mode);

STATIC int tpRunOptimization(TP_STRUCT * const tp);

STATIC inline int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc, int inc_id);

STATIC inline double tpGetMaxTargetVel(TP_STRUCT const * const tp, TC_STRUCT const * const tc);


STATIC int tpHandleBlendArc(TP_STRUCT * const tp, TC_STRUCT * const tc);

typedef struct {
    char axes[9];
} AxisMaskString;

static inline AxisMaskString axisBitMaskToString(unsigned failed_axes)
{
    AxisMaskString axis_str = {{"XYZABCUVW"}};
    int i;
    for (i = 0; i < 9;++i) {
        if (failed_axes & (1 << i)) {
            continue;
        }
        axis_str.axes[i] =' ';
    }
    return axis_str;
}


static inline void reportTPAxisError(TP_STRUCT const *tp, unsigned failed_axes, const char *msg_prefix)
{
    if (failed_axes)
    {
        AxisMaskString failed_axes_str = axisBitMaskToString(failed_axes);
        rtapi_print_msg(RTAPI_MSG_ERR, "%s, axes [%s] at %g sec\n",
                        msg_prefix ?: "unknown error",
                        failed_axes_str.axes,
                        tp->time_elapsed_sec);
    }
}


/**
 * @section tpcheck Internal state check functions.
 * These functions compartmentalize some of the messy state checks.
 * Hopefully this makes changes easier to track as much of the churn will be on small functions.
 */

/**
 * Returns true if there is motion along ABC or UVW axes, false otherwise.
 */
STATIC int tcRotaryMotionCheck(TC_STRUCT const * const tc) {
    switch (tc->motion_type) {
        //Note lack of break statements due to every path returning
        case TC_RIGIDTAP:
            return false;
        case TC_LINEAR:
            if (tc->coords.line.abc.tmag_zero && tc->coords.line.uvw.tmag_zero) {
                return false;
            } else {
                return true;
            }
        case TC_CIRCULAR:
            if (tc->coords.circle.abc.tmag_zero && tc->coords.circle.uvw.tmag_zero) {
                return false;
            } else {
                return true;
            }
        case TC_SPHERICAL:
            return true;
    }
    return false;
}


/**
 * @section tpgetset Internal Get/Set functions
 * @brief Calculation / status functions for commonly used values.
 * These functions return the "actual" values of things like a trajectory
 * segment's feed override, while taking into account the status of tp itself.
 */



/**
 * Wrapper to bounds-check the tangent kink ratio from HAL.
 */
STATIC double tpGetTangentKinkRatio(void) {
    const double max_ratio = 0.7071;
    const double min_ratio = 0.001;

    return fmax(fmin(emcmotConfig->arcBlendTangentKinkRatio,max_ratio),min_ratio);
}


/**
 * Get a segment's feed scale based on the current planner state and emcmotStatus.
 * @note depends on emcmotStatus for system information.
 */
STATIC double tpGetFeedScale(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {
    if (!tc) {
        return 0.0;
    }
    //All reasons to disable feed override go here
    bool pausing = tp->pausing && (tc->synchronized == TC_SYNC_NONE || tc->synchronized == TC_SYNC_VELOCITY);
    bool aborting = tp->aborting;
    if (pausing)  {
        tc_debug_print("pausing\n");
        return 0.0;
    } else if (aborting) {
        tc_debug_print("aborting\n");
        return 0.0;
    } else if (tc->synchronized == TC_SYNC_POSITION ) {
        return 1.0;
    } else if (tc->is_blending) {
        //KLUDGE: Don't allow feed override to keep blending from overruning max velocity
        return fmin(emcmotStatus->net_feed_scale, 1.0);
    } else {
        return emcmotStatus->net_feed_scale;
    }
}


/**
 * Get target velocity for a tc based on the trajectory planner state.
 * This gives the requested velocity, capped by the segments maximum velocity.
 */
STATIC inline double tpGetRealTargetVel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {

    if (!tc) {
        return 0.0;
    }
    // Start with the scaled target velocity based on the current feed scale
    double v_target = tc->synchronized ? tc->target_vel : tc->reqvel;

    // Get the maximum allowed target velocity, and make sure we're below it
    return fmin(v_target * tpGetFeedScale(tp,tc), tpGetMaxTargetVel(tp, tc));
}

STATIC inline double getMaxFeedScale(TC_STRUCT const * tc)
{
    //All reasons to disable feed override go here
    if (tc && tc->synchronized == TC_SYNC_POSITION ) {
        return 1.0;
    } else {
        return emcmotConfig->maxFeedScale;
    }
}

STATIC inline double getMaxBlendFeedScale(TC_STRUCT const * prev_tc, TC_STRUCT const * tc)
{
    //All reasons to disable feed override go here
    if ((tc && tc->synchronized == TC_SYNC_POSITION) ||
            (prev_tc && prev_tc->synchronized == TC_SYNC_POSITION)) {
        return 1.0;
    } else {
        return emcmotConfig->maxFeedScale;
    }
}


/**
 * Get the worst-case target velocity for a segment based on the trajectory planner state.
 * Note that this factors in the user-specified velocity limit.
 */
STATIC inline double tpGetMaxTargetVel(TP_STRUCT const * const tp, TC_STRUCT const * const tc)
{
    double max_scale = getMaxFeedScale(tc);
    if (tc->is_blending) {
        //KLUDGE: Don't allow feed override to keep blending from overruning max velocity
        max_scale = fmin(max_scale, 1.0);
    }
    double v_max_target = tcGetMaxTargetVel(tc, max_scale);

    /* Check if the cartesian velocity limit applies and clip the maximum
     * velocity. The vLimit is from the max velocity slider, and should
     * restrict the maximum velocity during non-synced moves and velocity
     * synchronization. However, position-synced moves have the target velocity
     * computed in the TP, so it would disrupt position tracking to apply this
     * limit here.
     */
    if (!tcPureRotaryCheck(tc) && (tc->synchronized != TC_SYNC_POSITION)){
        /*tc_debug_print("Cartesian velocity limit active\n");*/
        v_max_target = fmin(v_max_target, tp->vLimit);
    }

    return v_max_target;
}


/**
 * Get final velocity for a tc based on the trajectory planner state.
 * This function factors in the feed override and TC limits. It clamps the
 * final velocity to the maximum velocity and the next segment's target velocity
 */
STATIC inline double tpGetRealFinalVel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc, TC_STRUCT const * const nexttc) {
    /* If we're stepping, then it doesn't matter what the optimization says, we want to end at a stop.
     * If the term_cond gets changed out from under us, detect this and force final velocity to zero
     */
    if (emcmotDebug->stepping || tc->term_cond != TC_TERM_COND_TANGENT) {
        return 0.0;
    } 
    
    // Get target velocities for this segment and next segment
    double v_target_this = tpGetRealTargetVel(tp, tc);
    double v_target_next = 0.0;
    if (nexttc) {
        v_target_next = tpGetRealTargetVel(tp, nexttc);
    }

    // Limit final velocity to minimum of this and next target velocities
    double v_target = fmin(v_target_this, v_target_next);
    double finalvel = fmin(tc->finalvel, v_target);
    return finalvel;
}

/** 
 * Set up a spindle origin based on the current spindle COMMANDED direction and the given position.
 *
 * Convert the 2-part spindle position and sign to a signed double.
 * Get acceleration for a tc based on the trajectory planner state.
 * changes in displacement due to sign flips
 */
STATIC inline void resetSpindleOrigin(spindle_origin_t *origin)
{
    if (!origin) {
        return;
    }
    origin->direction = 0.0;
    origin->position = 0.0;
}


/**
 * Set up a spindle origin based on the current spindle COMMANDED direction and the given position.
 *
 * The origin is used to calculate displacements used in spindle position tracking.
 * The direction is stored as part of the origin to prevent discontinuous
 * changes in displacement due to sign flips
 */
STATIC inline void setSpindleOrigin(spindle_origin_t *origin, double position)
{
    if (!origin) {
        return;
    }
    origin->position = position;
    origin->direction = signum(emcmotStatus->spindle_cmd.velocity_rpm_out);
}

/**
 * @section tpaccess tp class-like API
 */

/**
 * Create the trajectory planner structure with an empty queue.
 */
int tpCreate(TP_STRUCT * const tp, int _queueSize, TC_STRUCT * const tcSpace)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (_queueSize <= 0) {
        tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->queueSize = _queueSize;
    }

    /* create the queue */
    if (-1 == tcqCreate(&tp->queue, tp->queueSize, tcSpace)) {
        return TP_ERR_FAIL;
    }

    /* init the rest of our data */
    return tpInit(tp);
}

/**
 * Clears any potential DIO toggles and anychanged.
 * If any DIOs need to be changed: dios[i] = 1, DIO needs to get turned on, -1
 * = off
 */
int tpClearDIOs(TP_STRUCT * const tp) {
    //XXX: All IO's will be flushed on next synced aio/dio! Is it ok?
    int i;
    tp->syncdio.anychanged = 0;
    tp->syncdio.dio_mask = 0;
    tp->syncdio.aio_mask = 0;
    for (i = 0; i < num_dio; i++) {
        tp->syncdio.dios[i] = 0;
    }
    for (i = 0; i < num_aio; i++) {
        tp->syncdio.aios[i] = 0;
    }

    return TP_ERR_OK;
}

/**
 *    "Soft initialize" the trajectory planner tp.
 *    This is a "soft" initialization in that TP_STRUCT configuration
 *    parameters (cycleTime, vMax, and aMax) are left alone, but the queue is
 *    cleared, and the flags are set to an empty, ready queue. The currentPos
 *    is left alone, and goalPos is set to this position.  This function is
 *    intended to put the motion queue in the state it would be if all queued
 *    motions finished at the current position.
 */
int tpClear(TP_STRUCT * const tp)
{
    tcqInit(&tp->queue);
    tp->queueSize = 0;
    tp->goalPos = tp->currentPos;
    ZERO_EMC_POSE(tp->currentVel);
    tp->time_elapsed_sec = 0.0;
    tp->time_elapsed_ticks = 0;
    tp->nextId = 0;
    tp->execId = 0;
    tp->motionType = 0;
    tp->termCond = TC_TERM_COND_PARABOLIC;
    tp->tolerance = 0.0;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->pausing = 0;
    tp->synchronized = 0;
    tp->uu_per_rev = 0.0;
    emcmotStatus->spindle_fb.synced = 0;
    emcmotStatus->current_vel = 0.0;
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->distance_to_go = 0.0;
    emcmotStatus->pos_tracking_error = 0;
    ZERO_EMC_POSE(emcmotStatus->dtg);
    SET_MOTION_INPOS_FLAG(1);

    return tpClearDIOs(tp);
}

/**
 * Fully initialize the tp structure.
 * Sets tp configuration to default values and calls tpClear to create a fresh,
 * empty queue.
 */
int tpInit(TP_STRUCT * const tp)
{
    tp->cycleTime = 0.0;
    //Velocity limits
    tp->vLimit = 0.0;
    tp->ini_maxvel = 0.0;
    //Angular limits
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;

    resetSpindleOrigin(&tp->spindle.origin);
    tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;

    ZERO_EMC_POSE(tp->currentPos);
    ZERO_EMC_POSE(tp->currentVel);

    PmCartesian vel_bound = getXYZVelBounds();
    tp->vMax = findMinNonZero(&vel_bound);

    return tpClear(tp);
}

/**
 * Set the cycle time for the trajectory planner.
 */
int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    if (0 == tp || secs <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->cycleTime = secs;

    return TP_ERR_OK;
}

/**
 * Set requested velocity and absolute maximum velocity (bounded by machine).
 * This is called before adding lines or circles, specifying vMax (the velocity
 * requested by the F word) and ini_maxvel, the max velocity possible before
 * meeting a machine constraint caused by an AXIS's max velocity.  (the TP is
 * allowed to go up to this high when feed override >100% is requested)  These
 * settings apply to subsequent moves until changed.
 */
int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel)
{
    if (0 == tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;

    return TP_ERR_OK;
}

/**
 * (?) Set the tool tip maximum velocity.
 * I think this is the [TRAJ] max velocity. This should be the max velocity of
 * const the TOOL TIP, not necessarily any particular axis. This applies to
 * subsequent moves until changed.
 */
int tpSetVlimit(TP_STRUCT * const tp, double vLimit)
{
    if (!tp) return TP_ERR_FAIL;

    if (vLimit < 0.)
        tp->vLimit = 0.;
    else
        tp->vLimit = vLimit;

    return TP_ERR_OK;
}

/**
 * Sets the id that will be used for the next appended motions.
 * nextId is incremented so that the next time a motion is appended its id will
 * be one more than the previous one, modulo a signed int. If you want your own
 * ids for each motion, call this before each motion you append and stick what
 * you want in here.
 */
int tpSetId(TP_STRUCT * const tp, int id)
{

    if (!MOTION_ID_VALID(id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpSetId: invalid motion id %d\n", id);
        return TP_ERR_FAIL;
    }

    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    tp->nextId = id;

    return TP_ERR_OK;
}

/** Returns the id of the last motion that is currently
  executing.*/
int tpGetExecId(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    return tp->execId;
}

/**
 * Sets the termination condition for all subsequent queued moves.
 * If cond is TC_TERM_COND_STOP, motion comes to a stop before a subsequent move
 * begins. If cond is TC_TERM_COND_PARABOLIC, the following move is begun when the
 * current move slows below a calculated blend velocity.
 */
int tpSetTermCond(TP_STRUCT * const tp, tc_term_cond_t cond, double tolerance)
{
    tp_err_t res = TP_ERR_FAIL;

    switch (cond) {
        //Purposeful waterfall for now
        case TC_TERM_COND_PARABOLIC:
        case TC_TERM_COND_TANGENT:
        case TC_TERM_COND_EXACT:
        case TC_TERM_COND_STOP:
        if (tp) {
            tp->termCond = cond;
            tp->tolerance = tolerance;
            res = TP_ERR_OK;
        }
            break;
    }

    return res;
}

/**
 * Used to tell the tp the initial position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    int res_invalid = tpSetCurrentPos(tp, pos);
    if (res_invalid) {
        return TP_ERR_FAIL;
    }

    tp->goalPos = *pos;
    return TP_ERR_OK;
}


/**
 * Set current position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (emcPoseValid(pos)) {
        tp->currentPos = *pos;
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpSetCurrentPos on id %d!"
                "pos is %.12g, %.12g, %.12g\n",
                tp->execId,
                pos->tran.x,
                pos->tran.y,
                pos->tran.z);
        return TP_ERR_INVALID;
    }
}


int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp)
{
    if (!tp || !disp) {
        return TP_ERR_MISSING_INPUT;
    }

    if (emcPoseValid(disp)) {
        emcPoseSelfAdd(&tp->currentPos, disp);
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpAddCurrentPos on id %d!"
                "disp is %.12g, %.12g, %.12g\n",
                tp->execId,
                disp->tran.x,
                disp->tran.y,
                disp->tran.z);
        return TP_ERR_INVALID;
    }
}


/**
 * Check for valid tp before queueing additional moves.
 */
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return TP_ERR_FAIL;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}


/**
 * Find the "peak" velocity a segment can achieve if its velocity profile is triangular.
 * This is used to estimate blend velocity, though by itself is not enough
 * (since requested velocity and max velocity could be lower).
 */
STATIC double tpCalculateTriangleVel(TC_STRUCT const *tc) {
    //Compute peak velocity for blend calculations
    double acc_scaled = tcGetTangentialMaxAccel(tc);
    double length = tc->target;
    if (!tc->finalized) {
        // blending may remove up to 1/2 of the segment
        length /= 2.0;
    }
    return findVPeak(acc_scaled, length);
}


/**
 * Handles the special case of blending into an unfinalized segment.
 * The problem here is that the last segment in the queue can always be cut
 * short by a blend to the next segment. However, we can only ever consume at
 * most 1/2 of the segment. This function computes the worst-case final
 * velocity the previous segment can have, if we want to exactly stop at the
 * halfway point.
 */
STATIC double tpCalculateOptimizationInitialVel(TP_STRUCT const * const tp, TC_STRUCT * const tc)
{
    double acc_scaled = tcGetTangentialMaxAccel(tc);
    double triangle_vel = findVPeak(acc_scaled, tc->target);
    double max_vel = tpGetMaxTargetVel(tp, tc);
    double optim_init_vel = fmin(triangle_vel, max_vel);
#ifdef TP_DEBUG
    print_json5_log_start(tpCalculateOptimizationInitialVel, Command);
    print_json5_double(triangle_vel);
    print_json5_double(optim_init_vel);
    print_json5_log_end();
#endif
    return optim_init_vel;
}


/**
 * Initialize a blend arc from its parent lines.
 * This copies and initializes properties from the previous and next lines to
 * initialize a blend arc. This function does not handle connecting the
 * segments together, however.
 */
STATIC int tpInitBlendArcFromPrev(TP_STRUCT const * const tp,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT* const blend_tc,
        double vel,
        double ini_maxvel,
        double acc) {

#ifdef TP_SHOW_BLENDS
    int canon_motion_type = EMC_MOTION_TYPE_ARC;
#else
    int canon_motion_type = prev_tc->canon_motion_type;
#endif

    tcInit(blend_tc,
            TC_SPHERICAL,
            canon_motion_type,
            tp->cycleTime,
            prev_tc->enables,
            false); // NOTE: blend arc never needs the atspeed flag, since the previous line will have it (and cannot be consumed).

    // Copy over state data from TP
    tcSetupState(blend_tc, tp);
    blend_tc->term_cond = TC_TERM_COND_TANGENT;

    // Set kinematics parameters from blend calculations
    tcSetupMotion(blend_tc,
            vel,
            ini_maxvel,
            acc);

    // Skip syncdio setup since this blend extends the previous line
    blend_tc->syncdio = prev_tc->syncdio; //enqueue the list of DIOs that need toggling

    // find "helix" length for target
    double length;
    arcLength(&blend_tc->coords.arc.xyz, &length);
    tp_info_print("blend tc length = %f\n",length);
    blend_tc->target = length;
    blend_tc->nominal_length = length;

    //NOTE: blend arc radius and everything else is finalized, so set this to 1.
    //In the future, radius may be adjustable.
    tcFinalizeLength(blend_tc);

    return TP_ERR_OK;
}

STATIC int tcSetLineXYZ(TC_STRUCT * const tc, PmCartLine const * const line)
{

    //Update targets with new arc length
    if (!line || tc->motion_type != TC_LINEAR) {
        return TP_ERR_FAIL;
    }
    if (!tc->coords.line.abc.tmag_zero || !tc->coords.line.uvw.tmag_zero) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetLineXYZ does not supportABC or UVW motion\n");
        return TP_ERR_FAIL;
    }

    tc->coords.line.xyz = *line;
    tc->target = line->tmag;
    return TP_ERR_OK;
}

static inline int find_max_element(double arr[], int sz)
{
    if (sz < 1) {
        return -1;
    }
    // Assumes at least one element
    int max_idx = 0;
    int idx;
    for (idx = 0; idx < sz; ++idx) {
        if (arr[idx] > arr[max_idx]) {
            max_idx = idx;
        }
    }
    return max_idx;
}

const char *blendTypeAsString(tc_blend_type_t c)
{
    switch(c) {
    case NO_BLEND:
        return "NO_BLEND";
    case PARABOLIC_BLEND:
        return "PARABOLIC_BLEND";
    case TANGENT_SEGMENTS_BLEND:
        return "TANGENT_SEGMENTS";
    case ARC_BLEND:
        return "ARC_BLEND";
    }
    return "";
}

/**
 * Compare performance of blend arc and equivalent tangent speed.
 * If we can go faster by assuming the segments are already tangent (and
 * slowing down some), then prefer this over using the blend arc. This is
 * mostly useful for some odd arc-to-arc cases where the blend arc becomes very
 * short (and therefore slow).
 */
STATIC tc_blend_type_t tpChooseBestBlend(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc,
        TC_STRUCT * const tc,
        TC_STRUCT * const blend_tc)
{
    if (!tc || !prev_tc) {
        return NO_BLEND;
    }

    switch  (prev_tc->term_cond)
    {
    case TC_TERM_COND_EXACT:
    case TC_TERM_COND_STOP:
    // Can't blend segments that are explicitly disallowed
        return NO_BLEND;
    case TC_TERM_COND_PARABOLIC:
    case TC_TERM_COND_TANGENT:
        break;
    }

    // Compute performance measures ("perf_xxx") for each method. This is
    // basically the blend velocity. However, because parabolic blends require
    // halving the acceleration of both blended segments, they in effect slow
    // down the next and previous blends as well. We model this loss by scaling
    // the blend velocity down to find an "equivalent" velocity.
    double perf_parabolic = estimateParabolicBlendPerformance(tp, prev_tc, tc) / 2.0;
    double perf_tangent = prev_tc->kink_vel;
    double perf_arc_blend = blend_tc ? blend_tc->maxvel : 0.0;

    // KLUDGE Order the performance measurements so that they match the enum values
    double perf[3] = {perf_parabolic, perf_tangent, perf_arc_blend};
    tc_blend_type_t best_blend = find_max_element(perf, 3);

    switch (best_blend) {
        case PARABOLIC_BLEND: // parabolic
            tcRemoveKinkProperties(prev_tc, tc);
            tcSetTermCond(prev_tc, tc, TC_TERM_COND_PARABOLIC);
            break;
        case TANGENT_SEGMENTS_BLEND: // tangent
            // NOTE: acceleration / velocity reduction is done dynamically in functions that access TC_STRUCT properties
            tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
            break;
        case ARC_BLEND: // arc blend
            tcRemoveKinkProperties(prev_tc, tc);
            break;
        case NO_BLEND:
            tcRemoveKinkProperties(prev_tc, tc);
            tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
            break;
    }

#ifdef TP_DEBUG
    print_json5_log_start(CheckBlendPerformance,Command);
    print_json5_double(perf_parabolic);
    print_json5_double(perf_tangent);
    print_json5_double(perf_arc_blend);
    print_json5_string_("best_blend", blendTypeAsString(best_blend));
    print_json5_log_end();
#endif

    return best_blend;
}


STATIC tp_err_t tpCreateLineArcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{
    tp_debug_print("-- Starting LineArc blend arc --\n");

    PmCartesian vel_bound = getXYZVelBounds();
    PmCartesian acc_bound = getXYZAccelBounds();
    
    //Populate blend geometry struct
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;

    int res_init = blendInit3FromLineArc(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            getMaxBlendFeedScale(prev_tc, tc));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    // Check for coplanarity based on binormal and tangents
    int coplanar = pmUnitCartsColinear(&geom.binormal,
            &tc->coords.circle.xyz.normal);

    if (!coplanar) {
        tp_debug_print("aborting arc, not coplanar\n");
        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);

    int res_points = blendFindPoints3(&points_approx, &geom, &param);
    
    int res_post = blendLineArcPostProcess(&points_exact,
            &points_approx,
            &param, 
            &geom, &prev_tc->coords.line.xyz,
            &tc->coords.circle.xyz);

    //Catch errors in blend setup
    if (res_init || res_param || res_points || res_post) {
        tp_debug_print("Got %d, %d, %d, %d for init, param, points, post, aborting arc\n",
                res_init,
                res_param,
                res_points,
                res_post);
        return TP_ERR_FAIL;
    }
    
    /* If blend calculations were successful, then we're ready to create the
     * blend arc.
     */

    if (points_exact.trim2 > param.phi2_max) {
        tp_debug_print("trim2 %f > phi2_max %f, aborting arc...\n",
                points_exact.trim2,
                param.phi2_max);
        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, emcmotConfig->arcBlendGapCycles);
    //Store working copies of geometry
    PmCartLine line1_temp = prev_tc->coords.line.xyz;
    PmCircle circ2_temp = tc->coords.circle.xyz;

    // Change lengths of circles
    double new_len1 = line1_temp.tmag - points_exact.trim1;
    int res_stretch1 = pmCartLineStretch(&line1_temp,
            new_len1,
            false);

    double phi2_new = tc->coords.circle.xyz.angle - points_exact.trim2;

    tp_debug_print("phi2_new = %f\n",phi2_new);
    int res_stretch2 = pmCircleStretch(&circ2_temp,
            phi2_new,
            true);
    //TODO create blends
    if (res_stretch1 || res_stretch2) {
        tp_debug_print("segment resize failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    //Get exact start and end points to account for spiral in arcs
    pmCartLinePoint(&line1_temp,
            line1_temp.tmag,
            &points_exact.arc_start);
    pmCirclePoint(&circ2_temp,
            0.0,
            &points_exact.arc_end);
    //TODO deal with large spiral values, or else detect and fall back?

    blendPoints3Print(&points_exact);
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz,
            &points_exact,
            &geom,
            &param);
    if (res_arc < 0) {
        tp_debug_print("arc creation failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = prev_tc->coords.line.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.line.uvw.end;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    int res_tangent = checkTangentAngle(&circ2_temp,
            &blend_tc->coords.arc.xyz,
            &geom,
            &param,
            tp->cycleTime,
            true);

    if (res_tangent < 0) {
        tp_debug_print("failed tangent check, aborting arc...\n");
        return TP_ERR_FAIL;
    }

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Passed all tests, updating segments\n");
    //TODO refactor to pass consume to connect function
    if (param.consume) {
        //Since we're consuming the previous segment, pop the last line off of the queue
        int res_pop = tcqPopBack(&tp->queue);
        if (res_pop) {
            tp_debug_print("failed to pop segment, aborting arc\n");
            return TP_ERR_FAIL;
        }
    } else {
        tcSetLineXYZ(prev_tc, &line1_temp);
    }
    tcSetCircleXYZ(tc, &circ2_temp);

    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);

    return TP_ERR_OK;
}


STATIC tp_err_t tpCreateArcLineBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    tp_debug_print("-- Starting ArcLine blend arc --\n");
    PmCartesian vel_bound = getXYZVelBounds();
    PmCartesian acc_bound = getXYZAccelBounds();

    //Populate blend geometry struct
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;
    param.consume = 0;

    int res_init = blendInit3FromArcLine(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            getMaxBlendFeedScale(prev_tc, tc));
    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    // Check for coplanarity based on binormal
    int coplanar = pmUnitCartsColinear(&geom.binormal,
            &prev_tc->coords.circle.xyz.normal);

    if (!coplanar) {
        tp_debug_print("aborting arc, not coplanar\n");
        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);

    int res_points = blendFindPoints3(&points_approx, &geom, &param);
    
    int res_post = blendArcLinePostProcess(&points_exact,
            &points_approx,
            &param, 
            &geom, &prev_tc->coords.circle.xyz,
            &tc->coords.line.xyz);

    //Catch errors in blend setup
    if (res_init || res_param || res_points || res_post) {
        tp_debug_print("Got %d, %d, %d, %d for init, param, points, post\n",
                res_init,
                res_param,
                res_points,
                res_post);
        return TP_ERR_FAIL;
    }
    
    blendCheckConsume(&param, &points_exact, prev_tc, emcmotConfig->arcBlendGapCycles);

    /* If blend calculations were successful, then we're ready to create the
     * blend arc.
     */

    // Store working copies of geometry
    PmCircle circ1_temp = prev_tc->coords.circle.xyz;
    PmCartLine line2_temp = tc->coords.line.xyz;

    // Update start and end points of segment copies
    double phi1_new = circ1_temp.angle - points_exact.trim1;

    if (points_exact.trim1 > param.phi1_max) {
        tp_debug_print("trim1 %f > phi1_max %f, aborting arc...\n",
                points_exact.trim1,
                param.phi1_max);
        return TP_ERR_FAIL;
    }

    int res_stretch1 = pmCircleStretch(&circ1_temp,
            phi1_new,
            false);
    if (res_stretch1 != TP_ERR_OK) {
        return TP_ERR_FAIL;
    }

    double new_len2 = tc->target - points_exact.trim2;
    int res_stretch2 = pmCartLineStretch(&line2_temp,
            new_len2,
            true);

    if (res_stretch1 || res_stretch2) {
        tp_debug_print("segment resize failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    pmCirclePoint(&circ1_temp,
            circ1_temp.angle,
            &points_exact.arc_start);

    pmCartLinePoint(&line2_temp,
            0.0,
            &points_exact.arc_end);

    blendPoints3Print(&points_exact);

    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points_exact, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = tc->coords.line.abc.start;
    blend_tc->coords.arc.uvw = tc->coords.line.uvw.start;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    int res_tangent = checkTangentAngle(&circ1_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, false);
    if (res_tangent) {
        tp_debug_print("failed tangent check, aborting arc...\n");
        return TP_ERR_FAIL;
    }

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Passed all tests, updating segments\n");

    tcSetCircleXYZ(prev_tc, &circ1_temp);
    tcSetLineXYZ(tc, &line2_temp);

    //Cleanup any mess from parabolic
    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
    return TP_ERR_OK;
}

STATIC tp_err_t tpCreateArcArcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    tp_debug_print("-- Starting ArcArc blend arc --\n");
    //TODO type checks
    int colinear = pmUnitCartsColinear(&prev_tc->coords.circle.xyz.normal,
            &tc->coords.circle.xyz.normal);
    if (!colinear) {
        // Fail out if not collinear
        tp_debug_print("arc abort: not coplanar\n");
        return TP_ERR_FAIL;
    }

    PmCartesian vel_bound = getXYZVelBounds();
    PmCartesian acc_bound = getXYZAccelBounds();

    //Populate blend geometry struct
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;

    int res_init = blendInit3FromArcArc(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            getMaxBlendFeedScale(prev_tc, tc));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    int coplanar1 = pmUnitCartsColinear(&geom.binormal,
            &prev_tc->coords.circle.xyz.normal);

    if (!coplanar1) {
        tp_debug_print("aborting blend arc, arc id %d is not coplanar with binormal\n", prev_tc->id);
        return TP_ERR_FAIL;
    }

    int coplanar2 = pmUnitCartsColinear(&geom.binormal,
            &tc->coords.circle.xyz.normal);
    if (!coplanar2) {
        tp_debug_print("aborting blend arc, arc id %d is not coplanar with binormal\n", tc->id);
        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);
    int res_points = blendFindPoints3(&points_approx, &geom, &param);
    
    int res_post = blendArcArcPostProcess(&points_exact,
            &points_approx,
            &param, 
            &geom, &prev_tc->coords.circle.xyz,
            &tc->coords.circle.xyz);

    //Catch errors in blend setup
    if (res_init || res_param || res_points || res_post) {
        tp_debug_print("Got %d, %d, %d, %d for init, param, points, post\n",
                res_init,
                res_param,
                res_points,
                res_post);

        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, emcmotConfig->arcBlendGapCycles);
    
    /* If blend calculations were successful, then we're ready to create the
     * blend arc. Begin work on temp copies of each circle here:
     */

    double phi1_new = prev_tc->coords.circle.xyz.angle - points_exact.trim1;
    double phi2_new = tc->coords.circle.xyz.angle - points_exact.trim2;

    // TODO pare down this debug output
    tp_debug_print("phi1_new = %f, trim1 = %f\n", phi1_new, points_exact.trim1);
    tp_debug_print("phi2_new = %f, trim2 = %f\n", phi2_new, points_exact.trim2);

    if (points_exact.trim1 > param.phi1_max) {
        tp_debug_print("trim1 %f > phi1_max %f, aborting arc...\n",
                points_exact.trim1,
                param.phi1_max);
        return TP_ERR_FAIL;
    }

    if (points_exact.trim2 > param.phi2_max) {
        tp_debug_print("trim2 %f > phi2_max %f, aborting arc...\n",
                points_exact.trim2,
                param.phi2_max);
        return TP_ERR_FAIL;
    }

    //Store working copies of geometry
    PmCircle circ1_temp = prev_tc->coords.circle.xyz;
    PmCircle circ2_temp = tc->coords.circle.xyz;

    int res_stretch1 = pmCircleStretch(&circ1_temp,
            phi1_new,
            false);
    if (res_stretch1 != TP_ERR_OK) {
        return TP_ERR_FAIL;
    }

    int res_stretch2 = pmCircleStretch(&circ2_temp,
            phi2_new,
            true);
    if (res_stretch1 || res_stretch2) {
        tp_debug_print("segment resize failed, aborting arc\n");
        return TP_ERR_FAIL;
    }

    //Get exact start and end points to account for spiral in arcs
    pmCirclePoint(&circ1_temp,
            circ1_temp.angle,
            &points_exact.arc_start);
    pmCirclePoint(&circ2_temp,
            0.0,
            &points_exact.arc_end);

    tp_debug_print("Modified arc points\n");
    blendPoints3Print(&points_exact);
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points_exact, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = prev_tc->coords.circle.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.circle.uvw.end;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    int res_tangent1 = checkTangentAngle(&circ1_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, false);
    int res_tangent2 = checkTangentAngle(&circ2_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, true);
    if (res_tangent1 || res_tangent2) {
        tp_debug_print("failed tangent check, aborting arc...\n");
        return TP_ERR_FAIL;
    }

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    tp_debug_print("Passed all tests, updating segments\n");

    tcSetCircleXYZ(prev_tc, &circ1_temp);
    tcSetCircleXYZ(tc, &circ2_temp);

    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
    return TP_ERR_OK;
}


STATIC tp_err_t tpCreateLineLineBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc,
        TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    tp_debug_print("-- Starting LineLine blend arc --\n");

    PmCartesian vel_bound = getXYZVelBounds();
    PmCartesian acc_bound = getXYZAccelBounds();

    // Setup blend data structures
    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points;

    int res_init = blendInit3FromLineLine(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            getMaxBlendFeedScale(prev_tc, tc));

    if (res_init != TP_ERR_OK) {
        tp_debug_print("blend init failed with code %d, aborting blend arc\n",
                res_init);
        return res_init;
    }

    int res_blend = blendComputeParameters(&param);
    if (res_blend != TP_ERR_OK) {
        return res_blend;
    }

    blendFindPoints3(&points, &geom, &param);

    blendCheckConsume(&param, &points, prev_tc, emcmotConfig->arcBlendGapCycles);

    // Set up actual blend arc here
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    // Note that previous restrictions don't allow ABC or UVW movement, so the
    // end and start points should be identical
    blend_tc->coords.arc.abc = prev_tc->coords.line.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.line.uvw.end;

    //set the max velocity to v_plan, since we'll violate constraints otherwise.
    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    tp_debug_print("blend_tc target_vel = %g\n", blend_tc->target_vel);

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    int retval = TP_ERR_FAIL;

    //TODO refactor to pass consume to connect function
    if (param.consume) {
        //Since we're consuming the previous segment, pop the last line off of the queue
        retval = tcqPopBack(&tp->queue);
        if (retval) {
            //This is unrecoverable since we've already changed the line. Something is wrong if we get here...
            rtapi_print_msg(RTAPI_MSG_ERR, "PopBack failed\n");
            return TP_ERR_FAIL;
        }
        //Since the blend arc meets the end of the previous line, we only need
        //to "connect" to the next line
        retval = tcConnectBlendArc(NULL, tc, &points.arc_start, &points.arc_end);
    } else {
        //TODO refactor connect function to stretch lines and check for bad stretching
        tp_debug_print("keeping previous line\n");
        retval = tcConnectBlendArc(prev_tc, tc, &points.arc_start, &points.arc_end);
    }
    return retval;
}


/**
 * Add a newly created motion segment to the tp queue.
 * Returns an error code if the queue operation fails, otherwise adds a new
 * segment to the queue and updates the end point of the trajectory planner.
 */
STATIC inline int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc, int inc_id) {

    tc->id = tp->nextId;
    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return TP_ERR_FAIL;
    }
    if (inc_id) {
        tp->nextId++;
    }

    // Store end of current move as new final goal of TP
    // KLUDGE: endpoint is garbage for rigid tap since it's supposed to retract past the start point.
    if (tc->motion_type != TC_RIGIDTAP) {
        tcGetEndpoint(tc, &tp->goalPos);
    }
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    //Fixing issue with duplicate id's?
#ifdef TP_DEBUG
    print_json5_log_start(Enqueue, Command);
    print_json5_long_("id", tc->id);
    print_json5_string_("motion_type", tcMotionTypeAsString(tc->motion_type));
    print_json5_double_("target", tc->target);
    print_json5_string_("term_cond", tcTermCondAsString(tc->term_cond));
    print_json5_end_();
#endif

    return TP_ERR_OK;
}

int handlePrevTermCondition(TC_STRUCT *prev_tc, TC_STRUCT *tc)
{
    if (!tc) {
        return TP_ERR_FAIL;
    }

    switch (tc->motion_type) {
    case TC_RIGIDTAP:
        tcSetTermCond(tc, NULL, TC_TERM_COND_EXACT);
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
        break;
    case TC_LINEAR:
    case TC_CIRCULAR:
    case TC_SPHERICAL:
    {
        tc_term_cond_t prev_term = prev_tc ? prev_tc->term_cond : TC_TERM_COND_STOP;
        tcSetTermCond(prev_tc, tc, prev_term);
    }
        break;
    }
    return TP_ERR_OK;
}

STATIC int handleModeChange(TC_STRUCT *prev_tc, TC_STRUCT *tc)
{
    if (!tc || !prev_tc) {
        return TP_ERR_FAIL;
    }

    if ((prev_tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE) ^
            (tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE)) {
        tp_debug_print("Blending disabled: can't blend between rapid and feed motions\n");
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
    }
    if (prev_tc->synchronized != TC_SYNC_POSITION &&
            tc->synchronized == TC_SYNC_POSITION) {
        tp_debug_print("Blending disabled: entering position-sync mode\n");
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
    }
    return TP_ERR_OK;
}

STATIC int tpSetupSyncedIO(TP_STRUCT * const tp, TC_STRUCT * const tc) {
    if (tp->syncdio.anychanged != 0) {
        tc->syncdio = tp->syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(tp); // clear out the list, in order to prepare for the next time we need to use it
        return TP_ERR_OK;
    } else {
        tc->syncdio.anychanged = 0;
        return TP_ERR_NO_ACTION;
    }


}

STATIC int tcUpdateArcLengthFit(TC_STRUCT * const tc)
{
    if (!tc) {return -1;}

    switch (tc->motion_type) {
    case TC_CIRCULAR:
        findSpiralArcLengthFit(&tc->coords.circle.xyz, &tc->coords.circle.fit);
        break;
    case TC_LINEAR:
    case TC_RIGIDTAP:
    case TC_SPHERICAL:
        // No update necessary
        break;
    }
    return 0;
}

STATIC int tpFinalizeAndEnqueue(TP_STRUCT * const tp, TC_STRUCT * const tc)
{
    //TODO refactor this into its own function
    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    // Make sure the blending flags are consistent w/ previous segment
    handlePrevTermCondition(prev_tc, tc);

    // Prevent blends for specific mode changes (where blending isn't possible anyway)
    handleModeChange(prev_tc, tc);

    if (emcmotConfig->arcBlendEnable){
        tpHandleBlendArc(tp, tc);
        tcUpdateArcLengthFit(tc);
    }
    tcFlagEarlyStop(prev_tc, tc);
    // KLUDGE order is important here, the parabolic blend check has to
    // happen after all other steps that affect the terminal condition
    tcFinalizeLength(prev_tc);

    int retval = tpAddSegmentToQueue(tp, tc, true);
    //Run speed optimization (will abort safely if there are no tangent segments)
    tpRunOptimization(tp);

    return retval;
}


/**
 * Adds a rigid tap cycle to the motion queue.
 */
int tpAddRigidTap(TP_STRUCT * const tp,
        EmcPose end,
        double vel,
        double ini_maxvel,
        double acc,
        unsigned char enables) {
    if (tpErrorCheck(tp)) {
        return TP_ERR_FAIL;
    }

    tp_info_print("== AddRigidTap ==\n");

    if(!tp->synchronized) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Cannot add unsynchronized rigid tap move.\n");
        return TP_ERR_FAIL;
    }

    TC_STRUCT tc = {0};

    /* Initialize rigid tap move.
     * NOTE: rigid tapping does not have a canonical type.
     * NOTE: always need atspeed since this is a synchronized movement.
     * */
    tcInit(&tc,
            TC_RIGIDTAP,
            0,
            tp->cycleTime,
            enables,
            1);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc);

    // Setup rigid tap geometry
    pmRigidTapInit(&tc.coords.rigidtap,
            &tp->goalPos,
            &end);
    tc.target = pmRigidTapTarget(&tc.coords.rigidtap, tp->uu_per_rev);

    return tpFinalizeAndEnqueue(tp, &tc);
}

STATIC blend_type_t tpCheckBlendArcType(
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc) {

    if (!prev_tc || !tc) {
        tp_debug_print("prev_tc or tc doesn't exist\n");
        return BLEND_NONE;
    }

    //If exact stop, we don't compute the arc
    if (prev_tc->term_cond != TC_TERM_COND_PARABOLIC) {
        tp_debug_print("Wrong term cond = %u\n", prev_tc->term_cond);
        return BLEND_NONE;
    }

    //If we have any rotary axis motion, then don't create a blend arc
    if (tcRotaryMotionCheck(tc) || tcRotaryMotionCheck(prev_tc)) {
        tp_debug_print("One of the segments has rotary motion, aborting blend arc\n");
        return BLEND_NONE;
    }

    if (tc->finalized || prev_tc->finalized) {
        tp_debug_print("Can't create blend when segment lengths are finalized\n");
        return BLEND_NONE;
    }

    tp_debug_print("Motion types: prev_tc = %u, tc = %u\n",
            prev_tc->motion_type,tc->motion_type);
    //If not linear blends, we can't easily compute an arc
    if ((prev_tc->motion_type == TC_LINEAR) && (tc->motion_type == TC_LINEAR)) {
        return BLEND_LINE_LINE;
    } else if (prev_tc->motion_type == TC_LINEAR && tc->motion_type == TC_CIRCULAR) {
        return BLEND_LINE_ARC;
    } else if (prev_tc->motion_type == TC_CIRCULAR && tc->motion_type == TC_LINEAR) {
        return BLEND_ARC_LINE;
    } else if (prev_tc->motion_type == TC_CIRCULAR && tc->motion_type == TC_CIRCULAR) {
        return BLEND_ARC_ARC;
    } else {
        return BLEND_NONE;
    }
}


/**
 * Based on the nth and (n-1)th segment, find a safe final velocity for the (n-1)th segment.
 * This function also caps the target velocity if velocity ramping is enabled. If we
 * don't do this, then the linear segments (with higher tangential
 * acceleration) will speed up and slow down to reach their target velocity,
 * creating "humps" in the velocity profile.
 */
STATIC int tpComputeOptimalVelocity(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT * const prev1_tc) {
    //Calculate the maximum starting velocity vs_back of segment tc, given the
    //trajectory parameters
    double acc_this = tcGetTangentialMaxAccel(tc);

    // Find the reachable velocity of tc, moving backwards in time
    double vs_back = pmSqrt(pmSq(tc->finalvel) + 2.0 * acc_this * tc->target);
    // Find the reachable velocity of prev1_tc, moving forwards in time

    double vf_limit_this = tc->maxvel;
    double vf_limit_prev = prev1_tc->maxvel;
    if (prev1_tc->kink_vel >=0  && prev1_tc->term_cond == TC_TERM_COND_TANGENT) {
        // Only care about kink_vel with tangent segments
        vf_limit_prev = fmin(vf_limit_prev, prev1_tc->kink_vel);
    }
    //Limit the PREVIOUS velocity by how much we can overshoot into
    double vf_limit = fmin(vf_limit_this, vf_limit_prev);

    if (vs_back >= vf_limit ) {
        //If we've hit the requested velocity, then prev_tc is definitely a "peak"
        vs_back = vf_limit;
        prev1_tc->optimization_state = TC_OPTIM_AT_MAX;
        tp_debug_print("found peak due to v_limit %f\n", vf_limit);
    }

    //Limit tc's target velocity to avoid creating "humps" in the velocity profile
    prev1_tc->finalvel = vs_back;

    //Reduce max velocity to match sample rate
    double sample_maxvel = tc->target / (tp->cycleTime * TP_MIN_SEGMENT_CYCLES);
    tc->maxvel = fmin(tc->maxvel, sample_maxvel);

    tp_info_print(" prev1_tc-> fv = %f, tc->fv = %f\n",
            prev1_tc->finalvel, tc->finalvel);

    return TP_ERR_OK;
}


/**
 * Do "rising tide" optimization to find allowable final velocities for each queued segment.
 * Walk along the queue from the back to the front. Based on the "current"
 * segment's final velocity, calculate the previous segment's maximum allowable
 * final velocity. The depth we walk along the queue is controlled by the
 * TP_LOOKAHEAD_DEPTH constant for now. The process safetly aborts early due to
 * a short queue or other conflicts.
 */
STATIC int tpRunOptimization(TP_STRUCT * const tp) {
    // Pointers to the "current", previous, and 2nd previous trajectory
    // components. Current in this context means the segment being optimized,
    // NOT the currently excecuting segment.

    TC_STRUCT *tc;
    TC_STRUCT *prev1_tc;

    int ind, x;
    int len = tcqLen(&tp->queue);
    //TODO make lookahead depth configurable from the INI file

    int hit_peaks = 0;
    // Flag that says we've hit at least 1 non-tangent segment
    bool hit_non_tangent = false;

    /* Starting at the 2nd to last element in the queue, work backwards towards
     * the front. We can't do anything with the very last element because its
     * length may change if a new line is added to the queue.*/

    for (x = 1; x < emcmotConfig->arcBlendOptDepth + 2; ++x) {
        tp_info_print("==== Optimization step %d ====\n",x);

        // Update the pointers to the trajectory segments in use
        ind = len-x;
        tc = tcqItem(&tp->queue, ind);
        prev1_tc = tcqItem(&tp->queue, ind-1);

        if ( !prev1_tc || !tc) {
            tp_debug_print(" Reached end of queue in optimization\n");
            return TP_ERR_OK;
        }

        // stop optimizing if we hit a non-tangent segment (final velocity
        // stays zero)
        if (prev1_tc->term_cond != TC_TERM_COND_TANGENT) {
            if (hit_non_tangent) {
                // 2 or more non-tangent segments means we're past where the optimizer can help
                tp_debug_print("Found 2nd non-tangent segment, stopping optimization\n");
                return TP_ERR_OK;
            } else  {
                tp_debug_print("Found first non-tangent segment, contining\n");
                hit_non_tangent = true;
                continue;
            }
        }

        double progress_ratio = prev1_tc->progress / prev1_tc->target;
        // can safely decelerate to halfway point of segment from 25% of segment
        double cutoff_ratio = BLEND_DIST_FRACTION / 2.0;

        if (progress_ratio >= cutoff_ratio) {
            tp_debug_print("segment %d has moved past %f percent progress, cannot blend safely!\n",
                    ind-1, cutoff_ratio * 100.0);
            return TP_ERR_OK;
        }

        //Somewhat pedantic check for other conditions that would make blending unsafe
        if (prev1_tc->splitting || prev1_tc->blending_next) {
            tp_debug_print("segment %d is already blending, cannot optimize safely!\n",
                    ind-1);
            return TP_ERR_OK;
        }

        tp_info_print("  current term = %u, type = %u, id = %u, accel_mode = %d\n",
                tc->term_cond, tc->motion_type, tc->id, tc->accel_mode);
        tp_info_print("  prev term = %u, type = %u, id = %u, accel_mode = %d\n",
                prev1_tc->term_cond, prev1_tc->motion_type, prev1_tc->id, prev1_tc->accel_mode);

        if (tc->atspeed) {
            //Assume worst case that we have a stop at this point. This may cause a
            //slight hiccup, but the alternative is a sudden hard stop.
            tp_debug_print("Found atspeed at id %d\n",tc->id);
            tc->finalvel = 0.0;
        }

        if (!tc->finalized) {
            tp_debug_print("Segment %d, type %d not finalized, continuing\n",tc->id,tc->motion_type);
            // use worst-case final velocity that allows for up to 1/2 of a segment to be consumed.

            prev1_tc->finalvel = fmin(prev1_tc->maxvel, tpCalculateOptimizationInitialVel(tp,tc));

            // Fixes acceleration violations when last segment is not finalized, and previous segment is tangent.
            if (prev1_tc->kink_vel >=0  && prev1_tc->term_cond == TC_TERM_COND_TANGENT) {
              prev1_tc->finalvel = fmin(prev1_tc->finalvel, prev1_tc->kink_vel);
            }
            tc->finalvel = 0.0;
        } else {
            tpComputeOptimalVelocity(tp, tc, prev1_tc);
        }

        tc->active_depth = x - 2 - hit_peaks;
#ifdef TP_OPTIMIZATION_LAZY
        if (tc->optimization_state == TC_OPTIM_AT_MAX) {
            hit_peaks++;
        }
        if (hit_peaks > TP_OPTIMIZATION_CUTOFF) {
            return TP_ERR_OK;
        }
#endif

    }
    tp_debug_print("Reached optimization depth limit\n");
    return TP_ERR_OK;
}


/**
 * Check for tangency between the current segment and previous segment.
 * If the current and previous segment are tangent, then flag the previous
 * segment as tangent, and limit the current segment's velocity by the sampling
 * rate.
 */
STATIC int tpSetupTangent(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc, TC_STRUCT * const tc) {
    if (!tc || !prev_tc) {
        tp_debug_print("missing tc or prev tc in tangent check\n");
        return TP_ERR_FAIL;
    }
    //If we have ABCUVW movement, then don't check for tangency
    if (tcRotaryMotionCheck(tc) || tcRotaryMotionCheck(prev_tc)) {
        tp_debug_print("found rotary axis motion\n");
        return TP_ERR_FAIL;
    }

    if (emcmotConfig->arcBlendOptDepth < 2) {
        tp_debug_print("Optimization depth %d too low for tangent optimization\n",
                emcmotConfig->arcBlendOptDepth);
        return TP_ERR_FAIL;
    }

    if (prev_tc->term_cond == TC_TERM_COND_STOP) {
        tp_debug_print("Found exact stop condition\n");
        return TP_ERR_FAIL;
    }

    PmCartesian prev_tan, this_tan;

    int res_endtan = tcGetEndTangentUnitVector(prev_tc, &prev_tan);
    int res_starttan = tcGetStartTangentUnitVector(tc, &this_tan);
    if (res_endtan || res_starttan) {
        tp_debug_print("Got %d and %d from tangent vector calc\n",
                res_endtan, res_starttan);
    }

    tp_debug_print("prev tangent vector: %f %f %f\n", prev_tan.x, prev_tan.y, prev_tan.z);
    tp_debug_print("this tangent vector: %f %f %f\n", this_tan.x, this_tan.y, this_tan.z);

    // Assume small angle approximation here
    const double SHARP_CORNER_DEG = 2.0;
    const double SHARP_CORNER_EPSILON = pmSq(PM_PI * ( SHARP_CORNER_DEG / 180.0));
    if (pmCartCartAntiParallel(&prev_tan, &this_tan, SHARP_CORNER_EPSILON))
    {
        tp_debug_print("Found sharp corner\n");
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
        return TP_ERR_FAIL;
    }

    // Calculate instantaneous acceleration required for change in direction
    // from v1 to v2, assuming constant speed
    double v_max1 = fmin(prev_tc->maxvel, prev_tc->reqvel * getMaxFeedScale(prev_tc));
    double v_max2 = fmin(tc->maxvel, tc->reqvel * getMaxFeedScale(tc));
    double v_max = fmin(v_max1, v_max2);
    tp_debug_print("tangent v_max = %f\n",v_max);

    // Account for acceleration past final velocity during a split cycle
    // (e.g. next segment starts accelerating again so the average velocity is higher at the end of the split cycle)
    double a_inst = v_max / tp->cycleTime + tc->maxaccel;
    // Set up worst-case final velocity
    // Compute the actual magnitude of acceleration required given the tangent directions
    // Do this by assuming that we decelerate to a stop on the previous segment,
    // and simultaneously accelerate up to the maximum speed on the next one.
    PmCartesian acc1, acc2, acc_diff;
    pmCartScalMult(&prev_tan, a_inst, &acc1);
    pmCartScalMult(&this_tan, a_inst, &acc2);
    pmCartCartSub(&acc2,&acc1,&acc_diff);

    PmCartesian acc_bound = getXYZAccelBounds();

    PmCartesian acc_scale;
    findAccelScale(&acc_diff,&acc_bound,&acc_scale);
    tp_debug_print("acc_diff: %f %f %f\n",
            acc_diff.x,
            acc_diff.y,
            acc_diff.z);
    tp_debug_print("acc_scale: %f %f %f\n",
            acc_scale.x,
            acc_scale.y,
            acc_scale.z);

    //FIXME this ratio is arbitrary, should be more easily tunable
    double acc_scale_max = pmCartAbsMax(&acc_scale);
    //KLUDGE lumping a few calculations together here
    if (prev_tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_CIRCULAR) {
        acc_scale_max /= BLEND_ACC_RATIO_TANGENTIAL;
    }

    // Controls the tradeoff between reduction of final velocity, and reduction of allowed segment acceleration
    // TODO: this should ideally depend on some function of segment length and acceleration for better optimization
    const double kink_ratio = tpGetTangentKinkRatio();

    if (acc_scale_max < kink_ratio) {
        tp_debug_print(" Kink acceleration within %g, using tangent blend\n", kink_ratio);
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
        tcSetKinkProperties(prev_tc, tc, v_max, acc_scale_max);
        return TP_ERR_OK;
    } else {
        tcSetKinkProperties(prev_tc, tc, v_max * kink_ratio / acc_scale_max, kink_ratio);
        tp_debug_print("Kink acceleration scale %f above %f, kink vel = %f, blend arc may be faster\n",
                acc_scale_max,
                kink_ratio,
                prev_tc->kink_vel);
        // NOTE: acceleration will be reduced later if tangent blend is used
        return TP_ERR_NO_ACTION;
    }
}

static bool tpCreateBlendIfPossible(
        TP_STRUCT *tp,
        TC_STRUCT *prev_tc,
        TC_STRUCT *tc,
        TC_STRUCT *blend_tc)
{
    tp_err_t res_create = TP_ERR_FAIL;
    blend_type_t blend_requested = tpCheckBlendArcType(prev_tc, tc);

    switch (blend_requested) {
        case BLEND_LINE_LINE:
            res_create = tpCreateLineLineBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_LINE_ARC:
            res_create = tpCreateLineArcBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_ARC_LINE:
            res_create = tpCreateArcLineBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_ARC_ARC:
            res_create = tpCreateArcArcBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_NONE:
            break;
    }

    return res_create == TP_ERR_OK;
}


/**
 * Handle creating a blend arc when a new line segment is about to enter the queue.
 * This function handles the checks, setup, and calculations for creating a new
 * blend arc. Essentially all of the blend arc functions are called through
 * here to isolate the process.
 */
STATIC tc_blend_type_t tpHandleBlendArc(TP_STRUCT * const tp, TC_STRUCT * const tc) {

    tp_debug_print("*****************************************\n** Handle Blend Arc **\n");

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    //If the previous segment has already started, then don't create a blend
    //arc for the next pair.
    // TODO May be able to lift this restriction if we can ensure that we leave
    // 1 timestep's worth of distance in prev_tc
    if ( !prev_tc) {
        tp_debug_print(" queue empty\n");
        return NO_BLEND;
    }
    if (prev_tc->progress > prev_tc->target / 2.0) {
        tp_debug_print(" prev_tc progress (%f) is too large, aborting blend arc\n", prev_tc->progress);
        return NO_BLEND;
    }

    // Check for tangency between segments and handle any errors
    // TODO possibly refactor this into a macro?
    int res_tan = tpSetupTangent(tp, prev_tc, tc);
    switch (res_tan) {
        // Abort blend arc creation in these cases
        case TP_ERR_FAIL:
            tp_debug_print(" tpSetupTangent failed, aborting blend arc\n");
        case TP_ERR_OK:
            return res_tan;
        case TP_ERR_NO_ACTION:
        default:
            //Continue with creation
            break;
    }

    TC_STRUCT blend_tc = {0};

    tc_blend_type_t blend_used = NO_BLEND;

    bool arc_blend_ok = tpCreateBlendIfPossible(tp, prev_tc, tc, &blend_tc);

    if (arc_blend_ok) {
        //Need to do this here since the length changed
        blend_used = ARC_BLEND;
        tpAddSegmentToQueue(tp, &blend_tc, false);
    } else {
        // If blend arc creation failed early on, catch it here and find the best blend
        blend_used = tpChooseBestBlend(tp, prev_tc, tc, NULL) ;
    }

    return blend_used;
}

//TODO final setup steps as separate functions
//
/**
 * Add a straight line to the tc queue.
 * end of the previous move to the new end specified here at the
 * currently-active accel and vel settings from the tp struct.
 */
int tpAddLine(TP_STRUCT * const tp,
        EmcPose end,
        int canon_motion_type,
        double vel,
        double ini_maxvel,
        double acc,
        unsigned char enables,
        char atspeed,
        int indexrotary,
        int pure_angular)
{

    if (tpErrorCheck(tp) < 0) {
        return TP_ERR_FAIL;
    }
#ifdef TP_DEBUG
    {
        // macros use the variable name, need a plain name to please the JSON5 parser
        print_json5_log_start(tpAddLine, Command);
        print_json5_long_("id", tp->nextId);
        print_json5_EmcPose_("goal", tp->goalPos);
        print_json5_EmcPose(end);
        print_json5_double(vel);
        print_json5_double(ini_maxvel);
        print_json5_double(acc);
        print_json5_unsigned(enables);
        print_json5_long(indexrotary);
        print_json5_long(atspeed);
        print_json5_long(canon_motion_type);
        EmcPose delta = tp->goalPos;
        emcPoseSub(&end, &tp->goalPos, &delta);
        print_json5_EmcPose(delta);
        print_json5_log_end();
    }
#endif

    // Initialize new tc struct for the line segment
    TC_STRUCT tc = {0};
    tcInit(&tc,
            TC_LINEAR,
            canon_motion_type,
            tp->cycleTime,
            enables,
            atspeed);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);
    tcSetTermCond(&tc, NULL, tp->termCond);

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc);
    // Setup line geometry
    pmLine9Init(&tc.coords.line,
            &tp->goalPos,
            &end);
    tc.target = pmLine9Target(&tc.coords.line, pure_angular);
    if (tc.target < TP_POS_EPSILON) {
        rtapi_print_msg(RTAPI_MSG_DBG,"failed to create line id %d, zero-length segment\n",tp->nextId);
        return TP_ERR_ZERO_LENGTH;
    }
    tc.nominal_length = tc.target;
    tcClampVelocityByLength(&tc);

    // For linear move, set rotary axis settings 
    tc.indexrotary = indexrotary;

    return tpFinalizeAndEnqueue(tp, &tc);
}


/**
 * Adds a circular (circle, arc, helix) move from the end of the
 * last move to this new position.
 *
 * @param end is the xyz/abc point of the destination.
 *
 * see pmCircleInit for further details on how arcs are specified. Note that
 * degenerate arcs/circles are not allowed. We are guaranteed to have a move in
 * xyz so the target is always the circle/arc/helical length.
 */
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
        char atspeed)
{
    if (tpErrorCheck(tp)<0) {
        return TP_ERR_FAIL;
    }

#ifdef TP_DEBUG
    {
        // macros use the variable name, need a plain name to please the JSON5 parser
        long nextId = tp->nextId;
        EmcPose goal = tp->goalPos;
        print_json5_log_start(tpAddCircle, Command);
        print_json5_long(nextId);
        print_json5_EmcPose(goal);
        print_json5_PmCartesian(center);
        print_json5_PmCartesian(normal);
        print_json5_long(turn);
        print_json5_double(vel);
        print_json5_double(ini_maxvel);
        print_json5_double(acc);
        print_json5_double(acc_normal);
        print_json5_unsigned(enables);
        print_json5_long(atspeed);
        print_json5_long(canon_motion_type);
        EmcPose delta = tp->goalPos;
        emcPoseSub(&end, &tp->goalPos, &delta);
        print_json5_EmcPose(delta);
        print_json5_log_end();
    }
#endif

    TC_STRUCT tc = {0};

    tcInit(&tc,
            TC_CIRCULAR,
            canon_motion_type,
            tp->cycleTime,
            enables,
            atspeed);
    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);
    tcSetTermCond(&tc, NULL, tp->termCond);

    // Setup circle geometry
    int res_init = pmCircle9Init(&tc.coords.circle,
            &tp->goalPos,
            &end,
            &center,
            &normal,
            turn);

    if (res_init) return res_init;

    tc.acc_normal_max = acc_normal;

    // Update tc target with existing circular segment
    tc.target = pmCircle9Target(&tc.coords.circle);
    if (tc.target < TP_POS_EPSILON) {
        return TP_ERR_ZERO_LENGTH;
    }
    tc.nominal_length = tc.target;

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc);

    //Reduce max velocity to match sample rate
    tcClampVelocityByLength(&tc);

    return tpFinalizeAndEnqueue(tp, &tc);
}


/**
 * Adjusts blend velocity and acceleration to safe limits.
 * If we are blending between tc and nexttc, then we need to figure out what a
 * safe blend velocity is based on the known trajectory parameters. This
 * function updates the TC_STRUCT data with a safe blend velocity.
 *
 * @note This function will compute the parabolic blend start / end velocities
 * regardless of the current terminal condition (useful for planning).
 */
STATIC int tpComputeBlendVelocity(
        TC_STRUCT const *tc,
        TC_STRUCT const *nexttc,
        double target_vel_this,
        double target_vel_next,
        double *v_blend_this,
        double *v_blend_next,
        double *v_blend_net)
{
    /* Pre-checks for valid pointers */
    if (!nexttc || !tc || !v_blend_this || !v_blend_next ) {
        return TP_ERR_FAIL;
    }

    double acc_this = tcGetTangentialMaxAccel(tc);
    double acc_next = tcGetTangentialMaxAccel(nexttc);

    double v_reachable_this = fmin(tpCalculateTriangleVel(tc), target_vel_this);
    double v_reachable_next = fmin(tpCalculateTriangleVel(nexttc), target_vel_next);

    /* Compute the maximum allowed blend time for each segment.
     * This corresponds to the minimum acceleration that will just barely reach
     * max velocity as we are 1/2 done the segment.
     */

    double t_max_this = tc->target / v_reachable_this;
    double t_max_next = nexttc->target / v_reachable_next;
    double t_max_reachable = fmin(t_max_this, t_max_next);

    // How long the blend phase would be at maximum acceleration
    double t_min_blend_this = v_reachable_this / acc_this;
    double t_min_blend_next = v_reachable_next / acc_next;

    double t_max_blend = fmax(t_min_blend_this, t_min_blend_next);
    // The longest blend time we can get that's still within the 1/2 segment restriction
    double t_blend = fmin(t_max_reachable, t_max_blend);

    // Now, use this blend time to find the best acceleration / velocity for each segment
    *v_blend_this = fmin(v_reachable_this, t_blend * acc_this);
    *v_blend_next = fmin(v_reachable_next, t_blend * acc_next);

    double theta;

    PmCartesian v1={0}, v2={0};

    int res1 = tcGetEndAccelUnitVector(tc, &v1);
    int res2 = tcGetStartAccelUnitVector(nexttc, &v2);

    if (res1 || res2) {
        rtapi_print_msg(RTAPI_MSG_WARN, "Got bad unit vectors in parabolic blend velocity calculation\n");
        return TP_ERR_FAIL;
    }
    findIntersectionAngle(&v1, &v2, &theta);

    double cos_theta = cos(theta);

    if (tc->tolerance > 0) {
        /* see diagram blend.fig.  T (blend tolerance) is given, theta
         * is calculated from dot(s1, s2)
         *
         * blend criteria: we are decelerating at the end of segment s1
         * and we pass distance d from the end.
         * find the corresponding velocity v when passing d.
         *
         * in the drawing note d = 2T/cos(theta)
         *
         * when v1 is decelerating at a to stop, v = at, t = v/a
         * so required d = .5 a (v/a)^2
         *
         * equate the two expressions for d and solve for v
         */
        double tblend_vel;
        /* Minimum value of cos(theta) to prevent numerical instability */
        const double min_cos_theta = cos(PM_PI / 2.0 - TP_MIN_ARC_ANGLE);
        if (cos_theta > min_cos_theta) {
            tblend_vel = 2.0 * pmSqrt(acc_this * tc->tolerance / cos_theta);
            *v_blend_this = fmin(*v_blend_this, tblend_vel);
            *v_blend_next = fmin(*v_blend_next, tblend_vel);
        }
    }
    if (v_blend_net) {
        /*
         * Find net velocity in the direction tangent to the blend.
         * When theta ~ 0, net velocity in tangent direction is very small.
         * When the segments are nearly tangent (theta ~ pi/2), the blend
         * velocity is almost entirely in the tangent direction.
         */
        *v_blend_net = sin(theta) * (*v_blend_this + *v_blend_next) / 2.0;
    }

    return TP_ERR_OK;
}

STATIC double estimateParabolicBlendPerformance(
        TP_STRUCT const *tp,
        TC_STRUCT const *tc,
        TC_STRUCT const *nexttc)
{
    double v_this = 0.0, v_next = 0.0;

    // Use maximum possible target velocity to get best-case performance
    double target_vel_this = tpGetMaxTargetVel(tp, tc);
    double target_vel_next = tpGetMaxTargetVel(tp, nexttc);

    double v_net = 0.0;
    tpComputeBlendVelocity(tc, nexttc, target_vel_this, target_vel_next, &v_this, &v_next, &v_net);

    return v_net;
}



/**
 * Calculate distance update from velocity and acceleration.
 */
STATIC int tcUpdateDistFromAccel(TC_STRUCT * const tc, double acc, double vel_desired)
{
    // If the resulting velocity is less than zero, than we're done. This
    // causes a small overshoot, but in practice it is very small.
    double v_next = tc->currentvel + acc * tc->cycle_time;
    // update position in this tc using trapezoidal integration
    // Note that progress can be greater than the target after this step.
    if (v_next < 0.0) {
        v_next = 0.0;
        //KLUDGE: the trapezoidal planner undershoots by half a cycle time, so
        //forcing the endpoint here is necessary. However, velocity undershoot
        //also occurs during pausing and stopping, which can happen far from
        //the end. If we could "cruise" to the endpoint within a cycle at our
        //current speed, then assume that we want to be at the end.
        if ((tc->target - tc->progress) < (tc->currentvel *  tc->cycle_time)) {
            tc->progress = tc->target;
        }
    } else {
        double displacement = (v_next + tc->currentvel) * 0.5 * tc->cycle_time;
        tc->progress += displacement;
        clip_max(&tc->progress,tc->target);
    }
    tc->currentvel = v_next;

    // Check if we can make the desired velocity
    tc->on_final_decel = (fabs(vel_desired - tc->currentvel) < TP_VEL_EPSILON) && (acc < 0.0);

    return TP_ERR_OK;
}

const char *cycleModeToString(UpdateCycleMode mode)
{
    switch (mode) {
    case UPDATE_NORMAL:
        return "normal";
    case UPDATE_PARABOLIC_BLEND:
        return "parabolic_blend";
    case UPDATE_SPLIT:
        return "split_cycle";
    }
    return "unknown";
}

STATIC void tpDebugCycleInfo(TP_STRUCT const * const tp, TC_STRUCT const * const tc, TC_STRUCT const * const nexttc, double acc, int accel_mode, UpdateCycleMode cycle) {
#ifdef TC_DEBUG
    // Find maximum allowed velocity from feed and machine limits
    const double v_target = tpGetRealTargetVel(tp, tc);
    // Store a copy of final velocity
    const double v_final = tpGetRealFinalVel(tp, tc, nexttc);
    const double v_max = tpGetMaxTargetVel(tp, tc);

    /* Debug Output */
    print_json5_log_start(tc_state, Run);
    print_json5_ll_("time_ticks", tp->time_elapsed_ticks);
    print_json5_double_("id", tc->id);
    print_json5_double(v_target);
    print_json5_double(v_final);
    print_json5_double(v_max);
    print_json5_double_("target", tc->target);
    print_json5_double_("progress", tc->progress);
    print_json5_double_("v_current", tc->currentvel);
    print_json5_double_("a_current", acc);
    print_json5_double_("feed_scale", tpGetFeedScale(tp, tc));
    print_json5_double_("dt", tc->cycle_time);
    print_json5_string_("term_cond", tcTermCondAsString((tc_term_cond_t)tc->term_cond));
    print_json5_bool_("final_decel", tc->on_final_decel);
    print_json5_bool_("need_split", tc->splitting);
    print_json5_bool_("is_blending", tc->is_blending);
    print_json5_long_("canon_type", tc->canon_motion_type);
    print_json5_string_("motion_type", tcMotionTypeAsString(tc->motion_type));
    print_json5_string_("accel_mode",accel_mode ? "ramp" : "trapezoidal");
    print_json5_string_("cycle", cycleModeToString(cycle));
    print_json5_end_();
#endif
}

/**
 * Compute updated position and velocity for a timestep based on a trapezoidal
 * motion profile.
 * @param tc trajectory segment being processed.
 *
 * Creates the trapezoidal velocity profile based on the segment's velocity and
 * acceleration limits. The formula has been tweaked slightly to allow a
 * non-zero velocity at the instant the target is reached.
 */
void tpCalculateTrapezoidalAccel(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc,
        double * const acc, double * const vel_desired)
{
    // Find maximum allowed velocity from feed and machine limits
    double tc_target_vel = tpGetRealTargetVel(tp, tc);
    // Store a copy of final velocity
    double tc_finalvel = tpGetRealFinalVel(tp, tc, nexttc);

#ifdef TP_PEDANTIC
    if (tc_finalvel > 0.0 && tc->term_cond != TC_TERM_COND_TANGENT) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Final velocity of %f with non-tangent segment!\n",tc_finalvel);
        tc_finalvel = 0.0;
    }
#endif

    /* Calculations for desired velocity based on trapezoidal profile */
    double dx = tc->target - tc->progress;
    double maxaccel = tcGetTangentialMaxAccel(tc);

    double maxnewvel = findTrapezoidalDesiredVel(
		maxaccel, dx, tc_finalvel, tc->currentvel, tc->cycle_time);

    // Find bounded new velocity based on target velocity
    // Note that we use a separate variable later to check if we're on final decel
    double newvel = saturate(maxnewvel, tc_target_vel);

    // Calculate acceleration needed to reach newvel, bounded by machine maximum
    double dt = fmax(tc->cycle_time, TP_TIME_EPSILON);
    double maxnewaccel = (newvel - tc->currentvel) / dt;
    *acc = saturate(maxnewaccel, maxaccel);
    *vel_desired = maxnewvel;
}

/**
 * Calculate "ramp" acceleration for a cycle.
 */
STATIC int tpCalculateRampAccel(TP_STRUCT const * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT const * const nexttc,
        double * const acc,
        double * const vel_desired)
{
    // displacement remaining in this segment
    double dx = tc->target - tc->progress;

    if (!tc->blending_next) {
        tc->vel_at_blend_start = tc->currentvel;
    }

    double vel_final = tpGetRealFinalVel(tp, tc, nexttc);

    /* Check if the final velocity is too low to properly ramp up.*/
    if (vel_final < TP_VEL_EPSILON) {
        tp_debug_print(" vel_final %f too low for velocity ramping\n", vel_final);
        return TP_ERR_FAIL;
    }

    double vel_avg = (tc->currentvel + vel_final) / 2.0;

    // Calculate time remaining in this segment assuming constant acceleration
    double dt = 1e-16;
    if (vel_avg > TP_VEL_EPSILON) {
        dt = fmax( dx / vel_avg, 1e-16);
    }

    // Calculate velocity change between final and current velocity
    double dv = vel_final - tc->currentvel;

    // Estimate constant acceleration required
    double acc_final = dv / dt;

    // Saturate estimated acceleration against maximum allowed by segment
    double acc_max = tcGetTangentialMaxAccel(tc);

    // Output acceleration and velocity for position update
    *acc = saturate(acc_final, acc_max);
    *vel_desired = vel_final;

    return TP_ERR_OK;
}

void tpToggleDIOs(TC_STRUCT * const tc) {

    int i=0;
    if (tc->syncdio.anychanged != 0) { // we have DIO's to turn on or off
        for (i=0; i < num_dio; i++) {
            if (!(tc->syncdio.dio_mask & (1 << i))) continue;
            if (tc->syncdio.dios[i] > 0) emcmotDioWrite(i, 1); // turn DIO[i] on
            if (tc->syncdio.dios[i] < 0) emcmotDioWrite(i, 0); // turn DIO[i] off
        }
        for (i=0; i < num_aio; i++) {
            if (!(tc->syncdio.aio_mask & (1 << i))) continue;
            emcmotAioWrite(i, tc->syncdio.aios[i]); // set AIO[i]
        }
        tc->syncdio.anychanged = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }
}
/**
 * Compute the spindle displacement used for spindle position tracking.
 *
 * The displacement is always computed with respect to a specie
 */
static inline double findSpindleDisplacement(
        double new_pos,
        spindle_origin_t origin
        )
{
    return origin.direction * (new_pos - origin.position);
}

/**
 * Helper function to compare commanded and actual spindle velocity.
 * If the signs of velocity don't match, then the spindle is reversing direction.
 */
static inline bool spindleReversed(spindle_origin_t origin, double prev_pos, double current_pos)
{
    return origin.direction * (current_pos - prev_pos) < 0;
}

static inline bool cmdReverseSpindle(void)
{
    static bool reversed = false;
    // Flip sign on commanded velocity
    emcmotStatus->spindle_cmd.velocity_rpm_out *= -1;
    // (Optional) set an internal flag so we know if the spindle is reversed from the user command
    reversed = !reversed;
    return reversed;
}

/**
 * Handle special cases for rigid tapping.
 * This function deals with updating the goal position and spindle position
 * during a rigid tap cycle. In particular, the target and spindle goal need to
 * be carefully handled since we're reversing direction.
 */
STATIC void tpUpdateRigidTapState(TP_STRUCT * const tp,
        TC_STRUCT * const tc) {

    static double old_spindle_pos = 0.0;
    double spindle_pos = emcmotStatus->spindle_fb.position_rev;

    RIGIDTAP_STATE const initial_state = tc->coords.rigidtap.state;
    switch (initial_state) {
        case RIGIDTAP_START:
            old_spindlepos = new_spindlepos;
            tc->coords.rigidtap.state = TAPPING;
            // Deliberate fallthrough
        case TAPPING:
            if (tc->progress >= tc->coords.rigidtap.reversal_target) {
                // command reversal
                cmdReverseSpindle();
                tc->coords.rigidtap.state = REVERSING;
            }
            break;
        case REVERSING:
            if (spindleReversed(tp->spindle.origin, old_spindle_pos, spindle_pos) && tc->currentvel <= 0.0) {
                PmCartesian start, end;
                PmCartLine *aux = &tc->coords.rigidtap.aux_xyz;
                // we've stopped, so set a new target at the original position
                setSpindleOrigin(&tp->spindle.origin, spindle_pos);

                pmCartLinePoint(&tc->coords.rigidtap.xyz, tc->progress, &start);
                end = tc->coords.rigidtap.xyz.start;
                pmCartLineInit(aux, &start, &end);
                rtapi_print_msg(RTAPI_MSG_DBG, "old target = %f", tc->target);
                tc->coords.rigidtap.reversal_target = aux->tmag;
                tc->target = aux->tmag + 10. * tc->uu_per_rev;
                // NOTE: reset both progress and sync location:
                // At the point of reversal, the spindle is already synchronized
                tc->progress = 0.0;
                tc->progress_at_sync = 0.0;
                rtapi_print_msg(RTAPI_MSG_DBG, "new target = %f", tc->target);

                tc->coords.rigidtap.state = RETRACTION;
            }
            break;
        case RETRACTION:
            if (tc->progress >= tc->coords.rigidtap.reversal_target) {
                // Flip spindle direction again to start final reversal
                cmdReverseSpindle();
                tc->coords.rigidtap.state = FINAL_REVERSAL;
            }
            break;
        case FINAL_REVERSAL:
            if (spindleReversed(tp->spindle.origin, old_spindle_pos, spindle_pos) && tc->currentvel <= 0.0) {
                PmCartesian start, end;
                PmCartLine *aux = &tc->coords.rigidtap.aux_xyz;

                setSpindleOrigin(&tp->spindle.origin, spindle_pos);
                pmCartLinePoint(aux, tc->progress, &start);
                end = tc->coords.rigidtap.xyz.start;
                pmCartLineInit(aux, &start, &end);
                tc->target = aux->tmag;
                tc->progress = 0.0;
                //No longer need spindle sync at this point
                tc->synchronized = 0;
                tc->target_vel = tc->maxvel;

                tc->coords.rigidtap.state = FINAL_PLACEMENT;
            }
            break;
        case FINAL_PLACEMENT:
            // this is a regular move now, it'll stop at target above.
            break;
    }
    old_spindle_pos = spindle_pos;
#ifdef TC_DEBUG
    RIGIDTAP_STATE current_state = tc->coords.rigidtap.state;
    print_json5_log_start(tpUpdateRigidTapState, Run);
    print_json5_unsigned(current_state);
    print_json5_log_end();
#endif
}


/**
 * Update emcMotStatus with information about trajectory motion.
 * Based on the specified trajectory segment tc, read its progress and status
 * flags. Then, update the emcmotStatus structure with this information.
 */
STATIC int tpUpdateMovementStatus(TP_STRUCT * const tp, TC_STRUCT const * const tc ) {


    if (!tp) {
        return TP_ERR_FAIL;
    }

    if (!tc) {
        // Assume that we have no active segment, so we should clear out the status fields
        emcmotStatus->distance_to_go = 0;
        emcmotStatus->enables_queued = emcmotStatus->enables_new;
        emcmotStatus->requested_vel = 0;
        emcmotStatus->current_vel = 0;
        emcmotStatus->spindleSync = 0;
        
        emcPoseZero(&emcmotStatus->dtg);

        tp->motionType = 0;
        tp->activeDepth = 0;
        return TP_ERR_STOPPED;
    }

    EmcPose tc_pos;
    tcGetEndpoint(tc, &tc_pos);

    tp->motionType = tc->canon_motion_type;
    tp->activeDepth = tc->active_depth;
    emcmotStatus->distance_to_go = tc->target - tc->progress;
    emcmotStatus->enables_queued = tc->enables;
    // report our line number to the guis
    tp->execId = tc->id;
    emcmotStatus->requested_vel = tc->reqvel;
    emcmotStatus->current_vel = tc->currentvel;

    emcPoseSub(&tc_pos, &tp->currentPos, &emcmotStatus->dtg);
    return TP_ERR_OK;
}


/**
 * Do a parabolic blend by updating the nexttc.
 * Perform the actual blending process by updating the target velocity for the
 * next segment, then running a cycle update.
 */
STATIC void tpUpdateBlend(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    if (!nexttc) {
        return;
    }
    double save_vel = nexttc->target_vel;

    if (tpGetFeedScale(tp, nexttc) > TP_VEL_EPSILON) {
        double dv = tc->vel_at_blend_start - tc->currentvel;
        double vel_start = fmax(tc->vel_at_blend_start, TP_VEL_EPSILON);
        // Clip the ratio at 1 and 0
        double blend_progress = fmax(fmin(dv / vel_start, 1.0), 0.0);
        double blend_scale = tc->vel_at_blend_start / tc->blend_vel;
        nexttc->target_vel = blend_progress * nexttc->blend_vel * blend_scale;
        // Mark the segment as blending so we handle the new target velocity properly
        nexttc->is_blending = true;
    } else {
        // Drive the target velocity to zero since we're stopping
        nexttc->target_vel = 0.0;
    }

    tpUpdateCycle(tp, nexttc, NULL, UPDATE_PARABOLIC_BLEND);
    //Restore the original target velocity
    nexttc->target_vel = save_vel;
}


/**
 * Cleanup if tc is not valid (empty queue).
 * If the program ends, or we hit QUEUE STARVATION, do a soft reset on the trajectory planner.
 * TODO merge with tpClear?
 */
STATIC void tpHandleEmptyQueue(TP_STRUCT * const tp)
{

    tcqInit(&tp->queue);
    tp->goalPos = tp->currentPos;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->execId = 0;
    tp->motionType = 0;

    tpUpdateMovementStatus(tp, NULL);

    tpResume(tp);
}

/** Wrapper function to unlock rotary axes */
STATIC void tpSetRotaryUnlock(int axis, int unlock) {
    emcmotSetRotaryUnlock(axis, unlock);
}

/** Wrapper function to check rotary axis lock */
STATIC int tpGetRotaryIsUnlocked(int axis) {
    return emcmotGetRotaryIsUnlocked(axis);
}


/**
 * Cleanup after a trajectory segment is complete.
 * If the current move is complete and we're not waiting on the spindle for
 * const this move, then pop if off the queue and perform cleanup operations.
 * Finally, get the next move in the queue.
 */
STATIC int tpCompleteSegment(TP_STRUCT * const tp,
        TC_STRUCT const * const tc) {

    if (tp->spindle.waiting_for_atspeed == tc->id) {
        return TP_ERR_FAIL;
    }

    // if we're synced, and this move is ending, save the
    // spindle position so the next synced move can be in
    // the right place.
    if(tc->synchronized != TC_SYNC_NONE) {
        tp->spindle.origin.position += (tc->target - tc->progress_at_sync) / tc->uu_per_rev;
    } else {
        tp->spindle.origin.position = 0.0;
    }

    if(tc->indexrotary != -1) {
        // this was an indexing move, so before we remove it we must
        // relock the axis
        tpSetRotaryUnlock(tc->indexrotary, 0);
        // if it is now locked, fall through and remove the finished move.
        // otherwise, just come back later and check again
        if(tpGetRotaryIsUnlocked(tc->indexrotary)) {
            return TP_ERR_FAIL;
        }
    }

    // done with this move
    tcqRemove(&tp->queue, 1);
    tp_debug_print("Finished tc id %d\n", tc->id);

    return TP_ERR_OK;
}


/**
 * Handle an abort command.
 * Based on the current motion state, handle the consequences of an abort command.
 */
STATIC tp_err_t tpHandleAbort(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    if(!tp->aborting) {
        //Don't need to do anything if not aborting
        return TP_ERR_NO_ACTION;
    }
    //If the motion has stopped, then it's safe to reset the TP struct.
    if( MOTION_ID_VALID(tp->spindle.waiting_for_index) ||
            MOTION_ID_VALID(tp->spindle.waiting_for_atspeed) ||
            (tc->currentvel == 0.0 && (!nexttc || nexttc->currentvel == 0.0))) {
        tcqInit(&tp->queue);
        tp->goalPos = tp->currentPos;
        tp->done = 1;
        tp->depth = tp->activeDepth = 0;
        tp->aborting = 0;
        tp->execId = 0;
        tp->motionType = 0;
        tp->synchronized = 0;
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        emcmotStatus->spindle_fb.synced = 0;
        tpResume(tp);
        return TP_ERR_STOPPED;
    }  //FIXME consistent error codes
    return TP_ERR_SLOWING;
}


/**
 * Check if the spindle has reached the required speed for a move.
 * Returns a "wait" code if the spindle needs to spin up before a move and it
 * has not reached the requested speed, or the spindle index has not been
 * detected.
 */
STATIC tp_err_t tpCheckAtSpeed(TP_STRUCT * const tp, TC_STRUCT * const tc)
{

    // this is no longer the segment we were waiting_for_index for
    if (MOTION_ID_VALID(tp->spindle.waiting_for_index) && tp->spindle.waiting_for_index != tc->id)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for index on motion id %d, but reached id %d\n",
                tp->spindle.waiting_for_index, tc->id);
        tp->spindle.waiting_for_index = MOTION_INVALID_ID;
    }

    if (MOTION_ID_VALID(tp->spindle.waiting_for_atspeed) && tp->spindle.waiting_for_atspeed != tc->id)
    {

        rtapi_print_msg(RTAPI_MSG_ERR,
                "Was waiting for atspeed on motion id %d, but reached id %d\n",
                tp->spindle.waiting_for_atspeed, tc->id);
        tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
    }

    if (MOTION_ID_VALID(tp->spindle.waiting_for_atspeed)) {
        if(!emcmotStatus->spindle_is_atspeed) {
            // spindle is still not at the right speed, so wait another cycle
            return TP_ERR_WAITING;
        } else {
            tp->spindle.waiting_for_atspeed = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->spindle.waiting_for_index)) {
        if (emcmotStatus->spindle_fb.index_enable) {
            /* haven't passed index yet */
            return TP_ERR_WAITING;
        } else {
            /* passed index, start the move */
            emcmotStatus->spindle_fb.synced = 1;
            tp->spindle.waiting_for_index = MOTION_INVALID_ID;
            tc->sync_accel = 1;
        }
    }
    return TP_ERR_OK;
}


void checkPositionMatch(TP_STRUCT const *tp, TC_STRUCT const *tc)
{
    if (!emcmotConfig->consistencyCheckConfig.extraConsistencyChecks || tc->is_blending){
        return;
    }

    EmcPose tp_position_error;
    ZERO_EMC_POSE(tp_position_error);
    tcGetPos(tc, &tp_position_error);
    emcPoseSelfSub(&tp_position_error, &tp->currentPos);

    unsigned position_mismatch_axes = findAbsThresholdViolations(tp_position_error, emcmotConfig->consistencyCheckConfig.maxPositionDriftError);
    reportTPAxisError(tp, position_mismatch_axes, "Motion start position difference exceeds threshold");

#ifdef TP_DEBUG
    int need_print = true;
#else
    int need_print = position_mismatch_axes != 0;
#endif

    // Log a bunch of TP internal state if required by debug level or position error
    if (need_print) {
        print_json5_log_start(ActivateSegment,Run);
        print_json5_ll_("time_ticks", tp->time_elapsed_ticks);
        print_json5_int_("id", tc->id);
        // Position settings
        print_json5_double_("target", tc->target);
        print_json5_double_("progress", tc->progress);
        // Velocity settings
        print_json5_double_("reqvel", tc->reqvel);
        print_json5_double_("target_vel", tc->target_vel);
        print_json5_double_("finalvel", tc->finalvel);
        // Acceleration settings
        print_json5_double_("accel_scale", tcGetAccelScale(tc));
        print_json5_double_("acc_overall", tcGetOverallMaxAccel(tc));
        print_json5_double_("acc_tangential", tcGetTangentialMaxAccel(tc));
        print_json5_bool_("accel_ramp", tc->accel_mode);
        print_json5_bool_("blend_prev", tc->blend_prev);
        print_json5_string_("sync_mode", tcSyncModeAsString(tc->synchronized));
        print_json5_log_end();
    }
}


/**
 * "Activate" a segment being read for the first time.
 * This function handles initial setup of a new segment read off of the queue
 * for the first time.
 */
STATIC tp_err_t tpActivateSegment(TP_STRUCT * const tp, TC_STRUCT * const tc) {
    //Check if already active
    if (!tc || tc->active) {
        return TP_ERR_OK;
    }

#ifdef TP_PEDANTIC
    if (!tp) {
        return TP_ERR_MISSING_INPUT;
    }
#endif

    /* Based on the INI setting for "cutoff frequency", this calculation finds
     * short segments that can have their acceleration be simple ramps, instead
     * of a trapezoidal motion. This leads to fewer jerk spikes, at a slight
     * performance cost.
     * */
    double cutoff_time = 1.0 / (fmax(emcmotConfig->arcBlendRampFreq, TP_TIME_EPSILON));

    double length = tc->target - tc->progress;
    // Given what velocities we can actually reach, estimate the total time for the segment under ramp conditions
    double segment_time = 2.0 * length / (tc->currentvel + fmin(tc->finalvel,tpGetRealTargetVel(tp,tc)));

    if (segment_time < cutoff_time &&
            tc->canon_motion_type != EMC_MOTION_TYPE_TRAVERSE &&
            tc->term_cond == TC_TERM_COND_TANGENT &&
            tc->motion_type != TC_RIGIDTAP)
    {
        tc->accel_mode = TC_ACCEL_RAMP;
    }

    // Do at speed checks that only happen once
    int needs_atspeed = tc->atspeed ||
        (tc->synchronized == TC_SYNC_POSITION && !(emcmotStatus->spindle_fb.synced));

    if ( needs_atspeed && !(emcmotStatus->spindle_is_atspeed)) {
        tp->spindle.waiting_for_atspeed = tc->id;
        return TP_ERR_WAITING;
    }

    if (tc->indexrotary != -1) {
        // request that the axis unlock
        tpSetRotaryUnlock(tc->indexrotary, 1);
        // if it is unlocked, fall through and start the move.
        // otherwise, just come back later and check again
        if (!tpGetRotaryIsUnlocked(tc->indexrotary))
            return TP_ERR_WAITING;
    }

    tc->active = 1;
    //Do not change initial velocity here, since tangent blending already sets this up
    tp->motionType = tc->canon_motion_type;
    tc->blending_next = 0;
    tc->on_final_decel = 0;

    tp_err_t res = TP_ERR_OK;

    if (TC_SYNC_POSITION == tc->synchronized && !(emcmotStatus->spindle_fb.synced)) {
        tp_debug_print("Setting up position sync\n");
        // if we aren't already synced, wait
        tp->spindle.waiting_for_index = tc->id;
        // ask for an index reset
        emcmotStatus->spindle_fb.index_enable = 1;

        tp->spindle.origin.position = 0.0;
        tp->spindle.origin.direction = signum(emcmotStatus->spindle_cmd.velocity_rpm_out);
        rtapi_print_msg(RTAPI_MSG_DBG, "Waiting on sync...\n");
        res = TP_ERR_WAITING;
    }

    checkPositionMatch(tp, tc);
    return res;
}


/**
 * Run velocity mode synchronization.
 * Update requested velocity to follow the spindle's velocity (scaled by feed rate).
 */
STATIC void tpSyncVelocityMode(TC_STRUCT * const tc, TC_STRUCT * const nexttc) {
    double speed = emcmotStatus->spindle_fb.velocity_rpm;
    double pos_error = fabs(speed) * tc->uu_per_rev;
    // Account for movement due to parabolic blending with next segment
    if(nexttc) {
        pos_error -= nexttc->progress;
    }
    tc->target_vel = pos_error;

    if (nexttc && nexttc->synchronized) {
        //If the next move is synchronized too, then match it's
        //requested velocity to the current move
        nexttc->target_vel = tc->target_vel;
    }
}


/**
 * Run position mode synchronization.
 * Updates requested velocity for a trajectory segment to track the spindle's position.
 */
STATIC void tpSyncPositionMode(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc ) {

    // Start with raw spindle position and our saved offset
    double spindle_pos = emcmotStatus->spindle_fb.position_rev;

    spindle_origin_t spindle_origin = tp->spindle.origin;

    // Note that this quantity should be non-negative under normal conditions.
    double spindle_displacement = findSpindleDisplacement(spindle_pos,
                                               spindle_origin);

    tc_debug_print("spindle_displacement %f, raw_pos %f, ", spindle_displacement, spindle_pos);

    const double spindle_vel = emcmotStatus->spindle_fb.velocity_rpm / 60.0;
    if(tc->sync_accel) {
        // detect when velocities match, and move the target accordingly.
        // acceleration will abruptly stop and we will be on our new target.
        // FIX: this is driven by TP cycle time, not the segment cycle time
        // Experiment: try syncing with averaged spindle speed
        double target_vel = spindle_vel * tc->uu_per_rev;
        if(tc->currentvel >= target_vel) {
            tc_debug_print("Hit accel target in pos sync\n");
            // move target so as to drive pos_error to 0 next cycle
            tp->spindle.origin.position = spindle_pos;
            tc->progress_at_sync = tc->progress;
            tc_debug_print("Spindle offset %f\n", tp->spindle.origin.position);
            tc->sync_accel = 0;
            tc->target_vel = target_vel;
        } else {
            tc_debug_print("accelerating in pos_sync\n");
            // beginning of move and we are behind: accel as fast as we can
            tc->target_vel = tc->maxvel;
        }
    } else {
        // Multiply by user feed rate to get equivalent desired position
        const double pos_desired = spindle_displacement * tc->uu_per_rev;
        double net_progress = tc->progress - tc->progress_at_sync;

        if(nexttc) {
            // If we're in a parabolic blend, the next segment will be active too,
            // so make sure to account for its progress
            net_progress += nexttc->progress;
        }
        const double pos_error = pos_desired - net_progress;
        tc_debug_print("pos_desired %f, net_progress %f, pos_error %f\n",
                       pos_desired,
                       net_progress,
                       pos_error);

        // we have synced the beginning of the move as best we can -
        // track position (minimize pos_error).
        // This is the velocity we should be at when the position error is c0
        double v_final = spindle_vel * tc->uu_per_rev;
        /*
         * Correct for position errors when tracking spindle motion.
         * This approach assumes that if position error is 0, the correct
         * velocity is just the nominal target velocity. If the position error
         * is non-zero, however, then we need to correct it, but then return to
         * the nominal velocity.
         *
         * velocity
         * |          v_p
         * |         /\
         * |        /..\         v_0
         * |--------....-----------
         * |        ....
         * |        ....
         * |_________________________
         *         |----| t      time
         *
         * To correct a position error x_err (shaded area above), we need to
         * momentarily increase the velocity, then decrease back to the nonimal
         * velocity.
         *
         * In effect, this is the trapezoidal velocity planning problem, if:
         * 1) remaining distance dx = x_err
         * 2) "final" velocity = v_0
         * 3) max velocity / acceleration from motion segment
         */
        double a_max = tcGetTangentialMaxAccel(tc) * emcmotStatus->spindle_tracking_gain;
        double v_max = tc->maxvel;

        switch(emcmotStatus->pos_tracking_mode) {
        case 2:
        {
            double v_sq_alt = pmSq(v_final) + pos_error * a_max;
            double v_target_alt = pmSqrt(fmax(v_sq_alt, 0.0));
            tc->target_vel = v_target_alt;
            break;
        }
        case 1:
        {
            double v_sq = a_max * pos_error;
            double v_target_stock = signum(v_sq) * pmSqrt(fabs(v_sq)) + v_final;
            tc->target_vel = v_target_stock;
            break;
        }
        case 0:
        default:
        {
            double v_target_trapz = fmin(findTrapezoidalDesiredVel(a_max, pos_error, v_final, tc->currentvel, tc->cycle_time), v_max);
            tc->target_vel = v_target_trapz;
            break;
        }
        }
        emcmotStatus->pos_tracking_error = pos_error;

        tc_debug_print("in position sync, target_vel = %f, ideal_vel = %f, vel_err = %f\n", tc->target_vel, v_final, v_final - tc->target_vel);
    }

    //Finally, clip requested velocity at zero
    if (tc->target_vel < 0.0) {
        tc->target_vel = 0.0;
    }

    if (nexttc && nexttc->synchronized) {
        //If the next move is synchronized too, then match it's
        //requested velocity to the current move
        nexttc->target_vel = tc->target_vel;
    }
}


/**
 * Perform parabolic blending if needed between segments and handle status updates.
 * This isolates most of the parabolic blend stuff to make the code path
 * between tangent and parabolic blends easier to follow.
 */
STATIC int tpDoParabolicBlending(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    tpUpdateBlend(tp,tc,nexttc);

    /* Status updates */
    //Decide which segment we're in depending on which is moving faster
    if(tc->currentvel > nexttc->currentvel) {
        tpUpdateMovementStatus(tp, tc);
    } else {
        tpToggleDIOs(nexttc);
        tpUpdateMovementStatus(tp, nexttc);
    }
#ifdef TP_SHOW_BLENDS
    // hack to show blends in axis
    tp->motionType = 0;
#endif

    //Update velocity status based on both tc and nexttc
    emcmotStatus->current_vel = tc->currentvel + nexttc->currentvel;

    return TP_ERR_OK;
}


/**
 * Do a complete update on one segment.
 * Handles the majority of updates on a single segment for the current cycle.
 */
STATIC int tpUpdateCycle(TP_STRUCT * const tp,
    TC_STRUCT * const tc,
    TC_STRUCT const * const nexttc,
    UpdateCycleMode cycle_mode)
{
    //placeholders for position for this update
    EmcPose before;

    //Store the current position due to this TC
    tcGetPos(tc, &before);

    // Update the start velocity if we're not blending yet
    if (!tc->blending_next) {
        tc->vel_at_blend_start = tc->currentvel;
    }

    // Run cycle update with stored cycle time
    int res_accel = 1;
    double acc=0, vel_desired=0;
    
    // If the slowdown is not too great, use velocity ramping instead of trapezoidal velocity
    // Also, don't ramp up for parabolic blends
    if (tc->accel_mode && tc->term_cond == TC_TERM_COND_TANGENT) {
        res_accel = tpCalculateRampAccel(tp, tc, nexttc, &acc, &vel_desired);
    }

    // Check the return in case the ramp calculation failed, fall back to trapezoidal
    if (res_accel != TP_ERR_OK) {
        tpCalculateTrapezoidalAccel(tp, tc, nexttc, &acc, &vel_desired);
    }

    int accel_mode_ramp = (res_accel == TP_ERR_OK);
    tcUpdateDistFromAccel(tc, acc, vel_desired);
    tpDebugCycleInfo(tp, tc, nexttc, acc, accel_mode_ramp, cycle_mode);

    //Check if we're near the end of the cycle and set appropriate changes
    tpCheckEndCondition(tp, tc, nexttc);

    EmcPose displacement;

    // Calculate displacement
    tcGetPos(tc, &displacement);
    emcPoseSelfSub(&displacement, &before);

    //Store displacement (checking for valid pose)
    int res_set = tpAddCurrentPos(tp, &displacement);

    return res_set;
}


/**
 * Send default values to status structure.
 */
STATIC int tpUpdateInitialStatus(TP_STRUCT const * const tp) {
    // Update queue length
    emcmotStatus->tcqlen = tcqLen(&tp->queue);
    // Set default value for requested speed
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->current_vel = 0.0;
    return TP_ERR_OK;
}


/**
 * Flag a segment as needing a split cycle.
 * In addition to flagging a segment as splitting, do any preparations to store
 * data for the next cycle.
 */
STATIC inline int tcSetSplitCycle(TC_STRUCT * const tc, double split_time,
        double v_f)
{
    tc_pdebug_print("split time for id %d is %.16g\n", tc->id, split_time);
    if (tc->splitting != 0 && split_time > 0.0) {
        rtapi_print_msg(RTAPI_MSG_ERR,"already splitting on id %d with cycle time %.16g, dx = %.16g, split time %.12g\n",
                tc->id,
                tc->cycle_time,
                tc->target-tc->progress,
                split_time);
        return TP_ERR_FAIL;
    }
    tc->splitting = 1;
    tc->cycle_time = split_time;
    tc->term_vel = v_f;
    return 0;
}


/**
 * Check remaining time in a segment and calculate split cycle if necessary.
 * This function estimates how much time we need to complete the next segment.
 * If it's greater than one timestep, then we do nothing and carry on. If not,
 * then we flag the segment as "splitting", so that during the next cycle,
 * it handles the transition to the next segment.
 */
STATIC int tpCheckEndCondition(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc)
{
    EndCondition ec = checkEndCondition(tp->cycleTime,
                                        tc->progress,
                                        tc->target,
                                        tc->currentvel,
                                        tpGetRealFinalVel(tp, tc, nexttc),
                                        tcGetTangentialMaxAccel(tc)
                                        );

    double dt = ec.dt;
    int splitting = dt < tp->cycleTime;
    if (splitting) {
        if (dt < TP_TIME_EPSILON) {
            dt = 0.0;
        }
        tcSetSplitCycle(tc, dt, ec.v_f);
    }
#ifdef TC_DEBUG
    {
        print_json5_log_start(checkEndCondition, Run);
        print_json5_long_("id", tc->id);
        print_json5_double_("v_final", ec.v_f);
        print_json5_double_("t_remaining", ec.dt);
        print_json5_double_("dt_used", dt);
        print_json5_bool_("remove", tc->remove);
        print_json5_bool_("need_split", splitting);
        print_json5_end_();
    }
#endif


    return TP_ERR_OK;
}


STATIC int tpHandleSplitCycle(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{
    if (tc->remove) {
        //Don't need to update since this segment is flagged for removal
        return TP_ERR_NO_ACTION;
    }

    //Pose data to calculate movement due to finishing current TC
    EmcPose before;
    tcGetPos(tc, &before);

#ifdef TC_DEBUG
    /* Debug Output */
    print_json5_log_start(tc_splitting, Run);
    print_json5_double_("id", tc->id);
    print_json5_double_("target", tc->target);
    print_json5_double_("progress", tc->progress);
    print_json5_double_("v_terminal", tc->term_vel);
    print_json5_double_("dt", tc->cycle_time);
    print_json5_end_();
#endif


    //Shortcut tc update by assuming we arrive at end
    tc->progress = tc->target;
    //Get displacement from prev. position
    EmcPose displacement;
    tcGetPos(tc, &displacement);
    emcPoseSelfSub(&displacement, &before);

    // Update tp's position (checking for valid pose)
    tpAddCurrentPos(tp, &displacement);

    // Trigger removal of current segment at the end of the cycle
    tc->remove = 1;

    if (!nexttc) {
        tp_debug_print("no nexttc in split cycle\n");
        return TP_ERR_OK;
    }

    // Handle various cases of updates for split cycles
    //  Tangent: next segment gets a partial update for the remaining cycle time
    //  Parabolic: next segment updates with full cycle time (since both current / next are active at the same time)
    //  Exact: NO motion in next segment here since current segment must stop completely
    switch (tc->term_cond) {
        case TC_TERM_COND_TANGENT:
            nexttc->cycle_time = tp->cycleTime - tc->cycle_time;
            nexttc->currentvel = tc->term_vel;
            // Intentional fallthrough
        case TC_TERM_COND_PARABOLIC:
        {
            TC_STRUCT *next2tc = tcqItem(&tp->queue, 2);
            tpUpdateCycle(tp, nexttc, next2tc, UPDATE_SPLIT);
        }
            break;
        case TC_TERM_COND_STOP:
        case TC_TERM_COND_EXACT:
            break;
    }

    // Update status for the split portion
    // FIXME redundant tangent check, refactor to switch
    if (tc->term_cond == TC_TERM_COND_STOP
            || tc->term_cond == TC_TERM_COND_EXACT
            || (tc->cycle_time > nexttc->cycle_time && tc->term_cond == TC_TERM_COND_TANGENT)) {
        //Majority of time spent in current segment
        tpToggleDIOs(tc);
        tpUpdateMovementStatus(tp, tc);
    } else {
        tpToggleDIOs(nexttc);
        tpUpdateMovementStatus(tp, nexttc);
    }

    return TP_ERR_OK;
}

STATIC int tpHandleRegularCycle(TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{
    if (!tc) {
        return TP_ERR_FAIL;
    }
    if (tc->remove) {
        //Don't need to update since this segment is flagged for removal
        return TP_ERR_NO_ACTION;
    }
    //Run with full cycle time
    tc->cycle_time = tp->cycleTime;
    tpUpdateCycle(tp, tc, nexttc, UPDATE_NORMAL);

    /* Parabolic blending */

    double v_this = 0.0, v_next = 0.0;

    // cap the blend velocity at the current requested speed (factoring in feed override)
    double target_vel_this = tpGetRealTargetVel(tp, tc);
    double target_vel_next = tpGetRealTargetVel(tp, nexttc);

    if (tc->term_cond == TC_TERM_COND_PARABOLIC) {
        tpComputeBlendVelocity(tc, nexttc, target_vel_this, target_vel_next, &v_this, &v_next, NULL);
        tc->blend_vel = v_this;
        if (nexttc) {
            nexttc->blend_vel = v_next;
        }
    }

    if (nexttc && tcIsBlending(tc)) {
        tpDoParabolicBlending(tp, tc, nexttc);
    } else {
        //Update status for a normal step
        tpToggleDIOs(tc);
        tpUpdateMovementStatus(tp, tc);
    }
    return TP_ERR_OK;
}

/**
 * Calculate an updated goal position for the next timestep.
 * This is the brains of the operation. It's called every TRAJ period and is
 * expected to set tp->currentPos to the new machine position. Lots of other
 * const tp fields (depth, done, etc) have to be twiddled to communicate the
 * status; I think those are spelled out here correctly and I can't clean it up
 * without breaking the API that the TP presents to motion.
 */
tp_err_t updateSyncTargets(TP_STRUCT *tp, TC_STRUCT *tc, TC_STRUCT *nexttc)
{
    switch (tc->synchronized) {
        case TC_SYNC_NONE:
            emcmotStatus->spindle_fb.synced = 0;
            return TP_ERR_OK;
        case TC_SYNC_VELOCITY:
            tc_debug_print("sync velocity\n");
            tpSyncVelocityMode(tc, nexttc);
            return TP_ERR_OK;
        case TC_SYNC_POSITION:
            tc_debug_print("sync position\n");
            tpSyncPositionMode(tp, tc, nexttc);
            return TP_ERR_OK;
    }
    return TP_ERR_INVALID;
}

int tpRunCycleInternal(TP_STRUCT * const tp)
{
    //Pointers to current and next trajectory component
    TC_STRUCT *tc;
    TC_STRUCT *nexttc;

    /* Get pointers to current and relevant future segments. It's ok here if
     * future segments don't exist (NULL pointers) as we check for this later).
     */
    tc = tcqItem(&tp->queue, 0);
    nexttc = tcqItem(&tp->queue, 1);

    //Set GUI status to "zero" state
    tpUpdateInitialStatus(tp);

    //If we have a NULL pointer, then the queue must be empty, so we're done.
    if(!tc) {
        tpHandleEmptyQueue(tp);
        return TP_ERR_WAITING;
    }

    /* If the queue empties enough, assume that the program is near the end.
     * This forces the last segment to be "finalized" to let the optimizer run.*/
    /*tpHandleLowQueue(tp);*/

    /* If we're aborting or pausing and the velocity has reached zero, then we
     * don't need additional planning and can abort here. */
    if (tpHandleAbort(tp, tc, nexttc) == TP_ERR_STOPPED) {
        return TP_ERR_STOPPED;
    }

    //Return early if we have a reason to wait (i.e. not ready for motion)
    if (tpCheckAtSpeed(tp, tc) != TP_ERR_OK){
        return TP_ERR_WAITING;
    }

    if(!tc->active) {
        int res = tpActivateSegment(tp, tc);
        // Need to wait to continue motion, end planning here
        if (res == TP_ERR_WAITING) {
            tp->time_at_wait = tp->time_elapsed_ticks;
            return TP_ERR_WAITING;
        }
    }

    // Preprocess rigid tap move (handles threading direction reversals)
    if (tc->motion_type == TC_RIGIDTAP) {
        tpUpdateRigidTapState(tp, tc);
    }

    // Assume zero tracking error unless position sync is active
    emcmotStatus->pos_tracking_error = 0.0;

    // If synchronized with spindle, calculate requested velocity to track spindle motion
    updateSyncTargets(tp, tc, nexttc);
	

    tcClearFlags(tc);
    tcClearFlags(nexttc);
    // Update the current tc
    if (tc->splitting) {
        tpHandleSplitCycle(tp, tc, nexttc);
    } else {
        tpHandleRegularCycle(tp, tc, nexttc);
    }

    // If TC is complete, remove it from the queue.
    if (tc->remove) {
        tpCompleteSegment(tp, tc);
    }

    return TP_ERR_OK;
}


int tpRunCycle(TP_STRUCT *tp, long period)
{
    // Before every TP update, ensure that elapsed time and
    // TP measurements are stored for error checks
    tp->time_elapsed_sec+=tp->cycleTime;
    ++tp->time_elapsed_ticks;
    EmcPose const axis_pos_old = tp->currentPos;

    int res = tpRunCycleInternal(tp);

    if (emcmotConfig->consistencyCheckConfig.extraConsistencyChecks) {
        // After update (even a no-op), update pos / vel / accel
        EmcPose const axis_vel_old = tp->currentVel;
        EmcPose const axis_pos = tp->currentPos;

        EmcPose axis_vel;
        emcPoseSub(&axis_pos, &axis_pos_old, &axis_vel);
        emcPoseMultScalar(&axis_vel, 1.0 / tp->cycleTime);
        tp->currentVel = axis_vel;

        EmcPose axis_accel;
        emcPoseSub(&axis_vel, &axis_vel_old, &axis_accel);
        emcPoseMultScalar(&axis_accel, 1.0 / tp->cycleTime);

        unsigned accel_error_mask = findAccelViolations(axis_accel);
        unsigned vel_error_mask = findVelocityViolations(axis_vel);
        unsigned pos_limit_error_mask = findPositionLimitViolations(axis_pos);

        reportTPAxisError(tp, accel_error_mask, "Acceleration limit exceeded");
        reportTPAxisError(tp, vel_error_mask, "Velocity limit exceeded");
        reportTPAxisError(tp, pos_limit_error_mask, "Position limits exceeded");

#ifdef TC_DEBUG
        unsigned debug_mask = (unsigned)(-1);
#else
        unsigned debug_mask = 0;
#endif

        if (debug_mask || (
                    accel_error_mask | vel_error_mask | pos_limit_error_mask)
                ) {
            print_json5_log_start(tpRunCycle, Run);
            print_json5_ll_("time_ticks", tp->time_elapsed_ticks);
            print_json5_EmcPose(axis_pos);
            print_json5_EmcPose(axis_vel);
            print_json5_EmcPose(axis_accel);
            double current_vel = emcmotStatus->current_vel;
            print_json5_double(current_vel);
            print_json5_double_("time", tp->time_elapsed_sec);
            print_json5_end_();
        }
    }
    return res;
}

int tpSetSpindleSync(TP_STRUCT * const tp, double sync, int mode) {
    // WARNING assumes positive sync
    if(sync > 0) {
        if (mode) {
            tp->synchronized = TC_SYNC_VELOCITY;
        } else {
            tp->synchronized = TC_SYNC_POSITION;
        }
        tp->uu_per_rev = sync;
    } else
        tp->synchronized = 0;

    return TP_ERR_OK;
}

int tpPause(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 1;
    return TP_ERR_OK;
}

int tpResume(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 0;
    return TP_ERR_OK;
}

int tpAbort(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (!tp->aborting) {
        /* const to abort, signal a pause and set our abort flag */
        tpPause(tp);
        tp->aborting = 1;
    }
    return tpClearDIOs(tp); //clears out any already cached DIOs
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{

    if (0 == tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    } else {
        *pos = tp->currentPos;
    }

    return TP_ERR_OK;
}

int tpIsDone(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->done;
}

int tpQueueDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->depth;
}

int tpActiveDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->activeDepth;
}

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1; //something has changed
    tp->syncdio.aio_mask |= (1 << index);
    tp->syncdio.aios[index] = start;
    return TP_ERR_OK;
}

int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->syncdio.anychanged = 1; //something has changed
    tp->syncdio.dio_mask |= (1 << index);
    if (start > 0)
        tp->syncdio.dios[index] = 1; // the end value can't be set from canon currently, and has the same value as start
    else
        tp->syncdio.dios[index] = -1;
    return TP_ERR_OK;
}


// vim:sw=4:sts=4:et:
