
#include "axis.h"
#include "emcmotcfg.h"      // EMCMOT_MAX_AXIS
#include "rtapi.h"
#include "rtapi_math.h"
#include "simple_tp.h"

typedef struct {
    double pos_cmd;                 /* commanded axis position */
    double teleop_vel_cmd;          /* commanded axis velocity */
    double max_pos_limit;           /* upper soft limit on axis pos */
    double min_pos_limit;           /* lower soft limit on axis pos */
    double vel_limit;               /* upper limit of axis speed */
    double acc_limit;               /* upper limit of axis accel */
    simple_tp_t teleop_tp;          /* planner for teleop mode motion */

    int old_ajog_counts;            /* prior value, used for deltas */
    int kb_ajog_active;             /* non-zero during a keyboard jog */
    int wheel_ajog_active;          /* non-zero during a wheel jog */
    int locking_joint;              /* locking_joint number, -1 ==> notused */

    double ext_offset_vel_limit;    /* upper limit of axis speed for ext offset */
    double ext_offset_acc_limit;    /* upper limit of axis accel for ext offset */
    int old_eoffset_counts;
    simple_tp_t ext_offset_tp;      /* planner for external coordinate offsets*/
} emcmot_axis_t;

typedef struct {
    hal_float_t *pos_cmd;           /* RPI: commanded position */
    hal_float_t *teleop_vel_cmd;    /* RPI: commanded velocity */
    hal_float_t *teleop_pos_cmd;    /* RPI: teleop traj planner pos cmd */
    hal_float_t *teleop_vel_lim;    /* RPI: teleop traj planner vel limit */
    hal_bit_t   *teleop_tp_enable;  /* RPI: teleop traj planner is running */

    hal_s32_t   *ajog_counts;       /* WPI: jogwheel position input */
    hal_bit_t   *ajog_enable;       /* RPI: enable jogwheel */
    hal_float_t *ajog_scale;        /* RPI: distance to jog on each count */
    hal_float_t *ajog_accel_fraction;  /* RPI: to limit wheel jog accel */
    hal_bit_t   *ajog_vel_mode;     /* RPI: true for "velocity mode" jogwheel */
    hal_bit_t   *kb_ajog_active;    /* RPI: executing keyboard jog */
    hal_bit_t   *wheel_ajog_active; /* RPI: executing handwheel jog */

    hal_bit_t   *eoffset_enable;
    hal_bit_t   *eoffset_clear;
    hal_s32_t   *eoffset_counts;
    hal_float_t *eoffset_scale;
    hal_float_t *external_offset;
    hal_float_t *external_offset_requested;
} axis_hal_t;


typedef struct {
    axis_hal_t axis[EMCMOT_MAX_AXIS];   /* data for each axis */
} axis_hal_data_t;

static emcmot_axis_t axis_array[EMCMOT_MAX_AXIS];
static axis_hal_data_t *hal_data = NULL;


// Mark strings for translation, but defer translation to userspace
#define _(s) (s)

void axis_init_all(void)
{
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        emcmot_axis_t *axis = &axis_array[n];
        axis->locking_joint = -1;
    }
}

void axis_initialize_external_offsets(void)
{
    int n;
    axis_hal_t *axis_data;

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis_data = &hal_data->axis[n];

        *(axis_data->external_offset) = 0;
        *(axis_data->external_offset_requested) = 0;
        axis_array[n].ext_offset_tp.pos_cmd  = 0;
        axis_array[n].ext_offset_tp.curr_pos = 0;
        axis_array[n].ext_offset_tp.curr_vel = 0;
    }
}

#define CALL_CHECK(expr) do {           \
        int _retval;                    \
        _retval = expr;                 \
        if (_retval) return _retval;    \
    } while (0);

static int export_axis(int mot_comp_id, char c, axis_hal_t * addr)
{
    int msg;

    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    CALL_CHECK(hal_pin_bit_newf(HAL_IN, &(addr->ajog_enable), mot_comp_id,"axis.%c.jog-enable", c));
    CALL_CHECK(hal_pin_float_newf(HAL_IN, &(addr->ajog_scale), mot_comp_id,"axis.%c.jog-scale", c));
    CALL_CHECK(hal_pin_s32_newf(HAL_IN, &(addr->ajog_counts), mot_comp_id,"axis.%c.jog-counts", c));
    CALL_CHECK(hal_pin_bit_newf(HAL_IN, &(addr->ajog_vel_mode), mot_comp_id,"axis.%c.jog-vel-mode", c));
    CALL_CHECK(hal_pin_bit_newf(HAL_OUT, &(addr->kb_ajog_active), mot_comp_id,"axis.%c.kb-jog-active", c));
    CALL_CHECK(hal_pin_bit_newf(HAL_OUT, &(addr->wheel_ajog_active), mot_comp_id,"axis.%c.wheel-jog-active", c));

    CALL_CHECK(hal_pin_float_newf(HAL_IN,&(addr->ajog_accel_fraction), mot_comp_id,"axis.%c.jog-accel-fraction", c));
    *addr->ajog_accel_fraction = 1.0; // fraction of accel for wheel ajogs

    rtapi_set_msg_level(msg);
    return 0;
}

int axis_init_hal_io(int mot_comp_id)
{
    int n, retval;

    hal_data = hal_malloc(sizeof(axis_hal_data_t));
    if (!hal_data) {
        rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: axis_hal_data hal_malloc() failed\n"));
        return -1;
    }

    // export axis pins and parameters
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        char c = "xyzabcuvw"[n];
        axis_hal_t *axis_data = &(hal_data->axis[n]);
        CALL_CHECK(hal_pin_float_newf(HAL_OUT, &axis_data->pos_cmd, mot_comp_id, "axis.%c.pos-cmd", c));
        CALL_CHECK(hal_pin_float_newf(HAL_OUT, &axis_data->teleop_vel_cmd, mot_comp_id, "axis.%c.teleop-vel-cmd", c));
        CALL_CHECK(hal_pin_float_newf(HAL_OUT, &axis_data->teleop_pos_cmd, mot_comp_id, "axis.%c.teleop-pos-cmd", c));
        CALL_CHECK(hal_pin_float_newf(HAL_OUT, &axis_data->teleop_vel_lim, mot_comp_id, "axis.%c.teleop-vel-lim", c));
        CALL_CHECK(hal_pin_bit_newf(HAL_OUT, &axis_data->teleop_tp_enable, mot_comp_id, "axis.%c.teleop-tp-enable",c));
        CALL_CHECK(hal_pin_bit_newf(HAL_IN, &axis_data->eoffset_enable, mot_comp_id, "axis.%c.eoffset-enable", c));
        CALL_CHECK(hal_pin_bit_newf(HAL_IN, &axis_data->eoffset_clear, mot_comp_id, "axis.%c.eoffset-clear", c));
        CALL_CHECK(hal_pin_s32_newf(HAL_IN, &axis_data->eoffset_counts, mot_comp_id, "axis.%c.eoffset-counts", c));
        CALL_CHECK(hal_pin_float_newf(HAL_IN, &axis_data->eoffset_scale, mot_comp_id, "axis.%c.eoffset-scale", c));
        CALL_CHECK(hal_pin_float_newf(HAL_OUT, &axis_data->external_offset, mot_comp_id, "axis.%c.eoffset", c));
        CALL_CHECK(hal_pin_float_newf(HAL_OUT, &axis_data->external_offset_requested,
           mot_comp_id, "axis.%c.eoffset-request", c));

        retval = export_axis(mot_comp_id, c, axis_data);
        if (retval) {
            rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: axis %c pin/param export failed\n"), c);
            return -1;
        }
    }

    return 0;
}

void axis_output_to_hal(double *pcmd_p[])
{
    int n;

    // output axis info to HAL for scoping, etc
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        emcmot_axis_t *axis = &axis_array[n];
        axis_hal_t *axis_data = &hal_data->axis[n];
        *(axis_data->teleop_vel_cmd)    = axis->teleop_vel_cmd;
        *(axis_data->teleop_pos_cmd)    = axis->teleop_tp.pos_cmd;
        *(axis_data->teleop_vel_lim)    = axis->teleop_tp.max_vel;
        *(axis_data->teleop_tp_enable)  = axis->teleop_tp.enable;
        *(axis_data->kb_ajog_active)    = axis->kb_ajog_active;
        *(axis_data->wheel_ajog_active) = axis->wheel_ajog_active;

        // hal pins: axis.L.pos-cmd reported without applied offsets:
        *(axis_data->pos_cmd) = *pcmd_p[n]
                              - axis->ext_offset_tp.curr_pos;
     }
}

void axis_set_max_pos_limit(int axis_num, double maxLimit)
{
    axis_array[axis_num].max_pos_limit = maxLimit;
}

void axis_set_min_pos_limit(int axis_num, double minLimit)
{
    axis_array[axis_num].min_pos_limit = minLimit;
}

void axis_set_vel_limit(int axis_num, double vel)
{
    axis_array[axis_num].vel_limit = vel;
}

void axis_set_acc_limit(int axis_num, double acc)
{
    axis_array[axis_num].acc_limit = acc;
}

void axis_set_ext_offset_vel_limit(int axis_num, double vel)
{
    axis_array[axis_num].ext_offset_vel_limit = vel;
}

void axis_set_ext_offset_acc_limit(int axis_num, double acc)
{
    axis_array[axis_num].ext_offset_acc_limit = acc;
}

void axis_set_locking_joint(int axis_num, int joint)
{
    axis_array[axis_num].locking_joint = joint;
}


double axis_get_min_pos_limit(int axis_num)
{
    return axis_array[axis_num].min_pos_limit;
}

double axis_get_max_pos_limit(int axis_num)
{
    return axis_array[axis_num].max_pos_limit;
}

double axis_get_vel_limit(int axis_num)
{
    return axis_array[axis_num].vel_limit;
}

double axis_get_acc_limit(int axis_num)
{
    return axis_array[axis_num].acc_limit;
}

double axis_get_teleop_vel_cmd(int axis_num)
{
    return axis_array[axis_num].teleop_vel_cmd;
}

int axis_get_locking_joint(int axis_num)
{
    return axis_array[axis_num].locking_joint;
}

double axis_get_compound_velocity(void)
{
    double v2 = 0.0;
    int n;

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        emcmot_axis_t *axis = &axis_array[n];
        if (axis->teleop_tp.active) {
            v2 += axis->teleop_vel_cmd * axis->teleop_vel_cmd;
        }
    }

    if (v2 > 0.0)
        return sqrt(v2);
    return 0.0;
}

double axis_get_ext_offset_curr_pos(int axis_num)
{
    return axis_array[axis_num].ext_offset_tp.curr_pos;
}


void axis_jog_cont(int axis_num, double vel, long servo_period)
{
    (void)servo_period;
    emcmot_axis_t *axis = &axis_array[axis_num];

    if (vel > 0.0) {
        axis->teleop_tp.pos_cmd = axis->max_pos_limit;
    } else {
        axis->teleop_tp.pos_cmd = axis->min_pos_limit;
    }

    axis->teleop_tp.max_vel = fabs(vel);
    axis->teleop_tp.max_acc = axis->acc_limit;
    axis->kb_ajog_active = 1;
    axis->teleop_tp.enable = 1;
}

void axis_jog_incr(int axis_num, double offset, double vel, long servo_period)
{
    (void)servo_period;
    emcmot_axis_t *axis = &axis_array[axis_num];
    double tmp1;

    if (vel > 0.0) {
        tmp1 = axis->teleop_tp.pos_cmd + offset;
    } else {
        tmp1 = axis->teleop_tp.pos_cmd - offset;
    }

    if (tmp1 > axis->max_pos_limit) { return; }
    if (tmp1 < axis->min_pos_limit) { return; }

    axis->teleop_tp.pos_cmd = tmp1;
    axis->teleop_tp.max_vel = fabs(vel);
    axis->teleop_tp.max_acc = axis->acc_limit;
    axis->kb_ajog_active = 1;
    axis->teleop_tp.enable = 1;
}

void axis_jog_abs(int axis_num, double offset, double vel)
{
    emcmot_axis_t *axis = &axis_array[axis_num];
    double tmp1;

    axis->kb_ajog_active = 1;
    if (axis->wheel_ajog_active) { return; }
    if (vel > 0.0) {
        tmp1 = axis->teleop_tp.pos_cmd + offset;
    } else {
        tmp1 = axis->teleop_tp.pos_cmd - offset;
    }
    if (tmp1 > axis->max_pos_limit) { return; }
    if (tmp1 < axis->min_pos_limit) { return; }
    axis->teleop_tp.pos_cmd = tmp1;
    axis->teleop_tp.max_vel = fabs(vel);
    axis->teleop_tp.max_acc = axis->acc_limit;
    axis->kb_ajog_active = 1;
    axis->teleop_tp.enable = 1;
}

bool axis_jog_abort(int axis_num, bool immediate)
{
    bool aborted = 0;
    emcmot_axis_t *axis = &axis_array[axis_num];
    if (axis->teleop_tp.enable) {
        aborted = 1;
    }
    axis->teleop_tp.enable = 0;
    axis->kb_ajog_active = 0;
    axis->wheel_ajog_active = 0;
    if (immediate) {
        axis->teleop_tp.curr_vel = 0.0;
    }
    return aborted;
}

bool axis_jog_abort_all(bool immediate)
{
    int n;
    bool aborted = 0;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        if (axis_jog_abort(n, immediate)) {aborted = 1;}
    }
    return aborted;
}

bool axis_jog_is_active(void)
{
    int n;
    emcmot_axis_t *axis;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        if (axis->kb_ajog_active || axis->wheel_ajog_active) {
            return 1;
        }
    }
    return 0;
}

void axis_handle_jogwheels(bool motion_teleop_flag, bool motion_enable_flag, bool homing_is_active)
{
    int axis_num;
    emcmot_axis_t *axis;
    axis_hal_t *axis_data;
    int new_ajog_counts, delta;
    double distance, pos, stop_dist;
    static int first_pass = 1;	/* used to set initial conditions */

    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        double aaccel_limit;
        axis = &axis_array[axis_num];
        axis_data = &hal_data->axis[axis_num];

        // disallow accel bogus fractions
        if (   (*(axis_data->ajog_accel_fraction) > 1)
            || (*(axis_data->ajog_accel_fraction) < 0) ) {
            aaccel_limit = axis->acc_limit;
        } else {
            aaccel_limit = *(axis_data->ajog_accel_fraction) * axis->acc_limit;
        }

        new_ajog_counts = *(axis_data->ajog_counts);
        delta = new_ajog_counts - axis->old_ajog_counts;
        axis->old_ajog_counts = new_ajog_counts;
        if ( first_pass ) { continue; }
        if ( delta == 0 ) {
            //just update counts
            continue;
        }
        if (!motion_teleop_flag) {
            axis->teleop_tp.enable = 0;
            return;
        }
        if (!motion_enable_flag)              { continue; }
        if ( *(axis_data->ajog_enable) == 0 ) { continue; }
        if (homing_is_active)                 { continue; }
        if (axis->kb_ajog_active)             { continue; }

        if (axis->locking_joint >= 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
            "Cannot wheel jog a locking indexer AXIS_%c\n",
            "XYZABCUVW"[axis_num]);
            continue;
        }

        distance = delta * *(axis_data->ajog_scale);
        pos = axis->teleop_tp.pos_cmd + distance;
        if ( *(axis_data->ajog_vel_mode) ) {
            double v = axis->vel_limit;
            /* compute stopping distance at max speed */
            stop_dist = v * v / ( 2 * aaccel_limit);
            /* if commanded position leads the actual position by more
               than stopping distance, discard excess command */
            if ( pos > axis->pos_cmd + stop_dist ) {
                pos = axis->pos_cmd + stop_dist;
            } else if ( pos < axis->pos_cmd - stop_dist ) {
                pos = axis->pos_cmd - stop_dist;
            }
        }
        if (pos > axis->max_pos_limit) { break; }
        if (pos < axis->min_pos_limit) { break; }
        axis->teleop_tp.pos_cmd = pos;
        axis->teleop_tp.max_vel = axis->vel_limit;
        axis->teleop_tp.max_acc = aaccel_limit;
        axis->wheel_ajog_active = 1;
        axis->teleop_tp.enable  = 1;
    }
    first_pass = 0;
}

void axis_sync_teleop_tp_to_carte_pos(int extfactor, double *pcmd_p[])
{
    int n;
    // expect extfactor =  -1 || 0 || +1
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis_array[n].teleop_tp.curr_pos = *pcmd_p[n]
                            + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}

void axis_sync_carte_pos_to_teleop_tp(int extfactor, double *pcmd_p[])
{
    int n;
    // expect extfactor =  -1 || 0 || +1
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *pcmd_p[n] = axis_array[n].teleop_tp.curr_pos
                            + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}

void axis_apply_ext_offsets_to_carte_pos(int extfactor, double *pcmd_p[])
{
    int n;
    // expect extfactor =  -1 || 0 || +1
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *pcmd_p[n] = *pcmd_p[n]
                            + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}

bool axis_plan_external_offsets(double servo_period, bool motion_enable_flag, bool all_homed)
{
    static int first_pass = 1;
    int n;
    emcmot_axis_t *axis;
    axis_hal_t *axis_data;
    int new_eoffset_counts, delta;
    static int last_eoffset_enable[EMCMOT_MAX_AXIS];
    double ext_offset_epsilon;
    hal_bit_t eoffset_active;

    eoffset_active = 0;

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        // coord,teleop updates done in get_pos_cmds()
        axis->ext_offset_tp.max_vel = axis->ext_offset_vel_limit;
        axis->ext_offset_tp.max_acc = axis->ext_offset_acc_limit;

        axis_data = &hal_data->axis[n];

        new_eoffset_counts       = *(axis_data->eoffset_counts);
        delta                    = new_eoffset_counts - axis->old_eoffset_counts;
        axis->old_eoffset_counts = new_eoffset_counts;

        *(axis_data->external_offset)  = axis->ext_offset_tp.curr_pos;
        axis->ext_offset_tp.enable = 1;
        if ( first_pass ) {
            *(axis_data->external_offset) = 0;
            continue;
        }

        // Use stopping criterion of simple_tp.c:
        ext_offset_epsilon = TINY_DP(axis->ext_offset_tp.max_acc, servo_period);
        if (fabs(*(axis_data->external_offset)) > ext_offset_epsilon) {
            eoffset_active = 1;
        }
        if ( !*(axis_data->eoffset_enable) ) {
            axis->ext_offset_tp.enable = 0;
            // Detect disabling of eoffsets:
            //   At very high accel, simple planner may terminate with
            //   a larger position value than occurs at more realistic accels.
            if (last_eoffset_enable[n]
                && (fabs(*(axis_data->external_offset)) > ext_offset_epsilon)
                && motion_enable_flag
                && axis->ext_offset_tp.enable) {
                // to stdout only:
                rtapi_print_msg(RTAPI_MSG_NONE,
                           "*** Axis_%c External Offset=%.4g eps=%.4g\n"
                           "*** External Offset disabled while NON-zero\n"
                           "*** To clear: re-enable & zero or use Machine-Off\n",
                           "XYZABCUVW"[n],
                           *(axis_data->external_offset),
                           ext_offset_epsilon);
            }
            last_eoffset_enable[n] = 0;
            continue; // Note: if   not eoffset_enable
                      //       then planner disabled and no pos_cmd updates
                      //       useful for eoffset_pid hold
        }
        last_eoffset_enable[n] = 1;
        if (*(axis_data->eoffset_clear)) {
            axis->ext_offset_tp.pos_cmd             = 0;
            *(axis_data->external_offset_requested) = 0;
            continue;
        }
        if (delta == 0)           { continue; }
        if (!all_homed)           { continue; }
        if (!motion_enable_flag)  { continue; }

        axis->ext_offset_tp.pos_cmd   += delta *  *(axis_data->eoffset_scale);
        *(axis_data->external_offset_requested) = axis->ext_offset_tp.pos_cmd;
    } // for n
    first_pass = 0;

    return eoffset_active;
}

/* For each axis, return -1 if over negative limit, 1 if over positive limit,
   or 0 if in range */
void axis_check_constraints(double pos[], int failing_axes[])
{
    int axis_num;
    double eps = 1e-308;

    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num += 1) {
        double nl = axis_array[axis_num].min_pos_limit;
        double pl = axis_array[axis_num].max_pos_limit;
        failing_axes[axis_num] = 0;

        if (   (fabs(pos[axis_num]) < eps)
            && (fabs(axis_array[axis_num].min_pos_limit) < eps)
            && (fabs(axis_array[axis_num].max_pos_limit) < eps) ) {
            continue;
        }

        if (pos[axis_num] < (nl - 0.000000000001)) { // see pull request #1047
            failing_axes[axis_num] = -1;
        }

        if (pos[axis_num] > (pl + 0.000000000001)) { // see pull request #1047
            failing_axes[axis_num] = 1;
        }
    }
}

int axis_update_coord_with_bound(double *pcmd_p[], double servo_period)
{
    int n;
    int ans = 0;
    emcmot_axis_t *axis;
    double save_pos_cmd[EMCMOT_MAX_AXIS];
    double save_offset_cmd[EMCMOT_MAX_AXIS];

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        save_pos_cmd[n]     = *pcmd_p[n];
        save_offset_cmd[n]  = axis->ext_offset_tp.pos_cmd;
        simple_tp_update(&(axis->ext_offset_tp), servo_period);
    }
    axis_apply_ext_offsets_to_carte_pos(+1, pcmd_p); // add external offsets

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        //workaround: axis letters not in [TRAJ]COORDINATES
        //            have min_pos_limit == max_pos_lim == 0
        if ( (0 == axis->max_pos_limit) && (0 == axis->min_pos_limit) ) {
            continue;
        }
        if (axis->ext_offset_tp.curr_pos == 0) {
           continue; // don't claim violation if no offset
        }

        if (*pcmd_p[n] >= axis->max_pos_limit) {
            // hold carte_pos_cmd at the limit:
            *pcmd_p[n]  = axis->max_pos_limit;
            // stop growth of offsetting position:
            axis->ext_offset_tp.curr_pos = axis->max_pos_limit
                                         - save_pos_cmd[n];
            if (axis->ext_offset_tp.pos_cmd > save_offset_cmd[n]) {
                axis->ext_offset_tp.pos_cmd = save_offset_cmd[n];
            }
            axis->ext_offset_tp.curr_vel = 0;
            ans++;
            continue;
        }
        if (*pcmd_p[n] <= axis->min_pos_limit) {
            *pcmd_p[n]  = axis->min_pos_limit;
            axis->ext_offset_tp.curr_pos = axis->min_pos_limit
                                         - save_pos_cmd[n];
            if (axis->ext_offset_tp.pos_cmd < save_offset_cmd[n]) {
                axis->ext_offset_tp.pos_cmd = save_offset_cmd[n];
            }
            axis->ext_offset_tp.curr_vel = 0;
            ans++;
        }
    }
    if (ans > 0) { return 1; }
    return 0;
}

static int update_teleop_with_check(int axis_num, simple_tp_t *the_tp, double servo_period)
{
    // 'the_tp' is the planner to update
    // the tests herein apply to the sum of the offsets for both
    // planners (teleop_tp and ext_offset_tp)
    double save_curr_pos;
    emcmot_axis_t *axis = &axis_array[axis_num];

    save_curr_pos = the_tp->curr_pos;
    simple_tp_update(the_tp, servo_period);

    //workaround: axis letters not in [TRAJ]COORDINATES
    //            have min_pos_limit == max_pos_lim == 0
    if  ( (0 == axis->max_pos_limit) && (0 == axis->min_pos_limit) ) {
        return 0;
    }
    if  ( (axis->ext_offset_tp.curr_pos + axis->teleop_tp.curr_pos)
          >= axis->max_pos_limit) {
        // positive error, restore save_curr_pos
        the_tp->curr_pos = save_curr_pos;
        the_tp->curr_vel = 0;
        return 1;
    }
    if  ( (axis->ext_offset_tp.curr_pos + axis->teleop_tp.curr_pos)
           <= axis->min_pos_limit) {
        // negative error, restore save_curr_pos
        the_tp->curr_pos = save_curr_pos;
        the_tp->curr_vel = 0;
        return 1;
    }
    return 0;
}

int axis_calc_motion(double servo_period)
{
    int axis_num;
    int violated_teleop_limit = 0;
    emcmot_axis_t *axis;

    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        axis = &axis_array[axis_num];
        // teleop_tp.max_vel is always positive
        if (axis->teleop_tp.max_vel > axis->vel_limit) {
            axis->teleop_tp.max_vel = axis->vel_limit;
        }
        if (update_teleop_with_check(axis_num, &(axis->teleop_tp), servo_period)) {
            violated_teleop_limit = 1;
        } else {
            axis->teleop_vel_cmd = axis->teleop_tp.curr_vel;
            axis->pos_cmd = axis->teleop_tp.curr_pos;
        }

        if (!axis->teleop_tp.active) {
            axis->kb_ajog_active = 0;
            axis->wheel_ajog_active = 0;
        }

        if (axis->ext_offset_tp.enable) {
            if (update_teleop_with_check(axis_num, &(axis->ext_offset_tp), servo_period)) {
                violated_teleop_limit = 1;
            }
        }
    }
    return violated_teleop_limit;
}
