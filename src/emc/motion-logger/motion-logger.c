
//
// motion-logger: a test program to log motion commands sent from
//     LinuxCNC's Task module to the Motion module
//
// Copyright (C) 2015, Sebastian Kuzminsky <seb@highlab.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//

#include <errno.h>
#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "hal.h"
#include "motion_debug.h"
#include "motion.h"
#include "motion_struct.h"
#include "motion_types.h"
#include "mot_priv.h"

static struct motion_logger_data_t {
    hal_bit_t *reopen;
} *motion_logger_data;

FILE *logfile = NULL;
char *logfile_name = NULL;

emcmot_struct_t *emcmotStruct = 0;

struct emcmot_command_t *c = 0;
struct emcmot_status_t *emcmotStatus = 0;
struct emcmot_config_t *emcmotConfig = 0;
struct emcmot_debug_t *emcmotDebug = 0;
struct emcmot_internal_t *emcmotInternal = 0;
struct emcmot_error_t *emcmotError = 0;

int mot_comp_id;

emcmot_joint_t joint_array[EMCMOT_MAX_JOINTS];
int num_joints = EMCMOT_MAX_JOINTS;
emcmot_joint_t *joints = 0;


void emcmot_config_change(void) {
    if (emcmotConfig->head == emcmotConfig->tail) {
        emcmotConfig->head++;
        emcmotConfig->config_num++;
        emcmotStatus->config_num = emcmotConfig->config_num;
        emcmotConfig->tail = emcmotConfig->head;
    }
}


static int init_comm_buffers(void) {
    int joint_num, n;
    emcmot_joint_t *joint;
    int retval;
    int shmem_id;

    rtapi_print_msg(RTAPI_MSG_INFO,
	"MOTION: init_comm_buffers() starting...\n");

    emcmotStruct = 0;
    emcmotDebug = 0;
    emcmotStatus = 0;
    c = 0;
    emcmotConfig = 0;

    /* allocate and initialize the shared memory structure */
    shmem_id = rtapi_shmem_new(DEFAULT_SHMEM_KEY, mot_comp_id, sizeof(emcmot_struct_t));
    if (shmem_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_new failed, returned %d\n", shmem_id);
	return -1;
    }
    retval = rtapi_shmem_getptr(shmem_id, (void **) &emcmotStruct);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_getptr failed, returned %d\n", retval);
	return -1;
    }

    /* zero shared memory before doing anything else. */
    memset(emcmotStruct, 0, sizeof(emcmot_struct_t));

    /* we'll reference emcmotStruct directly */
    c = &emcmotStruct->command;
    emcmotStatus = &emcmotStruct->status;
    emcmotConfig = &emcmotStruct->config;
    emcmotDebug = &emcmotStruct->debug;
    emcmotInternal = &emcmotStruct->internal;
    emcmotError = &emcmotStruct->error;

    emcmotConfig->numJoints = num_joints;

    emcmotStatus->vel = DEFAULT_VELOCITY;
    emcmotConfig->limitVel = DEFAULT_VELOCITY;
    emcmotStatus->acc = DEFAULT_ACCELERATION;
    emcmotStatus->feed_scale = 1.0;
    emcmotStatus->rapid_scale = 1.0;
    emcmotStatus->spindle_scale = 1.0;
    emcmotStatus->net_feed_scale = 1.0;
    /* adaptive feed is off by default, feed override, spindle 
       override, and feed hold are on */
    emcmotStatus->enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    SET_MOTION_INPOS_FLAG(1);
    emcmotConfig->kinematics_type = KINEMATICS_IDENTITY;

    emcmot_config_change();

    /* init pointer to joint structs */
    joints = joint_array;

    /* init per-joint stuff */
    for (joint_num = 0; joint_num < num_joints; joint_num++) {
	/* point to structure for this joint */
	joint = &joints[joint_num];

	/* init the config fields with some "reasonable" defaults" */

	joint->max_pos_limit = 1.0;
	joint->min_pos_limit = -1.0;
	joint->vel_limit = 1.0;
	joint->acc_limit = 1.0;
	joint->min_ferror = 0.01;
	joint->max_ferror = 1.0;
	joint->home_final_vel = -1;
	joint->home_sequence = -1;

	joint->comp.entry = &(joint->comp.array[0]);
	/* the compensation code has -DBL_MAX at one end of the table
	   and +DBL_MAX at the other so _all_ commanded positions are
	   guaranteed to be covered by the table */
	joint->comp.array[0].nominal = -DBL_MAX;
	joint->comp.array[0].fwd_trim = 0.0;
	joint->comp.array[0].rev_trim = 0.0;
	joint->comp.array[0].fwd_slope = 0.0;
	joint->comp.array[0].rev_slope = 0.0;
	for ( n = 1 ; n < EMCMOT_COMP_SIZE+2 ; n++ ) {
	    joint->comp.array[n].nominal = DBL_MAX;
	    joint->comp.array[n].fwd_trim = 0.0;
	    joint->comp.array[n].rev_trim = 0.0;
	    joint->comp.array[n].fwd_slope = 0.0;
	    joint->comp.array[n].rev_slope = 0.0;
	}

	/* init status info */
	joint->ferror_limit = joint->min_ferror;

	/* init misc other stuff in joint structure */
	joint->big_vel = 10.0 * joint->vel_limit;

	SET_JOINT_INPOS_FLAG(joint, 1);
    }

    emcmotDebug->start_time = time(NULL);

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_comm_buffers() complete\n");
    return 0;
}


void update_motion_state(void) {
    if (GET_MOTION_ENABLE_FLAG()) {
        if (GET_MOTION_TELEOP_FLAG()) {
            emcmotStatus->motion_state = EMCMOT_MOTION_TELEOP;
        } else if (GET_MOTION_COORD_FLAG()) {
            emcmotStatus->motion_state = EMCMOT_MOTION_COORD;
        } else {
            emcmotStatus->motion_state = EMCMOT_MOTION_FREE;
        }
    } else {
        emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
    }
}


void update_joint_status(void) {
    for (int j = 0; j < num_joints; j ++) {
        emcmot_joint_status_t *joint_status;
        emcmot_joint_t *joint;

        joint_status = &emcmotStatus->joint_status[j];
        joint = &joints[j];

	joint_status->flag = joint->flag;
	joint_status->pos_cmd = joint->pos_cmd;
	joint_status->pos_fb = joint->pos_fb;
	joint_status->vel_cmd = joint->vel_cmd;
	joint_status->ferror = joint->ferror;
	joint_status->ferror_high_mark = joint->ferror_high_mark;
	joint_status->backlash = joint->backlash;
	joint_status->max_pos_limit = joint->max_pos_limit;
	joint_status->min_pos_limit = joint->min_pos_limit;
	joint_status->min_ferror = joint->min_ferror;
	joint_status->max_ferror = joint->max_ferror;
	joint_status->home_offset = joint->home_offset;
    }
}


static void mark_joint_homed(int joint_num) {
    emcmot_joint_t *joint;

    joint = &joints[joint_num];

    SET_JOINT_HOMING_FLAG(joint, 0);
    SET_JOINT_HOMED_FLAG(joint, 1);
    SET_JOINT_AT_HOME_FLAG(joint, 1);
    joint->home_state = HOME_IDLE;
}

void maybe_reopen_logfile() {
    if(*motion_logger_data->reopen) {
        if(logfile != stdout) {
            fclose(logfile);
            logfile = NULL;
        }
        *motion_logger_data->reopen = 0;
    }
}

void log_print(const char *fmt, ...) {
    va_list ap;

    maybe_reopen_logfile();

    if (logfile == NULL) {
        if (logfile_name == NULL) {
            logfile = stdout;
        } else {
            logfile = fopen(logfile_name, "w");
            if (logfile == NULL) {
                fprintf(stderr, "error opening %s: %s\n", logfile_name, strerror(errno));
                exit(1);
            }
        }
    }

    va_start(ap, fmt);
    vfprintf(logfile, fmt, ap);
    va_end(ap);
    fflush(logfile);
}


int main(int argc, char* argv[]) {
    if (argc == 1) {
        logfile = stdout;
        logfile_name = NULL;
    } else if (argc == 2) {
        logfile = NULL;
        logfile_name = argv[1];
    } else {
        fprintf(stderr, "usage: motion-logger [LOGFILE]\n");
        exit(1);
    }

    mot_comp_id = hal_init("motion-logger");
    motion_logger_data = hal_malloc(sizeof(*motion_logger_data));
    int r = hal_pin_bit_new("motion-logger.reopen-log", HAL_IO, &motion_logger_data->reopen,
            mot_comp_id);
    if(r < 0) { errno = -r; perror("hal_pin_bit_new"); exit(1); }
    *motion_logger_data->reopen = 0;
    r = hal_ready(mot_comp_id);
    if(r < 0) { errno = -r; perror("hal_ready"); exit(1); }
    init_comm_buffers();

    while (1) {
        if (c->commandNum != c->tail) {
            // "split read"
            continue;
        }
        if (c->commandNum == emcmotStatus->commandNumEcho) {
            // nothing new
            maybe_reopen_logfile();
            usleep(10 * 1000);
            continue;
        }

        //
        // new incoming command!
        //

        emcmotStatus->head++;

        switch (c->command) {
            case EMCMOT_ABORT:
                log_print("ABORT\n");
                break;

            case EMCMOT_AXIS_ABORT:
                log_print("AXIS_ABORT joint=%d\n", c->axis);
                break;

            case EMCMOT_ENABLE:
                log_print("ENABLE\n");
                SET_MOTION_ENABLE_FLAG(1);
                update_motion_state();
                break;

            case EMCMOT_DISABLE:
                log_print("DISABLE\n");
                SET_MOTION_ENABLE_FLAG(0);
                update_motion_state();
                break;

            case EMCMOT_ENABLE_AMPLIFIER:
                log_print("ENABLE_AMPLIFIER\n");
                break;

            case EMCMOT_DISABLE_AMPLIFIER:
                log_print("DISABLE_AMPLIFIER\n");
                break;

            case EMCMOT_ENABLE_WATCHDOG:
                log_print("ENABLE_WATCHDOG\n");
                break;

            case EMCMOT_DISABLE_WATCHDOG:
                log_print("DISABLE_WATCHDOG\n");
                break;

            case EMCMOT_ACTIVATE_JOINT:
                log_print("ACTIVATE_JOINT joint=%d\n", c->axis);
                break;

            case EMCMOT_DEACTIVATE_JOINT:
                log_print("DEACTIVATE_JOINT joint=%d\n", c->axis);
                break;

            case EMCMOT_PAUSE:
                log_print("PAUSE\n");
                break;

            case EMCMOT_RESUME:
                log_print("RESUME\n");
                break;

            case EMCMOT_STEP:
                log_print("STEP\n");
                break;

            case EMCMOT_FREE:
                log_print("FREE\n");
                SET_MOTION_COORD_FLAG(0);
                SET_MOTION_TELEOP_FLAG(0);
                update_motion_state();
                break;

            case EMCMOT_COORD:
                log_print("COORD\n");
                SET_MOTION_COORD_FLAG(1);
                SET_MOTION_TELEOP_FLAG(0);
                SET_MOTION_ERROR_FLAG(0);
                update_motion_state();
                break;

            case EMCMOT_TELEOP:
                log_print("TELEOP\n");
                SET_MOTION_TELEOP_FLAG(1);
                SET_MOTION_ERROR_FLAG(0);
                update_motion_state();
                break;

            case EMCMOT_SPINDLE_SCALE:
                log_print("SPINDLE_SCALE\n");
                break;

            case EMCMOT_SS_ENABLE:
                log_print("SS_ENABLE\n");
                break;

            case EMCMOT_FEED_SCALE:
                log_print("FEED_SCALE\n");
                break;

            case EMCMOT_RAPID_SCALE:
                log_print("RAPID_SCALE\n");
                break;

            case EMCMOT_FS_ENABLE:
                log_print("FS_ENABLE\n");
                break;

            case EMCMOT_FH_ENABLE:
                log_print("FH_ENABLE\n");
                break;

            case EMCMOT_AF_ENABLE:
                log_print("AF_ENABLE\n");
                break;

            case EMCMOT_OVERRIDE_LIMITS:
                log_print("OVERRIDE_LIMITS\n");
                break;

            case EMCMOT_HOME:
                log_print("HOME joint=%d\n", c->axis);
                if (c->axis < 0) {
                    for (int j = 0; j < num_joints; j ++) {
                        mark_joint_homed(j);
                    }
                } else {
                    mark_joint_homed(c->axis);
                }
                break;

            case EMCMOT_UNHOME:
                log_print("UNHOME joint=%d\n", c->axis);
                break;

            case EMCMOT_JOG_CONT:
                log_print("JOG_CONT\n");
                break;

            case EMCMOT_JOG_INCR:
                log_print("JOG_INCR\n");
                break;

            case EMCMOT_JOG_ABS:
                log_print("JOG_ABS\n");
                break;

            case EMCMOT_SET_LINE:
                log_print(
                    "SET_LINE x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f, u=%.6f, v=%.6f, w=%.6f, id=%d, motion_type=%d, vel=%.6f, ini_maxvel=%.6f, acc=%.6f, turn=%d\n",
                    c->pos.tran.x, c->pos.tran.y, c->pos.tran.z,
                    c->pos.a, c->pos.b, c->pos.c,
                    c->pos.u, c->pos.v, c->pos.w,
                    c->id, c->motion_type,
                    c->vel, c->ini_maxvel,
                    c->acc, c->turn
                );
                break;

            case EMCMOT_SET_CIRCLE:
                log_print("SET_CIRCLE:\n");
                log_print(
                    "    pos: x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f, u=%.6f, v=%.6f, w=%.6f\n",
                    c->pos.tran.x, c->pos.tran.y, c->pos.tran.z,
                    c->pos.a, c->pos.b, c->pos.c,
                    c->pos.u, c->pos.v, c->pos.w
                );
                log_print("    center: x=%.6f, y=%.6f, z=%.6f\n", c->center.x, c->center.y, c->center.z);
                log_print("    normal: x=%.6f, y=%.6f, z=%.6f\n", c->normal.x, c->normal.y, c->normal.z);
                log_print("    id=%d, motion_type=%d, vel=%.6f, ini_maxvel=%.6f, acc=%.6f, turn=%d\n",
                    c->id, c->motion_type,
                    c->vel, c->ini_maxvel,
                    c->acc, c->turn
                );
                break;

            case EMCMOT_SET_TELEOP_VECTOR:
                log_print("SET_TELEOP_VECTOR\n");
                break;

            case EMCMOT_CLEAR_PROBE_FLAGS:
                log_print("CLEAR_PROBE_FLAGS\n");
                break;

            case EMCMOT_PROBE:
                log_print("PROBE\n");
                break;

            case EMCMOT_RIGID_TAP:
                log_print("RIGID_TAP\n");
                break;

            case EMCMOT_SET_POSITION_LIMITS:
                log_print(
                    "SET_POSITION_LIMITS joint=%d, min=%.6f, max=%.6f\n",
                    c->axis, c->minLimit, c->maxLimit
                );
                joints[c->axis].max_pos_limit = c->maxLimit;
                joints[c->axis].min_pos_limit = c->minLimit;
                break;

            case EMCMOT_SET_BACKLASH:
                log_print("SET_BACKLASH joint=%d, backlash=%.6f\n", c->axis, c->backlash);
                break;

            case EMCMOT_SET_MIN_FERROR:
                log_print("SET_MIN_FERROR joint=%d, minFerror=%.6f\n", c->axis, c->minFerror);
                break;

            case EMCMOT_SET_MAX_FERROR:
                log_print("SET_MAX_FERROR joint=%d, maxFerror=%.6f\n", c->axis, c->maxFerror);
                break;

            case EMCMOT_SET_VEL:
                log_print("SET_VEL vel=%.6f, ini_maxvel=%.6f\n", c->vel, c->ini_maxvel);
                break;

            case EMCMOT_SET_VEL_LIMIT:
                log_print("SET_VEL_LIMIT vel=%.6f\n", c->vel);
                break;

            case EMCMOT_SET_JOINT_VEL_LIMIT:
                log_print("SET_JOINT_VEL_LIMIT joint=%d, vel=%.6f\n", c->axis, c->vel);
                break;

            case EMCMOT_SET_JOINT_ACC_LIMIT:
                log_print("SET_JOINT_ACC_LIMIT joint=%d, acc=%.6f\n", c->axis, c->acc);
                break;

            case EMCMOT_SET_ACC:
                log_print("SET_ACC acc=%.6f\n", c->acc);
                break;

            case EMCMOT_SET_TERM_COND:
                log_print("SET_TERM_COND termCond=%d, tolerance=%.6f\n", c->termCond, c->tolerance);
                break;

            case EMCMOT_SET_NUM_AXES:
                log_print("SET_NUM_AXES %d\n", c->axis);
                num_joints = c->axis;
                break;

            case EMCMOT_SET_WORLD_HOME:
                log_print(
                    "SET_WORLD_HOME x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f, u=%.6f, v=%.6f, w=%.6f\n",
                    c->pos.tran.x, c->pos.tran.y, c->pos.tran.z,
                    c->pos.a, c->pos.b, c->pos.c,
                    c->pos.u, c->pos.v, c->pos.w
                );
                break;

            case EMCMOT_SET_HOMING_PARAMS:
                log_print(
                    "SET_HOMING_PARAMS joint=%d, offset=%.6f home=%.6f, final_vel=%.6f, search_vel=%.6f, latch_vel=%.6f, flags=0x%08x, sequence=%d, volatile=%d\n",
                    c->axis, c->offset, c->home, c->home_final_vel,
                    c->search_vel, c->latch_vel, c->flags,
                    c->home_sequence, c->volatile_home
                );
                break;

            case EMCMOT_SET_DEBUG:
                log_print("SET_DEBUG\n");
                break;

            case EMCMOT_SET_DOUT:
                log_print("SET_DOUT\n");
                break;

            case EMCMOT_SET_AOUT:
                log_print("SET_AOUT\n");
                break;

            case EMCMOT_SET_SPINDLESYNC:
                log_print("SET_SPINDLESYNC sync=%06f, flags=0x%08x\n", c->spindlesync, c->flags);
                break;

            case EMCMOT_SPINDLE_ON:
                log_print("SPINDLE_ON speed=%f, css_factor=%f, xoffset=%f\n",
                          c->spindle_speed, c->css_factor, c->css_xoffset);
                emcmotStatus->spindle_cmd.velocity_rpm_out = c->spindle_speed;
                break;

            case EMCMOT_SPINDLE_OFF:
                log_print("SPINDLE_OFF\n");
                emcmotStatus->spindle_cmd.velocity_rpm_out = 0;
                break;

            case EMCMOT_SPINDLE_INCREASE:
                log_print("SPINDLE_INCREASE\n");
                break;

            case EMCMOT_SPINDLE_DECREASE:
                log_print("SPINDLE_DECREASE\n");
                break;

            case EMCMOT_SPINDLE_BRAKE_ENGAGE:
                log_print("SPINDLE_BRAKE_ENGAGE\n");
                break;

            case EMCMOT_SPINDLE_BRAKE_RELEASE:
                log_print("SPINDLE_BRAKE_RELEASE\n");
                break;

            case EMCMOT_SPINDLE_ORIENT:
                log_print("SPINDLE_ORIENT\n");
                break;

            case EMCMOT_SET_MOTOR_OFFSET:
                log_print("SET_MOTOR_OFFSET\n");
                break;

            case EMCMOT_SET_JOINT_COMP:
                log_print("SET_JOINT_COMP\n");
                break;

            case EMCMOT_SET_OFFSET:
                log_print(
                    "SET_OFFSET x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f u=%.6f, v=%.6f, w=%.6f\n",
                    c->tool_offset.tran.x, c->tool_offset.tran.y, c->tool_offset.tran.z,
                    c->tool_offset.a, c->tool_offset.b, c->tool_offset.c,
                    c->tool_offset.u, c->tool_offset.v, c->tool_offset.w
                );
                break;

            case EMCMOT_SET_MAX_FEED_OVERRIDE:
                log_print("SET_MAX_FEED_OVERRIDE %.6f\n", c->maxFeedScale);
                break;

            case EMCMOT_SETUP_ARC_BLENDS:
                log_print("SETUP_ARC_BLENDS\n");
                break;

            case EMCMOT_SETUP_CONSISTENCY_CHECKS:
                log_print("SETUP_CONSISTENCY_CHECKS enabled=%d, position_drift=%f\n",
                          c->consistencyCheckConfig.extraConsistencyChecks,
                          c->consistencyCheckConfig.maxPositionDriftError);
                break;

            default:
                log_print("ERROR: unknown command %d\n", c->command);
                break;
        }

        update_joint_status();

        emcmotStatus->commandEcho = c->command;
        emcmotStatus->commandNumEcho = c->commandNum;
        emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;
        emcmotStatus->tail = emcmotStatus->head;
    }

    return 0;
}

