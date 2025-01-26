/********************************************************************
* Description: halui.cc
*   HAL User-Interface component.
*   This file exports various UI related hal pins, and communicates
*   with EMC through NML messages
*
*   Derived from a work by Fred Proctor & Will Shackleford (emcsh.cc)
*   some of the functions (sendFooBar() are adapted from there)
*
* Author: Alex Joni
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2006 All rights reserved.
*
* Last change:
********************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

#include "hal.h"		/* access to HAL functions/definitions */
#include "rtapi.h"		/* rtapi_print_msg */
#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "nml_oi.hh"
#include "timer.hh"
#include <rtapi_string.h>
#include "tooldata.hh"

/* Using halui: see the man page */

static int axis_mask = 0;
#define JOGJOINT  1
#define JOGTELEOP 0

#define MDI_MAX 64

#pragma GCC diagnostic push
#if defined(__GNUC__) && (__GNUC__ > 4)
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif

#define HAL_FIELDS \
    FIELD(hal_bit_t,machine_on) /* pin for setting machine On */ \
    FIELD(hal_bit_t,machine_off) /* pin for setting machine Off */ \
    FIELD(hal_bit_t,machine_is_on) /* pin for machine is On/Off */ \
    FIELD(hal_bit_t,estop_activate) /* pin for activating EMC ESTOP  */ \
    FIELD(hal_bit_t,estop_reset) /* pin for resetting ESTOP */ \
    FIELD(hal_bit_t,estop_is_activated) /* pin for status ESTOP is activated */ \
\
    FIELD(hal_bit_t,mode_manual) /* pin for requesting manual mode */ \
    FIELD(hal_bit_t,mode_is_manual) /* pin for manual mode is on */ \
    FIELD(hal_bit_t,mode_auto) /* pin for requesting auto mode */ \
    FIELD(hal_bit_t,mode_is_auto) /* pin for auto mode is on */ \
    FIELD(hal_bit_t,mode_mdi) /* pin for requesting mdi mode */ \
    FIELD(hal_bit_t,mode_is_mdi) /* pin for mdi mode is on */ \
    FIELD(hal_bit_t,mode_teleop) /* pin for requesting teleop mode */ \
    FIELD(hal_bit_t,mode_is_teleop) /* pin for teleop mode is on */ \
    FIELD(hal_bit_t,mode_joint) /* pin for requesting joint mode */ \
    FIELD(hal_bit_t,mode_is_joint) /* pin for joint mode is on */ \
\
    FIELD(hal_bit_t,mist_on) /* pin for starting mist */ \
    FIELD(hal_bit_t,mist_off) /* pin for stopping mist */ \
    FIELD(hal_bit_t,mist_is_on) /* pin for mist is on */ \
    FIELD(hal_bit_t,flood_on) /* pin for starting flood */ \
    FIELD(hal_bit_t,flood_off) /* pin for stopping flood */ \
    FIELD(hal_bit_t,flood_is_on) /* pin for flood is on */ \
\
    FIELD(hal_bit_t,program_is_idle) /* pin for notifying user that program is idle */ \
    FIELD(hal_bit_t,program_is_running) /* pin for notifying user that program is running */ \
    FIELD(hal_bit_t,halui_mdi_is_running) /* pin for notifying user that halui MDI commands is running */ \
    FIELD(hal_bit_t,program_is_paused) /* pin for notifying user that program is paused */ \
    FIELD(hal_bit_t,program_run) /* pin for running program */ \
    FIELD(hal_bit_t,program_pause) /* pin for pausing program */ \
    FIELD(hal_bit_t,program_resume) /* pin for resuming program */ \
    FIELD(hal_bit_t,program_step) /* pin for running one line of the program */ \
    FIELD(hal_bit_t,program_stop) /* pin for stopping the program */ \
    FIELD(hal_bit_t,program_os_on) /* pin for setting optional stop on */ \
    FIELD(hal_bit_t,program_os_off) /* pin for setting optional stop off */ \
    FIELD(hal_bit_t,program_os_is_on) /* status pin that optional stop is on */ \
    FIELD(hal_bit_t,program_bd_on) /* pin for setting block delete on */ \
    FIELD(hal_bit_t,program_bd_off) /* pin for setting block delete off */ \
    FIELD(hal_bit_t,program_bd_is_on) /* status pin that block delete is on */ \
\
    FIELD(hal_u32_t,tool_number) /* pin for current selected tool */ \
    FIELD(hal_float_t,tool_length_offset_x) /* current applied x tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_y) /* current applied y tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_z) /* current applied z tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_a) /* current applied a tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_b) /* current applied b tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_c) /* current applied c tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_u) /* current applied u tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_v) /* current applied v tool-length-offset */ \
    FIELD(hal_float_t,tool_length_offset_w) /* current applied w tool-length-offset */ \
    FIELD(hal_float_t,tool_diameter) /* current tool diameter (0 if no tool) */ \
\
    ARRAY(hal_bit_t,spindle_start,EMCMOT_MAX_SPINDLES+1) /* pin for starting the spindle */ \
    ARRAY(hal_bit_t,spindle_stop,EMCMOT_MAX_SPINDLES+1) /* pin for stopping the spindle */ \
    ARRAY(hal_bit_t,spindle_is_on,EMCMOT_MAX_SPINDLES+1) /* status pin for spindle is on */ \
    ARRAY(hal_bit_t,spindle_forward,EMCMOT_MAX_SPINDLES+1) /* pin for making the spindle go forward */ \
    ARRAY(hal_bit_t,spindle_runs_forward,EMCMOT_MAX_SPINDLES+1) /* status pin for spindle running forward */ \
    ARRAY(hal_bit_t,spindle_reverse,EMCMOT_MAX_SPINDLES+1) /* pin for making the spindle go reverse */ \
    ARRAY(hal_bit_t,spindle_runs_backward,EMCMOT_MAX_SPINDLES+1) /* status pin for spindle running backward */ \
    ARRAY(hal_bit_t,spindle_increase,EMCMOT_MAX_SPINDLES+1) /* pin for making the spindle go faster */ \
    ARRAY(hal_bit_t,spindle_decrease,EMCMOT_MAX_SPINDLES+1) /* pin for making the spindle go slower */ \
\
    ARRAY(hal_bit_t,spindle_brake_on,EMCMOT_MAX_SPINDLES) /* pin for activating spindle-brake */ \
    ARRAY(hal_bit_t,spindle_brake_off, EMCMOT_MAX_SPINDLES) /* pin for deactivating spindle/brake */ \
    ARRAY(hal_bit_t,spindle_brake_is_on, EMCMOT_MAX_SPINDLES) /* status pin that tells us if brake is on */ \
\
    ARRAY(hal_bit_t,joint_home,EMCMOT_MAX_JOINTS+1) /* pin for homing one joint */ \
    ARRAY(hal_bit_t,joint_unhome,EMCMOT_MAX_JOINTS+1) /* pin for unhoming one joint */ \
    ARRAY(hal_bit_t,joint_is_homed,EMCMOT_MAX_JOINTS+1) /* status pin that the joint is homed */ \
    ARRAY(hal_bit_t,joint_on_soft_min_limit,EMCMOT_MAX_JOINTS+1) /* status pin that the joint is on the software min limit */ \
    ARRAY(hal_bit_t,joint_on_soft_max_limit,EMCMOT_MAX_JOINTS+1) /* status pin that the joint is on the software max limit */ \
    ARRAY(hal_bit_t,joint_on_hard_min_limit,EMCMOT_MAX_JOINTS+1) /* status pin that the joint is on the hardware min limit */ \
    ARRAY(hal_bit_t,joint_on_hard_max_limit,EMCMOT_MAX_JOINTS+1) /* status pin that the joint is on the hardware max limit */ \
    ARRAY(hal_bit_t,joint_override_limits,EMCMOT_MAX_JOINTS+1) /* status pin that the joint is on the hardware max limit */ \
    ARRAY(hal_bit_t,joint_has_fault,EMCMOT_MAX_JOINTS+1) /* status pin that the joint has a fault */ \
    FIELD(hal_u32_t,joint_selected) /* status pin for the joint selected */ \
    FIELD(hal_u32_t,axis_selected) /* status pin for the axis selected */ \
\
    ARRAY(hal_bit_t,joint_nr_select,EMCMOT_MAX_JOINTS) /* nr. of pins to select a joint */ \
    ARRAY(hal_bit_t,axis_nr_select,EMCMOT_MAX_AXIS) /* nr. of pins to select a axis */ \
\
    ARRAY(hal_bit_t,joint_is_selected,EMCMOT_MAX_JOINTS) /* nr. of status pins for joint selected */ \
    ARRAY(hal_bit_t,axis_is_selected,EMCMOT_MAX_AXIS) /* nr. of status pins for axis selected */ \
\
    ARRAY(hal_float_t,axis_pos_commanded,EMCMOT_MAX_AXIS+1) /* status pin for commanded cartesian position */ \
    ARRAY(hal_float_t,axis_pos_feedback,EMCMOT_MAX_AXIS+1) /* status pin for actual cartesian position */ \
    ARRAY(hal_float_t,axis_pos_relative,EMCMOT_MAX_AXIS+1) /* status pin for relative cartesian position */ \
\
    FIELD(hal_float_t,jjog_speed) /* pin for setting the jog speed (halui internal) */ \
    ARRAY(hal_bit_t,jjog_minus,EMCMOT_MAX_JOINTS+1) /* pin to jog in positive direction */ \
    ARRAY(hal_bit_t,jjog_plus,EMCMOT_MAX_JOINTS+1) /* pin to jog in negative direction */ \
    ARRAY(hal_float_t,jjog_analog,EMCMOT_MAX_JOINTS+1) /* pin for analog jogging (-1..0..1) */ \
    ARRAY(hal_float_t,jjog_increment,EMCMOT_MAX_JOINTS+1) /* Incremental jogging */ \
    ARRAY(hal_bit_t,jjog_increment_plus,EMCMOT_MAX_JOINTS+1) /* Incremental jogging, positive direction */ \
    ARRAY(hal_bit_t,jjog_increment_minus,EMCMOT_MAX_JOINTS+1) /* Incremental jogging, negative direction */ \
\
    FIELD(hal_float_t,ajog_speed) /* pin for setting the jog speed (halui internal) */ \
    ARRAY(hal_bit_t,ajog_minus,EMCMOT_MAX_AXIS+1) /* pin to jog in positive direction */ \
    ARRAY(hal_bit_t,ajog_plus,EMCMOT_MAX_AXIS+1) /* pin to jog in negative direction */ \
    ARRAY(hal_float_t,ajog_analog,EMCMOT_MAX_AXIS+1) /* pin for analog jogging (-1..0..1) */ \
    ARRAY(hal_float_t,ajog_increment,EMCMOT_MAX_AXIS+1) /* Incremental jogging */ \
    ARRAY(hal_bit_t,ajog_increment_plus,EMCMOT_MAX_AXIS+1) /* Incremental jogging, positive direction */ \
    ARRAY(hal_bit_t,ajog_increment_minus,EMCMOT_MAX_AXIS+1) /* Incremental jogging, negative direction */ \
\
    FIELD(hal_float_t,jjog_deadband) /* pin for setting the jog analog deadband (where not to move) */ \
    FIELD(hal_float_t,ajog_deadband) /* pin for setting the jog analog deadband (where not to move) */ \
\
    FIELD(hal_s32_t,mv_counts) /* pin for the Max Velocity counting */ \
    FIELD(hal_bit_t,mv_count_enable) /* pin for the Max Velocity counting enable */ \
    FIELD(hal_bit_t,mv_direct_value) /* pin for enabling direct value option instead of counts */ \
    FIELD(hal_float_t,mv_scale) /* scale for the Max Velocity counting */ \
    FIELD(hal_float_t,mv_value) /* current Max Velocity value */ \
    FIELD(hal_bit_t,mv_increase) /* pin for increasing the MV (+=scale) */ \
    FIELD(hal_bit_t,mv_decrease) /* pin for decreasing the MV (-=scale) */ \
\
    FIELD(hal_s32_t,fo_counts) /* pin for the Feed Override counting */ \
    FIELD(hal_bit_t,fo_count_enable) /* pin for the Feed Override counting enable */ \
    FIELD(hal_bit_t,fo_direct_value) /* pin for enabling direct value option instead of counts  */ \
    FIELD(hal_float_t,fo_scale) /* scale for the Feed Override counting */ \
    FIELD(hal_float_t,fo_value) /* current Feed Override value */ \
    FIELD(hal_bit_t,fo_increase) /* pin for increasing the FO (+=scale) */ \
    FIELD(hal_bit_t,fo_decrease) /* pin for decreasing the FO (-=scale) */ \
    FIELD(hal_bit_t,fo_reset) /* pin for resetting Feed Override */ \
\
    FIELD(hal_s32_t,ro_counts) /* pin for the Feed Override counting */ \
    FIELD(hal_bit_t,ro_count_enable) /* pin for the Feed Override counting enable */ \
    FIELD(hal_bit_t,ro_direct_value) /* pin for enabling direct value option instead of counts  */ \
    FIELD(hal_float_t,ro_scale) /* scale for the Feed Override counting */ \
    FIELD(hal_float_t,ro_value) /* current Feed Override value */ \
    FIELD(hal_bit_t,ro_increase) /* pin ror increasing the FO (+=scale) */ \
    FIELD(hal_bit_t,ro_decrease) /* pin for decreasing the FO (-=scale) */ \
    FIELD(hal_bit_t,ro_reset) /* pin for resetting Feed Override */ \
\
    ARRAY(hal_s32_t,so_counts,EMCMOT_MAX_SPINDLES+1) /* pin for the Spindle Speed Override counting */ \
    ARRAY(hal_bit_t,so_count_enable,EMCMOT_MAX_SPINDLES+1) /* pin for the Spindle Speed Override counting enable */ \
    ARRAY(hal_bit_t,so_direct_value,EMCMOT_MAX_SPINDLES+1) /* pin for enabling direct value option instead of counts */ \
    ARRAY(hal_float_t,so_scale,EMCMOT_MAX_SPINDLES+1) /* scale for the Spindle Speed Override counting */ \
    ARRAY(hal_float_t,so_value,EMCMOT_MAX_SPINDLES+1) /* current Spindle speed Override value */ \
    ARRAY(hal_bit_t,so_increase,EMCMOT_MAX_SPINDLES+1) /* pin for increasing the SO (+=scale) */ \
    ARRAY(hal_bit_t,so_decrease,EMCMOT_MAX_SPINDLES+1) /* pin for decreasing the SO (-=scale) */ \
    ARRAY(hal_bit_t,so_reset,EMCMOT_MAX_SPINDLES+1) /* pin for resetting Spindle Speed Override */ \
\
    FIELD(hal_bit_t,home_all) /* pin for homing all joints in sequence */ \
    FIELD(hal_bit_t,abort) /* pin for aborting */ \
    ARRAY(hal_bit_t,mdi_commands,MDI_MAX) \
\
    FIELD(hal_float_t,units_per_mm) \

struct PTR {
    template<class T>
    struct field { typedef T *type; };
};

template<class T> struct NATIVE {};
template<> struct NATIVE<hal_bit_t> { typedef bool type; };
template<> struct NATIVE<hal_s32_t> { typedef rtapi_s32 type; };
template<> struct NATIVE<hal_u32_t> { typedef rtapi_u32 type; };
template<> struct NATIVE<hal_float_t> { typedef double type; };
struct VALUE {
    template<class T> struct field { typedef typename NATIVE<T>::type type; };
};

template<class T>
struct halui_str_base
{
#define FIELD(t,f) typename T::template field<t>::type f;
#define ARRAY(t,f,n) typename T::template field<t>::type f[n];
HAL_FIELDS
#undef FIELD
#undef ARRAY
};

typedef halui_str_base<PTR> halui_str;
typedef halui_str_base<VALUE> local_halui_str;
#pragma GCC diagnostic pop

static halui_str *halui_data;
static local_halui_str old_halui_data;

static char *mdi_commands[MDI_MAX];
static int num_mdi_commands=0;
static int have_home_all = 0;

static int comp_id, done;				/* component ID, main while loop */

static int num_axes = 0; //number of axes, taken from the INI [TRAJ] section
static int num_joints = 3; //number of joints, taken from the INI [KINS] section
static int num_spindles = 1; // number of spindles, [TRAJ]SPINDLES

static double maxFeedOverride=1;
static double maxMaxVelocity=1;
static double minSpindleOverride=0.0;
static double maxSpindleOverride=1.0;
static EMC_TASK_MODE halui_old_mode = EMC_TASK_MODE::MANUAL;
static int halui_sent_mdi = 0;

// the NML channels to the EMC task
static RCS_CMD_CHANNEL *emcCommandBuffer = 0;
static RCS_STAT_CHANNEL *emcStatusBuffer = 0;
EMC_STAT *emcStatus = 0;

// the NML channel for errors
static NML *emcErrorBuffer = 0;

// the serial number to use.
static int emcCommandSerialNumber = 0;

// how long to wait for Task to report that it has received our command
static double receiveTimeout = 10.0;

// how long to wait for Task to finish running our command
static double doneTimeout = 60.;

static void quit(int /*sig*/)
{
    done = 1;
}

static int emcTaskNmlGet()
{
    int retval = 0;

    // try to connect to EMC cmd
    if (emcCommandBuffer == 0) {
	emcCommandBuffer =
	    new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "xemc",
				emc_nmlfile);
	if (!emcCommandBuffer->valid()) {
	    delete emcCommandBuffer;
	    emcCommandBuffer = 0;
	    retval = -1;
	}
    }
    // try to connect to EMC status
    if (emcStatusBuffer == 0) {
	emcStatusBuffer =
	    new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc",
				 emc_nmlfile);
	if (!emcStatusBuffer->valid()) {
	    delete emcStatusBuffer;
	    emcStatusBuffer = 0;
	    emcStatus = 0;
	    retval = -1;
	} else {
	    emcStatus = (EMC_STAT *) emcStatusBuffer->get_address();
	}
    }

    return retval;
}

static int emcErrorNmlGet()
{
    int retval = 0;

    if (emcErrorBuffer == 0) {
	emcErrorBuffer =
	    new NML(nmlErrorFormat, "emcError", "xemc", emc_nmlfile);
	if (!emcErrorBuffer->valid()) {
	    delete emcErrorBuffer;
	    emcErrorBuffer = 0;
	    retval = -1;
	}
    }

    return retval;
}

static int tryNml()
{
    double end;
    int good;
#define RETRY_TIME 10.0		// seconds to wait for subsystems to come up
#define RETRY_INTERVAL 1.0	// seconds between wait tries for a subsystem

    end = RETRY_TIME;
    good = 0;
    do {
        if (0 == emcTaskNmlGet()) {
            good = 1;
            break;
        }
        esleep(RETRY_INTERVAL);
        end -= RETRY_INTERVAL;
    } while (end > 0.0);

    if (!good) {
        return -1;
    }

    end = RETRY_TIME;
    good = 0;
    do {
        if (0 == emcErrorNmlGet()) {
            good = 1;
            break;
        }
        esleep(RETRY_INTERVAL);
        end -= RETRY_INTERVAL;
    } while (end > 0.0);

    if (!good) {
        return -1;
    }

    return 0;

#undef RETRY_TIME
#undef RETRY_INTERVAL
}

static int updateStatus()
{
    NMLTYPE type;

    if (0 == emcStatus || 0 == emcStatusBuffer) {
        rtapi_print("halui: %s: no status buffer\n", __func__);
        return -1;
    }

    if (!emcStatusBuffer->valid()) {
        rtapi_print("halui: %s: status buffer is not valid\n", __func__);
	return -1;
    }

    switch (type = emcStatusBuffer->peek()) {
    case -1:
	// error on CMS channel
        rtapi_print("halui: %s: error peeking status buffer\n", __func__);
	return -1;
	break;

    case 0:			// no new data
    case EMC_STAT_TYPE:	// new data
	break;

    default:
        rtapi_print("halui: %s: unknown error peeking status buffer\n", __func__);
	return -1;
	break;
    }

    return 0;
}


#define EMC_COMMAND_DELAY   0.1	// how long to sleep between checks

static int emcCommandWaitDone()
{
    double end;
    for (end = 0.0; end < doneTimeout; end += EMC_COMMAND_DELAY) {
	updateStatus();
	int serial_diff = emcStatus->echo_serial_number - emcCommandSerialNumber;

	if (serial_diff < 0) {
	    continue;
	}

	if (serial_diff > 0) {
	    return 0;
	}

	if (emcStatus->status == RCS_STATUS::DONE) {
	    return 0;
	}

	if (emcStatus->status == RCS_STATUS::ERROR) {
	    return -1;
	}

	esleep(EMC_COMMAND_DELAY);
    }

    return -1;
}

static int emcCommandSend(RCS_CMD_MSG & cmd)
{
    // write command
    if (emcCommandBuffer->write(&cmd)) {
        rtapi_print("halui: %s: error writing to Task\n", __func__);
        return -1;
    }
    emcCommandSerialNumber = cmd.serial_number;

    // wait for receive
    double end;
    for (end = 0.0; end < receiveTimeout; end += EMC_COMMAND_DELAY) {
	updateStatus();
	int serial_diff = emcStatus->echo_serial_number - emcCommandSerialNumber;

	if (serial_diff >= 0) {
	    return 0;
	}

	esleep(EMC_COMMAND_DELAY);
    }

    rtapi_print("halui: %s: no echo from Task after %.3f seconds\n", __func__, receiveTimeout);
    return -1;
}

static void thisQuit()
{
    //don't forget the big HAL sin ;)
    hal_exit(comp_id);

    if(emcCommandBuffer) { delete emcCommandBuffer;  emcCommandBuffer = 0; }
    if(emcStatusBuffer) { delete emcStatusBuffer;  emcStatusBuffer = 0; }
    if(emcErrorBuffer) { delete emcErrorBuffer;  emcErrorBuffer = 0; }
    exit(0);
}

static enum {
    LINEAR_UNITS_CUSTOM = 1,
    LINEAR_UNITS_AUTO,
    LINEAR_UNITS_MM,
    LINEAR_UNITS_INCH,
    LINEAR_UNITS_CM
} linearUnitConversion = LINEAR_UNITS_AUTO;

static enum {
    ANGULAR_UNITS_CUSTOM = 1,
    ANGULAR_UNITS_AUTO,
    ANGULAR_UNITS_DEG,
    ANGULAR_UNITS_RAD,
    ANGULAR_UNITS_GRAD
} angularUnitConversion = ANGULAR_UNITS_AUTO;

#define CLOSE(a,b,eps) ((a)-(b) < +(eps) && (a)-(b) > -(eps))
#define LINEAR_CLOSENESS 0.0001
#define ANGULAR_CLOSENESS 0.0001
#define CM_PER_MM 0.1
#define GRAD_PER_DEG (100.0/90.0)
#define RAD_PER_DEG TO_RAD	// from posemath.h

int halui_export_pin_IN_bit(hal_bit_t **pin, const char *name)
{
    int retval;
    retval = hal_pin_bit_new(name, HAL_IN, pin, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,"HALUI: ERROR: halui pin %s export failed with err=%i\n", name, retval);
	hal_exit(comp_id);
	return -1;
    }
    return 0;
}

int halui_export_pin_IN_s32(hal_s32_t **pin, const char *name)
{
    int retval;
    retval = hal_pin_s32_new(name, HAL_IN, pin, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,"HALUI: ERROR: halui pin %s export failed with err=%i\n", name, retval);
	hal_exit(comp_id);
	return -1;
    }
    return 0;
}

int halui_export_pin_IN_float(hal_float_t **pin, const char *name)
{
    int retval;
    retval = hal_pin_float_new(name, HAL_IN, pin, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,"HALUI: ERROR: halui pin %s export failed with err=%i\n", name, retval);
	hal_exit(comp_id);
	return -1;
    }
    return 0;
}


int halui_export_pin_OUT_bit(hal_bit_t **pin, const char *name)
{
    int retval;
    retval = hal_pin_bit_new(name, HAL_OUT, pin, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,"HALUI: ERROR: halui pin %s export failed with err=%i\n", name, retval);
	hal_exit(comp_id);
	return -1;
    }
    return 0;
}


/********************************************************************
*
* Description: halui_hal_init(void)
*
* Side Effects: Exports HAL pins.
*
* Called By: main
********************************************************************/
int halui_hal_init(void)
{
    int retval;
    int joint;
    int spindle;
    int axis_num;

    /* STEP 1: initialise the hal component */
    comp_id = hal_init("halui");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"HALUI: ERROR: hal_init() failed\n");
	return -1;
    }

    /* STEP 2: allocate shared memory for halui data */
    halui_data = (halui_str *) hal_malloc(sizeof(halui_str));
    if (halui_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"HALUI: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }

    /* STEP 3a: export the out-pin(s) */

    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->units_per_mm), comp_id, "halui.machine.units-per-mm");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->machine_is_on), "halui.machine.is-on");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->estop_is_activated), "halui.estop.is-activated");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->mode_is_manual), "halui.mode.is-manual");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->mode_is_auto), "halui.mode.is-auto");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->mode_is_mdi), "halui.mode.is-mdi");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->mode_is_teleop), "halui.mode.is-teleop");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->mode_is_joint), "halui.mode.is-joint");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->mist_is_on), "halui.mist.is-on");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->flood_is_on), "halui.flood.is-on");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->program_is_idle), "halui.program.is-idle");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->program_is_running), "halui.program.is-running");
    if (retval < 0) return retval;
    
    if (num_mdi_commands>0){
    retval = halui_export_pin_OUT_bit(&(halui_data->halui_mdi_is_running), "halui.halui-mdi-is-running");
    if (retval < 0) return retval;
	}
    
    retval = halui_export_pin_OUT_bit(&(halui_data->program_is_paused), "halui.program.is-paused");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->program_os_is_on), "halui.program.optional-stop.is-on");
    if (retval < 0) return retval;
    retval = halui_export_pin_OUT_bit(&(halui_data->program_bd_is_on), "halui.program.block-delete.is-on");

    for (spindle = 0; spindle < num_spindles; spindle++){
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_OUT, &(halui_data->spindle_is_on[spindle]), comp_id,  "halui.spindle.%i.is-on", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_OUT, &(halui_data->spindle_runs_forward[spindle]),comp_id,  "halui.spindle.%i.runs-forward", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_OUT, &(halui_data->spindle_runs_backward[spindle]), comp_id, "halui.spindle.%i.runs-backward", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_OUT, &(halui_data->spindle_brake_is_on[spindle]), comp_id, "halui.spindle.%i.brake-is-on", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_start[spindle]), comp_id, "halui.spindle.%i.start", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_stop[spindle]), comp_id, "halui.spindle.%i.stop", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_forward[spindle]), comp_id, "halui.spindle.%i.forward", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_reverse[spindle]), comp_id, "halui.spindle.%i.reverse", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_increase[spindle]), comp_id, "halui.spindle.%i.increase", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_decrease[spindle]), comp_id, "halui.spindle.%i.decrease", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_brake_on[spindle]), comp_id, "halui.spindle.%i.brake-on", spindle);
		if (retval < 0) return retval;
		retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->spindle_brake_off[spindle]), comp_id, "halui.spindle.%i.brake-off", spindle);
		if (retval < 0) return retval;
	    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->so_value[spindle]), comp_id, "halui.spindle.%i.override.value", spindle);
	    if (retval < 0) return retval;
	    retval = hal_pin_s32_newf(HAL_IN,  &(halui_data->so_counts[spindle]), comp_id, "halui.spindle.%i.override.counts", spindle);
	    if (retval < 0) return retval;
	    *halui_data->so_counts = 0;
	    retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->so_count_enable[spindle]), comp_id, "halui.spindle.%i.override.count-enable", spindle);
	    if (retval < 0) return retval;
	    *halui_data->so_count_enable[spindle] = 1;
	    retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->so_direct_value[spindle]), comp_id, "halui.spindle.%i.override.direct-value", spindle);
	    if (retval < 0) return retval;
	    *halui_data->so_direct_value[spindle] = 0;
	    retval = hal_pin_float_newf(HAL_IN,  &(halui_data->so_scale[spindle]), comp_id, "halui.spindle.%i.override.scale", spindle);
	    if (retval < 0) return retval;
	    retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->so_increase[spindle]), comp_id, "halui.spindle.%i.override.increase", spindle);
	    if (retval < 0) return retval;
	    retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->so_decrease[spindle]), comp_id, "halui.spindle.%i.override.decrease", spindle);
            if (retval < 0) return retval;
            retval = hal_pin_bit_newf(HAL_IN,  &(halui_data->so_reset[spindle]), comp_id, "halui.spindle.%i.override.reset", spindle);
    }

    for (joint=0; joint < num_joints ; joint++) {
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_is_homed[joint]), comp_id, "halui.joint.%d.is-homed", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_is_selected[joint]), comp_id, "halui.joint.%d.is-selected", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_soft_min_limit[joint]), comp_id, "halui.joint.%d.on-soft-min-limit", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_soft_max_limit[joint]), comp_id, "halui.joint.%d.on-soft-max-limit", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_hard_min_limit[joint]), comp_id, "halui.joint.%d.on-hard-min-limit", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_hard_max_limit[joint]), comp_id, "halui.joint.%d.on-hard-max-limit", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_override_limits[joint]), comp_id, "halui.joint.%d.override-limits", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_has_fault[joint]), comp_id, "halui.joint.%d.has-fault", joint);
	if (retval < 0) return retval;
    }

    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_soft_min_limit[num_joints]), comp_id, "halui.joint.selected.on-soft-min-limit");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_soft_max_limit[num_joints]), comp_id, "halui.joint.selected.on-soft-max-limit");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_hard_min_limit[num_joints]), comp_id, "halui.joint.selected.on-hard-min-limit");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_on_hard_max_limit[num_joints]), comp_id, "halui.joint.selected.on-hard-max-limit");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_override_limits[num_joints]), comp_id, "halui.joint.selected.override-limits");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_has_fault[num_joints]), comp_id, "halui.joint.selected.has-fault");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->joint_is_homed[num_joints]), comp_id, "halui.joint.selected.is-homed");
    if (retval < 0) return retval;

    for (axis_num=0; axis_num < EMCMOT_MAX_AXIS ; axis_num++) {
        if ( !(axis_mask & (1 << axis_num)) ) { continue; }
        char c = "xyzabcuvw"[axis_num];

        retval =  hal_pin_bit_newf(HAL_OUT, &(halui_data->axis_is_selected[axis_num]), comp_id, "halui.axis.%c.is-selected", c);
        if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->axis_pos_commanded[axis_num]), comp_id, "halui.axis.%c.pos-commanded", c);
        if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->axis_pos_feedback[axis_num]), comp_id, "halui.axis.%c.pos-feedback", c);
        if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->axis_pos_relative[axis_num]), comp_id, "halui.axis.%c.pos-relative", c);
        if (retval < 0) return retval;
    }

    // at startup, indicate [0] item is selected:
    *halui_data->joint_is_selected[0] = 1;
    *halui_data->axis_is_selected[0] = 1;

    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->mv_value), comp_id, "halui.max-velocity.value");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->fo_value), comp_id, "halui.feed-override.value");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->ro_value), comp_id, "halui.rapid-override.value");
    if (retval < 0) return retval;
    retval = hal_pin_u32_newf(HAL_OUT, &(halui_data->joint_selected), comp_id, "halui.joint.selected");
    if (retval < 0) return retval;
    retval = hal_pin_u32_newf(HAL_OUT, &(halui_data->axis_selected), comp_id, "halui.axis.selected");
    if (retval < 0) return retval;
    retval = hal_pin_u32_newf(HAL_OUT, &(halui_data->tool_number), comp_id, "halui.tool.number");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_x), comp_id, "halui.tool.length_offset.x");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_y), comp_id, "halui.tool.length_offset.y");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_z), comp_id, "halui.tool.length_offset.z");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_a), comp_id, "halui.tool.length_offset.a");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_b), comp_id, "halui.tool.length_offset.b");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_c), comp_id, "halui.tool.length_offset.c");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_u), comp_id, "halui.tool.length_offset.u");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_v), comp_id, "halui.tool.length_offset.v");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_length_offset_w), comp_id, "halui.tool.length_offset.w");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_OUT, &(halui_data->tool_diameter), comp_id, "halui.tool.diameter");
    if (retval < 0) return retval;

    /* STEP 3b: export the in-pin(s) */

    retval = halui_export_pin_IN_bit(&(halui_data->machine_on), "halui.machine.on");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->machine_off), "halui.machine.off");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->estop_activate), "halui.estop.activate");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->estop_reset), "halui.estop.reset");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mode_manual), "halui.mode.manual");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mode_auto), "halui.mode.auto");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mode_mdi), "halui.mode.mdi");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mode_teleop), "halui.mode.teleop");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mode_joint), "halui.mode.joint");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mist_on), "halui.mist.on");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mist_off), "halui.mist.off");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->flood_on), "halui.flood.on");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->flood_off), "halui.flood.off");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_run), "halui.program.run");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_pause), "halui.program.pause");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_resume), "halui.program.resume");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_step), "halui.program.step");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_stop), "halui.program.stop");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_os_on), "halui.program.optional-stop.on");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_os_off), "halui.program.optional-stop.off");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_bd_on), "halui.program.block-delete.on");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->program_bd_off), "halui.program.block-delete.off");
    if (retval < 0) return retval;

    retval = halui_export_pin_IN_s32(&(halui_data->mv_counts), "halui.max-velocity.counts");
    if (retval < 0) return retval;
    *halui_data->mv_counts = 0;
    retval = halui_export_pin_IN_bit(&(halui_data->mv_count_enable), "halui.max-velocity.count-enable");
    if (retval < 0) return retval;
    *halui_data->mv_count_enable = 1;
    retval = halui_export_pin_IN_bit(&(halui_data->mv_direct_value), "halui.max-velocity.direct-value");
    if (retval < 0) return retval;
    *halui_data->mv_direct_value = 0;
    retval = halui_export_pin_IN_float(&(halui_data->mv_scale), "halui.max-velocity.scale");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mv_increase), "halui.max-velocity.increase");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->mv_decrease), "halui.max-velocity.decrease");
    if (retval < 0) return retval;

    retval = halui_export_pin_IN_s32(&(halui_data->fo_counts), "halui.feed-override.counts");
    if (retval < 0) return retval;
    *halui_data->fo_counts = 0;
    retval = halui_export_pin_IN_bit(&(halui_data->fo_count_enable), "halui.feed-override.count-enable");
    if (retval < 0) return retval;
    *halui_data->fo_count_enable = 1;
    retval = halui_export_pin_IN_bit(&(halui_data->fo_direct_value), "halui.feed-override.direct-value");
    if (retval < 0) return retval;
    *halui_data->fo_direct_value = 0;
    retval = halui_export_pin_IN_float(&(halui_data->fo_scale), "halui.feed-override.scale");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->fo_increase), "halui.feed-override.increase");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->fo_decrease), "halui.feed-override.decrease");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->fo_reset), "halui.feed-override.reset");
    if (retval < 0) return retval;

    retval = halui_export_pin_IN_s32(&(halui_data->ro_counts), "halui.rapid-override.counts");
    if (retval < 0) return retval;
    *halui_data->ro_counts = 0;
    retval = halui_export_pin_IN_bit(&(halui_data->ro_count_enable), "halui.rapid-override.count-enable");
    if (retval < 0) return retval;
    *halui_data->ro_count_enable = 1;
    retval = halui_export_pin_IN_bit(&(halui_data->ro_direct_value), "halui.rapid-override.direct-value");
    if (retval < 0) return retval;
    *halui_data->ro_direct_value = 0;
    retval = halui_export_pin_IN_float(&(halui_data->ro_scale), "halui.rapid-override.scale");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->ro_increase), "halui.rapid-override.increase");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->ro_decrease), "halui.rapid-override.decrease");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_bit(&(halui_data->ro_reset), "halui.rapid-override.reset");
    if (retval < 0) return retval;

    if (have_home_all) {
        retval = halui_export_pin_IN_bit(&(halui_data->home_all), "halui.home-all");
        if (retval < 0) return retval;
    }

    retval = halui_export_pin_IN_bit(&(halui_data->abort), "halui.abort");
    if (retval < 0) return retval;

    for (joint=0; joint < num_joints ; joint++) {
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->joint_home[joint]), comp_id, "halui.joint.%d.home", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->joint_unhome[joint]), comp_id, "halui.joint.%d.unhome", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->joint_nr_select[joint]), comp_id, "halui.joint.%d.select", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_plus[joint]), comp_id, "halui.joint.%d.plus", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_minus[joint]), comp_id, "halui.joint.%d.minus", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_IN, &(halui_data->jjog_analog[joint]), comp_id, "halui.joint.%d.analog", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_IN, &(halui_data->jjog_increment[joint]), comp_id, "halui.joint.%d.increment", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_increment_plus[joint]), comp_id, "halui.joint.%d.increment-plus", joint);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_increment_minus[joint]), comp_id, "halui.joint.%d.increment-minus", joint);
	if (retval < 0) return retval;
    }

    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        char c = "xyzabcuvw"[axis_num];
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->axis_nr_select[axis_num]), comp_id, "halui.axis.%c.select", c);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_plus[axis_num]), comp_id, "halui.axis.%c.plus", c);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_minus[axis_num]), comp_id, "halui.axis.%c.minus", c);
	if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_IN, &(halui_data->ajog_analog[axis_num]), comp_id, "halui.axis.%c.analog", c);
	if (retval < 0) return retval;
	retval =  hal_pin_float_newf(HAL_IN, &(halui_data->ajog_increment[axis_num]), comp_id, "halui.axis.%c.increment", c);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_increment_plus[axis_num]), comp_id, "halui.axis.%c.increment-plus", c);
	if (retval < 0) return retval;
	retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_increment_minus[axis_num]), comp_id, "halui.axis.%c.increment-minus", c);
	if (retval < 0) return retval;
    }

    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->joint_home[num_joints]), comp_id, "halui.joint.selected.home");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->joint_unhome[num_joints]), comp_id, "halui.joint.selected.unhome");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_plus[num_joints]), comp_id, "halui.joint.selected.plus");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_minus[num_joints]), comp_id, "halui.joint.selected.minus");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_IN, &(halui_data->jjog_increment[num_joints]), comp_id, "halui.joint.selected.increment");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_increment_plus[num_joints]), comp_id, "halui.joint.selected.increment-plus");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->jjog_increment_minus[num_joints]), comp_id, "halui.joint.selected.increment-minus");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_plus[EMCMOT_MAX_AXIS]), comp_id, "halui.axis.selected.plus");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_minus[EMCMOT_MAX_AXIS]), comp_id, "halui.axis.selected.minus");
    if (retval < 0) return retval;
    retval =  hal_pin_float_newf(HAL_IN, &(halui_data->ajog_increment[EMCMOT_MAX_AXIS]), comp_id, "halui.axis.selected.increment");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_increment_plus[EMCMOT_MAX_AXIS]), comp_id, "halui.axis.selected.increment-plus");
    if (retval < 0) return retval;
    retval =  hal_pin_bit_newf(HAL_IN, &(halui_data->ajog_increment_minus[EMCMOT_MAX_AXIS]), comp_id, "halui.axis.selected.increment-minus");
    if (retval < 0) return retval;

    retval = halui_export_pin_IN_float(&(halui_data->jjog_speed), "halui.joint.jog-speed");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_float(&(halui_data->jjog_deadband), "halui.joint.jog-deadband");
    if (retval < 0) return retval;

    retval = halui_export_pin_IN_float(&(halui_data->ajog_speed), "halui.axis.jog-speed");
    if (retval < 0) return retval;
    retval = halui_export_pin_IN_float(&(halui_data->ajog_deadband), "halui.axis.jog-deadband");
    if (retval < 0) return retval;

    for (int n=0; n<num_mdi_commands; n++) {
        retval = hal_pin_bit_newf(HAL_IN, &(halui_data->mdi_commands[n]), comp_id, "halui.mdi-command-%02d", n);
        if (retval < 0) return retval;
    }

    hal_ready(comp_id);
    return 0;
}

static int sendMachineOn()
{
    EMC_TASK_SET_STATE state_msg;

    state_msg.state = EMC_TASK_STATE::ON;
    return emcCommandSend(state_msg);
}

static int sendMachineOff()
{
    EMC_TASK_SET_STATE state_msg;

    state_msg.state = EMC_TASK_STATE::OFF;
    return emcCommandSend(state_msg);
}

static int sendEstop()
{
    EMC_TASK_SET_STATE state_msg;

    state_msg.state = EMC_TASK_STATE::ESTOP;
    return emcCommandSend(state_msg);
}

static int sendEstopReset()
{
    EMC_TASK_SET_STATE state_msg;

    state_msg.state = EMC_TASK_STATE::ESTOP_RESET;
    return emcCommandSend(state_msg);
}

static int sendManual()
{
    EMC_TASK_SET_MODE mode_msg;

    if (emcStatus->task.mode == EMC_TASK_MODE::MANUAL) {
        return 0;
    }

    mode_msg.mode = EMC_TASK_MODE::MANUAL;
    return emcCommandSend(mode_msg);
}

static int sendAuto()
{
    EMC_TASK_SET_MODE mode_msg;

    if (emcStatus->task.mode == EMC_TASK_MODE::AUTO) {
        return 0;
    }

    mode_msg.mode = EMC_TASK_MODE::AUTO;
    return emcCommandSend(mode_msg);
}

static int sendMdi()
{
    EMC_TASK_SET_MODE mode_msg;

    if (emcStatus->task.mode == EMC_TASK_MODE::MDI) {
        return 0;
    }

    mode_msg.mode = EMC_TASK_MODE::MDI;
    return emcCommandSend(mode_msg);
}

static int sendMdiCommand(int n)
{
    EMC_TASK_PLAN_EXECUTE emc_task_plan_execute_msg;

    if (updateStatus()) {
	return -1;
    }

    if (!halui_sent_mdi) {
        // There is currently no MDI command from halui executing, we're
        // currently starting the first one.  Record what the Task mode is,
        // so we can restore it when all the MDI commands finish.
        halui_old_mode = emcStatus->task.mode;
    }
    
    halui_sent_mdi = 1;

    if (num_mdi_commands>0){
    *(halui_data->halui_mdi_is_running) = halui_sent_mdi;
    updateStatus();
    }
    
    // switch to MDI mode if needed
    if (emcStatus->task.mode != EMC_TASK_MODE::MDI) {
	if (sendMdi() != 0) {
            rtapi_print("halui: %s: failed to Set Mode MDI\n", __func__);
            return -1;
	}
	if (updateStatus() != 0) {
            rtapi_print("halui: %s: failed to update status\n", __func__);
	    return -1;
	}
	if (emcStatus->task.mode != EMC_TASK_MODE::MDI) {
            rtapi_print("halui: %s: switched mode, but got %d instead of mdi\n", __func__, (int)emcStatus->task.mode);
	    return -1;
	}
    }
    rtapi_strxcpy(emc_task_plan_execute_msg.command, mdi_commands[n]);
    if (emcCommandSend(emc_task_plan_execute_msg)) {
        rtapi_print("halui: %s: failed to send mdi command %d\n", __func__, n);
	return -1;
    }

    return 0;
}

static int sendTeleop()
{
    EMC_TRAJ_SET_TELEOP_ENABLE emc_set_teleop_enable_msg;

    emc_set_teleop_enable_msg.enable = 1;
    if (emcCommandSend(emc_set_teleop_enable_msg)) {
        return -1;
    }
    return emcCommandWaitDone();
}

static int sendJoint()
{
    EMC_TRAJ_SET_TELEOP_ENABLE emc_set_teleop_enable_msg;

    emc_set_teleop_enable_msg.enable = 0;
    if (emcCommandSend(emc_set_teleop_enable_msg)) {
        return -1;
    }
    return emcCommandWaitDone();
}

static int sendMistOn()
{
    EMC_COOLANT_MIST_ON emc_coolant_mist_on_msg;

    return emcCommandSend(emc_coolant_mist_on_msg);
}

static int sendMistOff()
{
    EMC_COOLANT_MIST_OFF emc_coolant_mist_off_msg;

    return emcCommandSend(emc_coolant_mist_off_msg);
}

static int sendFloodOn()
{
    EMC_COOLANT_FLOOD_ON emc_coolant_flood_on_msg;

    return emcCommandSend(emc_coolant_flood_on_msg);
}

static int sendFloodOff()
{
    EMC_COOLANT_FLOOD_OFF emc_coolant_flood_off_msg;

    return emcCommandSend(emc_coolant_flood_off_msg);
}

// programStartLine is the saved valued of the line that
// sendProgramRun(int line) sent
static int programStartLine = 0;

static int sendProgramRun(int line)
{
    EMC_TASK_PLAN_RUN emc_task_plan_run_msg;

    updateStatus();

    if (0 == emcStatus->task.file[0]) {
	return -1; // no program open
    }
    // save the start line, to compare against active line later
    programStartLine = line;

    emc_task_plan_run_msg.line = line;
    sendAuto();
    return emcCommandSend(emc_task_plan_run_msg);
}

static int sendProgramPause()
{
    EMC_TASK_PLAN_PAUSE emc_task_plan_pause_msg;

    return emcCommandSend(emc_task_plan_pause_msg);
}

static int sendSetOptionalStop(bool state)
{
    EMC_TASK_PLAN_SET_OPTIONAL_STOP emc_task_plan_set_optional_stop_msg;

    emc_task_plan_set_optional_stop_msg.state = state;
    return emcCommandSend(emc_task_plan_set_optional_stop_msg);
}

static int sendSetBlockDelete(bool state)
{
    EMC_TASK_PLAN_SET_BLOCK_DELETE emc_task_plan_set_block_delete_msg;

    emc_task_plan_set_block_delete_msg.state = state;
    return emcCommandSend(emc_task_plan_set_block_delete_msg);
}


static int sendProgramResume()
{
    EMC_TASK_PLAN_RESUME emc_task_plan_resume_msg;

    return emcCommandSend(emc_task_plan_resume_msg);
}

static int sendProgramStep()
{
    EMC_TASK_PLAN_STEP emc_task_plan_step_msg;

    return emcCommandSend(emc_task_plan_step_msg);
}

static int sendSpindleForward(int spindle)
{
    EMC_SPINDLE_ON emc_spindle_on_msg;
    emc_spindle_on_msg.spindle = spindle;
    if (emcStatus->task.activeSettings[2] != 0) {
	emc_spindle_on_msg.speed = fabs(emcStatus->task.activeSettings[2]);
    } else {
	emc_spindle_on_msg.speed = +1;
    }
    return emcCommandSend(emc_spindle_on_msg);
}

static int sendSpindleReverse(int spindle)
{
    EMC_SPINDLE_ON emc_spindle_on_msg;
    emc_spindle_on_msg.spindle = spindle;
    if (emcStatus->task.activeSettings[2] != 0) {
	emc_spindle_on_msg.speed =
	    -1 * fabs(emcStatus->task.activeSettings[2]);
    } else {
	emc_spindle_on_msg.speed = -1;
    }
    return emcCommandSend(emc_spindle_on_msg);
}

static int sendSpindleOff(int spindle)
{
    EMC_SPINDLE_OFF emc_spindle_off_msg;
    emc_spindle_off_msg.spindle = spindle;
    return emcCommandSend(emc_spindle_off_msg);
}

static int sendSpindleIncrease(int spindle)
{
    EMC_SPINDLE_INCREASE emc_spindle_increase_msg;
    emc_spindle_increase_msg.spindle = spindle;
    return emcCommandSend(emc_spindle_increase_msg);
}

static int sendSpindleDecrease(int spindle)
{
    EMC_SPINDLE_DECREASE emc_spindle_decrease_msg;
    emc_spindle_decrease_msg.spindle = spindle;
    return emcCommandSend(emc_spindle_decrease_msg);
}

static int sendSpindleConstant(int spindle)
{
    EMC_SPINDLE_CONSTANT emc_spindle_constant_msg;
    emc_spindle_constant_msg.spindle = spindle;
    return emcCommandSend(emc_spindle_constant_msg);
}

static int sendBrakeEngage(int spindle)
{
    EMC_SPINDLE_BRAKE_ENGAGE emc_spindle_brake_engage_msg;
    emc_spindle_brake_engage_msg.spindle = spindle;
    return emcCommandSend(emc_spindle_brake_engage_msg);
}

static int sendBrakeRelease(int spindle)
{
    EMC_SPINDLE_BRAKE_RELEASE emc_spindle_brake_release_msg;
    emc_spindle_brake_release_msg.spindle = spindle;
    return emcCommandSend(emc_spindle_brake_release_msg);
}

static int sendHome(int joint)
{
    EMC_JOINT_HOME emc_joint_home_msg;

    emc_joint_home_msg.joint = joint;
    return emcCommandSend(emc_joint_home_msg);
}

static int sendUnhome(int joint)
{
    EMC_JOINT_UNHOME emc_joint_unhome_msg;

    emc_joint_unhome_msg.joint = joint;
    return emcCommandSend(emc_joint_unhome_msg);
}

static int sendAbort()
{
    EMC_TASK_ABORT task_abort_msg;

    return emcCommandSend(task_abort_msg);
}


static void sendJogStop(int ja, int jjogmode)
{
    EMC_JOG_STOP emc_jog_stop_msg;

    if (   ( (jjogmode == JOGJOINT) && (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::TELEOP) )
        || ( (jjogmode == JOGTELEOP ) && (emcStatus->motion.traj.mode != EMC_TRAJ_MODE::TELEOP) )
       ) {
       return;
    }

    if (  jjogmode &&  (ja < 0 || ja >= num_joints)) { rtapi_print("halui: unexpected_1 %d\n",ja); return; }
    if ( !jjogmode &&  (ja < 0))                     { rtapi_print("halui: unexpected_2 %d\n",ja); return; }
    if ( !jjogmode && !(axis_mask & (1 << ja)) )     { rtapi_print("halui: unexpected_3 %d\n",ja); return; }

    emc_jog_stop_msg.jjogmode = jjogmode;
    emc_jog_stop_msg.joint_or_axis = ja;
    emcCommandSend(emc_jog_stop_msg);
}


static void sendJogCont(int ja, double speed, int jjogmode)
{
    EMC_JOG_CONT emc_jog_cont_msg;

    if (emcStatus->task.state != EMC_TASK_STATE::ON) { return; }
    if (   ( (jjogmode == JOGJOINT) && (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::TELEOP) )
        || ( (jjogmode == JOGTELEOP ) && (emcStatus->motion.traj.mode != EMC_TRAJ_MODE::TELEOP) )
       ) {
       return;
    }

    if (  jjogmode &&  (ja < 0 || ja >= num_joints)) { rtapi_print("halui: unexpected_4 %d\n",ja); return; }
    if ( !jjogmode &&  (ja < 0))                     { rtapi_print("halui: unexpected_5 %d\n",ja); return; }
    if ( !jjogmode && !(axis_mask & (1 << ja)) )     { rtapi_print("halui: unexpected_6 %d\n",ja); return; }

    emc_jog_cont_msg.jjogmode = jjogmode;
    emc_jog_cont_msg.joint_or_axis = ja;
    emc_jog_cont_msg.vel = speed / 60.0;

    emcCommandSend(emc_jog_cont_msg);
}

static void sendJogIncr(int ja, double speed, double incr, int jjogmode)
{
    EMC_JOG_INCR emc_jog_incr_msg;

    if (emcStatus->task.state != EMC_TASK_STATE::ON) { return; }
    if (   ( (jjogmode == JOGJOINT) && (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::TELEOP) )
        || ( (jjogmode == JOGTELEOP ) && (emcStatus->motion.traj.mode != EMC_TRAJ_MODE::TELEOP) )
       ) {
       return;
    }

    if (  jjogmode &&  (ja < 0 || ja >= num_joints)) { rtapi_print("halui: unexpected_7 %d\n",ja); return; }
    if ( !jjogmode &&  (ja < 0))                     { rtapi_print("halui: unexpected_8 %d\n",ja); return; }
    if ( !jjogmode && !(axis_mask & (1 << ja)) )     { rtapi_print("halui: unexpected_9 %d\n",ja); return; }

    emc_jog_incr_msg.jjogmode = jjogmode;
    emc_jog_incr_msg.joint_or_axis = ja;
    emc_jog_incr_msg.vel = speed / 60.0;
    emc_jog_incr_msg.incr = incr;

    emcCommandSend(emc_jog_incr_msg);
}

static int sendFeedOverride(double override)
{
    EMC_TRAJ_SET_SCALE emc_traj_set_scale_msg;

    if (override < 0.0) {
	override = 0.0;
    }

    if (override > maxFeedOverride) {
	override = maxFeedOverride;
    }

    emc_traj_set_scale_msg.scale = override;
    return emcCommandSend(emc_traj_set_scale_msg);
}

static int sendRapidOverride(double override)
{
    EMC_TRAJ_SET_RAPID_SCALE emc_traj_set_scale_msg;

    if (override < 0.0) {
	override = 0.0;
    }

    if (override > 1.0) {
	override = 1.0;
    }

    emc_traj_set_scale_msg.scale = override;
    return emcCommandSend(emc_traj_set_scale_msg);
}

static int sendMaxVelocity(double velocity)
{
    EMC_TRAJ_SET_MAX_VELOCITY mv;

    if (velocity < 0.0) {
        velocity = 0.0;
    }

    if (velocity > maxMaxVelocity) {
        velocity = maxMaxVelocity;
    }

    mv.velocity = velocity;
    return emcCommandSend(mv);
}

static int sendSpindleOverride(int spindle, double override)
{
    EMC_TRAJ_SET_SPINDLE_SCALE emc_traj_set_spindle_scale_msg;

    if (override < minSpindleOverride) {
	override = minSpindleOverride;
    }

    if (override > maxSpindleOverride) {
	override = maxSpindleOverride;
    }

    emc_traj_set_spindle_scale_msg.spindle = spindle;
    emc_traj_set_spindle_scale_msg.scale = override;
    return emcCommandSend(emc_traj_set_spindle_scale_msg);
}

static int iniLoad(const char *filename)
{
    IniFile inifile;
    std::optional<const char*> inistring;
    char version[LINELEN], machine[LINELEN];
    double d;
    int i;

    // open it
    if (inifile.Open(filename) == false) {
	return -1;
    }

    // EMC debugging flags
	emc_debug = 0;  // disabled by default
    if ((inistring = inifile.Find("DEBUG", "EMC"))) {
        // parse to global
        if (sscanf(*inistring, "%x", &emc_debug) < 1) {
            perror("failed to parse [EMC] DEBUG");
        }
    }

    // set output for RCS messages
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);   // use stdout by default
    if ((inistring = inifile.Find("RCS_DEBUG_DEST", "EMC"))) {
        static RCS_PRINT_DESTINATION_TYPE type;
        if (!strcmp(*inistring, "STDOUT")) {
            type = RCS_PRINT_TO_STDOUT;
        } else if (!strcmp(*inistring, "STDERR")) {
            type = RCS_PRINT_TO_STDERR;
        } else if (!strcmp(*inistring, "FILE")) {
            type = RCS_PRINT_TO_FILE;
        } else if (!strcmp(*inistring, "LOGGER")) {
            type = RCS_PRINT_TO_LOGGER;
        } else if (!strcmp(*inistring, "MSGBOX")) {
            type = RCS_PRINT_TO_MESSAGE_BOX;
        } else if (!strcmp(*inistring, "NULL")) {
            type = RCS_PRINT_TO_NULL;
        } else {
             type = RCS_PRINT_TO_STDOUT;
        }
        set_rcs_print_destination(type);
    }

    // NML/RCS debugging flags
    set_rcs_print_flag(PRINT_RCS_ERRORS);  // only print errors by default
    // enable all debug messages by default if RCS or NML debugging is enabled
    if ((emc_debug & EMC_DEBUG_RCS) || (emc_debug & EMC_DEBUG_NML)) {
        // output all RCS debug messages
        set_rcs_print_flag(PRINT_EVERYTHING);
    }

    // set flags if RCS_DEBUG in ini file
    if ((inistring = inifile.Find("RCS_DEBUG", "EMC"))) {
        static long int flags;
        if (sscanf(*inistring, "%lx", &flags) < 1) {
            perror("failed to parse [EMC] RCS_DEBUG");
        }
        // clear all flags
        clear_rcs_print_flag(PRINT_EVERYTHING);
        // set parsed flags
        set_rcs_print_flag(flags);
    }
    // output infinite RCS errors by default
    max_rcs_errors_to_print = -1;
    if ((inistring = inifile.Find("RCS_MAX_ERR", "EMC"))) {
        if (sscanf(*inistring, "%d", &max_rcs_errors_to_print) < 1) {
            perror("failed to parse [EMC] RCS_MAX_ERR");
        }
    }

    strncpy(version, "unknown", LINELEN-1);
    if ((inistring = inifile.Find("VERSION", "EMC"))) {
	    strncpy(version, *inistring, LINELEN-1);
    }


    if ((inistring = inifile.Find("MACHINE", "EMC"))) {
	    strncpy(machine, *inistring, LINELEN-1);
    } else {
	    strncpy(machine, "unknown", LINELEN-1);
    }

    extern char *program_invocation_short_name;
    rcs_print(
        "%s (%d) halui: machine '%s'  version '%s'\n",
        program_invocation_short_name, getpid(), machine, version
    );

    if ((inistring = inifile.Find("NML_FILE", "EMC"))) {
	// copy to global
	rtapi_strxcpy(emc_nmlfile, *inistring);
    } else {
	// not found, use default
    }

    if ((inistring = inifile.Find("MAX_FEED_OVERRIDE", "DISPLAY"))) {
	if (1 == sscanf(*inistring, "%lf", &d) && d > 0.0) {
	    maxFeedOverride =  d;
	}
    }

    if(inifile.Find(&maxMaxVelocity, "MAX_LINEAR_VELOCITY", "TRAJ") &&
       inifile.Find(&maxMaxVelocity, "MAX_VELOCITY", "AXIS_X"))
        maxMaxVelocity = 1.0;

    if ((inistring = inifile.Find("MIN_SPINDLE_OVERRIDE", "DISPLAY"))) {
	if (1 == sscanf(*inistring, "%lf", &d) && d > 0.0) {
	    minSpindleOverride =  d;
	}
    }

    if ((inistring = inifile.Find("MAX_SPINDLE_OVERRIDE", "DISPLAY"))) {
	if (1 == sscanf(*inistring, "%lf", &d) && d > 0.0) {
	    maxSpindleOverride =  d;
	}
    }

    inistring = inifile.Find("COORDINATES", "TRAJ");
    num_axes = 0;
    if (inistring) {
        if(strchr(*inistring, 'x') || strchr(*inistring, 'X')) { axis_mask |= 0x0001; num_axes++; }
        if(strchr(*inistring, 'y') || strchr(*inistring, 'Y')) { axis_mask |= 0x0002; num_axes++; }
        if(strchr(*inistring, 'z') || strchr(*inistring, 'Z')) { axis_mask |= 0x0004; num_axes++; }
        if(strchr(*inistring, 'a') || strchr(*inistring, 'A')) { axis_mask |= 0x0008; num_axes++; }
        if(strchr(*inistring, 'b') || strchr(*inistring, 'B')) { axis_mask |= 0x0010; num_axes++; }
        if(strchr(*inistring, 'c') || strchr(*inistring, 'C')) { axis_mask |= 0x0020; num_axes++; }
        if(strchr(*inistring, 'u') || strchr(*inistring, 'U')) { axis_mask |= 0x0040; num_axes++; }
        if(strchr(*inistring, 'v') || strchr(*inistring, 'V')) { axis_mask |= 0x0080; num_axes++; }
        if(strchr(*inistring, 'w') || strchr(*inistring, 'W')) { axis_mask |= 0x0100; num_axes++; }
    }
    if (num_axes ==0) {
       rcs_print("halui: no [TRAJ]COORDINATES specified, enabling all axes\n");
       num_axes = EMCMOT_MAX_AXIS;
       axis_mask = 0xFFFF;
    }

    if ((inistring = inifile.Find("JOINTS", "KINS"))) {
        if (1 == sscanf(*inistring, "%d", &i) && i > 0) {
            num_joints =  i;
        }
    }

    if ((inistring = inifile.Find("SPINDLES", "TRAJ"))) {
        if (1 == sscanf(*inistring, "%d", &i) && i > 0) {
            num_spindles =  i;
        }
    }

    if (inifile.Find("HOME_SEQUENCE", "JOINT_0")) {
        have_home_all = 1;
    }

    if ((inistring = inifile.Find("LINEAR_UNITS", "DISPLAY"))) {
	if (!strcmp(*inistring, "AUTO")) {
	    linearUnitConversion = LINEAR_UNITS_AUTO;
	} else if (!strcmp(*inistring, "INCH")) {
	    linearUnitConversion = LINEAR_UNITS_INCH;
	} else if (!strcmp(*inistring, "MM")) {
	    linearUnitConversion = LINEAR_UNITS_MM;
	} else if (!strcmp(*inistring, "CM")) {
	    linearUnitConversion = LINEAR_UNITS_CM;
	}
    }

    if ((inistring = inifile.Find("ANGULAR_UNITS", "DISPLAY"))) {
	if (!strcmp(*inistring, "AUTO")) {
	    angularUnitConversion = ANGULAR_UNITS_AUTO;
	} else if (!strcmp(*inistring, "DEG")) {
	    angularUnitConversion = ANGULAR_UNITS_DEG;
	} else if (!strcmp(*inistring, "RAD")) {
	    angularUnitConversion = ANGULAR_UNITS_RAD;
	} else if (!strcmp(*inistring, "GRAD")) {
	    angularUnitConversion = ANGULAR_UNITS_GRAD;
	}
    }

    std::optional<const char*> mc;
    while(num_mdi_commands < MDI_MAX && (mc = inifile.Find("MDI_COMMAND", "HALUI", num_mdi_commands+1))) {
        mdi_commands[num_mdi_commands++] = strdup(*mc);
    }

    // close it
    inifile.Close();

    return 0;
}

static void hal_init_pins()
{
    int joint;
    int axis_num;
    int spindle;

    *(halui_data->machine_on) = old_halui_data.machine_on = 0;
    *(halui_data->machine_off) = old_halui_data.machine_off = 0;

    *(halui_data->estop_activate) = old_halui_data.estop_activate = 0;
    *(halui_data->estop_reset) = old_halui_data.estop_reset = 0;


    for (joint=0; joint < num_joints; joint++) {
	*(halui_data->joint_home[joint]) = old_halui_data.joint_home[joint] = 0;
	*(halui_data->joint_unhome[joint]) = old_halui_data.joint_unhome[joint] = 0;
	*(halui_data->joint_nr_select[joint]) = old_halui_data.joint_nr_select[joint] = 0;
	*(halui_data->jjog_minus[joint]) = old_halui_data.jjog_minus[joint] = 0;
	*(halui_data->jjog_plus[joint]) = old_halui_data.jjog_plus[joint] = 0;
	*(halui_data->jjog_analog[joint]) = old_halui_data.jjog_analog[joint] = 0;
	*(halui_data->jjog_increment[joint]) = old_halui_data.jjog_increment[joint] = 0.0;
	*(halui_data->jjog_increment_plus[joint]) = old_halui_data.jjog_increment_plus[joint] = 0;
	*(halui_data->jjog_increment_minus[joint]) = old_halui_data.jjog_increment_minus[joint] = 0;
    }

    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        if ( !(axis_mask & (1 << axis_num)) ) { continue; }
        *(halui_data->axis_nr_select[axis_num]) = old_halui_data.axis_nr_select[axis_num] = 0;
	*(halui_data->ajog_minus[axis_num]) = old_halui_data.ajog_minus[axis_num] = 0;
	*(halui_data->ajog_plus[axis_num]) = old_halui_data.ajog_plus[axis_num] = 0;
	*(halui_data->ajog_analog[axis_num]) = old_halui_data.ajog_analog[axis_num] = 0;
	*(halui_data->ajog_increment[axis_num]) = old_halui_data.ajog_increment[axis_num] = 0.0;
	*(halui_data->ajog_increment_plus[axis_num]) = old_halui_data.ajog_increment_plus[axis_num] = 0;
	*(halui_data->ajog_increment_minus[axis_num]) = old_halui_data.ajog_increment_minus[axis_num] = 0;
    }

    *(halui_data->joint_home[num_joints]) = old_halui_data.joint_home[num_joints] = 0;
    *(halui_data->jjog_minus[num_joints]) = old_halui_data.jjog_minus[num_joints] = 0;
    *(halui_data->jjog_plus[num_joints]) = old_halui_data.jjog_plus[num_joints] = 0;
    *(halui_data->jjog_increment[num_joints]) = old_halui_data.jjog_increment[num_joints] = 0.0;
    *(halui_data->jjog_increment_plus[num_joints]) = old_halui_data.jjog_increment_plus[num_joints] = 0;
    *(halui_data->jjog_increment_minus[num_joints]) = old_halui_data.jjog_increment_minus[num_joints] = 0;
    *(halui_data->jjog_deadband) = 0.2;
    *(halui_data->jjog_speed) = 0;
    *(halui_data->ajog_minus[EMCMOT_MAX_AXIS]) = old_halui_data.ajog_minus[EMCMOT_MAX_AXIS] = 0;
    *(halui_data->ajog_plus[EMCMOT_MAX_AXIS]) = old_halui_data.ajog_plus[EMCMOT_MAX_AXIS] = 0;
    *(halui_data->ajog_increment[EMCMOT_MAX_AXIS]) = old_halui_data.ajog_increment[EMCMOT_MAX_AXIS] = 0.0;
    *(halui_data->ajog_increment_plus[EMCMOT_MAX_AXIS]) = old_halui_data.ajog_increment_plus[EMCMOT_MAX_AXIS] = 0;
    *(halui_data->ajog_increment_minus[EMCMOT_MAX_AXIS]) = old_halui_data.ajog_increment_minus[EMCMOT_MAX_AXIS] = 0;
    *(halui_data->ajog_deadband) = 0.2;
    *(halui_data->ajog_speed) = 0;

    *(halui_data->joint_selected) = 0; // select joint 0 by default
    *(halui_data->axis_selected) = 0; // select axis 0 by default

    *(halui_data->fo_scale) = old_halui_data.fo_scale = 0.1; //sane default
    *(halui_data->ro_scale) = old_halui_data.ro_scale = 0.1; //sane default
    for (spindle = 0; spindle < num_spindles; spindle++){
        *(halui_data->so_scale[spindle]) = old_halui_data.so_scale[spindle] = 0.1; //sane default
        *(halui_data->so_increase[spindle]) = old_halui_data.so_increase[spindle] = 0;
        *(halui_data->so_decrease[spindle]) = old_halui_data.so_decrease[spindle] = 0;
        *(halui_data->spindle_increase[spindle]) = old_halui_data.spindle_increase[spindle] = 0;
        *(halui_data->spindle_decrease[spindle]) = old_halui_data.spindle_decrease[spindle] = 0;
    }
}

static int check_bit_changed(bool halpin, bool &newpin)
{
    if (halpin != newpin) {
	newpin = halpin;
	return halpin;
    }
    return 0;
}

static void copy_hal_data(const halui_str &i, local_halui_str &j)
{
    int x;
#define FIELD(t,f) j.f = (i.f)?*i.f:0;
#define ARRAY(t,f,n) do { for (x = 0; x < n; x++) j.f[x] = (i.f[x])?*i.f[x]:0; } while (0);
    HAL_FIELDS
#undef FIELD
#undef ARRAY
}


// Returns true if any of halui.JA.N.plus, halui.JA.N.minus, or
// halui.JA.N.analog are true (where JA is joint or axis and
// N is the passed-in joint/axis number). Otherwise, returns false.
static bool jogging_joint(local_halui_str &hal, int joint) {
    return (hal.jjog_plus[joint] || hal.jjog_minus[joint] || hal.jjog_analog[joint]);
}
static bool jogging_axis(local_halui_str &hal, int axis_num) {
    return (hal.ajog_plus[axis_num] || hal.ajog_minus[axis_num] || hal.ajog_analog[axis_num]);
}


// Returns true if any of halui.JA.selected.plus,
// halui.JA.selected.minus, or halui.JA.selected.analog are true.
// JA == joint or axis as appropriate
// Otherwise, returns false.
static bool jogging_selected_joint(local_halui_str &hal) {
    return (hal.jjog_plus[num_joints] || hal.jjog_minus[num_joints]);
}
static bool jogging_selected_axis(local_halui_str &hal) {
    return (hal.ajog_plus[EMCMOT_MAX_AXIS] || hal.ajog_minus[EMCMOT_MAX_AXIS]);
}


// this function looks if any of the hal pins has changed
// and sends appropriate messages if so
static void check_hal_changes()
{
    hal_s32_t counts;
    int jselect_changed, joint;
    int aselect_changed, axis_num;
    hal_bit_t bit;
    int js;
    hal_float_t floatt;
    int jjog_speed_changed;
    int ajog_speed_changed;

    local_halui_str new_halui_data_mutable;
    copy_hal_data(*halui_data, new_halui_data_mutable);
    const local_halui_str &new_halui_data = new_halui_data_mutable;


    //check if machine_on pin has changed (the rest work exactly the same)
    if (check_bit_changed(new_halui_data.machine_on, old_halui_data.machine_on) != 0)
	sendMachineOn();                //send MachineOn NML command

    if (check_bit_changed(new_halui_data.machine_off, old_halui_data.machine_off) != 0)
	sendMachineOff();

    if (check_bit_changed(new_halui_data.estop_activate, old_halui_data.estop_activate) != 0)
	sendEstop();

    if (check_bit_changed(new_halui_data.estop_reset, old_halui_data.estop_reset) != 0)
	sendEstopReset();

    if (check_bit_changed(new_halui_data.mode_manual, old_halui_data.mode_manual) != 0)
	sendManual();

    if (check_bit_changed(new_halui_data.mode_auto, old_halui_data.mode_auto) != 0)
	sendAuto();

    if (check_bit_changed(new_halui_data.mode_mdi, old_halui_data.mode_mdi) != 0)
	sendMdi();

    if (check_bit_changed(new_halui_data.mode_teleop, old_halui_data.mode_teleop) != 0)
	sendTeleop();

    if (check_bit_changed(new_halui_data.mode_joint, old_halui_data.mode_joint) != 0)
	sendJoint();

    if (check_bit_changed(new_halui_data.mist_on, old_halui_data.mist_on) != 0)
	sendMistOn();

    if (check_bit_changed(new_halui_data.mist_off, old_halui_data.mist_off) != 0)
	sendMistOff();

    if (check_bit_changed(new_halui_data.flood_on, old_halui_data.flood_on) != 0)
	sendFloodOn();

    if (check_bit_changed(new_halui_data.flood_off, old_halui_data.flood_off) != 0)
	sendFloodOff();

    if (check_bit_changed(new_halui_data.program_run, old_halui_data.program_run) != 0)
	sendProgramRun(0);

    if (check_bit_changed(new_halui_data.program_pause, old_halui_data.program_pause) != 0)
	sendProgramPause();

    if (check_bit_changed(new_halui_data.program_os_on, old_halui_data.program_os_on) != 0)
	sendSetOptionalStop(ON);

    if (check_bit_changed(new_halui_data.program_os_off, old_halui_data.program_os_off) != 0)
	sendSetOptionalStop(OFF);

    if (check_bit_changed(new_halui_data.program_bd_on, old_halui_data.program_bd_on) != 0)
	sendSetBlockDelete(ON);

    if (check_bit_changed(new_halui_data.program_bd_off, old_halui_data.program_bd_off) != 0)
	sendSetBlockDelete(OFF);

    if (check_bit_changed(new_halui_data.program_resume, old_halui_data.program_resume) != 0)
	sendProgramResume();

    if (check_bit_changed(new_halui_data.program_step, old_halui_data.program_step) != 0)
	sendProgramStep();

    if (check_bit_changed(new_halui_data.program_stop, old_halui_data.program_stop) != 0)
	sendAbort();

    //max-velocity stuff
    counts = new_halui_data.mv_counts;
    if (counts != old_halui_data.mv_counts) {
        if (new_halui_data.mv_count_enable) {
            if (new_halui_data.mv_direct_value) {
                sendMaxVelocity(counts * new_halui_data.mv_scale);
            } else {
                sendMaxVelocity( new_halui_data.mv_value + (counts - old_halui_data.mv_counts) *
                    new_halui_data.mv_scale);
            }
        }
        old_halui_data.mv_counts = counts;
    }

    //feed-override stuff
    counts = new_halui_data.fo_counts;
    if (counts != old_halui_data.fo_counts) {
        if (new_halui_data.fo_count_enable) {
            if (new_halui_data.fo_direct_value) {
                sendFeedOverride(counts * new_halui_data.fo_scale);
            } else {
                sendFeedOverride( new_halui_data.fo_value + (counts - old_halui_data.fo_counts) *
                    new_halui_data.fo_scale);
            }
        }
        old_halui_data.fo_counts = counts;
    }

    //rapid-override stuff
    counts = new_halui_data.ro_counts;
    if (counts != old_halui_data.ro_counts) {
        if (new_halui_data.ro_count_enable) {
            if (new_halui_data.ro_direct_value) {
                sendRapidOverride(counts * new_halui_data.ro_scale);
            } else {
                sendRapidOverride( new_halui_data.ro_value + (counts - old_halui_data.ro_counts) *
                    new_halui_data.ro_scale);
            }
        }
        old_halui_data.ro_counts = counts;
    }

    //spindle-override stuff
    for (int spindle = 0; spindle < num_spindles; spindle++){
		counts = new_halui_data.so_counts[spindle];
		if (counts != old_halui_data.so_counts[spindle]) {
			if (new_halui_data.so_count_enable[spindle]) {
				if (new_halui_data.so_direct_value[spindle]) {
					sendSpindleOverride(spindle, counts * new_halui_data.so_scale[spindle]);
				} else {
					sendSpindleOverride(spindle, new_halui_data.so_value[spindle] + (counts - old_halui_data.so_counts[spindle]) *
						new_halui_data.so_scale[spindle]);
				}
			}
			old_halui_data.so_counts[spindle] = counts;
		}
    }

    if (check_bit_changed(new_halui_data.mv_increase, old_halui_data.mv_increase) != 0)
        sendMaxVelocity(new_halui_data.mv_value + new_halui_data.mv_scale);
    if (check_bit_changed(new_halui_data.mv_decrease, old_halui_data.mv_decrease) != 0)
        sendMaxVelocity(new_halui_data.mv_value - new_halui_data.mv_scale);

    if (check_bit_changed(new_halui_data.fo_increase, old_halui_data.fo_increase) != 0)
        sendFeedOverride(new_halui_data.fo_value + new_halui_data.fo_scale);
    if (check_bit_changed(new_halui_data.fo_decrease, old_halui_data.fo_decrease) != 0)
        sendFeedOverride(new_halui_data.fo_value - new_halui_data.fo_scale);
    if (check_bit_changed(new_halui_data.fo_reset, old_halui_data.fo_reset) != 0)
        sendFeedOverride( 1.0 );

    if (check_bit_changed(new_halui_data.ro_increase, old_halui_data.ro_increase) != 0)
        sendRapidOverride(new_halui_data.ro_value + new_halui_data.ro_scale);
    if (check_bit_changed(new_halui_data.ro_decrease, old_halui_data.ro_decrease) != 0)
        sendRapidOverride(new_halui_data.ro_value - new_halui_data.ro_scale);
    if (check_bit_changed(new_halui_data.ro_reset, old_halui_data.ro_reset) != 0)
        sendRapidOverride( 1.0 );

	// spindle stuff
    for (int spindle = 0; spindle < num_spindles; spindle++){
		if (check_bit_changed(new_halui_data.so_increase[spindle], old_halui_data.so_increase[spindle]) != 0)
			sendSpindleOverride(spindle, new_halui_data.so_value[spindle] + new_halui_data.so_scale[spindle]);
		if (check_bit_changed(new_halui_data.so_decrease[spindle], old_halui_data.so_decrease[spindle]) != 0)
			sendSpindleOverride(spindle, new_halui_data.so_value[spindle] - new_halui_data.so_scale[spindle]);
                if (check_bit_changed(new_halui_data.so_reset[spindle], old_halui_data.so_reset[spindle]) != 0)
                        sendSpindleOverride(spindle, 1.0 );

		if (check_bit_changed(new_halui_data.spindle_start[spindle], old_halui_data.spindle_start[spindle]) != 0)
		sendSpindleForward(spindle);

		if (check_bit_changed(new_halui_data.spindle_stop[spindle], old_halui_data.spindle_stop[spindle]) != 0)
		sendSpindleOff(spindle);

		if (check_bit_changed(new_halui_data.spindle_forward[spindle], old_halui_data.spindle_forward[spindle]) != 0)
		sendSpindleForward(spindle);

		if (check_bit_changed(new_halui_data.spindle_reverse[spindle], old_halui_data.spindle_reverse[spindle]) != 0)
		sendSpindleReverse(spindle);

		bit = new_halui_data.spindle_increase[spindle];
		if (bit != old_halui_data.spindle_increase[spindle]) {
		if (bit != 0)
			sendSpindleIncrease(spindle);
		if (bit == 0)
			sendSpindleConstant(spindle);
		old_halui_data.spindle_increase[spindle]= bit;
		}

		bit = new_halui_data.spindle_decrease[spindle];
		if (bit != old_halui_data.spindle_decrease[spindle]) {
		if (bit != 0)
			sendSpindleDecrease(spindle);
		if (bit == 0)
			sendSpindleConstant(spindle);
		old_halui_data.spindle_decrease[spindle]= bit;
		}

		if (check_bit_changed(new_halui_data.spindle_brake_on[spindle], old_halui_data.spindle_brake_on[spindle]) != 0)
		sendBrakeEngage(spindle);

		if (check_bit_changed(new_halui_data.spindle_brake_off[spindle], old_halui_data.spindle_brake_off[spindle]) != 0)
		sendBrakeRelease(spindle);
    }

	if (check_bit_changed(new_halui_data.abort, old_halui_data.abort) != 0)
	sendAbort();

	if (check_bit_changed(new_halui_data.home_all, old_halui_data.home_all) != 0)
	sendHome(-1);

// joint stuff (selection, homing..)
    jselect_changed = -1; // flag to see if the selected joint changed

    // if the jog-speed changes while in a continuous jog, we want to
    // re-start the jog with the new speed
    if (fabs(old_halui_data.jjog_speed - new_halui_data.jjog_speed) > 0.00001) {
        old_halui_data.jjog_speed = new_halui_data.jjog_speed;
        jjog_speed_changed = 1;
    } else {
        jjog_speed_changed = 0;
    }
// axis stuff (selection, homing..)
    aselect_changed = -1; // flag to see if the selected joint changed

    // if the jog-speed changes while in a continuous jog, we want to
    // re-start the jog with the new speed
    if (fabs(old_halui_data.ajog_speed - new_halui_data.ajog_speed) > 0.00001) {
        old_halui_data.ajog_speed = new_halui_data.ajog_speed;
        ajog_speed_changed = 1;
    } else {
        ajog_speed_changed = 0;
    }

    for (joint=0; joint < num_joints; joint++) {
	if (check_bit_changed(new_halui_data.joint_home[joint], old_halui_data.joint_home[joint]) != 0)
	    sendHome(joint);

	if (check_bit_changed(new_halui_data.joint_unhome[joint], old_halui_data.joint_unhome[joint]) != 0)
	    sendUnhome(joint);

	bit = new_halui_data.jjog_minus[joint];
	if ((bit != old_halui_data.jjog_minus[joint]) || (bit && jjog_speed_changed)) {
	    if (bit != 0)
		sendJogCont(joint,-new_halui_data.jjog_speed,JOGJOINT);
	    else
		sendJogStop(joint,JOGJOINT);
	    old_halui_data.jjog_minus[joint] = bit;
	}

	bit = new_halui_data.jjog_plus[joint];
	if ((bit != old_halui_data.jjog_plus[joint]) || (bit && jjog_speed_changed)) {
	    if (bit != 0)
		sendJogCont(joint,new_halui_data.jjog_speed,JOGJOINT);
	    else
		sendJogStop(joint,JOGJOINT);
	    old_halui_data.jjog_plus[joint] = bit;
	}

	floatt = new_halui_data.jjog_analog[joint];
	bit = (fabs(floatt) > new_halui_data.jjog_deadband);
	if ((floatt != old_halui_data.jjog_analog[joint]) || (bit && jjog_speed_changed)) {
	    if (bit)
		sendJogCont(joint,(new_halui_data.jjog_speed) * (new_halui_data.jjog_analog[joint]),JOGJOINT);
	    else
		sendJogStop(joint,JOGJOINT);
	    old_halui_data.jjog_analog[joint] = floatt;
	}

	bit = new_halui_data.jjog_increment_plus[joint];
	if (bit != old_halui_data.jjog_increment_plus[joint]) {
	    if (bit)
		sendJogIncr(joint, new_halui_data.jjog_speed, new_halui_data.jjog_increment[joint],JOGJOINT);
	    old_halui_data.jjog_increment_plus[joint] = bit;
	}

	bit = new_halui_data.jjog_increment_minus[joint];
	if (bit != old_halui_data.jjog_increment_minus[joint]) {
	    if (bit)
		sendJogIncr(joint, new_halui_data.jjog_speed, -(new_halui_data.jjog_increment[joint]),JOGJOINT);
	    old_halui_data.jjog_increment_minus[joint] = bit;
	}

	// check to see if another joint has been selected
	bit = new_halui_data.joint_nr_select[joint];
	if (bit != old_halui_data.joint_nr_select[joint]) {
	    if (bit != 0) {
		*halui_data->joint_selected = joint;
		jselect_changed = joint; // flag that we changed the selected joint
	    }
	    old_halui_data.joint_nr_select[joint] = bit;
	}

    }

    if (jselect_changed >= 0) {
	for (joint = 0; joint < num_joints; joint++) {
	    if (joint != jselect_changed) {
		*(halui_data->joint_is_selected[joint]) = 0;
                if (jogging_selected_joint(old_halui_data) && !jogging_joint(old_halui_data, joint)) {
                    sendJogStop(joint,JOGJOINT);
                }
            } else {
		*(halui_data->joint_is_selected[joint]) = 1;
                if (*halui_data->jjog_plus[num_joints]) {
                    sendJogCont(joint, new_halui_data.jjog_speed,JOGJOINT);
                } else if (*halui_data->jjog_minus[num_joints]) {
                    sendJogCont(joint, -new_halui_data.jjog_speed,JOGJOINT);
                }
	    }
	}
    }

    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        if ( !(axis_mask & (1 << axis_num)) ) { continue; }
	bit = new_halui_data.ajog_minus[axis_num];
	if ((bit != old_halui_data.ajog_minus[axis_num]) || (bit && ajog_speed_changed)) {
	    if (bit != 0)
		sendJogCont(axis_num,-new_halui_data.ajog_speed,JOGTELEOP);
	    else
		sendJogStop(axis_num,JOGTELEOP);
	    old_halui_data.ajog_minus[axis_num] = bit;
	}

	bit = new_halui_data.ajog_plus[axis_num];
	if ((bit != old_halui_data.ajog_plus[axis_num]) || (bit && ajog_speed_changed)) {
	    if (bit != 0)
		sendJogCont(axis_num,new_halui_data.ajog_speed,JOGTELEOP);
	    else
		sendJogStop(axis_num,JOGTELEOP);
	    old_halui_data.ajog_plus[axis_num] = bit;
	}

	floatt = new_halui_data.ajog_analog[axis_num];
	bit = (fabs(floatt) > new_halui_data.ajog_deadband);
	if ((floatt != old_halui_data.ajog_analog[axis_num]) || (bit && ajog_speed_changed)) {
	    if (bit)
		sendJogCont(axis_num,(new_halui_data.ajog_speed) * (new_halui_data.ajog_analog[axis_num]),JOGTELEOP);
	    else
		sendJogStop(axis_num,JOGTELEOP);
	    old_halui_data.ajog_analog[axis_num] = floatt;
	}

	bit = new_halui_data.ajog_increment_plus[axis_num];
	if (bit != old_halui_data.ajog_increment_plus[axis_num]) {
	    if (bit)
		sendJogIncr(axis_num, new_halui_data.ajog_speed, new_halui_data.ajog_increment[axis_num],JOGTELEOP);
	    old_halui_data.ajog_increment_plus[axis_num] = bit;
	}

	bit = new_halui_data.ajog_increment_minus[axis_num];
	if (bit != old_halui_data.ajog_increment_minus[axis_num]) {
	    if (bit)
		sendJogIncr(axis_num, new_halui_data.ajog_speed, -(new_halui_data.ajog_increment[axis_num]),JOGTELEOP);
	    old_halui_data.ajog_increment_minus[axis_num] = bit;
	}

	// check to see if another axis has been selected
	bit = new_halui_data.axis_nr_select[axis_num];
	if (bit != old_halui_data.axis_nr_select[axis_num]) {
	    if (bit != 0) {
		*halui_data->axis_selected = axis_num;
		aselect_changed = axis_num; // flag that we changed the selected axis
	    }
	    old_halui_data.axis_nr_select[axis_num] = bit;
	}
    }

    if (aselect_changed >= 0) {
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        if ( !(axis_mask & (1 << axis_num)) ) { continue; }
	    if (axis_num != aselect_changed) {
		*(halui_data->axis_is_selected[axis_num]) = 0;
                if (jogging_selected_axis(old_halui_data) && !jogging_axis(old_halui_data, axis_num)) {
                    sendJogStop(axis_num,JOGTELEOP);
                }
            } else {
		*(halui_data->axis_is_selected[axis_num]) = 1;
                if (*halui_data->ajog_plus[num_axes]) {
                    sendJogCont(axis_num, new_halui_data.ajog_speed,JOGTELEOP);
                } else if (*halui_data->ajog_minus[num_axes]) {
                    sendJogCont(axis_num, -new_halui_data.ajog_speed,JOGTELEOP);
                }
	    }
	}
    }

    if (check_bit_changed(new_halui_data.joint_home[num_joints], old_halui_data.joint_home[num_joints]) != 0)
	sendHome(new_halui_data.joint_selected);

    if (check_bit_changed(new_halui_data.joint_unhome[num_joints], old_halui_data.joint_unhome[num_joints]) != 0)
	sendUnhome(new_halui_data.joint_selected);

    bit = new_halui_data.jjog_minus[num_joints];
    js = new_halui_data.joint_selected;
    if ((bit != old_halui_data.jjog_minus[num_joints]) || (bit && jjog_speed_changed)) {
        if (bit != 0)
	    sendJogCont(js, -new_halui_data.jjog_speed,JOGJOINT);
	else
	    sendJogStop(js,JOGJOINT);
	old_halui_data.jjog_minus[num_joints] = bit;
    }

    bit = new_halui_data.jjog_plus[num_joints];
    js = new_halui_data.joint_selected;
    if ((bit != old_halui_data.jjog_plus[num_joints]) || (bit && jjog_speed_changed)) {
        if (bit != 0)
	    sendJogCont(js,new_halui_data.jjog_speed,JOGJOINT);
	else
	    sendJogStop(js,JOGJOINT);
	old_halui_data.jjog_plus[num_joints] = bit;
    }

    bit = new_halui_data.jjog_increment_plus[num_joints];
    js = new_halui_data.joint_selected;
    if (bit != old_halui_data.jjog_increment_plus[num_joints]) {
	if (bit)
	    sendJogIncr(js, new_halui_data.jjog_speed, new_halui_data.jjog_increment[num_joints],JOGJOINT);
	old_halui_data.jjog_increment_plus[num_joints] = bit;
    }

    bit = new_halui_data.jjog_increment_minus[num_joints];
    js = new_halui_data.joint_selected;
    if (bit != old_halui_data.jjog_increment_minus[num_joints]) {
	if (bit)
	    sendJogIncr(js, new_halui_data.jjog_speed, -(new_halui_data.jjog_increment[num_joints]),JOGJOINT);
	old_halui_data.jjog_increment_minus[num_joints] = bit;
    }

    bit = new_halui_data.ajog_minus[EMCMOT_MAX_AXIS];
    js = new_halui_data.axis_selected;
    if ((bit != old_halui_data.ajog_minus[EMCMOT_MAX_AXIS]) || (bit && ajog_speed_changed)) {
        if (bit != 0)
	    sendJogCont(js, -new_halui_data.ajog_speed,JOGTELEOP);
	else
	    sendJogStop(js,JOGTELEOP);
	old_halui_data.ajog_minus[EMCMOT_MAX_AXIS] = bit;
    }

    bit = new_halui_data.ajog_plus[EMCMOT_MAX_AXIS];
    js = new_halui_data.axis_selected;
    if ((bit != old_halui_data.ajog_plus[EMCMOT_MAX_AXIS]) || (bit && ajog_speed_changed)) {
        if (bit != 0)
	    sendJogCont(js,new_halui_data.ajog_speed,JOGTELEOP);
	else
	    sendJogStop(js,JOGTELEOP);
	old_halui_data.ajog_plus[EMCMOT_MAX_AXIS] = bit;
    }

    bit = new_halui_data.ajog_increment_plus[EMCMOT_MAX_AXIS];
    js = new_halui_data.axis_selected;
    if (bit != old_halui_data.ajog_increment_plus[EMCMOT_MAX_AXIS]) {
	if (bit)
	    sendJogIncr(js, new_halui_data.ajog_speed, new_halui_data.ajog_increment[EMCMOT_MAX_AXIS],JOGTELEOP);
	old_halui_data.ajog_increment_plus[EMCMOT_MAX_AXIS] = bit;
    }

    bit = new_halui_data.ajog_increment_minus[EMCMOT_MAX_AXIS];
    js = new_halui_data.axis_selected;
    if (bit != old_halui_data.ajog_increment_minus[EMCMOT_MAX_AXIS]) {
	if (bit)
	    sendJogIncr(js, new_halui_data.ajog_speed, -(new_halui_data.ajog_increment[EMCMOT_MAX_AXIS]),JOGTELEOP);
	old_halui_data.ajog_increment_minus[EMCMOT_MAX_AXIS] = bit;
    }

    for(int n = 0; n < num_mdi_commands; n++) {
        if (check_bit_changed(new_halui_data.mdi_commands[n], old_halui_data.mdi_commands[n]) != 0)
            sendMdiCommand(n);
    }
}

// this function looks at the received NML status message
// and modifies the appropriate HAL pins
static void modify_hal_pins()
{
    int joint;
    int spindle;

    if (emcStatus->task.state == EMC_TASK_STATE::ON) {
	*(halui_data->machine_is_on)=1;
    } else {
	*(halui_data->machine_is_on)=0;
    }

    if (emcStatus->task.state == EMC_TASK_STATE::ESTOP) {
	*(halui_data->estop_is_activated)=1;
    } else {
	*(halui_data->estop_is_activated)=0;
    }

    if (halui_sent_mdi) { // we have an ongoing MDI command
	if (emcStatus->status == RCS_STATUS::DONE) { //which seems to have finished
	    switch (halui_old_mode) {
		case EMC_TASK_MODE::MANUAL: sendManual();break;
		case EMC_TASK_MODE::MDI: break;
		case EMC_TASK_MODE::AUTO: sendAuto();break;
		default: sendManual();break;
	    }
	}
    }
	

    if (emcStatus->task.mode == EMC_TASK_MODE::MANUAL) {
	*(halui_data->mode_is_manual)=1;
    } else {
	*(halui_data->mode_is_manual)=0;
    }

    if (emcStatus->task.mode == EMC_TASK_MODE::AUTO) {
	*(halui_data->mode_is_auto)=1;
    } else {
	*(halui_data->mode_is_auto)=0;
    }

    if (emcStatus->task.mode == EMC_TASK_MODE::MDI) {
	*(halui_data->mode_is_mdi)=1;
    } else {
	*(halui_data->mode_is_mdi)=0;
    }

    if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::TELEOP) {
	*(halui_data->mode_is_teleop)=1;
    } else {
	*(halui_data->mode_is_teleop)=0;
    }

    if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::FREE) {
	*(halui_data->mode_is_joint)=1;
    } else {
	*(halui_data->mode_is_joint)=0;
    }

    *(halui_data->program_is_paused) = emcStatus->task.interpState == EMC_TASK_INTERP::PAUSED;
    *(halui_data->program_is_running) = emcStatus->task.interpState == EMC_TASK_INTERP::READING ||
                                        emcStatus->task.interpState == EMC_TASK_INTERP::WAITING;
    *(halui_data->program_is_idle) = emcStatus->task.interpState == EMC_TASK_INTERP::IDLE;
    
    if (num_mdi_commands>0){
		// we wants initialize program_is_idle and mode_is_mdi before halui_sent_mdi
		if (halui_sent_mdi) { // we have an ongoing MDI command
			if (emcStatus->status == RCS_STATUS::DONE){ //which seems to have finished
			halui_sent_mdi = 0;
			esleep(0.02); //sleep for a while
			updateStatus();
			esleep(0.02); //sleep for a while
			}
		}
		*(halui_data->halui_mdi_is_running) = halui_sent_mdi;
	}


    
    *(halui_data->program_os_is_on) = emcStatus->task.optional_stop_state;
    *(halui_data->program_bd_is_on) = emcStatus->task.block_delete_state;

    *(halui_data->mv_value) = emcStatus->motion.traj.maxVelocity;
    *(halui_data->fo_value) = emcStatus->motion.traj.scale; //feedoverride from 0 to 1 for 100%
    *(halui_data->ro_value) = emcStatus->motion.traj.rapid_scale; //rapid override from 0 to 1 for 100%

    *(halui_data->mist_is_on) = emcStatus->io.coolant.mist;
    *(halui_data->flood_is_on) = emcStatus->io.coolant.flood;

    *(halui_data->tool_number) = emcStatus->io.tool.toolInSpindle;
    *(halui_data->tool_length_offset_x) = emcStatus->task.toolOffset.tran.x;
    *(halui_data->tool_length_offset_y) = emcStatus->task.toolOffset.tran.y;
    *(halui_data->tool_length_offset_z) = emcStatus->task.toolOffset.tran.z;
    *(halui_data->tool_length_offset_a) = emcStatus->task.toolOffset.a;
    *(halui_data->tool_length_offset_b) = emcStatus->task.toolOffset.b;
    *(halui_data->tool_length_offset_c) = emcStatus->task.toolOffset.c;
    *(halui_data->tool_length_offset_u) = emcStatus->task.toolOffset.u;
    *(halui_data->tool_length_offset_v) = emcStatus->task.toolOffset.v;
    *(halui_data->tool_length_offset_w) = emcStatus->task.toolOffset.w;

    if (emcStatus->io.tool.toolInSpindle == 0) {
        *(halui_data->tool_diameter) = 0.0;
    } else {
        int idx;
        for (idx = 0; idx <= tooldata_last_index_get(); idx ++) { // note <=
            CANON_TOOL_TABLE tdata;
            if (tooldata_get(&tdata,idx) != IDX_OK) {
                fprintf(stderr,"UNEXPECTED idx %s %d\n",__FILE__,__LINE__);
            }
            if (tdata.toolno == emcStatus->io.tool.toolInSpindle) {
                *(halui_data->tool_diameter) = tdata.diameter;
                break;
            }
        }
        if (idx == CANON_POCKETS_MAX) {
            // didn't find the tool
            *(halui_data->tool_diameter) = 0.0;
        }
    }

    for (spindle = 0; spindle < num_spindles; spindle++){
        *(halui_data->spindle_is_on[spindle]) = (emcStatus->motion.spindle[spindle].enabled);
        *(halui_data->spindle_runs_forward[spindle]) = (emcStatus->motion.spindle[spindle].direction == 1);
        *(halui_data->spindle_runs_backward[spindle]) = (emcStatus->motion.spindle[spindle].direction == -1);
        *(halui_data->spindle_brake_is_on[spindle]) = emcStatus->motion.spindle[spindle].brake;
        *(halui_data->so_value[spindle]) = emcStatus->motion.spindle[spindle].spindle_scale; //spindle-speed-override from 0 to 1 for 100%
    }

    for (joint=0; joint < num_joints; joint++) {
	*(halui_data->joint_is_homed[joint]) = emcStatus->motion.joint[joint].homed;
	*(halui_data->joint_on_soft_min_limit[joint]) = emcStatus->motion.joint[joint].minSoftLimit;
	*(halui_data->joint_on_soft_max_limit[joint]) = emcStatus->motion.joint[joint].maxSoftLimit;
	*(halui_data->joint_on_hard_min_limit[joint]) = emcStatus->motion.joint[joint].minHardLimit;
	*(halui_data->joint_on_hard_max_limit[joint]) = emcStatus->motion.joint[joint].maxHardLimit;
	*(halui_data->joint_override_limits[joint]) = emcStatus->motion.joint[joint].overrideLimits;
	*(halui_data->joint_has_fault[joint]) = emcStatus->motion.joint[joint].fault;
    }

    if (axis_mask & 0x0001) {
      *(halui_data->axis_pos_commanded[0]) = emcStatus->motion.traj.position.tran.x;
      *(halui_data->axis_pos_feedback[0]) = emcStatus->motion.traj.actualPosition.tran.x;
      double x = emcStatus->motion.traj.actualPosition.tran.x - emcStatus->task.g5x_offset.tran.x - emcStatus->task.toolOffset.tran.x;
      double y = emcStatus->motion.traj.actualPosition.tran.y - emcStatus->task.g5x_offset.tran.y - emcStatus->task.toolOffset.tran.y;
      x = x * cos(-emcStatus->task.rotation_xy * TO_RAD) - y * sin(-emcStatus->task.rotation_xy * TO_RAD);
      *(halui_data->axis_pos_relative[0]) = x - emcStatus->task.g92_offset.tran.x;
    }

    if (axis_mask & 0x0002) {
      *(halui_data->axis_pos_commanded[1]) = emcStatus->motion.traj.position.tran.y;
      *(halui_data->axis_pos_feedback[1]) = emcStatus->motion.traj.actualPosition.tran.y;
      double x = emcStatus->motion.traj.actualPosition.tran.x - emcStatus->task.g5x_offset.tran.x - emcStatus->task.toolOffset.tran.x;
      double y = emcStatus->motion.traj.actualPosition.tran.y - emcStatus->task.g5x_offset.tran.y - emcStatus->task.toolOffset.tran.y;
      y = y * cos(-emcStatus->task.rotation_xy * TO_RAD) + x * sin(-emcStatus->task.rotation_xy * TO_RAD);
      *(halui_data->axis_pos_relative[1]) = y - emcStatus->task.g92_offset.tran.y;
    }

    if (axis_mask & 0x0004) {
      *(halui_data->axis_pos_commanded[2]) = emcStatus->motion.traj.position.tran.z;
      *(halui_data->axis_pos_feedback[2]) = emcStatus->motion.traj.actualPosition.tran.z;
      *(halui_data->axis_pos_relative[2]) = emcStatus->motion.traj.actualPosition.tran.z - emcStatus->task.g5x_offset.tran.z - emcStatus->task.g92_offset.tran.z - emcStatus->task.toolOffset.tran.z;
    }

    if (axis_mask & 0x0008) {
      *(halui_data->axis_pos_commanded[3]) = emcStatus->motion.traj.position.a;
      *(halui_data->axis_pos_feedback[3]) = emcStatus->motion.traj.actualPosition.a;
      *(halui_data->axis_pos_relative[3]) = emcStatus->motion.traj.actualPosition.a - emcStatus->task.g5x_offset.a - emcStatus->task.g92_offset.a - emcStatus->task.toolOffset.a;
    }

    if (axis_mask & 0x0010) {
      *(halui_data->axis_pos_commanded[4]) = emcStatus->motion.traj.position.b;
      *(halui_data->axis_pos_feedback[4]) = emcStatus->motion.traj.actualPosition.b;
      *(halui_data->axis_pos_relative[4]) = emcStatus->motion.traj.actualPosition.b - emcStatus->task.g5x_offset.b - emcStatus->task.g92_offset.b - emcStatus->task.toolOffset.b;
    }

    if (axis_mask & 0x0020) {
      *(halui_data->axis_pos_commanded[5]) = emcStatus->motion.traj.position.c;
      *(halui_data->axis_pos_feedback[5]) = emcStatus->motion.traj.actualPosition.c;
      *(halui_data->axis_pos_relative[5]) = emcStatus->motion.traj.actualPosition.c - emcStatus->task.g5x_offset.c - emcStatus->task.g92_offset.c - emcStatus->task.toolOffset.c;
    }

    if (axis_mask & 0x0040) {
      *(halui_data->axis_pos_commanded[6]) = emcStatus->motion.traj.position.u;
      *(halui_data->axis_pos_feedback[6]) = emcStatus->motion.traj.actualPosition.u;
      *(halui_data->axis_pos_relative[6]) = emcStatus->motion.traj.actualPosition.u - emcStatus->task.g5x_offset.u - emcStatus->task.g92_offset.u - emcStatus->task.toolOffset.u;
    }

    if (axis_mask & 0x0080) {
      *(halui_data->axis_pos_commanded[7]) = emcStatus->motion.traj.position.v;
      *(halui_data->axis_pos_feedback[7]) = emcStatus->motion.traj.actualPosition.v;
      *(halui_data->axis_pos_relative[7]) = emcStatus->motion.traj.actualPosition.v - emcStatus->task.g5x_offset.v - emcStatus->task.g92_offset.v - emcStatus->task.toolOffset.v;
    }

    if (axis_mask & 0x0100) {
      *(halui_data->axis_pos_commanded[8]) = emcStatus->motion.traj.position.w;
      *(halui_data->axis_pos_feedback[8]) = emcStatus->motion.traj.actualPosition.w;
      *(halui_data->axis_pos_relative[8]) = emcStatus->motion.traj.actualPosition.w - emcStatus->task.g5x_offset.w - emcStatus->task.g92_offset.w - emcStatus->task.toolOffset.w;
    }

    *(halui_data->joint_is_homed[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].homed;
    *(halui_data->joint_on_soft_min_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].minSoftLimit;
    *(halui_data->joint_on_soft_max_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].maxSoftLimit;
    *(halui_data->joint_on_hard_min_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].minHardLimit;
    *(halui_data->joint_override_limits[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].overrideLimits;
    *(halui_data->joint_on_hard_max_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].maxHardLimit;
    *(halui_data->joint_has_fault[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].fault;

}



int main(int argc, char *argv[])
{
    // process command line args
    if (0 != emcGetArgs(argc, argv)) {
	rcs_print_error("error in argument list\n");
	exit(1);
    }

    // get configuration information
    if (0 != iniLoad(emc_inifile)) {
	rcs_print_error("iniLoad error\n");
	exit(2);
    }

    //init HAL and export pins
    if (0 != halui_hal_init()) {
	rcs_print_error("hal_init error\n");
	exit(1);
    }

    //initialize safe values
    hal_init_pins();

    // init NML
    if (0 != tryNml()) {
	rcs_print_error("can't connect to emc\n");
	thisQuit();
	exit(1);
    }

#ifdef TOOL_NML //{
    //fprintf(stderr,"%8d HALUI REGISTER %p\n",getpid(),
    tool_nml_register((CANON_TOOL_TABLE*)&emcStatus->io.tool.toolTable);
#else //}{
    tool_mmap_user();
#endif //}

    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();

    done = 0;
    /* Register the routine that catches the SIGINT signal */
    signal(SIGINT, quit);
    /* catch SIGTERM too - the run script uses it to shut things down */
    signal(SIGTERM, quit);

    while (!done) {
        static bool task_start_synced = 0;
        if (!task_start_synced) {
           // wait for task to establish nonzero linearUnits
           if (emcStatus->motion.traj.linearUnits != 0) {
              // set once at startup, no changes are expected:
              *(halui_data->units_per_mm) = emcStatus->motion.traj.linearUnits;
              task_start_synced = 1;
           }
        }
        check_hal_changes(); //if anything changed send NML messages
        modify_hal_pins(); //if status changed modify HAL too
        esleep(0.02); //sleep for a while
        updateStatus();
    }
    thisQuit();
    return 0;
}
