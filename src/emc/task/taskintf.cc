/********************************************************************
* Description: taskintf.cc
*   Interface functions for motion.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/

#include <cmath>
#include <float.h>		// DBL_MAX
#include <string.h>		// memcpy() strncpy()
#include <unistd.h>             // unlink()

#include "usrmotintf.h"		// usrmotInit(), usrmotReadEmcmotStatus(),
				// etc.
#include "motion.h"		// emcmot_command_t,STATUS, etc.
#include "homing.h"
#include "emc.hh"
#include "emccfg.h"		// EMC_INIFILE
#include "emcglb.h"		// EMC_INIFILE
#include "emc_nml.hh"
#include "rcs_print.hh"
#include "timer.hh"
#include "inifile.hh"
#include "iniaxis.hh"
#include "inijoint.hh"
#include "inispindle.hh"
#include "initraj.hh"
#include "inihal.hh"

value_inihal_data old_inihal_data;

/* define this to catch isnan errors, for rtlinux FPU register 
   problem testing */
#define ISNAN_TRAP

#ifdef ISNAN_TRAP
#define CATCH_NAN(cond) do {                           \
    if (cond) {                                        \
        printf("isnan error in %s()\n", __FUNCTION__); \
        return -1;                                     \
    }                                                  \
} while(0)
#else
#define CATCH_NAN(cond) do {} while(0)
#endif


// MOTION INTERFACE

/*! \todo FIXME - this decl was originally much later in the file, moved
here temporarily for debugging */
static emcmot_status_t emcmotStatus;

/*
  Implementation notes:

  Initing:  the emcmot interface needs to be inited once, but nml_traj_init()
  and nml_servo_init() can be called in any order. Similarly, the emcmot
  interface needs to be exited once, but nml_traj_exit() and nml_servo_exit()
  can be called in any order. They can also be called multiple times. Flags
  are used to signify if initing has been done, or if the final exit has
  been called.
  */

static struct TrajConfig_t TrajConfig;
static struct JointConfig_t JointConfig[EMCMOT_MAX_JOINTS];
static struct AxisConfig_t AxisConfig[EMCMOT_MAX_AXIS];
static struct SpindleConfig_t SpindleConfig[EMCMOT_MAX_SPINDLES];

static emcmot_command_t emcmotCommand;

// local status data, not provided by emcmot
static int localMotionCommandType = 0;
static int localMotionEchoSerialNumber = 0;

// axes and joints are numbered 0..NUM-1

/*
  In emcmot, we need to set the cycle time for traj, and the interpolation
  rate, in any order, but both need to be done. 
 */

/*! functions involving joints */

int emcJointSetType(int joint, unsigned char jointType)
{
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].Type = jointType;

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %d)\n", __FUNCTION__, joint, jointType);
    }
    return 0;
}

int emcJointSetUnits(int joint, double units)
{
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].Units = units;

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f)\n", __FUNCTION__, joint, units);
    }
    return 0;
}

int emcJointSetBacklash(int joint, double backlash)
{
#ifdef ISNAN_TRAP
    if (std::isnan(backlash)) {
	printf("std::isnan error in emcJointSetBacklash()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_JOINT_BACKLASH;
    emcmotCommand.joint = joint;
    emcmotCommand.backlash = backlash;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, backlash, retval);
    }
    return retval;
}

int emcJointSetMinPositionLimit(int joint, double limit)
{
#ifdef ISNAN_TRAP
    if (std::isnan(limit)) {
	printf("isnan error in emcJointSetMinPosition()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].MinLimit = limit;

    emcmotCommand.command = EMCMOT_SET_JOINT_POSITION_LIMITS;
    emcmotCommand.joint = joint;
    emcmotCommand.minLimit = JointConfig[joint].MinLimit;
    emcmotCommand.maxLimit = JointConfig[joint].MaxLimit;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4g) returned %d\n", __FUNCTION__, joint, limit, retval);
    }
    return retval;
}

int emcJointSetMaxPositionLimit(int joint, double limit)
{
#ifdef ISNAN_TRAP
    if (std::isnan(limit)) {
	printf("std::isnan error in emcJointSetMaxPosition()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].MaxLimit = limit;

    emcmotCommand.command = EMCMOT_SET_JOINT_POSITION_LIMITS;
    emcmotCommand.joint = joint;
    emcmotCommand.minLimit = JointConfig[joint].MinLimit;
    emcmotCommand.maxLimit = JointConfig[joint].MaxLimit;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4g) returned %d\n", __FUNCTION__, joint, limit, retval);
    }
    return retval;
}

int emcJointSetMotorOffset(int joint, double offset) 
{
#ifdef ISNAN_TRAP
    if (std::isnan(offset)) {
	printf("isnan error in emcJointSetMotorOffset()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    emcmotCommand.command = EMCMOT_SET_JOINT_MOTOR_OFFSET;
    emcmotCommand.joint = joint;
    emcmotCommand.motor_offset = offset;
    
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, offset, retval);
    }
    return retval;
}

int emcJointSetFerror(int joint, double ferror)
{
#ifdef ISNAN_TRAP
    if (std::isnan(ferror)) {
	printf("isnan error in emcJointSetFerror()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_JOINT_MAX_FERROR;
    emcmotCommand.joint = joint;
    emcmotCommand.maxFerror = ferror;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, ferror, retval);
    }
    return retval;
}

int emcJointSetMinFerror(int joint, double ferror)
{
#ifdef ISNAN_TRAP
    if (std::isnan(ferror)) {
	printf("isnan error in emcJointSetMinFerror()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    emcmotCommand.command = EMCMOT_SET_JOINT_MIN_FERROR;
    emcmotCommand.joint = joint;
    emcmotCommand.minFerror = ferror;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, ferror, retval);
    }
    return retval;
}

int emcJointSetHomingParams(int joint, double home, double offset, double home_final_vel,
			   double search_vel, double latch_vel,
			   int use_index, int encoder_does_not_reset,
			   int ignore_limits, int is_shared,
			   int sequence,int volatile_home, int locking_indexer,int absolute_encoder)
{
#ifdef ISNAN_TRAP
    if (std::isnan(home) || std::isnan(offset) || std::isnan(home_final_vel) ||
	std::isnan(search_vel) || std::isnan(latch_vel)) {
	printf("isnan error in emcJointSetHomingParams()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_JOINT_HOMING_PARAMS;
    emcmotCommand.joint = joint;
    emcmotCommand.home = home;
    emcmotCommand.offset = offset;
    emcmotCommand.home_final_vel = home_final_vel;
    emcmotCommand.search_vel = search_vel;
    emcmotCommand.latch_vel = latch_vel;
    emcmotCommand.flags = 0;
    emcmotCommand.home_sequence = sequence;
    emcmotCommand.volatile_home = volatile_home;
    if (use_index) {
	emcmotCommand.flags |= HOME_USE_INDEX;
    }
    if (encoder_does_not_reset) {
	emcmotCommand.flags |= HOME_INDEX_NO_ENCODER_RESET;
    }
    if (ignore_limits) {
	emcmotCommand.flags |= HOME_IGNORE_LIMITS;
    }
    if (is_shared) {
	emcmotCommand.flags |= HOME_IS_SHARED;
    }
    if (locking_indexer) {
        emcmotCommand.flags |= HOME_UNLOCK_FIRST;
    }
    if (absolute_encoder) {
        switch (absolute_encoder) {
          case 0: break;
          case 1: emcmotCommand.flags |= HOME_ABSOLUTE_ENCODER;
                  emcmotCommand.flags |= HOME_NO_REHOME;
                  break;
          case 2: emcmotCommand.flags |= HOME_ABSOLUTE_ENCODER;
                  emcmotCommand.flags |= HOME_NO_REHOME;
                  emcmotCommand.flags |= HOME_NO_FINAL_MOVE;
                  break;
          default: fprintf(stderr,
                   "Unknown option for absolute_encoder <%d>",absolute_encoder);
                  break;
        }
    }

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f, %.4f, %.4f, %.4f, %.4f, %d, %d, %d, %d, %d) returned %d\n",
          __FUNCTION__, joint, home, offset, home_final_vel, search_vel, latch_vel,
          use_index, ignore_limits, is_shared, sequence, volatile_home, retval);
    }
    return retval;
}

int emcJointUpdateHomingParams(int joint, double home, double offset, int sequence)
{
    CATCH_NAN(std::isnan(home) || std::isnan(offset) );

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_UPDATE_JOINT_HOMING_PARAMS;
    emcmotCommand.joint = joint;
    emcmotCommand.home = home;
    emcmotCommand.offset = offset;
    emcmotCommand.home_sequence = sequence;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f, %.4f) returned %d\n",
          __FUNCTION__, joint, home, offset,retval);
    }
    return retval;
}

int emcJointSetMaxVelocity(int joint, double vel)
{
    CATCH_NAN(std::isnan(vel));

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    if (vel < 0.0) {
	vel = 0.0;
    }

    JointConfig[joint].MaxVel = vel;

    emcmotCommand.command = EMCMOT_SET_JOINT_VEL_LIMIT;
    emcmotCommand.joint = joint;
    emcmotCommand.vel = vel;
    
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, vel, retval);
    }
    return retval;
}

int emcJointSetMaxAcceleration(int joint, double acc)
{
    CATCH_NAN(std::isnan(acc));

    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    if (acc < 0.0) {
	acc = 0.0;
    }
    JointConfig[joint].MaxAccel = acc;
    //FIXME-AJ: need functions for setting the AXIS_MAX_ACCEL (either from the INI, or from kins..)
    emcmotCommand.command = EMCMOT_SET_JOINT_ACC_LIMIT;
    emcmotCommand.joint = joint;
    emcmotCommand.acc = acc;
    
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4g) returned %d\n", __FUNCTION__, joint, acc, retval);
    }
    return retval;
}

/*! functions involving cartesian Axes (X,Y,Z,A,B,C,U,V,W) */
    
int emcAxisSetMinPositionLimit(int axis, double limit)
{
    CATCH_NAN(std::isnan(limit));

    if (axis < 0 || axis >= EMCMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    AxisConfig[axis].MinLimit = limit;

    emcmotCommand.command = EMCMOT_SET_AXIS_POSITION_LIMITS;
    emcmotCommand.axis = axis;
    emcmotCommand.minLimit = AxisConfig[axis].MinLimit;
    emcmotCommand.maxLimit = AxisConfig[axis].MaxLimit;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, limit, retval);
    }
    return retval;
}

int emcAxisSetMaxPositionLimit(int axis, double limit)
{
    CATCH_NAN(std::isnan(limit));

    if (axis < 0 || axis >= EMCMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    AxisConfig[axis].MaxLimit = limit;

    emcmotCommand.command = EMCMOT_SET_AXIS_POSITION_LIMITS;
    emcmotCommand.axis = axis;
    emcmotCommand.minLimit = AxisConfig[axis].MinLimit;
    emcmotCommand.maxLimit = AxisConfig[axis].MaxLimit;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, limit, retval);
    }
    return retval;
}

int emcAxisSetMaxVelocity(int axis, double vel,double ext_offset_vel)
{
    CATCH_NAN(std::isnan(vel));

    if (axis < 0 || axis >= EMCMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    if (vel < 0.0) {
	vel = 0.0;
    }

    AxisConfig[axis].MaxVel = vel;

    emcmotCommand.command = EMCMOT_SET_AXIS_VEL_LIMIT;
    emcmotCommand.axis = axis;
    emcmotCommand.vel = vel;
    emcmotCommand.ext_offset_vel = ext_offset_vel;
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, vel, retval);
    }
    return retval;
}

int emcAxisSetMaxAcceleration(int axis, double acc,double ext_offset_acc)
{
    CATCH_NAN(std::isnan(acc));

    if (axis < 0 || axis >= EMCMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    if (acc < 0.0) {
	acc = 0.0;
    }
    
    AxisConfig[axis].MaxAccel = acc;    

    emcmotCommand.command = EMCMOT_SET_AXIS_ACC_LIMIT;
    emcmotCommand.axis = axis;
    emcmotCommand.acc = acc;
    emcmotCommand.ext_offset_acc = ext_offset_acc;
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, acc, retval);
    }
    return retval;
}

int emcAxisSetLockingJoint(int axis, int joint)
{

    if (axis < 0 || axis >= EMCMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    if (joint < 0) {
	joint = -1;
    }

    emcmotCommand.command = EMCMOT_SET_AXIS_LOCKING_JOINT;
    emcmotCommand.axis    = axis;
    emcmotCommand.joint   = joint;
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %d) returned %d\n", __FUNCTION__, axis, joint, retval);
    }
    return retval;
}

double emcAxisGetMaxVelocity(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_AXIS) {
        return 0;
    }

    return AxisConfig[axis].MaxVel;
}

double emcAxisGetMaxAcceleration(int axis)
{
    if (axis < 0 || axis >= EMCMOT_MAX_AXIS) {
        return 0;
    }

    return AxisConfig[axis].MaxAccel;
}

int emcAxisUpdate(EMC_AXIS_STAT stat[], int axis_mask)
{
    int axis_num;
    emcmot_axis_status_t *axis;
    
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        if(!(axis_mask & (1 << axis_num))) continue;
        axis = &(emcmotStatus.axis_status[axis_num]);

        stat[axis_num].velocity = axis->teleop_vel_cmd;
        stat[axis_num].minPositionLimit = axis->min_pos_limit;
        stat[axis_num].maxPositionLimit = axis->max_pos_limit;
    }
    return 0;
}

/* This function checks to see if any joint or the traj has
   been inited already.  At startup, if none have been inited,
   usrmotIniLoad and usrmotInit must be called first.  At
   shutdown, after all have been halted, the usrmotExit must
   be called.
*/

static int JointOrTrajInited(void)
{
    int joint, spindle;

    for (joint = 0; joint < EMCMOT_MAX_JOINTS; joint++) {
	if (JointConfig[joint].Inited) {
	    return 1;
	}
    }
    for (spindle = 0; spindle < EMCMOT_MAX_SPINDLES; spindle++) {
        if (SpindleConfig[spindle].Inited) {
            return 1;
        }
    }
    if (TrajConfig.Inited) {
    return 1;
    }
    return 0;
}

int emcJointInit(int joint)
{
    int retval = 0;
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    // init emcmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(emc_inifile);
	if (0 != usrmotInit("emc2_task")) {
	    return -1;
	}
    }
    JointConfig[joint].Inited = 1;
    if (0 != iniJoint(joint, emc_inifile)) {
	retval = -1;
    }
    return retval;
}

int emcAxisInit(int axis)
{
    int retval = 0;

    if (axis < 0 || axis >= EMCMOT_MAX_AXIS) {
	return 0;
    }
    // init emcmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(emc_inifile);
	if (0 != usrmotInit("emc2_task")) {
	    return -1;
	}
    }
    AxisConfig[axis].Inited = 1;
    if (0 != iniAxis(axis, emc_inifile)) {
	retval = -1;
    }
    return retval;
}

int emcSpindleInit(int spindle)
{
    int retval = 0;

    if (spindle < 0 || spindle >= EMCMOT_MAX_SPINDLES) {
	return 0;
    }
    // init emcmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(emc_inifile);
	if (0 != usrmotInit("emc2_task")) {
	    return -1;
	}
    }
    SpindleConfig[spindle].Inited = 1;
    if (0 != iniSpindle(spindle, emc_inifile)) {
	retval = -1;
    }
    return retval;
}

int emcJointHalt(int joint)
{
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    /*! \todo FIXME-- refs global emcStatus; should make EMC_JOINT_STAT an arg here */
    JointConfig[joint].Inited = 0;

    if (!JointOrTrajInited()) {
	usrmotExit();		// ours is final exit
    }

    return 0;
}

int emcJogAbort(int joint)
{
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }
    emcmotCommand.command = EMCMOT_JOG_ABORT;
    emcmotCommand.joint = joint;
    emcmotCommand.axis  = -1; //NA

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJointActivate(int joint)
{
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_JOINT_ACTIVATE;
    emcmotCommand.joint = joint;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d) returned %d\n", __FUNCTION__, joint, retval);
    }
    return retval;
}

int emcJointDeactivate(int joint)
{
    if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_JOINT_DEACTIVATE;
    emcmotCommand.joint = joint;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJointOverrideLimits(int joint)
{
    // can have joint < 0, for resuming normal limit checking
    if (joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_OVERRIDE_LIMITS;
    emcmotCommand.joint = joint;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJointHome(int joint)
{
    if (joint < -1 || joint >= EMCMOT_MAX_JOINTS) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_JOINT_HOME;
    emcmotCommand.joint = joint;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJointUnhome(int joint)
{
	if (joint < -2 || joint >= EMCMOT_MAX_JOINTS) {
		return 0;
	}

	emcmotCommand.command = EMCMOT_JOINT_UNHOME;
	emcmotCommand.joint = joint;

	return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJogCont(int nr, double vel, int jjogmode)
{
    if (jjogmode) {
        if (nr < 0 || nr >= EMCMOT_MAX_JOINTS) { return 0; }
        if (vel > JointConfig[nr].MaxVel) {
            vel = JointConfig[nr].MaxVel;
        } else if (vel < -JointConfig[nr].MaxVel) {
            vel = -JointConfig[nr].MaxVel;
        }
        emcmotCommand.joint = nr;
        emcmotCommand.axis = -1;  //NA
    } else {
        if (nr < 0 || nr >= EMCMOT_MAX_AXIS) { return 0; }
        if (vel > AxisConfig[nr].MaxVel) {
            vel = AxisConfig[nr].MaxVel;
        } else if (vel < -AxisConfig[nr].MaxVel) {
            vel = -AxisConfig[nr].MaxVel;
        }
        emcmotCommand.joint = -1; //NA
        emcmotCommand.axis = nr;
    }
    emcmotCommand.command = EMCMOT_JOG_CONT;
    emcmotCommand.vel = vel;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJogIncr(int nr, double incr, double vel, int jjogmode)
{
    if (jjogmode) {
        if (nr < 0 || nr >= EMCMOT_MAX_JOINTS) { return 0; }
        if (vel > JointConfig[nr].MaxVel) {
            vel = JointConfig[nr].MaxVel;
        } else if (vel < -JointConfig[nr].MaxVel) {
            vel = -JointConfig[nr].MaxVel;
        }
        emcmotCommand.joint = nr;
        emcmotCommand.axis = -1; //NA
    } else {
        if (nr < 0 || nr >= EMCMOT_MAX_AXIS) { return 0; }
        if (vel > AxisConfig[nr].MaxVel) {
            vel = AxisConfig[nr].MaxVel;
        } else if (vel < -AxisConfig[nr].MaxVel) {
            vel = -AxisConfig[nr].MaxVel;
        }
        emcmotCommand.joint = -1; //NA
        emcmotCommand.axis = nr;
    }
    emcmotCommand.command = EMCMOT_JOG_INCR;
    emcmotCommand.vel = vel;
    emcmotCommand.offset = incr;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJogAbs(int nr, double pos, double vel, int jjogmode)
{
    if (jjogmode) {        
        if (nr < 0 || nr >= EMCMOT_MAX_JOINTS) { return 0; }
        if (vel > JointConfig[nr].MaxVel) {
            vel = JointConfig[nr].MaxVel;
        } else if (vel < -JointConfig[nr].MaxVel) {
            vel = -JointConfig[nr].MaxVel;
        }
        emcmotCommand.joint = nr;
        emcmotCommand.axis = -1; //NA
    } else {
        if (nr < 0 || nr >= EMCMOT_MAX_AXIS) { return 0; }
        if (vel > AxisConfig[nr].MaxVel) {
            vel = AxisConfig[nr].MaxVel;
        } else if (vel < -AxisConfig[nr].MaxVel) {
            vel = -AxisConfig[nr].MaxVel;
        }
        emcmotCommand.joint = -1; //NA
        emcmotCommand.axis = nr;
    }
    emcmotCommand.command = EMCMOT_JOG_ABS;
    emcmotCommand.vel = vel;
    emcmotCommand.offset = pos;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcJogStop(int nr, int jjogmode)
{
    if (jjogmode) {
        if (nr < 0 || nr >= EMCMOT_MAX_JOINTS) { return 0; }
        emcmotCommand.joint = nr;
        emcmotCommand.axis = -1; //NA
    } else {
        if (nr < 0 || nr >= EMCMOT_MAX_AXIS) { return 0; }
        emcmotCommand.joint = -1; //NA
        emcmotCommand.axis = nr;
    }
    emcmotCommand.command = EMCMOT_JOG_ABORT;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}


int emcJointLoadComp(int joint, const char *file, int type)
{
    return usrmotLoadComp(joint, file, type);
}

static emcmot_config_t emcmotConfig;
int get_emcmot_internal_info = 0;  // debug usage

/*
  these globals are set in emcMotionUpdate(), then referenced in
  emcJointUpdate(), emcTrajUpdate() to save calls to usrmotReadEmcmotStatus
 */
static emcmot_internal_t emcmotInternal;
static char errorString[EMCMOT_ERROR_LEN];
static int new_config = 0;

/*! \todo FIXME - debugging - uncomment the following line to log changes in
   JOINT_FLAG */
// #define WATCH_FLAGS 1

int emcJointUpdate(EMC_JOINT_STAT stat[], int numJoints)
{
/*! \todo FIXME - this function accesses data that has been
   moved.  Once I know what it is used for I'll fix it */

    int joint_num;
    emcmot_joint_status_t *joint;
#ifdef WATCH_FLAGS
    static int old_joint_flag[8];
#endif

    // check for valid range
    if (numJoints <= 0 || numJoints > EMCMOT_MAX_JOINTS) {
	return -1;
    }

    for (joint_num = 0; joint_num < numJoints; joint_num++) {
	/* point to joint data */

	joint = &(emcmotStatus.joint_status[joint_num]);

	stat[joint_num].jointType = JointConfig[joint_num].Type;
	stat[joint_num].units = JointConfig[joint_num].Units;
	if (new_config) {
	    stat[joint_num].backlash = joint->backlash;
	    stat[joint_num].minPositionLimit = joint->min_pos_limit;
	    stat[joint_num].maxPositionLimit = joint->max_pos_limit;
	    stat[joint_num].minFerror = joint->min_ferror;
	    stat[joint_num].maxFerror = joint->max_ferror;
/*! \todo FIXME - should all homing config params be included here? */
//	    stat[joint_num].homeOffset = joint->home_offset;
	}
	stat[joint_num].output = joint->pos_cmd;
	stat[joint_num].input = joint->pos_fb;
	stat[joint_num].velocity = joint->vel_cmd;
	stat[joint_num].ferrorCurrent = joint->ferror;
	stat[joint_num].ferrorHighMark = joint->ferror_high_mark;

	stat[joint_num].homing = joint->homing;
	stat[joint_num].homed  = joint->homed;

	stat[joint_num].fault = (joint->flag & EMCMOT_JOINT_FAULT_BIT ? 1 : 0);
	stat[joint_num].enabled = (joint->flag & EMCMOT_JOINT_ENABLE_BIT ? 1 : 0);
	stat[joint_num].inpos = (joint->flag & EMCMOT_JOINT_INPOS_BIT ? 1 : 0);

/* FIXME - soft limits are now applied to the command, and should never
   happen */
	stat[joint_num].minSoftLimit = 0;
	stat[joint_num].maxSoftLimit = 0;
	stat[joint_num].minHardLimit =
	    (joint->flag & EMCMOT_JOINT_MIN_HARD_LIMIT_BIT ? 1 : 0);
	stat[joint_num].maxHardLimit =
	    (joint->flag & EMCMOT_JOINT_MAX_HARD_LIMIT_BIT ? 1 : 0);
	stat[joint_num].overrideLimits = !!(emcmotStatus.overrideLimitMask);	// one
	// for
	// all

#ifdef WATCH_FLAGS
	if (old_joint_flag[joint_num] != joint->flag) {
	    printf("joint %d flag: %04X -> %04X\n", joint_num,
		   old_joint_flag[joint_num], joint->flag);
	    old_joint_flag[joint_num] = joint->flag;
	}
#endif
	if (joint->flag & EMCMOT_JOINT_ERROR_BIT) {
	    if (stat[joint_num].status != RCS_STATUS::ERROR) {
		rcs_print_error("Error on joint %d, command number %d\n",
				joint_num, emcmotStatus.commandNumEcho);
		stat[joint_num].status = RCS_STATUS::ERROR;
	    }
	} else if (joint->flag & EMCMOT_JOINT_INPOS_BIT) {
	    stat[joint_num].status = RCS_STATUS::DONE;
	} else {
	    stat[joint_num].status = RCS_STATUS::EXEC;
	}
    }
    return 0;
}

// EMC_TRAJ functions

int emcTrajSetJoints(int joints)
{
    if (joints <= 0 || joints > EMCMOT_MAX_JOINTS) {
	rcs_print("emcTrajSetJoints failing: joints=%d\n",
		joints);
	return -1;
    }

    TrajConfig.Joints = joints;
    emcmotCommand.command = EMCMOT_SET_NUM_JOINTS;
    emcmotCommand.joint = joints;
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d) returned %d\n", __FUNCTION__, joints, retval);
    }
    return retval;
}

// FIXME CJR move this to TrajConfig?
static struct state_tag_t localEmcTrajTag;

int emcTrajUpdateTag(StateTag const &tag) {
    localEmcTrajTag = tag.get_state_tag();
    return 0;
}

int emcTrajSetAxes(int axismask)
{
    int axes = 0;
    for(int i=0; i<EMCMOT_MAX_AXIS; i++)
        if(axismask & (1<<i)) axes = i+1;

    TrajConfig.AxisMask = axismask;
    
    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %d)\n", __FUNCTION__, axes, axismask);
    }
    return 0;
}

int emcTrajSetSpindles(int spindles)
{
    if (spindles <= 0 || spindles > EMCMOT_MAX_SPINDLES) {
	rcs_print("emcTrajSetSpindles failing: spindles=%d\n",
		spindles);
	return -1;
    }

    TrajConfig.Spindles = spindles;
    emcmotCommand.command = EMCMOT_SET_NUM_SPINDLES;
    emcmotCommand.spindle = spindles;
    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d) returned %d\n", __FUNCTION__, spindles, retval);
    }
    return retval;
}

int emcTrajSetUnits(double linearUnits, double angularUnits)
{
    if (linearUnits <= 0.0 || angularUnits <= 0.0) {
	return -1;
    }

    TrajConfig.LinearUnits = linearUnits;
    TrajConfig.AngularUnits = angularUnits;

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%.4f, %.4f)\n", __FUNCTION__, linearUnits, angularUnits);
    }
    return 0;
}

int emcTrajSetMode(EMC_TRAJ_MODE mode)
{
    switch (mode) {
    case EMC_TRAJ_MODE::FREE:
	emcmotCommand.command = EMCMOT_FREE;
	return usrmotWriteEmcmotCommand(&emcmotCommand);

    case EMC_TRAJ_MODE::COORD:
	emcmotCommand.command = EMCMOT_COORD;
	return usrmotWriteEmcmotCommand(&emcmotCommand);

    case EMC_TRAJ_MODE::TELEOP:
	emcmotCommand.command = EMCMOT_TELEOP;
	return usrmotWriteEmcmotCommand(&emcmotCommand);

    default:
	return -1;
    }
}

int emcTrajSetVelocity(double vel, double ini_maxvel)
{
    if (vel < 0.0) {
	vel = 0.0;
    } else if (vel > TrajConfig.MaxVel) {
	vel = TrajConfig.MaxVel;
    }

    if (ini_maxvel < 0.0) {
	    ini_maxvel = 0.0;
    } else if (vel > TrajConfig.MaxVel) {
	    ini_maxvel = TrajConfig.MaxVel;
    }

    emcmotCommand.command = EMCMOT_SET_VEL;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%.4f, %.4f) returned %d\n", __FUNCTION__, vel, ini_maxvel, retval);
    }
    return retval;
}

int emcTrajSetAcceleration(double acc)
{
    if (acc < 0.0) {
	acc = 0.0;
    } else if (acc > TrajConfig.MaxAccel) {
	acc = TrajConfig.MaxAccel;
    }

    emcmotCommand.command = EMCMOT_SET_ACC;
    emcmotCommand.acc = acc;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%.4g) returned %d\n", __FUNCTION__, acc, retval);
    }
    return retval;
}

/*
  emcmot has no limits on max velocity, acceleration so we'll save them
  here and apply them in the functions above
  */
int emcTrajSetMaxVelocity(double vel)
{
    if (vel < 0.0) {
	vel = 0.0;
    }

    TrajConfig.MaxVel = vel;

    emcmotCommand.command = EMCMOT_SET_VEL_LIMIT;
    emcmotCommand.vel = vel;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%.4f) returned %d\n", __FUNCTION__, vel, retval);
    }
    return retval;
}

int emcTrajSetMaxAcceleration(double acc)
{
    if (acc < 0.0) {
	acc = 0.0;
    }

    TrajConfig.MaxAccel = acc;

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%.4g)\n", __FUNCTION__, acc);
    }
    return 0;
}

int emcTrajSetHome(EmcPose home)
{
#ifdef ISNAN_TRAP
    if (std::isnan(home.tran.x) || std::isnan(home.tran.y) || std::isnan(home.tran.z) ||
	std::isnan(home.a) || std::isnan(home.b) || std::isnan(home.c) ||
	std::isnan(home.u) || std::isnan(home.v) || std::isnan(home.w)) {
	printf("std::isnan error in emcTrajSetHome()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_SET_WORLD_HOME;
    emcmotCommand.pos = home;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f) returned %d\n", 
          __FUNCTION__, home.tran.x, home.tran.y, home.tran.z, home.a, home.b, home.c, 
          home.u, home.v, home.w, retval);
    }
    return retval;
}

int emcTrajSetScale(double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    emcmotCommand.command = EMCMOT_FEED_SCALE;
    emcmotCommand.scale = scale;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetRapidScale(double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    emcmotCommand.command = EMCMOT_RAPID_SCALE;
    emcmotCommand.scale = scale;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetSpindleScale(int spindle, double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    emcmotCommand.command = EMCMOT_SPINDLE_SCALE;
    emcmotCommand.scale = scale;
    emcmotCommand.spindle = spindle;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetFOEnable(unsigned char mode)
{
    emcmotCommand.command = EMCMOT_FS_ENABLE;
    emcmotCommand.mode = mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetFHEnable(unsigned char mode)
{
    emcmotCommand.command = EMCMOT_FH_ENABLE;
    emcmotCommand.mode = mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetSOEnable(unsigned char mode)
{
    emcmotCommand.command = EMCMOT_SS_ENABLE;
    emcmotCommand.mode = mode;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetAFEnable(unsigned char enable)
{
    emcmotCommand.command = EMCMOT_AF_ENABLE;

    if ( enable ) {
	emcmotCommand.flags = 1;
    } else {
	emcmotCommand.flags = 0;
    }
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetMotionId(int id)
{

    if (EMC_DEBUG_MOTION_TIME & emc_debug) {
	if (id != TrajConfig.MotionId) {
	    rcs_print("Outgoing motion id is %d.\n", id);
	}
    }

    TrajConfig.MotionId = id;

    return 0;
}

int emcTrajInit()
{
    int retval = 0;

    TrajConfig.Inited = 0;
    TrajConfig.Joints = 0;
    TrajConfig.MaxAccel = DBL_MAX;
    TrajConfig.AxisMask = 0;
    TrajConfig.LinearUnits = 1.0;
    TrajConfig.AngularUnits = 1.0;
    TrajConfig.MotionId = 0;
    TrajConfig.MaxVel = DEFAULT_TRAJ_MAX_VELOCITY;

    // init emcmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(emc_inifile);
	if (0 != usrmotInit("emc2_task")) {
	    return -1;
	}
    }
    TrajConfig.Inited = 1;
    // initialize parameters from INI file
    if (0 != iniTraj(emc_inifile)) {
	retval = -1;
    }
    return retval;
}

int emcTrajHalt()
{
    TrajConfig.Inited = 0;

    if (!JointOrTrajInited()) {
	usrmotExit();		// ours is final exit
    }

    return 0;
}

int emcTrajEnable()
{
    emcmotCommand.command = EMCMOT_ENABLE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajDisable()
{
    emcmotCommand.command = EMCMOT_DISABLE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajAbort()
{
    emcmotCommand.command = EMCMOT_ABORT;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajPause()
{
    emcmotCommand.command = EMCMOT_PAUSE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajReverse()
{
    emcmotCommand.command = EMCMOT_REVERSE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajForward()
{
    emcmotCommand.command = EMCMOT_FORWARD;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajStep()
{
    emcmotCommand.command = EMCMOT_STEP;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajResume()
{
    emcmotCommand.command = EMCMOT_RESUME;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajDelay(double /*delay*/)
{
    /* nothing need be done here - it's done in task controller */

    return 0;
}

double emcTrajGetLinearUnits()
{
    return TrajConfig.LinearUnits;
}

double emcTrajGetAngularUnits()
{
    return TrajConfig.AngularUnits;
}

int emcTrajSetOffset(EmcPose tool_offset)
{
    emcmotCommand.command = EMCMOT_SET_OFFSET;
    emcmotCommand.tool_offset = tool_offset;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetSpindleSync(int spindle, double fpr, bool wait_for_index)
{
    emcmotCommand.command = EMCMOT_SET_SPINDLESYNC;
    emcmotCommand.spindle = spindle;
    emcmotCommand.spindlesync = fpr;
    emcmotCommand.flags = wait_for_index;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajSetTermCond(int cond, double tolerance)
{
    emcmotCommand.command = EMCMOT_SET_TERM_COND;
    // Direct passthrough since TP can handle the distinction now
    emcmotCommand.termCond = cond;
    emcmotCommand.tolerance = tolerance;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajLinearMove(EmcPose end, int type, double vel, double ini_maxvel, double acc,
                      int indexer_jnum)
{
#ifdef ISNAN_TRAP
    if (std::isnan(end.tran.x) || std::isnan(end.tran.y) || std::isnan(end.tran.z) ||
        std::isnan(end.a) || std::isnan(end.b) || std::isnan(end.c) ||
        std::isnan(end.u) || std::isnan(end.v) || std::isnan(end.w)) {
	printf("std::isnan error in emcTrajLinearMove()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_SET_LINE;

    emcmotCommand.pos = end;

    emcmotCommand.id = TrajConfig.MotionId;
    emcmotCommand.tag = localEmcTrajTag;
    emcmotCommand.motion_type = type;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.turn = indexer_jnum;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajCircularMove(EmcPose end, PM_CARTESIAN center,
			PM_CARTESIAN normal, int turn, int type, double vel, double ini_maxvel, double acc)
{
#ifdef ISNAN_TRAP
    if (std::isnan(end.tran.x) || std::isnan(end.tran.y) || std::isnan(end.tran.z) ||
	std::isnan(end.a) || std::isnan(end.b) || std::isnan(end.c) ||
	std::isnan(end.u) || std::isnan(end.v) || std::isnan(end.w) ||
	std::isnan(center.x) || std::isnan(center.y) || std::isnan(center.z) ||
	std::isnan(normal.x) || std::isnan(normal.y) || std::isnan(normal.z)) {
	printf("std::isnan error in emcTrajCircularMove()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_SET_CIRCLE;

    emcmotCommand.pos = end;
    emcmotCommand.motion_type = type;

    emcmotCommand.center.x = center.x;
    emcmotCommand.center.y = center.y;
    emcmotCommand.center.z = center.z;

    emcmotCommand.normal.x = normal.x;
    emcmotCommand.normal.y = normal.y;
    emcmotCommand.normal.z = normal.z;

    emcmotCommand.turn = turn;
    emcmotCommand.id = TrajConfig.MotionId;
    emcmotCommand.tag = localEmcTrajTag;

    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajClearProbeTrippedFlag()
{
    emcmotCommand.command = EMCMOT_CLEAR_PROBE_FLAGS;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajProbe(EmcPose pos, int type, double vel, double ini_maxvel, double acc, unsigned char probe_type)
{
#ifdef ISNAN_TRAP
    if (std::isnan(pos.tran.x) || std::isnan(pos.tran.y) || std::isnan(pos.tran.z) ||
        std::isnan(pos.a) || std::isnan(pos.b) || std::isnan(pos.c) ||
        std::isnan(pos.u) || std::isnan(pos.v) || std::isnan(pos.w)) {
	printf("std::isnan error in emcTrajProbe()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_PROBE;
    emcmotCommand.pos = pos;
    emcmotCommand.id = TrajConfig.MotionId;
    emcmotCommand.tag = localEmcTrajTag;
    emcmotCommand.motion_type = type;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.probe_type = probe_type;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajRigidTap(EmcPose pos, double vel, double ini_maxvel, double acc, double scale)
{
#ifdef ISNAN_TRAP
    if (std::isnan(pos.tran.x) || std::isnan(pos.tran.y) || std::isnan(pos.tran.z)) {
	printf("std::isnan error in emcTrajRigidTap()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    emcmotCommand.command = EMCMOT_RIGID_TAP;
    emcmotCommand.pos.tran = pos.tran;
    emcmotCommand.id = TrajConfig.MotionId;
    emcmotCommand.tag = localEmcTrajTag;
    emcmotCommand.vel = vel;
    emcmotCommand.ini_maxvel = ini_maxvel;
    emcmotCommand.acc = acc;
    emcmotCommand.scale = scale;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}


static int last_id = 0;
static int last_id_printed = 0;
static RCS_STATUS last_status = RCS_STATUS::UNINITIALIZED;
static double last_id_time;

int emcTrajUpdate(EMC_TRAJ_STAT * stat)
{
    int joint, enables;

    stat->joints = TrajConfig.Joints;
    stat->spindles = TrajConfig.Spindles;
    stat->axis_mask = TrajConfig.AxisMask;
    stat->linearUnits = TrajConfig.LinearUnits;
    stat->angularUnits = TrajConfig.AngularUnits;

    stat->mode =
	emcmotStatus.
	motionFlag & EMCMOT_MOTION_TELEOP_BIT ? EMC_TRAJ_MODE::TELEOP
	: (emcmotStatus.
	   motionFlag & EMCMOT_MOTION_COORD_BIT ? EMC_TRAJ_MODE::COORD :
	   EMC_TRAJ_MODE::FREE);

    /* enabled if motion enabled and all joints enabled */
    stat->enabled = 0;		/* start at disabled */
    if (emcmotStatus.motionFlag & EMCMOT_MOTION_ENABLE_BIT) {
	for (joint = 0; joint < TrajConfig.Joints; joint++) {
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - the axis flag has been moved to the joint struct */
	    if (!emcmotStatus.axisFlag[axis] & EMCMOT_JOINT_ENABLE_BIT) {
		break;
	    }
#endif
	    /* got here, then all are enabled */
	    stat->enabled = 1;
	}
    }

    stat->inpos = emcmotStatus.motionFlag & EMCMOT_MOTION_INPOS_BIT;
    stat->queue = emcmotStatus.depth;
    stat->activeQueue = emcmotStatus.activeDepth;
    stat->queueFull = emcmotStatus.queueFull;
    stat->id = emcmotStatus.id;
    StateTag newtag(emcmotStatus.tag);
    //TODO assignment operator
    stat->tag = newtag;
    stat->motion_type = emcmotStatus.motionType;
    stat->distance_to_go = emcmotStatus.distance_to_go;
    stat->dtg = emcmotStatus.dtg;
    stat->current_vel = emcmotStatus.current_vel;
    if (EMC_DEBUG_MOTION_TIME & emc_debug) {
	if (stat->id != last_id) {
	    if (last_id != last_id_printed) {
		rcs_print("Motion id %d took %f seconds.\n", last_id,
			  etime() - last_id_time);
		last_id_printed = last_id;
	    }
	    last_id = stat->id;
	    last_id_time = etime();
	}
    }

    stat->paused = emcmotStatus.paused;
    stat->scale = emcmotStatus.feed_scale;
    stat->rapid_scale = emcmotStatus.rapid_scale;

    stat->position = emcmotStatus.carte_pos_cmd;

    stat->actualPosition = emcmotStatus.carte_pos_fb;

    stat->velocity = emcmotStatus.vel;
    stat->acceleration = emcmotStatus.acc;
    stat->maxAcceleration = TrajConfig.MaxAccel;

    if (emcmotStatus.motionFlag & EMCMOT_MOTION_ERROR_BIT) {
	stat->status = RCS_STATUS::ERROR;
    } else if (stat->inpos && (stat->queue == 0)) {
	stat->status = RCS_STATUS::DONE;
    } else {
	stat->status = RCS_STATUS::EXEC;
    }

    if (EMC_DEBUG_MOTION_TIME & emc_debug) {
	if (stat->status == RCS_STATUS::DONE && last_status != RCS_STATUS::DONE
	    && stat->id != last_id_printed) {
	    rcs_print("Motion id %d took %f seconds.\n", last_id,
		      etime() - last_id_time);
	    last_id_printed = last_id = stat->id;
	    last_id_time = etime();
	}
    }

    stat->probedPosition = emcmotStatus.probedPos;

    stat->probeval = emcmotStatus.probeVal;
    stat->probing = emcmotStatus.probing;
    stat->probe_tripped = emcmotStatus.probeTripped;
    
    if (emcmotStatus.motionFlag & EMCMOT_MOTION_COORD_BIT)
        enables = emcmotStatus.enables_queued;
    else
        enables = emcmotStatus.enables_new;
    
    stat->feed_override_enabled = enables & FS_ENABLED;
    stat->adaptive_feed_enabled = enables & AF_ENABLED;
    stat->feed_hold_enabled = enables & FH_ENABLED;

    if (new_config) {
	stat->cycleTime = emcmotConfig.trajCycleTime;
	stat->kinematics_type = emcmotConfig.kinType;
	stat->maxVelocity = emcmotConfig.limitVel;
    }

    return 0;
}

int emcPositionLoad() {
    double positions[EMCMOT_MAX_JOINTS];
    IniFile ini;
    ini.Open(emc_inifile);
    auto posfile = ini.Find("POSITION_FILE", "TRAJ");
    ini.Close();
    if(!posfile || !posfile.value()[0]) return 0;
    FILE *f = fopen(*posfile, "r");
    if(!f) return 0;
    for(int i=0; i<EMCMOT_MAX_JOINTS; i++) {
	int r = fscanf(f, "%lf", &positions[i]);
	if(r != 1) {
            fclose(f);
            rcs_print("%s: failed to load joint %d position from %s, ignoring\n", __FUNCTION__, i, *posfile);
            return -1;
        }
    }
    fclose(f);
    int result = 0;
    for(int i=0; i<EMCMOT_MAX_JOINTS; i++) {
	if(emcJointSetMotorOffset(i, -positions[i]) != 0) {
            rcs_print("%s: failed to set joint %d position (%.6f) from %s, ignoring\n", __FUNCTION__, i, positions[i], *posfile);
            result = -1;
        }
    }
    return result;
}


int emcPositionSave() {
    IniFile ini;
    std::optional<const char*> posfile;

    ini.Open(emc_inifile);
    try {
        posfile = ini.Find("POSITION_FILE", "TRAJ");
    } catch (IniFile::Exception e) {
        ini.Close();
        return -1;
    }
    ini.Close();

    if(!posfile || !posfile.value()[0]) return 0;
    // like the var file, make sure the posfile is recreated according to umask
    unlink(*posfile);
    FILE *f = fopen(*posfile, "w");
    if(!f) return -1;
    for(int i=0; i<EMCMOT_MAX_JOINTS; i++) {
	int r = fprintf(f, "%.17f\n", emcmotStatus.joint_status[i].pos_fb);
	if(r < 0) { fclose(f); return -1; }
    }
    fclose(f);
    return 0;
}

// EMC_MOTION functions

// This function gets called by Task from emctask_startup().
// emctask_startup() calls this function in a loop, retrying it until
// it succeeds or until the retries time out.
int emcMotionInit()
{
    int r;
    int joint, axis, spindle;
    
    r = emcTrajInit(); // we want to check Traj first, the sane defaults for units are there
    // it also determines the number of existing joints, and axes
    if (r != 0) {
        rcs_print("%s: emcTrajInit failed\n", __FUNCTION__);
        return -1;
    }

    for (joint = 0; joint < TrajConfig.Joints; joint++) {
	if (0 != emcJointInit(joint)) {
            rcs_print("%s: emcJointInit(%d) failed\n", __FUNCTION__, joint);
            return -1;
	}
    }

    for (axis = 0; axis < EMCMOT_MAX_AXIS; axis++) {
        if (TrajConfig.AxisMask & (1<<axis)) {
	    if (0 != emcAxisInit(axis)) {
                rcs_print("%s: emcAxisInit(%d) failed\n", __FUNCTION__, axis);
                return -1;
	    }
	}
	}

    for (spindle = 0; spindle < TrajConfig.Spindles; spindle++) {
	    if (0 != emcSpindleInit(spindle)) {
                rcs_print("%s: emcSpindleInit(%d) failed\n", __FUNCTION__, spindle);
                return -1;
	    }
	}


    // Ignore errors from emcPositionLoad(), because what are you going to do?
    (void)emcPositionLoad();
    return 0;
}

int emcMotionHalt()
{
    int r1, r2, r3, r4, r5;
    int t;

    r1 = -1;
    for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	if (0 == emcJointHalt(t)) {
	    r1 = 0;		// at least one is okay
	}
    }

    r2 = emcTrajDisable();
    r3 = emcTrajHalt();
    r4 = emcPositionSave();
    r5 = ini_hal_exit();

    return (r1 == 0 && r2 == 0 && r3 == 0 && r4 == 0 && r5 == 0) ? 0 : -1;
}

int emcMotionAbort()
{
    int r1;
    int r2;
    int r3 = 0;
    int t;

    r1 = -1;
    for (t = 0; t < EMCMOT_MAX_JOINTS; t++) {
	if (0 == emcJogAbort(t)) {
	    r1 = 0;		// at least one is okay
	}
    }

    r2 = emcTrajAbort();

    return (r1 == 0 && r2 == 0 && r3 == 0) ? 0 : -1;
}

int emcMotionSetDebug(int debug)
{
    emcmotCommand.command = EMCMOT_SET_DEBUG;
    emcmotCommand.debug = debug;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

/*! \function emcMotionSetAout()

    This function sends a EMCMOT_SET_AOUT message to the motion controller.
    That one plans a AOUT command when motion starts or right now.

    @parameter index   which output gets modified
    @parameter now     whether change is immediate or synched with motion
    @parameter start   value set at start of motion
    @parameter end     value set at end of motion
*/
int emcMotionSetAout(unsigned char index, double start, double end, unsigned char now)
{
    emcmotCommand.command = EMCMOT_SET_AOUT;
    emcmotCommand.now = now;
    emcmotCommand.out = index;
  /*! \todo FIXME-- if this works, set up some dedicated cmd fields instead of
     borrowing these */
    emcmotCommand.minLimit = start;
    emcmotCommand.maxLimit = end;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

/*! \function emcMotionSetDout()

    This function sends a EMCMOT_SET_DOUT message to the motion controller.
    That one plans a DOUT command when motion starts or right now.

    @parameter index   which output gets modified
    @parameter now     whether change is immediate or synched with motion
    @parameter start   value set at start of motion
    @parameter end     value set at end of motion
*/
int emcMotionSetDout(unsigned char index, unsigned char start,
		     unsigned char end, unsigned char now)
{
    emcmotCommand.command = EMCMOT_SET_DOUT;
    emcmotCommand.now = now;
    emcmotCommand.out = index;
    emcmotCommand.start = start;
    emcmotCommand.end = end;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleSetParams(int spindle, double max_pos, double min_pos, double max_neg,
			   double min_neg, double search_vel, double home_angle, int sequence, double increment)
{

    if (spindle < 0 || spindle >= EMCMOT_MAX_SPINDLES) {
	return 0;
    }

    emcmotCommand.command = EMCMOT_SET_SPINDLE_PARAMS;
    emcmotCommand.spindle = spindle;
    emcmotCommand.maxLimit = max_pos;
    emcmotCommand.minLimit = min_neg;
    emcmotCommand.min_pos_speed = min_pos;
    emcmotCommand.max_neg_speed = max_neg;
    emcmotCommand.home = home_angle;
    emcmotCommand.search_vel = search_vel;
    emcmotCommand.home_sequence = sequence;
    emcmotCommand.offset = increment;

    int retval = usrmotWriteEmcmotCommand(&emcmotCommand);

    if (emc_debug & EMC_DEBUG_CONFIG) {
        rcs_print("%s(%d, %e, %e, %e, %e, %f, %f, %i, %f) returned %d\n",
          __FUNCTION__, spindle, max_pos, min_pos, max_neg, min_neg, search_vel, home_angle,
          sequence, increment, retval);
    }
    return retval;
}

int emcSpindleAbort(int spindle)
{
    return emcSpindleOff(spindle);
}

int emcSpindleSpeed(int spindle, double speed, double css_factor, double offset)
{
    emcmotCommand.command = EMCMOT_SPINDLE_ON;
    emcmotCommand.spindle = spindle;
    emcmotCommand.vel = speed;
    emcmotCommand.ini_maxvel = css_factor;
    emcmotCommand.acc = offset;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleOrient(int spindle, double orientation, int mode)
{
    emcmotCommand.command = EMCMOT_SPINDLE_ORIENT;
    emcmotCommand.spindle = spindle;
    emcmotCommand.orientation = orientation;
    emcmotCommand.mode = mode;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}


int emcSpindleOn(int spindle, double speed, double css_factor, double offset, int wait_for_at_speed)
{
    emcmotCommand.command = EMCMOT_SPINDLE_ON;
    emcmotCommand.spindle = spindle;
    emcmotCommand.state = 1;
    emcmotCommand.vel = speed;
    emcmotCommand.ini_maxvel = css_factor;
    emcmotCommand.acc = offset;
    emcmotCommand.wait_for_spindle_at_speed = wait_for_at_speed;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleOff(int spindle)
{
    emcmotCommand.command = EMCMOT_SPINDLE_OFF;
    emcmotCommand.state = 0;
    emcmotCommand.spindle = spindle;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleBrakeRelease(int spindle)
{
    emcmotCommand.command = EMCMOT_SPINDLE_BRAKE_RELEASE;
    emcmotCommand.spindle = spindle;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleBrakeEngage(int spindle)
{
    emcmotCommand.command = EMCMOT_SPINDLE_BRAKE_ENGAGE;
    emcmotCommand.spindle = spindle;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleIncrease(int spindle)
{
    emcmotCommand.command = EMCMOT_SPINDLE_INCREASE;
    emcmotCommand.spindle = spindle;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleDecrease(int spindle)
{
    emcmotCommand.command = EMCMOT_SPINDLE_DECREASE;
    emcmotCommand.spindle = spindle;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSpindleConstant(int /*spindle*/)
{
    return 0; // nothing to do
}

int emcSpindleUpdate(EMC_SPINDLE_STAT stat[], int num_spindles){
	int s;
	int enables;
    if (emcmotStatus.motionFlag & EMCMOT_MOTION_COORD_BIT)
        enables = emcmotStatus.enables_queued;
    else
        enables = emcmotStatus.enables_new;

    for (s = 0; s < num_spindles; s++){
		stat[s].spindle_override_enabled = enables & SS_ENABLED;
		stat[s].enabled = emcmotStatus.spindle_status[s].speed != 0;
		stat[s].speed = emcmotStatus.spindle_status[s].speed;
		stat[s].brake = emcmotStatus.spindle_status[s].brake;
		stat[s].direction = emcmotStatus.spindle_status[s].direction;
		stat[s].orient_state = emcmotStatus.spindle_status[s].orient_state;
		stat[s].orient_fault = emcmotStatus.spindle_status[s].orient_fault;
		stat[s].spindle_scale = emcmotStatus.spindle_status[s].scale;
    }
    return 0;
}

int emcMotionUpdate(EMC_MOTION_STAT * stat)
{
    int r1, r2, r3, r4;
    int joint;
    int error;
    int exec;
    int dio, aio, num_error;

    // read the emcmot status
    if (0 != usrmotReadEmcmotStatus(&emcmotStatus)) {
	return -1;
    }
    new_config = 0;
    if (emcmotStatus.config_num != emcmotConfig.config_num) {
	if (0 != usrmotReadEmcmotConfig(&emcmotConfig)) {
	    return -1;
	}
	new_config = 1;
    }

    if (get_emcmot_internal_info) {
	if (0 != usrmotReadEmcmotInternal(&emcmotInternal)) {
	    return -1;
	}
    }
    // read the emcmot error
    if (0 != usrmotReadEmcmotError(errorString)) {
	// no error, so ignore
    } else {
	// an error to report
	emcOperatorError("%s", errorString);
    }

    // save the heartbeat and command number locally,
    // for use with emcMotionUpdate
    localMotionCommandType = emcmotStatus.commandEcho;	/*! \todo FIXME-- not NML one! */
    localMotionEchoSerialNumber = emcmotStatus.commandNumEcho;

    r3 = emcTrajUpdate(&stat->traj);
    r1 = emcJointUpdate(&stat->joint[0], stat->traj.joints);
    r2 = emcAxisUpdate(&stat->axis[0], stat->traj.axis_mask);
    r3 = emcTrajUpdate(&stat->traj);
    r4 = emcSpindleUpdate(&stat->spindle[0], stat->traj.spindles);
    stat->command_type = localMotionCommandType;
    stat->echo_serial_number = localMotionEchoSerialNumber;
    stat->debug = emcmotConfig.debug;

    for (dio = 0; dio < EMCMOT_MAX_DIO; dio++) {
	stat->synch_di[dio] = emcmotStatus.synch_di[dio];
	stat->synch_do[dio] = emcmotStatus.synch_do[dio];
    }

    for (aio = 0; aio < EMCMOT_MAX_AIO; aio++) {
	stat->analog_input[aio] = emcmotStatus.analog_input[aio];
	stat->analog_output[aio] = emcmotStatus.analog_output[aio];
    }

    for (num_error = 0; num_error < EMCMOT_MAX_MISC_ERROR; num_error++){
      stat->misc_error[num_error] = emcmotStatus.misc_error[num_error];
    }

    stat->jogging_active = emcmotStatus.jogging_active;
    stat->numExtraJoints = emcmotStatus.numExtraJoints;

    // set the status flag
    error = 0;
    exec = 0;

    // FIXME-AJ: joints not axes
    for (joint = 0; joint < stat->traj.joints; joint++) {
	if (stat->joint[joint].status == RCS_STATUS::ERROR) {
	    error = 1;
	    break;
	}
	if (stat->joint[joint].status == RCS_STATUS::EXEC) {
	    exec = 1;
	    break;
	}
    }
    if (stat->traj.status == RCS_STATUS::ERROR) {
	error = 1;
    } else if (stat->traj.status == RCS_STATUS::EXEC) {
	exec = 1;
    }

    if (error) {
	stat->status = RCS_STATUS::ERROR;
    } else if (exec) {
	stat->status = RCS_STATUS::EXEC;
    } else {
	stat->status = RCS_STATUS::DONE;
    }
    return (r1 == 0 && r2 == 0 && r3 == 0 && r4 == 0) ? 0 : -1;
}

int emcSetupArcBlends(int arcBlendEnable,
        int arcBlendFallbackEnable,
        int arcBlendOptDepth,
        int arcBlendGapCycles,
        double arcBlendRampFreq,
        double arcBlendTangentKinkRatio) {

    emcmotCommand.command = EMCMOT_SETUP_ARC_BLENDS;
    emcmotCommand.arcBlendEnable = arcBlendEnable;
    emcmotCommand.arcBlendFallbackEnable = arcBlendFallbackEnable;
    emcmotCommand.arcBlendOptDepth = arcBlendOptDepth;
    emcmotCommand.arcBlendGapCycles = arcBlendGapCycles;
    emcmotCommand.arcBlendRampFreq = arcBlendRampFreq;
    emcmotCommand.arcBlendTangentKinkRatio = arcBlendTangentKinkRatio;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSetMaxFeedOverride(double maxFeedScale) {
    emcmotCommand.command = EMCMOT_SET_MAX_FEED_OVERRIDE;
    emcmotCommand.maxFeedScale = maxFeedScale;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcSetProbeErrorInhibit(int j_inhibit, int h_inhibit) {
    emcmotCommand.command = EMCMOT_SET_PROBE_ERR_INHIBIT;
    emcmotCommand.probe_jog_err_inhibit = j_inhibit;
    emcmotCommand.probe_home_err_inhibit = h_inhibit;
    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcGetExternalOffsetApplied(void) {
    return emcmotStatus.external_offsets_applied;
}

EmcPose emcGetExternalOffsets(void) {
    return emcmotStatus.eoffset_pose;
}
