/********************************************************************
* Description: emc.hh
*   Declarations for EMC NML vocabulary
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef EMC_HH
#define EMC_HH

#include "config.h"
#include "emcglb.h"		// EMC_AXIS_MAX
#include "nml_type.hh"
#include "motion_types.h"
#include <stdint.h>

// Forward class declarations
class EMC_AXIS_STAT;
class EMC_TRAJ_STAT;
class EMC_MOTION_STAT;
class EMC_TASK_STAT;
class EMC_TOOL_STAT;
class EMC_AUX_STAT;
class EMC_SPINDLE_STAT;
class EMC_COOLANT_STAT;
class EMC_LUBE_STAT;
class EMC_IO_STAT;
class EMC_STAT;
class CMS;
class RCS_CMD_CHANNEL;
class RCS_STAT_CHANNEL;
class NML;
struct EmcPose;
struct PM_CARTESIAN;

// ---------------------
// EMC TYPE DECLARATIONS
// ---------------------

// NML for base EMC

#define EMC_OPERATOR_ERROR_TYPE                      ((NMLTYPE) 11)
#define EMC_OPERATOR_TEXT_TYPE                       ((NMLTYPE) 12)
#define EMC_OPERATOR_DISPLAY_TYPE                    ((NMLTYPE) 13)

#define EMC_NULL_TYPE                                ((NMLTYPE) 21)

#define EMC_SET_DEBUG_TYPE                           ((NMLTYPE) 22)

#define EMC_SYSTEM_CMD_TYPE                          ((NMLTYPE) 30)

// NML for EMC_AXIS

#define EMC_AXIS_SET_AXIS_TYPE                       ((NMLTYPE) 101)
#define EMC_AXIS_SET_UNITS_TYPE                      ((NMLTYPE) 102)
/* gap because of deleted message types */




#define EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE         ((NMLTYPE) 107)
#define EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE         ((NMLTYPE) 108)
#define EMC_AXIS_SET_FERROR_TYPE                     ((NMLTYPE) 111)
#define EMC_AXIS_SET_HOMING_PARAMS_TYPE              ((NMLTYPE) 112)
// gap because of deleted message types

#define EMC_AXIS_SET_MIN_FERROR_TYPE                 ((NMLTYPE) 115)
#define EMC_AXIS_SET_MAX_VELOCITY_TYPE               ((NMLTYPE) 116)
// gap because of deleted message types

#define EMC_AXIS_INIT_TYPE                           ((NMLTYPE) 118)
#define EMC_AXIS_HALT_TYPE                           ((NMLTYPE) 119)
#define EMC_AXIS_ABORT_TYPE                          ((NMLTYPE) 120)
#define EMC_AXIS_ENABLE_TYPE                         ((NMLTYPE) 121)
#define EMC_AXIS_DISABLE_TYPE                        ((NMLTYPE) 122)
#define EMC_AXIS_HOME_TYPE                           ((NMLTYPE) 123)
#define EMC_AXIS_UNHOME_TYPE                           ((NMLTYPE) 135)
#define EMC_AXIS_JOG_TYPE                            ((NMLTYPE) 124)
#define EMC_AXIS_INCR_JOG_TYPE                       ((NMLTYPE) 125)
#define EMC_AXIS_ABS_JOG_TYPE                        ((NMLTYPE) 126)
#define EMC_AXIS_ACTIVATE_TYPE                       ((NMLTYPE) 127)
#define EMC_AXIS_DEACTIVATE_TYPE                     ((NMLTYPE) 128)
#define EMC_AXIS_OVERRIDE_LIMITS_TYPE                ((NMLTYPE) 129)
#define EMC_AXIS_LOAD_COMP_TYPE                      ((NMLTYPE) 131)
// gap because of deleted message type (EMC_AXIS_ALTER_TYPE)
#define EMC_AXIS_SET_BACKLASH_TYPE                   ((NMLTYPE) 134)

#define EMC_AXIS_STAT_TYPE                           ((NMLTYPE) 199)

// NML for EMC_TRAJ

// defs for termination conditions

#define EMC_TRAJ_TERM_COND_STOP  0
#define EMC_TRAJ_TERM_COND_EXACT 1
#define EMC_TRAJ_TERM_COND_BLEND 2

#define EMC_TRAJ_SET_AXES_TYPE                       ((NMLTYPE) 201)
#define EMC_TRAJ_SET_UNITS_TYPE                      ((NMLTYPE) 202)
#define EMC_TRAJ_SET_CYCLE_TIME_TYPE                 ((NMLTYPE) 203)
#define EMC_TRAJ_SET_MODE_TYPE                       ((NMLTYPE) 204)
#define EMC_TRAJ_SET_VELOCITY_TYPE                   ((NMLTYPE) 205)
#define EMC_TRAJ_SET_ACCELERATION_TYPE               ((NMLTYPE) 206)
#define EMC_TRAJ_SET_MAX_VELOCITY_TYPE               ((NMLTYPE) 207)
#define EMC_TRAJ_SET_MAX_ACCELERATION_TYPE           ((NMLTYPE) 208)
#define EMC_TRAJ_SET_SCALE_TYPE                      ((NMLTYPE) 209)
#define EMC_TRAJ_SET_RAPID_SCALE_TYPE                ((NMLTYPE) 238)
#define EMC_TRAJ_SET_MOTION_ID_TYPE                  ((NMLTYPE) 210)

#define EMC_TRAJ_INIT_TYPE                           ((NMLTYPE) 211)
#define EMC_TRAJ_HALT_TYPE                           ((NMLTYPE) 212)
#define EMC_TRAJ_ENABLE_TYPE                         ((NMLTYPE) 213)
#define EMC_TRAJ_DISABLE_TYPE                        ((NMLTYPE) 214)
#define EMC_TRAJ_ABORT_TYPE                          ((NMLTYPE) 215)
#define EMC_TRAJ_PAUSE_TYPE                          ((NMLTYPE) 216)
#define EMC_TRAJ_STEP_TYPE                           ((NMLTYPE) 217)
#define EMC_TRAJ_RESUME_TYPE                         ((NMLTYPE) 218)
#define EMC_TRAJ_DELAY_TYPE                          ((NMLTYPE) 219)
#define EMC_TRAJ_LINEAR_MOVE_TYPE                    ((NMLTYPE) 220)
#define EMC_TRAJ_CIRCULAR_MOVE_TYPE                  ((NMLTYPE) 221)
#define EMC_TRAJ_SET_TERM_COND_TYPE                  ((NMLTYPE) 222)
#define EMC_TRAJ_SET_OFFSET_TYPE                     ((NMLTYPE) 223)
#define EMC_TRAJ_SET_G5X_TYPE                        ((NMLTYPE) 224)
#define EMC_TRAJ_SET_HOME_TYPE                       ((NMLTYPE) 225)
#define EMC_TRAJ_SET_ROTATION_TYPE                   ((NMLTYPE) 226)
#define EMC_TRAJ_SET_G92_TYPE                        ((NMLTYPE) 227)
/* gap because of removed messages */

#define EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE       ((NMLTYPE) 228)
#define EMC_TRAJ_PROBE_TYPE                          ((NMLTYPE) 229)
#define EMC_TRAJ_SET_TELEOP_ENABLE_TYPE              ((NMLTYPE) 230)
#define EMC_TRAJ_SET_TELEOP_VECTOR_TYPE              ((NMLTYPE) 231)
#define EMC_TRAJ_SET_SPINDLESYNC_TYPE                ((NMLTYPE) 232)
#define EMC_TRAJ_SET_SPINDLE_SCALE_TYPE              ((NMLTYPE) 233)
#define EMC_TRAJ_SET_FO_ENABLE_TYPE                  ((NMLTYPE) 234)
#define EMC_TRAJ_SET_SO_ENABLE_TYPE                  ((NMLTYPE) 235)
#define EMC_TRAJ_SET_FH_ENABLE_TYPE                  ((NMLTYPE) 236)
#define EMC_TRAJ_RIGID_TAP_TYPE                      ((NMLTYPE) 237)

#define EMC_TRAJ_STAT_TYPE                           ((NMLTYPE) 299)

// EMC_MOTION aggregate class type declaration

#define EMC_MOTION_INIT_TYPE                         ((NMLTYPE) 301)
#define EMC_MOTION_HALT_TYPE                         ((NMLTYPE) 302)
#define EMC_MOTION_ABORT_TYPE                        ((NMLTYPE) 303)
#define EMC_MOTION_SET_AOUT_TYPE                     ((NMLTYPE) 304)
#define EMC_MOTION_SET_DOUT_TYPE                     ((NMLTYPE) 305)
#define EMC_MOTION_ADAPTIVE_TYPE                     ((NMLTYPE) 306)

#define EMC_MOTION_STAT_TYPE                         ((NMLTYPE) 399)

// NML for EMC_TASK

#define EMC_TASK_INIT_TYPE                           ((NMLTYPE) 501)
#define EMC_TASK_HALT_TYPE                           ((NMLTYPE) 502)
#define EMC_TASK_ABORT_TYPE                          ((NMLTYPE) 503)
#define EMC_TASK_SET_MODE_TYPE                       ((NMLTYPE) 504)
#define EMC_TASK_SET_STATE_TYPE                      ((NMLTYPE) 505)
#define EMC_TASK_PLAN_OPEN_TYPE                      ((NMLTYPE) 506)
#define EMC_TASK_PLAN_RUN_TYPE                       ((NMLTYPE) 507)
#define EMC_TASK_PLAN_READ_TYPE                      ((NMLTYPE) 508)
#define EMC_TASK_PLAN_EXECUTE_TYPE                   ((NMLTYPE) 509)
#define EMC_TASK_PLAN_PAUSE_TYPE                     ((NMLTYPE) 510)
#define EMC_TASK_PLAN_STEP_TYPE                      ((NMLTYPE) 511)
#define EMC_TASK_PLAN_RESUME_TYPE                    ((NMLTYPE) 512)
#define EMC_TASK_PLAN_END_TYPE                       ((NMLTYPE) 513)
#define EMC_TASK_PLAN_CLOSE_TYPE                     ((NMLTYPE) 514)
#define EMC_TASK_PLAN_INIT_TYPE                      ((NMLTYPE) 515)
#define EMC_TASK_PLAN_SYNCH_TYPE                     ((NMLTYPE) 516)
#define EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE         ((NMLTYPE) 517)
#define EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE          ((NMLTYPE) 518)
#define EMC_TASK_PLAN_OPTIONAL_STOP_TYPE             ((NMLTYPE) 519)

#define EMC_TASK_STAT_TYPE                           ((NMLTYPE) 599)

// EMC_TOOL type declarations

#define EMC_TOOL_INIT_TYPE                           ((NMLTYPE) 1101)
#define EMC_TOOL_HALT_TYPE                           ((NMLTYPE) 1102)
#define EMC_TOOL_ABORT_TYPE                          ((NMLTYPE) 1103)
#define EMC_TOOL_PREPARE_TYPE                        ((NMLTYPE) 1104)
#define EMC_TOOL_LOAD_TYPE                           ((NMLTYPE) 1105)
#define EMC_TOOL_UNLOAD_TYPE                         ((NMLTYPE) 1106)
#define EMC_TOOL_LOAD_TOOL_TABLE_TYPE                ((NMLTYPE) 1107)
#define EMC_TOOL_SET_OFFSET_TYPE                     ((NMLTYPE) 1108)
#define EMC_TOOL_SET_NUMBER_TYPE                     ((NMLTYPE) 1109)
// the following message is sent to io at the very start of an M6
// even before emccanon issues the move to toolchange position
#define EMC_TOOL_START_CHANGE_TYPE                   ((NMLTYPE) 1110)

#define EMC_EXEC_PLUGIN_CALL_TYPE                   ((NMLTYPE) 1112)
#define EMC_IO_PLUGIN_CALL_TYPE                   ((NMLTYPE) 1113)
#define EMC_TOOL_STAT_TYPE                           ((NMLTYPE) 1199)

// EMC_AUX type declarations

/* removed #define EMC_AUX_INIT_TYPE                             ((NMLTYPE) 1201) */
/* removed #define EMC_AUX_HALT_TYPE                             ((NMLTYPE) 1202) */
/* removed #define EMC_AUX_ABORT_TYPE                            ((NMLTYPE) 1203) */
/* removed #define EMC_AUX_DIO_WRITE_TYPE                        ((NMLTYPE) 1204) */
/* removed #define EMC_AUX_AIO_WRITE_TYPE                        ((NMLTYPE) 1205) */
#define EMC_AUX_ESTOP_ON_TYPE                         ((NMLTYPE) 1206)
#define EMC_AUX_ESTOP_OFF_TYPE                        ((NMLTYPE) 1207)
#define EMC_AUX_ESTOP_RESET_TYPE                      ((NMLTYPE) 1208)
#define EMC_AUX_INPUT_WAIT_TYPE                       ((NMLTYPE) 1209)

#define EMC_AUX_STAT_TYPE                             ((NMLTYPE) 1299)

// EMC_SPINDLE type declarations

/* removed #define EMC_SPINDLE_INIT_TYPE                        ((NMLTYPE) 1301) */
/* removed #define EMC_SPINDLE_HALT_TYPE                        ((NMLTYPE) 1302) */
/* removed #define EMC_SPINDLE_ABORT_TYPE                       ((NMLTYPE) 1303) */
#define EMC_SPINDLE_ON_TYPE                          ((NMLTYPE) 1304)
#define EMC_SPINDLE_OFF_TYPE                         ((NMLTYPE) 1305)
/* removed #define EMC_SPINDLE_FORWARD_TYPE                     ((NMLTYPE) 1306) */
/* removed #define EMC_SPINDLE_REVERSE_TYPE                     ((NMLTYPE) 1307) */
/* removed #define EMC_SPINDLE_STOP_TYPE                        ((NMLTYPE) 1308) */
#define EMC_SPINDLE_INCREASE_TYPE                    ((NMLTYPE) 1309)
#define EMC_SPINDLE_DECREASE_TYPE                    ((NMLTYPE) 1310)
#define EMC_SPINDLE_CONSTANT_TYPE                    ((NMLTYPE) 1311)
#define EMC_SPINDLE_BRAKE_RELEASE_TYPE               ((NMLTYPE) 1312)
#define EMC_SPINDLE_BRAKE_ENGAGE_TYPE                ((NMLTYPE) 1313)
/* removed #define EMC_SPINDLE_ENABLE_TYPE                      ((NMLTYPE) 1314) */
/* removed #define EMC_SPINDLE_DISABLE_TYPE                     ((NMLTYPE) 1315) */
#define EMC_SPINDLE_SPEED_TYPE                       ((NMLTYPE) 1316)
#define EMC_SPINDLE_ORIENT_TYPE                      ((NMLTYPE) 1317)
#define EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE        ((NMLTYPE) 1318)

#define EMC_SPINDLE_STAT_TYPE                        ((NMLTYPE) 1399)

// EMC_COOLANT type declarations

/* removed #define EMC_COOLANT_INIT_TYPE                        ((NMLTYPE) 1401) */
/* removed #define EMC_COOLANT_HALT_TYPE                        ((NMLTYPE) 1402) */
/* removed #define EMC_COOLANT_ABORT_TYPE                       ((NMLTYPE) 1403) */
#define EMC_COOLANT_MIST_ON_TYPE                     ((NMLTYPE) 1404)
#define EMC_COOLANT_MIST_OFF_TYPE                    ((NMLTYPE) 1405)
#define EMC_COOLANT_FLOOD_ON_TYPE                    ((NMLTYPE) 1406)
#define EMC_COOLANT_FLOOD_OFF_TYPE                   ((NMLTYPE) 1407)

#define EMC_COOLANT_STAT_TYPE                        ((NMLTYPE) 1499)

// EMC_LUBE type declarations

/* removed #define EMC_LUBE_INIT_TYPE                           ((NMLTYPE) 1501) */
/* removed #define EMC_LUBE_HALT_TYPE                           ((NMLTYPE) 1502) */
/* removed #define EMC_LUBE_ABORT_TYPE                          ((NMLTYPE) 1503) */
#define EMC_LUBE_ON_TYPE                             ((NMLTYPE) 1504)
#define EMC_LUBE_OFF_TYPE                            ((NMLTYPE) 1505)

#define EMC_LUBE_STAT_TYPE                           ((NMLTYPE) 1599)

// EMC IO point configuration declarations

/* removed #define EMC_SET_DIO_INDEX_TYPE                       ((NMLTYPE) 5001) */
/* removed #define EMC_SET_AIO_INDEX_TYPE                       ((NMLTYPE) 5002) */

// EMC_IO aggregate class type declaration

#define EMC_IO_INIT_TYPE                             ((NMLTYPE) 1601)
#define EMC_IO_HALT_TYPE                             ((NMLTYPE) 1602)
#define EMC_IO_ABORT_TYPE                            ((NMLTYPE) 1603)
#define EMC_IO_SET_CYCLE_TIME_TYPE                   ((NMLTYPE) 1604)

#define EMC_IO_STAT_TYPE                             ((NMLTYPE) 1699)

// EMC aggregate class type declaration

// these are placeholders
#define EMC_LOG_TYPE_IO_CMD      21	// command into EMC IO controller
#define EMC_LOG_TYPE_TASK_CMD    51	// command into EMC Task controller

#define EMC_INIT_TYPE                                ((NMLTYPE) 1901)
#define EMC_HALT_TYPE                                ((NMLTYPE) 1902)
#define EMC_ABORT_TYPE                               ((NMLTYPE) 1903)

#define EMC_STAT_TYPE                                ((NMLTYPE) 1999)

// types for EMC_TASK mode
enum EMC_TASK_MODE_ENUM {
    EMC_TASK_MODE_MANUAL = 1,
    EMC_TASK_MODE_AUTO = 2,
    EMC_TASK_MODE_MDI = 3
};

// types for EMC_TASK state
enum EMC_TASK_STATE_ENUM {
    EMC_TASK_STATE_ESTOP = 1,
    EMC_TASK_STATE_ESTOP_RESET = 2,
    EMC_TASK_STATE_OFF = 3,
    EMC_TASK_STATE_ON = 4
};

// types for EMC_TASK execState
enum EMC_TASK_EXEC_ENUM {
    EMC_TASK_EXEC_ERROR = 1,
    EMC_TASK_EXEC_DONE = 2,
    EMC_TASK_EXEC_WAITING_FOR_MOTION = 3,
    EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE = 4,
    EMC_TASK_EXEC_WAITING_FOR_IO = 5,
    EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO = 7,
    EMC_TASK_EXEC_WAITING_FOR_DELAY = 8,
    EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD = 9,
    EMC_TASK_EXEC_WAITING_FOR_SPINDLE_ORIENTED = 10
};

// types for EMC_TASK interpState
enum EMC_TASK_INTERP_ENUM {
    EMC_TASK_INTERP_IDLE = 1,
    EMC_TASK_INTERP_READING = 2,
    EMC_TASK_INTERP_PAUSED = 3,
    EMC_TASK_INTERP_WAITING = 4
};

// types for motion control
enum EMC_TRAJ_MODE_ENUM {
    EMC_TRAJ_MODE_FREE = 1,	// independent-axis motion,
    EMC_TRAJ_MODE_COORD = 2,	// coordinated-axis motion,
    EMC_TRAJ_MODE_TELEOP = 3	// velocity based world coordinates motion,
};

// types for emcIoAbort() reasons
enum EMC_IO_ABORT_REASON_ENUM {
	EMC_ABORT_TASK_EXEC_ERROR = 1,
	EMC_ABORT_AUX_ESTOP = 2,
	EMC_ABORT_MOTION_OR_IO_RCS_ERROR = 3,
	EMC_ABORT_TASK_STATE_OFF = 4,
	EMC_ABORT_TASK_STATE_ESTOP_RESET = 5,
	EMC_ABORT_TASK_STATE_ESTOP = 6,
	EMC_ABORT_TASK_STATE_NOT_ON = 7,
	EMC_ABORT_TASK_ABORT = 8,
	EMC_ABORT_INTERPRETER_ERROR = 9,	// interpreter failed during readahead
	EMC_ABORT_INTERPRETER_ERROR_MDI = 10,	// interpreter failed during MDI execution
	EMC_ABORT_USER = 100  // user-defined abort codes start here
};
// --------------
// EMC VOCABULARY
// --------------

// NML formatting function
extern int emcFormat(NMLTYPE type, void *buffer, CMS * cms);

// NML Symbol Lookup Function
extern const char *emc_symbol_lookup(uint32_t type);
#define emcSymbolLookup(a) emc_symbol_lookup(a)

// decls for command line args-- mains are responsible for setting these
// so that other modules can get cmd line args for ad hoc processing
extern int Argc;
extern char **Argv;

// ------------------------
// IMPLEMENTATION FUNCTIONS
// ------------------------

// implementation functions for EMC error, message types
// intended to be implemented in main() file, by writing to NML buffer

// print an error
extern int emcOperatorError(int id, const char *fmt, ...) __attribute__((format(printf,2,3)));

// print general text
extern int emcOperatorText(int id, const char *fmt, ...) __attribute__((format(printf,2,3)));

// print note to operator
extern int emcOperatorDisplay(int id, const char *fmt, ...) __attribute__((format(printf,2,3)));

// implementation functions for EMC_AXIS types

extern int emcAxisSetAxis(int axis, unsigned char axisType);
extern int emcAxisSetUnits(int axis, double units);
extern int emcAxisSetBacklash(int axis, double backlash);
extern int emcAxisSetMinPositionLimit(int axis, double limit);
extern int emcAxisSetMaxPositionLimit(int axis, double limit);
extern int emcAxisSetMotorOffset(int axis, double offset);
extern int emcAxisSetFerror(int axis, double ferror);
extern int emcAxisSetMinFerror(int axis, double ferror);
extern int emcAxisSetHomingParams(int axis, double home, double offset, double home_final_vel,
				  double search_vel, double latch_vel,
				  int use_index, int ignore_limits,
				  int is_shared, int home_sequence, int volatile_home, int locking_indexer);
extern int emcAxisSetMaxVelocity(int axis, double vel);
extern int emcAxisSetMaxAcceleration(int axis, double acc);

extern int emcAxisInit(int axis);
extern int emcAxisHalt(int axis);
extern int emcAxisAbort(int axis);
extern int emcAxisEnable(int axis);
extern int emcAxisDisable(int axis);
extern int emcAxisHome(int axis);
extern int emcAxisUnhome(int axis);
extern int emcAxisJog(int axis, double vel);
extern int emcAxisIncrJog(int axis, double incr, double vel);
extern int emcAxisAbsJog(int axis, double pos, double vel);
extern int emcAxisActivate(int axis);
extern int emcAxisDeactivate(int axis);
extern int emcAxisOverrideLimits(int axis);
extern int emcAxisLoadComp(int axis, const char *file, int type);


extern int emcAxisUpdate(EMC_AXIS_STAT stat[], int numAxes);

// implementation functions for EMC_TRAJ types

extern int emcTrajSetAxes(int axes, int axismask);
extern int emcTrajSetUnits(double linearUnits, double angularUnits);
extern int emcTrajSetCycleTime(double cycleTime);
extern int emcTrajSetMode(int traj_mode);
extern int emcTrajSetTeleopVector(EmcPose vel);
extern int emcTrajSetVelocity(double vel, double ini_maxvel);
extern int emcTrajSetAcceleration(double acc);
extern int emcTrajSetMaxVelocity(double vel);
extern int emcTrajSetMaxAcceleration(double acc);
extern int emcTrajSetScale(double scale);
extern int emcTrajSetRapidScale(double scale);
extern int emcTrajSetFOEnable(unsigned char mode);   //feed override enable
extern int emcTrajSetFHEnable(unsigned char mode);   //feed hold enable
extern int emcTrajSetSpindleScale(double scale);
extern int emcTrajSetSOEnable(unsigned char mode);   //spindle speed override enable
extern int emcTrajSetAFEnable(unsigned char enable); //adaptive feed enable
extern int emcTrajSetMotionId(int id);
extern double emcTrajGetLinearUnits();
extern double emcTrajGetAngularUnits();

extern int emcTrajInit();
extern int emcTrajHalt();
extern int emcTrajEnable();
extern int emcTrajDisable();
extern int emcTrajAbort();
extern int emcTrajPause();
extern int emcTrajStep();
extern int emcTrajResume();
extern int emcTrajDelay(double delay);
extern int emcTrajLinearMove(EmcPose end, int type, double vel,
                             double ini_maxvel, double acc, int indexrotary, int pure_angular);
extern int emcTrajCircularMove(EmcPose end, PM_CARTESIAN center, PM_CARTESIAN
        normal, int turn, int type, double vel, double ini_maxvel, double acc, double acc_normal);
extern int emcTrajSetTermCond(int cond, double tolerance);
extern int emcTrajSetSpindleSync(double feed_per_revolution, bool wait_for_index);
extern int emcTrajSetOffset(EmcPose tool_offset);
extern int emcTrajSetOrigin(EmcPose origin);
extern int emcTrajSetRotation(double rotation);
extern int emcTrajSetHome(EmcPose home);
extern int emcTrajClearProbeTrippedFlag();
extern int emcTrajProbe(EmcPose pos, int type, double vel, 
                        double ini_maxvel, double acc, int pure_angular, unsigned char probe_type);
extern int emcAuxInputWait(int index, int input_type, int wait_type, int timeout);
extern int emcTrajRigidTap(EmcPose pos, double vel, double ini_maxvel, double acc);

extern int emcTrajUpdate(EMC_TRAJ_STAT * stat);

// implementation functions for EMC_MOTION aggregate types

extern int emcMotionInit();
extern int emcMotionHalt();
extern int emcMotionAbort();
extern int emcMotionSetDebug(int debug);
extern int emcMotionSetAout(unsigned char index, double start, double end,
                            unsigned char now);
extern int emcMotionSetDout(unsigned char index, unsigned char start,
			    unsigned char end, unsigned char now);

extern int emcMotionUpdate(EMC_MOTION_STAT * stat);

extern int emcAbortCleanup(int reason,const char *message = "");

// implementation functions for EMC_TOOL types

extern int emcToolInit();
extern int emcToolHalt();
extern int emcToolAbort();
extern int emcToolPrepare(int pocket, int tool);
extern int emcToolLoad();
extern int emcToolUnload();
extern int emcToolLoadToolTable(const char *file);
extern int emcToolSetOffset(int pocket, int toolno, EmcPose offset, double diameter,
                            double frontangle, double backangle, int orientation);
extern int emcToolSetNumber(int number);
extern int emcToolStartChange();

extern int emcToolSetToolTableFile(const char *file);

extern int emcToolUpdate(EMC_TOOL_STAT * stat);

// implementation functions for EMC_AUX types

extern int emcAuxEstopOn();
extern int emcAuxEstopOff();

extern int emcAuxUpdate(EMC_AUX_STAT * stat);

// implementation functions for EMC_SPINDLE types

extern int emcSpindleAbort();
extern int emcSpindleSpeed(double speed, double factor, double xoffset);
extern int emcSpindleOn(double speed, double factor, double xoffset);
extern int emcSpindleOrient(double orientation, int direction);
extern int emcSpindleWaitOrientComplete(double timout);
extern int emcSpindleOff();
extern int emcSpindleIncrease();
extern int emcSpindleDecrease();
extern int emcSpindleConstant();
extern int emcSpindleBrakeRelease();
extern int emcSpindleBrakeEngage();

extern int emcSpindleSetMode(int mode); //determines if Spindle needs to reset on abort

extern int emcSpindleUpdate(EMC_SPINDLE_STAT * stat);

// implementation functions for EMC_COOLANT types

extern int emcCoolantMistOn();
extern int emcCoolantMistOff();
extern int emcCoolantFloodOn();
extern int emcCoolantFloodOff();

extern int emcCoolantUpdate(EMC_COOLANT_STAT * stat);

// implementation functions for EMC_LUBE types

extern int emcLubeOn();
extern int emcLubeOff();

extern int emcLubeUpdate(EMC_LUBE_STAT * stat);

// implementation functions for EMC_IO types

extern int emcIoInit();
extern int emcIoHalt();
extern int emcIoAbort(int reason);
extern int emcIoSetCycleTime(double cycleTime);
extern int emcIoSetDebug(int debug);

extern int emcIoUpdate(EMC_IO_STAT * stat);

// implementation functions for EMC aggregate types

extern int emcInit();
extern int emcHalt();
extern int emcAbort();

int emcSetMaxFeedOverride(double maxFeedScale);
int emcSetupArcBlends(int arcBlendEnable,
        int arcBlendFallbackEnable,
        int arcBlendOptDepth,
        int arcBlendGapCycles,
        double arcBlendRampFreq,
        double arcBlendTangentKinkRatio);

int emcSetupConsistencyChecks(int consistency_checks, double max_position_drift_error);

extern int emcUpdate(EMC_STAT * stat);
// full EMC status
extern EMC_STAT *emcStatus;

// EMC IO status
extern EMC_IO_STAT *emcIoStatus;

// EMC MOTION status
extern EMC_MOTION_STAT *emcMotionStatus;

// values for EMC_AXIS_SET_AXIS, axisType
enum EmcAxisType {
    EMC_AXIS_LINEAR             = 1,
    EMC_AXIS_ANGULAR            = 2,
};

/**
 * Set the units conversion factor.
 * @see EMC_AXIS_SET_INPUT_SCALE
 */
typedef double                  EmcLinearUnits;
typedef double                  EmcAngularUnits;

#endif				// #ifndef EMC_HH
