/********************************************************************
* Description: emctaskmain.cc
*   Main program for EMC task level
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
/*
  Principles of operation:

  1.  The main program calls emcTaskPlan() and emcTaskExecute() cyclically.

  2.  emcTaskPlan() reads the new command, and decides what to do with
  it based on the mode (manual, auto, mdi) or state (estop, on) of the
  machine. Many of the commands just go out immediately to the
  subsystems (motion and IO). In auto mode, the interpreter is called
  and as a result the interp_list is appended with NML commands.

  3.  emcTaskExecute() executes a big switch on execState. If it's done,
  it gets the next item off the interp_list, and sets execState to the
  preconditions for that. These preconditions include waiting for motion,
  waiting for IO, etc. Once they are satisfied, it issues the command, and
  sets execState to the postconditions. Once those are satisfied, it gets
  the next item off the interp_list, and so on.

  4.  preconditions and postconditions are only looked at in conjunction
  with commands on the interp_list. Immediate commands won't have any
  pre- or postconditions associated with them looked at.

  5.  At this point, nothing in this file adds anything to the interp_list.
  This could change, for example, when defining pre- and postconditions for
  jog or home commands. If this is done, make sure that the corresponding
  abort command clears out the interp_list.

  6. Single-stepping is handled in checkPreconditions() as the first
  condition. If we're in single-stepping mode, as indicated by the
  variable 'stepping', we set the state to waiting-for-step. This
  polls on the variable 'steppingWait' which is reset to zero when a
  step command is received, and set to one when the command is
  issued.
  */

#include <stdio.h>		// vsprintf()
#include <string.h>		// strcpy()
#include <stdarg.h>		// va_start()
#include <stdlib.h>		// exit()
#include <signal.h>		// signal(), SIGINT
#include <float.h>		// DBL_MAX
#include <sys/types.h>		// pid_t
#include <unistd.h>		// fork()
#include <sys/wait.h>		// waitpid(), WNOHANG, WIFEXITED
#include <ctype.h>		// isspace()
#include <libintl.h>
#include <locale.h>


#if 0
// Enable this to niftily trap floating point exceptions for debugging
#include <fpu_control.h>
fpu_control_t __fpu_control = _FPU_IEEE & ~(_FPU_MASK_IM | _FPU_MASK_ZM | _FPU_MASK_OM);
#endif

#include "rcs.hh"		// NML classes, nmlErrorFormat()
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "canon.hh"		// CANON_TOOL_TABLE stuff
#include "inifile.hh"		// INIFILE
#include "interpl.hh"		// NML_INTERP_LIST, interp_list
#include "emcglb.h"		// EMC_INIFILE,NMLFILE, EMC_TASK_CYCLE_TIME
#include "interp_return.hh"	// public interpreter return values
#include "interp_internal.hh"	// interpreter private definitions
#include "rcs_print.hh"
#include "timer.hh"
#include "nml_oi.hh"
#include "task.hh"		// emcTaskCommand etc
#include "taskclass.hh"
#include "motion.h"             // EMCMOT_ORIENT_*
#include "inihal.hh"

/* time after which the user interface is declared dead
 * because it would'nt read any more messages
 */
#define DEFAULT_EMC_UI_TIMEOUT 5.0


// NML channels
static RCS_CMD_CHANNEL *emcCommandBuffer = 0;
static RCS_STAT_CHANNEL *emcStatusBuffer = 0;
static NML *emcErrorBuffer = 0;

// NML command channel data pointer
static RCS_CMD_MSG *emcCommand = 0;

// global EMC status
EMC_STAT *emcStatus = 0;

// timer stuff
static RCS_TIMER *timer = 0;

// flag signifying that ini file [TASK] CYCLE_TIME is <= 0.0, so
// we should not delay at all between cycles. This means also that
// the EMC_TASK_CYCLE_TIME global will be set to the measured cycle
// time each cycle, in case other code references this.
static int emcTaskNoDelay = 0;
// flag signifying that on the next loop, there should be no delay.
// this is set when transferring trajectory data from userspace to kernel
// space, annd reset otherwise.
static int emcTaskEager = 0;

static int no_force_homing = 0; // forces the user to home first before allowing MDI and Program run
//can be overriden by [TRAJ]NO_FORCE_HOMING=1

static double EMC_TASK_CYCLE_TIME_ORIG = 0.0;

// delay counter
static double taskExecDelayTimeout = 0.0;

// emcTaskIssueCommand issues command immediately
static int emcTaskIssueCommand(NMLmsg * cmd);

// pending command to be sent out by emcTaskExecute()
NMLmsg *emcTaskCommand = 0;

// signal handling code to stop main loop
int done;
static int emctask_shutdown(void);
extern void backtrace(int signo);
int _task = 1; // control preview behaviour when remapping

// for operator display on iocontrol signalling a toolchanger fault if io.fault is set
// %d receives io.reason
static const char *io_error = "toolchanger error %d";

extern void setup_signal_handlers(); // backtrace, gdb-in-new-window supportx

static int all_homed(void) {
    for(int i=0; i<9; i++) {
        unsigned int mask = 1<<i;
        if((emcStatus->motion.traj.axis_mask & mask) && !emcStatus->motion.axis[i].homed)
            return 0;
    }
    return 1;
}

void emctask_quit(int sig)
{
    // set main's done flag
    done = 1;
    // restore signal handler
    signal(sig, emctask_quit);
}

/* make sure at least space bytes are available on
 * error channel; wait a bit to drain if needed
 */
int emcErrorBufferOKtoWrite(int space, const char *caller)
{
    // check channel for validity
    if (emcErrorBuffer == NULL)
	return -1;
    if (!emcErrorBuffer->valid())
	return -1;

    double send_errorchan_timout = etime() + DEFAULT_EMC_UI_TIMEOUT;

    while (etime() < send_errorchan_timout) {
	if (emcErrorBuffer->get_space_available() < space) {
	    esleep(0.01);
	    continue;
	} else {
	    break;
	}
    }
    if (etime() >= send_errorchan_timout) {
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print("timeout waiting for error channel to drain, caller=`%s' request=%d\n", caller,space);
	}
	return -1;
    } else {
	// printf("--- %d bytes available after %f seconds\n", space, etime() - send_errorchan_timout + DEFAULT_EMC_UI_TIMEOUT);
    }
    return 0;
}


// implementation of EMC error logger
int emcOperatorError(int id, const char *fmt, ...)
{
    EMC_OPERATOR_ERROR error_msg;
    va_list ap;

    if ( emcErrorBufferOKtoWrite(sizeof(error_msg) * 2, "emcOperatorError"))
	return -1;

    if (NULL == fmt) {
	return -1;
    }
    if (0 == *fmt) {
	return -1;
    }
    // prepend error code, leave off 0 ad-hoc code
    error_msg.error[0] = 0;
    if (0 != id) {
	snprintf(error_msg.error, sizeof(error_msg.error), "[%d] ", id);
    }
    // append error string
    va_start(ap, fmt);
    vsnprintf(&error_msg.error[strlen(error_msg.error)], 
	      sizeof(error_msg.error) - strlen(error_msg.error), fmt, ap);
    va_end(ap);

    // force a NULL at the end for safety
    error_msg.error[LINELEN - 1] = 0;

    // write it
    rcs_print("%s\n", error_msg.error);
    return emcErrorBuffer->write(error_msg);
}

int emcOperatorText(int id, const char *fmt, ...)
{
    EMC_OPERATOR_TEXT text_msg;
    va_list ap;

    if ( emcErrorBufferOKtoWrite(sizeof(text_msg) * 2, "emcOperatorText"))
	return -1;

    // write args to NML message (ignore int text code)
    va_start(ap, fmt);
    vsnprintf(text_msg.text, sizeof(text_msg.text), fmt, ap);
    va_end(ap);

    // force a NULL at the end for safety
    text_msg.text[LINELEN - 1] = 0;

    // write it
    return emcErrorBuffer->write(text_msg);
}

int emcOperatorDisplay(int id, const char *fmt, ...)
{
    EMC_OPERATOR_DISPLAY display_msg;
    va_list ap;

    if ( emcErrorBufferOKtoWrite(sizeof(display_msg) * 2, "emcOperatorDisplay"))
	return -1;

    // write args to NML message (ignore int display code)
    va_start(ap, fmt);
    vsnprintf(display_msg.display, sizeof(display_msg.display), fmt, ap);
    va_end(ap);

    // force a NULL at the end for safety
    display_msg.display[LINELEN - 1] = 0;

    // write it
    return emcErrorBuffer->write(display_msg);
}

/*
  handling of EMC_SYSTEM_CMD
 */

/* convert string to arg/argv set */

static int argvize(const char *src, char *dst, char *argv[], int len)
{
    char *bufptr;
    int argvix;
    char inquote;
    char looking;

    strncpy(dst, src, len);
    dst[len - 1] = 0;
    bufptr = dst;
    inquote = 0;
    argvix = 0;
    looking = 1;

    while (0 != *bufptr) {
	if (*bufptr == '"') {
	    *bufptr = 0;
	    if (inquote) {
		inquote = 0;
		looking = 1;
	    } else {
		inquote = 1;
	    }
	} else if (isspace(*bufptr) && !inquote) {
	    looking = 1;
	    *bufptr = 0;
	} else if (looking) {
	    looking = 0;
	    argv[argvix] = bufptr;
	    argvix++;
	}
	bufptr++;
    }

    argv[argvix] = 0;		// null-terminate the argv list

    return argvix;
}

static pid_t emcSystemCmdPid = 0;

int emcSystemCmd(char *s)
{
    char buffer[EMC_SYSTEM_CMD_LEN];
    char *argv[EMC_SYSTEM_CMD_LEN / 2 + 1];

    if (0 != emcSystemCmdPid) {
	// something's already running, and we can only handle one
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print
		("emcSystemCmd: abandoning process %d, running ``%s''\n",
		 emcSystemCmdPid, s);
	}
    }

    emcSystemCmdPid = fork();

    if (-1 == emcSystemCmdPid) {
	// we're still the parent, with no child created
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print("system command ``%s'' can't be executed\n", s);
	}
	return -1;
    }

    if (0 == emcSystemCmdPid) {
	// we're the child
	// convert string to argc/argv
	argvize(s, buffer, argv, EMC_SYSTEM_CMD_LEN);
	// drop any setuid privileges
	setuid(getuid());
	execvp(argv[0], argv);
	// if we get here, we didn't exec
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print("emcSystemCmd: can't execute ``%s''\n", s);
	}
	exit(-1);
    }
    // else we're the parent
    return 0;
}

// shorthand typecasting ptrs
static EMC_AXIS_HALT *axis_halt_msg;
static EMC_AXIS_DISABLE *disable_msg;
static EMC_AXIS_ENABLE *enable_msg;
static EMC_AXIS_HOME *home_msg;
static EMC_AXIS_UNHOME *unhome_msg;
static EMC_AXIS_JOG *jog_msg;
static EMC_AXIS_ABORT *axis_abort_msg;
static EMC_AXIS_INCR_JOG *incr_jog_msg;
static EMC_AXIS_ABS_JOG *abs_jog_msg;
static EMC_AXIS_SET_BACKLASH *set_backlash_msg;
static EMC_AXIS_SET_HOMING_PARAMS *set_homing_params_msg;
static EMC_AXIS_SET_FERROR *set_ferror_msg;
static EMC_AXIS_SET_MIN_FERROR *set_min_ferror_msg;
static EMC_AXIS_SET_MAX_POSITION_LIMIT *set_max_limit_msg;
static EMC_AXIS_SET_MIN_POSITION_LIMIT *set_min_limit_msg;
static EMC_AXIS_OVERRIDE_LIMITS *axis_lim_msg;
//static EMC_AXIS_SET_OUTPUT *axis_output_msg;
static EMC_AXIS_LOAD_COMP *axis_load_comp_msg;
//static EMC_AXIS_SET_STEP_PARAMS *set_step_params_msg;

static EMC_TRAJ_SET_SCALE *emcTrajSetScaleMsg;
static EMC_TRAJ_SET_RAPID_SCALE *emcTrajSetRapidScaleMsg;
static EMC_TRAJ_SET_MAX_VELOCITY *emcTrajSetMaxVelocityMsg;
static EMC_TRAJ_SET_SPINDLE_SCALE *emcTrajSetSpindleScaleMsg;
static EMC_TRAJ_SET_VELOCITY *emcTrajSetVelocityMsg;
static EMC_TRAJ_SET_ACCELERATION *emcTrajSetAccelerationMsg;
static EMC_TRAJ_LINEAR_MOVE *emcTrajLinearMoveMsg;
static EMC_TRAJ_CIRCULAR_MOVE *emcTrajCircularMoveMsg;
static EMC_TRAJ_DELAY *emcTrajDelayMsg;
static EMC_TRAJ_SET_TERM_COND *emcTrajSetTermCondMsg;
static EMC_TRAJ_SET_SPINDLESYNC *emcTrajSetSpindlesyncMsg;

// These classes are commented out because the compiler
// complains that they are "defined but not used".
//static EMC_MOTION_SET_AOUT *emcMotionSetAoutMsg;
//static EMC_MOTION_SET_DOUT *emcMotionSetDoutMsg;

static EMC_SPINDLE_SPEED *spindle_speed_msg;
static EMC_SPINDLE_ORIENT *spindle_orient_msg;
static EMC_SPINDLE_WAIT_ORIENT_COMPLETE *wait_spindle_orient_complete_msg;
static EMC_SPINDLE_ON *spindle_on_msg;
static EMC_TOOL_PREPARE *tool_prepare_msg;
static EMC_TOOL_LOAD_TOOL_TABLE *load_tool_table_msg;
static EMC_TOOL_SET_OFFSET *emc_tool_set_offset_msg;
static EMC_TOOL_SET_NUMBER *emc_tool_set_number_msg;
static EMC_TASK_SET_MODE *mode_msg;
static EMC_TASK_SET_STATE *state_msg;
static EMC_TASK_PLAN_RUN *run_msg;
static EMC_TASK_PLAN_EXECUTE *execute_msg;
static EMC_TASK_PLAN_OPEN *open_msg;
static EMC_TASK_PLAN_SET_OPTIONAL_STOP *os_msg;
static EMC_TASK_PLAN_SET_BLOCK_DELETE *bd_msg;

static EMC_AUX_INPUT_WAIT *emcAuxInputWaitMsg;
static int emcAuxInputWaitType = 0;
static int emcAuxInputWaitIndex = -1;

// commands we compose here
static EMC_TASK_PLAN_RUN taskPlanRunCmd;	// 16-Aug-1999 FMP
static EMC_TASK_PLAN_INIT taskPlanInitCmd;
static EMC_TASK_PLAN_SYNCH taskPlanSynchCmd;

static int interpResumeState = EMC_TASK_INTERP_IDLE;
static int programStartLine = 0;	// which line to run program from
// how long the interp list can be

int stepping = 0;
int steppingWait = 0;
static int steppedLine = 0;

// Variables to handle MDI call interrupts
// Depth of call level before interrupted MDI call
static int mdi_execute_level = -1;
// Schedule execute(0) command
static int mdi_execute_next = 0;
// Wait after interrupted command
static int mdi_execute_wait = 0;
// Side queue to store MDI commands
static NML_INTERP_LIST mdi_execute_queue;

// MDI input queue
static NML_INTERP_LIST mdi_input_queue;
#define  MAX_MDI_QUEUE 10
static int max_mdi_queued_commands = MAX_MDI_QUEUE;

/*
  checkInterpList(NML_INTERP_LIST *il, EMC_STAT *stat) takes a pointer
  to an interpreter list and a pointer to the EMC status, pops each NML
  message off the list, and checks it against limits, resource availability,
  etc. in the status.

  It returns 0 if all messages check out, -1 if any of them fail. If one
  fails, the rest of the list is not checked.
 */
static int checkInterpList(NML_INTERP_LIST * il, EMC_STAT * stat)
{
    NMLmsg *cmd = 0;
    // let's create some shortcuts to casts at compile time
#define operator_error_msg ((EMC_OPERATOR_ERROR *) cmd)
#define linear_move ((EMC_TRAJ_LINEAR_MOVE *) cmd)
#define circular_move ((EMC_TRAJ_CIRCULAR_MOVE *) cmd)

    while (il->len() > 0) {
	cmd = il->get();

	switch (cmd->type) {

	case EMC_OPERATOR_ERROR_TYPE:
	    emcOperatorError(operator_error_msg->id, "%s",
			     operator_error_msg->error);
	    break;

	case EMC_TRAJ_LINEAR_MOVE_TYPE:
	    if (linear_move->end.tran.x >
		stat->motion.axis[0].maxPositionLimit) {
		emcOperatorError(0, _("%s exceeds +X limit"), stat->task.command);
		return -1;
	    }
	    if (linear_move->end.tran.y >
		stat->motion.axis[1].maxPositionLimit) {
		emcOperatorError(0, _("%s exceeds +Y limit"), stat->task.command);
		return -1;
	    }
	    if (linear_move->end.tran.z >
		stat->motion.axis[2].maxPositionLimit) {
		emcOperatorError(0, _("%s exceeds +Z limit"), stat->task.command);
		return -1;
	    }
	    if (linear_move->end.tran.x <
		stat->motion.axis[0].minPositionLimit) {
		emcOperatorError(0, _("%s exceeds -X limit"), stat->task.command);
		return -1;
	    }
	    if (linear_move->end.tran.y <
		stat->motion.axis[1].minPositionLimit) {
		emcOperatorError(0, _("%s exceeds -Y limit"), stat->task.command);
		return -1;
	    }
	    if (linear_move->end.tran.z <
		stat->motion.axis[2].minPositionLimit) {
		emcOperatorError(0, _("%s exceeds -Z limit"), stat->task.command);
		return -1;
	    }
	    break;

	case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
	    if (circular_move->end.tran.x >
		stat->motion.axis[0].maxPositionLimit) {
		emcOperatorError(0, _("%s exceeds +X limit"), stat->task.command);
		return -1;
	    }
	    if (circular_move->end.tran.y >
		stat->motion.axis[1].maxPositionLimit) {
		emcOperatorError(0, _("%s exceeds +Y limit"), stat->task.command);
		return -1;
	    }
	    if (circular_move->end.tran.z >
		stat->motion.axis[2].maxPositionLimit) {
		emcOperatorError(0, _("%s exceeds +Z limit"), stat->task.command);
		return -1;
	    }
	    if (circular_move->end.tran.x <
		stat->motion.axis[0].minPositionLimit) {
		emcOperatorError(0, _("%s exceeds -X limit"), stat->task.command);
		return -1;
	    }
	    if (circular_move->end.tran.y <
		stat->motion.axis[1].minPositionLimit) {
		emcOperatorError(0, _("%s exceeds -Y limit"), stat->task.command);
		return -1;
	    }
	    if (circular_move->end.tran.z <
		stat->motion.axis[2].minPositionLimit) {
		emcOperatorError(0, _("%s exceeds -Z limit"), stat->task.command);
		return -1;
	    }
	    break;

	default:
	    break;
	}
    }

    return 0;

    // get rid of the compile-time cast shortcuts
#undef circular_move_msg
#undef linear_move_msg
#undef operator_error_msg
}
extern int emcTaskMopup();

void readahead_reading(void)
{
    int readRetval;
    int execRetval;

		if (interp_list.len() <= emc_task_interp_max_len) {
                    int count = 0;
interpret_again:
		    if (emcTaskPlanIsWait()) {
			// delay reading of next line until all is done
			if (interp_list.len() == 0 &&
			    emcTaskCommand == 0 &&
			    emcStatus->task.execState ==
			    EMC_TASK_EXEC_DONE) {
			    emcTaskPlanClearWait();
			 }
		    } else {
			readRetval = emcTaskPlanRead();
			/*! \todo MGS FIXME
			   This if() actually evaluates to if (readRetval != INTERP_OK)...
			   *** Need to look at all calls to things that return INTERP_xxx values! ***
			   MGS */
			if (readRetval > INTERP_MIN_ERROR
				|| readRetval == INTERP_ENDFILE
				|| readRetval == INTERP_EXIT
				|| readRetval == INTERP_EXECUTE_FINISH) {
			    /* emcTaskPlanRead retval != INTERP_OK
			       Signal to the rest of the system that that the interp
			       is now in a paused state. */
			    /*! \todo FIXME The above test *should* be reduced to:
			       readRetVal != INTERP_OK
			       (N.B. Watch for negative error codes.) */
			    emcStatus->task.interpState =
				EMC_TASK_INTERP_WAITING;
			} else {
			    // got a good line
			    // record the line number and command
			    emcStatus->task.readLine = emcTaskPlanLine();

			    emcTaskPlanCommand((char *) &emcStatus->task.
					       command);
			    // and execute it
			    execRetval = emcTaskPlanExecute(0);
			    // line number may need update after
			    // returns from subprograms in external
			    // files
			    emcStatus->task.readLine = emcTaskPlanLine();
			    if (execRetval > INTERP_MIN_ERROR) {
				emcStatus->task.interpState =
				    EMC_TASK_INTERP_WAITING;
				interp_list.clear();
				emcAbortCleanup(EMC_ABORT_INTERPRETER_ERROR,
						"interpreter error"); 
			    } else if (execRetval == -1
				    || execRetval == INTERP_EXIT ) {
				emcStatus->task.interpState =
				    EMC_TASK_INTERP_WAITING;
			    } else if (execRetval == INTERP_EXECUTE_FINISH) {
				// INTERP_EXECUTE_FINISH signifies
				// that no more reading should be done until
				// everything
				// outstanding is completed
				emcTaskPlanSetWait();
				// and resynch interp WM
				emcTaskQueueCommand(&taskPlanSynchCmd);
			    } else if (execRetval != 0) {
				// end of file
				emcStatus->task.interpState =
				    EMC_TASK_INTERP_WAITING;
                                emcStatus->task.motionLine = 0;
                                emcStatus->task.readLine = 0;
			    } else {

				// executed a good line
			    }

			    // throw the results away if we're supposed to
			    // read
			    // through it
			    if ((programStartLine < 0 ||
				 emcStatus->task.readLine < programStartLine) &&
				emcTaskPlanLevel() == 0) {
				// we're stepping over lines, so check them
				// for
				// limits, etc. and clear then out
				if (0 != checkInterpList(&interp_list,
							 emcStatus)) {
				    // problem with actions, so do same as we
				    // did
				    // for a bad read from emcTaskPlanRead()
				    // above
				    emcStatus->task.interpState =
					EMC_TASK_INTERP_WAITING;
				}
				// and clear it regardless
				interp_list.clear();
			    }

			    if (emcStatus->task.readLine < programStartLine &&
				emcTaskPlanLevel() == 0) {
			    
				//update the position with our current position, as the other positions are only skipped through
				CANON_UPDATE_END_POINT(emcStatus->motion.traj.actualPosition.tran.x,
						       emcStatus->motion.traj.actualPosition.tran.y,
						       emcStatus->motion.traj.actualPosition.tran.z,
						       emcStatus->motion.traj.actualPosition.a,
						       emcStatus->motion.traj.actualPosition.b,
						       emcStatus->motion.traj.actualPosition.c,
						       emcStatus->motion.traj.actualPosition.u,
						       emcStatus->motion.traj.actualPosition.v,
						       emcStatus->motion.traj.actualPosition.w);

				if ((emcStatus->task.readLine + 1 == programStartLine)  &&
				    (emcTaskPlanLevel() == 0))  {

				    emcTaskPlanSynch();

                                    // reset programStartLine so we don't fall into our stepping routines
                                    // if we happen to execute lines before the current point later (due to subroutines).
                                    programStartLine = 0;
                                }
			    }

                            if (count++ < emc_task_interp_max_len
                                    && emcStatus->task.interpState == EMC_TASK_INTERP_READING
                                    && interp_list.len() <= emc_task_interp_max_len * 2/3) {
                                goto interpret_again;
                            }

			}	// else read was OK, so execute
		    }		// else not emcTaskPlanIsWait
		}		// if interp len is less than max
}

static void mdi_execute_abort(void)
{
    int queued_mdi_commands;

    // XXX: Reset needed?
    if (mdi_execute_wait || mdi_execute_next)
        emcTaskPlanReset();
    mdi_execute_level = -1;
    mdi_execute_wait = 0;
    mdi_execute_next = 0;

    queued_mdi_commands = mdi_execute_queue.len();
    if (queued_mdi_commands > 0) {
        rcs_print("mdi_execute_abort: dropping %d queued MDI commands\n", queued_mdi_commands);
    }
    mdi_execute_queue.clear();
    emcStatus->task.queuedMDIcommands = 0;

    emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
}

static void mdi_execute_hook(void)
{
    if (mdi_execute_wait && emcTaskPlanIsWait()) {
	// delay reading of next line until all is done
	if (interp_list.len() == 0 &&
	    emcTaskCommand == 0 &&
	    emcStatus->task.execState ==
	    EMC_TASK_EXEC_DONE) {
	    emcTaskPlanClearWait(); 
	    mdi_execute_wait = 0;
	    mdi_execute_hook();
	}
	return;
    }

    if (
        (mdi_execute_level < 0)
        && (mdi_execute_wait == 0)
        && (mdi_execute_queue.len() > 0)
        && (interp_list.len() == 0)
        && (emcTaskCommand == NULL)
    ) {
	interp_list.append(mdi_execute_queue.get());
        emcStatus->task.queuedMDIcommands = mdi_execute_queue.len();
	return;
    }

    // determine when a MDI command actually finishes normally.
    if (interp_list.len() == 0 &&
	emcTaskCommand == 0 &&
	emcStatus->task.execState ==  EMC_TASK_EXEC_DONE && 
	emcStatus->task.interpState != EMC_TASK_INTERP_IDLE && 
	emcStatus->motion.traj.queue == 0 &&
	emcStatus->io.status == RCS_DONE && 
	!mdi_execute_wait && 
	!mdi_execute_next) {

	// finished. Check for dequeuing of queued MDI command is done in emcTaskPlan().
	if (emc_debug & EMC_DEBUG_TASK_ISSUE)
	    rcs_print("mdi_execute_hook: MDI command '%s' done (remaining: %d)\n",
		      emcStatus->task.command, mdi_input_queue.len());
	emcStatus->task.command[0] = 0;
	emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
    }

    if (!mdi_execute_next) return;

    if (interp_list.len() > emc_task_interp_max_len) return;

    mdi_execute_next = 0;

    EMC_TASK_PLAN_EXECUTE msg;
    msg.command[0] = (char) 0xff;

    interp_list.append(msg);
}

void readahead_waiting(void)
{
	// now handle call logic
	// check for subsystems done
	if (interp_list.len() == 0 &&
	    emcTaskCommand == 0 &&
	    emcStatus->motion.traj.queue == 0 &&
	    emcStatus->io.status == RCS_DONE)
	    // finished
	{
	    int was_open = taskplanopen;
	    if (was_open) {
		emcTaskPlanClose();
		if (emc_debug & EMC_DEBUG_INTERP && was_open) {
		    rcs_print
			("emcTaskPlanClose() called at %s:%d\n",
			 __FILE__, __LINE__);
		}
		// then resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
	    } else {
		emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
	    }
	    emcStatus->task.readLine = 0;
	} else {
	    // still executing
        }
}

/*
  emcTaskPlan()

  Planner for NC code or manual mode operations
  */
static int emcTaskPlan(void)
{
    NMLTYPE type;
    int retval = 0;

    // check for new command
    if (emcCommand->serial_number != emcStatus->echo_serial_number) {
	// flag it here locally as a new command
	type = emcCommand->type;
    } else {
	// no new command-- reset local flag
	type = 0;
    }

    // handle any new command
    switch (emcStatus->task.state) {
    case EMC_TASK_STATE_OFF:
    case EMC_TASK_STATE_ESTOP:
    case EMC_TASK_STATE_ESTOP_RESET:

	// now switch on the mode
	switch (emcStatus->task.mode) {
	case EMC_TASK_MODE_MANUAL:
	case EMC_TASK_MODE_AUTO:
	case EMC_TASK_MODE_MDI:

	    // now switch on the command
	    switch (type) {
	    case 0:
	    case EMC_NULL_TYPE:
		// no command
		break;

		// immediate commands
	    case EMC_AXIS_SET_BACKLASH_TYPE:
	    case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
	    case EMC_AXIS_DISABLE_TYPE:
	    case EMC_AXIS_ENABLE_TYPE:
	    case EMC_AXIS_SET_FERROR_TYPE:
	    case EMC_AXIS_SET_MIN_FERROR_TYPE:
	    case EMC_AXIS_ABORT_TYPE:
	    case EMC_AXIS_LOAD_COMP_TYPE:
	    case EMC_AXIS_UNHOME_TYPE:
	    case EMC_TRAJ_SET_SCALE_TYPE:
	    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
	    case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
	    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
	    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
	    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
	    case EMC_TRAJ_SET_VELOCITY_TYPE:
	    case EMC_TRAJ_SET_ACCELERATION_TYPE:
	    case EMC_TASK_INIT_TYPE:
	    case EMC_TASK_SET_MODE_TYPE:
	    case EMC_TASK_SET_STATE_TYPE:
	    case EMC_TASK_PLAN_INIT_TYPE:
	    case EMC_TASK_PLAN_OPEN_TYPE:
	    case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	    case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	    case EMC_TASK_ABORT_TYPE:
	    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	    case EMC_TRAJ_PROBE_TYPE:
	    case EMC_AUX_INPUT_WAIT_TYPE:
	    case EMC_MOTION_SET_DOUT_TYPE:
	    case EMC_MOTION_ADAPTIVE_TYPE:
	    case EMC_MOTION_SET_AOUT_TYPE:
	    case EMC_TRAJ_RIGID_TAP_TYPE:
	    case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
	    case EMC_SET_DEBUG_TYPE:
		retval = emcTaskIssueCommand(emcCommand);
		break;

		// one case where we need to be in manual mode
	    case EMC_AXIS_OVERRIDE_LIMITS_TYPE:
		retval = 0;
		if (emcStatus->task.mode == EMC_TASK_MODE_MANUAL) {
		    retval = emcTaskIssueCommand(emcCommand);
		}
		break;

	    case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
	    case EMC_TOOL_SET_OFFSET_TYPE:
		// send to IO
		emcTaskQueueCommand(emcCommand);
		// signify no more reading
		emcTaskPlanSetWait();
		// then resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    case EMC_TOOL_SET_NUMBER_TYPE:
		// send to IO
		emcTaskQueueCommand(emcCommand);
		// then resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    default:
		emcOperatorError(0,
				 _
				 ("command (%s) cannot be executed until the machine is out of E-stop and turned on"),
				 emc_symbol_lookup(type));
		retval = -1;
		break;

	    }			// switch (type)

	default:
	    // invalid mode
	    break;

	}			// switch (mode)

	break;			// case EMC_TASK_STATE_OFF,ESTOP,ESTOP_RESET

    case EMC_TASK_STATE_ON:
	/* we can do everything (almost) when the machine is on, so let's
	   switch on the execution mode */
	switch (emcStatus->task.mode) {
	case EMC_TASK_MODE_MANUAL:	// ON, MANUAL
	    switch (type) {
	    case 0:
	    case EMC_NULL_TYPE:
		// no command
		break;

		// immediate commands

	    case EMC_AXIS_DISABLE_TYPE:
	    case EMC_AXIS_ENABLE_TYPE:
	    case EMC_AXIS_SET_BACKLASH_TYPE:
	    case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
	    case EMC_AXIS_SET_FERROR_TYPE:
	    case EMC_AXIS_SET_MIN_FERROR_TYPE:
	    case EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE:
	    case EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE:
	    case EMC_AXIS_ABORT_TYPE:
	    case EMC_AXIS_HALT_TYPE:
	    case EMC_AXIS_HOME_TYPE:
	    case EMC_AXIS_UNHOME_TYPE:
	    case EMC_AXIS_JOG_TYPE:
	    case EMC_AXIS_INCR_JOG_TYPE:
	    case EMC_AXIS_ABS_JOG_TYPE:
	    case EMC_AXIS_OVERRIDE_LIMITS_TYPE:
	    case EMC_TRAJ_PAUSE_TYPE:
	    case EMC_TRAJ_RESUME_TYPE:
	    case EMC_TRAJ_ABORT_TYPE:
	    case EMC_TRAJ_SET_SCALE_TYPE:
	    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
	    case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
	    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
	    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
	    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
	    case EMC_SPINDLE_SPEED_TYPE:
	    case EMC_SPINDLE_ON_TYPE:
	    case EMC_SPINDLE_OFF_TYPE:
	    case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
	    case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
	    case EMC_SPINDLE_INCREASE_TYPE:
	    case EMC_SPINDLE_DECREASE_TYPE:
	    case EMC_SPINDLE_CONSTANT_TYPE:
	    case EMC_COOLANT_MIST_ON_TYPE:
	    case EMC_COOLANT_MIST_OFF_TYPE:
	    case EMC_COOLANT_FLOOD_ON_TYPE:
	    case EMC_COOLANT_FLOOD_OFF_TYPE:
	    case EMC_LUBE_ON_TYPE:
	    case EMC_LUBE_OFF_TYPE:
	    case EMC_TASK_SET_MODE_TYPE:
	    case EMC_TASK_SET_STATE_TYPE:
	    case EMC_TASK_ABORT_TYPE:
	    case EMC_TASK_PLAN_PAUSE_TYPE:
	    case EMC_TASK_PLAN_RESUME_TYPE:
	    case EMC_TASK_PLAN_INIT_TYPE:
	    case EMC_TASK_PLAN_SYNCH_TYPE:
	    case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	    case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
	    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	    case EMC_TRAJ_PROBE_TYPE:
	    case EMC_AUX_INPUT_WAIT_TYPE:
	    case EMC_MOTION_SET_DOUT_TYPE:
	    case EMC_MOTION_SET_AOUT_TYPE:
	    case EMC_MOTION_ADAPTIVE_TYPE:
	    case EMC_TRAJ_RIGID_TAP_TYPE:
	    case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
	    case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
	    case EMC_SET_DEBUG_TYPE:
		retval = emcTaskIssueCommand(emcCommand);
		break;

		// queued commands

	    case EMC_TASK_PLAN_EXECUTE_TYPE:
		// resynch the interpreter, since we may have moved
		// externally
		emcTaskIssueCommand(&taskPlanSynchCmd);
		// and now call for interpreter execute
		retval = emcTaskIssueCommand(emcCommand);
		break;

	    case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
	    case EMC_TOOL_SET_OFFSET_TYPE:
		// send to IO
		emcTaskQueueCommand(emcCommand);
		// signify no more reading
		emcTaskPlanSetWait();
		// then resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    case EMC_TOOL_SET_NUMBER_TYPE:
		// send to IO
		emcTaskQueueCommand(emcCommand);
		// then resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
		break;

		// otherwise we can't handle it

	    default:
		emcOperatorError(0, _("can't do that (%s:%d) in manual mode"),
				 emc_symbol_lookup(type),(int) type);
		retval = -1;
		break;

	    }			// switch (type) in ON, MANUAL

	    break;		// case EMC_TASK_MODE_MANUAL

	case EMC_TASK_MODE_AUTO:	// ON, AUTO
	    switch (emcStatus->task.interpState) {
	    case EMC_TASK_INTERP_IDLE:	// ON, AUTO, IDLE
		switch (type) {
		case 0:
		case EMC_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case EMC_AXIS_SET_BACKLASH_TYPE:
		case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
		case EMC_AXIS_SET_FERROR_TYPE:
		case EMC_AXIS_SET_MIN_FERROR_TYPE:
		case EMC_AXIS_UNHOME_TYPE:
		case EMC_TRAJ_PAUSE_TYPE:
		case EMC_TRAJ_RESUME_TYPE:
		case EMC_TRAJ_ABORT_TYPE:
		case EMC_TRAJ_SET_SCALE_TYPE:
		case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
		case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
		case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	        case EMC_TRAJ_SET_FH_ENABLE_TYPE:
		case EMC_TRAJ_SET_SO_ENABLE_TYPE:
		case EMC_SPINDLE_SPEED_TYPE:
		case EMC_SPINDLE_ORIENT_TYPE:
		case EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
		case EMC_SPINDLE_ON_TYPE:
		case EMC_SPINDLE_OFF_TYPE:
		case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
		case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
		case EMC_SPINDLE_INCREASE_TYPE:
		case EMC_SPINDLE_DECREASE_TYPE:
		case EMC_SPINDLE_CONSTANT_TYPE:
		case EMC_COOLANT_MIST_ON_TYPE:
		case EMC_COOLANT_MIST_OFF_TYPE:
		case EMC_COOLANT_FLOOD_ON_TYPE:
		case EMC_COOLANT_FLOOD_OFF_TYPE:
		case EMC_LUBE_ON_TYPE:
		case EMC_LUBE_OFF_TYPE:
		case EMC_TASK_SET_MODE_TYPE:
		case EMC_TASK_SET_STATE_TYPE:
		case EMC_TASK_ABORT_TYPE:
		case EMC_TASK_PLAN_INIT_TYPE:
		case EMC_TASK_PLAN_OPEN_TYPE:
		case EMC_TASK_PLAN_RUN_TYPE:
		case EMC_TASK_PLAN_EXECUTE_TYPE:
		case EMC_TASK_PLAN_PAUSE_TYPE:
		case EMC_TASK_PLAN_RESUME_TYPE:
		case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case EMC_TRAJ_PROBE_TYPE:
		case EMC_AUX_INPUT_WAIT_TYPE:
		case EMC_TRAJ_RIGID_TAP_TYPE:
		case EMC_SET_DEBUG_TYPE:
		    retval = emcTaskIssueCommand(emcCommand);
		    break;

		case EMC_TASK_PLAN_STEP_TYPE:
		    // handles case where first action is to step the program
		    taskPlanRunCmd.line = 1;	// run from start
		    /*! \todo FIXME-- can have GUI set this; send a run instead of a 
		       step */
		    retval = emcTaskIssueCommand(&taskPlanRunCmd);
		    if(retval != 0) break;
		    emcTrajPause();
		    if (emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED) {
			interpResumeState = emcStatus->task.interpState;
		    }
		    emcStatus->task.interpState = EMC_TASK_INTERP_PAUSED;
		    emcStatus->task.task_paused = 1;
		    retval = 0;
		    break;

		case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
		case EMC_TOOL_SET_OFFSET_TYPE:
		    // send to IO
		    emcTaskQueueCommand(emcCommand);
		    // signify no more reading
		    emcTaskPlanSetWait();
		    // then resynch interpreter
		    emcTaskQueueCommand(&taskPlanSynchCmd);
		    break;

		    // otherwise we can't handle it
		default:
		    emcOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter idle"),
			    emc_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, IDLE

		break;		// EMC_TASK_INTERP_IDLE

	    case EMC_TASK_INTERP_READING:	// ON, AUTO, READING
		switch (type) {
		case 0:
		case EMC_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case EMC_AXIS_SET_BACKLASH_TYPE:
		case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
		case EMC_AXIS_SET_FERROR_TYPE:
		case EMC_AXIS_SET_MIN_FERROR_TYPE:
		case EMC_AXIS_UNHOME_TYPE:
		case EMC_TRAJ_PAUSE_TYPE:
		case EMC_TRAJ_RESUME_TYPE:
		case EMC_TRAJ_ABORT_TYPE:
		case EMC_TRAJ_SET_SCALE_TYPE:
		case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
                case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
		case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case EMC_TRAJ_SET_FO_ENABLE_TYPE:
		case EMC_TRAJ_SET_FH_ENABLE_TYPE:
		case EMC_TRAJ_SET_SO_ENABLE_TYPE:
		case EMC_SPINDLE_INCREASE_TYPE:
		case EMC_SPINDLE_DECREASE_TYPE:
		case EMC_SPINDLE_CONSTANT_TYPE:
		case EMC_TASK_PLAN_PAUSE_TYPE:
		case EMC_TASK_PLAN_RESUME_TYPE:
		case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case EMC_TASK_SET_MODE_TYPE:
		case EMC_TASK_SET_STATE_TYPE:
		case EMC_TASK_ABORT_TYPE:
		case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case EMC_TRAJ_PROBE_TYPE:
		case EMC_AUX_INPUT_WAIT_TYPE:
		case EMC_TRAJ_RIGID_TAP_TYPE:
		case EMC_SET_DEBUG_TYPE:
                case EMC_COOLANT_MIST_ON_TYPE:
                case EMC_COOLANT_MIST_OFF_TYPE:
                case EMC_COOLANT_FLOOD_ON_TYPE:
                case EMC_COOLANT_FLOOD_OFF_TYPE:
                case EMC_LUBE_ON_TYPE:
                case EMC_LUBE_OFF_TYPE:
		    retval = emcTaskIssueCommand(emcCommand);
		    return retval;
		    break;

		case EMC_TASK_PLAN_STEP_TYPE:
		    stepping = 1;	// set stepping mode in case it's not
		    steppingWait = 0;	// clear the wait
		    break;

		    // otherwise we can't handle it
		default:
		    emcOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter reading"),
			    emc_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, READING

               // handle interp readahead logic
                readahead_reading();
                
		break;		// EMC_TASK_INTERP_READING

	    case EMC_TASK_INTERP_PAUSED:	// ON, AUTO, PAUSED
		switch (type) {
		case 0:
		case EMC_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case EMC_AXIS_SET_BACKLASH_TYPE:
		case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
		case EMC_AXIS_SET_FERROR_TYPE:
		case EMC_AXIS_SET_MIN_FERROR_TYPE:
		case EMC_AXIS_UNHOME_TYPE:
		case EMC_TRAJ_PAUSE_TYPE:
		case EMC_TRAJ_RESUME_TYPE:
		case EMC_TRAJ_ABORT_TYPE:
		case EMC_TRAJ_SET_SCALE_TYPE:
		case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
		case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
		case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	        case EMC_TRAJ_SET_FH_ENABLE_TYPE:
		case EMC_TRAJ_SET_SO_ENABLE_TYPE:
		case EMC_SPINDLE_SPEED_TYPE:
		case EMC_SPINDLE_ORIENT_TYPE:
		case EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
		case EMC_SPINDLE_ON_TYPE:
		case EMC_SPINDLE_OFF_TYPE:
		case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
		case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
		case EMC_SPINDLE_INCREASE_TYPE:
		case EMC_SPINDLE_DECREASE_TYPE:
		case EMC_SPINDLE_CONSTANT_TYPE:
		case EMC_COOLANT_MIST_ON_TYPE:
		case EMC_COOLANT_MIST_OFF_TYPE:
		case EMC_COOLANT_FLOOD_ON_TYPE:
		case EMC_COOLANT_FLOOD_OFF_TYPE:
		case EMC_LUBE_ON_TYPE:
		case EMC_LUBE_OFF_TYPE:
		case EMC_TASK_SET_MODE_TYPE:
		case EMC_TASK_SET_STATE_TYPE:
		case EMC_TASK_ABORT_TYPE:
		case EMC_TASK_PLAN_EXECUTE_TYPE:
		case EMC_TASK_PLAN_PAUSE_TYPE:
		case EMC_TASK_PLAN_RESUME_TYPE:
		case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case EMC_TRAJ_PROBE_TYPE:
		case EMC_AUX_INPUT_WAIT_TYPE:
		case EMC_TRAJ_RIGID_TAP_TYPE:
		case EMC_SET_DEBUG_TYPE:
		    retval = emcTaskIssueCommand(emcCommand);
		    break;

		case EMC_TASK_PLAN_STEP_TYPE:
		    stepping = 1;
		    steppingWait = 0;
		    if (emcStatus->motion.traj.paused &&
			emcStatus->motion.traj.queue > 0) {
			// there are pending motions paused; step them
			emcTrajStep();
		    } else {
			emcStatus->task.interpState = (enum EMC_TASK_INTERP_ENUM) interpResumeState;
		    }
		    emcStatus->task.task_paused = 1;
		    break;

		    // otherwise we can't handle it
		default:
		    emcOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter paused"),
			    emc_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, PAUSED

		break;		// EMC_TASK_INTERP_PAUSED

	    case EMC_TASK_INTERP_WAITING:
		// interpreter ran to end
		// handle input commands
		switch (type) {
		case 0:
		case EMC_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case EMC_AXIS_SET_BACKLASH_TYPE:
		case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
		case EMC_AXIS_SET_FERROR_TYPE:
		case EMC_AXIS_SET_MIN_FERROR_TYPE:
		case EMC_AXIS_UNHOME_TYPE:
		case EMC_TRAJ_PAUSE_TYPE:
		case EMC_TRAJ_RESUME_TYPE:
		case EMC_TRAJ_ABORT_TYPE:
		case EMC_TRAJ_SET_SCALE_TYPE:
		case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
		case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
		case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	        case EMC_TRAJ_SET_FH_ENABLE_TYPE:
		case EMC_TRAJ_SET_SO_ENABLE_TYPE:
		case EMC_SPINDLE_INCREASE_TYPE:
		case EMC_SPINDLE_DECREASE_TYPE:
		case EMC_SPINDLE_CONSTANT_TYPE:
		case EMC_TASK_PLAN_EXECUTE_TYPE:
		case EMC_TASK_PLAN_PAUSE_TYPE:
		case EMC_TASK_PLAN_RESUME_TYPE:
		case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case EMC_TASK_SET_MODE_TYPE:
		case EMC_TASK_SET_STATE_TYPE:
		case EMC_TASK_ABORT_TYPE:
		case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case EMC_TRAJ_PROBE_TYPE:
		case EMC_AUX_INPUT_WAIT_TYPE:
	        case EMC_TRAJ_RIGID_TAP_TYPE:
		case EMC_SET_DEBUG_TYPE:
                case EMC_COOLANT_MIST_ON_TYPE:
                case EMC_COOLANT_MIST_OFF_TYPE:
                case EMC_COOLANT_FLOOD_ON_TYPE:
                case EMC_COOLANT_FLOOD_OFF_TYPE:
                case EMC_LUBE_ON_TYPE:
                case EMC_LUBE_OFF_TYPE:
		    retval = emcTaskIssueCommand(emcCommand);
		    break;

		case EMC_TASK_PLAN_STEP_TYPE:
		    stepping = 1;	// set stepping mode in case it's not
		    steppingWait = 0;	// clear the wait
		    break;

		    // otherwise we can't handle it
		default:
		    emcOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter waiting"),
			    emc_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, WAITING

                // handle interp readahead logic
                readahead_waiting();

		break;		// end of case EMC_TASK_INTERP_WAITING

	    default:
		// coding error
		rcs_print_error("invalid mode(%d)", emcStatus->task.mode);
		retval = -1;
		break;

	    }			// switch (mode) in ON, AUTO

	    break;		// case EMC_TASK_MODE_AUTO

	case EMC_TASK_MODE_MDI:	// ON, MDI
	    switch (type) {
	    case 0:
	    case EMC_NULL_TYPE:
		// no command
		break;

		// immediate commands

	    case EMC_AXIS_SET_BACKLASH_TYPE:
	    case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
	    case EMC_AXIS_SET_FERROR_TYPE:
	    case EMC_AXIS_SET_MIN_FERROR_TYPE:
	    case EMC_AXIS_UNHOME_TYPE:
	    case EMC_TRAJ_SET_SCALE_TYPE:
	    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
	    case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
	    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
	    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
	    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
	    case EMC_SPINDLE_SPEED_TYPE:
	    case EMC_SPINDLE_ORIENT_TYPE:
	    case EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	    case EMC_SPINDLE_ON_TYPE:
	    case EMC_SPINDLE_OFF_TYPE:
	    case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
	    case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
	    case EMC_SPINDLE_INCREASE_TYPE:
	    case EMC_SPINDLE_DECREASE_TYPE:
	    case EMC_SPINDLE_CONSTANT_TYPE:
	    case EMC_COOLANT_MIST_ON_TYPE:
	    case EMC_COOLANT_MIST_OFF_TYPE:
	    case EMC_COOLANT_FLOOD_ON_TYPE:
	    case EMC_COOLANT_FLOOD_OFF_TYPE:
	    case EMC_LUBE_ON_TYPE:
	    case EMC_LUBE_OFF_TYPE:
	    case EMC_TASK_SET_MODE_TYPE:
	    case EMC_TASK_SET_STATE_TYPE:
	    case EMC_TASK_PLAN_INIT_TYPE:
	    case EMC_TASK_PLAN_OPEN_TYPE:
	    case EMC_TASK_PLAN_PAUSE_TYPE:
	    case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	    case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	    case EMC_TASK_PLAN_RESUME_TYPE:
	    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
	    case EMC_TASK_ABORT_TYPE:
	    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	    case EMC_TRAJ_PROBE_TYPE:
	    case EMC_AUX_INPUT_WAIT_TYPE:
	    case EMC_MOTION_SET_DOUT_TYPE:
	    case EMC_MOTION_SET_AOUT_TYPE:
	    case EMC_MOTION_ADAPTIVE_TYPE:
	    case EMC_TRAJ_RIGID_TAP_TYPE:
	    case EMC_SET_DEBUG_TYPE:
		retval = emcTaskIssueCommand(emcCommand);
		break;

	    case EMC_TASK_PLAN_EXECUTE_TYPE:
                // If there are no queued MDI commands, no commands in
                // interp_list, and waitFlag isn't set, then this new
                // incoming MDI command can just be issued directly.
                // Otherwise we need to queue it and deal with it
                // later.
                if (
                    (mdi_execute_queue.len() == 0)
                    && (interp_list.len() == 0)
                    && (emcTaskCommand == NULL)
		    && (emcTaskPlanIsWait() == 0)
                ) {
                    retval = emcTaskIssueCommand(emcCommand);
                } else {
                    mdi_execute_queue.append(emcCommand);
                    emcStatus->task.queuedMDIcommands = mdi_execute_queue.len();
                    retval = 0;
                }
                break;

	    case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
	    case EMC_TOOL_SET_OFFSET_TYPE:
		// send to IO
		emcTaskQueueCommand(emcCommand);
		// signify no more reading
		emcTaskPlanSetWait();
		// then resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
		break;

		// otherwise we can't handle it
	    default:
		emcOperatorError(0, _("can't do that (%s:%d) in MDI mode"),
			emc_symbol_lookup(type),(int) type);

		retval = -1;
		break;

	    }			// switch (type) in ON, MDI
	    mdi_execute_hook();

	    break;		// case EMC_TASK_MODE_MDI

	default:
	    break;

	}			// switch (mode)

	break;			// case EMC_TASK_STATE_ON

    default:
	break;

    }				// switch (task.state)

    return retval;
}

/*
   emcTaskCheckPreconditions() is called for commands on the interp_list.
   Immediate commands, i.e., commands sent from calls to emcTaskIssueCommand()
   in emcTaskPlan() directly, are not handled here.

   The return value is a state for emcTaskExecute() to wait on, e.g.,
   EMC_TASK_EXEC_WAITING_FOR_MOTION, before the command can be sent out.
   */
static int emcTaskCheckPreconditions(NMLmsg * cmd)
{
    if (0 == cmd) {
	return EMC_TASK_EXEC_DONE;
    }

    switch (cmd->type) {
	// operator messages, if queued, will go out when everything before
	// them is done
    case EMC_OPERATOR_ERROR_TYPE:
    case EMC_OPERATOR_TEXT_TYPE:
    case EMC_OPERATOR_DISPLAY_TYPE:
    case EMC_SYSTEM_CMD_TYPE:
    case EMC_TRAJ_PROBE_TYPE:	// prevent blending of this
    case EMC_TRAJ_RIGID_TAP_TYPE: //and this
    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:	// and this
    case EMC_AUX_INPUT_WAIT_TYPE:
    case EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_TRAJ_LINEAR_MOVE_TYPE:
    case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
    case EMC_TRAJ_SET_VELOCITY_TYPE:
    case EMC_TRAJ_SET_ACCELERATION_TYPE:
    case EMC_TRAJ_SET_TERM_COND_TYPE:
    case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_IO;
	break;

    case EMC_TRAJ_SET_OFFSET_TYPE:
	// this applies the tool length offset variable after previous
	// motions
    case EMC_TRAJ_SET_G5X_TYPE:
    case EMC_TRAJ_SET_G92_TYPE:
    case EMC_TRAJ_SET_ROTATION_TYPE:
	// this applies the program origin after previous motions
	return EMC_TASK_EXEC_WAITING_FOR_MOTION;
	break;

    case EMC_TOOL_LOAD_TYPE:
    case EMC_TOOL_UNLOAD_TYPE:
    case EMC_TOOL_START_CHANGE_TYPE:
    case EMC_COOLANT_MIST_ON_TYPE:
    case EMC_COOLANT_MIST_OFF_TYPE:
    case EMC_COOLANT_FLOOD_ON_TYPE:
    case EMC_COOLANT_FLOOD_OFF_TYPE:
    case EMC_SPINDLE_SPEED_TYPE:
    case EMC_SPINDLE_ON_TYPE:
    case EMC_SPINDLE_OFF_TYPE:
    case EMC_SPINDLE_ORIENT_TYPE: // not sure
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_TOOL_PREPARE_TYPE:
    case EMC_LUBE_ON_TYPE:
    case EMC_LUBE_OFF_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_IO;
	break;

    case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
    case EMC_TOOL_SET_OFFSET_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_TOOL_SET_NUMBER_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_IO;
	break;

    case EMC_TASK_PLAN_PAUSE_TYPE:
    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
	/* pause on the interp list is queued, so wait until all are done */
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_TASK_PLAN_END_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_TASK_PLAN_INIT_TYPE:
    case EMC_TASK_PLAN_RUN_TYPE:
    case EMC_TASK_PLAN_SYNCH_TYPE:
    case EMC_TASK_PLAN_EXECUTE_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_TRAJ_DELAY_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case EMC_MOTION_SET_AOUT_TYPE:
	if (((EMC_MOTION_SET_AOUT *) cmd)->now) {
    	    return EMC_TASK_EXEC_WAITING_FOR_MOTION;
	}
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_MOTION_SET_DOUT_TYPE:
	if (((EMC_MOTION_SET_DOUT *) cmd)->now) {
    	    return EMC_TASK_EXEC_WAITING_FOR_MOTION;
	}
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_MOTION_ADAPTIVE_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_MOTION;
	break;

    case EMC_EXEC_PLUGIN_CALL_TYPE:
    case EMC_IO_PLUGIN_CALL_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;


    default:
	// unrecognized command
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print_error("preconditions: unrecognized command %d:%s\n",
			    (int)cmd->type, emc_symbol_lookup(cmd->type));
	}
	return EMC_TASK_EXEC_ERROR;
	break;
    }

    return EMC_TASK_EXEC_DONE;
}

// puts command on interp list
int emcTaskQueueCommand(NMLmsg * cmd)
{
    if (0 == cmd) {
	return 0;
    }

    interp_list.append(cmd);

    return 0;
}

// issues command immediately
static int emcTaskIssueCommand(NMLmsg * cmd)
{
    int retval = 0;
    int execRetval = 0;

    if (0 == cmd) {
        if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
            rcs_print("emcTaskIssueCommand() null command\n");
        }
	return 0;
    }
    if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	rcs_print("Issuing %s -- \t (%s)\n", emcSymbolLookup(cmd->type),
		  emcCommandBuffer->msg2str(cmd));
    }
    switch (cmd->type) {
	// general commands

    case EMC_OPERATOR_ERROR_TYPE:
	retval = emcOperatorError(((EMC_OPERATOR_ERROR *) cmd)->id,
				  "%s", ((EMC_OPERATOR_ERROR *) cmd)->error);
	break;

    case EMC_OPERATOR_TEXT_TYPE:
	retval = emcOperatorText(((EMC_OPERATOR_TEXT *) cmd)->id,
				 "%s", ((EMC_OPERATOR_TEXT *) cmd)->text);
	break;

    case EMC_OPERATOR_DISPLAY_TYPE:
	retval = emcOperatorDisplay(((EMC_OPERATOR_DISPLAY *) cmd)->id,
				    "%s", ((EMC_OPERATOR_DISPLAY *) cmd)->
				    display);
	break;

    case EMC_SYSTEM_CMD_TYPE:
	retval = emcSystemCmd(((EMC_SYSTEM_CMD *) cmd)->string);
	break;

	// axis commands

    case EMC_AXIS_DISABLE_TYPE:
	disable_msg = (EMC_AXIS_DISABLE *) cmd;
	retval = emcAxisDisable(disable_msg->axis);
	break;

    case EMC_AXIS_ENABLE_TYPE:
	enable_msg = (EMC_AXIS_ENABLE *) cmd;
	retval = emcAxisEnable(enable_msg->axis);
	break;

    case EMC_AXIS_HOME_TYPE:
	home_msg = (EMC_AXIS_HOME *) cmd;
	retval = emcAxisHome(home_msg->axis);
	break;

    case EMC_AXIS_UNHOME_TYPE:
	unhome_msg = (EMC_AXIS_UNHOME *) cmd;
	retval = emcAxisUnhome(unhome_msg->axis);
	break;

    case EMC_AXIS_JOG_TYPE:
	jog_msg = (EMC_AXIS_JOG *) cmd;
	retval = emcAxisJog(jog_msg->axis, jog_msg->vel);
	break;

    case EMC_AXIS_ABORT_TYPE:
	axis_abort_msg = (EMC_AXIS_ABORT *) cmd;
	retval = emcAxisAbort(axis_abort_msg->axis);
	break;

    case EMC_AXIS_INCR_JOG_TYPE:
	incr_jog_msg = (EMC_AXIS_INCR_JOG *) cmd;
	retval = emcAxisIncrJog(incr_jog_msg->axis,
				incr_jog_msg->incr, incr_jog_msg->vel);
	break;

    case EMC_AXIS_ABS_JOG_TYPE:
	abs_jog_msg = (EMC_AXIS_ABS_JOG *) cmd;
	retval = emcAxisAbsJog(abs_jog_msg->axis,
			       abs_jog_msg->pos, abs_jog_msg->vel);
	break;

    case EMC_AXIS_SET_BACKLASH_TYPE:
	set_backlash_msg = (EMC_AXIS_SET_BACKLASH *) cmd;
	retval =
	    emcAxisSetBacklash(set_backlash_msg->axis,
			       set_backlash_msg->backlash);
	break;

    case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
	set_homing_params_msg = (EMC_AXIS_SET_HOMING_PARAMS *) cmd;
	retval = emcAxisSetHomingParams(set_homing_params_msg->axis,
					set_homing_params_msg->home,
					set_homing_params_msg->offset,
					set_homing_params_msg->home_final_vel,
					set_homing_params_msg->search_vel,
					set_homing_params_msg->latch_vel,
					set_homing_params_msg->use_index,
					set_homing_params_msg->ignore_limits,
					set_homing_params_msg->is_shared,
					set_homing_params_msg->home_sequence,
					set_homing_params_msg->volatile_home,
                                        set_homing_params_msg->locking_indexer);
	break;

    case EMC_AXIS_SET_FERROR_TYPE:
	set_ferror_msg = (EMC_AXIS_SET_FERROR *) cmd;
	retval = emcAxisSetFerror(set_ferror_msg->axis,
				  set_ferror_msg->ferror);
	break;

    case EMC_AXIS_SET_MIN_FERROR_TYPE:
	set_min_ferror_msg = (EMC_AXIS_SET_MIN_FERROR *) cmd;
	retval = emcAxisSetMinFerror(set_min_ferror_msg->axis,
				     set_min_ferror_msg->ferror);
	break;

    case EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE:
	set_max_limit_msg = (EMC_AXIS_SET_MAX_POSITION_LIMIT *) cmd;
	retval = emcAxisSetMaxPositionLimit(set_max_limit_msg->axis,
					    set_max_limit_msg->limit);
	break;

    case EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE:
	set_min_limit_msg = (EMC_AXIS_SET_MIN_POSITION_LIMIT *) cmd;
	retval = emcAxisSetMinPositionLimit(set_min_limit_msg->axis,
					    set_min_limit_msg->limit);
	break;

    case EMC_AXIS_HALT_TYPE:
	axis_halt_msg = (EMC_AXIS_HALT *) cmd;
	retval = emcAxisHalt(axis_halt_msg->axis);
	break;

    case EMC_AXIS_OVERRIDE_LIMITS_TYPE:
	axis_lim_msg = (EMC_AXIS_OVERRIDE_LIMITS *) cmd;
	retval = emcAxisOverrideLimits(axis_lim_msg->axis);
	break;

    case EMC_AXIS_LOAD_COMP_TYPE:
	axis_load_comp_msg = (EMC_AXIS_LOAD_COMP *) cmd;
	retval = emcAxisLoadComp(axis_load_comp_msg->axis,
				 axis_load_comp_msg->file,
				 axis_load_comp_msg->type);
	break;

	// traj commands

    case EMC_TRAJ_SET_SCALE_TYPE:
	emcTrajSetScaleMsg = (EMC_TRAJ_SET_SCALE *) cmd;
	retval = emcTrajSetScale(emcTrajSetScaleMsg->scale);
	break;

    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
	emcTrajSetRapidScaleMsg = (EMC_TRAJ_SET_RAPID_SCALE *) cmd;
	retval = emcTrajSetRapidScale(emcTrajSetRapidScaleMsg->scale);
	break;

    case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
	emcTrajSetMaxVelocityMsg = (EMC_TRAJ_SET_MAX_VELOCITY *) cmd;
	retval = emcTrajSetMaxVelocity(emcTrajSetMaxVelocityMsg->velocity);
	break;

    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
	emcTrajSetSpindleScaleMsg = (EMC_TRAJ_SET_SPINDLE_SCALE *) cmd;
	retval = emcTrajSetSpindleScale(emcTrajSetSpindleScaleMsg->scale);
	break;

    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
	retval = emcTrajSetFOEnable(((EMC_TRAJ_SET_FO_ENABLE *) cmd)->mode);  // feed override enable/disable
	break;

    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
	retval = emcTrajSetFHEnable(((EMC_TRAJ_SET_FH_ENABLE *) cmd)->mode); //feed hold enable/disable
	break;

    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
	retval = emcTrajSetSOEnable(((EMC_TRAJ_SET_SO_ENABLE *) cmd)->mode); //spindle speed override enable/disable
	break;

    case EMC_TRAJ_SET_VELOCITY_TYPE:
	emcTrajSetVelocityMsg = (EMC_TRAJ_SET_VELOCITY *) cmd;
	retval = emcTrajSetVelocity(emcTrajSetVelocityMsg->velocity,
			emcTrajSetVelocityMsg->ini_maxvel);
	break;

    case EMC_TRAJ_SET_ACCELERATION_TYPE:
	emcTrajSetAccelerationMsg = (EMC_TRAJ_SET_ACCELERATION *) cmd;
	retval = emcTrajSetAcceleration(emcTrajSetAccelerationMsg->acceleration);
	break;

    case EMC_TRAJ_LINEAR_MOVE_TYPE:
	emcTrajLinearMoveMsg = (EMC_TRAJ_LINEAR_MOVE *) cmd;
        retval = emcTrajLinearMove(emcTrajLinearMoveMsg->end,
                                   emcTrajLinearMoveMsg->type, emcTrajLinearMoveMsg->vel,
                                   emcTrajLinearMoveMsg->ini_maxvel, emcTrajLinearMoveMsg->acc,
                                   emcTrajLinearMoveMsg->indexrotary, emcTrajLinearMoveMsg->pure_angular);
	break;

    case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
	emcTrajCircularMoveMsg = (EMC_TRAJ_CIRCULAR_MOVE *) cmd;
        retval = emcTrajCircularMove(emcTrajCircularMoveMsg->end,
                emcTrajCircularMoveMsg->center, emcTrajCircularMoveMsg->normal,
                emcTrajCircularMoveMsg->turn, emcTrajCircularMoveMsg->type,
                emcTrajCircularMoveMsg->vel,
                emcTrajCircularMoveMsg->ini_maxvel,
                emcTrajCircularMoveMsg->acc,
                emcTrajCircularMoveMsg->acc_normal);
	break;

    case EMC_TRAJ_PAUSE_TYPE:
	emcStatus->task.task_paused = 1;
	retval = emcTrajPause();
	break;

    case EMC_TRAJ_RESUME_TYPE:
	emcStatus->task.task_paused = 0;
	retval = emcTrajResume();
	break;

    case EMC_TRAJ_ABORT_TYPE:
	retval = emcTrajAbort();
	break;

    case EMC_TRAJ_DELAY_TYPE:
	emcTrajDelayMsg = (EMC_TRAJ_DELAY *) cmd;
	// set the timeout clock to expire at 'now' + delay time
	taskExecDelayTimeout = etime() + emcTrajDelayMsg->delay;
	retval = 0;
	break;

    case EMC_TRAJ_SET_TERM_COND_TYPE:
	emcTrajSetTermCondMsg = (EMC_TRAJ_SET_TERM_COND *) cmd;
	retval = emcTrajSetTermCond(emcTrajSetTermCondMsg->cond, emcTrajSetTermCondMsg->tolerance);
	break;

    case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
        emcTrajSetSpindlesyncMsg = (EMC_TRAJ_SET_SPINDLESYNC *) cmd;
        retval = emcTrajSetSpindleSync(emcTrajSetSpindlesyncMsg->feed_per_revolution, emcTrajSetSpindlesyncMsg->velocity_mode);
        break;

    case EMC_TRAJ_SET_OFFSET_TYPE:
	// update tool offset
	emcStatus->task.toolOffset = ((EMC_TRAJ_SET_OFFSET *) cmd)->offset;
        retval = emcTrajSetOffset(emcStatus->task.toolOffset);
	break;

    case EMC_TRAJ_SET_ROTATION_TYPE:
        emcStatus->task.rotation_xy = ((EMC_TRAJ_SET_ROTATION *) cmd)->rotation;
        retval = 0;
        break;

    case EMC_TRAJ_SET_G5X_TYPE:
	// struct-copy program origin
	emcStatus->task.g5x_offset = ((EMC_TRAJ_SET_G5X *) cmd)->origin;
        emcStatus->task.g5x_index = ((EMC_TRAJ_SET_G5X *) cmd)->g5x_index;
	retval = 0;
	break;
    case EMC_TRAJ_SET_G92_TYPE:
	// struct-copy program origin
	emcStatus->task.g92_offset = ((EMC_TRAJ_SET_G92 *) cmd)->origin;
	retval = 0;
	break;
    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	retval = emcTrajClearProbeTrippedFlag();
	break;

    case EMC_TRAJ_PROBE_TYPE:
	retval = emcTrajProbe(
	    ((EMC_TRAJ_PROBE *) cmd)->pos, 
	    ((EMC_TRAJ_PROBE *) cmd)->type,
	    ((EMC_TRAJ_PROBE *) cmd)->vel,
            ((EMC_TRAJ_PROBE *) cmd)->ini_maxvel,  
	    ((EMC_TRAJ_PROBE *) cmd)->acc,
        ((EMC_TRAJ_PROBE *) cmd)->pure_angular,
            ((EMC_TRAJ_PROBE *) cmd)->probe_type);
	break;

    case EMC_AUX_INPUT_WAIT_TYPE:
	emcAuxInputWaitMsg = (EMC_AUX_INPUT_WAIT *) cmd;
	if (emcAuxInputWaitMsg->timeout == WAIT_MODE_IMMEDIATE) { //nothing to do, CANON will get the needed value when asked by the interp
	    emcStatus->task.input_timeout = 0; // no timeout can occur
	    emcAuxInputWaitIndex = -1;
	    taskExecDelayTimeout = 0.0;
	} else {
	    emcAuxInputWaitType = emcAuxInputWaitMsg->wait_type; // remember what we are waiting for 
	    emcAuxInputWaitIndex = emcAuxInputWaitMsg->index; // remember the input to look at
	    emcStatus->task.input_timeout = 2; // set timeout flag, gets cleared if input changes before timeout happens
	    // set the timeout clock to expire at 'now' + delay time
	    taskExecDelayTimeout = etime() + emcAuxInputWaitMsg->timeout;
	}
	break;

    case EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	wait_spindle_orient_complete_msg = (EMC_SPINDLE_WAIT_ORIENT_COMPLETE *) cmd;
	taskExecDelayTimeout = etime() + wait_spindle_orient_complete_msg->timeout;
	break;

    case EMC_TRAJ_RIGID_TAP_TYPE:
	retval = emcTrajRigidTap(((EMC_TRAJ_RIGID_TAP *) cmd)->pos,
	        ((EMC_TRAJ_RIGID_TAP *) cmd)->vel,
        	((EMC_TRAJ_RIGID_TAP *) cmd)->ini_maxvel,  
		((EMC_TRAJ_RIGID_TAP *) cmd)->acc);
	break;

    case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
	if (((EMC_TRAJ_SET_TELEOP_ENABLE *) cmd)->enable) {
	    retval = emcTrajSetMode(EMC_TRAJ_MODE_TELEOP);
	} else {
	    retval = emcTrajSetMode(EMC_TRAJ_MODE_FREE);
	}
	break;

    case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
	retval =
	    emcTrajSetTeleopVector(((EMC_TRAJ_SET_TELEOP_VECTOR *) cmd)->
				   vector);
	break;

    case EMC_MOTION_SET_AOUT_TYPE:
	retval = emcMotionSetAout(((EMC_MOTION_SET_AOUT *) cmd)->index,
				  ((EMC_MOTION_SET_AOUT *) cmd)->start,
				  ((EMC_MOTION_SET_AOUT *) cmd)->end,
				  ((EMC_MOTION_SET_AOUT *) cmd)->now);
	break;

    case EMC_MOTION_SET_DOUT_TYPE:
	retval = emcMotionSetDout(((EMC_MOTION_SET_DOUT *) cmd)->index,
				  ((EMC_MOTION_SET_DOUT *) cmd)->start,
				  ((EMC_MOTION_SET_DOUT *) cmd)->end,
				  ((EMC_MOTION_SET_DOUT *) cmd)->now);
	break;

    case EMC_MOTION_ADAPTIVE_TYPE:
	retval = emcTrajSetAFEnable(((EMC_MOTION_ADAPTIVE *) cmd)->status);
	break;

    case EMC_SET_DEBUG_TYPE:
	/* set the debug level here */
	emc_debug = ((EMC_SET_DEBUG *) cmd)->debug;
	/* and in IO and motion */
	emcIoSetDebug(emc_debug);
	emcMotionSetDebug(emc_debug);
	/* and reflect it in the status-- this isn't updated continually */
	emcStatus->debug = emc_debug;
	break;

	// unimplemented ones

	// IO commands

    case EMC_SPINDLE_SPEED_TYPE:
	spindle_speed_msg = (EMC_SPINDLE_SPEED *) cmd;
	retval = emcSpindleSpeed(spindle_speed_msg->speed, spindle_speed_msg->factor, spindle_speed_msg->xoffset);
	break;

    case EMC_SPINDLE_ORIENT_TYPE:
	spindle_orient_msg = (EMC_SPINDLE_ORIENT *) cmd;
	retval = emcSpindleOrient(spindle_orient_msg->orientation, spindle_orient_msg->mode);
	break;

   case EMC_SPINDLE_ON_TYPE:
	spindle_on_msg = (EMC_SPINDLE_ON *) cmd;
	retval = emcSpindleOn(spindle_on_msg->speed, spindle_on_msg->factor, spindle_on_msg->xoffset);
	break;

    case EMC_SPINDLE_OFF_TYPE:
	retval = emcSpindleOff();
	break;

    case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
	retval = emcSpindleBrakeRelease();
	break;

    case EMC_SPINDLE_INCREASE_TYPE:
	retval = emcSpindleIncrease();
	break;

    case EMC_SPINDLE_DECREASE_TYPE:
	retval = emcSpindleDecrease();
	break;

    case EMC_SPINDLE_CONSTANT_TYPE:
	retval = emcSpindleConstant();
	break;

    case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
	retval = emcSpindleBrakeEngage();
	break;

    case EMC_COOLANT_MIST_ON_TYPE:
	retval = emcCoolantMistOn();
	break;

    case EMC_COOLANT_MIST_OFF_TYPE:
	retval = emcCoolantMistOff();
	break;

    case EMC_COOLANT_FLOOD_ON_TYPE:
	retval = emcCoolantFloodOn();
	break;

    case EMC_COOLANT_FLOOD_OFF_TYPE:
	retval = emcCoolantFloodOff();
	break;

    case EMC_LUBE_ON_TYPE:
	retval = emcLubeOn();
	break;

    case EMC_LUBE_OFF_TYPE:
	retval = emcLubeOff();
	break;

    case EMC_TOOL_PREPARE_TYPE:
	tool_prepare_msg = (EMC_TOOL_PREPARE *) cmd;
	retval = emcToolPrepare(tool_prepare_msg->pocket,tool_prepare_msg->tool);
	break;

    case EMC_TOOL_START_CHANGE_TYPE:
        retval = emcToolStartChange();
	break;

    case EMC_TOOL_LOAD_TYPE:
	retval = emcToolLoad();
	break;

    case EMC_TOOL_UNLOAD_TYPE:
	retval = emcToolUnload();
	break;

    case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
	load_tool_table_msg = (EMC_TOOL_LOAD_TOOL_TABLE *) cmd;
	retval = emcToolLoadToolTable(load_tool_table_msg->file);
	break;

    case EMC_TOOL_SET_OFFSET_TYPE:
	emc_tool_set_offset_msg = (EMC_TOOL_SET_OFFSET *) cmd;
	retval = emcToolSetOffset(emc_tool_set_offset_msg->pocket,
                                  emc_tool_set_offset_msg->toolno,
                                  emc_tool_set_offset_msg->offset,
                                  emc_tool_set_offset_msg->diameter,
                                  emc_tool_set_offset_msg->frontangle,
                                  emc_tool_set_offset_msg->backangle,
                                  emc_tool_set_offset_msg->orientation);
	break;

    case EMC_TOOL_SET_NUMBER_TYPE:
	emc_tool_set_number_msg = (EMC_TOOL_SET_NUMBER *) cmd;
	retval = emcToolSetNumber(emc_tool_set_number_msg->tool);
	break;

	// task commands

    case EMC_TASK_INIT_TYPE:
	retval = emcTaskInit();
	break;

    case EMC_TASK_ABORT_TYPE:
	// abort everything
	emcTaskAbort();
        emcIoAbort(EMC_ABORT_TASK_ABORT);
        emcSpindleAbort();
	mdi_execute_abort();
	emcAbortCleanup(EMC_ABORT_TASK_ABORT);
	retval = 0;
	break;

	// mode and state commands

    case EMC_TASK_SET_MODE_TYPE:
	mode_msg = (EMC_TASK_SET_MODE *) cmd;
	if (emcStatus->task.mode == EMC_TASK_MODE_AUTO &&
	    emcStatus->task.interpState != EMC_TASK_INTERP_IDLE &&
	    mode_msg->mode != EMC_TASK_MODE_AUTO) {
	    emcOperatorError(0, _("Can't switch mode while mode is AUTO and interpreter is not IDLE"));
	} else { // we can honour the modeswitch
	    if (mode_msg->mode == EMC_TASK_MODE_MANUAL &&
		emcStatus->task.mode != EMC_TASK_MODE_MANUAL) {
		// leaving auto or mdi mode for manual

		/*! \todo FIXME-- duplicate code for abort,
	        also near end of main, when aborting on subordinate errors,
	        and in emcTaskExecute() */

		// abort motion
		emcTaskAbort();
		mdi_execute_abort();

		// without emcTaskPlanClose(), a new run command resumes at
		// aborted line-- feature that may be considered later
		{
		    int was_open = taskplanopen;
		    emcTaskPlanClose();
		    if (emc_debug & EMC_DEBUG_INTERP && was_open) {
			rcs_print("emcTaskPlanClose() called at %s:%d\n",
			      __FILE__, __LINE__);
		    }
		}

		// clear out the pending command
		emcTaskCommand = 0;
		interp_list.clear();
                emcStatus->task.currentLine = 0;

		// clear out the interpreter state
		emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
		emcStatus->task.execState = EMC_TASK_EXEC_DONE;
		stepping = 0;
		steppingWait = 0;

		// now queue up command to resynch interpreter
		emcTaskQueueCommand(&taskPlanSynchCmd);
	    }
	    retval = emcTaskSetMode(mode_msg->mode);
	}
	break;

    case EMC_TASK_SET_STATE_TYPE:
	state_msg = (EMC_TASK_SET_STATE *) cmd;
	retval = emcTaskSetState(state_msg->state);
	break;

	// interpreter commands

    case EMC_TASK_PLAN_OPEN_TYPE:
	open_msg = (EMC_TASK_PLAN_OPEN *) cmd;
	retval = emcTaskPlanOpen(open_msg->file);
	if (retval > INTERP_MIN_ERROR) {
	    retval = -1;
	}
	if (-1 == retval) {
	    emcOperatorError(0, _("can't open %s"), open_msg->file);
	} else {
	    strcpy(emcStatus->task.file, open_msg->file);
	    retval = 0;
	}
	break;

    case EMC_TASK_PLAN_EXECUTE_TYPE:
	stepping = 0;
	steppingWait = 0;
	execute_msg = (EMC_TASK_PLAN_EXECUTE *) cmd;
        if (!all_homed() && !no_force_homing) { //!no_force_homing = force homing before MDI
            emcOperatorError(0, _("Can't issue MDI command when not homed"));
            retval = -1;
            break;
        }
        if (emcStatus->task.mode != EMC_TASK_MODE_MDI) {
            emcOperatorError(0, _("Must be in MDI mode to issue MDI command"));
            retval = -1;
            break;
        }
	// track interpState also during MDI - it might be an oword sub call
	emcStatus->task.interpState = EMC_TASK_INTERP_READING;

	if (execute_msg->command[0] != 0) {
	    char * command = execute_msg->command;
	    if (command[0] == (char) 0xff) {
		// Empty command received. Consider it is NULL
		command = NULL;
	    } else {
		// record initial MDI command
		strcpy(emcStatus->task.command, execute_msg->command);
	    }

	    int level = emcTaskPlanLevel();
	    if (emcStatus->task.mode == EMC_TASK_MODE_MDI) {
		if (mdi_execute_level < 0)
		    mdi_execute_level = level;
	    }

	    execRetval = emcTaskPlanExecute(command, 0);

	    level = emcTaskPlanLevel();

	    if (emcStatus->task.mode == EMC_TASK_MODE_MDI) {
		if (mdi_execute_level == level) {
		    mdi_execute_level = -1;
		} else if (level > 0) {
		    // Still insude call. Need another execute(0) call
		    // but only if we didnt encounter an error
		    if (execRetval == INTERP_ERROR) {
			mdi_execute_next = 0;
		    } else {
			mdi_execute_next = 1;
		    }
		}
	    }
	    switch (execRetval) {

	    case INTERP_EXECUTE_FINISH:
		// Flag MDI wait
		mdi_execute_wait = 1;
		// need to flush execution, so signify no more reading
		// until all is done
		emcTaskPlanSetWait();
		// and resynch the interpreter WM
		emcTaskQueueCommand(&taskPlanSynchCmd);
		// it's success, so retval really is 0
		retval = 0;
		break;

	    case INTERP_ERROR:
		// emcStatus->task.interpState =  EMC_TASK_INTERP_WAITING;
		interp_list.clear();
		// abort everything
		emcTaskAbort();
		emcIoAbort(EMC_ABORT_INTERPRETER_ERROR_MDI);
		emcSpindleAbort(); 
		mdi_execute_abort(); // sets emcStatus->task.interpState to  EMC_TASK_INTERP_IDLE
		emcAbortCleanup(EMC_ABORT_INTERPRETER_ERROR_MDI, "interpreter error during MDI");
		retval = -1;
		break;

	    case INTERP_EXIT:
	    case INTERP_ENDFILE:
	    case INTERP_FILE_NOT_OPEN:
		// this caused the error msg on M2 in MDI mode - execRetval == INTERP_EXIT which is would be ok (I think). mah
		retval = -1;
		break;

	    default:
		// other codes are OK
		retval = 0;
	    }
	}
	break;

    case EMC_TASK_PLAN_RUN_TYPE:
        if (!all_homed() && !no_force_homing) { //!no_force_homing = force homing before Auto
            emcOperatorError(0, _("Can't run a program when not homed"));
            retval = -1;
            break;
        }
	stepping = 0;
	steppingWait = 0;
	if (!taskplanopen && emcStatus->task.file[0] != 0) {
	    emcTaskPlanOpen(emcStatus->task.file);
	}
	run_msg = (EMC_TASK_PLAN_RUN *) cmd;
	programStartLine = run_msg->line;
	emcStatus->task.interpState = EMC_TASK_INTERP_READING;
	emcStatus->task.task_paused = 0;
	retval = 0;
	break;

    case EMC_TASK_PLAN_PAUSE_TYPE:
	emcTrajPause();
	if (emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED) {
	    interpResumeState = emcStatus->task.interpState;
	}
	emcStatus->task.interpState = EMC_TASK_INTERP_PAUSED;
	emcStatus->task.task_paused = 1;
	retval = 0;
	break;

    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
	if (GET_OPTIONAL_PROGRAM_STOP() == ON) {
	    emcTrajPause();
	    if (emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED) {
		interpResumeState = emcStatus->task.interpState;
	    }
	    emcStatus->task.interpState = EMC_TASK_INTERP_PAUSED;
	    emcStatus->task.task_paused = 1;
	}
	retval = 0;
	break;

    case EMC_TASK_PLAN_RESUME_TYPE:
	emcTrajResume();
	emcStatus->task.interpState =
	    (enum EMC_TASK_INTERP_ENUM) interpResumeState;
	emcStatus->task.task_paused = 0;
	stepping = 0;
	steppingWait = 0;
	retval = 0;
	break;

    case EMC_TASK_PLAN_END_TYPE:
	retval = 0;
	break;

    case EMC_TASK_PLAN_INIT_TYPE:
	retval = emcTaskPlanInit();
	if (retval > INTERP_MIN_ERROR) {
	    retval = -1;
	}
	break;

    case EMC_TASK_PLAN_SYNCH_TYPE:
	retval = emcTaskPlanSynch();
	if (retval > INTERP_MIN_ERROR) {
	    retval = -1;
	}
	break;

    case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	os_msg = (EMC_TASK_PLAN_SET_OPTIONAL_STOP *) cmd;
	emcTaskPlanSetOptionalStop(os_msg->state);
	retval = 0;
	break;

    case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	bd_msg = (EMC_TASK_PLAN_SET_BLOCK_DELETE *) cmd;
	emcTaskPlanSetBlockDelete(bd_msg->state);
	retval = 0;
	break;

    case EMC_EXEC_PLUGIN_CALL_TYPE:
	retval =  emcPluginCall( (EMC_EXEC_PLUGIN_CALL *) cmd);
	break;

    case EMC_IO_PLUGIN_CALL_TYPE:
	retval =  emcIoPluginCall( (EMC_IO_PLUGIN_CALL *) cmd);
	break;

     default:
	// unrecognized command
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print_error("ignoring issue of unknown command %d:%s\n",
			    (int)cmd->type, emc_symbol_lookup(cmd->type));
	}
	retval = 0;		// don't consider this an error
	break;
    }

    if (retval == -1) {
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print_error("error executing command %d:%s\n", (int)cmd->type,
			    emc_symbol_lookup(cmd->type));
	}
    }
    /* debug */
    if ((emc_debug & EMC_DEBUG_TASK_ISSUE) && retval) {
    	rcs_print("emcTaskIssueCommand() returning: %d\n", retval);
    }
    return retval;
}

/*
   emcTaskCheckPostconditions() is called for commands on the interp_list.
   Immediate commands, i.e., commands sent from calls to emcTaskIssueCommand()
   in emcTaskPlan() directly, are not handled here.

   The return value is a state for emcTaskExecute() to wait on, e.g.,
   EMC_TASK_EXEC_WAITING_FOR_MOTION, after the command has finished and
   before any other commands can be sent out.
   */
static int emcTaskCheckPostconditions(NMLmsg * cmd)
{
    if (0 == cmd) {
	return EMC_TASK_EXEC_DONE;
    }

    switch (cmd->type) {
    case EMC_OPERATOR_ERROR_TYPE:
    case EMC_OPERATOR_TEXT_TYPE:
    case EMC_OPERATOR_DISPLAY_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_SYSTEM_CMD_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD;
	break;

    case EMC_TRAJ_LINEAR_MOVE_TYPE:
    case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
    case EMC_TRAJ_SET_VELOCITY_TYPE:
    case EMC_TRAJ_SET_ACCELERATION_TYPE:
    case EMC_TRAJ_SET_TERM_COND_TYPE:
    case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
    case EMC_TRAJ_SET_OFFSET_TYPE:
    case EMC_TRAJ_SET_G5X_TYPE:
    case EMC_TRAJ_SET_G92_TYPE:
    case EMC_TRAJ_SET_ROTATION_TYPE:
    case EMC_TRAJ_PROBE_TYPE:
    case EMC_TRAJ_RIGID_TAP_TYPE:
    case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
    case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
    case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
    case EMC_TRAJ_SET_FO_ENABLE_TYPE:
    case EMC_TRAJ_SET_FH_ENABLE_TYPE:
    case EMC_TRAJ_SET_SO_ENABLE_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_TOOL_PREPARE_TYPE:
    case EMC_TOOL_LOAD_TYPE:
    case EMC_TOOL_UNLOAD_TYPE:
    case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
    case EMC_TOOL_START_CHANGE_TYPE:
    case EMC_TOOL_SET_OFFSET_TYPE:
    case EMC_TOOL_SET_NUMBER_TYPE:
    case EMC_SPINDLE_SPEED_TYPE:
    case EMC_SPINDLE_ON_TYPE:
    case EMC_SPINDLE_OFF_TYPE:
    case EMC_SPINDLE_ORIENT_TYPE:
    case EMC_COOLANT_MIST_ON_TYPE:
    case EMC_COOLANT_MIST_OFF_TYPE:
    case EMC_COOLANT_FLOOD_ON_TYPE:
    case EMC_COOLANT_FLOOD_OFF_TYPE:
    case EMC_LUBE_ON_TYPE:
    case EMC_LUBE_OFF_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_TASK_PLAN_RUN_TYPE:
    case EMC_TASK_PLAN_PAUSE_TYPE:
    case EMC_TASK_PLAN_END_TYPE:
    case EMC_TASK_PLAN_INIT_TYPE:
    case EMC_TASK_PLAN_SYNCH_TYPE:
    case EMC_TASK_PLAN_EXECUTE_TYPE:
    case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_SPINDLE_ORIENTED;
	break;

    case EMC_TRAJ_DELAY_TYPE:
    case EMC_AUX_INPUT_WAIT_TYPE:
	return EMC_TASK_EXEC_WAITING_FOR_DELAY;
	break;

    case EMC_MOTION_SET_AOUT_TYPE:
    case EMC_MOTION_SET_DOUT_TYPE:
    case EMC_MOTION_ADAPTIVE_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;

    case EMC_EXEC_PLUGIN_CALL_TYPE:
    case EMC_IO_PLUGIN_CALL_TYPE:
	return EMC_TASK_EXEC_DONE;
	break;

    default:
	// unrecognized command
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print_error("postconditions: unrecognized command %d:%s\n",
			    (int)cmd->type, emc_symbol_lookup(cmd->type));
	}
	return EMC_TASK_EXEC_DONE;
	break;
    }
    return EMC_TASK_EXEC_DONE; // unreached
}

/*
  STEPPING_CHECK() is a macro that prefaces a switch-case with a check
  for stepping. If stepping is active, it waits until the step has been
  given, then falls through to the rest of the case statement.
*/

#define STEPPING_CHECK()                                                   \
if (stepping) {                                                            \
  if (! steppingWait) {                                                    \
    steppingWait = 1;                                                      \
    steppedLine = emcStatus->task.currentLine;                             \
  }                                                                        \
  else {                                                                   \
    if (emcStatus->task.currentLine != steppedLine) {                      \
      break;                                                               \
    }                                                                      \
  }                                                                        \
}

// executor function
static int emcTaskExecute(void)
{
    int retval = 0;
    int status;			// status of child from EMC_SYSTEM_CMD
    pid_t pid;			// pid returned from waitpid()

    // first check for an abandoned system command and abort it
    if (emcSystemCmdPid != 0 &&
	emcStatus->task.execState !=
	EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD) {
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print("emcSystemCmd: abandoning process %d\n",
		      emcSystemCmdPid);
	}
	kill(emcSystemCmdPid, SIGINT);
	emcSystemCmdPid = 0;
    }

    switch (emcStatus->task.execState) {
    case EMC_TASK_EXEC_ERROR:

	/*! \todo FIXME-- duplicate code for abort,
	   also near end of main, when aborting on subordinate errors,
	   and in emcTaskIssueCommand() */

	// abort everything
	emcTaskAbort();
        emcIoAbort(EMC_ABORT_TASK_EXEC_ERROR);
        emcSpindleAbort();
	mdi_execute_abort();

	// without emcTaskPlanClose(), a new run command resumes at
	// aborted line-- feature that may be considered later
	{
	    int was_open = taskplanopen;
	    emcTaskPlanClose();
	    if (emc_debug & EMC_DEBUG_INTERP && was_open) {
		rcs_print("emcTaskPlanClose() called at %s:%d\n", __FILE__,
			  __LINE__);
	    }
	}

	// clear out pending command
	emcTaskCommand = 0;
	interp_list.clear();
	emcAbortCleanup(EMC_ABORT_TASK_EXEC_ERROR);
        emcStatus->task.currentLine = 0;

	// clear out the interpreter state
	emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
	emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	stepping = 0;
	steppingWait = 0;

	// now queue up command to resynch interpreter
	emcTaskQueueCommand(&taskPlanSynchCmd);

	retval = -1;
	break;

    case EMC_TASK_EXEC_DONE:
	STEPPING_CHECK();
	if (!emcStatus->motion.traj.queueFull &&
	    emcStatus->task.interpState != EMC_TASK_INTERP_PAUSED) {
	    if (0 == emcTaskCommand) {
		// need a new command
		emcTaskCommand = interp_list.get();
		// interp_list now has line number associated with this-- get
		// it
		if (0 != emcTaskCommand) {
		    emcTaskEager = 1;
		    emcStatus->task.currentLine =
			interp_list.get_line_number();
		    emcStatus->task.callLevel = emcTaskPlanLevel();
		    // and set it for all subsystems which use queued ids
		    emcTrajSetMotionId(emcStatus->task.currentLine);
		    if (emcStatus->motion.traj.queueFull) {
			emcStatus->task.execState =
			    EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE;
		    } else {
			emcStatus->task.execState =
			    (enum EMC_TASK_EXEC_ENUM)
			    emcTaskCheckPreconditions(emcTaskCommand);
		    }
		}
	    } else {
		// have an outstanding command
		if (0 != emcTaskIssueCommand(emcTaskCommand)) {
		    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
		    retval = -1;
		} else {
		    emcStatus->task.execState = (enum EMC_TASK_EXEC_ENUM)
			emcTaskCheckPostconditions(emcTaskCommand);
		    emcTaskEager = 1;
		}
		emcTaskCommand = 0;	// reset it
	    }
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_MOTION_QUEUE:
	STEPPING_CHECK();
	if (!emcStatus->motion.traj.queueFull) {
	    if (0 != emcTaskCommand) {
		emcStatus->task.execState = (enum EMC_TASK_EXEC_ENUM)
		    emcTaskCheckPreconditions(emcTaskCommand);
		emcTaskEager = 1;
	    } else {
		emcStatus->task.execState = EMC_TASK_EXEC_DONE;
		emcTaskEager = 1;
	    }
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_MOTION:
	STEPPING_CHECK();
	if (emcStatus->motion.status == RCS_ERROR) {
	    // emcOperatorError(0, "error in motion controller");
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	} else if (emcStatus->motion.status == RCS_DONE) {
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    emcTaskEager = 1;
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_IO:
	STEPPING_CHECK();
	if (emcStatus->io.status == RCS_ERROR) {
	    // emcOperatorError(0, "error in IO controller");
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	} else if (emcStatus->io.status == RCS_DONE) {
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    emcTaskEager = 1;
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_MOTION_AND_IO:
	STEPPING_CHECK();
	if (emcStatus->motion.status == RCS_ERROR) {
	    // emcOperatorError(0, "error in motion controller");
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	} else if (emcStatus->io.status == RCS_ERROR) {
	    // emcOperatorError(0, "error in IO controller");
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	} else if (emcStatus->motion.status == RCS_DONE &&
		   emcStatus->io.status == RCS_DONE) {
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    emcTaskEager = 1;
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_SPINDLE_ORIENTED:
	STEPPING_CHECK(); // not sure
	switch (emcStatus->motion.spindle.orient_state) {
	case EMCMOT_ORIENT_NONE:
	case EMCMOT_ORIENT_COMPLETE:
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    emcStatus->task.delayLeft = 0;
	    emcTaskEager = 1;
	    rcs_print("wait for orient complete: nothing to do\n");
	    break;

	case EMCMOT_ORIENT_IN_PROGRESS:
	    emcStatus->task.delayLeft = taskExecDelayTimeout - etime();
	    if (etime() >= taskExecDelayTimeout) {
		emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
		emcStatus->task.delayLeft = 0;
		emcTaskEager = 1;
		emcOperatorError(0, "wait for orient complete: TIMED OUT");
	    }
	    break;

	case EMCMOT_ORIENT_FAULTED:
	    // actually the code in main() should trap this before we get here
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	    emcStatus->task.delayLeft = 0;
	    emcTaskEager = 1;
	    emcOperatorError(0, "wait for orient complete: FAULTED code=%d", 
			     emcStatus->motion.spindle.orient_fault);
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_DELAY:
	STEPPING_CHECK();
	// check if delay has passed
	emcStatus->task.delayLeft = taskExecDelayTimeout - etime();
	if (etime() >= taskExecDelayTimeout) {
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    emcStatus->task.delayLeft = 0;
	    if (emcStatus->task.input_timeout != 0)
		emcStatus->task.input_timeout = 1; // timeout occurred
	    emcTaskEager = 1;
	}
	// delay can be also be because we wait for an input
	// if the index is set (not -1)
	if (emcAuxInputWaitIndex >= 0) { 
	    switch (emcAuxInputWaitType) {
		case WAIT_MODE_HIGH:
		    if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] != 0) {
			emcStatus->task.input_timeout = 0; // clear timeout flag
			emcAuxInputWaitIndex = -1;
			emcStatus->task.execState = EMC_TASK_EXEC_DONE;
			emcStatus->task.delayLeft = 0;
		    }
		    break;

    		case WAIT_MODE_RISE: 
		    if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] == 0) {
			emcAuxInputWaitType = WAIT_MODE_HIGH;
		    }
		    break;
		    
		case WAIT_MODE_LOW:
		    if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] == 0) {
			emcStatus->task.input_timeout = 0; // clear timeout flag
			emcAuxInputWaitIndex = -1;
			emcStatus->task.execState = EMC_TASK_EXEC_DONE;
			emcStatus->task.delayLeft = 0;
		    }
		    break;

		case WAIT_MODE_FALL: //FIXME: implement different fall mode if needed
		    if (emcStatus->motion.synch_di[emcAuxInputWaitIndex] != 0) {
			emcAuxInputWaitType = WAIT_MODE_LOW;
		    }
		    break;

		case WAIT_MODE_IMMEDIATE:
		    emcStatus->task.input_timeout = 0; // clear timeout flag
		    emcAuxInputWaitIndex = -1;
		    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
		    emcStatus->task.delayLeft = 0;
		    break;
		
		default:
		    emcOperatorError(0, "Unknown Wait Mode");
	    }
	}
	break;

    case EMC_TASK_EXEC_WAITING_FOR_SYSTEM_CMD:
	STEPPING_CHECK();

	// if we got here without a system command pending, say we're done
	if (0 == emcSystemCmdPid) {
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    break;
	}
	// check the status of the system command
	pid = waitpid(emcSystemCmdPid, &status, WNOHANG);

	if (0 == pid) {
	    // child is still executing
	    break;
	}

	if (-1 == pid) {
	    // execution error
	    if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
		rcs_print("emcSystemCmd: error waiting for %d\n",
			  emcSystemCmdPid);
	    }
	    emcSystemCmdPid = 0;
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	    break;
	}

	if (emcSystemCmdPid != pid) {
	    // somehow some other child finished, which is a coding error
	    if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
		rcs_print
		    ("emcSystemCmd: error waiting for system command %d, we got %d\n",
		     emcSystemCmdPid, pid);
	    }
	    emcSystemCmdPid = 0;
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	    break;
	}
	// else child has finished
	if (WIFEXITED(status)) {
	    if (0 == WEXITSTATUS(status)) {
		// child exited normally
		emcSystemCmdPid = 0;
		emcStatus->task.execState = EMC_TASK_EXEC_DONE;
		emcTaskEager = 1;
	    } else {
		// child exited with non-zero status
		if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
		    rcs_print
			("emcSystemCmd: system command %d exited abnormally with value %d\n",
			 emcSystemCmdPid, WEXITSTATUS(status));
		}
		emcSystemCmdPid = 0;
		emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	    }
	} else if (WIFSIGNALED(status)) {
	    // child exited with an uncaught signal
	    if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
		rcs_print("system command %d terminated with signal %d\n",
			  emcSystemCmdPid, WTERMSIG(status));
	    }
	    emcSystemCmdPid = 0;
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	} else if (WIFSTOPPED(status)) {
	    // child is currently being traced, so keep waiting
	} else {
	    // some other status, we'll call this an error
	    emcSystemCmdPid = 0;
	    emcStatus->task.execState = EMC_TASK_EXEC_ERROR;
	}
	break;

    default:
	// coding error
	if (emc_debug & EMC_DEBUG_TASK_ISSUE) {
	    rcs_print_error("invalid execState");
	}
	retval = -1;
	break;
    }
    return retval;
}

// called to allocate and init resources
static int emctask_startup()
{
    double end;
    int good;

#define RETRY_TIME 10.0		// seconds to wait for subsystems to come up
#define RETRY_INTERVAL 1.0	// seconds between wait tries for a subsystem

    // moved up so it can be exposed in taskmodule at init time
    // // get our status data structure
    // emcStatus = new EMC_STAT;

    // get the NML command buffer
    if (!(emc_debug & EMC_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (NULL != emcCommandBuffer) {
	    delete emcCommandBuffer;
	}
	emcCommandBuffer =
	    new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "emc",
				emc_nmlfile);
	if (emcCommandBuffer->valid()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't get emcCommand buffer\n");
	return -1;
    }
    // get our command data structure
    emcCommand = emcCommandBuffer->get_address();

    // get the NML status buffer
    if (!(emc_debug & EMC_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (NULL != emcStatusBuffer) {
	    delete emcStatusBuffer;
	}
	emcStatusBuffer =
	    new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "emc",
				 emc_nmlfile);
	if (emcStatusBuffer->valid()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't get emcStatus buffer\n");
	return -1;
    }

    if (!(emc_debug & EMC_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (NULL != emcErrorBuffer) {
	    delete emcErrorBuffer;
	}
	emcErrorBuffer =
	    new NML(nmlErrorFormat, "emcError", "emc", emc_nmlfile);
	if (emcErrorBuffer->valid()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't get emcError buffer\n");
	return -1;
    }
    // get the timer
    if (!emcTaskNoDelay) {
	timer = new RCS_TIMER(emc_task_cycle_time, "", "");
    }
    // initialize the subsystems

    // IO first

    if (!(emc_debug & EMC_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == emcIoInit()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't initialize IO\n");
	return -1;
    }

    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == emcIoUpdate(&emcStatus->io)) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rcs_print_error("can't read IO status\n");
	return -1;
    }


    // now motion

    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == emcMotionInit()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rcs_print_error("can't initialize motion\n");
	return -1;
    }

    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == emcMotionUpdate(&emcStatus->motion)) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    emctask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rcs_print_error("can't read motion status\n");
	return -1;
    }
    // now the interpreter

    if (0 != emcTaskPlanInit()) {
	rcs_print_error("can't initialize interpreter\n");
	return -1;
    }

    if (done ) {
	emctask_shutdown();
	exit(1);
    }

    // now task
    if (0 != emcTaskInit()) {
	rcs_print_error("can't initialize task\n");
	return -1;
    }
    emcTaskUpdate(&emcStatus->task);

    return 0;
}

// called to deallocate resources
static int emctask_shutdown(void)
{
    // shut down the subsystems
    if (0 != emcStatus) {
	emcTaskHalt();
	emcTaskPlanExit();
	emcMotionHalt();
	emcIoHalt();
    }
    // delete the timer
    if (0 != timer) {
	delete timer;
	timer = 0;
    }
    // delete the NML channels

    if (0 != emcErrorBuffer) {
	delete emcErrorBuffer;
	emcErrorBuffer = 0;
    }

    if (0 != emcStatusBuffer) {
	delete emcStatusBuffer;
	emcStatusBuffer = 0;
	emcStatus = 0;
    }

    if (0 != emcCommandBuffer) {
	delete emcCommandBuffer;
	emcCommandBuffer = 0;
	emcCommand = 0;
    }

    if (0 != emcStatus) {
	delete emcStatus;
	emcStatus = 0;
    }
    return 0;
}

static int iniLoad(const char *filename)
{
    IniFile inifile;
    const char *inistring;
    char version[LINELEN], machine[LINELEN];
    double saveDouble;
    int saveInt;

    // open it
    if (inifile.Open(filename) == false) {
	return -1;
    }

    if (NULL != (inistring = inifile.Find("DEBUG", "EMC"))) {
	// copy to global
	if (1 != sscanf(inistring, "%i", &emc_debug)) {
	    emc_debug = 0;
	}
    } else {
	// not found, use default
	emc_debug = 0;
    }
    if (emc_debug & EMC_DEBUG_RCS) {
	// set_rcs_print_flag(PRINT_EVERYTHING);
	max_rcs_errors_to_print = -1;
    }

    if (emc_debug & EMC_DEBUG_VERSIONS) {
	if (NULL != (inistring = inifile.Find("VERSION", "EMC"))) {
	    if(sscanf(inistring, "$Revision: %s", version) != 1) {
		strncpy(version, "unknown", LINELEN-1);
	    }
	} else {
	    strncpy(version, "unknown", LINELEN-1);
	}

	if (NULL != (inistring = inifile.Find("MACHINE", "EMC"))) {
	    strncpy(machine, inistring, LINELEN-1);
	} else {
	    strncpy(machine, "unknown", LINELEN-1);
	}
	rcs_print("task: machine: '%s'  version '%s'\n", machine, version);
    }

    if (NULL != (inistring = inifile.Find("NML_FILE", "EMC"))) {
	// copy to global
	strcpy(emc_nmlfile, inistring);
    } else {
	// not found, use default
    }

    saveInt = emc_task_interp_max_len; //remember default or previously set value
    if (NULL != (inistring = inifile.Find("INTERP_MAX_LEN", "TASK"))) {
	if (1 == sscanf(inistring, "%d", &emc_task_interp_max_len)) {
	    if (emc_task_interp_max_len <= 0) {
	    	emc_task_interp_max_len = saveInt;
	    }
	} else {
	    emc_task_interp_max_len = saveInt;
	}
    }

    if (NULL != (inistring = inifile.Find("RS274NGC_STARTUP_CODE", "EMC"))) {
	// copy to global
	strcpy(rs274ngc_startup_code, inistring);
    } else {
	if (NULL != (inistring = inifile.Find("RS274NGC_STARTUP_CODE", "RS274NGC"))) {
	    // copy to global
	    strcpy(rs274ngc_startup_code, inistring);
	} else {
	// not found, use default
	}
    }
    saveDouble = emc_task_cycle_time;
    EMC_TASK_CYCLE_TIME_ORIG = emc_task_cycle_time;
    emcTaskNoDelay = 0;
    if (NULL != (inistring = inifile.Find("CYCLE_TIME", "TASK"))) {
	if (1 == sscanf(inistring, "%lf", &emc_task_cycle_time)) {
	    // found it
	    // if it's <= 0.0, then flag that we don't want to
	    // wait at all, which will set the EMC_TASK_CYCLE_TIME
	    // global to the actual time deltas
	    if (emc_task_cycle_time <= 0.0) {
		emcTaskNoDelay = 1;
	    }
	} else {
	    // found, but invalid
	    emc_task_cycle_time = saveDouble;
	    rcs_print
		("invalid [TASK] CYCLE_TIME in %s (%s); using default %f\n",
		 filename, inistring, emc_task_cycle_time);
	}
    } else {
	// not found, using default
	rcs_print("[TASK] CYCLE_TIME not found in %s; using default %f\n",
		  filename, emc_task_cycle_time);
    }


    if (NULL != (inistring = inifile.Find("NO_FORCE_HOMING", "TRAJ"))) {
	if (1 == sscanf(inistring, "%d", &no_force_homing)) {
	    // found it
	    // if it's <= 0.0, then set it 0 so that homing is required before MDI or Auto
	    if (no_force_homing <= 0) {
		no_force_homing = 0;
	    }
	} else {
	    // found, but invalid
	    no_force_homing = 0;
	    rcs_print
		("invalid [TRAJ] NO_FORCE_HOMING in %s (%s); using default %d\n",
		 filename, inistring, no_force_homing);
	}
    } else {
	// not found, using default
	no_force_homing = 0;
    }

    // configurable template for iocontrol reason display
    if (NULL != (inistring = inifile.Find("IO_ERROR", "TASK"))) {
	io_error = strdup(inistring);
    }

    // max number of queued MDI commands
    if (NULL != (inistring = inifile.Find("MDI_QUEUED_COMMANDS", "TASK"))) {
	max_mdi_queued_commands = atoi(inistring);
    }

    // close it
    inifile.Close();

    return 0;
}

/*
  syntax: a.out {-d -ini <inifile>} {-nml <nmlfile>} {-shm <key>}
  */
int main(int argc, char *argv[])
{
    int taskPlanError = 0;
    int taskExecuteError = 0;
    double startTime, endTime, deltaTime;
    double first_start_time;
    int num_latency_warnings = 0;
    int latency_excursion_factor = 10;  // if latency is worse than (factor * expected), it's an excursion
    double minTime, maxTime;

    bindtextdomain("linuxcnc", EMC2_PO_DIR);
    setlocale(LC_MESSAGES,"");
    setlocale(LC_CTYPE,"");
    textdomain("linuxcnc");

    // loop until done
    done = 0;
    // trap ^C
    signal(SIGINT, emctask_quit);
    // and SIGTERM (used by runscript to shut down)
    signal(SIGTERM, emctask_quit);

    // create a backtrace on stderr
    signal(SIGSEGV, backtrace);
    signal(SIGFPE, backtrace);
    signal(SIGUSR1, backtrace);

    // set print destination to stdout, for console apps
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);
    // process command line args
    if (0 != emcGetArgs(argc, argv)) {
	rcs_print_error("error in argument list\n");
	exit(1);
    }

    if (done) {
	emctask_shutdown();
	exit(1);
    }
    // initialize globals
    emcInitGlobals();

    if (done) {
	emctask_shutdown();
	exit(1);
    }
    // get configuration information
    iniLoad(emc_inifile);

    if (done) {
	emctask_shutdown();
	exit(1);
    }

    // get our status data structure
    // moved up from emc_startup so we can expose it in Python right away
    emcStatus = new EMC_STAT;

    // get the Python plugin going

    // inistantiate task methods object, too
    emcTaskOnce(emc_inifile);
    if (task_methods == NULL) {
	set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
	rcs_print_error("can't initialize Task methods\n");
	emctask_shutdown();
	exit(1);
    }

    // this is the place to run any post-HAL-creation halcmd files
    emcRunHalFiles(emc_inifile);

    // initialize everything
    if (0 != emctask_startup()) {
	emctask_shutdown();
	exit(1);
    }
    // set the default startup modes
    emcMotionAbort();
    emcSpindleAbort();
    emcAuxEstopOn();
    for (int t = 0; t < emcStatus->motion.traj.axes; t++) {
        emcAxisDisable(t);
    }
    emcTrajDisable();
    emcLubeOff();
    emcIoAbort(EMC_ABORT_TASK_STATE_ESTOP);
    emcAxisUnhome(-2);

    emcTrajSetMode(EMC_TRAJ_MODE_FREE);

    // reflect the initial value of EMC_DEBUG in emcStatus->debug
    emcStatus->debug = emc_debug;

    startTime = etime();	// set start time before entering loop;
    first_start_time = startTime;
    endTime = startTime;
    // it will be set at end of loop from now on
    minTime = DBL_MAX;		// set to value that can never be exceeded
    maxTime = 0.0;		// set to value that can never be underset

    while (!done) {
        check_ini_hal_items();
	// read command
	if (0 != emcCommandBuffer->read()) {
	    // got a new command, so clear out errors
	    taskPlanError = 0;
	    taskExecuteError = 0;
	}
	// run control cycle
	if (0 != emcTaskPlan()) {
	    taskPlanError = 1;
	}
	if (0 != emcTaskExecute()) {
	    taskExecuteError = 1;
	}
	// update subordinate status

	emcIoUpdate(&emcStatus->io);
	emcMotionUpdate(&emcStatus->motion);
	// synchronize subordinate states
	if (emcStatus->io.aux.estop) {
	    if (emcStatus->motion.traj.enabled) {
		emcTrajDisable();
		emcTaskAbort();
		emcIoAbort(EMC_ABORT_AUX_ESTOP);
		emcSpindleAbort();
		emcAxisUnhome(-2); // only those joints which are volatile_home
		mdi_execute_abort();
		emcAbortCleanup(EMC_ABORT_AUX_ESTOP);
		emcTaskPlanSynch();
	    }
	    if (emcStatus->io.coolant.mist) {
		emcCoolantMistOff();
	    }
	    if (emcStatus->io.coolant.flood) {
		emcCoolantFloodOff();
	    }
	    if (emcStatus->io.lube.on) {
		emcLubeOff();
	    }
	    if (emcStatus->motion.spindle.enabled) {
		emcSpindleOff();
	    }
	}

	// toolchanger indicated fault code > 0
	if ((emcStatus->io.status == RCS_ERROR) &&
	    emcStatus->io.fault) {
	    static int reported = -1;
	    if (emcStatus->io.reason > 0) {
		if (reported ^ emcStatus->io.fault) {
		    rcs_print("M6: toolchanger soft fault=%d, reason=%d\n",
			      emcStatus->io.fault, emcStatus->io.reason);
		    reported = emcStatus->io.fault;
		}
		emcStatus->io.status = RCS_DONE; // let program continue
	    } else {
		rcs_print("M6: toolchanger hard fault, reason=%d\n",
			  emcStatus->io.reason);
		// abort since io.status is RCS_ERROR
	    }

	}

	// check for subordinate errors, and halt task if so
	if (emcStatus->motion.status == RCS_ERROR ||
	    ((emcStatus->io.status == RCS_ERROR) &&
	     (emcStatus->io.reason <= 0))) {

	    /*! \todo FIXME-- duplicate code for abort,
	      also in emcTaskExecute()
	      and in emcTaskIssueCommand() */

	    if (emcStatus->io.status == RCS_ERROR) {
		// this is an aborted M6.
		if (emc_debug & EMC_DEBUG_RCS ) {
		    rcs_print("io.status=RCS_ERROR, fault=%d reason=%d\n",
			      emcStatus->io.fault, emcStatus->io.reason);
		}
		if (emcStatus->io.reason < 0) {
		    emcOperatorError(0, io_error, emcStatus->io.reason);
		}
	    }
	    // motion already should have reported this condition (and set RCS_ERROR?)
	    // an M19 orient failed to complete within timeout
	    // if ((emcStatus->motion.status == RCS_ERROR) && 
	    // 	(emcStatus->motion.spindle.orient_state == EMCMOT_ORIENT_FAULTED) &&
	    // 	(emcStatus->motion.spindle.orient_fault != 0)) {
	    // 	emcOperatorError(0, "wait for orient complete timed out");
	    // }

            // abort everything
            emcTaskAbort();
            emcIoAbort(EMC_ABORT_MOTION_OR_IO_RCS_ERROR);
            emcSpindleAbort();
	    mdi_execute_abort();
	    // without emcTaskPlanClose(), a new run command resumes at
	    // aborted line-- feature that may be considered later
	    {
		int was_open = taskplanopen;
		emcTaskPlanClose();
		if (emc_debug & EMC_DEBUG_INTERP && was_open) {
		    rcs_print("emcTaskPlanClose() called at %s:%d\n",
			      __FILE__, __LINE__);
		}
	    }

	    // clear out the pending command
	    emcTaskCommand = 0;
	    interp_list.clear();
	    emcStatus->task.currentLine = 0;

	    emcAbortCleanup(EMC_ABORT_MOTION_OR_IO_RCS_ERROR);

	    // clear out the interpreter state
	    emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
	    emcStatus->task.execState = EMC_TASK_EXEC_DONE;
	    stepping = 0;
	    steppingWait = 0;

	    // now queue up command to resynch interpreter
	    emcTaskQueueCommand(&taskPlanSynchCmd);
	}

	// update task-specific status
	emcTaskUpdate(&emcStatus->task);

	// handle RCS_STAT_MSG base class members explicitly, since this
	// is not an NML_MODULE and they won't be set automatically

	// do task
	emcStatus->task.command_type = emcCommand->type;
	emcStatus->task.echo_serial_number = emcCommand->serial_number;

	// do top level
	emcStatus->command_type = emcCommand->type;
	emcStatus->echo_serial_number = emcCommand->serial_number;

	if (taskPlanError || taskExecuteError ||
	    emcStatus->task.execState == EMC_TASK_EXEC_ERROR ||
	    emcStatus->motion.status == RCS_ERROR ||
	    emcStatus->io.status == RCS_ERROR) {
	    emcStatus->status = RCS_ERROR;
	    emcStatus->task.status = RCS_ERROR;
	} else if (!taskPlanError && !taskExecuteError &&
		   emcStatus->task.execState == EMC_TASK_EXEC_DONE &&
		   emcStatus->motion.status == RCS_DONE &&
		   emcStatus->io.status == RCS_DONE &&
		   mdi_execute_queue.len() == 0 &&
		   interp_list.len() == 0 &&
		   emcTaskCommand == 0 &&
		   emcStatus->task.interpState == EMC_TASK_INTERP_IDLE) {
	    emcStatus->status = RCS_DONE;
	    emcStatus->task.status = RCS_DONE;
	} else {
	    emcStatus->status = RCS_EXEC;
	    emcStatus->task.status = RCS_EXEC;
	}

	// write it
	// since emcStatus was passed to the WM init functions, it
	// will be updated in the _update() functions above. There's
	// no need to call the individual functions on all WM items.
	emcStatusBuffer->write(emcStatus);

	// wait on timer cycle, if specified, or calculate actual
	// interval if ini file says to run full out via
	// [TASK] CYCLE_TIME <= 0.0d
	// emcTaskEager = 0;
        endTime = etime();
        deltaTime = endTime - startTime;
        if (deltaTime < minTime)
            minTime = deltaTime;
        else if (deltaTime > maxTime)
            maxTime = deltaTime;
        startTime = endTime;
        if (deltaTime > (latency_excursion_factor * emc_task_cycle_time)) {
            if (num_latency_warnings < 10) {
                rcs_print("task: main loop took %.6f seconds\n", deltaTime);
            }
            num_latency_warnings ++;
        }

	if ((emcTaskNoDelay) || (emcTaskEager)) {
	    emcTaskEager = 0;
	} else {
	    timer->wait();
	}
    }
    // end of while (! done)

    rcs_print(
        "task: %u cycles, min=%.6f, max=%.6f, avg=%.6f, %u latency excursions (> %dx expected cycle time of %.6fs)\n",
        emcStatus->task.heartbeat,
        minTime,
        maxTime,
        (emcStatus->task.heartbeat != 0) ?  (endTime - first_start_time) / emcStatus->task.heartbeat : -1.0,
        num_latency_warnings,
        latency_excursion_factor,
        emc_task_cycle_time
    );

    // clean up everything
    emctask_shutdown();

    // and leave
    exit(0);
}
