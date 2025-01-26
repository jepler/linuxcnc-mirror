/********************************************************************
* Description: emcsh.cc
*   Extended-Tcl-based EMC automatic test interface
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
*         Reorganized by Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <tcl.h>
#include <tk.h>

#include "emc/linuxcnc.h"
#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"		// EMC NML
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "timer.hh"
#include <rtapi_string.h>

#include "shcom.hh"

#define setresult(t,s) Tcl_SetObjResult((t), Tcl_NewStringObj((s),-1))

/*
  Using tcl package Linuxcnc:
  Using emcsh:

  % package require Linuxcnc
  % emc_init -ini <INI file> # to start with an INI file
  or
  % emc_init # to start with the default inifilename (emc.ini)

  With filename, it opens NML buffers to the EMC, runs the script, closes
  the buffers, and quits.

  With -ini <INI file>, uses specified INI file instead of default emc.ini.

  Commands in the LinuxCNC package are all prefixed with "emc_", which makes them
  somewhat inconvenient for typing but avoids name conflicts, e.g., open.

  Some commands take 0 or more arguments. 0 arguments means they return
  the associated value; the argument would be to set the value.

  Commands are sent to the EMC, and control resumes immediately. You can
  call a timed wait until the command got there, or a timed wait until the
  command completed, or not wait at all.

  EMC commands:

  emc_plat
  Returns the platform for which this was compiled, e.g., linux_2_0_36

  emc_ini <var> <section>
  Returns the string value of <var> in section <section>, in EMC_INIFILE

  emc_debug {<new value>}
  With no arg, returns the integer value of EMC_DEBUG, in the EMC. Note that
  it may not be true that the local EMC_DEBUG variable here (in emcsh and
  the GUIs that use it) is the same as the EMC_DEBUG value in the EMC. This
  can happen if the EMC is started from one INI file, and the GUI is started
  with another that has a different value for DEBUG.
  With an arg, sends a command to the EMC to set the new debug level,
  and sets the EMC_DEBUG global here to the same value. This will make
  the two values the same, since they really ought to be the same.

  emc_set_wait none | received | done
  Set the wait for commands to return to be right away (none), after the
  command was sent and received (received), or after the command was
  done (done).

  emc_wait received | done
  Force a wait for the previous command to be received, or done. This lets
  you wait in the event that "emc_set_wait none" is in effect.

  emc_set_timeout <timeout>
  Set the timeout for commands to return to <timeout>, in seconds. Timeout
  is a real number. If it's <= 0.0, it means wait forever. Default is 0.0,
  wait forever.

  emc_update (none) | none | auto
  With no arg, forces an update of the EMC status. With "none", doesn't
  cause an automatic update of status with other emc_ words. With "auto",
  makes emc_ words automatically update status before they return values.

  emc_error
  Returns the current EMC error string, or "ok" if no error.

  emc_operator_display
  Returns the current EMC operator display string, or "ok" if none.

  emc_operator_text
  Returns the current EMC operator text string, or "ok" if none.

  emc_time
  Returns the time, in seconds, from the start of the epoch. This starting
  time depends on the platform.

  emc_estop (none) | on | off
  With no arg, returns the estop setting as "on" or "off". Otherwise,
  sends an estop on or off command.

  emc_machine (none) | on | off
  With no arg, returns the machine setting as "on" or "off". Otherwise,
  sends a machine on or off command.

  emc_mode (none) | manual | auto | mdi
  With no arg, returns the mode setting as "manual", "auto", or "mdi".
  Otherwise, sends a mode manual, auto, or mdi command.

  emc_mist (none) | on | off
  With no arg, returns the mist setting as "on" or "off". Otherwise,
  sends a mist on or off command.

  emc_flood (none) | on | off
  With no arg, returns the flood setting as "on" or "off". Otherwise,
  sends a flood on or off command.

  emc_spindle (spindle_number) (none) | forward | reverse | increase | decrease | constant | off
  With no spindle_number defaults to spindle 0. This is a little different
  from the default behaviour elsewhere where specifyin no spindle affects all spindles.
  With no arg, returns the value of the spindle state as "forward",
  "reverse", "increase", "decrease", or "off". With arg, sends the spindle
  command. Note that "increase" and "decrease" will cause a speed change in
  the corresponding direction until a "constant" command is sent.

  emc_brake (none) | on | off
  With no arg, returns the brake setting. Otherwise sets the brake.

  emc_tool
  Returns the id of the currently loaded tool

  emc_tool_offset X | Y | ...
  Returns the currently applied tool length offset

  emc_load_tool_table <file>
  Loads the tool table specified by <file>

  emc_home 0 | 1 | 2 | ...
  Homes the indicated joint.

  emc_unhome 0 | 1 | 2 | ...
  Unhomes the indicated joint.

  emc_jog_stop 0 | 1 | 2 | ...
  Stop the joint jog

  emc_jog 0 | 1 | 2 | ... <speed>
  Jog the indicated joint at <speed>; sign of speed is direction

  emc_jog_incr 0 | 1 | 2 | ... <speed> <incr>
  Jog the indicated joint by increment <incr> at the <speed>; sign of
  speed is direction

  emc_feed_override {<percent>}
  With no args, returns the current feed override, as a percent. With
  argument, set the feed override to be the percent value

  emc_rapid_override {<percent>}
  With no args, returns the current rapid override, as a percent. With
  argument, set the rapid override to be the percent value

  emc_spindle_override {<percent>}
  With no args, returns the current spindle override, as a percent. With
  argument, set the spindle override to be the percent value

  emc_abs_cmd_pos X | Y | ...
  Returns double obj containing the commanded pos in abs coords,
  at given index, 0 = X, etc.

  emc_abs_act_pos X | Y | ...
  Returns double objs containing the actual pos in abs coords

  emc_rel_cmd_pos  X | Y | ...
  Returns double obj containing the commanded pos in rel coords,
  including tool length offset

  emc_rel_act_pos  X | Y | ...
  Returns double objs containing the actual pos in rel coords,
  including tool length offset

  emc_joint_pos
  Returns double objs containing the actual pos in absolute coords of individual
  joint/slider positions, excludes tool length offset

  emc_pos_offset X | Y | Z | A | B | C | U | V | W
  Returns the position offset associated with the world coordinate provided

  emc_joint_limit 0 | 1 | ...
  Returns "ok", "minsoft", "minhard", "maxsoft", "maxhard"

  emc_joint_fault 0 | 1 | ...
  Returns "ok" or "fault"

  emc_joint_homed 0 | 1 | ...
  Returns "homed", "not"

  emc_mdi <string>
  Sends the <string> as an MDI command

  emc_task_plan_init
  Initializes the program interpreter

  emc_open <filename>
  Opens the named file

  emc_run {<start line>}
  Without start line, runs the opened program from the beginning. With
  start line, runs from that line. A start line of -1 runs in verify mode.

  emc_pause
  Pause program execution

  emc_resume
  Resume program execution

  emc_step
  Step the program one line

  emc_program
  Returns the name of the currently opened program, or "none"

  emc_program_line
  Returns the currently executing line of the program

  emc_program_status
  Returns "idle", "running", or "paused"

  emc_program_codes
  Returns the string for the currently active program codes

  emc_override_limit none | 0 | 1
  returns state of override, sets it or deactivates it (used to jog off hardware limit switches)

  emc_optional_stop  none | 0 | 1
  returns state of optional setop, sets it or deactivates it (used to stop/continue on M1)

  emc_program_codes
  Returns the string for the currently active program codes

  emc_joint_type <joint>
  Returns "linear", "angular", or "custom" for the type of the specified joint

  emc_joint_units <joint>
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the corresponding native units of the specified joint. The type
  of the joint (linear or angular) is used to resolve which type of units
  are returned. The units are obtained heuristically, based on the
  EMC_JOINT_STAT::units numerical value of user units per mm or deg.
  For linear joints, something close to 0.03937 is deemed "inch",
  1.000 is "mm", 0.1 is "cm", otherwise it's "custom".
  For angular joints, something close to 1.000 is deemed "deg",
  PI/180 is "rad", 100/90 is "grad", otherwise it's "custom".

  emc_program_units
  emc_program_linear_units
  Returns "inch", "mm", "cm", or "none", for the corresponding linear
  units that are active in the program interpreter.

  emc_program_angular_units
  Returns "deg", "rad", "grad", or "none" for the corresponding angular
  units that are active in the program interpreter.

  emc_user_linear_units
  Returns "inch", "mm", "cm", or "custom", for the
  corresponding native user linear units of the EMC trajectory
  level. This is obtained heuristically, based on the
  EMC_TRAJ_STAT::linearUnits numerical value of user units per mm.
  Something close to 0.03937 is deemed "inch", 1.000 is "mm", 0.1 is
  "cm", otherwise it's "custom".

  emc_user_angular_units
  Returns "deg", "rad", "grad", or "custom" for the corresponding native
  user angular units of the EMC trajectory level. Like with linear units,
  this is obtained heuristically.

  emc_display_linear_units
  emc_display_angular_units
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the linear or angular units that are active in the display.
  This is effectively the value of linearUnitConversion or
  angularUnitConversion, resp.

  emc_linear_unit_conversion {inch | mm | cm | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.

  emc_angular_unit_conversion {deg | rad | grad | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.

  emc_probe_clear
  Clear the probe tripped flag.

  emc_probe_tripped
  Has the probe been tripped since the last clear.

  emc_probe_value
  Value of current probe signal. (read-only)

  emc_probe
  Move toward a certain location. If the probe is tripped on the way stop
  motion, record the position and raise the probe tripped flag.

  emc_teleop_enable
  Should motion run in teleop mode? (No args
  gets it, one arg sets it.)

  emc_kinematics_type
  returns the type of kinematics functions used identity=1, serial=2,
  parallel=3, custom=4
*/

#define CHECKEMC \
    if (!checkStatus() ) {\
        setresult(interp,"emc not connected");\
        return TCL_ERROR;\
    }

static void thisQuit(ClientData /*clientData*/)
{
    EMC_NULL emc_null_msg;

    if (0 != emcStatusBuffer) {
	// wait until current message has been received
	emcCommandWaitReceived();
    }

    // clean up NML buffers

    if (emcErrorBuffer != 0) {
	delete emcErrorBuffer;
	emcErrorBuffer = 0;
    }

    if (emcStatusBuffer != 0) {
	delete emcStatusBuffer;
	emcStatusBuffer = 0;
	emcStatus = 0;
    }

    if (emcCommandBuffer != 0) {
	delete emcCommandBuffer;
	emcCommandBuffer = 0;
    }

    return;
}


/* EMC command functions */

static int emc_plat(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST * /*objv*/)
{
    if (objc == 1) {
	setresult(interp,"Linux");
	return TCL_OK;
    }

    setresult(interp,"emc_plat: need no args");
    return TCL_ERROR;
}

static int emc_ini(ClientData /*clientdata*/,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    IniFile inifile;
    std::optional<const char *> inistring;
    const char *varstr, *secstr, *defaultstr;
    defaultstr = 0;

    if (objc != 3 && objc != 4) {
	setresult(interp,"emc_ini: need 'var' and 'section'");
	return TCL_ERROR;
    }
    // open it
    if (inifile.Open(emc_inifile) == false) {
	return TCL_OK;
    }

    varstr = Tcl_GetStringFromObj(objv[1], 0);
    secstr = Tcl_GetStringFromObj(objv[2], 0);

    if (objc == 4) {
	defaultstr = Tcl_GetStringFromObj(objv[3], 0);
    }

    if (!(inistring = inifile.Find(varstr, secstr))) {
	if (defaultstr != 0) {
	    setresult(interp,(char *) defaultstr);
	}
	return TCL_OK;
    }

    setresult(interp, *inistring);

    // close it
    inifile.Close();

    return TCL_OK;
}

static int emc_Debug(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    Tcl_Obj *debug_obj;
    int debug;

    CHECKEMC
    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (objc == 1) {
	// no arg-- return current value
	debug_obj = Tcl_NewIntObj(emcStatus->debug);
	Tcl_SetObjResult(interp, debug_obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (0 != Tcl_GetIntFromObj(0, objv[1], &debug)) {
	    setresult(interp,"emc_debug: need debug level as integer");
	    return TCL_ERROR;
	}
	sendDebug(debug);
	emc_debug = debug;
	return TCL_OK;
    }
    // wrong number of args
    setresult(interp,"emc_debug: need zero or one arg");
    return TCL_ERROR;
}

static int emc_set_wait(ClientData /*clientdata*/,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	switch (emcWaitType) {
	case EMC_WAIT_RECEIVED:
	    setresult(interp,"received");
	    break;
	case EMC_WAIT_DONE:
	    setresult(interp,"done");
	    break;
	default:
	    setresult(interp,"(invalid)");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "received")) {
	    emcWaitType = EMC_WAIT_RECEIVED;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "done")) {
	    emcWaitType = EMC_WAIT_DONE;
	    return TCL_OK;
	}
    }

    setresult(interp, "emc_set_wait: need 'received', 'done', or no args");
    return TCL_ERROR;
}

static int emc_wait(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "received")) {
	    if (0 != emcCommandWaitReceived()) {
		setresult(interp,"timeout");
	    }
	    return TCL_OK;
	}
	if (!strcmp(objstr, "done")) {
	    if (0 != emcCommandWaitDone()) {
		setresult(interp,"timeout");
	    }
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_wait: need 'received' or 'done'");
    return TCL_ERROR;
}

static int emc_set_timeout(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    double timeout;
    Tcl_Obj *timeout_obj;

    CHECKEMC
    if (objc == 1) {
	timeout_obj = Tcl_NewDoubleObj(emcTimeout);
	Tcl_SetObjResult(interp, timeout_obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (TCL_OK == Tcl_GetDoubleFromObj(0, objv[1], &timeout)) {
	    emcTimeout = timeout;
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_set_timeout: need time as real number");
    return TCL_ERROR;
}

static int emc_update(ClientData /*clientdata*/,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	updateStatus();
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "none")) {
	    emcUpdateType = EMC_UPDATE_NONE;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    emcUpdateType = EMC_UPDATE_AUTO;
	    return TCL_OK;
	}
    }

    return TCL_OK;
}

static int emc_time(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc == 1) {
	Tcl_SetObjResult(interp, Tcl_NewDoubleObj(etime()));
	return TCL_OK;
    }

    setresult(interp,"emc_time: needs no arguments");
    return TCL_ERROR;
}

static int emc_error(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST * /*objv*/)
{

    CHECKEMC
    if (objc == 1) {
	// get any new error, it's saved in global error_string[]
	if (0 != updateError()) {
	    setresult(interp,"emc_error: bad status from EMC");
	    return TCL_ERROR;
	}
	// put error on result list
	if (error_string[0] == 0) {
	    setresult(interp,"ok");
	} else {
	    setresult(interp,error_string);
	    error_string[0] = 0;
	}
	return TCL_OK;
    }

    setresult(interp,"emc_error: need no args");
    return TCL_ERROR;
}

static int emc_operator_text(ClientData /*clientdata*/,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST * /*objv*/)
{

    CHECKEMC
    if (objc == 1) {
	// get any new string, it's saved in global operator_text_string[]
	if (0 != updateError()) {
	    setresult(interp,"emc_operator_text: bad status from EMC");
	    return TCL_ERROR;
	}
	// put error on result list
	if (operator_text_string[0] == 0) {
	    setresult(interp,"ok");
	    operator_text_string[0] = 0;
	} else {
	    setresult(interp,operator_text_string);
	}
	return TCL_OK;
    }

    setresult(interp,"emc_operator_text: need no args");
    return TCL_ERROR;
}

static int emc_operator_display(ClientData /*clientdata*/,
				Tcl_Interp * interp, int objc,
				Tcl_Obj * CONST * /*objv*/)
{

    CHECKEMC
    if (objc == 1) {
	// get any new string, it's saved in global operator_display_string[]
	if (0 != updateError()) {
	    setresult(interp,"emc_operator_display: bad status from EMC");
	    return TCL_ERROR;
	}
	// put error on result list
	if (operator_display_string[0] == 0) {
	    setresult(interp,"ok");
	} else {
	    setresult(interp,operator_display_string);
	    operator_display_string[0] = 0;
	}
	return TCL_OK;
    }

    setresult(interp,"emc_operator_display: need no args");
    return TCL_ERROR;
}

static int emc_estop(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	if (emcStatus->task.state == EMC_TASK_STATE::ESTOP) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendEstop();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendEstopReset();
	    return TCL_OK;
	}
    }

    setresult(interp, "emc_estop: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int emc_machine(ClientData /*clientdata*/,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	if (emcStatus->task.state == EMC_TASK_STATE::ON) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendMachineOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendMachineOff();
	    return TCL_OK;
	}
    }

    setresult(interp, "emc_machine: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int emc_mode(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	switch (emcStatus->task.mode) {
	case EMC_TASK_MODE::MANUAL:
	    setresult(interp,"manual");
	    break;
	case EMC_TASK_MODE::AUTO:
	    setresult(interp,"auto");
	    break;
	case EMC_TASK_MODE::MDI:
	    setresult(interp,"mdi");
	    break;
	default:
	    setresult(interp,"?");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "manual")) {
	    sendManual();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    sendAuto();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "mdi")) {
	    sendMdi();
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_mode: need 'manual', 'auto', 'mdi', or no args");
    return TCL_ERROR;
}

static int emc_mist(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	if (emcStatus->io.coolant.mist == 1) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendMistOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendMistOff();
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_mist: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int emc_flood(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	if (emcStatus->io.coolant.flood == 1) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendFloodOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendFloodOff();
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_flood: need 'on', 'off', or no args"); return TCL_ERROR;
}

static int emc_spindle(ClientData /*clientdata*/,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    char *objstr = NULL;
    int spindle = 0;

    CHECKEMC

    if (objc >= 2) {
        if (Tcl_GetIntFromObj(interp, objv[1], &spindle) != TCL_OK){ // not a likely spindle index first, then
            spindle = 0;
            objstr = Tcl_GetStringFromObj(objv[1], 0);
        } else {
            if (spindle < 0 || spindle > EMCMOT_MAX_SPINDLES){ // should really be num_spindles, but not sure we know that here
                setresult(interp,"invalid spindle index number");
                return TCL_ERROR;
            }
            objstr = Tcl_GetStringFromObj(objv[2], 0);
        }
    }
    if (objstr) {
        if (!strcmp(objstr, "forward")) {
            sendSpindleForward(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "reverse")) {
            sendSpindleReverse(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "increase")) {
            sendSpindleIncrease(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "decrease")) {
            sendSpindleDecrease(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "constant")) {
            sendSpindleConstant(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "off")) {
            sendSpindleOff(spindle);
            return TCL_OK;
        }
    }
    else
    {
    // no arg-- return status
        if (emcUpdateType == EMC_UPDATE_AUTO) {
            updateStatus();
        }
        if (emcStatus->motion.spindle[spindle].increasing > 0) {
            setresult(interp,"increase");
        } else if (emcStatus->motion.spindle[spindle].increasing < 0) {
            setresult(interp,"decrease");
        } else if (emcStatus->motion.spindle[spindle].direction > 0) {
            setresult(interp,"forward");
        } else if (emcStatus->motion.spindle[spindle].direction < 0) {
            setresult(interp,"reverse");
        } else {
            setresult(interp,"off");
        }
        return TCL_OK;
    }
    setresult(interp,"emc_spindle: need 'on', 'off', a spindle index or no args");
    return TCL_ERROR;
}

static int emc_brake(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr = NULL;
    int spindle = 0;

    CHECKEMC

    if (objc >= 2) {
        if (Tcl_GetIntFromObj(interp, objv[1], &spindle) != TCL_OK){ // not a likely spindle index first, then
            spindle = 0;
            objstr = Tcl_GetStringFromObj(objv[1], 0);
        } else {
            if (spindle < 0 || spindle > EMCMOT_MAX_SPINDLES){ // FIXME: should really be num_spindles, but not sure we know that here
                setresult(interp,"invalid spindle index number");
                return TCL_ERROR;
            }
            objstr = Tcl_GetStringFromObj(objv[2], 0);
        }
    }

    if (objstr) {
        if (!strcmp(objstr, "on")) {
            sendBrakeEngage(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "off")) {
            sendBrakeRelease(spindle);
            return TCL_OK;
        }
    }
    else
    {
        // no arg-- return status
        if (emcUpdateType == EMC_UPDATE_AUTO) {
            updateStatus();
        }
        if (emcStatus->motion.spindle[spindle].brake == 1) {
            setresult(interp,"on");
        } else {
            setresult(interp,"off");
        }
    return TCL_OK;
    }
    setresult(interp,"emc_brake: need 'on', 'off', spindle index or no args");
    return TCL_ERROR;
}

static int emc_tool(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *toolobj;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_tool: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    toolobj = Tcl_NewIntObj(emcStatus->io.tool.toolInSpindle);

    Tcl_SetObjResult(interp, toolobj);
    return TCL_OK;
}

static int emc_tool_offset(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *tlobj;
    string[0] = 'Z'; //default if not specified

    CHECKEMC
    if (objc > 2) {
	setresult(interp,"emc_tool_offset: need 0 or 1 args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (objc != 1) {
       strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);
    }

    switch (string[0]) {
    case 'x': case 'X':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                emcStatus->task.toolOffset.tran.x));
        break;
    case 'y': case 'Y':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                emcStatus->task.toolOffset.tran.y));
        break;
    case 'z': case 'Z':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                emcStatus->task.toolOffset.tran.z));
        break;
    case 'a': case 'A':
        tlobj = Tcl_NewDoubleObj(convertAngularUnits(
                                 emcStatus->task.toolOffset.a));
        break;
    case 'b': case 'B':
        tlobj = Tcl_NewDoubleObj(convertAngularUnits(
                                 emcStatus->task.toolOffset.b));
        break;
    case 'c': case 'C':
        tlobj = Tcl_NewDoubleObj(convertAngularUnits(
                                 emcStatus->task.toolOffset.c));
        break;
    case 'u': case 'U':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                 emcStatus->task.toolOffset.u));
        break;
    case 'v': case 'V':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                 emcStatus->task.toolOffset.v));
        break;
    case 'w': case 'W':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                 emcStatus->task.toolOffset.w));
        break;
    default:
        setresult(interp,"emc_tool_offset: bad coordinate letter argument");
        return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, tlobj);
    return TCL_OK;
}

static int emc_load_tool_table(ClientData /*clientdata*/,
			       Tcl_Interp * interp, int objc,
			       Tcl_Obj * CONST objv[])
{
    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_load_tool_table: need file");
	return TCL_ERROR;
    }

    if (0 != sendLoadToolTable(Tcl_GetStringFromObj(objv[1], 0))) {
	setresult(interp,"emc_load_tool_table: can't open file");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_set_tool_offset(ClientData /*clientdata*/,
			       Tcl_Interp * interp, int objc,
			       Tcl_Obj * CONST objv[])
{
    int tool;
    double length;
    double diameter;

    CHECKEMC
    if (objc != 4) {
	setresult(interp,"emc_set_tool_offset: need <tool> <length> <diameter>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &tool)) {
	setresult(interp,"emc_set_tool_offset: need tool as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[2], &length)) {
	setresult(interp,"emc_set_tool_offset: need length as real number");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &diameter)) {
	setresult(interp,"emc_set_tool_offset: need diameter as real number");
	return TCL_ERROR;
    }

    if (0 != sendToolSetOffset(tool, length, diameter)) {
	setresult(interp,"emc_set_tool_offset: can't set it");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_abs_cmd_pos(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_abs_cmd_pos: need exactly 1 coordinate letter");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.position.tran.x));
        break;
    case 'y': case 'Y':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.position.tran.y));
        break;
    case 'z': case 'Z':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.position.tran.z));
        break;
    case 'a': case 'A':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.position.a));
        break;
    case 'b': case 'B':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.position.b));
        break;
    case 'c': case 'C':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.position.c));
        break;
    case 'u': case 'U':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.position.u));
        break;
    case 'v': case 'V':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.position.v));
        break;
    case 'w': case 'W':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.position.w));
        break;
    default:
        setresult(interp,"emc_abs_cmd_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int emc_abs_act_pos(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_abs_act_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.actualPosition.tran.x));
        break;
    case 'y': case 'Y':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.actualPosition.tran.y));
        break;
    case 'z': case 'Z':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.actualPosition.tran.z));
        break;
    case 'a': case 'A':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.actualPosition.a));
        break;
    case 'b': case 'B':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.actualPosition.b));
        break;
    case 'c': case 'C':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.actualPosition.c));
        break;
    case 'u': case 'U':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.actualPosition.u));
        break;
    case 'v': case 'V':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.actualPosition.v));
        break;
    case 'w': case 'W':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.actualPosition.w));
        break;
    default:
        setresult(interp,"emc_abs_act_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int emc_rel_cmd_pos(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_rel_cmd_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    double d = 0.0;
    switch (string[0]) {
    case 'x': case 'X':
        d = convertLinearUnits(emcStatus->motion.traj.position.tran.x -
                               emcStatus->task.g5x_offset.tran.x -
                               emcStatus->task.g92_offset.tran.x -
                               emcStatus->task.toolOffset.tran.x);
        break;
    case 'y': case 'Y':
        d = convertLinearUnits(emcStatus->motion.traj.position.tran.y -
                               emcStatus->task.g5x_offset.tran.y -
                               emcStatus->task.g92_offset.tran.y -
                               emcStatus->task.toolOffset.tran.y);
        break;
    case 'z': case 'Z':
        d = convertLinearUnits(emcStatus->motion.traj.position.tran.z -
                               emcStatus->task.g5x_offset.tran.z -
                               emcStatus->task.g92_offset.tran.z -
                               emcStatus->task.toolOffset.tran.z);
        break;
    case 'a': case 'A':
        d = convertAngularUnits(emcStatus->motion.traj.position.a -
                                emcStatus->task.g5x_offset.a -
                                emcStatus->task.g92_offset.a -
                                emcStatus->task.toolOffset.a);
        break;
    case 'b': case 'B':
        d = convertAngularUnits(emcStatus->motion.traj.position.b -
                                emcStatus->task.g5x_offset.b -
                                emcStatus->task.g92_offset.b -
                                emcStatus->task.toolOffset.b);
        break;
    case 'c': case 'C':
        d = convertAngularUnits(emcStatus->motion.traj.position.c -
                                emcStatus->task.g5x_offset.c -
                                emcStatus->task.g92_offset.c -
                                emcStatus->task.toolOffset.c);
        break;
    case 'u': case 'U':
        d = convertLinearUnits(emcStatus->motion.traj.position.u -
                               emcStatus->task.g5x_offset.u -
                               emcStatus->task.g92_offset.u -
                               emcStatus->task.toolOffset.u);
        break;
    case 'v': case 'V':
        d = convertLinearUnits(emcStatus->motion.traj.position.v -
                               emcStatus->task.g5x_offset.v -
                               emcStatus->task.g92_offset.v -
                               emcStatus->task.toolOffset.v);
        break;
    case 'w': case 'W':
        d = convertLinearUnits(emcStatus->motion.traj.position.w -
                               emcStatus->task.g5x_offset.w -
                               emcStatus->task.g92_offset.w -
                               emcStatus->task.toolOffset.w);
        break;
    default:
        setresult(interp,"emc_rel_cmd_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }
    posobj = Tcl_NewDoubleObj(d);
    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int emc_rel_act_pos(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_rel_act_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    double d = 0.0;
    switch (string[0]) {
    case 'x': case 'X':
        d = convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.x -
                               emcStatus->task.g5x_offset.tran.x -
                               emcStatus->task.g92_offset.tran.x -
                               emcStatus->task.toolOffset.tran.x);
        break;
    case 'y': case 'Y':
        d = convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.y -
                               emcStatus->task.g5x_offset.tran.y -
                               emcStatus->task.g92_offset.tran.y -
                               emcStatus->task.toolOffset.tran.y);
        break;
    case 'z': case 'Z':
        d = convertLinearUnits(emcStatus->motion.traj.actualPosition.tran.z -
                               emcStatus->task.g5x_offset.tran.z -
                               emcStatus->task.g92_offset.tran.z -
                               emcStatus->task.toolOffset.tran.z);
        break;
    case 'a': case 'A':
        d = convertAngularUnits(emcStatus->motion.traj.actualPosition.a -
                                emcStatus->task.g5x_offset.a -
                                emcStatus->task.g92_offset.a -
                                emcStatus->task.toolOffset.a);
        break;
    case 'b': case 'B':
        d = convertAngularUnits(emcStatus->motion.traj.actualPosition.b -
                                emcStatus->task.g5x_offset.b -
                                emcStatus->task.g92_offset.b -
                                emcStatus->task.toolOffset.b);
        break;
    case 'c': case 'C':
        d = convertAngularUnits(emcStatus->motion.traj.actualPosition.c -
                                emcStatus->task.g5x_offset.c -
                                emcStatus->task.g92_offset.c -
                                emcStatus->task.toolOffset.c);
        break;
    case 'u': case 'U':
        d = convertLinearUnits(emcStatus->motion.traj.actualPosition.u -
                               emcStatus->task.g5x_offset.u -
                               emcStatus->task.g92_offset.u -
                               emcStatus->task.toolOffset.u);
        break;
    case 'v': case 'V':
        d = convertLinearUnits(emcStatus->motion.traj.actualPosition.v -
                               emcStatus->task.g5x_offset.v -
                               emcStatus->task.g92_offset.v -
                               emcStatus->task.toolOffset.v);
        break;
    case 'w': case 'W':
        d = convertLinearUnits(emcStatus->motion.traj.actualPosition.w -
                               emcStatus->task.g5x_offset.w -
                               emcStatus->task.g92_offset.w -
                               emcStatus->task.toolOffset.w);
        break;
    default:
        setresult(interp,"emc_rel_act_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }

    posobj = Tcl_NewDoubleObj(d);
    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int emc_joint_pos(ClientData /*clientdata*/,
			 Tcl_Interp * interp, int objc,
			 Tcl_Obj * CONST objv[])
{
    int joint;
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_joint_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	posobj = Tcl_NewDoubleObj(emcStatus->motion.joint[joint].input);
    } else {
	setresult(interp,"emc_joint_pos: bad integer argument");
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int emc_pos_offset(ClientData /*clientdata*/,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_pos_offset: need exactly 1 coordinate letter");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.g5x_offset.tran.x
                                                    +emcStatus->task.g92_offset.tran.x));
        break;
    case 'y': case 'Y':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.g5x_offset.tran.y
                                                    +emcStatus->task.g92_offset.tran.y));
        break;
    case 'z': case 'Z':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.g5x_offset.tran.z
                                                    +emcStatus->task.g92_offset.tran.z));
        break;
    case 'a': case 'A':
	posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.g5x_offset.a
                                                    +emcStatus->task.g92_offset.a));
        break;
    case 'b': case 'B':
	posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.g5x_offset.b
                                                    +emcStatus->task.g92_offset.b));
        break;
    case 'c': case 'C':
	posobj = Tcl_NewDoubleObj(convertAngularUnits(emcStatus->task.g5x_offset.c
                                                    +emcStatus->task.g92_offset.c));
        break;
    case 'u': case 'U':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.g5x_offset.u
                                                    +emcStatus->task.g92_offset.u));
        break;
    case 'v': case 'V':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.g5x_offset.v
                                                    +emcStatus->task.g92_offset.v));
        break;
    case 'w': case 'W':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(emcStatus->task.g5x_offset.w
                                                    +emcStatus->task.g92_offset.w));
        break;
    default:
	setresult(interp,"emc_pos_offset: bad coordinate letter argument");
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int emc_joint_limit(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_joint_limit: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	    setresult(interp,"emc_joint_limit: joint out of bounds");
	    return TCL_ERROR;
	}

	if (emcStatus->motion.joint[joint].minHardLimit) {
	    setresult(interp,"minhard");
	    return TCL_OK;
	} else if (emcStatus->motion.joint[joint].minSoftLimit) {
	    setresult(interp,"minsoft");
	    return TCL_OK;
	} else if (emcStatus->motion.joint[joint].maxSoftLimit) {
	    setresult(interp,"maxsoft");
	    return TCL_OK;
	} else if (emcStatus->motion.joint[joint].maxHardLimit) {
	    setresult(interp,"maxsoft");
	    return TCL_OK;
	} else {
	    setresult(interp,"ok");
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_joint_limit: joint out of bounds");
    return TCL_ERROR;
}

static int emc_joint_fault(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_joint_fault: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	    setresult(interp,"emc_joint_fault: joint out of bounds");
	    return TCL_ERROR;
	}

	if (emcStatus->motion.joint[joint].fault) {
	    setresult(interp,"fault");
	    return TCL_OK;
	} else {
	    setresult(interp,"ok");
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_joint_fault: joint out of bounds");
    return TCL_ERROR;
}

static int emc_override_limit(ClientData /*clientdata*/,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    Tcl_Obj *obj;
    int on;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	// motion overrides all axes at same time, so just reference index 0
	obj = Tcl_NewIntObj(emcStatus->motion.joint[0].overrideLimits);
	Tcl_SetObjResult(interp, obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &on)) {
	    if (on) {
		if (0 != sendOverrideLimits(0)) {
		    setresult(interp,"emc_override_limit: can't send command");
		    return TCL_OK;
		}
	    } else {
		if (0 != sendOverrideLimits(-1)) {
		    setresult(interp,"emc_override_limit: can't send command");
		    return TCL_OK;
		}
	    }
	    return TCL_OK;
	} else {
	    setresult(interp,"emc_override_limit: need 0 or 1");
	    return TCL_ERROR;
	}
    }

    setresult(interp,"emc_override_limit: need no args, 0 or 1");
    return TCL_ERROR;
}

static int emc_joint_homed(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_joint_homed: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	    setresult(interp,"emc_joint_homed: joint out of bounds");
	    return TCL_ERROR;
	}

	if (emcStatus->motion.joint[joint].homed) {
	    setresult(interp,"homed");
	    return TCL_OK;
	} else {
	    setresult(interp,"not");
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_joint_homed: joint out of bounds");
    return TCL_ERROR;
}

static int emc_mdi(ClientData /*clientdata*/,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char string[256];
    int t;

    CHECKEMC
    if (objc < 2) {
	setresult(interp,"emc_mdi: need command");
	return TCL_ERROR;
    }
    // bug-- check for string overflow
    rtapi_strxcpy(string, Tcl_GetStringFromObj(objv[1], 0));
    for (t = 2; t < objc; t++) {
	rtapi_strxcat(string, " ");
	rtapi_strxcat(string, Tcl_GetStringFromObj(objv[t], 0));
    }

    if (0 != sendMdiCmd(string)) {
	setresult(interp,"emc_mdi: error executing command");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_home(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_home: need joint");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	sendHome(joint);
	return TCL_OK;
    }

    setresult(interp,"emc_home: need joint as integer, 0..");
    return TCL_ERROR;
}

static int emc_unhome(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_unhome: need joint");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	sendUnHome(joint);
	return TCL_OK;
    }

    setresult(interp,"emc_unhome: need joint as integer, 0..");
    return TCL_ERROR;
}

static int emc_jog_stop(ClientData /*clientdata*/,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{
    int joint;
    int jjogmode;

    CHECKEMC
    if (objc != 3) {
	setresult(interp,"emc_jog_stop: need joint,jogmode");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint)) {
	setresult(interp,"emc_jog_stop: need joint as integer, 0|1");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[2], &jjogmode)) {
	setresult(interp,"emc_jog_stop: need jogmode as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != sendJogStop(joint, jjogmode)) {
	setresult(interp,"emc_jog_stop: can't send jog stop msg");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_jog(ClientData /*clientdata*/,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int joint;
    int jjogmode;
    double speed;

    CHECKEMC
    if (objc != 4) {
	setresult(interp,"emc_jog: need joint,jjogmode and speed");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint)) {
	setresult(interp,"emc_jog: need joint as integer, 0|1");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[2], &jjogmode)) {
	setresult(interp,"emc_jog: need jogmode as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &speed)) {
	setresult(interp,"emc_jog: need speed as real number");
	return TCL_ERROR;
    }

    if (0 != sendJogCont(joint, jjogmode, speed)) {
	setresult(interp,"emc_jog: can't jog");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_jog_incr(ClientData /*clientdata*/,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{
    int joint;
    int jjogmode;
    double speed;
    double incr;

    CHECKEMC
    if (objc != 5) {
	setresult(interp,"emc_jog_incr: need jjogmode,joint, speed, and increment");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint)) {
	setresult(interp,"emc_jog_incr: need joint as integer, 0|1");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[2], &jjogmode)) {
	setresult(interp,"emc_jog_incr: need jogmode as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &speed)) {
	setresult(interp,"emc_jog_incr: need speed as real number");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[4], &incr)) {
	setresult(interp,"emc_jog_incr: need increment as real number");
	return TCL_ERROR;
    }
    if (0 != sendJogIncr(joint, jjogmode, speed, incr)) {
	setresult(interp,"emc_jog_incr: can't jog");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_feed_override(ClientData /*clientdata*/,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *feedobj;
    int percent;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	feedobj =
	    Tcl_NewIntObj((int)
			  (emcStatus->motion.traj.scale * 100.0 + 0.5));
	Tcl_SetObjResult(interp, feedobj);
	return TCL_OK;
    }

    if (objc != 2) {
	setresult(interp,"emc_feed_override: need percent");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &percent)) {
	sendFeedOverride(((double) percent) / 100.0);
	return TCL_OK;
    }

    setresult(interp,"emc_feed_override: need percent");
    return TCL_ERROR;
}

static int emc_rapid_override(ClientData /*clientdata*/,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *rapidobj;
    int percent;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	rapidobj =
	    Tcl_NewIntObj((int)
			  (emcStatus->motion.traj.rapid_scale * 100.0 + 0.5));
	Tcl_SetObjResult(interp, rapidobj);
	return TCL_OK;
    }

    if (objc != 2) {
	setresult(interp,"emc_rapid_override: need percent");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &percent)) {
	sendRapidOverride(((double) percent) / 100.0);
	return TCL_OK;
    }

    setresult(interp,"emc_rapid_override: need percent");
    return TCL_ERROR;
}

static int emc_spindle_override(ClientData /*clientdata*/,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *feedobj;
    int spindle = 0;
    int percent;

    CHECKEMC
    if (objc == 1) {
		// no arg-- return status
		if (emcUpdateType == EMC_UPDATE_AUTO) {
			updateStatus();
		}
		feedobj = Tcl_NewIntObj((int)(emcStatus->motion.spindle[spindle].spindle_scale * 100.0 + 0.5));
		Tcl_SetObjResult(interp, feedobj);
		return TCL_OK;
    }

    if (objc == 2){
		if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &percent)) {
		sendSpindleOverride(spindle, ((double) percent) / 100.0);
		return TCL_OK;
		}
    }

    if (objc == 3){ // spindle number included
		if (TCL_OK != Tcl_GetIntFromObj(0, objv[1], &spindle)) {
		    setresult(interp,"emc_spindle_override: malformed spindle number");
		    return TCL_ERROR;
		}
		if (TCL_OK != Tcl_GetIntFromObj(0, objv[2], &percent)) {
		    setresult(interp,"emc_spindle_override: need percent");
		    return TCL_ERROR;
		}
		sendSpindleOverride(spindle, ((double) percent) / 100.0);
		return TCL_OK;
    }

    setresult(interp,"emc_spindle_override: need percent");
    return TCL_ERROR;
}

static int emc_task_plan_init(ClientData /*clientdata*/,
			      Tcl_Interp * interp, int /*objc*/,
			      Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (0 != sendTaskPlanInit()) {
	setresult(interp,"emc_task_plan_init: can't init interpreter");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_open(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_open: need file");
	return TCL_ERROR;
    }

    if (0 != sendProgramOpen(Tcl_GetStringFromObj(objv[1], 0))) {
	setresult(interp,"emc_open: can't open file");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_run(ClientData /*clientdata*/,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int line;

    CHECKEMC
    if (objc == 1) {
	if (0 != sendProgramRun(0)) {
	    setresult(interp,"emc_run: can't execute program");
	    return TCL_OK;
	}
    }

    if (objc == 2) {
	if (0 != Tcl_GetIntFromObj(0, objv[1], &line)) {
	    setresult(interp,"emc_run: need integer start line");
	    return TCL_ERROR;
	}
	if (0 != sendProgramRun(line)) {
	    setresult(interp,"emc_run: can't execute program");
	    return TCL_OK;
	}
    }

    return TCL_OK;
}

static int emc_pause(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int /*objc*/, Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (0 != sendProgramPause()) {
	setresult(interp,"emc_pause: can't pause program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_optional_stop(ClientData /*clientdata*/,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    Tcl_Obj *obj;
    int on;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return status
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	// get the current state from the status
	obj = Tcl_NewIntObj(emcStatus->task.optional_stop_state);
	Tcl_SetObjResult(interp, obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &on)) {
	    if (0 != sendSetOptionalStop(on)) {
		    setresult(interp,"emc_optional_stop: can't send command");
		    return TCL_OK;
	    }
	    return TCL_OK;
	} else {
	    setresult(interp,"emc_optional_stop: need 0 or 1");
	    return TCL_ERROR;
	}
    }

    setresult(interp,"emc_optional_stop: need no args, 0 or 1");
    return TCL_ERROR;
}

static int emc_resume(ClientData /*clientdata*/,
		      Tcl_Interp * interp, int /*objc*/,
		      Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (0 != sendProgramResume()) {
	setresult(interp,"emc_resume: can't resume program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_step(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int /*objc*/, Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (0 != sendProgramStep()) {
	setresult(interp,"emc_step: can't step program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_abort(ClientData /*clientdata*/,
		     Tcl_Interp * interp, int /*objc*/, Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (0 != sendAbort()) {
	setresult(interp,"emc_abort: can't execute program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int emc_program(ClientData /*clientdata*/,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_program: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (0 != emcStatus->task.file[0]) {
	setresult(interp,emcStatus->task.file);
	return TCL_OK;
    }

    setresult(interp,"none");
    return TCL_OK;
}

static int emc_program_status(ClientData /*clientdata*/,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_program_status: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    switch (emcStatus->task.interpState) {
    case EMC_TASK_INTERP::READING:
    case EMC_TASK_INTERP::WAITING:
	setresult(interp,"running");
	return TCL_OK;
	break;

    case EMC_TASK_INTERP::PAUSED:
	setresult(interp,"paused");
	return TCL_OK;
	break;

    default:
	setresult(interp,"idle");
	return TCL_OK;
    }

    setresult(interp,"idle");
    return TCL_OK;
}

static int emc_program_line(ClientData /*clientdata*/,
			    Tcl_Interp * interp, int objc,
			    Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *lineobj;
    int programActiveLine = 0;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_program_line: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (programStartLine < 0
	|| emcStatus->task.readLine < programStartLine) {
	// controller is skipping lines
	programActiveLine = emcStatus->task.readLine;
    } else {			// controller is not skipping lines
	if (emcStatus->task.currentLine > 0) {
	    if (emcStatus->task.motionLine > 0 &&
		emcStatus->task.motionLine < emcStatus->task.currentLine) {
		// active line is the motion line, which lags
		programActiveLine = emcStatus->task.motionLine;
	    } else {
		// active line is the current line-- no motion lag
		programActiveLine = emcStatus->task.currentLine;
	    }
	} else {
	    // no active line at all
	    programActiveLine = 0;
	}
    }				// end of else controller is not skipping
    // lines

    lineobj = Tcl_NewIntObj(programActiveLine);

    Tcl_SetObjResult(interp, lineobj);
    return TCL_OK;
}

static int emc_program_codes(ClientData /*clientdata*/,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST * /*objv*/)
{
    char codes_string[256];
    char string[256];
    int t;
    int code;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_program_codes: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }
    // fill in the active G-codes
    codes_string[0] = 0;
    for (t = 1; t < ACTIVE_G_CODES; t++) {
	code = emcStatus->task.activeGCodes[t];
	if (code == -1) {
	    continue;
	}
	if (code % 10) {
	    snprintf(string, sizeof(string), "G%.1f ", (double) code / 10.0);
	} else {
	    snprintf(string, sizeof(string), "G%d ", code / 10);
	}
	rtapi_strxcat(codes_string, string);
    }

    // fill in the active M codes, settings too
    for (t = 1; t < ACTIVE_M_CODES; t++) {
	code = emcStatus->task.activeMCodes[t];
	if (code == -1) {
	    continue;
	}
	snprintf(string, sizeof(string), "M%d ", code);
	rtapi_strxcat(codes_string, string);
    }

    // fill in F and S codes also
    snprintf(string, sizeof(string), "F%.0f ", emcStatus->task.activeSettings[1]);
    rtapi_strxcat(codes_string, string);
    snprintf(string, sizeof(string), "S%.0f", fabs(emcStatus->task.activeSettings[2]));
    rtapi_strxcat(codes_string, string);

    setresult(interp,codes_string);
    return TCL_OK;
}

static int emc_joint_type(ClientData /*clientdata*/,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_joint_type: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	    setresult(interp,"emc_joint_type: joint out of bounds");
	    return TCL_ERROR;
	}

	switch (emcStatus->motion.joint[joint].jointType) {
	case EMC_LINEAR:
	    setresult(interp,"linear");
	    break;
	case EMC_ANGULAR:
	    setresult(interp,"angular");
	    break;
	default:
	    setresult(interp,"custom");
	    break;
	}

	return TCL_OK;
    }

    setresult(interp,"emc_joint_type: invalid joint number");
    return TCL_ERROR;
}

static int emc_joint_units(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_joint_units: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	    setresult(interp,"emc_joint_units: joint out of bounds");
	    return TCL_ERROR;
	}

	switch (emcStatus->motion.joint[joint].jointType) {
	case EMC_LINEAR:
	    /* try mm */
	    if (CLOSE(emcStatus->motion.joint[joint].units, 1.0,
		      LINEAR_CLOSENESS)) {
		setresult(interp,"mm");
		return TCL_OK;
	    }
	    /* now try inch */
	    else if (CLOSE
		     (emcStatus->motion.joint[joint].units, INCH_PER_MM,
		      LINEAR_CLOSENESS)) {
		setresult(interp,"inch");
		return TCL_OK;
	    }
	    /* now try cm */
	    else if (CLOSE(emcStatus->motion.joint[joint].units, CM_PER_MM,
			   LINEAR_CLOSENESS)) {
		setresult(interp,"cm");
		return TCL_OK;
	    }
	    /* else it's custom */
	    setresult(interp,"custom");
	    return TCL_OK;
	    break;

	case EMC_ANGULAR:
	    /* try degrees */
	    if (CLOSE(emcStatus->motion.joint[joint].units, 1.0,
		      ANGULAR_CLOSENESS)) {
		setresult(interp,"deg");
		return TCL_OK;
	    }
	    /* now try radians */
	    else if (CLOSE
		     (emcStatus->motion.joint[joint].units, RAD_PER_DEG,
		      ANGULAR_CLOSENESS)) {
		setresult(interp,"rad");
		return TCL_OK;
	    }
	    /* now try grads */
	    else if (CLOSE
		     (emcStatus->motion.joint[joint].units, GRAD_PER_DEG,
		      ANGULAR_CLOSENESS)) {
		setresult(interp,"grad");
		return TCL_OK;
	    }
	    /* else it's custom */
	    setresult(interp,"custom");
	    return TCL_OK;
	    break;

	default:
	    setresult(interp,"custom");
	    return TCL_OK;
	    break;
	}
    }

    setresult(interp,"emc_joint_units: invalid joint number");
    return TCL_ERROR;
}

static int emc_program_linear_units(ClientData /*clientdata*/,
				    Tcl_Interp * interp, int objc,
				    Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_program_linear_units: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    switch (emcStatus->task.programUnits) {
    case CANON_UNITS_INCHES:
	setresult(interp,"inch");
	return TCL_OK;
	break;

    case CANON_UNITS_MM:
	setresult(interp,"mm");
	return TCL_OK;
	break;

    case CANON_UNITS_CM:
	setresult(interp,"cm");
	return TCL_OK;
	break;

    default:
	setresult(interp,"custom");
	return TCL_OK;
	break;
    }

    setresult(interp,"custom");
    return TCL_OK;
}

static int emc_program_angular_units(ClientData /*clientdata*/,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_program_angular_units: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }
    // currently the EMC doesn't have separate program angular units, so
    // these are simply "deg"
    setresult(interp,"deg");
    return TCL_OK;
}

static int emc_user_linear_units(ClientData /*clientdata*/,
				 Tcl_Interp * interp, int objc,
				 Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_user_linear_units: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    /* try mm */
    if (CLOSE(emcStatus->motion.traj.linearUnits, 1.0, LINEAR_CLOSENESS)) {
	setresult(interp,"mm");
	return TCL_OK;
    }
    /* now try inch */
    else if (CLOSE(emcStatus->motion.traj.linearUnits, INCH_PER_MM,
		   LINEAR_CLOSENESS)) {
	setresult(interp,"inch");
	return TCL_OK;
    }
    /* now try cm */
    else if (CLOSE(emcStatus->motion.traj.linearUnits, CM_PER_MM,
		   LINEAR_CLOSENESS)) {
	setresult(interp,"cm");
	return TCL_OK;
    }

    /* else it's custom */
    setresult(interp,"custom");
    return TCL_OK;
}

static int emc_user_angular_units(ClientData /*clientdata*/,
				  Tcl_Interp * interp, int objc,
				  Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_user_angular_units: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    /* try degrees */
    if (CLOSE(emcStatus->motion.traj.angularUnits, 1.0, ANGULAR_CLOSENESS)) {
	setresult(interp,"deg");
	return TCL_OK;
    }
    /* now try radians */
    else if (CLOSE(emcStatus->motion.traj.angularUnits, RAD_PER_DEG,
		   ANGULAR_CLOSENESS)) {
	setresult(interp,"rad");
	return TCL_OK;
    }
    /* now try grads */
    else if (CLOSE(emcStatus->motion.traj.angularUnits, GRAD_PER_DEG,
		   ANGULAR_CLOSENESS)) {
	setresult(interp,"grad");
	return TCL_OK;
    }

    /* else it's an arbitrary number, so just return it */
    setresult(interp,"custom");
    return TCL_OK;
}

static int emc_display_linear_units(ClientData /*clientdata*/,
				    Tcl_Interp * interp, int objc,
				    Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_display_linear_units: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    switch (linearUnitConversion) {
    case LINEAR_UNITS_INCH:
	setresult(interp,"inch");
	break;
    case LINEAR_UNITS_MM:
	setresult(interp,"mm");
	break;
    case LINEAR_UNITS_CM:
	setresult(interp,"cm");
	break;
    case LINEAR_UNITS_AUTO:
	switch (emcStatus->task.programUnits) {
	case CANON_UNITS_MM:
	    setresult(interp,"(mm)");
	    break;
	case CANON_UNITS_INCHES:
	    setresult(interp,"(inch)");
	    break;
	case CANON_UNITS_CM:
	    setresult(interp,"(cm)");
	    break;
	}
	break;
    default:
	setresult(interp,"custom");
	break;
    }

    return TCL_OK;
}

static int emc_display_angular_units(ClientData /*clientdata*/,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST * /*objv*/)
{
    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_display_angular_units: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    switch (angularUnitConversion) {
    case ANGULAR_UNITS_DEG:
	setresult(interp,"deg");
	break;
    case ANGULAR_UNITS_RAD:
	setresult(interp,"rad");
	break;
    case ANGULAR_UNITS_GRAD:
	setresult(interp,"grad");
	break;
    case ANGULAR_UNITS_AUTO:
	setresult(interp,"(deg)");	/*! \todo FIXME-- always deg? */
	break;
    default:
	setresult(interp,"custom");
	break;
    }

    return TCL_OK;
}

static int emc_linear_unit_conversion(ClientData /*clientdata*/,
				      Tcl_Interp * interp, int objc,
				      Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return unit setting
	switch (linearUnitConversion) {
	case LINEAR_UNITS_INCH:
	    setresult(interp,"inch");
	    break;
	case LINEAR_UNITS_MM:
	    setresult(interp,"mm");
	    break;
	case LINEAR_UNITS_CM:
	    setresult(interp,"cm");
	    break;
	case LINEAR_UNITS_AUTO:
	    setresult(interp,"auto");
	    break;
	default:
	    setresult(interp,"custom");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "inch")) {
	    linearUnitConversion = LINEAR_UNITS_INCH;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "mm")) {
	    linearUnitConversion = LINEAR_UNITS_MM;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "cm")) {
	    linearUnitConversion = LINEAR_UNITS_CM;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    linearUnitConversion = LINEAR_UNITS_AUTO;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "custom")) {
	    linearUnitConversion = LINEAR_UNITS_CUSTOM;
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_linear_unit_conversion: need 'inch', 'mm', 'cm', 'auto', 'custom', or no args");
    return TCL_ERROR;
}

static int emc_angular_unit_conversion(ClientData /*clientdata*/,
				       Tcl_Interp * interp, int objc,
				       Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKEMC
    if (objc == 1) {
	// no arg-- return unit setting
	switch (angularUnitConversion) {
	case ANGULAR_UNITS_DEG:
	    setresult(interp,"deg");
	    break;
	case ANGULAR_UNITS_RAD:
	    setresult(interp,"rad");
	    break;
	case ANGULAR_UNITS_GRAD:
	    setresult(interp,"grad");
	    break;
	case ANGULAR_UNITS_AUTO:
	    setresult(interp,"auto");
	    break;
	default:
	    setresult(interp,"custom");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "deg")) {
	    angularUnitConversion = ANGULAR_UNITS_DEG;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "rad")) {
	    angularUnitConversion = ANGULAR_UNITS_RAD;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "grad")) {
	    angularUnitConversion = ANGULAR_UNITS_GRAD;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    angularUnitConversion = ANGULAR_UNITS_AUTO;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "custom")) {
	    angularUnitConversion = ANGULAR_UNITS_CUSTOM;
	    return TCL_OK;
	}
    }

    setresult(interp,"emc_angular_unit_conversion: need 'deg', 'rad', 'grad', 'auto', 'custom', or no args");
    return TCL_ERROR;
}

static int emc_task_command(ClientData /*clientdata*/,
			    Tcl_Interp * interp, int objc,
			    Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandobj;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_task_command: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandobj = Tcl_NewIntObj(emcStatus->task.command_type);

    Tcl_SetObjResult(interp, commandobj);
    return TCL_OK;
}

static int emc_task_command_number(ClientData /*clientdata*/,
				   Tcl_Interp * interp, int objc,
				   Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandnumber;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_task_command_number: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandnumber = Tcl_NewIntObj(emcStatus->task.echo_serial_number);

    Tcl_SetObjResult(interp, commandnumber);
    return TCL_OK;
}

static int emc_task_command_status(ClientData /*clientdata*/,
				   Tcl_Interp * interp, int objc,
				   Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandstatus;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_task_command_status: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandstatus = Tcl_NewIntObj((int)emcStatus->task.status);

    Tcl_SetObjResult(interp, commandstatus);
    return TCL_OK;
}

static int emc_io_command(ClientData /*clientdata*/,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandobj;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_io_command: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandobj = Tcl_NewIntObj(emcStatus->io.command_type);

    Tcl_SetObjResult(interp, commandobj);
    return TCL_OK;
}

static int emc_io_command_number(ClientData /*clientdata*/,
				 Tcl_Interp * interp, int objc,
				 Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandnumber;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_io_command_number: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandnumber = Tcl_NewIntObj(emcStatus->io.echo_serial_number);

    Tcl_SetObjResult(interp, commandnumber);
    return TCL_OK;
}

static int emc_io_command_status(ClientData /*clientdata*/,
				 Tcl_Interp * interp, int objc,
				 Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandstatus;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_io_command_status: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandstatus = Tcl_NewIntObj((int)emcStatus->io.status);

    Tcl_SetObjResult(interp, commandstatus);
    return TCL_OK;
}

static int emc_motion_command(ClientData /*clientdata*/,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandobj;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_motion_command: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandobj = Tcl_NewIntObj(emcStatus->motion.command_type);

    Tcl_SetObjResult(interp, commandobj);
    return TCL_OK;
}

static int emc_motion_command_number(ClientData /*clientdata*/,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandnumber;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_motion_command_number: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandnumber = Tcl_NewIntObj(emcStatus->motion.echo_serial_number);

    Tcl_SetObjResult(interp, commandnumber);
    return TCL_OK;
}

static int emc_motion_command_status(ClientData /*clientdata*/,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST * /*objv*/)
{
    Tcl_Obj *commandstatus;

    CHECKEMC
    if (objc != 1) {
	setresult(interp,"emc_motion_command_status: need no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    commandstatus = Tcl_NewIntObj((int)emcStatus->motion.status);

    Tcl_SetObjResult(interp, commandstatus);
    return TCL_OK;
}

static int emc_joint_backlash(ClientData /*clientdata*/,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *valobj;
    int joint;
    double backlash;

    // syntax is emc_joint_backlash <joint> {<backlash>}
    // if <backlash> is not specified, returns current value,
    // otherwise, sets backlash to specified value

    CHECKEMC
    // check number of args supplied
    if ((objc < 2) || (objc > 3)) {
	setresult(interp,"emc_joint_backlash: need <joint> {<backlash>}");
	return TCL_ERROR;
    }
    // get joint number
    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint) ||
	joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	setresult(interp,"emc_joint_backlash: need joint as integer, 0..EMCMOT_MAX_JOINTS-1");
	return TCL_ERROR;
    }
    // test for get or set
    if (objc == 2) {
	// want to get present value
	valobj = Tcl_NewDoubleObj(emcStatus->motion.joint[joint].backlash);
	Tcl_SetObjResult(interp, valobj);
	return TCL_OK;
    } else {
	// want to set new value
	if (0 != Tcl_GetDoubleFromObj(0, objv[2], &backlash)) {
	    setresult(interp,"emc_joint_backlash: need backlash as real number");
	    return TCL_ERROR;
	}
	// write it out
	sendJointSetBacklash(joint, backlash);
	return TCL_OK;
    }
}

static int emc_joint_enable(ClientData /*clientdata*/,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;
    int val;
    Tcl_Obj *enobj;

    // syntax is emc_joint_output <joint> {0 | 1}

    CHECKEMC
    if (objc < 2) {
	setresult(interp,"emc_joint_enable: need <joint>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint) ||
	joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	setresult(interp,"emc_joint_enable: need joint as integer, 0..EMCMOT_MAX_JOINTS-1");
	return TCL_ERROR;
    }

    if (objc == 2) {
	if (emcUpdateType == EMC_UPDATE_AUTO) {
	    updateStatus();
	}
	enobj = Tcl_NewIntObj(emcStatus->motion.joint[joint].enabled);
	Tcl_SetObjResult(interp, enobj);
	return TCL_OK;
    }
    // else we were given 0 or 1 to enable/disable it
    if (0 != Tcl_GetIntFromObj(0, objv[2], &val)) {
	setresult(interp,"emc_joint_enable: need 0, 1 for disable, enable");
	return TCL_ERROR;
    }
    return TCL_OK;
}

static int emc_joint_load_comp(ClientData /*clientdata*/,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    int joint, type;
    char file[256];

    CHECKEMC
    // syntax is emc_joint_load_comp <joint> <file>

    if (objc != 4) {
	setresult(interp,"emc_joint_load_comp: need <joint> <file> <type>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint) ||
	joint < 0 || joint >= EMCMOT_MAX_JOINTS) {
	setresult(interp,"emc_joint_load_comp: need joint as integer, 0..EMCMOT_MAX_JOINTS-1");
	return TCL_ERROR;
    }
    // copy objv[1] to file arg, to make sure it's not modified
    rtapi_strxcpy(file, Tcl_GetStringFromObj(objv[2], 0));

    if (0 != Tcl_GetIntFromObj(0, objv[3], &type)) {
	setresult(interp,"emc_joint_load_comp: <type> must be an int");
    }

    // now write it out
    sendJointLoadComp(joint, file, type);
    return TCL_OK;
}

int emc_teleop_enable(ClientData /*clientdata*/,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    int enable;

    if (objc != 1) {
	if (0 != Tcl_GetIntFromObj(0, objv[1], &enable)) {
	    setresult(interp,"emc_teleop_enable: <enable> must be an integer");
	    return TCL_ERROR;
	}
	sendSetTeleopEnable(enable);
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(emcStatus->motion.traj.mode ==
				   EMC_TRAJ_MODE::TELEOP));
    return TCL_OK;
}

int emc_kinematics_type(ClientData /*clientdata*/,
			Tcl_Interp * interp, int /*objc*/,
			Tcl_Obj * CONST * /*objv*/)
{

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(emcStatus->motion.traj.
				   kinematics_type));
    return TCL_OK;
}

int emc_probe_clear(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST * /*objv*/)
{
    if (objc != 1) {
	setresult(interp,"emc_probe_clear: needs no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp, Tcl_NewIntObj(sendClearProbeTrippedFlag()));
    return TCL_OK;
}

int emc_probe_value(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST * /*objv*/)
{
    if (objc != 1) {
	setresult(interp,"emc_probe_value: needs no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(emcStatus->motion.traj.probeval));
    return TCL_OK;
}

int emc_probe_tripped(ClientData /*clientdata*/,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST * /*objv*/)
{
    if (objc != 1) {
	setresult(interp,"emc_probe_tripped: needs no args");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(emcStatus->motion.traj.probe_tripped));
    return TCL_OK;
}

int emc_probe_move(ClientData /*clientdata*/,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    double x, y, z;

    if (objc != 4) {
	setresult(interp,"emc_probe_move: <x> <y> <z>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetDoubleFromObj(0, objv[1], &x)) {
	setresult(interp,"emc_probe_move: <x> must be a double");
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[2], &y)) {
	setresult(interp,"emc_probe_move: <y> must be a double");
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &z)) {
	setresult(interp,"emc_probe_move: <z> must be a double");
    }

    Tcl_SetObjResult(interp, Tcl_NewIntObj(sendProbe(x, y, z)));
    return TCL_OK;
}

static int emc_probed_pos(ClientData /*clientdata*/,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKEMC
    if (objc != 2) {
	setresult(interp,"emc_probed_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (emcUpdateType == EMC_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.probedPosition.tran.x));
        break;
    case 'y': case 'Y':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.probedPosition.tran.y));
        break;
    case 'z': case 'Z':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.probedPosition.tran.z));
        break;
    case 'a': case 'A':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.probedPosition.a));
        break;
    case 'b': case 'B':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.probedPosition.b));
        break;
    case 'c': case 'C':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  emcStatus->motion.traj.probedPosition.c));
        break;
    case 'u': case 'U':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.probedPosition.u));
        break;
    case 'v': case 'V':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.probedPosition.v));
        break;
    case 'w': case 'W':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  emcStatus->motion.traj.probedPosition.w));
        break;
    default:
        setresult(interp,"emc_probed_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }
    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

// ********************************************************************
//      Pendant read routine from /dev/psaux, /dev/ttyS0, or /dev/ttyS1
// *********************************************************************

static int emc_pendant(ClientData /*clientdata*/,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    FILE *inFile;

    char inBytes[5];
    const char *port;

    inBytes[0] = 0;
    inBytes[1] = 0;
    inBytes[2] = 0;
    inBytes[3] = 0;
    inBytes[4] = 0;

    CHECKEMC
    if (objc == 2) {
	port = Tcl_GetStringFromObj(objv[1], 0);
	if ((!strcmp(port, "/dev/psaux")) | (!strcmp(port,
						     "/dev/ttyS0")) |
	    (!strcmp(port, "/dev/ttyS1"))) {
	    inFile = fopen(port, "r+b");

	    if (inFile) {
		if (strcmp(port, "/dev/psaux")) {	// For Serial mice
		    inBytes[1] = fgetc(inFile);	// read the first Byte
		    if (inBytes[1] != 77) {	// If first byte not "M"
			fputc(77, inFile);	// Request data resent
			fflush(inFile);
			inBytes[1] = fgetc(inFile);	// and hope it is
			// correct
		    }
		}
		inBytes[4] = fgetc(inFile);	// Status byte
		inBytes[2] = fgetc(inFile);	// Horizontal movement
		inBytes[3] = fgetc(inFile);	// Vertical Movement
	    }
	    fclose(inFile);

	    if (!strcmp(port, "/dev/psaux")) {	// For PS/2
		inBytes[0] = (inBytes[4] & 0x01);	// Left button
		inBytes[1] = (inBytes[4] & 0x02) >> 1;	// Right button
	    } else {		// For serial mice
		inBytes[0] = (inBytes[4] & 0x20) >> 5;	// Left button
		inBytes[1] = (inBytes[4] & 0x10) >> 4;	// Right button
		if (inBytes[4] & 0x02) {
		    inBytes[2] = inBytes[2] | 0xc0;
		}
		if (inBytes[4] & 0x08) {
		    inBytes[3] = inBytes[3] | 0xc0;
		}
	    }

	    char buf[80];
	    snprintf(buf, sizeof(buf), "%i %i %d %d %i", inBytes[0],
		    inBytes[1], inBytes[2], inBytes[3], inBytes[4]);
	    Tcl_SetResult(interp, buf, TCL_VOLATILE);
	    return TCL_OK;
	}
    }
    setresult(interp,"Need /dev/psaux, /dev/ttyS0 or /dev/ttyS1 as Arg");
    return TCL_ERROR;
}

// *******************************************************************

// provide some of the extended Tcl builtins not available for various plats
// "int", as in "int 3.9" which returns 3
static int localint(ClientData /*clientdata*/,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    double val;
    char resstring[80];

    if (objc != 2) {
	// need exactly one arg
	setresult(interp, "wrong # args: should be \"int value\"");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetDoubleFromObj(0, objv[1], &val)) {
	resstring[0] = 0;
	rtapi_strxcat(resstring, "expected number but got \"");
	strncat(resstring, Tcl_GetStringFromObj(objv[1], 0),
		sizeof(resstring) - strlen(resstring) - 2);
	rtapi_strxcat(resstring, "\"");
	setresult(interp, resstring);
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, Tcl_NewIntObj((int) val));
    return TCL_OK;
}

static const char *one_head(int x0, int y0, int x1, int y1)
{
    static char buf[100];
    snprintf(buf, sizeof(buf), "%d %d %d %d", x0, y0, x1, y1);
    return buf;
}

// "round", as in "round 3.9" which returns 4
static int localround(ClientData /*clientdata*/,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    double val;
    char resstring[80];

    if (objc != 2) {
	// need exactly one arg
	setresult(interp,"wrong # args: should be \"round value\"");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetDoubleFromObj(0, objv[1], &val)) {
	resstring[0] = 0;
	rtapi_strxcat(resstring, "expected number but got \"");
	strncat(resstring, Tcl_GetStringFromObj(objv[1], 0),
		sizeof(resstring) - strlen(resstring) - 2);
	rtapi_strxcat(resstring, "\"");
	setresult(interp,resstring);
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(val <
				   0.0 ? (int) (val - 0.5) : (int) (val +
								    0.5)));
    return TCL_OK;
}

#include <X11/extensions/Xinerama.h>

static int multihead(ClientData /*clientdata*/,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST * /*objv*/)
{
    if(objc > 1)
	setresult(interp,"wrong # args: should be \"multihead\"");

    Tk_Window tkwin = Tk_MainWindow(interp);
    if(!tkwin) return TCL_ERROR;

    Display *d = Tk_Display(tkwin);
    if(!d) return TCL_ERROR;

    Tcl_ResetResult(interp);

    XineramaScreenInfo *inf = NULL;
    int count = 0;

    int i, j;
    if(XineramaQueryExtension(d, &i, &j)) {
        inf = XineramaQueryScreens(d, &count);
    }

    if( !inf ) {
        Tcl_AppendElement(interp, one_head(0, 0,
                   DisplayWidth(d, DefaultScreen(d)),
                   DisplayHeight(d, DefaultScreen(d))));
    } else {
        for(i=0; i<count; i++) {
            Tcl_AppendElement(interp, one_head(inf[i].x_org, inf[i].y_org,
                        inf[i].x_org + inf[i].width,
                        inf[i].y_org + inf[i].height));
        }
        XFree(inf);
    }
    return TCL_OK;
}

static void sigQuit(int /*sig*/)
{
    thisQuit((ClientData) 0);
}

static void initMain()
{
    emcWaitType = EMC_WAIT_RECEIVED;
    emcCommandSerialNumber = 0;
    emcTimeout = 0.0;
    emcUpdateType = EMC_UPDATE_AUTO;
    linearUnitConversion = LINEAR_UNITS_AUTO;
    angularUnitConversion = ANGULAR_UNITS_AUTO;
    emcCommandBuffer = 0;
    emcStatusBuffer = 0;
    emcStatus = 0;

    emcErrorBuffer = 0;
    error_string[LINELEN-1] = 0;
    operator_text_string[LINELEN-1] = 0;
    operator_display_string[LINELEN-1] = 0;
    programStartLine = 0;
}

int emc_init(ClientData /*cd*/, Tcl_Interp *interp, int argc, const char **argv)
{
    bool quick = false;
    initMain();
    // process command line args
    // use -ini inifilename to set EMC_INIFILE
    // see emcargs.c for other arguments
    // use -quick to return quickly if emc is not running
    if (0 != emcGetArgs(argc, (char**)argv)) {
        setresult(interp,"error in argument list\n");
        return TCL_ERROR;
    }
    // get configuration information
    iniLoad(emc_inifile);

    for(int i=1; i<argc; i++)
    {
	if(!strcmp(argv[i], "-quick")) quick = true;
    }

    // update tcl's idea of the inifile name
    Tcl_SetVar(interp, "EMC_INIFILE", emc_inifile, TCL_GLOBAL_ONLY);

    // init NML
    if (0 != tryNml(quick ? 0.0 : 10.0, quick ? 0.0 : 1.0)) {
        setresult(interp,"no emc connection");
        thisQuit(NULL);
        return TCL_ERROR;
    }
    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();
    emcCommandSerialNumber = emcStatus->echo_serial_number;

    // attach our quit function to exit
    Tcl_CreateExitHandler(thisQuit, (ClientData) 0);

    // attach our quit function to SIGINT
    signal(SIGINT, sigQuit);

    setresult(interp,"");
    return TCL_OK;
}

extern "C"
int Linuxcnc_Init(Tcl_Interp * interp);
int Linuxcnc_Init(Tcl_Interp * interp)
{
    if (Tcl_InitStubs(interp, "8.1", 0) == NULL)
    {
        return TCL_ERROR;
    }

    /*
     * Call Tcl_CreateCommand for application-specific commands, if
     * they weren't already created by the init procedures called above.
     */

    Tcl_CreateCommand(interp, "emc_init", emc_init, (ClientData) NULL,
                         (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_plat", emc_plat, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_ini", emc_ini, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_debug", emc_Debug, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_set_wait", emc_set_wait,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_wait", emc_wait, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_set_timeout", emc_set_timeout,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_update", emc_update,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_time", emc_time, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_error", emc_error, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_operator_text", emc_operator_text,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_operator_display",
			 emc_operator_display, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_estop", emc_estop, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_machine", emc_machine,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_mode", emc_mode, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_mist", emc_mist, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_flood", emc_flood, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_spindle", emc_spindle,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_brake", emc_brake, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_tool", emc_tool, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_tool_offset", emc_tool_offset,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_load_tool_table",
			 emc_load_tool_table, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_set_tool_offset",
			 emc_set_tool_offset, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_abs_cmd_pos", emc_abs_cmd_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_abs_act_pos", emc_abs_act_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_rel_cmd_pos", emc_rel_cmd_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_rel_act_pos", emc_rel_act_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_pos", emc_joint_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_pos_offset", emc_pos_offset,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_limit", emc_joint_limit,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_fault", emc_joint_fault,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_override_limit", emc_override_limit,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_optional_stop", emc_optional_stop,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_homed", emc_joint_homed,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_mdi", emc_mdi, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_home", emc_home, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_unhome", emc_unhome, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_jog_stop", emc_jog_stop,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_jog", emc_jog, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_jog_incr", emc_jog_incr,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_feed_override", emc_feed_override,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_rapid_override", emc_rapid_override,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_spindle_override", emc_spindle_override,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_task_plan_init", emc_task_plan_init,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_open", emc_open, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_run", emc_run, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_pause", emc_pause, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_resume", emc_resume,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_step", emc_step, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_abort", emc_abort, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_program", emc_program,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_program_line", emc_program_line,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_program_status", emc_program_status,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_program_codes", emc_program_codes,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_type", emc_joint_type,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_units", emc_joint_units,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_program_linear_units",
			 emc_program_linear_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_program_angular_units",
			 emc_program_angular_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_user_linear_units",
			 emc_user_linear_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_user_angular_units",
			 emc_user_angular_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_display_linear_units",
			 emc_display_linear_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_display_angular_units",
			 emc_display_angular_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_linear_unit_conversion",
			 emc_linear_unit_conversion, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_angular_unit_conversion",
			 emc_angular_unit_conversion, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);


    Tcl_CreateObjCommand(interp, "emc_task_command", emc_task_command,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_task_command_number",
			 emc_task_command_number, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_task_command_status",
			 emc_task_command_status, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_io_command", emc_io_command,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_io_command_number",
			 emc_io_command_number, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_io_command_status",
			 emc_io_command_status, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);


    Tcl_CreateObjCommand(interp, "emc_motion_command", emc_motion_command,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_motion_command_number",
			 emc_motion_command_number, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_motion_command_status",
			 emc_motion_command_status, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_backlash", emc_joint_backlash,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_load_comp", emc_joint_load_comp,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_joint_enable", emc_joint_enable,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_teleop_enable", emc_teleop_enable,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_kinematics_type",
			 emc_kinematics_type, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_probe_clear", emc_probe_clear,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "emc_probe_value", emc_probe_value,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "emc_probe_tripped", emc_probe_tripped,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "emc_probe_move", emc_probe_move,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "emc_probed_pos", emc_probed_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "emc_pendant", emc_pendant,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    // provide builtins that may have been left out

    Tcl_CreateObjCommand(interp, "int", localint, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "round", localround, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "multihead", multihead, (ClientData) NULL,
                         (Tcl_CmdDeleteProc*) NULL);

    /*
     * Specify a user-specific startup file to invoke if the application
     * is run interactively.  Typically the startup file is "~/.apprc"
     * where "app" is the name of the application.  If this line is deleted
     * then no user-specific startup file will be run under any conditions.
     */

    Tcl_SetVar(interp, "tcl_rcFileName", "~/.emcshrc", TCL_GLOBAL_ONLY);

    // set app-specific global variables
    Tcl_SetVar(interp, "EMC_INIFILE", emc_inifile, TCL_GLOBAL_ONLY);
    Tcl_PkgProvide(interp, "Linuxcnc", "1.0");

    Tcl_ResetResult(interp);

    return TCL_OK;
}

