/*misnomer: _setup.current_pocket,selected_pocket
**          These are indexes to sequential tooldata entries
**          but the names are not changed due to frequent
**          legacy usage in py files used for remapping
*/
/********************************************************************
* Description: interp_namedparams.cc
*
* collect all code related to named parameter handling
*
* Author: mostly K. Lerman
* rewrite by Michael Haberler to use STL containers
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change: Juli 2011
********************************************************************/

#include "config.h"

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define BOOST_PYTHON_MAX_ARITY 4
#include "python_plugin.hh"
#include <boost/python/dict.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/tuple.hpp>
namespace bp = boost::python;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sstream>
#include <map>

#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "interp_internal.hh"
#include "rs274ngc_interp.hh"
#include "inifile.hh"

// for HAL pin variables
#include "hal.h"

enum predefined_named_parameters {
    NP_LINE,
    NP_MOTION_MODE,
    NP_PLANE,
    NP_CCOMP,
    NP_METRIC,
    NP_IMPERIAL,
    NP_ABSOLUTE,
    NP_INCREMENTAL,
    NP_INVERSE_TIME,
    NP_UNITS_PER_MINUTE,
    NP_UNITS_PER_REV,
    NP_COORD_SYSTEM,
    NP_TOOL_OFFSET,
    NP_RETRACT_R_PLANE,
    NP_RETRACT_OLD_Z,
    NP_SPINDLE_RPM_MODE,
    NP_SPINDLE_CSS_MODE,
    NP_IJK_ABSOLUTE_MODE,
    NP_LATHE_DIAMETER_MODE,
    NP_LATHE_RADIUS_MODE,
    NP_SPINDLE_ON,
    NP_SPINDLE_CW,
    NP_MIST,
    NP_FLOOD,
    NP_SPEED_OVERRIDE,
    NP_FEED_OVERRIDE,
    NP_ADAPTIVE_FEED,
    NP_FEED_HOLD,
    NP_FEED,
    NP_RPM,
    NP_CURRENT_TOOL,
    NP_SELECTED_POCKET,
    NP_CURRENT_POCKET,
    NP_X,
    NP_Y,
    NP_Z,
    NP_A,
    NP_B,
    NP_C,
    NP_U,
    NP_V,
    NP_W,
    NP_ABS_X,
    NP_ABS_Y,
    NP_ABS_Z,
    NP_ABS_A,
    NP_ABS_B,
    NP_ABS_C,
    NP_VALUE,
    NP_CALL_LEVEL,
    NP_REMAP_LEVEL,
    NP_SELECTED_TOOL,
    NP_VALUE_RETURNED,
    NP_TASK,
};

/****************************************************************************/

/*! read_named_parameter

Returned Value: int
   If read_integer_value returns an error code, this returns that code.
   If any of the following errors occur, this returns the error code shown.
   Otherwise, this returns INTERP_OK.
   1. The first character read is not a <:
      NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED
   2. The named parameter string is not terminated by >:
      NCE_NAMED_PARAMETER_NOT_TERSINATED
   3. The named parameter has not been defined before use:
      NCE_NAMED_PARAMETER_NOT_DEFINED

Side effects:
   The value of the given parameter is put into what double_ptr points at.
   The counter is reset to point to the first character after the
   characters which make up the value.

Called by:  read_parameter

This attempts to read the value of a parameter out of the line,
starting at the index given by the counter.

According to the RS274/NGC manual [NCMS, p. 62], the characters following
# may be any "parameter expression". Thus, the following are legal
and mean the same thing (the value of the parameter whose number is
stored in parameter 2):
  ##2
  #[#2]


ADDED by K. Lerman
Named parameters are now supported.
#<_abcd> is a parameter with name "abcd" of global scope
#<abce> is a named parameter of local scope.

*/

int Interp::read_named_parameter(
				 char *line,   //!< string: line of RS274/NGC code being processed
				 int *counter, //!< pointer to a counter for position on the line
				 double *double_ptr,   //!< pointer to double to be read
				 double * /*parameters*/,   //!< array of system parameters
				 bool check_exists)    //!< test for existence, not value
{
    static char name[] = "read_named_parameter";
    char paramNameBuf[LINELEN+1];
    int exists;
    double value;
    parameter_map_iterator pi;

    CHKS((line[*counter] != '<'),
	 NCE_BUG_FUNCTION_SHOULD_NOT_HAVE_BEEN_CALLED);
    CHP(read_name(line, counter, paramNameBuf));

    CHP(find_named_param(paramNameBuf, &exists, &value));
    if (check_exists) {
	*double_ptr = exists ? 1.0 : 0.0;
	return INTERP_OK;
    }
    if (exists) {
	*double_ptr = value;
	return INTERP_OK;
    } else {
        // do not require named parameters to be defined during a 
        // subroutine definition:
        if (_setup.defining_sub)
            return INTERP_OK;

	logNP("%s: referencing undefined named parameter '%s' level=%d",
	      name, paramNameBuf, (paramNameBuf[0] == '_') ? 0 : _setup.call_level);
	ERS(_("Named parameter #<%s> not defined"), paramNameBuf);
    }
    return INTERP_OK;
}

// if the variable is of the form '_ini[section]name', then treat it as
// an inifile  variable. Lookup section/name and cache the value
// as global and read-only.
// the shortest possible INI variable is '_ini[s]n' or 8 chars long .
int Interp::fetch_ini_param( const char *nameBuf, int *status, double *value)
{
    char *s;
    *status = 0;
    int n = strlen(nameBuf);

     if ((n > 7) &&
	((s = (char *) strchr(&nameBuf[6],']')) != NULL)) {

	IniFile inifile;
	const char *iniFileName;
	int retval;
	int closeBracket = s - nameBuf;

	if ((iniFileName = getenv("INI_FILE_NAME")) == NULL) {
	    logNP("warning: referencing INI parameter '%s': no INI file",nameBuf);
	    *status = 0;
	    return INTERP_OK;
	}
	if (!inifile.Open(iniFileName)) {
	    *status = 0;
	    ERS(_("can\'t open INI file '%s'"), iniFileName);
	}

	char capName[LINELEN];

	snprintf(capName, LINELEN, "%s", nameBuf);
	for (char *p = capName; *p != 0; p++)
	    *p = toupper(*p);
	capName[closeBracket] = '\0';

	if ((retval = inifile.Find( value, &capName[closeBracket+1], &capName[5])) == 0) {
	    *status = 1;
	    inifile.Close();
	} else {
	    inifile.Close();
	    *status = 0;
	    ERS(_("Named INI parameter #<%s> not found in INI file '%s': error=0x%x"),
		nameBuf, iniFileName, retval);
	}
    }
    return INTERP_OK;
}

// if the variable is of the form '_hal[hal_name]', then treat it as
// a HAL pin, signal or param. Lookup value, convert to float, and export as global and read-only.
// do not cache.
// the shortest possible INI variable is '_hal[x]' or 7 chars long .
int Interp::fetch_hal_param( const char *nameBuf, int *status, double *value)
{
    static int comp_id;
    int retval;
    hal_type_t type = HAL_TYPE_UNINITIALIZED;
    hal_data_u* ptr;
    bool conn;
    char hal_name[HAL_NAME_LEN];

    *status = 0;
    if (!comp_id) {
	char hal_comp[HAL_NAME_LEN];
	snprintf(hal_comp, sizeof(hal_comp),"interp%d",getpid());
	comp_id = hal_init(hal_comp); // manpage says: NULL ok - which fails miserably
	CHKS(comp_id < 0,_("fetch_hal_param: hal_init(%s): %d"), hal_comp,comp_id);
	CHKS((retval = hal_ready(comp_id)), _("fetch_hal_param: hal_ready(): %d"),retval);
    }
    char *s;
    int n = strlen(nameBuf);
    if ((n > 6) &&
	((s = (char *) strchr(&nameBuf[5],']')) != NULL)) {

	int closeBracket = s - nameBuf;

	strncpy(hal_name, &nameBuf[5], closeBracket);
	hal_name[closeBracket - 5] = '\0';
	if (nameBuf[closeBracket + 1]) {
	    logOword("%s: trailing garbage after closing bracket", hal_name);
	    *status = 0;
	    ERS("%s: trailing garbage after closing bracket", nameBuf);
	}
	// the result of these lookups could be cached in the parameter struct, but I'm not sure
	// this is a good idea - a removed pin/signal will not be noticed

	// I dont think that's needed - no change in pins/sigs/params
	// rtapi_mutex_get(&(hal_data->mutex)); 
        // rtapi_mutex_give(&(hal_data->mutex));

        if (hal_get_pin_value_by_name(hal_name, &type, &ptr, &conn) == 0) {
            if (!conn)
		logOword("%s: no signal connected", hal_name);
	    goto assign;
	}
        if (hal_get_signal_value_by_name(hal_name, &type, &ptr, &conn) == 0) {
	    if (!conn)
		logOword("%s: signal has no writer", hal_name);
	    goto assign;
	}
        if (hal_get_param_value_by_name(hal_name, &type, &ptr) == 0) {
	    goto assign;
	}
	*status = 0;
	ERS("Named hal parameter #<%s> not found", nameBuf);
    }
    return INTERP_OK;

    assign:
    switch (type) {
    case HAL_BIT: *value = (double) (ptr->b); break;
    case HAL_U32: *value = (double) (ptr->u); break;
    case HAL_S32: *value = (double) (ptr->s); break;
    case HAL_FLOAT: *value = (double) (ptr->f); break;
    default: return -1;
    }
    logOword("%s: value=%f", hal_name, *value);
    *status = 1;
    return INTERP_OK; 
}

int Interp::find_named_param(
    const char *nameBuf, //!< pointer to name to be read
    int *status,    //!< pointer to return status 1 => found
    double *value   //!< pointer to value of found parameter
    )
{
  context_pointer frame;
  parameter_map_iterator pi;
  int level;

  level = (nameBuf[0] == '_') ? 0 : _setup.call_level; // determine scope
  frame = &_setup.sub_context[level];
  *status = 0;

  pi = frame->named_params.find(nameBuf);
  if (pi == frame->named_params.end()) { // not found
      int exists = 0;
      double inivalue;
      if (FEATURE(INI_VARS) && (strncasecmp(nameBuf,"_ini[",5) == 0)) {
	 fetch_ini_param(nameBuf, &exists, &inivalue);
	  if (exists) {
	      logNP("parameter '%s' retrieved from INI: %f",nameBuf,inivalue);
	      *value = inivalue;
	      *status = 1;
	      parameter_value param;  // cache the value
	      param.value = inivalue;
	      param.attr = PA_GLOBAL | PA_READONLY | PA_FROM_INI;
	      _setup.sub_context[0].named_params[strstore(nameBuf)] = param;
	      return INTERP_OK;
	  } 
      }
      if (FEATURE(HAL_PIN_VARS) && (strncasecmp(nameBuf,"_hal[",5) == 0)) {
	  fetch_hal_param(nameBuf, &exists, &inivalue);
	  if (exists) {
	      logNP("parameter '%s' retrieved from HAL: %f",nameBuf,inivalue);
	      *value = inivalue;
	      *status = 1;
	      return INTERP_OK;
	  } 
      }
      *value = 0.0;
      *status = 0;
  } else {
      parameter_pointer pv = &pi->second;
      if (pv->attr & PA_UNSET)
	  logNP("warning: referencing unset variable '%s'",nameBuf);
      if (pv->attr & PA_USE_LOOKUP) {
	  CHP(lookup_named_param(nameBuf, pv->value, value));
	  *status = 1;
      } else if (pv->attr & PA_PYTHON) {
	  bp::object retval, tupleargs, kwargs;
	  bp::list plist;

	  plist.append(*_setup.pythis); // self
	  tupleargs = bp::tuple(plist);
	  kwargs = bp::dict();

	  python_plugin->call(NAMEDPARAMS_MODULE, nameBuf, tupleargs, kwargs, retval);
	  CHKS(python_plugin->plugin_status() == PLUGIN_EXCEPTION,
	       "named param - pycall(%s):\n%s", nameBuf,
	       python_plugin->last_exception().c_str());
	  CHKS(retval.ptr() == Py_None, "Python namedparams.%s returns no value", nameBuf);
      if (PyUnicode_Check(retval.ptr())) {
	      // returning a string sets the interpreter error message and aborts
	      *status = 0;
	      char *msg = bp::extract<char *>(retval);
	      ERS("%s", msg);
	  }
      if (PyLong_Check(retval.ptr())) { // widen
	      *value = (double) bp::extract<int>(retval);
	      *status = 1;
	      return INTERP_OK;
	  }
	  if (PyFloat_Check(retval.ptr())) {
	      *value =  bp::extract<double>(retval);
	      *status = 1;
	      return INTERP_OK;
	  }
	  // ok, that callable returned something botched.
	  *status = 0;
	  PyObject *res_str = PyObject_Str(retval.ptr());
	  Py_XDECREF(res_str);
	  ERS("Python call %s.%s returned '%s' - expected double, int or string, got %s",
	      NAMEDPARAMS_MODULE, nameBuf,
          PyUnicode_AsUTF8(res_str),
	      retval.ptr()->ob_type->tp_name);
      } else {
	  *value = pv->value;
	  *status = 1;
      }
  }
  return INTERP_OK;
}


int Interp::store_named_param(setup_pointer settings,
    const char *nameBuf, //!< pointer to name to be written
    double value,   //!< value to be written
    int override_readonly  //!< set to true to init a r/o parameter
    )
{
  context_pointer frame;
  int level;
  parameter_map_iterator pi;

  level = (nameBuf[0] == '_') ? 0 : _setup.call_level; // determine scope
  frame = &settings->sub_context[level];

  pi = frame->named_params.find(nameBuf);
  if (pi == frame->named_params.end()) {
      ERS(_("Internal error: Could not assign #<%s>"), nameBuf);
  } else {
      parameter_pointer pv = &pi->second;

      CHKS(((pv->attr & PA_GLOBAL)  && level),
	   "BUG: variable '%s' marked global, but assigned at level %d", nameBuf, level);

      if ((pv->attr & PA_READONLY) && !override_readonly) {
	  ERS(_("Cannot assign to read-only parameter #<%s>"), nameBuf);
      } else {
	  pv->value = value;
	  pv->attr &= ~PA_UNSET;
	  logNP("store_named_parameter: level[%d] %s value=%lf",
		level, nameBuf, value);
      }
  }
  return INTERP_OK;
}



int Interp::add_named_param(
    const char *nameBuf, //!< pointer to name to be added
    int attr) //!< see PA_* defs in interp_internal.hh
{
  static char name[] = "add_named_param";
  int findStatus;
  double value;
  int level;
  parameter_value param;

  // look it up to see if already exists
  CHP(find_named_param(nameBuf, &findStatus, &value));

  if (findStatus) {
      logNP("%s: parameter:|%s| already exists", name, nameBuf);
      return INTERP_OK;
  }
  attr |= PA_UNSET;

  if (nameBuf[0] != '_') {  // local scope
      level = _setup.call_level;
  } else {
      level = 0;    // call level zero is global scope
      attr |= PA_GLOBAL;
  }
  param.value = 0.0;
  param.attr = attr;
  _setup.sub_context[level].named_params[strstore(nameBuf)] = param;
  return INTERP_OK;
}


int Interp::free_named_parameters(context_pointer frame)
{
    frame->named_params.clear();
    return INTERP_OK;
}


// just a shorthand
int Interp::init_readonly_param(
    const char *nameBuf, //!< pointer to name to be added
    double value,  //!< initial value
    int attr)       //!< see PA_* defs in interp_internal.hh
{
    // static char name[] = "init_readonly_param";
    CHKS( add_named_param((char *) nameBuf, PA_READONLY|attr),
	  "adding r/o '%s'", nameBuf);
    CHKS(store_named_param(&_setup, (char *) nameBuf, value, OVERRIDE_READONLY),
	 "storing r/o '%s' %f", nameBuf, value);
    return INTERP_OK;
}


int Interp::lookup_named_param(const char *nameBuf,
			   double index,
			   double *value)
{
    int cmd = round_to_int(index);

    switch (cmd) {

	// some active_g_codes fields

    case NP_LINE: // _line - sequence number
	*value = _setup.sequence_number;
	break;

    case NP_MOTION_MODE: // _motion_mode
	*value = _setup.motion_mode;
	break;

    case NP_PLANE: // _plane
	switch(_setup.plane) {
	case CANON_PLANE::XY:
	    *value = G_17;
	    break;
	case CANON_PLANE::XZ:
	    *value = G_18;
	    break;
	case CANON_PLANE::YZ:
	    *value = G_19;
	    break;
	case CANON_PLANE::UV:
	    *value = G_17_1;
	    break;
	case CANON_PLANE::UW:
	    *value = G_18_1;
	    break;
	case CANON_PLANE::VW:
	    *value = G_19_1;
	    break;
	}
	break;

    case NP_CCOMP: // _ccomp - cutter compensation
	*value =
	    (_setup.cutter_comp_side == CUTTER_COMP::RIGHT) ? G_42 :
	(_setup.cutter_comp_side == CUTTER_COMP::LEFT) ? G_41 : G_40;
	break;

    case NP_METRIC: // _metric
	*value = (_setup.length_units == CANON_UNITS_MM);
	break;

    case NP_IMPERIAL: // _imperial
	*value = (_setup.length_units == CANON_UNITS_INCHES);
	break;

    case NP_ABSOLUTE: // _absolute - distance mode
	*value = (_setup.distance_mode == DISTANCE_MODE::ABSOLUTE);
	break;

    case NP_INCREMENTAL: // _incremental - distance mode
	*value = (_setup.distance_mode == DISTANCE_MODE::INCREMENTAL);
	break;

    case NP_INVERSE_TIME: // _inverse_time - feed mode
	*value = (_setup.feed_mode == FEED_MODE::INVERSE_TIME);
	break;

    case NP_UNITS_PER_MINUTE: // _units_per_minute - feed mode
	*value = (_setup.feed_mode == FEED_MODE::UNITS_PER_MINUTE);
	break;

    case NP_UNITS_PER_REV: // _units_per_rev - feed mode
	*value = (_setup.feed_mode == FEED_MODE::UNITS_PER_REVOLUTION);
	break;

    case NP_COORD_SYSTEM: // _coord_system - 0-9
	*value =
	    (_setup.origin_index < 7) ? (530 + (10 * _setup.origin_index)) :
	    (584 + _setup.origin_index);
	break;

    case NP_TOOL_OFFSET: // _tool_offset
	*value =  (_setup.tool_offset.tran.x || _setup.tool_offset.tran.y || _setup.tool_offset.tran.z ||
		   _setup.tool_offset.a || _setup.tool_offset.b || _setup.tool_offset.c ||
		   _setup.tool_offset.u || _setup.tool_offset.v || _setup.tool_offset.w) ;
	break;

    case NP_RETRACT_R_PLANE: // _retract_r_plane - G98
	*value = (_setup.retract_mode == RETRACT_MODE::R_PLANE);
	break;

    case NP_RETRACT_OLD_Z: // _retract_old_z - G99
	*value = (_setup.retract_mode == RETRACT_MODE::OLD_Z);
	break;

    case NP_SPINDLE_RPM_MODE: // _spindle_rpm_mode G97 currently only reports for spindle 0
	*value = (_setup.spindle_mode[0] == SPINDLE_MODE::CONSTANT_RPM);
	break;

    case NP_SPINDLE_CSS_MODE: // _spindle_css_mode G96
	*value = (_setup.spindle_mode[0] == SPINDLE_MODE::CONSTANT_SURFACE);
	break;

    case NP_IJK_ABSOLUTE_MODE: //_ijk_absolute_mode - G90.1
	*value = (_setup.ijk_distance_mode == DISTANCE_MODE::ABSOLUTE);
	break;

    case NP_LATHE_DIAMETER_MODE: // _lathe_diameter_mode - G7
	*value = _setup.lathe_diameter_mode;
	break;

    case NP_LATHE_RADIUS_MODE: // _lathe_radius_mode - G8
	*value = (_setup.lathe_diameter_mode == 0);
	break;

	// some active_m_codes fields

    case NP_SPINDLE_ON: // _spindle_on
	*value = (_setup.spindle_turning[0] != CANON_STOPPED);
	break;

    case NP_SPINDLE_CW: // spindle_cw
	*value = (_setup.spindle_turning[0] == CANON_CLOCKWISE);
	break;

    case NP_MIST: // mist
	*value = _setup.mist;
	break;

    case NP_FLOOD: // flood
	*value = _setup.flood;
	break;

    case NP_SPEED_OVERRIDE: // speed override
	*value = _setup.speed_override[0];
	break;

    case NP_FEED_OVERRIDE: // feed override
	*value = _setup.feed_override;
	break;

    case NP_ADAPTIVE_FEED: // adaptive feed
	*value = _setup.adaptive_feed;
	break;

    case NP_FEED_HOLD: // feed hold
	*value = _setup.feed_hold;
	break;

	// from active_settings:
    case NP_FEED: // feed
	*value = _setup.feed_rate;
	break;

    case NP_RPM: // speed (rpm)
	*value = abs(_setup.speed[0]);
	break;

    case NP_CURRENT_TOOL:
	*value = _setup.parameters[5400];
	break;

    case NP_SELECTED_POCKET:
    if(_setup.random_toolchanger){//random changers already report the real pocket number
        *value = _setup.selected_pocket;
    }
    else{//non random get it from the tool table
        if(_setup.tool_table[_setup.selected_pocket].pocketno == 0){//pocket 0 is special on non-random changers
            *value = -1;
        }
        else{
	        *value = _setup.tool_table[_setup.selected_pocket].pocketno;
        }
    }
	break;

    case NP_CURRENT_POCKET:
    if (_setup.current_pocket == -1) {
	    *value = -1;
	    break;
    }
    if(_setup.random_toolchanger){//random changers already report the real pocket number
	    *value = _setup.current_pocket;
    }
    else{//non random get it from the tool table
        *value = _setup.tool_table[_setup.current_pocket].pocketno;
    }
	break;

    case NP_SELECTED_TOOL:
	*value = _setup.selected_tool;
	break;

    case NP_X:  // current position
	*value = _setup.current_x;
	break;

    case NP_Y:  // current position
	*value = _setup.current_y;
	break;

    case NP_Z:  // current position
	*value = _setup.current_z;
	break;

    case NP_A:  // current position
	*value = _setup.AA_current;
	break;

    case NP_B:  // current position
	*value = _setup.BB_current;
	break;

    case NP_C:  // current position
	*value = _setup.CC_current;
	break;

    case NP_U:  // current position
	*value = _setup.u_current;
	break;

    case NP_V:  // current position
	*value = _setup.v_current;
	break;

    case NP_W:  // current position
	*value = _setup.w_current;
	break;

    case NP_ABS_X:  // abs position
        {
            double x = _setup.current_x + _setup.axis_offset_x;
            double y = _setup.current_y + _setup.axis_offset_y;
            rotate(&x, &y, _setup.rotation_xy);
	    *value = x + _setup.origin_offset_x + _setup.tool_offset.tran.x;
        }
	break;

    case NP_ABS_Y:  // abs position
        {
            double x = _setup.current_x + _setup.axis_offset_x;
            double y = _setup.current_y + _setup.axis_offset_y;
            rotate(&x, &y, _setup.rotation_xy);
	    *value = y + _setup.origin_offset_y + _setup.tool_offset.tran.y;
        }
	break;


    case NP_ABS_Z:  // abs position
	*value = _setup.current_z + _setup.axis_offset_z +
                 _setup.origin_offset_z + _setup.tool_offset.tran.z;
	break;

    case NP_ABS_A:  // abs position
	*value = _setup.AA_current + _setup.AA_axis_offset +
                 _setup.AA_origin_offset + _setup.tool_offset.a;
	break;

    case NP_ABS_B:  // abs position
	*value = _setup.BB_current + _setup.BB_axis_offset +
                 _setup.BB_origin_offset + _setup.tool_offset.b;
	break;

    case NP_ABS_C:  // abs position
	*value = _setup.CC_current + _setup.CC_axis_offset +
                 _setup.CC_origin_offset + _setup.tool_offset.c;
	break;

	// o-word subs may optionally have an
	// expression after endsub and return
	// this 'function return value' is accessible as '_value'
    case NP_VALUE:
	*value = _setup.return_value;
	break;

	// predicate: the last NGC procedure did/did not return a value
    case NP_VALUE_RETURNED:
	*value = _setup.value_returned;
	break;

    case NP_CALL_LEVEL:
	*value = _setup.call_level;
	break;

    case NP_REMAP_LEVEL:
	*value = _setup.remap_level;
	break;
	
    case NP_TASK:
	extern int _task;  // zero in gcodemodule, 1 in milltask
	*value = _task;
	break;

    default:
	ERS(_("BUG: lookup_named_param(%s): unhandled index=%fn"),
	      nameBuf,index);
    }
    return INTERP_OK;
}

int Interp::init_python_predef_parameter(const char *name)
{
    int exists = 0;
    double value;
    parameter_value param;

    if (name[0] == '_') { // globals only
	find_named_param(name, &exists, &value);
	if (exists) {
	    fprintf(stderr, "warning: redefining named parameter %s\n",name);
	    _setup.sub_context[0].named_params.erase(name);
	}
	param.value = 0.0;
	param.attr = PA_READONLY|PA_PYTHON|PA_GLOBAL;
	_setup.sub_context[0].named_params[strstore(name)] = param;
    }
    return INTERP_OK;
}

int Interp::init_named_parameters()
{

// version       major   minor      Note
// ------------ -------- ---------- -------------------------------------
// M.N.m         M.N     0.m        normal format
// M.N.m~xxx     M.N     0.m        pre-release format
  const char *pkgversion = PACKAGE_VERSION;  //examples: 2.4.6, 2.5.0~pre
  const char *version_major = "_vmajor";// named_parameter name (use lower case)
  const char *version_minor = "_vminor";// named_parameter name (use lower case)
  const char *metric_machine = "_metric_machine";// named_parameter name (use lower case)
  double vmajor=0.0, vminor=0.0, munits = 1.0;
  sscanf(pkgversion, "%lf%lf", &vmajor, &vminor);

  init_readonly_param(version_major,vmajor,0);
  init_readonly_param(version_minor,vminor,0);

  munits = inicheck();
  init_readonly_param(metric_machine,munits,0);

  // params tagged with PA_USE_LOOKUP will call the lookup_named_param()
  // method. The value is used as a index for the switch() statement.

  // the active_g_codes fields

  // I guess this is the line number
  init_readonly_param("_line", NP_LINE, PA_USE_LOOKUP);

  // any of G1 G2 G3 G5.2 G73 G80 G82 G83 G86 G87 G88 G89
  // value is number after 'G' multiplied by 10 (10,20,30,52..)

  init_readonly_param("_motion_mode", NP_MOTION_MODE, PA_USE_LOOKUP);

  // G17/18/19/17.1/18.1/19.1 -> return 170/180/190/171/181/191
  init_readonly_param("_plane", NP_PLANE, PA_USE_LOOKUP);

  // return 400,410,420 depending if (G40,G41,G42) is on
  init_readonly_param("_ccomp", NP_CCOMP, PA_USE_LOOKUP);

  // 1.0 if G21 is on
  init_readonly_param("_metric", NP_METRIC, PA_USE_LOOKUP);

  // 1.0 if G20 is on
  init_readonly_param("_imperial", NP_IMPERIAL, PA_USE_LOOKUP);

  //1.0 if G90 is on
  init_readonly_param("_absolute", NP_ABSOLUTE, PA_USE_LOOKUP);

  //1.0 if G91 is on
  init_readonly_param("_incremental", NP_INCREMENTAL, PA_USE_LOOKUP);

  // 1.0 if G93 is on
  init_readonly_param("_inverse_time", NP_INVERSE_TIME, PA_USE_LOOKUP);

  // 1.0 if G94 is on
  init_readonly_param("_units_per_minute", NP_UNITS_PER_MINUTE, PA_USE_LOOKUP);

  // 1.0 if G95 is on
  init_readonly_param("_units_per_rev", NP_UNITS_PER_REV, PA_USE_LOOKUP);

  // 0..9 for G54..G59.3
  init_readonly_param("_coord_system", NP_COORD_SYSTEM, PA_USE_LOOKUP);

  // 1.0 if G43 is on
  init_readonly_param("_tool_offset", NP_TOOL_OFFSET, PA_USE_LOOKUP);

  // 1 if G98 set
  init_readonly_param("_retract_r_plane", NP_RETRACT_R_PLANE, PA_USE_LOOKUP);

  // 1 if G99 set
  init_readonly_param("_retract_old_z", NP_RETRACT_OLD_Z, PA_USE_LOOKUP);

  // really esoteric
  // init_readonly_param("_control_mode", 110, PA_USE_LOOKUP);


  // 1 if G97 is on
  init_readonly_param("_spindle_rpm_mode", NP_SPINDLE_RPM_MODE, PA_USE_LOOKUP);

  init_readonly_param("_spindle_css_mode", NP_SPINDLE_CSS_MODE, PA_USE_LOOKUP);

  // 1 if G90.1 is on
  init_readonly_param("_ijk_absolute_mode", NP_IJK_ABSOLUTE_MODE, PA_USE_LOOKUP);

  // 1 if G7 is on
  init_readonly_param("_lathe_diameter_mode", NP_LATHE_DIAMETER_MODE, PA_USE_LOOKUP);

  // 1 if G8 is on
  init_readonly_param("_lathe_radius_mode", NP_LATHE_RADIUS_MODE, PA_USE_LOOKUP);


  // the active_m_codes fields
  init_readonly_param("_spindle_on", NP_SPINDLE_ON, PA_USE_LOOKUP);
  init_readonly_param("_spindle_cw", NP_SPINDLE_CW, PA_USE_LOOKUP);

  init_readonly_param("_mist", NP_MIST, PA_USE_LOOKUP);
  init_readonly_param("_flood", NP_FLOOD, PA_USE_LOOKUP);
  init_readonly_param("_speed_override", NP_SPEED_OVERRIDE, PA_USE_LOOKUP);
  init_readonly_param("_feed_override", NP_FEED_OVERRIDE, PA_USE_LOOKUP);
  init_readonly_param("_adaptive_feed", NP_ADAPTIVE_FEED, PA_USE_LOOKUP);
  init_readonly_param("_feed_hold", NP_FEED_HOLD, PA_USE_LOOKUP);

  // active_settings
  init_readonly_param("_feed", NP_FEED, PA_USE_LOOKUP);
  init_readonly_param("_rpm", NP_RPM, PA_USE_LOOKUP);


  // tool related
  init_readonly_param("_current_tool", NP_CURRENT_TOOL, PA_USE_LOOKUP);
  init_readonly_param("_current_pocket", NP_CURRENT_POCKET, PA_USE_LOOKUP);
  init_readonly_param("_selected_pocket", NP_SELECTED_POCKET, PA_USE_LOOKUP);
  init_readonly_param("_selected_tool", NP_SELECTED_TOOL, PA_USE_LOOKUP);

  // current position - alias to #5420-#5429
  init_readonly_param("_x", NP_X, PA_USE_LOOKUP);
  init_readonly_param("_y", NP_Y, PA_USE_LOOKUP);
  init_readonly_param("_z", NP_Z, PA_USE_LOOKUP);
  init_readonly_param("_a", NP_A, PA_USE_LOOKUP);
  init_readonly_param("_b", NP_B, PA_USE_LOOKUP);
  init_readonly_param("_c", NP_C, PA_USE_LOOKUP);
  init_readonly_param("_u", NP_U, PA_USE_LOOKUP);
  init_readonly_param("_v", NP_V, PA_USE_LOOKUP);
  init_readonly_param("_w", NP_W, PA_USE_LOOKUP);

  // current abs position, does not include any offset
  init_readonly_param("_abs_x", NP_ABS_X, PA_USE_LOOKUP);
  init_readonly_param("_abs_y", NP_ABS_Y, PA_USE_LOOKUP);
  init_readonly_param("_abs_z", NP_ABS_Z, PA_USE_LOOKUP);
  init_readonly_param("_abs_a", NP_ABS_A, PA_USE_LOOKUP);
  init_readonly_param("_abs_b", NP_ABS_B, PA_USE_LOOKUP);
  init_readonly_param("_abs_c", NP_ABS_C, PA_USE_LOOKUP);

  // last (optional) endsub/return value
  init_readonly_param("_value", NP_VALUE, PA_USE_LOOKUP);

  // predicate: last NGC procedure did return a value on endsub/return
  init_readonly_param("_value_returned", NP_VALUE_RETURNED, PA_USE_LOOKUP);

  // predicate: 1 in milltask instance, 0 in UI - control preview behaviour
  init_readonly_param("_task", NP_TASK, PA_USE_LOOKUP);

  // debugging aids
  init_readonly_param("_call_level", NP_CALL_LEVEL, PA_USE_LOOKUP);
  init_readonly_param("_remap_level", NP_REMAP_LEVEL, PA_USE_LOOKUP);

  return INTERP_OK;
}

double Interp::inicheck()
{
    IniFile inifile;
    const char *filename;
    std::optional<const char*> inistring;
    double result = -1.0;

	if ((filename = getenv("INI_FILE_NAME")) == NULL) {
	    return -1.0;
    }

    // open it
    if (inifile.Open(filename) == false) {
	    return -1.0;
    }

    if ((inistring = inifile.Find("LINEAR_UNITS", "TRAJ"))) {
        if (!strcmp(*inistring, "inch")) {
             result = 0.0;
        } else {
            result = 1.0;
        }
    }
    // close it
    inifile.Close();

    return result;
}
