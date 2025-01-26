/* misnomer: _setup.current_pocket,selected_pocket
** These variables are actually indexes to sequential tool
** data structs (not real pockets).
** Future renaming will affect current usage in python remaps.
*/

/*    This is a component of LinuxCNC
 *    Copyright 2011, 2012, 2013 Michael Haberler <git@mah.priv.at>,
 *    Sebastian Kuzminsky <seb@highlab.com>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
// Interpreter internals - Python bindings
// Michael Haberler 7/2011
//

#define BOOST_PYTHON_MAX_ARITY 4
#include <boost/python/class.hpp>
#include <boost/python/def.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/module.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost_pyenum_macros.hh>
#include <map>

namespace bp = boost::python;
extern int _task;  // zero in gcodemodule, 1 in milltask

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "rs274ngc.hh"
#include "interp_return.hh"
#include "interp_internal.hh"
#include "rs274ngc_interp.hh"
#include "units.h"
#include "array1.hh"

extern void export_Internals();
extern void export_Arrays();
extern void export_Block();
extern void export_EmcTypes();
#include "paramclass.hh"

namespace pp = pyplusplus::containers::static_sized;
#include "interp_array_types.hh"

#define IS_STRING(x) (PyObject_IsInstance(x.ptr(), (PyObject*)&PyString_Type))
#define IS_INT(x) (PyObject_IsInstance(x.ptr(), (PyObject*)&PyInt_Type))

static  spindle_speed_array spindle_speed_wrapper (Interp & inst) {
    return spindle_speed_array(inst._setup.speed);
}

static  active_g_codes_array active_g_codes_wrapper ( Interp & inst) {
    return active_g_codes_array(inst._setup.active_g_codes);
}
static  active_m_codes_array active_m_codes_wrapper ( Interp & inst) {
    return active_m_codes_array(inst._setup.active_m_codes);
}

static  active_settings_array active_settings_wrapper ( Interp & inst) {
    return active_settings_array(inst._setup.active_settings);
}

static  blocks_array blocks_wrapper ( Interp & inst) {
    return blocks_array(inst._setup.blocks);
}

static  parameters_array parameters_wrapper ( Interp & inst) {
    return parameters_array(inst._setup.parameters);
}

static  tool_table_array tool_table_wrapper ( Interp & inst) {
    return tool_table_array(inst._setup.tool_table);
}

static  sub_context_array sub_context_wrapper ( Interp & inst) {
    return sub_context_array(inst._setup.sub_context);
}


static void setErrorMsg(Interp &interp, const char *s)
{
    setup *settings  = &interp._setup;

    if ((s == NULL) || (strlen(s) == 0))
	s = "###";
    interp.setError ("%s", s);
    settings->stack_index = 0;
    strncpy(settings->stack[settings->stack_index],
	    "Python", STACK_ENTRY_LEN);
    settings->stack[settings->stack_index][STACK_ENTRY_LEN-1] = 0;
    settings->stack_index++;
    settings->stack[settings->stack_index][0] = 0;
}

static bp::object errorStack(Interp &interp)
{
    setup *settings  = &interp._setup;
    bp::list msgs;

    for (int i = 0; i < settings->stack_index; i++)
	msgs.append(bp::object( (const char *) settings->stack[i]));
    return msgs;
}

static bp::object wrap_find_tool_pocket(Interp &interp, int toolno)
{
    int status, pocket;
    setup *settings  =  &interp._setup;

    status = interp.find_tool_pocket(settings, toolno, &pocket);
    return bp::make_tuple(status, pocket);
}


// FIXME not sure if this is really needed
static  ParamClass param_wrapper ( Interp & inst) {
    return ParamClass(inst);
}

static int get_task(Interp & /*i*/) { return _task; };
static const char *get_filename(Interp &i) { return i._setup.filename; };
static const char *get_linetext(Interp &i) { return i._setup.linetext; };

// those are exposed here because they look useful for regression testing
static bool __equal(double a, double b) { return fabs(a - b) < TOLERANCE_EQUAL; }
// see interp_convert.cc
static bool is_near_int(double value) {
    int i = (int)(value + .5);
    return fabs(i - value) < .0001;
}
static int nearest_int(double value) { return (int)(value + .5); }

int (Interp::*execute_1)(const char *command) = &Interp::execute;
int (Interp::*execute_2)(const char *command, int line_number) = &Interp::execute;

// lifted from http://stackoverflow.com/questions/2261858/boostpython-export-custom-exception


class InterpreterException : public std::exception {
private:
    std::string error_message;
    int line_number;
    std::string line_text;
public:
    InterpreterException(std::string error_message, int line_number, std::string line_text)  {
	this->error_message = error_message;
	this->line_number = line_number;
	this->line_text = line_text;
    }
    const char *what() const noexcept { return this->error_message.c_str();  }

    ~InterpreterException() noexcept  {}
    std::string get_error_message()  { return this->error_message;  }
    int get_line_number()    { return this->line_number;  }
    std::string get_line_text()      { return this->line_text; }
};

static PyObject *InterpreterExceptionType = NULL;

void translateInterpreterException(InterpreterException const &e)
{
  assert(InterpreterExceptionType != NULL);
  bp::object pythonExceptionInstance(e);
  PyErr_SetObject(InterpreterExceptionType, pythonExceptionInstance.ptr());
}

static int throw_exceptions = 1;

static int wrap_interp_execute_1(Interp &interp, const char *command)
{    
    setup &_setup = interp._setup;
    block saved_block = _setup.blocks[0];
    int saved_call_state = _setup.call_state;

    // Temporarily set the call state to CS_NORMAL
    _setup.call_state = CS_NORMAL;
    int status = interp.execute(command);
    _setup.call_state = saved_call_state;
    _setup.blocks[0] = saved_block;

    // printf("ie1: tc=%d if=%d pf=%d\n", _setup.toolchange_flag,_setup.input_flag,_setup.probe_flag);

    if ((status > INTERP_MIN_ERROR) && throw_exceptions) {
	throw InterpreterException(interp.getSavedError(),
				   _setup.blocks[0].line_number, // not sure
				   _setup.linetext);
    }
    return status;
}

static int wrap_interp_execute_2(Interp &interp, const char *command, int lineno)
{
    setup &_setup = interp._setup;
    block saved_block = _setup.blocks[0];

    // use the remap stack to save/restore the current block
    CHP(interp.enter_remap());
    int status = interp.execute(command, lineno);
    CHP(interp.leave_remap());
    _setup.blocks[0] = saved_block;
    
    //printf("ie2: tc=%d if=%d pf=%d\n", _setup.toolchange_flag,_setup.input_flag,_setup.probe_flag);
    if ((status > INTERP_MIN_ERROR) && throw_exceptions) {
	throw InterpreterException(interp.getSavedError(),
				   lineno, // not sure
				   _setup.linetext);
    }
    return status;
}

// this might not be a good idea - it destroys the block which has a 'o<ngcbody> call' parsed in it
static int wrap_interp_read(Interp &interp, const char *command)
{
    setup &_setup = interp._setup;
    block saved_block = _setup.blocks[0];

    int status = interp.read(command);
    if ((status > INTERP_MIN_ERROR) && throw_exceptions) {
	throw InterpreterException(interp.getSavedError(),
				   _setup.blocks[0].line_number, // not sure
				   _setup.linetext);
    }
    return status;
}

// static inline remap_map wrap_remaps(Interp &interp)  {
//     return interp._setup.remaps;
// }

// what a barren desert
static inline EmcPose get_tool_offset (Interp &interp)  {
    return interp._setup.tool_offset;
}
static inline void set_tool_offset(Interp &interp, EmcPose value)  {
    interp._setup.tool_offset = value;
}
static inline bool get_arc_not_allowed (Interp &interp)  {
    return interp._setup.arc_not_allowed;
}
static inline void set_arc_not_allowed(Interp &interp, bool value)  {
    interp._setup.arc_not_allowed = value;
}
static inline bool get_cutter_comp_firstmove (Interp &interp)  {
    return interp._setup.cutter_comp_firstmove;
}
static inline void set_cutter_comp_firstmove(Interp &interp, bool value)  {
    interp._setup.cutter_comp_firstmove = value;
}
static inline bool get_feed_override (Interp &interp)  {
    return interp._setup.feed_override;
}
static inline void set_feed_override(Interp &interp, bool value)  {
    interp._setup.feed_override = value;
}
static inline bool get_input_digital (Interp &interp)  {
    return interp._setup.input_digital;
}
static inline void set_input_digital(Interp &interp, bool value)  {
    interp._setup.input_digital = value;
}
static inline bool get_input_flag (Interp &interp)  {
    return interp._setup.input_flag;
}
static inline void set_input_flag(Interp &interp, bool value)  {
    interp._setup.input_flag = value;
}
static inline bool get_mdi_interrupt (Interp &interp)  {
    return interp._setup.mdi_interrupt;
}
static inline void set_mdi_interrupt(Interp &interp, bool value)  {
    interp._setup.mdi_interrupt = value;
}
static inline bool get_mist (Interp &interp)  {
    return interp._setup.mist;
}
static inline void set_mist(Interp &interp, bool value)  {
    interp._setup.mist = value;
}
static inline bool get_flood (Interp &interp)  {
    return interp._setup.flood;
}
static inline void set_flood(Interp &interp, bool value)  {
    interp._setup.flood = value;
}
static inline bool get_percent_flag (Interp &interp)  {
    return interp._setup.percent_flag;
}
static inline void set_percent_flag(Interp &interp, bool value)  {
    interp._setup.percent_flag = value;
}
static inline bool get_probe_flag (Interp &interp)  {
    return interp._setup.probe_flag;
}
static inline void set_probe_flag(Interp &interp, bool value)  {
    interp._setup.probe_flag = value;
}
static inline bool get_speed_override (Interp &interp, int spindle)  {
    return interp._setup.speed_override[spindle];
}
static inline void set_speed_override(Interp &interp, int spindle, bool value)  {
    interp._setup.speed_override[spindle] = value;
}
static inline bool get_toolchange_flag (Interp &interp)  {
    return interp._setup.toolchange_flag;
}
static inline void set_toolchange_flag(Interp &interp, bool value)  {
    interp._setup.toolchange_flag = value;
}
static inline double get_u_current (Interp &interp)  {
    return interp._setup.u_current;
}
static inline void set_u_current(Interp &interp, double value)  {
    interp._setup.u_current = value;
}
static inline double get_AA_axis_offset (Interp &interp)  {
    return interp._setup.AA_axis_offset;
}
static inline void set_AA_axis_offset(Interp &interp, double value)  {
    interp._setup.AA_axis_offset = value;
}
static inline double get_AA_current (Interp &interp)  {
    return interp._setup.AA_current;
}
static inline void set_AA_current(Interp &interp, double value)  {
    interp._setup.AA_current = value;
}
static inline double get_AA_origin_offset (Interp &interp)  {
    return interp._setup.AA_origin_offset;
}
static inline void set_AA_origin_offset(Interp &interp, double value)  {
    interp._setup.AA_origin_offset = value;
}
static inline double get_BB_axis_offset (Interp &interp)  {
    return interp._setup.BB_axis_offset;
}
static inline void set_BB_axis_offset(Interp &interp, double value)  {
    interp._setup.BB_axis_offset = value;
}
static inline double get_BB_current (Interp &interp)  {
    return interp._setup.BB_current;
}
static inline void set_BB_current(Interp &interp, double value)  {
    interp._setup.BB_current = value;
}
static inline double get_BB_origin_offset (Interp &interp)  {
    return interp._setup.BB_origin_offset;
}
static inline void set_BB_origin_offset(Interp &interp, double value)  {
    interp._setup.BB_origin_offset = value;
}
static inline double get_CC_axis_offset (Interp &interp)  {
    return interp._setup.CC_axis_offset;
}
static inline void set_CC_axis_offset(Interp &interp, double value)  {
    interp._setup.CC_axis_offset = value;
}
static inline double get_CC_current (Interp &interp)  {
    return interp._setup.CC_current;
}
static inline void set_CC_current(Interp &interp, double value)  {
    interp._setup.CC_current = value;
}
static inline double get_CC_origin_offset (Interp &interp)  {
    return interp._setup.CC_origin_offset;
}
static inline void set_CC_origin_offset(Interp &interp, double value)  {
    interp._setup.CC_origin_offset = value;
}

static inline int get_active_spindle (Interp const & interp)  {
    return interp._setup.active_spindle;
}
static inline void set_active_spindle(Interp & interp, int value)  {
    interp._setup.active_spindle = value;
}

static inline double get_axis_offset_x (Interp &interp)  {
    return interp._setup.axis_offset_x;
}
static inline void set_axis_offset_x(Interp &interp, double value)  {
    interp._setup.axis_offset_x = value;
}
static inline double get_axis_offset_y (Interp &interp)  {
    return interp._setup.axis_offset_y;
}
static inline void set_axis_offset_y(Interp &interp, double value)  {
    interp._setup.axis_offset_y = value;
}
static inline double get_axis_offset_z (Interp &interp)  {
    return interp._setup.axis_offset_z;
}
static inline void set_axis_offset_z(Interp &interp, double value)  {
    interp._setup.axis_offset_z = value;
}
static inline double get_current_x (Interp &interp)  {
    return interp._setup.current_x;
}
static inline void set_current_x(Interp &interp, double value)  {
    interp._setup.current_x = value;
}
static inline double get_current_y (Interp &interp)  {
    return interp._setup.current_y;
}
static inline void set_current_y(Interp &interp, double value)  {
    interp._setup.current_y = value;
}
static inline double get_current_z (Interp &interp)  {
    return interp._setup.current_z;
}
static inline void set_current_z(Interp &interp, double value)  {
    interp._setup.current_z = value;
}
static inline double get_cutter_comp_radius (Interp &interp)  {
    return interp._setup.cutter_comp_radius;
}
static inline void set_cutter_comp_radius(Interp &interp, double value)  {
    interp._setup.cutter_comp_radius = value;
}
static inline double get_cycle_cc (Interp &interp)  {
    return interp._setup.cycle_cc;
}
static inline void set_cycle_cc(Interp &interp, double value)  {
    interp._setup.cycle_cc = value;
}
static inline double get_cycle_i (Interp &interp)  {
    return interp._setup.cycle_i;
}
static inline void set_cycle_i(Interp &interp, double value)  {
    interp._setup.cycle_i = value;
}
static inline double get_cycle_il (Interp &interp)  {
    return interp._setup.cycle_il;
}
static inline void set_cycle_il(Interp &interp, double value)  {
    interp._setup.cycle_il = value;
}
static inline double get_cycle_j (Interp &interp)  {
    return interp._setup.cycle_j;
}
static inline void set_cycle_j(Interp &interp, double value)  {
    interp._setup.cycle_j = value;
}
static inline double get_cycle_k (Interp &interp)  {
    return interp._setup.cycle_k;
}
static inline void set_cycle_k(Interp &interp, double value)  {
    interp._setup.cycle_k = value;
}
static inline double get_cycle_p (Interp &interp)  {
    return interp._setup.cycle_p;
}
static inline void set_cycle_p(Interp &interp, double value)  {
    interp._setup.cycle_p = value;
}
static inline double get_cycle_q (Interp &interp)  {
    return interp._setup.cycle_q;
}
static inline void set_cycle_q(Interp &interp, double value)  {
    interp._setup.cycle_q = value;
}
static inline double get_cycle_r (Interp &interp)  {
    return interp._setup.cycle_r;
}
static inline void set_cycle_r(Interp &interp, double value)  {
    interp._setup.cycle_r = value;
}
static inline double get_feed_rate (Interp &interp)  {
    return interp._setup.feed_rate;
}
static inline void set_feed_rate(Interp &interp, double value)  {
    interp._setup.feed_rate = value;
}
static inline double get_origin_offset_x (Interp &interp)  {
    return interp._setup.origin_offset_x;
}
static inline void set_origin_offset_x(Interp &interp, double value)  {
    interp._setup.origin_offset_x = value;
}
static inline double get_origin_offset_y (Interp &interp)  {
    return interp._setup.origin_offset_y;
}
static inline void set_origin_offset_y(Interp &interp, double value)  {
    interp._setup.origin_offset_y = value;
}
static inline double get_origin_offset_z (Interp &interp)  {
    return interp._setup.origin_offset_z;
}
static inline void set_origin_offset_z(Interp &interp, double value)  {
    interp._setup.origin_offset_z = value;
}
static inline double get_program_x (Interp &interp)  {
    return interp._setup.program_x;
}
static inline void set_program_x(Interp &interp, double value)  {
    interp._setup.program_x = value;
}
static inline double get_program_y (Interp &interp)  {
    return interp._setup.program_y;
}
static inline void set_program_y(Interp &interp, double value)  {
    interp._setup.program_y = value;
}
static inline double get_program_z (Interp &interp)  {
    return interp._setup.program_z;
}
static inline void set_program_z(Interp &interp, double value)  {
    interp._setup.program_z = value;
}
static inline double get_return_value (Interp &interp)  {
    return interp._setup.return_value;
}
static inline void set_return_value(Interp &interp, double value)  {
    interp._setup.return_value = value;
}
static inline double get_rotation_xy (Interp &interp)  {
    return interp._setup.rotation_xy;
}
static inline void set_rotation_xy(Interp &interp, double value)  {
    interp._setup.rotation_xy = value;
}
static inline double get_traverse_rate (Interp &interp)  {
    return interp._setup.traverse_rate;
}
static inline void set_traverse_rate(Interp &interp, double value)  {
    interp._setup.traverse_rate = value;
}
static inline double get_u_axis_offset (Interp &interp)  {
    return interp._setup.u_axis_offset;
}
static inline void set_u_axis_offset(Interp &interp, double value)  {
    interp._setup.u_axis_offset = value;
}
static inline double get_u_origin_offset (Interp &interp)  {
    return interp._setup.u_origin_offset;
}
static inline void set_u_origin_offset(Interp &interp, double value)  {
    interp._setup.u_origin_offset = value;
}
static inline double get_v_axis_offset (Interp &interp)  {
    return interp._setup.v_axis_offset;
}
static inline void set_v_axis_offset(Interp &interp, double value)  {
    interp._setup.v_axis_offset = value;
}
static inline double get_v_current(Interp &interp)  {
    return interp._setup.v_current;
}
static inline void set_v_current(Interp &interp, double value)  {
    interp._setup.v_current = value;
}
static inline double get_v_origin_offset (Interp &interp)  {
    return interp._setup.v_origin_offset;
}
static inline void set_v_origin_offset(Interp &interp, double value)  {
    interp._setup.v_origin_offset = value;
}
static inline double get_w_axis_offset (Interp &interp)  {
    return interp._setup.w_axis_offset;
}
static inline void set_w_axis_offset(Interp &interp, double value)  {
    interp._setup.w_axis_offset = value;
}
static inline double get_w_current (Interp &interp)  {
    return interp._setup.w_current;
}
static inline void set_w_current(Interp &interp, double value)  {
    interp._setup.w_current = value;
}
static inline double get_w_origin_offset (Interp &interp)  {
    return interp._setup.w_origin_offset;
}
static inline void set_w_origin_offset(Interp &interp, double value)  {
    interp._setup.w_origin_offset = value;
}
static inline int get_a_axis_wrapped (Interp &interp)  {
    return interp._setup.a_axis_wrapped;
}
static inline void set_a_axis_wrapped(Interp &interp, int value)  {
    interp._setup.a_axis_wrapped = value;
}
static inline int get_a_indexer (Interp &interp)  {
    return interp._setup.a_indexer_jnum;
}
static inline void set_a_indexer(Interp &interp, int value)  {
    interp._setup.a_indexer_jnum = value;
}
static inline int get_b_axis_wrapped (Interp &interp)  {
    return interp._setup.b_axis_wrapped;
}
static inline void set_b_axis_wrapped(Interp &interp, int value)  {
    interp._setup.b_axis_wrapped = value;
}
static inline int get_b_indexer (Interp &interp)  {
    return interp._setup.b_indexer_jnum;
}
static inline void set_b_indexer(Interp &interp, int value)  {
    interp._setup.b_indexer_jnum = value;
}
static inline int get_c_axis_wrapped (Interp &interp)  {
    return interp._setup.c_axis_wrapped;
}
static inline void set_c_axis_wrapped(Interp &interp, int value)  {
    interp._setup.c_axis_wrapped = value;
}
static inline int get_c_indexer (Interp &interp)  {
    return interp._setup.c_indexer_jnum;
}
static inline void set_c_indexer(Interp &interp, int value)  {
    interp._setup.c_indexer_jnum = value;
}
static inline int get_call_level (Interp &interp)  {
    return interp._setup.call_level;
}
static inline void set_call_level(Interp &interp, int value)  {
    interp._setup.call_level = value;
}
static inline int get_current_pocket (Interp &interp)  {
    return interp._setup.current_pocket;
}
static inline void set_current_pocket(Interp &interp, int value)  {
    interp._setup.current_pocket = value;
}
static inline int get_cutter_comp_orientation (Interp &interp)  {
    return interp._setup.cutter_comp_orientation;
}
static inline void set_cutter_comp_orientation(Interp &interp, int value)  {
    interp._setup.cutter_comp_orientation = value;
}
static inline int get_cutter_comp_side (Interp &interp)  {
    return (int)interp._setup.cutter_comp_side;
}
static inline void set_cutter_comp_side(Interp &interp, int value)  {
    interp._setup.cutter_comp_side = (CUTTER_COMP)value;
}
static inline int get_cycle_il_flag (Interp &interp)  {
    return interp._setup.cycle_il_flag;
}
static inline void set_cycle_il_flag(Interp &interp, int value)  {
    interp._setup.cycle_il_flag = value;
}
static inline int get_cycle_l (Interp &interp)  {
    return interp._setup.cycle_l;
}
static inline void set_cycle_l(Interp &interp, int value)  {
    interp._setup.cycle_l = value;
}
static inline int get_debugmask (Interp &interp)  {
    return interp._setup.debugmask;
}
static inline void set_debugmask(Interp &interp, int value)  {
    interp._setup.debugmask = value;
}
static inline int get_distance_mode (Interp &interp)  {
    return (int)interp._setup.distance_mode;
}
static inline void set_distance_mode(Interp &interp, DISTANCE_MODE value)  {
    interp._setup.distance_mode = value;
}
static inline int get_feed_mode (Interp &interp)  {
    return (int)interp._setup.feed_mode;
}
static inline void set_feed_mode(Interp &interp, int value)  {
    interp._setup.feed_mode = (FEED_MODE)value;
}
static inline int get_ijk_distance_mode (Interp &interp)  {
    return (int)interp._setup.ijk_distance_mode;
}
static inline void set_ijk_distance_mode(Interp &interp, DISTANCE_MODE value)  {
    interp._setup.ijk_distance_mode = value;
}
static inline int get_input_index (Interp &interp)  {
    return interp._setup.input_index;
}
static inline void set_input_index(Interp &interp, int value)  {
    interp._setup.input_index = value;
}
static inline int get_length_units (Interp &interp)  {
    return interp._setup.length_units;
}
static inline void set_length_units(Interp &interp, int value)  {
    interp._setup.length_units = static_cast<CANON_UNITS>(value);
}
static inline int get_loggingLevel (Interp &interp)  {
    return interp._setup.loggingLevel;
}
static inline void set_loggingLevel(Interp &interp, int value)  {
    interp._setup.loggingLevel = value;
}
static inline int get_motion_mode (Interp &interp)  {
    return interp._setup.motion_mode;
}
static inline void set_motion_mode(Interp &interp, int value)  {
    interp._setup.motion_mode = value;
}
static inline int get_origin_index (Interp &interp)  {
    return interp._setup.origin_index;
}
static inline void set_origin_index(Interp &interp, int value)  {
    interp._setup.origin_index = value;
}
static inline int get_plane (Interp &interp)  {
    return (int)interp._setup.plane;
}
static inline void set_plane(Interp &interp, int value)  {
    interp._setup.plane = static_cast<CANON_PLANE>(value);
}
static inline int get_random_toolchanger (Interp &interp)  {
    return interp._setup.random_toolchanger;
}
static inline void set_random_toolchanger(Interp &interp, int value)  {
    interp._setup.random_toolchanger = value;
}
static inline int get_remap_level (Interp &interp)  {
    return interp._setup.remap_level;
}
static inline void set_remap_level(Interp &interp, int value)  {
    interp._setup.remap_level = value;
}
static inline int get_retract_mode (Interp &interp)  {
    return (int)interp._setup.retract_mode;
}
static inline void set_retract_mode(Interp &interp, RETRACT_MODE value)  {
    interp._setup.retract_mode = value;
}
static inline int get_selected_pocket (Interp &interp)  {
    return interp._setup.selected_pocket;
}
static inline void set_selected_pocket(Interp &interp, int value)  {
    interp._setup.selected_pocket = value;
}
static inline int get_selected_tool (Interp &interp)  {
    return interp._setup.selected_tool;
}
static inline void set_selected_tool(Interp &interp, int value)  {
    interp._setup.selected_tool = value;
}
static inline int get_sequence_number (Interp &interp)  {
    return interp._setup.sequence_number;
}
static inline void set_sequence_number(Interp &interp, int value)  {
    interp._setup.sequence_number = value;
}
static inline int get_speed_feed_mode (Interp &interp)  {
    return interp._setup.speed_feed_mode;
}
static inline void set_speed_feed_mode(Interp &interp, int value)  {
    interp._setup.speed_feed_mode = static_cast<CANON_SPEED_FEED_MODE>(value);
}
static inline int get_spindle_mode (Interp &interp, int spindle)  {
    return (int)interp._setup.spindle_mode[spindle];
}
static inline void set_spindle_mode(Interp &interp, int spindle, SPINDLE_MODE value)  {
    interp._setup.spindle_mode[spindle] = value;
}
static inline int get_num_spindles (Interp &interp)  {
    return interp._setup.num_spindles;
}
static inline void set_num_spindles(Interp &interp, int value)  {
    interp._setup.num_spindles = value;
}
static inline int get_spindle_turning (Interp &interp, int spindle)  {
    return interp._setup.spindle_turning[spindle];
}
static inline void set_spindle_turning(Interp &interp, int spindle, int value)  {
    interp._setup.spindle_turning[spindle] = (CANON_DIRECTION)value;
}
static inline int get_stack_index (Interp &interp)  {
    return interp._setup.stack_index;
}
static inline void set_stack_index(Interp &interp, int value)  {
    interp._setup.stack_index = value;
}
static inline int get_value_returned (Interp &interp)  {
    return interp._setup.value_returned;
}
static inline void set_value_returned(Interp &interp, int value)  {
    interp._setup.value_returned = value;
}
static inline int get_current_tool(Interp &interp)  {
    return interp._setup.tool_table[0].toolno;
}
static inline void set_current_tool(Interp &interp, int value)  {
    interp._setup.tool_table[0].toolno = value;
}
static inline int get_tool_change_at_g30 (Interp &interp)  {
    return interp._setup.tool_change_at_g30;
}
static inline void set_tool_change_at_g30(Interp &interp, int value)  {
    interp._setup.tool_change_at_g30 = value;
}
static inline int get_tool_change_quill_up (Interp &interp)  {
    return interp._setup.tool_change_quill_up;
}
static inline void set_tool_change_quill_up(Interp &interp, int value)  {
    interp._setup.tool_change_quill_up = value;
}
static inline int get_tool_change_with_spindle_on(Interp &interp)  {
    return interp._setup.tool_change_with_spindle_on;
}
static inline void set_tool_change_with_spindle_on(Interp &interp, int value)  {
    interp._setup.tool_change_with_spindle_on = value;
}
static inline double get_parameter_g73_peck_clearance (Interp &interp)  {
    return interp._setup.parameter_g73_peck_clearance;
}
static inline void set_parameter_g73_peck_clearance(Interp &interp, double value)  {
    interp._setup.parameter_g73_peck_clearance = value;
}
static inline double get_parameter_g83_peck_clearance (Interp &interp)  {
    return interp._setup.parameter_g83_peck_clearance;
}
static inline void set_parameter_g83_peck_clearance(Interp &interp, double value)  {
    interp._setup.parameter_g83_peck_clearance = value;
}

BOOST_PYTHON_MODULE(interpreter) {
    using namespace boost::python;

    scope().attr("__doc__") =
        "Interpreter introspection\n"
        ;
    scope().attr("throw_exceptions") = throw_exceptions;

    BOOST_PYENUM_(InterpReturn)
            .BOOST_PYENUM_VAL(INTERP_OK)
            .BOOST_PYENUM_VAL(INTERP_EXIT)
            .BOOST_PYENUM_VAL(INTERP_EXECUTE_FINISH)
            .BOOST_PYENUM_VAL(INTERP_ENDFILE)
            .BOOST_PYENUM_VAL(INTERP_FILE_NOT_OPEN)
            .BOOST_PYENUM_VAL(INTERP_ERROR)
            .export_values();

    scope().attr("INTERP_MIN_ERROR") = (int)INTERP_MIN_ERROR;
    scope().attr("TOLERANCE_EQUAL") = TOLERANCE_EQUAL;

    BOOST_PYENUM_(DISTANCE_MODE)
            .value("MODE_ABSOLUTE", DISTANCE_MODE::ABSOLUTE)
            .value("MODE_INCREMENTAL", DISTANCE_MODE::INCREMENTAL)
            .export_values();

    BOOST_PYENUM_(RETRACT_MODE)
            .value("R_PLANE", RETRACT_MODE::R_PLANE)
            .value("OLD_Z", RETRACT_MODE::OLD_Z)
            .export_values();

    BOOST_PYENUM_(FEED_MODE)
            .value("UNITS_PER_MINUTE", FEED_MODE::UNITS_PER_MINUTE)
            .value("INVERSE_TIME", FEED_MODE::INVERSE_TIME)
            .value("UNITS_PER_REVOLUTION", FEED_MODE::UNITS_PER_REVOLUTION)
            .export_values();

    enum_<CUTTER_COMP>("CUTTER_COMP_DIRECTION")
            .value("OFF", CUTTER_COMP::OFF)
            .value("RIGHT", CUTTER_COMP::RIGHT)
            .value("LEFT", CUTTER_COMP::LEFT)
            .export_values();

    BOOST_PYENUM_(SPINDLE_MODE)
            .value("CONSTANT_RPM", SPINDLE_MODE::CONSTANT_RPM)
            .value("CONSTANT_SURFACE", SPINDLE_MODE::CONSTANT_SURFACE)
            .export_values();


    def("equal", &__equal);  // EMC's perception of equality of doubles
    def("is_near_int", &is_near_int);  // EMC's perception of closeness to an int
    def("nearest_int", &nearest_int);

    export_EmcTypes();
    export_ParamClass();
    export_Internals();
    export_Block();
    bp::class_<InterpreterException>InterpreterExceptionClass("InterpreterException",	bp::init<std::string, int, std::string>());
    InterpreterExceptionClass
	.add_property("error_message", &InterpreterException::get_error_message)
	.add_property("line_number", &InterpreterException::get_line_number)
	.add_property("line_text", &InterpreterException::get_line_text)
	;

    InterpreterExceptionType = InterpreterExceptionClass.ptr();

    bp::register_exception_translator<InterpreterException>
	(&translateInterpreterException);

    bp::class_< Interp, boost::noncopyable >("Interp",bp::no_init)

	.def("find_tool_pocket", &wrap_find_tool_pocket)
	.def("load_tool_table", &Interp::load_tool_table)
	.def("set_tool_parameters", &Interp::set_tool_parameters)


	.def("set_errormsg", &setErrorMsg)
	.def("get_errormsg", &Interp::getSavedError)
	.def("stack", &errorStack)
	.def("synch", &Interp::synch)

	// those will raise exceptions on return value < INTERP_MIN_ERROR  if throw_exceptions is set.
	.def("execute", &wrap_interp_execute_1)
	.def("execute",  &wrap_interp_execute_2)
	.def("read", &wrap_interp_read)

	// until I know better
	//.def_readwrite("remaps",  &wrap_remaps)

	.add_property("task", &get_task) // R/O
	.add_property("filename", &get_filename) // R/O
	.add_property("linetext", &get_linetext) // R/O
	.add_property("tool_offset", &get_tool_offset, &set_tool_offset)
	.add_property("arc_not_allowed", &get_arc_not_allowed, &set_arc_not_allowed)
	.add_property("cutter_comp_firstmove",
		      &get_cutter_comp_firstmove,
		      &set_cutter_comp_firstmove)
	.add_property("feed_override", &get_feed_override, &set_feed_override)
	.add_property("input_digital", &get_input_digital, &set_input_digital)
	.add_property("input_flag", &get_input_flag, &set_input_flag)
	.add_property("mdi_interrupt", &get_mdi_interrupt, &set_mdi_interrupt)
	.add_property("mist", &get_mist, &set_mist)
	.add_property("flood", &get_flood, &set_flood)
	.add_property("percent_flag", &get_percent_flag, &set_percent_flag)
	.add_property("probe_flag", &get_probe_flag, &set_probe_flag)
	.add_property("speed_override", &get_speed_override, &set_speed_override)
	.add_property("toolchange_flag", &get_toolchange_flag, &set_toolchange_flag)
	.add_property("u_current", &get_u_current, &set_u_current)
	.add_property("AA_axis_offset", &get_AA_axis_offset, &set_AA_axis_offset)
	.add_property("AA_current", &get_AA_current, &set_AA_current)
	.add_property("AA_origin_offset", &get_AA_origin_offset, &set_AA_origin_offset)
	.add_property("BB_axis_offset", &get_BB_axis_offset, &set_BB_axis_offset)
	.add_property("BB_current", &get_BB_current, &set_BB_current)
	.add_property("BB_origin_offset", &get_BB_origin_offset, &set_BB_origin_offset)
	.add_property("CC_axis_offset", &get_CC_axis_offset, &set_CC_axis_offset)
	.add_property("CC_current", &get_CC_current, &set_CC_current)
	.add_property("CC_origin_offset", &get_CC_origin_offset, &set_CC_origin_offset)
        .add_property("active_spindle", &get_active_spindle, &set_active_spindle)
	.add_property("axis_offset_x", &get_axis_offset_x, &set_axis_offset_x)
	.add_property("axis_offset_y", &get_axis_offset_y, &set_axis_offset_y)
	.add_property("axis_offset_z", &get_axis_offset_z, &set_axis_offset_z)
	.add_property("current_x", &get_current_x, &set_current_x)
	.add_property("current_y", &get_current_y, &set_current_y)
	.add_property("current_z", &get_current_z, &set_current_z)
	.add_property("cutter_comp_radius", &get_cutter_comp_radius, &set_cutter_comp_radius)
	.add_property("cycle_cc", &get_cycle_cc, &set_cycle_cc)
	.add_property("cycle_i", &get_cycle_i, &set_cycle_i)
	.add_property("cycle_il", &get_cycle_il, &set_cycle_il)
	.add_property("cycle_j", &get_cycle_j, &set_cycle_j)
	.add_property("cycle_k", &get_cycle_k, &set_cycle_k)
	.add_property("cycle_p", &get_cycle_p, &set_cycle_p)
	.add_property("cycle_q", &get_cycle_q, &set_cycle_q)
	.add_property("cycle_r", &get_cycle_r, &set_cycle_r)
	.add_property("feed_rate", &get_feed_rate, &set_feed_rate)
	.add_property("origin_offset_x", &get_origin_offset_x, &set_origin_offset_x)
	.add_property("origin_offset_y", &get_origin_offset_y, &set_origin_offset_y)
	.add_property("origin_offset_z", &get_origin_offset_z, &set_origin_offset_z)
	.add_property("program_x", &get_program_x, &set_program_x)
	.add_property("program_y", &get_program_y, &set_program_y)
	.add_property("program_z", &get_program_z, &set_program_z)
	.add_property("return_value", &get_return_value, &set_return_value)
	.add_property("rotation_xy", &get_rotation_xy, &set_rotation_xy)
	.add_property("traverse_rate", &get_traverse_rate, &set_traverse_rate)
	.add_property("u_axis_offset", &get_u_axis_offset, &set_u_axis_offset)
	.add_property("u_origin_offset", &get_u_origin_offset, &set_u_origin_offset)
	.add_property("v_axis_offset", &get_v_axis_offset, &set_v_axis_offset)
	.add_property("v_current", &get_v_current, &set_v_current)
	.add_property("v_origin_offset", &get_v_origin_offset, &set_v_origin_offset)
	.add_property("w_axis_offset", &get_w_axis_offset, &set_w_axis_offset)
	.add_property("w_current", &get_w_current, &set_w_current)
	.add_property("w_origin_offset", &get_w_origin_offset, &set_w_origin_offset)
	.add_property("a_axis_wrapped", &get_a_axis_wrapped, &set_a_axis_wrapped)
	.add_property("a_indexer_jnum", &get_a_indexer, &set_a_indexer)
	.add_property("b_axis_wrapped", &get_b_axis_wrapped, &set_b_axis_wrapped)
	.add_property("b_indexer_jnum", &get_b_indexer, &set_b_indexer)
	.add_property("c_axis_wrapped", &get_c_axis_wrapped, &set_c_axis_wrapped)
	.add_property("c_indexer_jnum", &get_c_indexer, &set_c_indexer)
	.add_property("call_level", &get_call_level, &set_call_level)
	.add_property("current_pocket", &get_current_pocket, &set_current_pocket)
	.add_property("cutter_comp_orientation",
		      &get_cutter_comp_orientation, &set_cutter_comp_orientation)
	.add_property("cutter_comp_side", &get_cutter_comp_side, &set_cutter_comp_side)
	.add_property("cycle_il_flag", &get_cycle_il_flag, &set_cycle_il_flag)
	.add_property("cycle_l", &get_cycle_l, &set_cycle_l)
	.add_property("debugmask", &get_debugmask, &set_debugmask)
	.add_property("distance_mode", &get_distance_mode, &set_distance_mode)
	.add_property("feed_mode", &get_feed_mode, &set_feed_mode)
	.add_property("ijk_distance_mode", &get_ijk_distance_mode, &set_ijk_distance_mode)
	.add_property("input_index", &get_input_index, &set_input_index)
	.add_property("length_units", &get_length_units, &set_length_units)
	.add_property("loggingLevel", &get_loggingLevel, &set_loggingLevel)
	.add_property("motion_mode", &get_motion_mode, &set_motion_mode)
	.add_property("origin_index", &get_origin_index, &set_origin_index)
	.add_property("plane", &get_plane, &set_plane)
	.add_property("random_toolchanger", &get_random_toolchanger, &set_random_toolchanger)
	.add_property("remap_level", &get_remap_level, &set_remap_level)
	.add_property("retract_mode", &get_retract_mode, &set_retract_mode)
	.add_property("selected_pocket", &get_selected_pocket, &set_selected_pocket)
	.add_property("selected_tool", &get_selected_tool, &set_selected_tool)
	.add_property("sequence_number", &get_sequence_number, &set_sequence_number)
	.add_property("speed_feed_mode", &get_speed_feed_mode, &set_speed_feed_mode)
	.add_property("spindle_mode", &get_spindle_mode, &set_spindle_mode)
	.add_property("num_spindles", &get_num_spindles, &set_num_spindles)
	.add_property("spindle_turning", &get_spindle_turning, &set_spindle_turning)
	.add_property("stack_index", &get_stack_index, &set_stack_index)
	.add_property("value_returned", &get_value_returned, &set_value_returned)

	.add_property("current_tool", &get_current_tool, &set_current_tool)
	.add_property("tool_change_at_g30", &get_tool_change_at_g30, &set_tool_change_at_g30)
	.add_property("tool_change_quill_up", &get_tool_change_quill_up, &set_tool_change_quill_up)
	.add_property("tool_change_with_spindle_on", &get_tool_change_with_spindle_on,
             &set_tool_change_with_spindle_on)
	.add_property("parameter_g73_peck_clearance", &get_parameter_g73_peck_clearance, &set_parameter_g73_peck_clearance)
    .add_property("parameter_g83_peck_clearance", &get_parameter_g83_peck_clearance, &set_parameter_g83_peck_clearance)

	.add_property( "params",
		       bp::make_function( &param_wrapper,
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))


	// _setup arrays
        .add_property(
            "speed",
            bp::make_function(
                spindle_speed_w(&spindle_speed_wrapper),
                bp::with_custodian_and_ward_postcall<0, 1>()
            )
        )

	.add_property( "active_g_codes",
		       bp::make_function( active_g_codes_w(&active_g_codes_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "active_m_codes",
		       bp::make_function( active_m_codes_w(&active_m_codes_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "active_settings",
		       bp::make_function( active_settings_w(&active_settings_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "blocks",
		       bp::make_function( blocks_w(&blocks_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "parameters",
		       bp::make_function( parameters_w(&parameters_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "tool_table",
		       bp::make_function( tool_table_w(&tool_table_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "sub_context",
		       bp::make_function( sub_context_w(&sub_context_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	;

    export_Arrays();
}
