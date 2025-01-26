//    This is a component of AXIS, a front-end for emc
//    Copyright 2004, 2005, 2006 Jeff Epler <jepler@unpythonic.net> and 
//    Chris Radek <chris@timeguy.com>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

/*

  Notes:

  NURBS
  -----
  The code in this file for nurbs calculations is from University of Palermo.
  The publications can be found at: http://wiki.linuxcnc.org/cgi-bin/wiki.pl?NURBS
  AMST08_art837759.pdf and ECME14.pdf

  1: 
  M. Leto, R. Licari, E. Lo Valvo1 , M. Piacentini:
  CAD/CAM INTEGRATION FOR NURBS PATH INTERPOLATION ON PC BASED REAL-TIME NUMERICAL CONTROL
  Proceedings of AMST 2008 Conference, 2008, pp. 223-233

  2:
  ERNESTO LO VALVO, STEFANO DRAGO:
  An Efficient NURBS Path Generator for a Open Source CNC
  Recent Advances in Mechanical Engineering (pp.173-180). WSEAS Press

  The code from University of Palermo is modified to work on planes xy, yz and zx by Joachim Franek
  */


#include <sys/time.h>

#include <Python.h>
#include <structmember.h>

#include "rs274ngc.hh"
#include "rs274ngc_interp.hh"
#include "interp_return.hh"
#include "canon.hh"
#include "config.h"		// LINELEN
#include "units.h"

int _task = 0; // control preview behaviour when remapping

char _parameter_file_name[LINELEN];

extern "C" PyObject* PyInit_interpreter(void);
extern "C" PyObject* PyInit_emccanon(void);
extern "C" struct _inittab builtin_modules[];
struct _inittab builtin_modules[] = {
    { "interpreter", PyInit_interpreter },
    { "emccanon", PyInit_emccanon },
    { NULL, NULL }
};


static PyObject *int_array(int *arr, int sz) {
    PyObject *res = PyTuple_New(sz);
    for(int i = 0; i < sz; i++) {
        PyTuple_SET_ITEM(res, i, PyLong_FromLong(arr[i]));
    }
    return res;
}

typedef struct {
    PyObject_HEAD
    double settings[ACTIVE_SETTINGS];
    int gcodes[ACTIVE_G_CODES];
    int mcodes[ACTIVE_M_CODES];
} LineCode;

static PyObject *LineCode_gcodes(LineCode *l, void *) {
    return int_array(l->gcodes, ACTIVE_G_CODES);
}
static PyObject *LineCode_mcodes(LineCode *l, void *) {
    return int_array(l->mcodes, ACTIVE_M_CODES);
}

static PyGetSetDef LineCodeGetSet[] = {
    {(char*)"gcodes", (getter)LineCode_gcodes, NULL, NULL, NULL},
    {(char*)"mcodes", (getter)LineCode_mcodes, NULL, NULL, NULL},
    {},
};

static PyMemberDef LineCodeMembers[] = {
    {(char*)"sequence_number", T_INT, offsetof(LineCode, gcodes[0]), READONLY, NULL},

    {(char*)"feed_rate", T_DOUBLE, offsetof(LineCode, settings[1]), READONLY, NULL},
    {(char*)"speed", T_DOUBLE, offsetof(LineCode, settings[2]), READONLY, NULL},
    {(char*)"motion_mode", T_INT, offsetof(LineCode, gcodes[1]), READONLY, NULL},
    {(char*)"block", T_INT, offsetof(LineCode, gcodes[2]), READONLY, NULL},
    {(char*)"plane", T_INT, offsetof(LineCode, gcodes[3]), READONLY, NULL},
    {(char*)"cutter_side", T_INT, offsetof(LineCode, gcodes[4]), READONLY, NULL},
    {(char*)"units", T_INT, offsetof(LineCode, gcodes[5]), READONLY, NULL},
    {(char*)"distance_mode", T_INT, offsetof(LineCode, gcodes[6]), READONLY, NULL},
    {(char*)"feed_mode", T_INT, offsetof(LineCode, gcodes[7]), READONLY, NULL},
    {(char*)"origin", T_INT, offsetof(LineCode, gcodes[8]), READONLY, NULL},
    {(char*)"tool_length_offset", T_INT, offsetof(LineCode, gcodes[9]), READONLY, NULL},
    {(char*)"retract_mode", T_INT, offsetof(LineCode, gcodes[10]), READONLY, NULL},
    {(char*)"path_mode", T_INT, offsetof(LineCode, gcodes[11]), READONLY, NULL},

    {(char*)"stopping", T_INT, offsetof(LineCode, mcodes[1]), READONLY, NULL},
    {(char*)"spindle", T_INT, offsetof(LineCode, mcodes[2]), READONLY, NULL},
    {(char*)"toolchange", T_INT, offsetof(LineCode, mcodes[3]), READONLY, NULL},
    {(char*)"mist", T_INT, offsetof(LineCode, mcodes[4]), READONLY, NULL},
    {(char*)"flood", T_INT, offsetof(LineCode, mcodes[5]), READONLY, NULL},
    {(char*)"overrides", T_INT, offsetof(LineCode, mcodes[6]), READONLY, NULL},
    {}
};

static PyTypeObject LineCodeType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "gcode.linecode",       /*tp_name*/
    sizeof(LineCode),       /*tp_basicsize*/
    0,                      /*tp_itemsize*/
    /* methods */
    0,                      /*tp_dealloc*/
    0,                      /*tp_print*/
    0,                      /*tp_getattr*/
    0,                      /*tp_setattr*/
    0,                      /*tp_compare*/
    0,                      /*tp_repr*/
    0,                      /*tp_as_number*/
    0,                      /*tp_as_sequence*/
    0,                      /*tp_as_mapping*/
    0,                      /*tp_hash*/
    0,                      /*tp_call*/
    0,                      /*tp_str*/
    0,                      /*tp_getattro*/
    0,                      /*tp_setattro*/
    0,                      /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT,     /*tp_flags*/
    0,                      /*tp_doc*/
    0,                      /*tp_traverse*/
    0,                      /*tp_clear*/
    0,                      /*tp_richcompare*/
    0,                      /*tp_weaklistoffset*/
    0,                      /*tp_iter*/
    0,                      /*tp_iternext*/
    0,                      /*tp_methods*/
    LineCodeMembers,     /*tp_members*/
    LineCodeGetSet,      /*tp_getset*/
    0,                      /*tp_base*/
    0,                      /*tp_dict*/
    0,                      /*tp_descr_get*/
    0,                      /*tp_descr_set*/
    0,                      /*tp_dictoffset*/
    0,                      /*tp_init*/
    0,                      /*tp_alloc*/
    PyType_GenericNew,      /*tp_new*/
    0,                      /*tp_free*/
    0,                      /*tp_is_gc*/
    0,                      /*tp_bases*/
    0,                      /*tp_mro*/
    0,                      /*tp_cache*/
    0,                      /*tp_subclasses*/
    0,                      /*tp_weaklink*/
    0,                      /*tp_del*/
    0,                      /*tp_version_tag*/
    0,                      /*tp_finalize*/
#if PY_VERSION_HEX >= 0x030800f0	// 3.8
    0,                      /*tp_vectorcall*/
#if PY_VERSION_HEX >= 0x030c00f0	// 3.12
    0,                      /*tp_watched*/
#endif
#endif
};

static PyObject *callback;
static int interp_error;
static int last_sequence_number;
static int selected_tool = 0;
static bool metric;
static double _pos_x, _pos_y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w;
EmcPose tool_offset;

static InterpBase *pinterp;

#define callmethod(o, m, f, ...) PyObject_CallMethod((o), (char*)(m), (char*)(f), ## __VA_ARGS__)

static void maybe_new_line(int sequence_number=pinterp->sequence_number());
static void maybe_new_line(int sequence_number) {
    if(!pinterp) return;
    if(interp_error) return;
    if(sequence_number == last_sequence_number)
        return;
    LineCode *new_line_code =
        (LineCode*)(PyObject_New(LineCode, &LineCodeType));
    pinterp->active_settings(new_line_code->settings);
    pinterp->active_g_codes(new_line_code->gcodes);
    pinterp->active_m_codes(new_line_code->mcodes);
    new_line_code->gcodes[0] = sequence_number;
    last_sequence_number = sequence_number;
    PyObject *result = 
        callmethod(callback, "next_line", "O", new_line_code);
    Py_DECREF(new_line_code);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

//das ist für die Vorschau
/* G_5_2/G_5_3*/
void NURBS_G5_FEED(int line_number, std::vector<NURBS_CONTROL_POINT> nurbs_control_points, unsigned int nurbs_order, CANON_PLANE plane) 
    {
    double u = 0.0;
    unsigned int n = nurbs_control_points.size() - 1;
    double umax = n - nurbs_order + 2;
    unsigned int div = nurbs_control_points.size()*15;
    std::vector<unsigned int> knot_vector = nurbs_G5_knot_vector_creator(n, nurbs_order);	
    NURBS_PLANE_POINT P1;
    while (u+umax/div < umax) {
        NURBS_PLANE_POINT P1 = nurbs_G5_point(u+umax/div,nurbs_order,nurbs_control_points,knot_vector);
        //printf("P1 X: %8.4f Y: %8.4f pos_x: %8.4f pos_y: %8.4f pos_z: %8.4f (F: %s L: %d)\n",P1.NURBS_X,P1.NURBS_Y,_pos_x,_pos_y,_pos_z,__FILE__,__LINE__);

        //STRAIGHT_FEED(line_number, P1.X,P1.Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
        if(plane==CANON_PLANE::XY) {
            //printf("XY (F: %s L: %d)\n",__FILE__,__LINE__);
            STRAIGHT_FEED(line_number, P1.NURBS_X, P1.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w); //
            }
        if(plane==CANON_PLANE::YZ) {
            //printf("YZ (F: %s L: %d)\n",__FILE__,__LINE__);
            STRAIGHT_FEED(line_number, _pos_x, P1.NURBS_X, P1.NURBS_Y, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w); //
            }
        if(plane==CANON_PLANE::XZ) {
            //printf("XZ (F: %s L: %d)\n",__FILE__,__LINE__);
            STRAIGHT_FEED(line_number, P1.NURBS_Y, _pos_y, P1.NURBS_X, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w); //
            }
        u = u + umax/div;
        } 
    P1.NURBS_X = nurbs_control_points[n].NURBS_X;
    P1.NURBS_Y = nurbs_control_points[n].NURBS_Y;
    //printf("Pn X: %8.4f Y: %8.4f pos_x: %8.4f pos_y: %8.4f pos_z: %8.4f (F: %s L: %d)\n",P1.X,P1.Y,_pos_x,_pos_y,_pos_z,__FILE__,__LINE__);
    //STRAIGHT_FEED(line_number, P1.X,P1.Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
    if(plane==CANON_PLANE::XY) {
        STRAIGHT_FEED(line_number, P1.NURBS_X, P1.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w); //
        }
    if(plane==CANON_PLANE::YZ) {
        STRAIGHT_FEED(line_number, _pos_x, P1.NURBS_X, P1.NURBS_Y, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w); //
        }
    if(plane==CANON_PLANE::XZ) {
        STRAIGHT_FEED(line_number, P1.NURBS_Y, _pos_y, P1.NURBS_X, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w); //
        }
    knot_vector.clear();
}

/* G_6_2  L_option is unused */
//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_G6_FEED(int line_number, std::vector<NURBS_G6_CONTROL_POINT> nurbs_control_points, unsigned int k, double /*feedrate*/, int /*L_option*/, CANON_PLANE plane) { // (L_option: NICU, NICL, NICC see publication from Lo Valvo and Drago)
    double u = 0.0;
    unsigned int n = nurbs_control_points.size() - 1-k;
    double umax = nurbs_control_points[n+k].NURBS_K;
    unsigned int div = (nurbs_control_points.size()-k)*15;
    std::vector<double> knot_vector = nurbs_g6_knot_vector_creator(n, k, nurbs_control_points);

    //printf("gcodemodule NURBS_G6_FEED cps: %ld k: %d L: %d fr: %f (F: %s L: %d)\n",nurbs_control_points.size(), k, L_option, feedrate, __FILE__, __LINE__);
    NURBS_PLANE_POINT P1x, P1;
    std::vector< std::vector<double> > A6;
    A6 = nurbs_G6_Nmix_creator(u+umax/div, k, n+1, knot_vector);
    P1 = nurbs_G6_pointx(knot_vector[0],k,nurbs_control_points,knot_vector,A6);	
    //printf("%.3d P1  X: %8.4f Y: %8.4f pos_x: %8.4f pos_y: %8.4f pos_z: %8.4f (F: %s L: %d)\n",line_number,P1.NURBS_X,P1.NURBS_Y,_pos_x,_pos_y,_pos_z,__FILE__,__LINE__);
    //STRAIGHT_FEED(line_number, P1.NURBS_X,P1.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
    if(plane==CANON_PLANE::XY) {
		    STRAIGHT_FEED(line_number, P1.NURBS_X, P1.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
        }
    if(plane==CANON_PLANE::YZ) {
		    STRAIGHT_FEED(line_number, _pos_x, P1.NURBS_X, P1.NURBS_Y, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
        }
    if(plane==CANON_PLANE::XZ) {
		    STRAIGHT_FEED(line_number, P1.NURBS_Y, _pos_y, P1.NURBS_X, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
        }
    u=0.1;
    while (u+umax/div < umax) {
        P1x = nurbs_G6_point_x(u+umax/div,k,nurbs_control_points,knot_vector);
        //printf("%.3d P1x X: %8.4f Y: %8.4f pos_x: %8.4f pos_y: %8.4f pos_z: %8.4f (F: %s L: %d)\n",line_number,P1x.NURBS_X,P1x.NURBS_Y,_pos_x,_pos_y,_pos_z,__FILE__,__LINE__);
        //STRAIGHT_FEED(line_number, P1x.NURBS_X,P1x.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
		if(plane==CANON_PLANE::XY) {
			    STRAIGHT_FEED(line_number, P1x.NURBS_X, P1x.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
			}
		if(plane==CANON_PLANE::YZ) {
			STRAIGHT_FEED(line_number, _pos_x, P1x.NURBS_X, P1x.NURBS_Y, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
			}
		if(plane==CANON_PLANE::XZ) {
			STRAIGHT_FEED(line_number, P1x.NURBS_Y, _pos_y, P1x.NURBS_X, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
			}
		u = u + umax/div;
    } 
    A6 = nurbs_G6_Nmix_creator (umax,  k, n+1, knot_vector);
    P1 = nurbs_G6_pointx(umax,k,nurbs_control_points,knot_vector,A6);	
    //printf("%.3d P1  X: %8.4f Y: %8.4f pos_x: %8.4f pos_y: %8.4f pos_z: %8.4f (F: %s L: %d)\n",line_number,P1.NURBS_X,P1.NURBS_Y,_pos_x,_pos_y,_pos_z,__FILE__,__LINE__);
    //STRAIGHT_FEED(line_number, P1.NURBS_X,P1.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
    if(plane==CANON_PLANE::XY) {
        STRAIGHT_FEED(line_number, P1.NURBS_X, P1.NURBS_Y, _pos_z, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
    	}
    if(plane==CANON_PLANE::YZ) {
		STRAIGHT_FEED(line_number, _pos_x, P1.NURBS_X, P1.NURBS_Y, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
    	}
    if(plane==CANON_PLANE::XZ) {
		STRAIGHT_FEED(line_number, P1.NURBS_Y, _pos_y, P1.NURBS_X, _pos_a, _pos_b, _pos_c, _pos_u, _pos_v, _pos_w);
    	}
    knot_vector.clear();
	}

//-----------------------------------------------------------------------------------------------------------------------------------------
void ARC_FEED(int line_number,
              double first_end, double second_end, double first_axis,
              double second_axis, int rotation, double axis_end_point,
              double a_position, double b_position, double c_position,
              double u_position, double v_position, double w_position) {
    // XXX: set _pos_*
    if(metric) {
        first_end /= 25.4;
        second_end /= 25.4;
        first_axis /= 25.4;
        second_axis /= 25.4;
        axis_end_point /= 25.4;
        u_position /= 25.4;
        v_position /= 25.4;
        w_position /= 25.4;
    }
    maybe_new_line(line_number);
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "arc_feed", "ffffifffffff",
                            first_end, second_end, first_axis, second_axis,
                            rotation, axis_end_point, 
                            a_position, b_position, c_position,
                            u_position, v_position, w_position);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void STRAIGHT_FEED(int line_number,
                   double x, double y, double z,
                   double a, double b, double c,
                   double u, double v, double w) {
    _pos_x=x; _pos_y=y; _pos_z=z; 
    _pos_a=a; _pos_b=b; _pos_c=c;
    _pos_u=u; _pos_v=v; _pos_w=w;
    if(metric) { x /= 25.4; y /= 25.4; z /= 25.4; u /= 25.4; v /= 25.4; w /= 25.4; }
    maybe_new_line(line_number);
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "straight_feed", "fffffffff",
                            x, y, z, a, b, c, u, v, w);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void STRAIGHT_TRAVERSE(int line_number,
                       double x, double y, double z,
                       double a, double b, double c,
                       double u, double v, double w) {
    _pos_x=x; _pos_y=y; _pos_z=z; 
    _pos_a=a; _pos_b=b; _pos_c=c;
    _pos_u=u; _pos_v=v; _pos_w=w;
    if(metric) { x /= 25.4; y /= 25.4; z /= 25.4; u /= 25.4; v /= 25.4; w /= 25.4; }
    maybe_new_line(line_number);
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "straight_traverse", "fffffffff",
                            x, y, z, a, b, c, u, v, w);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_G5X_OFFSET(int g5x_index,
                    double x, double y, double z,
                    double a, double b, double c,
                    double u, double v, double w) {
    if(metric) { x /= 25.4; y /= 25.4; z /= 25.4; u /= 25.4; v /= 25.4; w /= 25.4; }
    maybe_new_line();
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "set_g5x_offset", "ifffffffff",
                            g5x_index, x, y, z, a, b, c, u, v, w);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_G92_OFFSET(double x, double y, double z,
                    double a, double b, double c,
                    double u, double v, double w) {
    if(metric) { x /= 25.4; y /= 25.4; z /= 25.4; u /= 25.4; v /= 25.4; w /= 25.4; }
    maybe_new_line();
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "set_g92_offset", "fffffffff",
                            x, y, z, a, b, c, u, v, w);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_XY_ROTATION(double t) {
    maybe_new_line();
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "set_xy_rotation", "f", t);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
};

void USE_LENGTH_UNITS(CANON_UNITS u) { metric = u == CANON_UNITS_MM; }

void SELECT_PLANE(CANON_PLANE pl) {
    maybe_new_line();   
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "set_plane", "i", pl);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_TRAVERSE_RATE(double rate) {
    maybe_new_line();   
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "set_traverse_rate", "f", rate);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_FEED_MODE(int /*spindle*/, int /*mode*/) {
#if 0
    maybe_new_line();   
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "set_feed_mode", "i", mode);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
#endif
}

void CHANGE_TOOL() {
    maybe_new_line();
    if(interp_error) return;
    PyObject *result = 
        callmethod(callback, "change_tool", "i", selected_tool);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void CHANGE_TOOL_NUMBER(int /*pocket*/) {
    maybe_new_line();
    if(interp_error) return;
}

void RELOAD_TOOLDATA(void) {
    return;
}

/* XXX: This needs to be re-thought.  Sometimes feed rate is not in linear
 * units--e.g., it could be inverse time feed mode.  in that case, it's wrong
 * to convert from mm to inch here.  but the gcode time estimate gets inverse
 * time feed wrong anyway..
 */
void SET_FEED_RATE(double rate) {
    maybe_new_line();   
    if(interp_error) return;
    if(metric) rate /= 25.4;
    PyObject *result =
        callmethod(callback, "set_feed_rate", "f", rate);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void DWELL(double time) {
    maybe_new_line();   
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "dwell", "f", time);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void MESSAGE(char *comment) {
    maybe_new_line();   
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "message", "s", comment);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void LOG(char * /*s*/) {}
void LOGOPEN(char * /*f*/) {}
void LOGAPPEND(char * /*f*/) {}
void LOGCLOSE() {}

void COMMENT(const char *comment) {
    maybe_new_line();   
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "comment", "s", comment);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_TOOL_TABLE_ENTRY(int /*pocket*/, int /*toolno*/, EmcPose /*offset*/, double /*diameter*/,
                          double /*frontangle*/, double /*backangle*/, int /*orientation*/) {
}

void USE_TOOL_LENGTH_OFFSET(EmcPose offset) {
    tool_offset = offset;
    maybe_new_line();
    if(interp_error) return;
    if(metric) {
        offset.tran.x /= 25.4; offset.tran.y /= 25.4; offset.tran.z /= 25.4;
        offset.u /= 25.4; offset.v /= 25.4; offset.w /= 25.4; }
    PyObject *result = callmethod(callback, "tool_offset", "ddddddddd", offset.tran.x, offset.tran.y, offset.tran.z,
        offset.a, offset.b, offset.c, offset.u, offset.v, offset.w);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}

void SET_FEED_REFERENCE(double /*reference*/) { }
void SET_CUTTER_RADIUS_COMPENSATION(double /*radius*/) {}
void START_CUTTER_RADIUS_COMPENSATION(int /*direction*/) {}
void STOP_CUTTER_RADIUS_COMPENSATION(int /*direction*/) {}
void START_SPEED_FEED_SYNCH() {}
void START_SPEED_FEED_SYNCH(int /*spindle*/, double /*sync*/, bool /*vel*/) {}
void STOP_SPEED_FEED_SYNCH() {}
void START_SPINDLE_COUNTERCLOCKWISE(int /*spindle*/, int /*wait_for_at_speed*/) {}
void START_SPINDLE_CLOCKWISE(int /*spindle*/, int /*wait_for_at_speed*/) {}
void SET_SPINDLE_MODE(int /*spindle*/, double) {}
void STOP_SPINDLE_TURNING(int /*spindle*/) {}
void SET_SPINDLE_SPEED(int /*spindle*/, double /*rpm*/) {}
void ORIENT_SPINDLE(int /*spindle*/, double /*d*/, int /*i*/) {}
void WAIT_SPINDLE_ORIENT_COMPLETE(int /*s*/, double /*timeout*/) {}
void PROGRAM_STOP() {}
void PROGRAM_END() {}
void FINISH() {}
void ON_RESET() {}
void PALLET_SHUTTLE() {}
void SELECT_TOOL(int tool) {selected_tool = tool;}
void UPDATE_TAG(StateTag /*tag*/) {}
void OPTIONAL_PROGRAM_STOP() {}
int  GET_EXTERNAL_TC_FAULT() {return 0;}
int  GET_EXTERNAL_TC_REASON() {return 0;}


extern bool GET_BLOCK_DELETE(void) { 
    int bd = 0;
    if(interp_error) return 0;
    PyObject *result =
        callmethod(callback, "get_block_delete", "");
    if(result == NULL) {
        interp_error++;
    } else {
        bd = PyObject_IsTrue(result);
    }
    Py_XDECREF(result);
    return bd;
}

void CANON_ERROR(const char * /*fmt*/, ...) {};
void CLAMP_AXIS(CANON_AXIS /*axis*/) {}
bool GET_OPTIONAL_PROGRAM_STOP() { return false;}
void SET_OPTIONAL_PROGRAM_STOP(bool /*state*/) {}
void SPINDLE_RETRACT_TRAVERSE() {}
void SPINDLE_RETRACT() {}
void STOP_CUTTER_RADIUS_COMPENSATION() {}
void USE_NO_SPINDLE_FORCE() {}
void SET_BLOCK_DELETE(bool /*enabled*/) {}

void DISABLE_FEED_OVERRIDE() {}
void DISABLE_FEED_HOLD() {}
void ENABLE_FEED_HOLD() {}
void DISABLE_SPEED_OVERRIDE(int /*spindle*/) {}
void ENABLE_FEED_OVERRIDE() {}
void ENABLE_SPEED_OVERRIDE(int /*spindle*/) {}
void MIST_OFF() {}
void FLOOD_OFF() {}
void MIST_ON() {}
void FLOOD_ON() {}
void CLEAR_AUX_OUTPUT_BIT(int /*bit*/) {}
void SET_AUX_OUTPUT_BIT(int /*bit*/) {}
void SET_AUX_OUTPUT_VALUE(int /*index*/, double /*value*/) {}
void CLEAR_MOTION_OUTPUT_BIT(int /*bit*/) {}
void SET_MOTION_OUTPUT_BIT(int /*bit*/) {}
void SET_MOTION_OUTPUT_VALUE(int /*index*/, double /*value*/) {}
void TURN_PROBE_ON() {}
void TURN_PROBE_OFF() {}
int UNLOCK_ROTARY(int /*line_no*/, int /*joint_num*/) {return 0;}
int LOCK_ROTARY(int /*line_no*/, int /*joint_num*/) {return 0;}
void INTERP_ABORT(int /*reason*/, const char * /*message*/) {}

void STRAIGHT_PROBE(int line_number, 
                    double x, double y, double z, 
                    double a, double b, double c,
                    double u, double v, double w, unsigned char /*probe_type*/) {
    _pos_x=x; _pos_y=y; _pos_z=z; 
    _pos_a=a; _pos_b=b; _pos_c=c;
    _pos_u=u; _pos_v=v; _pos_w=w;
    if(metric) { x /= 25.4; y /= 25.4; z /= 25.4; u /= 25.4; v /= 25.4; w /= 25.4; }
    maybe_new_line(line_number);
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "straight_probe", "fffffffff",
                            x, y, z, a, b, c, u, v, w);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);

}
void RIGID_TAP(int line_number,
               double x, double y, double z, double /*scale*/) {
    if(metric) { x /= 25.4; y /= 25.4; z /= 25.4; }
    maybe_new_line(line_number);
    if(interp_error) return;
    PyObject *result =
        callmethod(callback, "rigid_tap", "fff",
            x, y, z);
    if(result == NULL) interp_error ++;
    Py_XDECREF(result);
}
double GET_EXTERNAL_MOTION_CONTROL_TOLERANCE() { return 0.1; }
double GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE() { return 0.1; }
double GET_EXTERNAL_PROBE_POSITION_X() { return _pos_x; }
double GET_EXTERNAL_PROBE_POSITION_Y() { return _pos_y; }
double GET_EXTERNAL_PROBE_POSITION_Z() { return _pos_z; }
double GET_EXTERNAL_PROBE_POSITION_A() { return _pos_a; }
double GET_EXTERNAL_PROBE_POSITION_B() { return _pos_b; }
double GET_EXTERNAL_PROBE_POSITION_C() { return _pos_c; }
double GET_EXTERNAL_PROBE_POSITION_U() { return _pos_u; }
double GET_EXTERNAL_PROBE_POSITION_V() { return _pos_v; }
double GET_EXTERNAL_PROBE_POSITION_W() { return _pos_w; }
double GET_EXTERNAL_PROBE_VALUE() { return 0.0; }
int GET_EXTERNAL_PROBE_TRIPPED_VALUE() { return 0; }
double GET_EXTERNAL_POSITION_X() { return _pos_x; }
double GET_EXTERNAL_POSITION_Y() { return _pos_y; }
double GET_EXTERNAL_POSITION_Z() { return _pos_z; }
double GET_EXTERNAL_POSITION_A() { return _pos_a; }
double GET_EXTERNAL_POSITION_B() { return _pos_b; }
double GET_EXTERNAL_POSITION_C() { return _pos_c; }
double GET_EXTERNAL_POSITION_U() { return _pos_u; }
double GET_EXTERNAL_POSITION_V() { return _pos_v; }
double GET_EXTERNAL_POSITION_W() { return _pos_w; }
void INIT_CANON() {}

void SET_PARAMETER_FILE_NAME(const char *name)
{
  strncpy(_parameter_file_name, name, PARAMETER_FILE_NAME_LENGTH);
}

void GET_EXTERNAL_PARAMETER_FILE_NAME(char *name, int max_size) {
    PyObject *result = PyObject_GetAttrString(callback, "parameter_file");
    if(!result) { name[0] = 0; return; }
    char *s = (char*)PyUnicode_AsUTF8(result);
    if(!s) { name[0] = 0; return; }
    memset(name, 0, max_size);
    strncpy(name, s, max_size - 1);
}
CANON_UNITS GET_EXTERNAL_LENGTH_UNIT_TYPE() { return CANON_UNITS_INCHES; }
CANON_TOOL_TABLE GET_EXTERNAL_TOOL_TABLE(int pocket) {
    CANON_TOOL_TABLE tdata = {-1,-1,{{0,0,0},0,0,0,0,0,0},0,0,0,0,{}};
    if(interp_error) return tdata;
    PyObject *result =
        callmethod(callback, "get_tool", "i", pocket);
    if(result == NULL ||
       !PyArg_ParseTuple(result, "iddddddddddddi",
             &tdata.toolno,
             &tdata.offset.tran.x, &tdata.offset.tran.y, &tdata.offset.tran.z,
             &tdata.offset.a,      &tdata.offset.b,      &tdata.offset.c,
             &tdata.offset.u,      &tdata.offset.v,      &tdata.offset.w,
             &tdata.diameter,      &tdata.frontangle,    &tdata.backangle,
             &tdata.orientation)) {
       interp_error ++;
    }
    Py_XDECREF(result);
    return tdata;
}

int GET_EXTERNAL_DIGITAL_INPUT(int /*index*/, int def) { return def; }
double GET_EXTERNAL_ANALOG_INPUT(int /*index*/, double def) { return def; }
int WAIT(int /*index*/, int /*input_type*/, int /*wait_type*/, double /*timeout*/) { return 0;}

static void user_defined_function(int num, double arg1, double arg2) {
    if(interp_error) return;
    maybe_new_line();
    PyObject *result =
        callmethod(callback, "user_defined_function",
                            "idd", num, arg1, arg2);
    if(result == NULL) interp_error++;
    Py_XDECREF(result);
}

void SET_FEED_REFERENCE(CANON_FEED_REFERENCE /*ref*/) {}
int GET_EXTERNAL_QUEUE_EMPTY() { return true; }
CANON_DIRECTION GET_EXTERNAL_SPINDLE(int) { return CANON_STOPPED; }
int GET_EXTERNAL_TOOL_SLOT() { return 0; }
int GET_EXTERNAL_SELECTED_TOOL_SLOT() { return 0; }
double GET_EXTERNAL_FEED_RATE() { return 1; }
double GET_EXTERNAL_TRAVERSE_RATE() { return 0; }
int GET_EXTERNAL_FLOOD() { return 0; }
int GET_EXTERNAL_MIST() { return 0; }
CANON_PLANE GET_EXTERNAL_PLANE() { return CANON_PLANE::XY; }
double GET_EXTERNAL_SPEED(int /*spindle*/) { return 0; }
void DISABLE_ADAPTIVE_FEED() {} 
void ENABLE_ADAPTIVE_FEED() {} 

int GET_EXTERNAL_FEED_OVERRIDE_ENABLE() {return 1;}
int GET_EXTERNAL_SPINDLE_OVERRIDE_ENABLE(int /*spindle*/) {return 1;}
int GET_EXTERNAL_ADAPTIVE_FEED_ENABLE() {return 0;}
int GET_EXTERNAL_FEED_HOLD_ENABLE() {return 1;}

int GET_EXTERNAL_OFFSET_APPLIED() {return 0;}
EmcPose GET_EXTERNAL_OFFSETS() {
    EmcPose e;
    e.tran.x = 0;
    e.tran.y = 0;
    e.tran.z = 0;
    e.a      = 0;
    e.b      = 0;
    e.c      = 0;
    e.u      = 0;
    e.v      = 0;
    e.w      = 0;
    return e;
};

int GET_EXTERNAL_AXIS_MASK() {
    if(interp_error) return 7;
    PyObject *result =
        callmethod(callback, "get_axis_mask", "");
    if(!result) { interp_error ++; return 7 /* XYZABC */; }
    if(!PyLong_Check(result)) { interp_error ++; return 7 /* XYZABC */; }
    int mask = PyLong_AsLong(result);
    Py_DECREF(result);
    return mask;
}

double GET_EXTERNAL_TOOL_LENGTH_XOFFSET() {
    return tool_offset.tran.x;
}
double GET_EXTERNAL_TOOL_LENGTH_YOFFSET() {
    return tool_offset.tran.y;
}
double GET_EXTERNAL_TOOL_LENGTH_ZOFFSET() {
    return tool_offset.tran.z;
}
double GET_EXTERNAL_TOOL_LENGTH_AOFFSET() {
    return tool_offset.a;
}
double GET_EXTERNAL_TOOL_LENGTH_BOFFSET() {
    return tool_offset.b;
}
double GET_EXTERNAL_TOOL_LENGTH_COFFSET() {
    return tool_offset.c;
}
double GET_EXTERNAL_TOOL_LENGTH_UOFFSET() {
    return tool_offset.u;
}
double GET_EXTERNAL_TOOL_LENGTH_VOFFSET() {
    return tool_offset.v;
}
double GET_EXTERNAL_TOOL_LENGTH_WOFFSET() {
    return tool_offset.w;
}

static bool PyLong_CheckAndError(const char *func, PyObject *p)  {
    if(PyLong_Check(p)) return true;
    PyErr_Format(PyExc_TypeError,
            "%s: Expected int, got %s", func, Py_TYPE(p)->tp_name);
    return false;
}

static bool PyFloat_CheckAndError(const char *func, PyObject *p)  {
    if(PyFloat_Check(p)) return true;
    PyErr_Format(PyExc_TypeError,
            "%s: Expected float, got %s", func, Py_TYPE(p)->tp_name);
    return false;
}

double GET_EXTERNAL_ANGLE_UNITS() {
    PyObject *result =
        callmethod(callback, "get_external_angular_units", "");
    if(result == NULL) interp_error++;

    double dresult = 1.0;
    if(!result || !PyFloat_CheckAndError("get_external_angle_units", result)) {
        interp_error++;
    } else {
        dresult = PyFloat_AsDouble(result);
    }
    Py_XDECREF(result);
    return dresult;
}

double GET_EXTERNAL_LENGTH_UNITS() {
    PyObject *result =
        callmethod(callback, "get_external_length_units", "");
    if(result == NULL) interp_error++;

    double dresult = 0.03937007874016;
    if(!result || !PyFloat_CheckAndError("get_external_length_units", result)) {
        interp_error++;
    } else {
        dresult = PyFloat_AsDouble(result);
    }
    Py_XDECREF(result);
    return dresult;
}

static bool check_abort() {
    PyObject *result =
        callmethod(callback, "check_abort", "");
    if(!result) return 1;
    if(PyObject_IsTrue(result)) {
        Py_DECREF(result);
        PyErr_Format(PyExc_KeyboardInterrupt, "Load aborted");
        return 1;
    }
    Py_DECREF(result);
    return 0;
}

USER_DEFINED_FUNCTION_TYPE USER_DEFINED_FUNCTION[USER_DEFINED_FUNCTION_NUM];

CANON_MOTION_MODE motion_mode;
void SET_MOTION_CONTROL_MODE(CANON_MOTION_MODE mode, double /*tolerance*/) { motion_mode = mode; }
void SET_MOTION_CONTROL_MODE(double /*tolerance*/) { }
void SET_MOTION_CONTROL_MODE(CANON_MOTION_MODE mode) { motion_mode = mode; }
CANON_MOTION_MODE GET_EXTERNAL_MOTION_CONTROL_MODE() { return motion_mode; }
void SET_NAIVECAM_TOLERANCE(double /*tolerance*/) { }

#define RESULT_OK (result == INTERP_OK || result == INTERP_EXECUTE_FINISH)
static PyObject *parse_file(PyObject * /*self*/, PyObject *args) {
    char *f;
    char *unitcode=0, *initcode=0, *interpname=0;
    PyObject *initcodes=0;
    int error_line_offset = 0;
    struct timeval t0, t1;
    int wait = 1;

    if(!PyArg_ParseTuple(args, "sOO!|s:new-parse",
            &f, &callback, &PyList_Type, &initcodes, &interpname))
    {
        initcodes = nullptr;
        PyErr_Clear();
        if(!PyArg_ParseTuple(args, "sO|sss:parse",
                &f, &callback, &unitcode, &initcode, &interpname))
            return NULL;
    }

    if(pinterp) {
        delete pinterp;
        pinterp = 0;
    }
    if(interpname && *interpname)
        pinterp = interp_from_shlib(interpname);
    if(!pinterp)
        pinterp = new Interp;

    for(int i=0; i<USER_DEFINED_FUNCTION_NUM; i++) 
        USER_DEFINED_FUNCTION[i] = user_defined_function;

    gettimeofday(&t0, NULL);

    metric=false;
    interp_error = 0;
    last_sequence_number = -1;

    _pos_x = _pos_y = _pos_z = _pos_a = _pos_b = _pos_c = 0;
    _pos_u = _pos_v = _pos_w = 0;

    pinterp->init();
    pinterp->open(f);

    maybe_new_line();

    int result = INTERP_OK;
    if(initcodes) {
        for(int i=0; i<PyList_Size(initcodes) && RESULT_OK; i++)
        {
            PyObject *item = PyList_GetItem(initcodes, i);
            if(!item) return NULL;
            const char *code = PyUnicode_AsUTF8(item);
            if(!code) return NULL;
            result = pinterp->read(code);
            if(!RESULT_OK) goto out_error;
            result = pinterp->execute();
        }
    }
    if(unitcode && RESULT_OK) {
        result = pinterp->read(unitcode);
        if(!RESULT_OK) goto out_error;
        result = pinterp->execute();
    }
    if(initcode && RESULT_OK) {
        result = pinterp->read(initcode);
        if(!RESULT_OK) goto out_error;
        result = pinterp->execute();
    }
    while(!interp_error && RESULT_OK) {
        error_line_offset = 1;
        result = pinterp->read();
        gettimeofday(&t1, NULL);
        if(t1.tv_sec > t0.tv_sec + wait) {
            if(check_abort()) return NULL;
            t0 = t1;
        }
        if(!RESULT_OK) break;
        error_line_offset = 0;
        result = pinterp->execute();
    }
out_error:
    if(pinterp)
    {
        auto interp = dynamic_cast<Interp*>(pinterp);
        if(interp) interp->_setup.use_lazy_close = false;
        pinterp->close();
    }
    if(interp_error) {
        if(!PyErr_Occurred()) {
            PyErr_Format(PyExc_RuntimeError,
                    "interp_error > 0 but no Python exception set");
        } else {
            // seems a PyErr_Ocurred(), but no exception was set ?
            // so return error info that can be caught and handled
            PyErr_Format(PyExc_RuntimeError,"parse_file interp_error");
            fprintf(stderr,"!!!%s: parse_file() f=%s\n"
                    "!!!interp_error=%d result=%d last_sequence_number=%d\n",
                    __FILE__,f,interp_error,result,last_sequence_number);
        }
        return NULL;
    }
    PyErr_Clear();
    maybe_new_line();
    if(PyErr_Occurred()) { interp_error = 1; goto out_error; }
    PyObject *retval = PyTuple_New(2);
    PyTuple_SetItem(retval, 0, PyLong_FromLong(result));
    PyTuple_SetItem(retval, 1, PyLong_FromLong(last_sequence_number + error_line_offset));
    return retval;
}


static int maxerror = -1;

static char savedError[LINELEN+1];
static PyObject *rs274_strerror(PyObject * /*s*/, PyObject *o) {
    int err;
    if(!PyArg_ParseTuple(o, "i", &err)) return nullptr;
    pinterp->error_text(err, savedError, LINELEN);
    return PyUnicode_FromString(savedError);
}

static PyObject *rs274_calc_extents(PyObject * /*self*/, PyObject *args) {
    double min_x = 9e99, min_y = 9e99, min_z = 9e99,
           min_xt = 9e99, min_yt = 9e99, min_zt = 9e99,
           max_x = -9e99, max_y = -9e99, max_z = -9e99,
           max_xt = -9e99, max_yt = -9e99, max_zt = -9e99;
    for(int i=0; i<PySequence_Length(args); i++) {
        PyObject *si = PyTuple_GetItem(args, i);
        if(!si) return NULL;
        int j;
        double xs, ys, zs, xe, ye, ze, xt, yt, zt;
        for(j=0; j<PySequence_Length(si); j++) {
            PyObject *sj = PySequence_GetItem(si, j);
            PyObject *unused;
            int r;
            if(PyTuple_Size(sj) == 4)
                r = PyArg_ParseTuple(sj,
                    "O(dddOOOOOO)(dddOOOOOO)(ddd):calc_extents item",
                    &unused,
                    &xs, &ys, &zs, &unused, &unused, &unused, &unused, &unused, &unused,
                    &xe, &ye, &ze, &unused, &unused, &unused, &unused, &unused, &unused,
                    &xt, &yt, &zt);
            else
                r = PyArg_ParseTuple(sj,
                    "O(dddOOOOOO)(dddOOOOOO)O(ddd):calc_extents item",
                    &unused,
                    &xs, &ys, &zs, &unused, &unused, &unused, &unused, &unused, &unused,
                    &xe, &ye, &ze, &unused, &unused, &unused, &unused, &unused, &unused,
                    &unused, &xt, &yt, &zt);
            Py_DECREF(sj);
            if(!r) return NULL;
            max_x = std::max(max_x, xs);
            max_y = std::max(max_y, ys);
            max_z = std::max(max_z, zs);
            min_x = std::min(min_x, xs);
            min_y = std::min(min_y, ys);
            min_z = std::min(min_z, zs);
            max_xt = std::max(max_xt, xs+xt);
            max_yt = std::max(max_yt, ys+yt);
            max_zt = std::max(max_zt, zs+zt);
            min_xt = std::min(min_xt, xs+xt);
            min_yt = std::min(min_yt, ys+yt);
            min_zt = std::min(min_zt, zs+zt);
        }
        if(j > 0) {
            max_x = std::max(max_x, xe);
            max_y = std::max(max_y, ye);
            max_z = std::max(max_z, ze);
            min_x = std::min(min_x, xe);
            min_y = std::min(min_y, ye);
            min_z = std::min(min_z, ze);
            max_xt = std::max(max_xt, xe+xt);
            max_yt = std::max(max_yt, ye+yt);
            max_zt = std::max(max_zt, ze+zt);
            min_xt = std::min(min_xt, xe+xt);
            min_yt = std::min(min_yt, ye+yt);
            min_zt = std::min(min_zt, ze+zt);
        }
    }
    return Py_BuildValue("[ddd][ddd][ddd][ddd]",
        min_x, min_y, min_z,  max_x, max_y, max_z,
        min_xt, min_yt, min_zt,  max_xt, max_yt, max_zt);
}

static bool get_attr(PyObject *o, const char *attr_name, int *v) {
    PyObject *attr = PyObject_GetAttrString(o, attr_name);
    if(attr && PyLong_CheckAndError(attr_name, attr)) {
        *v = PyLong_AsLong(attr);
        Py_DECREF(attr);
        return true;
    }
    Py_XDECREF(attr);
    return false;
}

static bool get_attr(PyObject *o, const char *attr_name, double *v) {
    PyObject *attr = PyObject_GetAttrString(o, attr_name);
    if(attr && PyFloat_CheckAndError(attr_name, attr)) {
        *v = PyFloat_AsDouble(attr);
        Py_DECREF(attr);
        return true;
    }
    Py_XDECREF(attr);
    return false;
}

static bool get_attr(PyObject *o, const char *attr_name, const char *fmt, ...) {
    bool result = false;
    va_list ap;
    va_start(ap, fmt);
    PyObject *attr = PyObject_GetAttrString(o, attr_name);
    if(attr) result = PyArg_VaParse(attr, fmt, ap);
    va_end(ap);
    Py_XDECREF(attr);
    return result;
}

static void unrotate(double &x, double &y, double c, double s) {
    double tx = x * c + y * s;
    y = -x * s + y * c;
    x = tx;
}

static void rotate(double &x, double &y, double c, double s) {
    double tx = x * c - y * s;
    y = x * s + y * c;
    x = tx;
}

static PyObject *rs274_arc_to_segments(PyObject * /*self*/, PyObject *args) {
    PyObject *canon;
    double x1, y1, cx, cy, z1, a, b, c, u, v, w;
    double o[9], n[9], g5xoffset[9], g92offset[9];
    int rot, plane;
    int X, Y, Z;
    double rotation_cos, rotation_sin;
    int max_segments = 128;

    if(!PyArg_ParseTuple(args, "Oddddiddddddd|i:arcs_to_segments",
        &canon, &x1, &y1, &cx, &cy, &rot, &z1, &a, &b, &c, &u, &v, &w, &max_segments)) return NULL;
    if(!get_attr(canon, "lo", "ddddddddd:arcs_to_segments lo", &o[0], &o[1], &o[2],
                    &o[3], &o[4], &o[5], &o[6], &o[7], &o[8]))
        return NULL;
    if(!get_attr(canon, "plane", &plane)) return NULL;
    if(!get_attr(canon, "rotation_cos", &rotation_cos)) return NULL;
    if(!get_attr(canon, "rotation_sin", &rotation_sin)) return NULL;
    if(!get_attr(canon, "g5x_offset_x", &g5xoffset[0])) return NULL;
    if(!get_attr(canon, "g5x_offset_y", &g5xoffset[1])) return NULL;
    if(!get_attr(canon, "g5x_offset_z", &g5xoffset[2])) return NULL;
    if(!get_attr(canon, "g5x_offset_a", &g5xoffset[3])) return NULL;
    if(!get_attr(canon, "g5x_offset_b", &g5xoffset[4])) return NULL;
    if(!get_attr(canon, "g5x_offset_c", &g5xoffset[5])) return NULL;
    if(!get_attr(canon, "g5x_offset_u", &g5xoffset[6])) return NULL;
    if(!get_attr(canon, "g5x_offset_v", &g5xoffset[7])) return NULL;
    if(!get_attr(canon, "g5x_offset_w", &g5xoffset[8])) return NULL;
    if(!get_attr(canon, "g92_offset_x", &g92offset[0])) return NULL;
    if(!get_attr(canon, "g92_offset_y", &g92offset[1])) return NULL;
    if(!get_attr(canon, "g92_offset_z", &g92offset[2])) return NULL;
    if(!get_attr(canon, "g92_offset_a", &g92offset[3])) return NULL;
    if(!get_attr(canon, "g92_offset_b", &g92offset[4])) return NULL;
    if(!get_attr(canon, "g92_offset_c", &g92offset[5])) return NULL;
    if(!get_attr(canon, "g92_offset_u", &g92offset[6])) return NULL;
    if(!get_attr(canon, "g92_offset_v", &g92offset[7])) return NULL;
    if(!get_attr(canon, "g92_offset_w", &g92offset[8])) return NULL;

    if(plane == 1) {
        X=0; Y=1; Z=2;
    } else if(plane == 3) {
        X=2; Y=0; Z=1;
    } else {
        X=1; Y=2; Z=0;
    }
    n[X] = x1;
    n[Y] = y1;
    n[Z] = z1;
    n[3] = a;
    n[4] = b;
    n[5] = c;
    n[6] = u;
    n[7] = v;
    n[8] = w;
    for(int ax=0; ax<9; ax++) o[ax] -= g5xoffset[ax];
    unrotate(o[0], o[1], rotation_cos, rotation_sin);
    for(int ax=0; ax<9; ax++) o[ax] -= g92offset[ax];

    double theta1 = atan2(o[Y]-cy, o[X]-cx);
    double theta2 = atan2(n[Y]-cy, n[X]-cx);
    /* Issue #1528 1/2/22 andypugh */
    /*_posemath checks for small arcs too, but uses config units */
    double len = hypot(o[X]-n[X], o[Y]-n[Y]) * (25.4 * GET_EXTERNAL_LENGTH_UNITS());
    /* If the signs of the angles differ, make them the same to allow monotonic progress through the arc */
    /* If start and end points are nearly identical, then interpret as a full turn */
    if(rot < 0) { // CW G2
        if (theta1 < theta2) theta2 -= 2*M_PI;
        if (len < CART_FUZZ) theta2 -= 2*M_PI;
    } else { // CCW G3
        if (theta1 > theta2) theta2 += 2*M_PI;
        if (len < CART_FUZZ) theta2 += 2*M_PI;
    }

    // if multi-turn, add the right number of full circles
    if(rot < -1) theta2 += 2*M_PI*(rot+1);
    if(rot > 1) theta2 += 2*M_PI*(rot-1);

    int steps = std::max(3, int(max_segments * fabs(theta1 - theta2) / M_PI));
    double rsteps = 1. / steps;
    PyObject *segs = PyList_New(steps);

    double dtheta = theta2 - theta1;
    double d[9] = {0, 0, 0, n[3]-o[3], n[4]-o[4], n[5]-o[5], n[6]-o[6], n[7]-o[7], n[8]-o[8]};
    d[Z] = n[Z] - o[Z];

    double tx = o[X] - cx, ty = o[Y] - cy, dc = cos(dtheta*rsteps), ds = sin(dtheta*rsteps);
    for(int i=0; i<steps-1; i++) {
        double f = (i+1) * rsteps;
        double p[9];
        rotate(tx, ty, dc, ds);
        p[X] = tx + cx;
        p[Y] = ty + cy;
        p[Z] = o[Z] + d[Z] * f;
        p[3] = o[3] + d[3] * f;
        p[4] = o[4] + d[4] * f;
        p[5] = o[5] + d[5] * f;
        p[6] = o[6] + d[6] * f;
        p[7] = o[7] + d[7] * f;
        p[8] = o[8] + d[8] * f;
        for(int ax=0; ax<9; ax++) p[ax] += g92offset[ax];
        rotate(p[0], p[1], rotation_cos, rotation_sin);
        for(int ax=0; ax<9; ax++) p[ax] += g5xoffset[ax];
        PyList_SET_ITEM(segs, i,
            Py_BuildValue("ddddddddd", p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]));
    }
    for(int ax=0; ax<9; ax++) n[ax] += g92offset[ax];
    rotate(n[0], n[1], rotation_cos, rotation_sin);
    for(int ax=0; ax<9; ax++) n[ax] += g5xoffset[ax];
    PyList_SET_ITEM(segs, steps-1,
        Py_BuildValue("ddddddddd", n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8]));
    return segs;
}

static PyMethodDef gcode_methods[] = {
    {"parse", (PyCFunction)parse_file, METH_VARARGS, "Parse a G-Code file"},
    {"strerror", (PyCFunction)rs274_strerror, METH_VARARGS,
        "Convert a numeric error to a string"},
    {"calc_extents", (PyCFunction)rs274_calc_extents, METH_VARARGS,
        "Calculate information about extents of gcode"},
    {"arc_to_segments", (PyCFunction)rs274_arc_to_segments, METH_VARARGS,
        "Convert an arc to straight segments"},
    {}
};

static struct PyModuleDef gcode_moduledef = {
    PyModuleDef_HEAD_INIT,                    /* m_base    */
    "gcode",                                  /* m_name    */
    "Interface to EMC rs274ngc interpreter",  /* m_doc     */
    -1,                                       /* m_size    */
    gcode_methods,                            /* m_methods */
    NULL,                                     /* m_slots   */
    NULL,                                     /* m_traverse*/
    NULL,                                     /* m_clear   */
    NULL,                                     /* m_free    */
};

PyMODINIT_FUNC PyInit_gcode(void);
PyMODINIT_FUNC PyInit_gcode(void)
{

    PyObject *m = PyModule_Create(&gcode_moduledef);
    PyType_Ready(&LineCodeType);
    PyModule_AddObject(m, "linecode", (PyObject*)&LineCodeType);
    PyObject_SetAttrString(m, "MAX_ERROR", PyLong_FromLong(maxerror));
    PyObject_SetAttrString(m, "MIN_ERROR",
            PyLong_FromLong(INTERP_MIN_ERROR));
    return m;
}
// vim:ts=8:sts=4:sw=4:et:
