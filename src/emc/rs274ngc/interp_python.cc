/*    This is a component of LinuxCNC
 *    Copyright 2011, 2012 Michael Haberler <git@mah.priv.at>
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

#define BOOST_PYTHON_MAX_ARITY 4
#include "python_plugin.hh"
#include "interp_python.hh"
#include <boost/python/extract.hpp>
#include <boost/python/import.hpp>
#include <boost/python/str.hpp>
namespace bp = boost::python;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <exception>

#include "rs274ngc.hh"
#include "interp_return.hh"
#include "interp_internal.hh"
#include "rs274ngc_interp.hh"
#include "units.h"

extern    PythonPlugin *python_plugin;

#define PYCHK(bad, fmt, ...)				       \
    do {                                                       \
        if (bad) {                                             \
	    logPy(fmt, ## __VA_ARGS__);			       \
	    ERM(fmt, ## __VA_ARGS__);                          \
	    goto error;					       \
        }                                                      \
    } while(0)

// decode a Python exception into a string.
std::string handle_pyerror()
{
    PyObject *exc,*val,*tb;
    bp::object formatted_list, formatted;

    PyErr_Fetch(&exc,&val,&tb);
    PyErr_NormalizeException(&exc,&val,&tb);
    bp::handle<> 
        hexc(bp::allow_null(exc)), 
        hval(bp::allow_null(val)),
        htb(bp::allow_null(tb));
    if(!hval)
    {
        return bp::extract<std::string>(bp::str(hexc));
    } else{
        bp::object traceback(bp::import("traceback"));
    if (!tb) {
        bp::object format_exception_only(traceback.attr("format_exception_only"));
	formatted_list = format_exception_only(hexc,hval);
    } else {
        bp::object format_exception(traceback.attr("format_exception"));
	formatted_list = format_exception(hexc,hval,htb);
    }
    formatted = bp::str("\n").join(formatted_list);
    return bp::extract<std::string>(formatted);
    }
}

int Interp::py_reload()
{
    if (PYUSABLE) {
	CHKS((python_plugin->initialize() == PLUGIN_EXCEPTION),
	     "py_reload:\n%s",  python_plugin->last_exception().c_str());
    }
    return INTERP_OK;
}

// determine whether [module.]funcname is callable
bool Interp::is_pycallable(setup_pointer /*settings*/,
			   const char *module,
			   const char *funcname)
{
    if (!PYUSABLE)
      return false;

    return python_plugin->is_callable(module,funcname);
}

// all parameters to/results from Python calls go through the callframe, which looks a bit awkward
// the reason is not to expose boost.python through the interpreter public interface
int Interp::pycall(setup_pointer /*settings*/,
		   context_pointer frame,
		   const char *module,
		   const char *funcname,
		   int calltype)
{
    bp::object retval, function;
    std::string msg;
    bool py_exception = false;
    int status = INTERP_OK;
    PyObject *res_str;

    if (_setup.loggingLevel > 4)
	logPy("pycall(%s.%s) \n", module ? module : "", funcname);

    CHKS(!PYUSABLE, "pycall(%s): Python plugin not initialized",funcname);
    frame->pystuff.impl->py_return_type = 0;

    switch (calltype) {
    case PY_EXECUTE: // just run a string
	python_plugin->run_string(funcname, retval);
	CHKS(python_plugin->plugin_status() == PLUGIN_EXCEPTION,
	     "run_string(%s):\n%s", funcname,
	     python_plugin->last_exception().c_str());
	break;
    // default:
    // 	switch (frame->entry_at) {   //FIXTHIS terminally ugly
    case PY_FINISH_OWORDCALL:
    case PY_FINISH_PROLOG:
    case PY_FINISH_BODY:
    case PY_FINISH_EPILOG:
	logPy("pycall: call generator.next()" );
	
	// check inputs here, since _read() may not be called
	read_inputs(&_setup);
	// handler continuation if a generator was used
	try {
	    retval = frame->pystuff.impl->generator_next();
	}
	catch (bp::error_already_set &) {
	    if (PyErr_Occurred()) {
		// StopIteration is raised when the generator executes 'return'
		// instead of another 'yield INTERP_EXECUTE_FINISH
		// Technically this means a normal end of the handler and hence we 
		// treat it as INTERP_OK indicating this handler is now done
		if (PyErr_ExceptionMatches(PyExc_StopIteration)) {
		    frame->pystuff.impl->py_return_type = RET_STOPITERATION;
		    bp::handle_exception();
		    PyErr_Clear();
		    logPy("pycall: call generator - StopIteration exception");
		    return INTERP_OK;
		} else  {
		    msg = handle_pyerror();
		    bp::handle_exception();
		    PyErr_Clear();
		    logPy("pycall: call generator - exception: %s",msg.c_str());
		    
		    ERS("exception during generator call: %s", msg.c_str());
		}
	    } else 
		Error("calling generator: duh");
	}
	break;
    default:
	python_plugin->call(module,funcname, frame->pystuff.impl->tupleargs,frame->pystuff.impl->kwargs,retval);
	CHKS(python_plugin->plugin_status() == PLUGIN_EXCEPTION,
	     "pycall(%s):\n%s", funcname,
	     python_plugin->last_exception().c_str());
    }

    try {
	status = INTERP_OK;

	switch  (calltype) {
	case PY_OWORDCALL:
	case PY_PROLOG:
	case PY_BODY:
	case PY_EPILOG:
	case PY_FINISH_OWORDCALL:
	case PY_FINISH_PROLOG:
	case PY_FINISH_BODY:
	case PY_FINISH_EPILOG:

	    // these may return values in several flavours:
	    // - an int (INTERP_OK, INTERP_ERROR, INTERP_EXECUTE_FINISH...)
	    // - a double (Python oword subroutine)
	    // - a generator object, in case the handler contained a yield statement
	    if (retval.ptr() != Py_None) {
		if (PyGen_Check(retval.ptr()))  {

		    // a generator was returned. This must have been the first time call to a handler
		    // which contains a yield. Extract next() method.
		    frame->pystuff.impl->generator_next = bp::getattr(retval, "__next__");
		    // and  call it for the first time.
		    // Expect execution up to first 'yield INTERP_EXECUTE_FINISH'.
		    frame->pystuff.impl->py_returned_int = bp::extract<int>(frame->pystuff.impl->generator_next());
		    frame->pystuff.impl->py_return_type = RET_YIELD;
        } else if (PyUnicode_Check(retval.ptr())) {
		    // returning a string sets the interpreter error message and aborts
		    char *msg = bp::extract<char *>(retval);
		    ERM("%s", msg);
		    frame->pystuff.impl->py_return_type = RET_ERRORMSG;
		    status = INTERP_ERROR;
		} else if (PyLong_Check(retval.ptr())) {  
		    frame->pystuff.impl->py_returned_int = bp::extract<int>(retval);
		    frame->pystuff.impl->py_return_type = RET_INT;
		    logPy("Python call %s.%s returned int: %d", module, funcname, frame->pystuff.impl->py_returned_int);
		} else if (PyFloat_Check(retval.ptr())) { 
		    frame->pystuff.impl->py_returned_double = bp::extract<double>(retval);
		    frame->pystuff.impl->py_return_type = RET_DOUBLE;
		    logPy("Python call %s.%s returned float: %f", module, funcname, frame->pystuff.impl->py_returned_double);
		} else {
		    // not a generator, int, or float - strange
		    PyObject *res_str = PyObject_Str(retval.ptr());
		    Py_XDECREF(res_str);
		    ERM("Python call %s.%s returned '%s' - expected generator, int, or float value, got %s",
			module, funcname,
			PyUnicode_AsUTF8(res_str),
			Py_TYPE(retval.ptr())->tp_name);
		    status = INTERP_ERROR;
		}
	    } else {
		logPy("call: O <%s> call returned None",funcname);
		frame->pystuff.impl->py_return_type = RET_NONE;
	    }
	    break;

	case PY_INTERNAL:
	case PY_PLUGIN_CALL:
	    // a plain int (INTERP_OK, INTERP_ERROR, INTERP_EXECUTE_FINISH...) is expected
	    // must have returned an int
	    if ((retval.ptr() != Py_None) &&
		(PyLong_Check(retval.ptr()))) {

// FIXME check new return value convention
		status = frame->pystuff.impl->py_returned_int = bp::extract<int>(retval);
		frame->pystuff.impl->py_return_type = RET_INT;
		logPy("pycall(%s):  PY_INTERNAL/PY_PLUGIN_CALL: return code=%d", funcname,status);
	    } else {
		logPy("pycall(%s):  PY_INTERNAL: expected an int return code", funcname);
		res_str = PyObject_Str(retval.ptr());
		ERM("Python internal function '%s' expected tuple or int return value, got '%s' (%s)",
		    funcname,
		    PyUnicode_AsUTF8(res_str),
		    Py_TYPE(retval.ptr())->tp_name);
		Py_XDECREF(res_str);
		status = INTERP_ERROR;
	    }
	    break;
	case PY_EXECUTE:
	    break;

	default: ;
	}
    }
    catch (bp::error_already_set &) {
	if (PyErr_Occurred()) {
	    msg = handle_pyerror();
	}
	py_exception = true;
	bp::handle_exception();
	PyErr_Clear();
    }
    if (py_exception) {
	ERM("pycall: %s.%s:\n%s", module ? module:"", funcname, msg.c_str());
	status = INTERP_ERROR;
    }
    return status;
}

// called by  (py, ....) or ';py,...' comments
int Interp::py_execute(const char *cmd, bool as_file)
{
    bp::object retval;

    logPy("py_execute(%s)",cmd);

    CHKS(!PYUSABLE, "py_execute(%s): Python plugin not initialized",cmd);

    python_plugin->run_string(cmd, retval, as_file);
    CHKS((python_plugin->plugin_status() == PLUGIN_EXCEPTION),
	 "py_execute(%s)%s:\n%s", cmd,
	 as_file ? " as file" : "", python_plugin->last_exception().c_str());
    return INTERP_OK;
}

pycontext::pycontext() : impl(new pycontext_impl) {}
pycontext::~pycontext() { delete impl; }
pycontext::pycontext(const pycontext &other)
    : impl(new pycontext_impl(*other.impl)) {}
pycontext &pycontext::operator=(const pycontext &other) {
    delete impl;
    impl = new pycontext_impl(*other.impl);
    return *this;
}
