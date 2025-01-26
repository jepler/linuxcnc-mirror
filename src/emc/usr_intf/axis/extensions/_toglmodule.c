//    Copyright 2006-2009 Jeff Epler <jepler@unpythonic.net>
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
#include <Python.h>
#include <emc/usr_intf/axis/extensions/togl.c>
static int first_time = 1;

static Tcl_Interp *get_interpreter(PyObject *tkapp) {
    PyObject *interpaddrobj = PyObject_CallMethod(tkapp, "interpaddr", NULL);
    if(interpaddrobj == NULL) { return NULL; }
    void *interpaddr = PyLong_AsVoidPtr(interpaddrobj);
    Py_DECREF(interpaddrobj);
    if(interpaddr == (void*)-1) { return NULL; }
    return (Tcl_Interp*)interpaddr;
}

PyObject *install(PyObject *s, PyObject *arg) {
    (void)s;
    Tcl_Interp *trp = get_interpreter(arg);
    if(!trp) {
        PyErr_SetString(PyExc_TypeError, "get_interpreter() returned NULL");
        return NULL;
    }
    if (Tcl_InitStubs(trp, "8.1", 0) == NULL) 
    {
        PyErr_SetString(PyExc_RuntimeError, "Tcl_InitStubs returned NULL");
        return NULL;
    }
    if (Tk_InitStubs(trp, "8.1", 0) == NULL) 
    {
        PyErr_SetString(PyExc_RuntimeError, "Tk_InitStubs returned NULL");
        return NULL;
    }
    if (Tcl_PkgPresent(trp, "Togl", TOGL_VERSION, 0)) {
        Py_INCREF(Py_None);
        return Py_None;
    }
    if (Tcl_PkgProvide(trp, "Togl", TOGL_VERSION) != TCL_OK) {
        PyErr_Format(PyExc_RuntimeError, "Tcl_PkgProvide failed: %s", Tcl_GetStringResult(trp));
        return NULL;
    }

    Tcl_CreateCommand(trp, "togl", (Tcl_CmdProc *)Togl_Cmd,
                      (ClientData) Tk_MainWindow(trp), NULL);

    if(first_time) {
        Tcl_InitHashTable(&CommandTable, TCL_STRING_KEYS);
        first_time = 0;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

PyMethodDef togl_methods[] = {
    {"install", (PyCFunction)install, METH_O, "install togl in a tkinter application"},
    {}
};

static struct PyModuleDef togl_moduledef = {
    PyModuleDef_HEAD_INIT, /* m_base */
    "_togl",               /* m_name */
    "togl extension for Tkinter", /* m_doc */
    -1,                    /* m_size */
    togl_methods,          /* m_methods */
    NULL,                  /* m_slots */
    NULL,                  /* m_traverse */
    NULL,                  /* m_clear */
    NULL,                  /* m_free */
};

PyMODINIT_FUNC PyInit__togl(void);
PyMODINIT_FUNC PyInit__togl(void)
{
    PyObject *m = PyModule_Create(&togl_moduledef);
    return m;
}
