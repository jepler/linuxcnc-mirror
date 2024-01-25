/********************************************************************
* Description: tc_debug.h
*
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2013-2019 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef TP_DEBUG_H
#define TP_DEBUG_H

#include "rtapi.h"  /* printing functions */
#include "posemath.h"
#include "emcpos.h"
#include "rtapi_json5.h"

/** TP debug stuff */
#ifdef TP_DEBUG
//Kludge because I didn't know any better at the time
//FIXME replace these with better names?
#define tp_debug_print(...) rtapi_print(__VA_ARGS__)
#define tp_debug_only(funcname,...) funcname(__VA_ARGS__)
#elif defined(UNIT_TEST)
#include <stdio.h>
#define tp_debug_print(...) printf(__VA_ARGS__)
#define tp_debug_only(funcname,...) funcname(__VA_ARGS__)
#else
#define tp_debug_print(...)
#define tp_debug_only(funcname,...)
#endif

#define print_json5_log_start(fname_, level_) do { \
    print_json5_start_(); \
    print_json5_string_("log_entry", #fname_); \
    print_json5_string_("log_level", #level_); \
} while (0)

#define print_json5_log_end() print_json5_end_()

// Another layer of macro wrappers to conditionally compile debug-only statements
#define tp_debug_json_double(varname_) tp_debug_only(print_json5_double, varname_)
#define tp_debug_json_string(varname_) tp_debug_only(print_json5_string, varname_)
#define tp_debug_json_long(varname_) tp_debug_only(print_json5_long, varname_)
#define tp_debug_json_unsigned(varname_) tp_debug_only(print_json5_unsigned, varname_)
#define tp_debug_json_PmCartesian(varname_) tp_debug_only(print_json5_PmCartesian, varname_)
#define tp_debug_json_EmcPose(varname_) tp_debug_only(print_json5_EmcPose, varname_)
#define tp_debug_json_PmCartLine(name_, value_) tp_debug_only(print_json5_PmCartLine_, name_, value_)
#define tp_debug_json_PmCircle(name_, value_) tp_debug_only(print_json5_PmCircle_, name_, value_)
#define tp_debug_json_log_start(fname_, level) tp_debug_only(print_json5_log_start, fname_, level)
#define tp_debug_json_log_end() tp_debug_only(print_json5_end_)

/** Use for profiling to make static function names visible */
#ifdef TP_PROFILE
#define STATIC
#else
#define STATIC static
#endif

/** "TC" debug info for inspecting trajectory planner output at each timestep */
#ifdef TC_DEBUG
#define tc_debug_print(...) rtapi_print(__VA_ARGS__)
#else
#define tc_debug_print(...) 
#endif

/** "TC" debug info for inspecting trajectory planner output at each timestep */
#ifdef TP_PEDANTIC_DEBUG
#define tc_pdebug_print(...) rtapi_print(__VA_ARGS__)
#else
#define tc_pdebug_print(...)
#endif

/** TP position data output to debug acceleration spikes */
#ifdef TP_POSEMATH_DEBUG
#define tp_posemath_debug(...) rtapi_print(__VA_ARGS__)
#else
#define tp_posemath_debug(...)
#endif

/** TP misc data logging */
#ifdef TP_INFO_LOGGING
#define tp_info_print(...) rtapi_print(__VA_ARGS__)
#else
#define tp_info_print(...)
#endif

int gdb_fake_catch(int condition);
int gdb_fake_assert(int condition);
#endif
