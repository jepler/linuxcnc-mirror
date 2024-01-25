/**
 * @file rtapi_json5.h
 *
 * @author Robert W. Ellenberg
 * @copyright Copyright (c) 2019 All rights reserved. This project is released under the GPL v2 license.
 */

#ifndef RTAPI_JSON5_H
#define RTAPI_JSON5_H

#include "rtapi.h"  /* printing functions */
#include "posemath.h"
#include "emcpos.h"
#include "blendmath.h"

// Macro wrappers to stringify the variable name
// NOTE: callers can directly call the underlying print function if a custom field name is needed
#define print_json5_double(varname_) print_json5_double_(#varname_, (double)varname_)
#define print_json5_string(varname_) print_json5_string_(#varname_, varname_)
#define print_json5_long(varname_) print_json5_long_(#varname_, (long)varname_)
#define print_json5_unsigned(varname_) print_json5_unsigned_(#varname_, (unsigned)varname_)
#define print_json5_PmCartesian(varname_) print_json5_PmCartesian_(#varname_, varname_)
#define print_json5_EmcPose(varname_) print_json5_EmcPose_(#varname_, varname_)
#define print_json5_PmCartLine(varname_) print_json5_PmCartLine_( #varname_, varname_)
#define print_json5_PmCircle(varname_) print_json5_PmCircle_(#varname_, varname_)

// Hacky way to do realtime-logging of
// KLUDGE shove these in the header to avoid linking issues

static inline void print_json5_start_()
{
    rtapi_print("{");
}

static inline void print_json5_end_()
{
    rtapi_print("}\n");
}

static inline void print_json5_object_start_(const char *fname)
{
    rtapi_print("%s: {", fname ?: "NULL");
}

static inline void print_json5_object_end_()
{
    rtapi_print("}, ");
}

static inline void print_json5_double_(const char *varname, double value)
{
    rtapi_print("%s: %0.17g, ", varname ?: "NO_NAME", value);
}

static inline void print_json5_bool_(const char *varname, double value)
{
    rtapi_print("%s: %s, ", varname ?: "NO_NAME", value ? "true" : "false");
}

static inline void print_json5_int_(const char *varname, int value)
{
    rtapi_print("%s: %d, ", varname ?: "NO_NAME", value);
}

static inline void print_json5_long_(const char *varname, long value)
{
    rtapi_print("%s: %ld, ", varname ?: "NO_NAME", value);
}

static inline void print_json5_ll_(const char *varname, long long value)
{
    rtapi_print("%s: %lld, ", varname ?: "NO_NAME", value);
}

static inline void print_json5_unsigned_(const char *varname, unsigned value)
{
    rtapi_print("%s: %u, ", varname ?: "NO_NAME", value);
}

static inline void print_json5_string_(const char *field_name, const char *value)
{
    // TODO handle nulls gracefully with proper JSON null
    rtapi_print("%s: \"%s\", ", field_name ?: "NULL", value ?: "NULL");
}

static inline void print_json5_PmCartesian_(const char *varname, PmCartesian value)
{

    rtapi_print("%s: [%0.17g, %0.17g, %0.17g], ", varname ?: "NO_NAME", value.x, value.y, value.z);
}

static inline void print_json5_EmcPose_(const char *varname, EmcPose value)
{
    rtapi_print("%s: [%0.17g, %0.17g, %0.17g, %0.17g, %0.17g, %0.17g, %0.17g, %0.17g, %0.17g], ",
                    varname,
                    value.tran.x,
                    value.tran.y,
                    value.tran.z,
                    value.a,
                    value.b,
                    value.c,
                    value.u,
                    value.v,
                    value.w);
}

static inline void print_json5_PmCartLine_(const char *name, PmCartLine value)
{
    print_json5_object_start_(name);
    // Need manual field names here
    print_json5_PmCartesian_("start", value.start);
    print_json5_PmCartesian_("end", value.start);
    print_json5_PmCartesian_("uVec", value.uVec);
    print_json5_double_("tmag", value.tmag);
    print_json5_double_("tmag_zero", value.tmag_zero);
    print_json5_object_end_();
}

static inline void print_json5_PmCircle_(const char *name, PmCircle circle)
{
    print_json5_object_start_(name);
    print_json5_PmCartesian_("center", circle.center);
    print_json5_PmCartesian_("normal", circle.normal);
    print_json5_PmCartesian_("rTan", circle.rTan);
    print_json5_PmCartesian_("rPerp", circle.rPerp);
    print_json5_PmCartesian_("rHelix", circle.rHelix);
    print_json5_double_("radius", circle.radius);
    print_json5_double_("angle", circle.angle);
    print_json5_double_("spiral", circle.spiral);
    print_json5_object_end_();
}

static inline void print_json5_SpiralArcLengthFit_(const char *name, SpiralArcLengthFit fit)
{
    print_json5_object_start_(name);
    print_json5_double_("b0", fit.b0);
    print_json5_double_("b1", fit.b1);
    print_json5_double_("total_planar_length", fit.total_planar_length);
    print_json5_long_("spiral_in", fit.spiral_in);
    print_json5_object_end_();
}

#endif // RTAPI_JSON5_H
