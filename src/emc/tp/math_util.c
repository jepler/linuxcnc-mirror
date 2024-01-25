/********************************************************************
* Description: math_util.c
*
* Simpler math helper functions that seem to be missing from rtapi_math.
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include "math_util.h"

long max(long y, long x) {
    return y > x ? y : x;
}

long min(long y, long x) {
    return y < x ? y : x;
}

double signum(double x) {
    return (x > 0.0) ? 1.0 : (x < 0.0) ? -1.0 : 0.0;
}
