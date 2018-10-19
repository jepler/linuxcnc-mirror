#ifndef MATH_UTIL_H
#define MATH_UTIL_H

inline long max(long y, long x) {
    return y > x ? y : x;
}
inline long min(long y, long x) {
    return y < x ? y : x;
}

inline double signum(double x) {
    return (x > 0.0) ? 1.0 : (x < 0.0) ? -1.0 : 0.0;
}

#endif // MATH_UTIL_H
