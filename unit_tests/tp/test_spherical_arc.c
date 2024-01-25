#include "tp_debug.h"
#include "spherical_arc.h"
#include "greatest.h"
#include "blendmath.h"
#include "tp_types.h"
#include "math.h"
#include "rtapi.h"
#include "motion_debug.h"

/* Expand to all the definitions that need to be in
   the test runner's main file. */
GREATEST_MAIN_DEFS();

#include "mock_rtapi.inc"

TEST arcInitFromPoints_simple() {
    SphericalArc arc;
    PmCartesian start = {1,2,0};
    PmCartesian end = {2,3,0};
    PmCartesian center = {1,3,0};

    int res = arcInitFromPoints(&arc, &start, &end, &center);
    ASSERT_FALSE(res);

    ASSERT_PMCARTESIAN_IN_RANGE(arc.end, end, V_FUZZ);
    ASSERT_PMCARTESIAN_IN_RANGE(arc.start, start, V_FUZZ);
    ASSERT_PMCARTESIAN_IN_RANGE(arc.center, center, V_FUZZ);

    double mag;
    pmCartMag(&arc.rStart, &mag);
    ASSERT_FLOAT_EQ(mag, 1.0);
    pmCartMag(&arc.rEnd, &mag);
    ASSERT_FLOAT_EQ(mag, 1.0);

    ASSERT_FLOAT_EQ(arc.radius, 1.0);
    ASSERT_FLOAT_EQ(arc.Sangle, 1.0);
    ASSERT_FLOAT_EQ(arc.spiral, 0.0);
    ASSERT_FLOAT_EQ(arc.angle, M_PI_2);

    PmCartesian pt={0};
    arcPoint(&arc, 0, &pt);
    ASSERT_PMCARTESIAN_IN_RANGE(arc.center, center, V_FUZZ);

    PASS();
}

/**
 * Tests the orientation of a line-arc intersection
 * @return
 */
TEST test_rayIntersectsArc() {
    PmCartesian center = {DOUBLE_FUZZ,1,0};
    PmCartesian P = {0,0,0};
    PmCartesian uVec = {1,0,0};
    int reverse_dir = 0;

    ASSERT(checkRayIntersectsArc(&center, &P, &uVec, reverse_dir));
    center.x*=-1.0;;
    ASSERT_FALSE(checkRayIntersectsArc(&center, &P, &uVec, reverse_dir));
    PASS();
}

SUITE(arc_shape) {
    RUN_TEST(arcInitFromPoints_simple);
    RUN_TEST(test_rayIntersectsArc);
}

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(arc_shape);
    GREATEST_MAIN_END();
}
