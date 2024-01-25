#include "posemath.h"
#include "greatest.h"
#include "rtapi.h"
#include "math.h"
#include "motion_debug.h"

/* Expand to all the definitions that need to be in
   the test runner's main file. */
GREATEST_MAIN_DEFS();

#include "mock_rtapi.inc"

#define ASSERT_PMCARTESIAN_EQ(EXP, GOT) ASSERT_PMCARTESIAN_IN_RANGE(EXP, GOT, V_FUZZ)

TEST test_pmCartCartCompare_self()
{
    PmCartesian start = {10.0, 2.0, -3.45};

    // Always self-compare positive
    ASSERT(pmCartCartCompare(&start, &start));
    PASS();
}

TEST test_pmCartCartCompare_small()
{
    PmCartesian start = {DOUBLE_FUZZ, DOUBLE_FUZZ, DOUBLE_FUZZ};

    PmCartesian large_diff=start;
    large_diff.z += 1.0;
    ASSERT(pmCartCartCompare(&start, &large_diff) == 0);

    // Always self-compare positive
    ASSERT(pmCartCartCompare(&start, &start));

    // NOTE: the initial point is <0,0,0> so that initial magnitude is small,
    // and adding the fuzz value does not round away the difference.
    PmCartesian too_small=start;
    too_small.y += nextafter(CART_FUZZ, 0.0);
    ASSERT(pmCartCartCompare(&start, &too_small) == 1);

    PmCartesian big_enough=start;
    big_enough.x += nextafter(CART_FUZZ, 1.0);
    ASSERT(pmCartCartCompare(&start, &big_enough) == 0);

    PASS();
}

TEST test_pmCartCartDot_parallel()
{
    PmCartesian zero = {0.0,0.0,0.0};
    double pmdot;
    pmCartCartDot(&zero, &zero, &pmdot);
    ASSERT_FLOAT_EQ(pmdot, 0.0);

    PmCartesian one = {1.0,-2.0,0.5};
    pmCartCartDot(&one, &one, &pmdot);
    ASSERT_FLOAT_EQ(pmdot, 5.25);
    PASS();
}

TEST test_pmCartCartDot_perpendicular()
{
    PmCartesian v1 = {cos(M_PI_4), sin(M_PI_4), 0.0};
    PmCartesian v2 = {sin(M_PI_4), -cos(M_PI_4), 0.0};
    double pmdot;
    pmCartCartDot(&v1, &v2, &pmdot);
    ASSERT_FLOAT_EQ(pmdot, 0.0);

    PASS();
}

TEST test_pmCartInfNorm()
{
    PmCartesian v1 ={3.25, -1, 2.0};
    PmCartesian v2 = v1;
    double v1_inf;
    pmCartInfNorm(&v1, &v1_inf);
    ASSERT_FLOAT_EQ(v1_inf, 3.25);

    v2.y = -4;
    double v2_inf;
    pmCartInfNorm(&v2, &v2_inf);
    ASSERT_FLOAT_EQ(v2_inf, 4);

    PASS();
}

static const PmCartesian v1 = {0.0011892247147802617, 0.019855106334368509, -0.00050476166977278807};
static const PmCartesian v2 = {-0.15453530685917072, -0.28437966952533078, -0.15744435569802101};
static const double k = -0.21253452789880767;
static const double mag_v1 = 0.019897092433684791;
static const double mag_v2 = 0.35991899456205328;
static const double mag_diff = 0.37608372040023119;
static const double dot = -0.0057506939081026628;
static const PmCartesian cross = {-0.0032696183809693856, 0.00026524021852782629, 0.0027301236187225355};
static const PmCartesian diff = {-0.15572453157395097, -0.30423477585969927, -0.15693959402824822};
static const PmCartesian sum = {-0.15334608214439047, -0.26452456319096229, -0.1579491173677938};
static const PmCartesian v1_neg = {-0.0011892247147802617, -0.019855106334368509, 0.00050476166977278807};
static const PmCartesian v1_mult_k = {-0.0002527513133214171, -0.0042198956511556368, 0.00010727928318657337};
static const PmCartesian v1_div_k = {-0.0055954424278133177, -0.093420615137988119, 0.002374963140168529};
static const PmCartesian elem_mult = {-0.00018377720622307753, -0.005646388577758019, 7.9471875878433867e-05};
static const PmCartesian elem_div = {-0.0076954887459084794, -0.069819007693164015, 0.0032059686581646868};

TEST test_PmCartesian_arithmetic()
{
    double pm_mag_v1=0.0;
    pmCartMag(&v1, &pm_mag_v1);
    ASSERT_IN_RANGE(1e-16, mag_v1, pm_mag_v1);

    double pm_mag_v2=0.0;
    pmCartMag(&v2, &pm_mag_v2);
    ASSERT_IN_RANGE(mag_v2, 1e-16, pm_mag_v2);

    double pm_mag_sq=0.0;
    pmCartMagSq(&v1, &pm_mag_sq);
    ASSERT_FLOAT_EQ(pow(mag_v1, 2), pm_mag_sq);

    double pm_disp;
    ASSERT_EQ(PM_OK, pmCartCartDisp(&v1, &v2, &pm_disp));
    ASSERT_FLOAT_EQ(mag_diff, pm_disp);

    PmCartesian pmsum;
    ASSERT_EQ(PM_OK, pmCartCartAdd(&v1, &v2, &pmsum));
    ASSERT_PMCARTESIAN_EQ(sum, pmsum);

    PmCartesian pm_diff;
    ASSERT_EQ(PM_OK, pmCartCartSub(&v2, &v1, &pm_diff));
    ASSERT_PMCARTESIAN_EQ(diff, pm_diff);

    PmCartesian pm_mult_k;
    ASSERT_EQ(PM_OK, pmCartScalMult(&v1, k, &pm_mult_k));
    ASSERT_PMCARTESIAN_EQ(v1_mult_k, pm_mult_k);

    PmCartesian pm_div_k;
    ASSERT_EQ(PM_OK, pmCartScalDiv(&v1, k, &pm_div_k));
    ASSERT_PMCARTESIAN_EQ(v1_div_k, pm_div_k);

    PmCartesian pm_neg;
    ASSERT_EQ(PM_OK, pmCartNeg(&v1, &pm_neg));
    ASSERT_PMCARTESIAN_EQ(v1_neg, pm_neg);

    double pm_dot=0.0;
    pmCartCartDot(&v1, &v2, &pm_dot);
    ASSERT_FLOAT_EQ(pm_dot, dot);

    PmCartesian pm_cross;
    pmCartCartCross(&v1, &v2, &pm_cross);
    ASSERT_PMCARTESIAN_EQ(pm_cross, cross);

    PmCartesian pm_elem_mult;
    pmCartCartMult(&v1, &v2, &pm_elem_mult);
    ASSERT_PMCARTESIAN_EQ(pm_elem_mult, elem_mult);

    PmCartesian pm_elem_div;
    pmCartCartDiv(&v1, &v2, &pm_elem_div);
    ASSERT_PMCARTESIAN_EQ(pm_elem_div, elem_div);

    PASS();
}

TEST test_pmCartUnit()
{
    PmCartesian pm_u1;
    ASSERT_FALSE(pmCartUnit(&v1, &pm_u1));

    double rx = pm_u1.x / v1.x;
    double ry = pm_u1.y / v1.y;
    double rz = pm_u1.z / v1.z;

    // Verify that unit vector is in the same direction
    ASSERT_FLOAT_EQ(rx, ry);
    ASSERT_FLOAT_EQ(ry, rz);
    ASSERT_FLOAT_EQ(rz, rx);

    double pm_umag;
    ASSERT_FALSE(pmCartMag(&pm_u1, &pm_umag));

    // And that the magnitude is close enough
    ASSERT_IN_RANGE(1.0, 1e-16, pm_umag);

    // Show that the self-modifying version is the same output
    PmCartesian ueq = v1;
    ASSERT_FALSE(pmCartUnitEq(&ueq));
    ASSERT_PMCARTESIAN_EQ(ueq, pm_u1);
    PASS();
}

TEST test_pmCartAbs()
{
    PmCartesian mixed = {-0.1, 2.0, -5.0};

    PmCartesian pm_abs;
    ASSERT_FALSE(pmCartAbs(&mixed, &pm_abs));
    ASSERT(pm_abs.x >= 0.0);
    ASSERT(pm_abs.y >= 0.0);
    ASSERT(pm_abs.z >= 0.0);
    PASS();
}

SUITE(pm_cart_basics)
{
    RUN_TEST(test_pmCartCartCompare_self);
    RUN_TEST(test_pmCartCartCompare_small);
    RUN_TEST(test_pmCartCartDot_parallel);
    RUN_TEST(test_pmCartCartDot_perpendicular);
    RUN_TEST(test_pmCartInfNorm);
    RUN_TEST(test_PmCartesian_arithmetic);
    RUN_TEST(test_pmCartUnit);
    RUN_TEST(test_pmCartAbs);
}

TEST test_pmCircleInit_simple()
{
    PmCircle c;
    PmCartesian start = {10.0,0, 0};
    PmCartesian end = start;
    PmCartesian center = {0,0,0};
    PmCartesian normal = {0,0,1};
    pmCircleInit(&c, &start, &end, &center, &normal, 0);

    ASSERT_FLOAT_EQ(c.angle, 2.0 * M_PI);
    ASSERT_FLOAT_EQ(c.radius, 10.0);

    PASS();
}

TEST test_pmCircleInit()
{
    PmCircle c;
    PmCartesian center = {2.2, 3.3, 4.4};

    const double nominal_radius = 0.5;
    const double theta1 = 0.123;
    const double theta2 = 1.456;

    PmCartesian start = {0.0, nominal_radius * cos(theta1), nominal_radius * sin(theta1)};
    pmCartCartAddEq(&start, &center);
    PmCartesian end = {0.0, nominal_radius * cos(theta2), nominal_radius * sin(theta2)};
    pmCartCartAddEq(&end, &center);

    PmCartesian normal = {1,0,0};
    pmCircleInit(&c, &start, &end, &center, &normal, 0);

    ASSERT_IN_RANGE(c.angle, theta2-theta1, CART_FUZZ);
    ASSERT_IN_RANGE(c.radius, nominal_radius, CART_FUZZ);

    PASS();
}

TEST test_pmCircleInit_spiral_short()
{
    PmCircle c;
    PmCartesian center = {0.0, 0.0, 0.0};
    PmCartesian start = {1,0, 0};
    PmCartesian end = {0, 1 + 0.3,0};

    PmCartesian normal = {0,0,1};
    pmCircleInit(&c, &start, &end, &center, &normal, 0);

    ASSERT_IN_RANGE(M_PI_2, c.angle, CART_FUZZ);
    ASSERT_IN_RANGE(0.3, c.spiral, CART_FUZZ);

    PASS();
}

TEST test_pmCircleInit_spiral_long()
{
    PmCircle c;
    PmCartesian center = {0.0, 0.0, 0.0};
    PmCartesian start = {1,0, 0};
    PmCartesian end = {-1-0.3, 0,0};

    PmCartesian normal = {0,0,1};
    pmCircleInit(&c, &start, &end, &center, &normal, 3);

    ASSERT_IN_RANGE(M_PI * 7, c.angle, CART_FUZZ);
    ASSERT_IN_RANGE(0.3, c.spiral, CART_FUZZ);

    PASS();
}

TEST test_pmCircleInit_helix()
{
    PmCircle c;
    // Assume center is zero for ease of writing the test
    const PmCartesian center = {1.0, 2.0, 3.0};

    PmCartesian start = {1,0, 0};
    PmCartesian end = {1.0,0,0.4};
    pmCartCartAddEq(&start, &center);
    pmCartCartAddEq(&end, &center);

    PmCartesian normal = {0,0,1};
    pmCircleInit(&c, &start, &end, &center, &normal, 0);

    PmCartesian const expect_rHelix = {0,0,0.4};

    ASSERT_IN_RANGE(2.0 * M_PI, c.angle, CART_FUZZ);
    ASSERT_PMCARTESIAN_EQ(expect_rHelix, c.rHelix);
    const double expect_radius = 1.0;

    // Radius should be constant for an ideal helix
    for (double angle = 0.0; angle <= M_PI * 2.0; angle += M_PI / 3.0) {
        PmCartesian p = {0};
        pmCirclePoint(&c, 0.0, &p);
        pmCartCartSubEq(&p, &center);
        // Ignore perpendicular component
        p.z=0;
        double r;
        pmCartMag(&p, &r);

        ASSERT_FLOAT_EQ(expect_radius, r);
    }

    PASS();
}


TEST test_pmCircleStretch()
{
    PmCircle c;
    PmCartesian center = {0.0, 0.0, 0.0};

    const double nominal_radius = 0.5;
    const double theta1 = M_PI_4;
    const double theta2 = 3.0 / 8.0 * M_PI;

    PmCartesian start = {nominal_radius * cos(theta1), nominal_radius * sin(theta1), 0.0};
    PmCartesian end = {nominal_radius * cos(theta2), nominal_radius * sin(theta2), 0.0};
    PmCartesian normal = {0,0,1};

    pmCircleInit(&c, &start, &end, &center, &normal, 0);

    PmCartesian sample_before;
    pmCirclePoint(&c, M_PI / 8.0, &sample_before);

    // Stretch to extend past the end point
    pmCircleStretch(&c, M_PI, 0);

    PmCartesian sample_end_stretch;
    pmCirclePoint(&c, M_PI / 8.0, &sample_end_stretch);

    ASSERT_PMCARTESIAN_EQ(sample_before, sample_end_stretch);

    // Stretch to extend past the starting point
    pmCircleStretch(&c, M_PI * 5.0/4.0, 1);

    PmCartesian sample_start_stretch;
    pmCirclePoint(&c, M_PI / 8.0 + M_PI_4, &sample_start_stretch);

    ASSERT_PMCARTESIAN_EQ(sample_before, sample_start_stretch);
    PASS();
}

SUITE(pm_circle) {
    RUN_TEST(test_pmCircleInit_simple);
    RUN_TEST(test_pmCircleInit);
    RUN_TEST(test_pmCircleInit_spiral_short);
    RUN_TEST(test_pmCircleInit_spiral_long);
    RUN_TEST(test_pmCircleInit_helix);
    RUN_TEST(test_pmCircleStretch);
}

TEST test_pmCartLineInit_simple()
{
    PmCartLine tst;
    PmCartesian start = {0,0,0};
    PmCartesian end = {1,2,3};

    pmCartLineInit(&tst, &start, &end);

    ASSERT_FLOAT_EQ(tst.tmag, sqrt(1 + 4 + 9));
    ASSERT_PMCARTESIAN_EQ(start, tst.start);
    ASSERT_PMCARTESIAN_EQ(end, tst.end);
    PASS();
}

TEST test_pmCartLineInit_ex1()
{
    PmCartLine tst;
    const PmCartesian start = {
        0.51320891452372031,
        0.3513817033641376,
        0.30203143034775765,
    };
    const PmCartesian end = {
        0.96211392505131188,
        0.20784727108932183,
        0.73266325488481165,
    };
    const double mag = 0.63840552161564434;
    pmCartLineInit(&tst, &start, &end);

    ASSERT_FLOAT_EQ(tst.tmag, mag);
    ASSERT_PMCARTESIAN_EQ(start, tst.start);
    ASSERT_PMCARTESIAN_EQ(end, tst.end);
    PASS();
}

TEST test_pmCartLineInit_ex2()
{
    PmCartLine tst;
    const PmCartesian start = {
        1.2196110251384671,
        -0.99958732370407333,
        0.41726932038630371,
    };
    const PmCartesian end = {
        0.29786615882106027,
        0.91903453415863678,
        0.7079824705329727,
    };
    const double mag = 2.1483103983640843;
    pmCartLineInit(&tst, &start, &end);

    ASSERT_FLOAT_EQ(tst.tmag, mag);
    ASSERT_PMCARTESIAN_EQ(start, tst.start);
    ASSERT_PMCARTESIAN_EQ(end, tst.end);
    PASS();
}

TEST test_pmCartLineInit_small_belowfuzz()
{
    PmCartLine tst;
    const PmCartesian start = {
        0,0,0
    };
    const double almost_fuzz = nextafter(CART_FUZZ, 0.0);
    const PmCartesian end = {
        almost_fuzz,
        -almost_fuzz,
        almost_fuzz,
    };
    pmCartLineInit(&tst, &start, &end);
    double mag=0.0;
    pmCartMag(&end, &mag);

    // Store the actual magnitude
    ASSERT_FLOAT_EQ(tst.tmag, mag);
    ASSERT(tst.tmag_zero > 0);
    ASSERT_PMCARTESIAN_EQ(start, tst.start);
    ASSERT_PMCARTESIAN_EQ(end, tst.end);
    PASS();
}

TEST test_pmCartLineInit_small_abovefuzz()
{
    PmCartLine tst;
    const PmCartesian start = {
        0,0,0
    };
    const double above_fuzz = nextafter(CART_FUZZ, 1.0);
    const PmCartesian end = {
        above_fuzz,
        -above_fuzz,
        above_fuzz,
    };
    pmCartLineInit(&tst, &start, &end);

    ASSERT(tst.tmag > 0.0);
    ASSERT(tst.tmag_zero == 0);
    ASSERT_PMCARTESIAN_EQ(start, tst.start);
    ASSERT_PMCARTESIAN_EQ(end, tst.end);
    PASS();
}

TEST test_pmCartLinePoint()
{
    PmCartLine tst;
    const PmCartesian start = {
        0.020215567195396768,
        -0.01189543243460452,
        0.025741132979321263,
    };
    const PmCartesian end = {
        4.6752714631010335,
        -2.6867637044820922,
        -3.6444762156871819,
    };

    pmCartLineInit(&tst, &start, &end);
    PmCartesian point = {0,0,0};
    pmCartLinePoint(&tst, 1.4828560169111997, &point);

    ASSERT(tst.tmag > 0.0);
    ASSERT(tst.tmag_zero == 0);
    ASSERT_PMCARTESIAN_EQ(start, tst.start);
    ASSERT_PMCARTESIAN_EQ(end, tst.end);
    PASS();
}

SUITE(pm_line) {
    RUN_TEST(test_pmCartLineInit_simple);
    RUN_TEST(test_pmCartLineInit_ex1);
    RUN_TEST(test_pmCartLineInit_ex2);
    RUN_TEST(test_pmCartLineInit_small_belowfuzz);
    RUN_TEST(test_pmCartLineInit_small_abovefuzz);
    RUN_TEST(test_pmCartLinePoint);
}

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(pm_cart_basics);
    RUN_SUITE(pm_circle);
    RUN_SUITE(pm_line);
    GREATEST_MAIN_END();
}
