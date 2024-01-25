#include "tp_debug.h"
#include "greatest.h"
#include "math.h"
#include "rtapi.h"
#include "joint_util.h"
#include "motion_debug.h"
#include "mot_priv.h"

static const struct emcmot_debug_t emcmotDebug_empty = {0};

/* Expand to all the definitions that need to be in
   the test runner's main file. */
GREATEST_MAIN_DEFS();

#include "mock_rtapi.inc"

static void emc_globals_setup(void *arg) {
    printf("-- in EMC global vars setup callback\n");
    emcmotDebug->joints[0].acc_limit = 10.0;
    emcmotDebug->joints[1].acc_limit = 5.0;
    emcmotDebug->joints[2].acc_limit = 2.0;
    (void)arg;
}

static void emc_globals_teardown(void *arg) {
    printf("-- in EMC global vars teardown callback\n");
    *emcmotDebug = emcmotDebug_empty;
    (void)arg;
}

TEST findAccelViolations_basic()
{
    const EmcPose bad_accel = {{10.01,5.01,2.01},0,0,0,0,0,0};
    const EmcPose good_accel = {{10,5,2},0,0,0,0,0,0};
    unsigned bad_axes = findAccelViolations(bad_accel);

    ASSERT_EQ(bad_axes, 1 | (1<<1) | (1<<2));
    unsigned good_axes = findAccelViolations(good_accel);

    ASSERT_FALSE(good_axes);
    PASS();
}

TEST findAccelViolations_single()
{
    const EmcPose accel = {{10.0,5.01,2.0},0,0,0,0,0,0};

    unsigned bad_axes = findAccelViolations(accel);
    ASSERT_EQ(bad_axes, (1<<1));
    PASS();
}

TEST findMinNonZero_cases()
{
    {
        PmCartesian v1 = {1,2,0};
        ASSERT_IN_RANGE(findMinNonZero(&v1), 1.0,DOUBLE_FUZZ);
    }

    {
        PmCartesian v2 = {3,1,4};
        ASSERT_IN_RANGE(findMinNonZero(&v2), 1.0,DOUBLE_FUZZ);
    }
    {
        PmCartesian v3 = {3,0,0};
        ASSERT_IN_RANGE(findMinNonZero(&v3), 3.0,DOUBLE_FUZZ);
    }
    PASS();
}

SUITE(joint_util) {
    GREATEST_SET_SETUP_CB(emc_globals_setup, NULL);
    GREATEST_SET_TEARDOWN_CB(emc_globals_teardown, NULL);
    RUN_TEST(findAccelViolations_basic);
    RUN_TEST(findAccelViolations_single);
    RUN_TEST(findMinNonZero_cases);
}

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();      /* command-line arguments, initialization. */
    RUN_SUITE(joint_util);
    GREATEST_MAIN_END();        /* display results */
}
