/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4.  Built only with: make TEST=1
 *  Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01, ...
 *
 *  Run with the GY-511 wired to Grove 2 and the car still and level on
 *  the bench (the self-test calibrates at rest).  Without the GY-511,
 *  S4-U05 and the checks that need sensor data FAIL by design; S4-U04
 *  then shows the task still reaches a defined state.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

#include "imu_task.h"
#include "imu_logic.h"
#include "imu_fixmath.h"

#define WAIT_STEP_MS        100
#define SELFTEST_WAIT_MS    8000    /* 3 tries x (1 s + 1 s) and margin  */
#define RATE_WINDOW_MS      1000
#define CYCLES_MIN          90u     /* in 1 s at 100 Hz                  */
#define CYCLES_MAX          110u
#define NFR20_US            1000u
#define BUS_US_MAX          2000u   /* elapsed, so pre-emption counts */
#define LEVEL_CDEG          200     /* bench "level": within 2 deg       */

static imu_ctx_t g_test_ctx;        /* 4.5 KB: static, not on the stack  */

static bool
logic_tilt_ok(void)
{
    imu_in_t in;
    imu_out_t out;
    int i;
    int c;

    imu_logic_init(&g_test_ctx, false);
    for (c = 0; c < 50; c++)
    {
        in.n_acc = 4u;
        in.b_mag = false;
        in.b_motors = false;
        for (i = 0; i < 4; i++)
        {
            in.acc[i][0] = 695;     /* 4000 x sin 10 deg                 */
            in.acc[i][1] = 0;
            in.acc[i][2] = 3939;    /* 4000 x cos 10 deg                 */
        }
        imu_logic_step(&g_test_ctx, &in, &out);
    }
    return imu_abs(g_test_ctx.view.pitch_cdeg - 1000) <= 20;
}

EXPORT void
subsystem4_test(void)
{
    imu_task_dbg_t d0;
    imu_task_dbg_t d1;
    s4_tlm_t tlm;
    INT waited = 0;
    ER er;

    TEST_INIT();

    /* S4-U01 the IMU task runs every 10 ms */
    imu_task_debug(&d0);
    (void)tk_dly_tsk(RATE_WINDOW_MS);
    imu_task_debug(&d1);
    TEST_CHECK("S4-U01", ((d1.cycles - d0.cycles) >= CYCLES_MIN) &&
               ((d1.cycles - d0.cycles) <= CYCLES_MAX),
               "IMU task cycles at 100 Hz");

    /* S4-U02 integer maths on the Cortex-M0+ */
    TEST_CHECK("S4-U02", (imu_atan2_cdeg(1000, 1000) == 4500) &&
               (imu_atan2_cdeg(-1, -1000) == -17994) &&
               (imu_isqrt(1000000u) == 1000u), "atan2 and isqrt on target");

    /* S4-U03 processing on target: 10 deg nose up reads 10 deg */
    TEST_CHECK("S4-U03", logic_tilt_ok(), "logic reads a 10 deg tilt");

    /* S4-U04 the self-test ends in a defined state */
    imu_task_debug(&d1);
    while ((IMU_MODE_SELFTEST == d1.mode) && (waited < SELFTEST_WAIT_MS))
    {
        (void)tk_dly_tsk(WAIT_STEP_MS);
        waited += WAIT_STEP_MS;
        imu_task_debug(&d1);
    }
    TEST_CHECK("S4-U04", IMU_MODE_SELFTEST != d1.mode,
               "self-test finished (pass or fault) within 8 s");

    /* S4-U05 the GY-511 passes: ID, settings read back, 1 g at rest */
    TEST_CHECK("S4-U05", d1.b_selftest_ok && d1.b_ready_sent &&
               d1.b_calibrated, "GY-511 (LSM303D at 0x1D or 0x1E) self-test "
               "passed, ready sent");

    /* S4-U06 IF-07 snapshot agrees with the task's mode */
    er = s4_get_tlm(&tlm);
    TEST_CHECK("S4-U06", (E_OK == er) &&
               (((IMU_MODE_FAULT == d1.mode) &&
                 (S4_IMU_FAULT == tlm.imu_status)) ||
                ((IMU_MODE_MONITOR == d1.mode) &&
                 (S4_IMU_OK == tlm.imu_status)) ||
                ((IMU_MODE_IDLE == d1.mode) &&
                 (S4_IMU_IDLE == tlm.imu_status))),
               "s4_get_tlm status matches the task");

    /* S4-U07..U10 two seconds of monitoring */
    imu_task_debug(&d0);
    (void)tk_dly_tsk(2 * RATE_WINDOW_MS);
    imu_task_debug(&d1);
    TEST_CHECK("S4-U07", (IMU_MODE_MONITOR == d1.mode) &&
               (d1.read_fails == d0.read_fails), "2 s of reads, none failed");
    TEST_CHECK("S4-U08", (d1.logic_us_max > 0u) &&
               (d1.logic_us_max < NFR20_US),
               "NFR20: processing under 1 ms per cycle");
    TEST_CHECK("S4-U09", (d1.bus_us_max > 0u) && (d1.bus_us_max < BUS_US_MAX),
               "I2C time per cycle under 2 ms");
    TEST_CHECK("S4-U10", d1.missed == d0.missed, "no 10 ms cycle missed");
    tm_printf((UB *)"[TEST] S4 info: LSM303D at 0x%02x, |g| %d mg, processing "
              "max %u us, I2C max %u us per cycle, %u read fails, %u missed "
              "cycles\n", (unsigned int)d1.addr,
              (int)d1.g_mg, (unsigned int)d1.logic_us_max,
              (unsigned int)d1.bus_us_max, (unsigned int)d1.read_fails,
              (unsigned int)d1.missed);

    /* S4-U11 on the bench: level and still after calibration */
    er = s4_get_tlm(&tlm);
    TEST_CHECK("S4-U11", (E_OK == er) &&
               (tlm.pitch_half_deg <= (LEVEL_CDEG / 50)) &&
               (tlm.roll_half_deg <= (LEVEL_CDEG / 50)) &&
               ((unsigned int)S4_CLASS_STATIONARY == tlm.motion_class),
               "car on the bench reads level and Stationary");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
