/*
 *  imu_task.c
 *  IMU task (design review section 6.4): owns the GY-511 on I2C1, runs
 *  every 10 ms from its cyclic handler, and turns samples into the IF-07
 *  snapshot, IF-08 telemetry events and IF-01 control events.
 *
 *  SELF-TEST --pass--> IDLE <--SENSORS_ENABLED--> MONITOR
 *  SELF-TEST --fails 3x--> FAULT;  MONITOR --5 failed reads--> FAULT
 *  MONITOR --1 s mean |a| impossible for a car--> FAULT
 *  FAULT --every 1 s, sensor answers--> SELF-TEST (runtime)
 *  Any state --SM_FLG_INITIALISING rising--> SELF-TEST (initial)
 *
 *  The self-test calibrates on 1 s at rest, standing level (within
 *  IMU_ST_LEVEL_CDEG of the board's attitude on level ground).  If the
 *  car is not still or not level then, the sensor still passes (it
 *  works) and MONITOR calibrates on the first still, level second while
 *  the motors are off.  2026-11-23: the car is touched in the seconds
 *  after boot or imu-zero, and the old 3 tries in 5 s failed every
 *  time, although it then sat still.
 *
 *  Wrong data from a chip that answers (not 1 g at rest, or impossible)
 *  makes the next attempt reload the chip's factory trimming (CTRL0
 *  BOOT) first, and always run the 1 s average.  On 2026-11-23 a chip
 *  left powered read a steady 5156 mg until a power cycle; BOOT is the
 *  software stand-in for that power cycle.
 *
 *  Only a self-test started by INITIALISING (or at boot) may send
 *  SM_EVT_SELFTEST_FAIL, and only with S4_CFG_REPORT_ONLY 0.  A runtime
 *  re-test after a lost sensor fails back to FAULT quietly (FR27: losing
 *  the IMU during a run never stops the car).  The self-test reads the
 *  sensor although SENSORS_ENABLED is clear (NFR14 exception for FR20).
 *
 *  Kernel objects: this task, its 10 ms cyclic handler, one TA_INHERIT
 *  mutex for the snapshot (held only while copying, NFR19).  Control
 *  events are posted with sm_post_event (never blocks) and retried next
 *  cycle if the queue is full.
 */

#include "imu_task.h"
#include "../demo_config.h"
#if ENCODER_FREE_DEMO
#include "demo_imu.h"
#include "demo_pickup.h"
#endif
#include "subsystem4.h"
#include "imu_cfg.h"
#include "imu_console.h"
#include "imu_logic.h"
#include "imu_params.h"
#include "imu_fixmath.h"
#include "drv_imu.h"
#include "../subsystem1/state_machine.h"
#include "../subsystem1/networking.h"

/* s4_motion_class_t and imu_class_t must agree (telemetry numbering) */
typedef char imu_class_check_t[((int)S4_CLASS_SUDDEN_IMPACT ==
                                 (int)IMU_CLASS_SUDDEN_IMPACT) ? 1 : -1];

/* IMU_FAIL_* continue imu_cal_result_t: one reason code for the console */
typedef char imu_fail_check_t[(IMU_FAIL_NOT_FOUND ==
                               ((unsigned int)IMU_CAL_IMPOSSIBLE + 1u)) ?
                              1 : -1];

#define ST_CYCLES           100u    /* 1 s at rest (FR20)                */
#define ST_TRIES            3u      /* then "self-test failed"           */
#define ST_MIN_SAMPLES      360u    /* of about 400 at 400 Hz            */
#define RELOAD_CYCLES       5u      /* after CTRL0 BOOT, before set-up   */
#define SANE_CYCLES         100u    /* MONITOR: |a| checked every 1 s    */
#define FAIL_LIMIT          5u      /* failed reads in a row -> FAULT    */
#define RETRY_CYCLES        100u    /* FAULT retry every 1 s             */
#define CONSOLE_CYCLES      25u     /* bench readout 4 times a second    */
#define PEND_LEN            8u      /* control events awaiting the queue */

#define HALF_DEG_CDEG       50
#define DEG_CDEG            100
#define HALF_DEG_MAX        180
#define HEADING_MAX_DEG     360
#define RATE_MAX_DPS        511

typedef struct
{
    uint8_t  code;              /* sm_evt_code_t                         */
    uint16_t arg;
} pend_t;

static ID       g_tid_imu;
static ID       g_cyc_imu;
static ID       g_mtx_tlm;
static s4_tlm_t g_tlm;          /* IF-07 snapshot, under g_mtx_tlm       */
static imu_task_dbg_t g_dbg_pub; /* copy of g_dbg, under g_mtx_tlm        */

/* Task-private state (only the IMU task touches it) */
static imu_task_dbg_t g_dbg;
static imu_ctx_t    g_ctx;
static imu_cal_t    g_cal;
static imu_cal_t    g_sane;         /* MONITOR: the current 1 s of data  */
static uint32_t     g_sane_cycles;
static bool         g_b_tilt_said;  /* "not standing level" printed      */
static bool         g_b_bad_data;   /* last 1 s mean wrong: reload chip  */
static bool         g_b_reloaded;   /* ... done in the current attempt   */
static imu_align_t  g_align;
static uint8_t      g_mode;
static bool         g_b_initial;    /* this self-test came from INIT     */
static uint8_t      g_tries;
static uint32_t     g_st_cycle;     /* cycles into the current attempt   */
static uint32_t     g_wait;         /* cycles until the next attempt     */
static uint32_t     g_fail_run;
static uint32_t     g_prev_flags;
static uint32_t     g_now_ms;
static pend_t       g_pend[PEND_LEN];
static uint8_t      g_pend_head;
static uint8_t      g_pend_count;
static bool         g_b_fault_reported;

/* Zero values (kernel headers and <string.h> clash over size_t) */
static imu_in_t const g_in_zero;
static s4_tlm_t const g_tlm_zero;

#if ENCODER_FREE_DEMO
static ID g_demo_request;
static demo_imu_t g_demo, g_demo_pub;
static demo_pickup_t g_pickup;
static uint32_t wall_ms(void){SYSTIM t;tk_get_otm(&t);return t.lo;}
ER demo_imu_read(demo_imu_t *out){ER e=tk_loc_mtx(g_mtx_tlm,TMO_POL);if(e==E_OK){*out=g_demo_pub;tk_unl_mtx(g_mtx_tlm);}return e;}
bool demo_imu_ready(void){demo_imu_t v;return demo_imu_read(&v)==E_OK&&v.ready&&!v.latched&&wall_ms()-v.time_ms<=100u;}
bool demo_imu_latched(void){demo_imu_t v;return demo_imu_read(&v)==E_OK&&v.latched;}
ER demo_imu_zero(void){if(SM_STATUS_STATE(sm_status_word())!=SM_STATE_STANDBY)return E_OBJ;return tk_set_flg(g_demo_request,1u);}
#endif
/* ---- Events ------------------------------------------------------- */

static void
post_sm(sm_evt_code_t code, uint16_t arg)
{
    if (g_pend_count >= PEND_LEN)
    {
        g_dbg.sm_posts_lost++;
        return;
    }
    g_pend[(g_pend_head + g_pend_count) % PEND_LEN].code = (uint8_t)code;
    g_pend[(g_pend_head + g_pend_count) % PEND_LEN].arg = arg;
    g_pend_count++;
}

/* Send what the State Machine's queue will take; keep the rest (E_QOVR) */
static void
flush_sm(void)
{
    while (0u != g_pend_count)
    {
        pend_t const *p = &g_pend[g_pend_head];
        ER const er = sm_post_event((sm_evt_code_t)p->code, p->arg);

        if (E_QOVR == er)
        {
            break;                      /* full: try again next cycle   */
        }
        if (E_OK != er)
        {
            g_dbg.sm_posts_lost++;
        }
        g_pend_head = (uint8_t)((g_pend_head + 1u) % PEND_LEN);
        g_pend_count--;
    }
}

/* ---- Reading the sensor ------------------------------------------- */

/* One cycle of samples, mapped to the body frame (and aligned once the
   self-test has calibrated).  false if a read failed. */
static bool
read_cycle(imu_in_t *p_in, bool b_align)
{
    drv_imu_raw_t raw[IMU_FIFO_MAX];
    drv_imu_raw_t mag;
    uint32_t const t0 = drv_imu_now_us();
    uint32_t bus;
    uint8_t n = 0u;
    uint8_t i;
    bool b_ovr;
    bool b_ok;

    *p_in = g_in_zero;
    b_ok = (E_OK == drv_imu_read_acc(raw, (uint8_t)IMU_FIFO_MAX, &n, &b_ovr));
    if (b_ok && (0u == (g_dbg.cycles & 1u)))
    {
        b_ok = (E_OK == drv_imu_read_mag(&mag));
        if (b_ok)
        {
            int16_t body[3];

            p_in->b_mag = imu_map_mag(mag.v, body);
            imu_align_mag(&g_align, body, p_in->mag);
        }
    }
    bus = drv_imu_now_us() - t0;
    g_dbg.bus_us_max = (bus > g_dbg.bus_us_max) ? bus : g_dbg.bus_us_max;
    if (!b_ok)
    {
        return false;
    }

    for (i = 0u; i < n; i++)
    {
        int16_t body[3];

        imu_map_acc(raw[i].v, body);
        if (b_align)
        {
            imu_align_acc(&g_align, body, p_in->acc[i]);
        }
        else
        {
            p_in->acc[i][0] = body[0];
            p_in->acc[i][1] = body[1];
            p_in->acc[i][2] = body[2];
        }
    }
    p_in->n_acc = n;
    return true;
}

/* ---- Modes -------------------------------------------------------- */

static void
enter_mode(imu_mode_t mode)
{
    if ((IMU_MODE_MONITOR == mode) && (IMU_MODE_MONITOR != g_mode))
    {
        imu_logic_restart(&g_ctx);
        imu_cal_reset(&g_sane);
        g_sane_cycles = 0u;
        g_b_tilt_said = false;
    }
    g_mode = (uint8_t)mode;
}

static void
start_selftest(bool b_initial)
{
    g_b_initial = b_initial;
    g_tries = 0u;
    g_st_cycle = 0u;
    g_wait = 0u;
    g_b_reloaded = false;
    enter_mode(IMU_MODE_SELFTEST);
}

/* reason: imu_cal_result_t or IMU_FAIL_* (printed; er only for the bus) */
static void
enter_fault(ER er, uint8_t reason)
{
    enter_mode(IMU_MODE_FAULT);
    g_wait = RETRY_CYCLES;
    g_fail_run = 0u;
    if (!g_b_fault_reported)
    {
        /* Once per loss, not on every failed retry */
        g_b_fault_reported = true;
        g_dbg.faults++;
        (void)net_post_event(NET_EVT_IMU_FAULT, 0u);
        post_sm(SM_EVT_FAULT, (uint16_t)SM_FAULT_IMU);  /* non-fatal, FR27 */
        imu_console_fault(er, reason, g_now_ms);
    }
}

static void
selftest_passed(uint32_t flags)
{
    g_dbg.b_selftest_ok = true;
    g_b_fault_reported = false;
    g_b_bad_data = false;
    if (g_b_initial)
    {
        post_sm(SM_EVT_READY, (uint16_t)SM_SUBSYS_IMU);
        g_dbg.b_ready_sent = true;
    }
    enter_mode((0u != (flags & SM_FLG_SENSORS_ENABLED)) ? IMU_MODE_MONITOR :
               IMU_MODE_IDLE);
    imu_console_selftest(true, g_dbg.cal_result, g_dbg.g_mg, g_dbg.addr,
                         g_dbg.b_calibrated, g_now_ms);
}

/* reason: imu_cal_result_t or IMU_FAIL_* */
static void
selftest_failed(ER er, uint8_t reason)
{
    g_dbg.b_selftest_ok = false;
    g_dbg.cal_result = reason;
    if (((uint8_t)IMU_CAL_NOT_1G == reason) ||
        ((uint8_t)IMU_CAL_IMPOSSIBLE == reason))
    {
        g_b_bad_data = true;            /* the chip answers, data wrong  */
    }
    g_tries++;
    if (g_tries < ST_TRIES)
    {
        g_wait = RETRY_CYCLES;          /* next attempt in 1 s           */
        g_st_cycle = 0u;
        g_b_reloaded = false;
        return;
    }

    imu_console_selftest(false, g_dbg.cal_result, g_dbg.g_mg, g_dbg.addr,
                         g_dbg.b_calibrated, g_now_ms);
    if (g_b_initial)
    {
#if S4_CFG_REPORT_ONLY
        /* Never block the system on the IMU: report ready, flag a
           non-fatal IMU fault, keep retrying */
        post_sm(SM_EVT_READY, (uint16_t)SM_SUBSYS_IMU);
        g_dbg.b_ready_sent = true;
#else
        post_sm(SM_EVT_SELFTEST_FAIL, (uint16_t)SM_SUBSYS_IMU);
#endif
    }
    enter_fault(er, reason);
}

/*
 * One cycle of SELF-TEST & CALIBRATE.  Cycle 0: the chip answers at 0x1D
 * or 0x1E with WHO_AM_I = 49h (after a trimming reload if its last data
 * was wrong), and its settings read back.  Cycles 1..100: average at rest
 * (skipped if already calibrated and the data was not the problem; for a
 * runtime re-test the at-rest check is only attempted, never required).
 */
static void
selftest_cycle(uint32_t flags)
{
    imu_in_t in;
    uint8_t addr = DRV_IMU_NO_ADDR;
    int32_t tilt;
    ER er;

    if (0u != g_wait)
    {
        g_wait--;
        return;
    }

    if (0u == g_st_cycle)
    {
        /* Find the chip afresh (0x1D or 0x1E, WHO_AM_I 49h) before
           anything is written to it */
        drv_imu_recover();
        er = drv_imu_find(&addr);
        g_dbg.addr = addr;
        if (E_OK != er)
        {
            selftest_failed(er, IMU_FAIL_NOT_FOUND);
            return;
        }
        if (g_b_bad_data && !g_b_reloaded)
        {
            /* Its last data was wrong: reload the factory trimming, wait,
               then come back here to find and set it up again */
            g_b_reloaded = true;
            g_dbg.reloads++;
            imu_console_reload(g_dbg.cal_result, g_dbg.g_mg, g_now_ms);
            er = drv_imu_reboot();
            if (E_OK != er)
            {
                selftest_failed(er, IMU_FAIL_SETUP);
                return;
            }
            g_wait = RELOAD_CYCLES;
            return;
        }
        er = drv_imu_configure();
        if (E_OK != er)
        {
            selftest_failed(er, IMU_FAIL_SETUP);
            return;
        }
        if (g_dbg.b_calibrated && !g_b_bad_data)
        {
            g_dbg.cal_result = (uint8_t)IMU_CAL_OK;
            selftest_passed(flags);     /* design: skip if calibrated    */
            return;
        }
        imu_cal_reset(&g_cal);
        g_st_cycle = 1u;
        return;
    }

    if (!read_cycle(&in, false))
    {
        selftest_failed(E_IO, IMU_FAIL_READ);
        return;
    }
    imu_cal_add(&g_cal, (int16_t const (*)[3])in.acc, in.n_acc);
    g_st_cycle++;
    if (g_st_cycle <= ST_CYCLES)
    {
        return;
    }

    g_dbg.cal_result = (uint8_t)imu_cal_finish(&g_cal, ST_MIN_SAMPLES,
                                               &g_align, &g_dbg.g_mg);
    tilt = imu_cal_tilt_cdeg(&g_cal);
    if ((IMU_CAL_OK == g_dbg.cal_result) && (tilt <= IMU_ST_LEVEL_CDEG))
    {
        g_dbg.b_calibrated = true;
        selftest_passed(flags);
    }
    else if ((IMU_CAL_OK == g_dbg.cal_result) ||
             (IMU_CAL_MOVED == g_dbg.cal_result))
    {
        int32_t swing[3];

        /* The sensor works (|g| is possible, or the result would be
           IMU_CAL_IMPOSSIBLE), the car just is not still, or not
           standing level.  Keep the calibration it has; without one, run
           uncalibrated (identity: sane_cycle() calibrates on body data)
           until the first still, level second. */
        if (IMU_CAL_MOVED == g_dbg.cal_result)
        {
            imu_cal_swing_mg(&g_cal, swing);
            imu_console_moved(swing, g_now_ms);
        }
        else
        {
            imu_console_not_level(tilt, g_now_ms);
        }
        if (!g_dbg.b_calibrated)
        {
            imu_align_identity(&g_align);
        }
        selftest_passed(flags);
    }
    else
    {
        selftest_failed(E_IO, g_dbg.cal_result);
    }
}

/* Not calibrated yet, motors off, a full second of data: calibrate on
   it if the car was still and standing level.  Its data is body data
   (identity alignment while uncalibrated).  A still second that is not
   1 g re-tests the chip (trimming reload first, as in the self-test).
   true if the alignment changed or the self-test was started. */
static bool
still_calibrate(void)
{
    imu_align_t align;
    int32_t g_mg;
    int32_t swing[3];
    int32_t tilt;
    imu_cal_result_t res;

    res = imu_cal_finish(&g_sane, ST_MIN_SAMPLES, &align, &g_mg);
    tilt = imu_cal_tilt_cdeg(&g_sane);
    if ((IMU_CAL_OK == res) && (tilt > IMU_ST_LEVEL_CDEG))
    {
        if (!g_b_tilt_said)
        {
            g_b_tilt_said = true;       /* once per stay in MONITOR      */
            imu_console_not_level(tilt, g_now_ms);
        }
        return false;
    }
    if (IMU_CAL_OK == res)
    {
        g_align = align;
        g_dbg.b_calibrated = true;
        g_dbg.cal_result = (uint8_t)IMU_CAL_OK;
        g_dbg.g_mg = g_mg;
        imu_logic_restart(&g_ctx);      /* attitude restarts level       */
        imu_cal_swing_mg(&g_sane, swing);
        imu_console_calibrated(swing, g_mg, tilt, g_now_ms);
        return true;
    }
    if (IMU_CAL_NOT_1G == res)
    {
        g_dbg.cal_result = (uint8_t)IMU_CAL_NOT_1G;
        g_dbg.g_mg = g_mg;
        g_b_bad_data = true;
        start_selftest(false);
        return true;
    }
    return false;                       /* moving: next second           */
}

/* MONITOR: every 1 s, the mean |a| must be possible for a car.  Wrong
   data is handled like a lost sensor (FR27), and the re-test reloads
   the chip.  The same second calibrates the car if it still needs it
   (still_calibrate).  false if this cycle's data must not go on: the
   sensor was just declared faulty or re-tested, or the alignment just
   changed. */
static bool
sane_cycle(imu_in_t const *p_in, uint32_t flags)
{
    int32_t g_mg;
    bool b_judged;
    bool b_changed = false;

    imu_cal_add(&g_sane, (int16_t const (*)[3])p_in->acc, p_in->n_acc);
    g_sane_cycles++;
    if (g_sane_cycles < SANE_CYCLES)
    {
        return true;
    }
    g_mg = imu_cal_mean_mg(&g_sane);
    b_judged = (g_sane.n >= ST_MIN_SAMPLES);
#if (S4_CONSOLE_STILL == S4_CFG_CONSOLE)
    {
        int32_t swing[3];

        imu_cal_swing_mg(&g_sane, swing);
        imu_console_still(swing, g_mg, g_sane.n, g_now_ms);
    }
#endif
    if (b_judged && imu_g_plausible(g_mg) && !g_dbg.b_calibrated &&
        (0u == (flags & SM_FLG_MOTORS_ENABLED)))
    {
        b_changed = still_calibrate();
    }
    imu_cal_reset(&g_sane);
    g_sane_cycles = 0u;
    if (!b_judged || imu_g_plausible(g_mg))
    {
        return !b_changed;
    }
    g_dbg.g_mg = g_mg;
    g_dbg.cal_result = (uint8_t)IMU_CAL_IMPOSSIBLE;
    g_b_bad_data = true;
    enter_fault(E_OK, (uint8_t)IMU_CAL_IMPOSSIBLE);
    return false;
}

static void
handle_outputs(imu_out_t const *p_out)
{
    if (p_out->b_hump)
    {
        (void)net_post_event(NET_EVT_HUMP,
                             NET_EVT_HUMP_PAYLOAD(p_out->hump.peak_mm));
        imu_console_hump(&p_out->hump, g_now_ms);
    }
    if (p_out->b_class)
    {
        (void)net_post_event(NET_EVT_MOTION_CLASS,
                             NET_EVT_CLASS_PAYLOAD(p_out->cls));
    }
    if (p_out->b_collision)
    {
#if !S4_CFG_REPORT_ONLY
        post_sm(SM_EVT_FAULT, (uint16_t)SM_FAULT_COLLISION);
#endif
        imu_console_collision(p_out->collision_mg, !S4_CFG_REPORT_ONLY,
                              g_now_ms);
    }
    if (p_out->b_tipover)
    {
#if !S4_CFG_REPORT_ONLY
        post_sm(SM_EVT_FAULT, (uint16_t)SM_FAULT_TIP_OVER);
#endif
        imu_console_tipover(g_ctx.view.pitch_cdeg, g_ctx.view.roll_cdeg,
                            !S4_CFG_REPORT_ONLY, g_now_ms);
    }
}

static void
monitor_cycle(uint32_t flags)
{
    imu_in_t in;
    imu_out_t out;
    uint32_t t0;
    uint32_t took;

    if (!read_cycle(&in, true))
    {
        g_dbg.read_fails++;
        g_fail_run++;
        if (g_fail_run >= FAIL_LIMIT)
        {
            enter_fault(E_IO, IMU_FAIL_READ);
        }
        return;
    }
    g_fail_run = 0u;
    if (!sane_cycle(&in, flags))
    {
        return;
    }
    in.b_motors = (0u != (flags & SM_FLG_MOTORS_ENABLED)) ||
                  (S4_CFG_BENCH_MOTORS &&
                   (0u != (flags & SM_FLG_SENSORS_ENABLED)));

    t0 = drv_imu_now_us();
    imu_logic_step(&g_ctx, &in, &out);
    took = drv_imu_now_us() - t0;
    g_dbg.logic_us_max = (took > g_dbg.logic_us_max) ? took :
                         g_dbg.logic_us_max;
#if ENCODER_FREE_DEMO
    g_demo.time_ms=wall_ms();
    if(in.n_acc){for(int i=0;i<3;i++)g_demo.mg[i]=in.acc[in.n_acc-1][i]/IMU_Q2_PER_MG;}
    g_demo.latched=demo_pickup_step(&g_pickup,g_demo.time_ms,
        g_ctx.view.pitch_cdeg,g_ctx.view.roll_cdeg,in.b_motors,
        out.b_collision||out.b_tipover);
    if(out.b_hump){g_demo.peak_mm=out.hump.peak_mm;g_demo.height_valid=true;}
#endif
    handle_outputs(&out);
}

static void
fault_cycle(void)
{
    uint8_t addr;

    if (0u != g_wait)
    {
        g_wait--;
        return;
    }
    g_wait = RETRY_CYCLES;
    drv_imu_recover();
    if (E_OK == drv_imu_find(&addr))    /* identified again              */
    {
        g_dbg.addr = addr;
        start_selftest(false);          /* runtime re-test               */
    }
}

/* ---- Snapshot (IF-07) --------------------------------------------- */

static uint8_t
status_now(void)
{
    switch (g_mode)
    {
        case IMU_MODE_SELFTEST:
            return (uint8_t)S4_IMU_SELFTEST;
        case IMU_MODE_FAULT:
            return (uint8_t)S4_IMU_FAULT;
        case IMU_MODE_IDLE:
            return (uint8_t)S4_IMU_IDLE;
        default:
            return (uint8_t)S4_IMU_OK;
    }
}

static void
publish(void)
{
    imu_view_t const *p_v = &g_ctx.view;
    s4_tlm_t t = g_tlm_zero;

    t.pitch_half_deg = (unsigned int)imu_clamp(
        imu_div_round(imu_abs(p_v->pitch_cdeg), HALF_DEG_CDEG), 0,
        HALF_DEG_MAX);
    t.b_pitch_down = (p_v->pitch_cdeg < 0) ? 1u : 0u;
    t.roll_half_deg = (unsigned int)imu_clamp(
        imu_div_round(imu_abs(p_v->roll_cdeg), HALF_DEG_CDEG), 0,
        HALF_DEG_MAX);
    t.b_roll_left = (p_v->roll_cdeg < 0) ? 1u : 0u;     /* left side down */
    t.heading_deg = (unsigned int)(imu_div_round(p_v->heading_cdeg, DEG_CDEG) %
                                   HEADING_MAX_DEG);
    t.imu_status = status_now();
    t.turn_rate_dps = (unsigned int)imu_clamp(
        imu_div_round(imu_abs(p_v->rate_cdps), DEG_CDEG), 0, RATE_MAX_DPS);
    t.b_turn_left = (p_v->rate_cdps < 0) ? 1u : 0u;     /* anticlockwise  */
    t.motion_class = p_v->cls;
    t.last_peak_mm = p_v->last_peak_mm;
    t.max_peak_mm = p_v->max_peak_mm;

    g_dbg.mode = g_mode;
    if (E_OK == tk_loc_mtx(g_mtx_tlm, TMO_FEVR))
    {
        g_tlm = t;
#if ENCODER_FREE_DEMO
        g_demo.ready=g_dbg.b_calibrated&&g_mode==IMU_MODE_MONITOR;
        g_demo_pub=g_demo;
#endif
        g_dbg_pub = g_dbg;
        (void)tk_unl_mtx(g_mtx_tlm);
    }
}

/* ---- Task --------------------------------------------------------- */

static void
imu_cyclic_handler(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(g_tid_imu);
}

static void
wait_tick(void)
{
    INT missed;

    (void)tk_slp_tsk(TMO_FEVR);
    missed = tk_can_wup(TSK_SELF);      /* never "catch up" in a burst  */
    if (missed > 0)
    {
        g_dbg.missed += (uint32_t)missed;
    }
}

static void
imu_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

#if (S4_CONSOLE_RAW == S4_CFG_CONSOLE)
    imu_console_raw(wait_tick);         /* Stage 2 bring-up only         */
#elif (S4_CONSOLE_ACCDIAG == S4_CFG_CONSOLE)
    imu_console_accdiag(wait_tick);     /* accelerometer diagnosis only  */
#endif
    drv_imu_init();
    imu_logic_init(&g_ctx, 0 != S4_CFG_COLLISION_LEVEL_CHECK);
    imu_align_identity(&g_align);
    start_selftest(true);               /* boot counts as INITIALISING   */

    for (;;)
    {
        uint32_t flags;

        wait_tick();
        g_dbg.cycles++;
#if ENCODER_FREE_DEMO
        uint32_t actual_now = wall_ms();
        if (g_demo.time_ms != 0u && actual_now - g_demo.time_ms > 30u &&
            g_mode == IMU_MODE_MONITOR) {
            imu_logic_restart(&g_ctx);
            g_demo.height_valid = false;
        }
#endif
        g_now_ms += IMU_CYCLE_MS;
        flags = sm_mode_flags();
#if ENCODER_FREE_DEMO
        UINT req=0;tk_wai_flg(g_demo_request,1u,TWF_ORW|TWF_BITCLR,&req,TMO_POL);
        if(req&&SM_STATUS_STATE(sm_status_word())==SM_STATE_STANDBY){g_demo.latched=false;g_demo.height_valid=false;g_demo.peak_mm=0;g_dbg.b_calibrated=false;g_pickup=(demo_pickup_t){0};start_selftest(true);}
#endif

        if ((0u != (flags & SM_FLG_INITIALISING)) &&
            (0u == (g_prev_flags & SM_FLG_INITIALISING)) &&
            (IMU_MODE_SELFTEST != g_mode))
        {
            start_selftest(true);
        }
        g_prev_flags = flags;

        switch (g_mode)
        {
            case IMU_MODE_SELFTEST:
                selftest_cycle(flags);
                break;
            case IMU_MODE_IDLE:
                if (0u != (flags & SM_FLG_SENSORS_ENABLED))
                {
                    enter_mode(IMU_MODE_MONITOR);
                }
                break;
            case IMU_MODE_MONITOR:
                if (0u == (flags & SM_FLG_SENSORS_ENABLED))
                {
                    enter_mode(IMU_MODE_IDLE);  /* NFR14: stop reading  */
                }
                else
                {
                    monitor_cycle(flags);
                }
                break;
            default:
                fault_cycle();
                break;
        }

        flush_sm();
        publish();
        if (0u == (g_dbg.cycles % CONSOLE_CYCLES))
        {
            imu_console_status(&g_ctx.view, status_now(),
                               imu_logic_take_spike(&g_ctx), &g_dbg,
                               g_now_ms);
        }
    }
}

/* ---- Interface ---------------------------------------------------- */

EXPORT ER
s4_get_tlm(s4_tlm_t *p_tlm)
{
    ER er;

    if (NULL == p_tlm)
    {
        return E_PAR;
    }
    er = tk_loc_mtx(g_mtx_tlm, TMO_FEVR);
    if (E_OK != er)
    {
        return er;
    }
    *p_tlm = g_tlm;
    (void)tk_unl_mtx(g_mtx_tlm);
    return E_OK;
}

EXPORT void
imu_task_debug(imu_task_dbg_t *p_dbg)
{
    if (E_OK == tk_loc_mtx(g_mtx_tlm, TMO_FEVR))
    {
        *p_dbg = g_dbg_pub;
        (void)tk_unl_mtx(g_mtx_tlm);
    }
}

EXPORT ER
imu_task_init(void)
{
    T_CMTX cmtx = {0};
    T_CCYC ccyc = {0};

#if ENCODER_FREE_DEMO
    T_CFLG f={0};f.flgatr=TA_TFIFO;g_demo_request=tk_cre_flg(&f);if(g_demo_request<0)return g_demo_request;
#endif
    g_tlm.imu_status = (unsigned int)S4_IMU_SELFTEST;
    g_mode = (uint8_t)IMU_MODE_SELFTEST;

    cmtx.mtxatr = TA_INHERIT;
    g_mtx_tlm = tk_cre_mtx(&cmtx);
    if (g_mtx_tlm < E_OK)
    {
        return g_mtx_tlm;
    }

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)imu_cyclic_handler;
    ccyc.cyctim = IMU_CYCLE_MS;
    ccyc.cycphs = IMU_CYCLE_MS;
    g_cyc_imu = tk_cre_cyc(&ccyc);
    if (g_cyc_imu < E_OK)
    {
        return g_cyc_imu;
    }
    return E_OK;
}

EXPORT ER
imu_task_start(void)
{
    g_tid_imu = app_start_task(imu_task, S4_PRI_MAIN, 0u, "imu_task");
    if (g_tid_imu < E_OK)
    {
        return g_tid_imu;
    }
    return tk_sta_cyc(g_cyc_imu);
}
