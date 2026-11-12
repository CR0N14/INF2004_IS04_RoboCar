/*
 *  imu_task.c
 *  IMU task (design review section 6.4): owns the GY-511 on I2C1, runs
 *  every 10 ms from its cyclic handler, and turns samples into the IF-07
 *  snapshot, IF-08 telemetry events and IF-01 control events.
 *
 *  SELF-TEST --pass--> IDLE <--SENSORS_ENABLED--> MONITOR
 *  SELF-TEST --fails 3x--> FAULT;  MONITOR --5 failed reads--> FAULT
 *  FAULT --every 1 s, sensor answers--> SELF-TEST (runtime)
 *  Any state --SM_FLG_INITIALISING rising--> SELF-TEST (initial)
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
#include "subsystem4.h"
#include "imu_cfg.h"
#include "imu_console.h"
#include "imu_logic.h"
#include "imu_fixmath.h"
#include "drv_imu.h"
#include "../subsystem1/state_machine.h"
#include "../subsystem1/networking.h"

/* s4_motion_class_t and imu_class_t must agree (telemetry numbering) */
typedef char imu_class_check_t[((int)S4_CLASS_SUDDEN_IMPACT ==
                                 (int)IMU_CLASS_SUDDEN_IMPACT) ? 1 : -1];

#define ST_CYCLES           100u    /* 1 s at rest (FR20)                */
#define ST_TRIES            3u      /* then "self-test failed"           */
#define ST_MIN_SAMPLES      360u    /* of about 400 at 400 Hz            */
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
    enter_mode(IMU_MODE_SELFTEST);
}

static void
enter_fault(ER er)
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
        imu_console_fault(er, g_now_ms);
    }
}

static void
selftest_passed(uint32_t flags)
{
    g_dbg.b_selftest_ok = true;
    g_b_fault_reported = false;
    if (g_b_initial)
    {
        post_sm(SM_EVT_READY, (uint16_t)SM_SUBSYS_IMU);
        g_dbg.b_ready_sent = true;
    }
    enter_mode((0u != (flags & SM_FLG_SENSORS_ENABLED)) ? IMU_MODE_MONITOR :
               IMU_MODE_IDLE);
    imu_console_selftest(true, g_dbg.cal_result, g_dbg.g_mg, g_dbg.addr,
                         g_now_ms);
}

static void
selftest_failed(ER er)
{
    g_dbg.b_selftest_ok = false;
    g_tries++;
    if (g_tries < ST_TRIES)
    {
        g_wait = RETRY_CYCLES;          /* next attempt in 1 s           */
        g_st_cycle = 0u;
        return;
    }

    imu_console_selftest(false, g_dbg.cal_result, g_dbg.g_mg, g_dbg.addr,
                         g_now_ms);
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
    enter_fault(er);
}

/*
 * One cycle of SELF-TEST & CALIBRATE.  Cycle 0: the chip answers at 0x1D
 * or 0x1E with WHO_AM_I = 49h, and its settings read back.  Cycles
 * 1..100: average at rest (skipped if already calibrated, and for a
 * runtime re-test the at-rest check is only attempted, never required).
 */
static void
selftest_cycle(uint32_t flags)
{
    imu_in_t in;
    uint8_t addr = DRV_IMU_NO_ADDR;
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
        if (E_OK == er)
        {
            er = drv_imu_configure();
        }
        if (E_OK != er)
        {
            selftest_failed(er);
            return;
        }
        if (g_dbg.b_calibrated)
        {
            selftest_passed(flags);     /* design: skip if calibrated    */
            return;
        }
        imu_cal_reset(&g_cal);
        g_st_cycle = 1u;
        return;
    }

    if (!read_cycle(&in, false))
    {
        selftest_failed(E_IO);
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
    if (IMU_CAL_OK == g_dbg.cal_result)
    {
        g_dbg.b_calibrated = true;
        selftest_passed(flags);
    }
    else if (!g_b_initial && (IMU_CAL_MOVED == g_dbg.cal_result))
    {
        /* Back during a run: the sensor works, the car just is not
           still.  Run uncalibrated rather than stay without data. */
        imu_align_identity(&g_align);
        selftest_passed(flags);
    }
    else
    {
        selftest_failed(E_IO);
    }
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
            enter_fault(E_IO);
        }
        return;
    }
    g_fail_run = 0u;
    in.b_motors = (0u != (flags & SM_FLG_MOTORS_ENABLED)) ||
                  (S4_CFG_BENCH_MOTORS &&
                   (0u != (flags & SM_FLG_SENSORS_ENABLED)));

    t0 = drv_imu_now_us();
    imu_logic_step(&g_ctx, &in, &out);
    took = drv_imu_now_us() - t0;
    g_dbg.logic_us_max = (took > g_dbg.logic_us_max) ? took :
                         g_dbg.logic_us_max;
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
        g_now_ms += IMU_CYCLE_MS;
        flags = sm_mode_flags();

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
