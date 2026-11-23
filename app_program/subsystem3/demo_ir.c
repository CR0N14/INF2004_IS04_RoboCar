#include "../demo_config.h"
#if ENCODER_FREE_DEMO
#include "demo_ir.h"
#include "drv_ir.h"
#include "ir_config.h"
#include "../subsystem1/state_machine.h"
#include "../subsystem2/subsystem2.h"
static ID          lock, requests;
static demo_line_t published;
static s3_tlm_t    published_tlm;

/* Full-rate right IR diagnostics, buffered independently of MQTT.
   512 samples hold 5 s at 10 ms, enough to ride out a WiFi stall. */
#define DIAG_RING 512u
static uint32_t diag_ring[DIAG_RING][5], diag_next[5];
static unsigned diag_head, diag_tail;
static uint32_t diag_sequence;

ER demo_ir_diag (uint8_t * out)
{
    if (lock <= 0)
        return E_OBJ;
    ER e = tk_loc_mtx(lock, TMO_POL);
    if (e != E_OK)
        return e;
    if (diag_head - diag_tail < 3u)
    {
        tk_unl_mtx(lock);
        return E_OBJ;
    }
    uint32_t w[16] = {0x31425249u}; /* IRB1 */
    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned j = 0; j < 5; j++)
            w[1 + i * 5 + j] = diag_ring[diag_tail % DIAG_RING][j];
        ++diag_tail;
    }
    tk_unl_mtx(lock);
    for (unsigned i = 0; i < 16; i++)
        for (unsigned b = 0; b < 4; b++)
            out[i * 4 + b] = (uint8_t)(w[i] >> (8 * b));
    return E_OK;
}

static uint32_t clock_ms (void)
{
    SYSTIM t;
    tk_get_otm(&t);
    return t.lo;
}

ER demo_ir_read (demo_line_t * out)
{
    if (lock <= 0)
        return E_OBJ;
    ER e = tk_loc_mtx(lock, TMO_POL);
    if (e == E_OK)
    {
        *out = published;
        tk_unl_mtx(lock);
    }
    return e;
}

ER demo_ir_tlm (s3_tlm_t * out)
{
    if (!out)
        return E_PAR;
    if (lock <= 0)
        return E_OBJ;
    ER e = tk_loc_mtx(lock, TMO_POL);
    if (e == E_OK)
    {
        *out = published_tlm;
        tk_unl_mtx(lock);
    }
    return e;
}

ER demo_ir_calibrate (UINT surface)
{
    if (surface > 1 || requests <= 0 ||
        SM_STATUS_STATE(sm_status_word()) != SM_STATE_STANDBY)
        return E_OBJ;
    return tk_set_flg(requests, 1u << surface);
}

static bool bind (void * ctx, uint8_t symbol, uint16_t version,
                  uint8_t * action)
{
    sm_action_t a;
    (void)ctx;
    (void)version;
    if (!sm_barcode_lookup(symbol, &a))
        return false;
    *action = a;
    return true;
}

static s3_tlm_t          tlm     = {0};
static demo_line_t       s       = {0};
static barcode_decoder_t decoder = {0};
static uint32_t          sum[3] = {0}, start = 0, cross = 0, approach = 0;
static uint16_t          lo[3] = {0}, hi[3] = {0};
static unsigned          count = 0, capture = 0, have = 0;
static bool              route = false, qualified = false, crossing = false,
            approaching          = false;
static uint16_t      gen         = 0;
static uint32_t      last_status = ~0u;
static uint32_t      bound = 0, old_bound = 0;
static uint64_t      actions = 0, old_actions = 0;
static uint16_t      binding     = 0;
static sm_evt_code_t pending     = SM_EVT_NONE;
static uint16_t      pending_arg = 0;

static void context_step (uint32_t status, sm_nav_mode_t nav)
{
    uint32_t previous = last_status;
    uint32_t context  = status & 63u;
    sm_get_bindings(&bound, &actions);
    if (bound != old_bound || actions != old_actions)
    {
        old_bound   = bound;
        old_actions = actions;
        ++binding;
        route     = false;
        qualified = false;
        barcode_decoder_reset(&decoder);
    }
    if (context != last_status)
    {
        last_status = context;
        ++gen;
        barcode_decoder_reset(&decoder);
        crossing    = false;
        approaching = false;
        qualified   = false;
        pending     = SM_EVT_NONE;
        if ((SM_STATUS_STATE(status) != SM_STATE_NAVIGATING &&
             SM_STATUS_STATE(status) != SM_STATE_OBSTACLE_AVOIDANCE) ||
            nav == SM_NAV_MANOEUVRE)
            route = false;
    }
    if (DEMO_JUNCTION_TEST && previous != context)
    {
        sm_action_t action;
        if (SM_STATUS_STATE(previous) == SM_STATE_STANDBY &&
            SM_STATUS_STATE(status) == SM_STATE_NAVIGATING &&
            sm_barcode_lookup(0u, &action))
        {
            pending         = SM_EVT_BARCODE;
            pending_arg     = SM_BARCODE_ARG(0u, action);
            route           = true;
            tlm.last_symbol = 0u;
            tlm.last_action = action;
            /* Synthetic instruction: do not claim optical decoding. */
            tlm.b_barcode = 0u;
        }
    }
}

static void request_calibration (uint32_t now, uint32_t status)
{
    UINT req = 0;
    tk_wai_flg(requests, 3, TWF_ORW | TWF_BITCLR, &req, TMO_POL);
    if ((req == 1 || req == 2) && SM_STATUS_STATE(status) == SM_STATE_STANDBY)
    {
        capture      = req;
        count        = 0;
        start        = now;
        s.calibrated = false;
        s.cal_phase  = req;
        for (int i = 0; i < 3; i++)
        {
            sum[i] = 0;
            lo[i]  = 4095;
            hi[i]  = 0;
        }
    }
}

static void finish_calibration (void)
{
    bool stable = true;
    for (int i = 0; i < 3; i++)
        if (hi[i] - lo[i] > 150u)
            stable = false;
    if (stable)
    {
        for (int i = 0; i < 3; i++)
        {
            if (capture == 1)
                s.white[i] = sum[i] / count;
            else
                s.black[i] = sum[i] / count;
        }
        have |= capture;
    }
    s.cal_phase = stable ? 0 : 4;
    capture     = 0;
    if (have == 3)
    {
        s.calibrated = stable;
        for (int i = 0; i < 3; i++)
        {
            int span = (int)s.black[i] - s.white[i];
            if (span < 200 && span > -200)
                s.calibrated = false;
        }
        if (s.calibrated)
        {
            ir_calibration_t cal = {.white = s.white[2], .black = s.black[2]};
            barcode_config_t config = ir_config_barcode(cal);
            config.p_bind           = bind;
            if (DEMO_BARCODE_BENCH)
            {
                config.clearance_ms   = 2000u;
                config.max_capture_ms = 30000u;
            }
            if (DEMO_BARCODE_DRIVE_MS)
            {
                config.clearance_ms = DEMO_BARCODE_CLEAR_MS;
            }
            barcode_decoder_init(&decoder, &config);
            s.cal_phase = 3;
        }
        else
            s.cal_phase = 4;
    }
}

static void calibration_step (bool valid, uint32_t now, uint32_t status)
{
    if (capture)
    {
        if (SM_STATUS_STATE(status) != SM_STATE_STANDBY || !valid ||
            now - start > 2000u)
        {
            capture     = 0;
            s.cal_phase = 4;
        }
        else
        {
            for (int i = 0; i < 3; i++)
            {
                sum[i] += s.raw[i];
                if (s.raw[i] < lo[i])
                    lo[i] = s.raw[i];
                if (s.raw[i] > hi[i])
                    hi[i] = s.raw[i];
            }
            ++count;
            if (count == 50)
            {
                finish_calibration();
            }
        }
    }
}

static void junction_step (bool enabled, uint32_t now)
{
    if (route && enabled)
    {
        if ((s.mask & 3u) == 2u)
        {
            if (!approaching)
            {
                approaching = true;
                approach    = now;
            }
            if (now - approach >= 100u)
                qualified = true;
        }
        else
            approaching = false;
        if (qualified && (s.mask & 3u) == 3u)
        {
            if (!crossing)
            {
                crossing = true;
                cross    = now;
            }
            if (now - cross >= DEMO_JUNCTION_MS && pending == SM_EVT_NONE)
            {
                pending     = SM_EVT_JUNCTION;
                pending_arg = 0;
                route       = false;
            }
        }
        else
            crossing = false;
    }
}

static void process_frame (ir_frame_t const * frame, uint32_t now,
                           uint32_t status, sm_nav_mode_t nav)
{
    for (int i = 0; i < 3; i++)
    {
        ir_calibration_t cal = {.white = s.white[i], .black = s.black[i]};
        uint16_t         d   = 0;
        ir_processing_normalise(cal, s.raw[i], 200, &d);
        if (d >= 2400)
            s.mask |= 1u << i;
        if (d <= 1600)
            s.mask &= ~(1u << i);
    }
    /* Centre-only is straight; left-only turns left. White triggers
       search. With only left+centre trackers the right side is not
       directly observed. */
    s.steer      = (s.mask & 3u) == 1u ? -DEMO_LINE_CORRECTION : 0;
    bool enabled = SM_STATUS_STATE(status) == SM_STATE_NAVIGATING &&
                   nav == SM_NAV_TRACKING;
    if (DEMO_JUNCTION_TEST)
    {
        junction_step(enabled, now);
        tlm.decoder_state = S3_DECODER_IDLE;
        return;
    }
    bool bench =
        DEMO_BARCODE_BENCH && SM_STATUS_STATE(status) == SM_STATE_STANDBY;
    enabled     = enabled || bench;
    int      dl = 0, dr = 0;
    uint32_t stamp = 0;
    s2_demo_duty(&dl, &dr, &stamp);
    ir_permission_t perm = {
        .generation = gen, .mode = IR_MODE_TRACKING, .b_enabled = enabled};
    bool moving =
        enabled && dl > 0 && dr > 0 && clock_ms() - stamp <= DEMO_STALE_MS;
    if (bench)
    {
        moving = true;
        stamp  = s.time_ms;
    }
    barcode_motion_t motion       = {.time_ms    = (uint16_t)stamp,
                                     .generation = gen,
                                     .direction  = BARCODE_FORWARD,
                                     .b_valid    = moving};
    unsigned         before_state = decoder.state, before_count = decoder.count;
    barcode_result_t result = barcode_decoder_step(
        &decoder, frame, perm, binding, motion, (uint16_t)clock_ms());
    uint16_t darkness = 0;
    ir_processing_normalise(decoder.config.calibration, s.raw[2], 200,
                            &darkness);
    diag_next[0] = now;
    diag_next[1] = s.raw[2] | ((uint32_t)darkness << 16);
    diag_next[2] =
        (s.mask & 7u) | (enabled ? 8u : 0u) | (motion.b_valid ? 16u : 0u) |
        (before_state << 5) | ((uint32_t)decoder.state << 7) |
        (before_count << 9) | ((uint32_t)decoder.count << 15) |
        ((uint32_t)result.outcome << 21) | ((uint32_t)result.symbol << 25) |
        ((uint32_t)result.action << 30);
    diag_next[3] = (uint16_t)dl | ((uint32_t)(uint16_t)dr << 16);
    /* Bench and timed drive tests log decodes but never act on them. */
    bool const log_only = bench || (DEMO_BARCODE_DRIVE_MS != 0u);
    if (result.outcome == BARCODE_ACCEPTED && pending == SM_EVT_NONE)
    {
        if (!log_only)
            pending = SM_EVT_BARCODE;
        pending_arg     = SM_BARCODE_ARG(result.symbol, result.action);
        route           = !log_only;
        qualified       = false;
        approaching     = false;
        tlm.last_symbol = result.symbol;
        tlm.last_action = result.action;
        tlm.b_barcode   = 1;
    }
    junction_step(enabled && !log_only, now);
    tlm.decoder_state =
        decoder.state == BARCODE_CAPTURING
            ? S3_DECODER_CAPTURING
            : (decoder.state == BARCODE_IDLE ? S3_DECODER_SEARCHING
                                             : S3_DECODER_WAIT_CLEAR);
}

static void publish_frame (bool valid, sm_nav_mode_t nav)
{
    if (pending != SM_EVT_NONE && sm_post_event(pending, pending_arg) == E_OK)
        pending = SM_EVT_NONE;
    tlm.ir_pattern = ((s.mask & 1) << 2) | (s.mask & 2) | ((s.mask & 4) >> 2);
    tlm.b_line_valid =
        s.calibrated && s.valid && ((s.mask & 3) == 1 || (s.mask & 3) == 2);
    tlm.line_state =
        !s.calibrated || !valid
            ? S3_LINE_FAULT
            : (nav == SM_NAV_RECOVERY
                   ? S3_LINE_REACQUIRING
                   : (tlm.b_line_valid ? S3_LINE_TRACKING : S3_LINE_LOST));
    tlm.line_pos    = s.steer < 0 ? -s.steer : s.steer;
    tlm.b_line_left = s.steer < 0;
    if (tk_loc_mtx(lock, TMO_POL) == E_OK)
    {
        diag_next[4] = diag_sequence++;
        if (diag_head - diag_tail >= DIAG_RING)
            ++diag_tail;
        for (unsigned j = 0; j < 5; j++)
            diag_ring[diag_head % DIAG_RING][j] = diag_next[j];
        ++diag_head;
        published     = s;
        published_tlm = tlm;
        tk_unl_mtx(lock);
    }
}

static void task (INT code, void * arg)
{
    (void)code;
    (void)arg;
    drv_ir_t driver = {0};
    if (drv_ir_open(&driver) != E_OK)
    {
        sm_post_event(SM_EVT_FAULT, SM_FAULT_IR_SENSOR);
        tk_ext_tsk();
        return;
    }
    ir_source_t source = {drv_ir_clock, drv_ir_read, &driver, 0};
    for (;;)
    {
        uint32_t      now = clock_ms(), status = sm_status_word();
        sm_nav_mode_t nav = SM_STATUS_NAV(status);
        context_step(status, nav);
        request_calibration(now, status);
        ir_frame_t  frame;
        ir_sample_t sample = {0};
        bool        valid  = ir_frame_acquire(&source, gen, 10, &frame) &&
                     ir_frame_unpack(frame.bytes, IR_FRAME_BYTES, &sample) &&
                     sample.valid_mask == 7 && !driver.b_clock_failed;
        s.time_ms = now;
        s.valid   = valid;
        if (valid)
        {
            for (int i = 0; i < 3; i++)
            {
                s.raw[i] = sample.raw[i];
            }
        }
        /* Include invalid/unconfigured samples instead of hiding gaps. */
        diag_next[0] = now;
        diag_next[1] = s.raw[2];
        diag_next[2] = 15u << 21;
        diag_next[3] = 0;
        calibration_step(valid, now, status);
        if (valid && s.calibrated)
        {
            process_frame(&frame, now, status, nav);
        }
        publish_frame(valid, nav);
#if DEMO_IR_PERIOD_MS
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF); /* skip a missed period, never burst */
#else
        tk_dly_tsk(10);
#endif
    }
}

#if DEMO_IR_PERIOD_MS
static ID ir_task_id;

static void ir_cyclic (void * exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(ir_task_id);
}
#endif

ER demo_ir_start (void)
{
    T_CMTX m = {0};
    m.mtxatr = TA_INHERIT;
    lock     = tk_cre_mtx(&m);
    if (lock < 0)
        return lock;
    T_CFLG f = {0};
    f.flgatr = TA_TFIFO;
    requests = tk_cre_flg(&f);
    if (requests < 0)
        return requests;
    ID id = app_start_task((FP)task, PRI_LINE_FOLLOWING, 0, "demo_ir");
    if (id < 0)
        return id;
#if DEMO_IR_PERIOD_MS
    ir_task_id   = id;
    T_CCYC ccyc  = {0};
    ccyc.cycatr  = TA_HLNG | TA_STA;
    ccyc.cychdr  = (FP)ir_cyclic;
    ccyc.cyctim  = DEMO_IR_PERIOD_MS;
    ccyc.cycphs  = DEMO_IR_PERIOD_MS;
    ID const cyc = tk_cre_cyc(&ccyc);
    if (cyc < 0)
        return cyc;
#endif
    return E_OK;
}
#else
typedef int demo_ir_disabled_translation_unit_t;
#endif
/* End of file. */

