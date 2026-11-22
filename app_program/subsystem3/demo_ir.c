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
    uint32_t context = status & 63u;
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
    int      dl = 0, dr = 0;
    uint32_t stamp = 0;
    s2_demo_duty(&dl, &dr, &stamp);
    ir_permission_t perm = {
        .generation = gen, .mode = IR_MODE_TRACKING, .b_enabled = enabled};
    barcode_motion_t motion = {.time_ms    = (uint16_t)stamp,
                               .generation = gen,
                               .direction  = BARCODE_FORWARD,
                               .b_valid    = enabled && dl > 0 && dr > 0 &&
                                          clock_ms() - stamp <= DEMO_STALE_MS};
    barcode_result_t result = barcode_decoder_step(
        &decoder, frame, perm, binding, motion, (uint16_t)clock_ms());
    if (result.outcome == BARCODE_ACCEPTED && pending == SM_EVT_NONE)
    {
        pending         = SM_EVT_BARCODE;
        pending_arg     = SM_BARCODE_ARG(result.symbol, result.action);
        route           = true;
        qualified       = false;
        approaching     = false;
        tlm.last_symbol = result.symbol;
        tlm.last_action = result.action;
        tlm.b_barcode   = 1;
    }
    junction_step(enabled, now);
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
        calibration_step(valid, now, status);
        if (valid && s.calibrated)
        {
            process_frame(&frame, now, status, nav);
        }
        publish_frame(valid, nav);
        tk_dly_tsk(10);
    }
}

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
    return id < 0 ? id : E_OK;
}
#else
typedef int demo_ir_disabled_translation_unit_t;
#endif
/* End of file. */

