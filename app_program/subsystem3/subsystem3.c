/** @file subsystem3.c
 * @brief Application lifecycle and protected IF-07 adapter for the pure core.
 */
#include "subsystem3.h"
#include "../subsystem1/state_machine.h"

static ID                g_h_mutex         = 0;
static ir_processing_t   g_line            = {0};
static barcode_decoder_t g_decoder         = {0};
static ir_permission_t   g_permission      = {0};
static ir_result_t       g_target          = {0};
static s3_tlm_t          g_telemetry       = {0};
static uint16_t          g_binding_version = 0u;

static struct
{
    bool b_configured         : 1;
    bool b_started            : 1;
    bool b_fault              : 1;
    bool b_navigation_pending : 1;
} g_state = {0};

static bool lookup_action (void * p_context, uint8_t symbol, uint16_t version,
                           uint8_t * p_action);
static ER   enter (void);
static ER   leave (ER result);
static void clear_context (void);
static void update_line (ir_result_t result);
static void update_decoder (barcode_result_t result);

ER subsystem3_init (void)
{
    T_CMTX attributes = {0};

    if (0 != g_h_mutex)
    {
        return E_OBJ;
    }

    attributes.mtxatr = TA_INHERIT;
    ID const h_mutex  = tk_cre_mtx(&attributes);

    if (h_mutex < E_OK)
    {
        return h_mutex;
    }

    g_h_mutex = h_mutex;
    clear_context();
    return E_OK;
}

ER subsystem3_start (void)
{
    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if (g_state.b_started)
    {
        return leave(E_OBJ);
    }

    /* This service does not claim ADC readiness or create a fake sensor task.
     */
    g_state.b_started = true;
    return leave(E_OK);
}

ER subsystem3_configure (ir_config_t const *      p_line,
                         barcode_config_t const * p_barcode)
{
    ir_processing_t   line    = {0};
    barcode_decoder_t decoder = {0};

    if ((!ir_processing_init(&line, p_line)) ||
        (!barcode_decoder_init(&decoder, p_barcode)))
    {
        return E_PAR;
    }

    if ((0u != p_line->left_channel) || (1u != p_line->centre_channel) ||
        (2u != p_line->barcode_channel) ||
        (p_line->barcode_channel != p_barcode->channel) ||
        (p_line->calibration[p_barcode->channel].white !=
         p_barcode->calibration.white) ||
        (p_line->calibration[p_barcode->channel].black !=
         p_barcode->calibration.black))
    {
        return E_PAR;
    }

    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if (g_state.b_started)
    {
        return leave(E_OBJ);
    }

    g_line               = line;
    g_decoder            = decoder;
    g_state.b_configured = true;
    if (NULL == g_decoder.config.p_bind)
    {
        g_decoder.config.p_bind = lookup_action;
    }
    clear_context();
    return leave(E_OK);
}

ER subsystem3_set_context (ir_permission_t permission, uint16_t binding_version)
{
    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if ((!g_state.b_configured) || (!g_state.b_started))
    {
        return leave(E_OBJ);
    }

    if (g_state.b_fault && permission.b_enabled &&
        (permission.generation == g_permission.generation))
    {
        return leave(E_OBJ);
    }

    bool const b_changed =
        (permission.generation != g_permission.generation) ||
        (binding_version != g_binding_version) ||
        (permission.mode != g_permission.mode) ||
        (permission.b_enabled != g_permission.b_enabled) ||
        (permission.b_junction_expected != g_permission.b_junction_expected) ||
        (permission.b_recovery_expected != g_permission.b_recovery_expected) ||
        (permission.b_barcode_gap_expected !=
         g_permission.b_barcode_gap_expected);

    if (permission.generation != g_permission.generation)
    {
        g_state.b_fault = false;
    }

    if (b_changed)
    {
        clear_context();
    }

    g_permission      = permission;
    g_binding_version = binding_version;
    return leave(E_OK);
}

ER subsystem3_line_step (ir_frame_t const * p_frame, uint16_t now_ms,
                         ir_result_t * p_result)
{
    ir_result_t const empty = {0};

    if (NULL == p_result)
    {
        return E_PAR;
    }

    *p_result      = empty;
    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if ((!g_state.b_configured) || (!g_state.b_started))
    {
        return leave(E_OBJ);
    }

    ir_permission_t permission = g_permission;
    permission.b_junction_expected =
        permission.b_junction_expected || g_state.b_navigation_pending;
    *p_result = ir_processing_step(&g_line, p_frame, permission, now_ms);
    g_target  = *p_result;
    update_line(*p_result);
    if (p_result->b_fault)
    {
        g_state.b_fault        = true;
        g_permission.b_enabled = false;
        barcode_decoder_reset(&g_decoder);
        g_telemetry.b_barcode = 0u;
    }

    if (p_result->b_junction && g_state.b_navigation_pending)
    {
        ER const delivery            = sm_post_event(SM_EVT_JUNCTION, 0u);
        g_state.b_navigation_pending = false;
        return leave(delivery);
    }
    return leave(E_OK);
}

ER subsystem3_barcode_step (ir_frame_t const * p_frame, barcode_motion_t motion,
                            uint16_t now_ms, barcode_result_t * p_result)
{
    barcode_result_t const empty = {0};

    if (NULL == p_result)
    {
        return E_PAR;
    }

    *p_result      = empty;
    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if ((!g_state.b_configured) || (!g_state.b_started))
    {
        return leave(E_OBJ);
    }

    *p_result = barcode_decoder_step(&g_decoder, p_frame, g_permission,
                                     g_binding_version, motion, now_ms);
    update_decoder(*p_result);
    if (BARCODE_ACCEPTED == p_result->outcome)
    {
        /* Validated symbol/action occupy seven bits. */
        uint16_t const argument =
            (uint16_t)((p_result->symbol * 4u) + p_result->action);
        ER const delivery            = sm_post_event(SM_EVT_BARCODE, argument);
        g_state.b_navigation_pending = E_OK == delivery;
        return leave(delivery);
    }
    return leave(E_OK);
}

ER subsystem3_get_target (uint16_t now_ms, ir_result_t * p_result)
{
    ir_result_t const empty = {0};

    if (NULL == p_result)
    {
        return E_PAR;
    }

    *p_result      = empty;
    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if ((!g_state.b_configured) || (!g_state.b_started))
    {
        return leave(E_OBJ);
    }

    *p_result = g_target;
    p_result->b_target_valid =
        p_result->b_target_valid && g_permission.b_enabled &&
        (!g_state.b_fault) && (IR_MODE_TRACKING == g_permission.mode) &&
        (p_result->generation == g_permission.generation) &&
        ir_frame_age(now_ms, p_result->time_ms, g_line.config.max_gap_ms);
    return leave(E_OK);
}

ER s3_get_tlm (s3_tlm_t * p_tlm)
{
    s3_tlm_t const empty = {0};

    if (NULL == p_tlm)
    {
        return E_PAR;
    }

    *p_tlm         = empty;
    ER const error = enter();

    if (E_OK != error)
    {
        return error;
    }

    if (!g_state.b_configured)
    {
        return leave(E_NOSPT);
    }

    SYSTIM   time        = {0};
    ER const clock_error = tk_get_otm(&time);

    if (E_OK != clock_error)
    {
        return leave(clock_error);
    }

    *p_tlm = g_telemetry;
    /* The conversion selects the same modulo-65536 clock as acquisition. */
    p_tlm->b_line_valid =
        p_tlm->b_line_valid && ir_frame_age((uint16_t)time.lo, g_target.time_ms,
                                            g_line.config.max_gap_ms);
    return leave(E_OK);
}

/** Bound lock wait: callers handle E_TMOUT rather than waiting indefinitely. */
static ER enter (void)
{
    if (g_h_mutex <= 0)
    {
        return E_OBJ;
    }

    return tk_loc_mtx(g_h_mutex, TMO_POL);
}

/** Preserve the original error unless unlocking itself fails. */
static ER leave (ER result)
{
    ER const error = tk_unl_mtx(g_h_mutex);

    return (E_OK == error) ? result : error;
}

/** Called under the mutex, or during initialisation before users exist. */
static void clear_context (void)
{
    ir_result_t const target    = {0};
    s3_tlm_t const    telemetry = {0};

    g_state.b_navigation_pending = false;
    ir_processing_reset(&g_line);
    barcode_decoder_reset(&g_decoder);
    g_target                  = target;
    g_telemetry               = telemetry;
    g_telemetry.line_state    = g_state.b_fault ? S3_LINE_FAULT : S3_LINE_IDLE;
    g_telemetry.decoder_state = S3_DECODER_WAIT_CLEAR;
}

/** Preserve S1's established magnitude/sign and physical-channel bit order. */
static void update_line (ir_result_t result)
{
    g_telemetry.b_line_valid = result.b_target_valid;
    g_telemetry.line_pos     = 0u;
    g_telemetry.b_line_left  = 0u;
    g_telemetry.ir_pattern   = ((g_line.dark_mask & 1u) << 2u) |
                             (g_line.dark_mask & 2u) |
                             ((g_line.dark_mask & 4u) >> 2u);
    if (result.b_fault || g_state.b_fault)
    {
        g_telemetry.line_state = S3_LINE_FAULT;
        return;
    }

    if (result.b_target_valid)
    {
        g_telemetry.line_state  = S3_LINE_TRACKING;
        g_telemetry.b_line_left = result.correction < IR_STEERING_LIMIT;
        g_telemetry.line_pos    = (result.correction < IR_STEERING_LIMIT)
                                      ? IR_STEERING_LIMIT - result.correction
                                      : result.correction - IR_STEERING_LIMIT;
        return;
    }

    if (result.b_lost)
    {
        g_telemetry.line_state = S3_LINE_LOST;
        return;
    }

    switch (g_permission.mode)
    {
        case IR_MODE_TRACKING:
            /* Retain an already reported loss until tracking resumes. */
            if (S3_LINE_LOST != g_telemetry.line_state)
            {
                g_telemetry.line_state = S3_LINE_TRACKING;
            }
        break;

        case IR_MODE_RECOVERY:
            g_telemetry.line_state = S3_LINE_REACQUIRING;
        break;

        case IR_MODE_SUSPENDED:
            g_telemetry.line_state = S3_LINE_SUSPENDED;
        break;

        default:
            g_telemetry.line_state = S3_LINE_IDLE;
        break;
    }
}

/** Map private states explicitly; their enum ordinals differ from IF-07. */
static void update_decoder (barcode_result_t result)
{
    switch (g_decoder.state)
    {
        case BARCODE_IDLE:
            g_telemetry.decoder_state = S3_DECODER_SEARCHING;
        break;

        case BARCODE_CAPTURING:
            g_telemetry.decoder_state = S3_DECODER_CAPTURING;
        break;

        default:
            g_telemetry.decoder_state = S3_DECODER_WAIT_CLEAR;
        break;
    }

    if (BARCODE_NONE != result.outcome)
    {
        g_telemetry.b_barcode = BARCODE_ACCEPTED == result.outcome;
    }

    if ((BARCODE_ACCEPTED == result.outcome) ||
        (BARCODE_UNBOUND == result.outcome))
    {
        g_telemetry.last_symbol = result.symbol;
        g_telemetry.last_action = result.action;
    }
}

/** Resolve instructor bindings through S1; no private action defaults. */
static bool lookup_action (void * p_context, uint8_t symbol, uint16_t version,
                           uint8_t * p_action)
{
    sm_action_t action = SM_ACTION_LEFT;
    (void)p_context;
    (void)version;
    if (!sm_barcode_lookup(symbol, &action))
    {
        return false;
    }
    /* S1 returns one of the four validated action codes. */
    *p_action = (uint8_t)action;
    return true;
}

/* End of file. */

