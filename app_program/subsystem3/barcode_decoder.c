/** @file barcode_decoder.c
 * @brief Tribunal C: gaps, stop and restart invalidate the entire encounter.
 */
#include "barcode_decoder.h"
#include "code39.h"
#include <stddef.h>

static bool              motion_matches (barcode_decoder_t const * p_decoder,
                                         ir_sample_t const *       p_sample,
                                         barcode_motion_t          motion);
static barcode_outcome_t validate_frame (barcode_decoder_t const * p_decoder,
                                         ir_sample_t const *       p_sample,
                                         ir_permission_t           permission,
                                         uint16_t binding_version,
                                         uint16_t now_ms);
static barcode_outcome_t append_run (barcode_decoder_t * p_decoder,
                                     uint16_t            time_ms);
static barcode_outcome_t recognise (barcode_decoder_t * p_decoder,
                                    barcode_result_t *  p_result);
static void clear_encounter (barcode_decoder_t * p_decoder, bool b_dark,
                             uint16_t time_ms);
static barcode_outcome_t capture_step (barcode_decoder_t * p_decoder,
                                       bool b_dark, uint16_t time_ms,
                                       barcode_motion_t   motion,
                                       barcode_result_t * p_result);
static barcode_outcome_t advance (barcode_decoder_t * p_decoder, bool b_dark,
                                  ir_sample_t const * p_sample,
                                  barcode_motion_t    motion,
                                  barcode_result_t *  p_result);

void barcode_decoder_reset (barcode_decoder_t * p_decoder)
{
    if (NULL != p_decoder)
    {
        p_decoder->state        = BARCODE_WAIT_CLEAR;
        p_decoder->count        = 0u;
        p_decoder->b_have_frame = false;
        p_decoder->b_clearing   = false;
        /* Unknown reflectance cannot count as proven background. */
        p_decoder->b_dark = true;
    }
}

bool barcode_decoder_init (barcode_decoder_t *      p_decoder,
                           barcode_config_t const * p_config)
{
    barcode_decoder_t const empty    = {0};
    uint16_t                darkness = 0u;

    if (NULL == p_decoder)
    {
        return false;
    }

    *p_decoder = empty;
    if ((NULL == p_config) || (p_config->channel >= IR_CHANNEL_COUNT) ||
        (p_config->dark_off >= p_config->dark_on) ||
        (0u == p_config->max_gap_ms) ||
        (p_config->max_gap_ms >= IR_TIME_HALF_RANGE) ||
        (0u == p_config->clearance_ms) ||
        (p_config->clearance_ms >= p_config->max_capture_ms) ||
        (p_config->max_capture_ms >= IR_TIME_HALF_RANGE) ||
        (p_config->max_motion_skew_ms >= IR_TIME_HALF_RANGE) ||
        (0u == p_config->minimum_runs) ||
        (p_config->minimum_runs > BARCODE_RUN_CAPACITY))
    {
        return false;
    }

    if (!ir_processing_normalise(p_config->calibration, 0u,
                                 p_config->minimum_contrast, &darkness))
    {
        return false;
    }

    p_decoder->config        = *p_config;
    p_decoder->b_initialised = true;
    barcode_decoder_reset(p_decoder);
    return true;
}

barcode_result_t barcode_decoder_step (barcode_decoder_t * p_decoder,
                                       ir_frame_t const *  p_frame,
                                       ir_permission_t     permission,
                                       uint16_t            binding_version,
                                       barcode_motion_t motion, uint16_t now_ms)
{
    barcode_result_t result   = {0};
    ir_sample_t      sample   = {0};
    uint16_t         darkness = 0u;
    bool             b_dark   = false;

    result.generation      = permission.generation;
    result.binding_version = binding_version;
    if ((NULL == p_decoder) || (!p_decoder->b_initialised))
    {
        result.outcome = BARCODE_SENSOR_FAULT;
        return result;
    }

    if ((!permission.b_enabled) || (IR_MODE_TRACKING != permission.mode))
    {
        result.outcome = (BARCODE_CAPTURING == p_decoder->state)
                             ? BARCODE_INTERRUPTED
                             : BARCODE_NONE;
        barcode_decoder_reset(p_decoder);
        return result;
    }

    if ((NULL == p_frame) ||
        (!ir_frame_unpack(p_frame->bytes, IR_FRAME_BYTES, &sample)))
    {
        barcode_decoder_reset(p_decoder);
        result.outcome = BARCODE_SENSOR_FAULT;
        return result;
    }

    result.time_ms = sample.time_ms;
    /* All outcomes are 0..12 and fit the four-bit field. */
    result.outcome = (unsigned int)validate_frame(
        p_decoder, &sample, permission, binding_version, now_ms);
    if (BARCODE_NONE != result.outcome)
    {
        barcode_decoder_reset(p_decoder);
        return result;
    }

    if ((!motion_matches(p_decoder, &sample, motion)) ||
        (!ir_processing_normalise(p_decoder->config.calibration,
                                  sample.raw[p_decoder->config.channel],
                                  p_decoder->config.minimum_contrast,
                                  &darkness)))
    {
        barcode_decoder_reset(p_decoder);
        result.outcome = BARCODE_DIRECTION;
        return result;
    }

    p_decoder->generation      = permission.generation;
    p_decoder->binding_version = binding_version;
    p_decoder->last_time_ms    = sample.time_ms;
    p_decoder->last_sequence   = sample.sequence;
    p_decoder->b_have_frame    = true;
    b_dark                     = p_decoder->b_dark;
    if (darkness >= p_decoder->config.dark_on)
    {
        b_dark = true;
    }

    if (darkness <= p_decoder->config.dark_off)
    {
        b_dark = false;
    }

    /* All outcomes are 0..12 and fit the four-bit field. */
    result.outcome =
        (unsigned int)advance(p_decoder, b_dark, &sample, motion, &result);
    p_decoder->b_dark = b_dark;
    if (BARCODE_NONE != result.outcome)
    {
        /* Accept and reject both consume the encounter until new clearance. */
        barcode_decoder_reset(p_decoder);
    }

    return result;
}

/** Require a valid, generation-matched direction near the acquisition time. */
static bool motion_matches (barcode_decoder_t const * p_decoder,
                            ir_sample_t const *       p_sample,
                            barcode_motion_t          motion)
{
    uint16_t const skew = p_decoder->config.max_motion_skew_ms;
    bool const b_near = ir_frame_age(motion.time_ms, p_sample->time_ms, skew) ||
                        ir_frame_age(p_sample->time_ms, motion.time_ms, skew);

    return motion.b_valid && (motion.generation == p_sample->generation) &&
           ((BARCODE_FORWARD == motion.direction) ||
            (BARCODE_REVERSE == motion.direction)) &&
           b_near;
}

/** Validate before replacing the current frame cursor. */
static barcode_outcome_t validate_frame (barcode_decoder_t const * p_decoder,
                                         ir_sample_t const *       p_sample,
                                         ir_permission_t           permission,
                                         uint16_t binding_version,
                                         uint16_t now_ms)
{
    if (p_sample->generation != permission.generation)
    {
        return BARCODE_STALE;
    }

    if ((0u == (p_sample->valid_mask & (1u << p_decoder->config.channel))) ||
        (!ir_frame_age(now_ms, p_sample->time_ms,
                       p_decoder->config.max_gap_ms)))
    {
        return BARCODE_SENSOR_FAULT;
    }

    if (!p_decoder->b_have_frame)
    {
        return BARCODE_NONE;
    }

    if ((p_decoder->generation != permission.generation) ||
        (p_decoder->binding_version != binding_version))
    {
        return BARCODE_INTERRUPTED;
    }

    if ((1u !=
         ir_frame_elapsed(p_sample->sequence, p_decoder->last_sequence)) ||
        (0u == ir_frame_elapsed(p_sample->time_ms, p_decoder->last_time_ms)) ||
        (!ir_frame_age(p_sample->time_ms, p_decoder->last_time_ms,
                       p_decoder->config.max_gap_ms)))
    {
        return BARCODE_GAP;
    }

    return BARCODE_NONE;
}

/** Retain completed colour runs; reject rather than truncate at capacity. */
static barcode_outcome_t append_run (barcode_decoder_t * p_decoder,
                                     uint16_t            time_ms)
{
    uint16_t const width = ir_frame_elapsed(time_ms, p_decoder->edge_ms);

    if ((0u == width) || (width > BARCODE_RUN_WIDTH))
    {
        return BARCODE_INCOMPLETE;
    }

    if (p_decoder->count >= BARCODE_RUN_CAPACITY)
    {
        return BARCODE_OVERFLOW;
    }

    p_decoder->runs[p_decoder->count] =
        width | (p_decoder->b_dark ? BARCODE_RUN_DARK : 0u);
    ++p_decoder->count;
    p_decoder->edge_ms = time_ms;
    return BARCODE_NONE;
}

/** Decode the symbol before consulting the current movement mapping. */
static barcode_outcome_t recognise (barcode_decoder_t * p_decoder,
                                    barcode_result_t *  p_result)
{
    uint8_t                  symbol   = 0u;
    uint8_t                  action   = 0u;
    barcode_config_t const * p_config = &p_decoder->config;

    if (p_decoder->count < p_config->minimum_runs)
    {
        return BARCODE_INCOMPLETE;
    }

    barcode_recognise_fn_t const p_recognise = (NULL == p_config->p_recognise)
                                                   ? code39_recognise
                                                   : p_config->p_recognise;

    /* Direction was checked against the two legal nonzero enum values. */
    if ((!p_recognise(p_config->p_context, p_decoder->runs, p_decoder->count,
                      (barcode_direction_t)p_decoder->direction, &symbol)) ||
        (symbol >= BARCODE_SYMBOL_COUNT))
    {
        return BARCODE_INVALID_CODE;
    }

    p_result->symbol = symbol;
    if ((NULL == p_config->p_bind) ||
        (!p_config->p_bind(p_config->p_context, symbol,
                           p_decoder->binding_version, &action)) ||
        (action >= BARCODE_ACTION_COUNT))
    {
        return BARCODE_UNBOUND;
    }

    p_result->symbol = symbol;
    p_result->action = action;
    return BARCODE_ACCEPTED;
}

/** Require sustained white space before starting another barcode. */
static void clear_encounter (barcode_decoder_t * p_decoder, bool b_dark,
                             uint16_t time_ms)
{
    if (b_dark)
    {
        p_decoder->b_clearing = false;
        return;
    }

    if (!p_decoder->b_clearing)
    {
        p_decoder->clear_since_ms = time_ms;
        p_decoder->b_clearing     = true;
    }

    if (ir_frame_elapsed(time_ms, p_decoder->clear_since_ms) >=
        p_decoder->config.clearance_ms)
    {
        p_decoder->state      = BARCODE_IDLE;
        p_decoder->b_clearing = false;
    }
}

/** Advance a capture by one sample, with bounded work and storage. */
static barcode_outcome_t capture_step (barcode_decoder_t * p_decoder,
                                       bool b_dark, uint16_t time_ms,
                                       barcode_motion_t   motion,
                                       barcode_result_t * p_result)
{
    barcode_outcome_t outcome = BARCODE_NONE;

    if (motion.direction != p_decoder->direction)
    {
        return BARCODE_DIRECTION;
    }

    if (ir_frame_elapsed(time_ms, p_decoder->started_ms) >=
        p_decoder->config.max_capture_ms)
    {
        return BARCODE_TIMEOUT;
    }

    if (b_dark != p_decoder->b_dark)
    {
        outcome           = append_run(p_decoder, time_ms);
        p_decoder->b_dark = b_dark;
    }

    if ((BARCODE_NONE == outcome) && (!b_dark) &&
        (ir_frame_elapsed(time_ms, p_decoder->edge_ms) >=
         p_decoder->config.clearance_ms))
    {
        outcome = recognise(p_decoder, p_result);
    }

    return outcome;
}

/** Apply one calibrated sample to the encounter state machine. */
static barcode_outcome_t advance (barcode_decoder_t * p_decoder, bool b_dark,
                                  ir_sample_t const * p_sample,
                                  barcode_motion_t    motion,
                                  barcode_result_t *  p_result)
{
    switch (p_decoder->state)
    {
        case BARCODE_WAIT_CLEAR:
            clear_encounter(p_decoder, b_dark, p_sample->time_ms);
        break;

        case BARCODE_IDLE:
            if (b_dark)
            {
                p_decoder->state      = BARCODE_CAPTURING;
                p_decoder->started_ms = p_sample->time_ms;
                p_decoder->edge_ms    = p_sample->time_ms;
                p_decoder->direction  = motion.direction;
                p_decoder->count      = 0u;
                p_decoder->b_dark     = true;
            }
        break;

        case BARCODE_CAPTURING:
            return capture_step(p_decoder, b_dark, p_sample->time_ms, motion,
                                p_result);

        default:
            return BARCODE_INTERRUPTED;
    }

    return BARCODE_NONE;
}

/* End of file. */

