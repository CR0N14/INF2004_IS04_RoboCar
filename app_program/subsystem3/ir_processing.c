/** @file ir_processing.c
 * @brief FR28-37: calibrated steering and context-qualified persistence.
 */
#include "ir_processing.h"
#include <stddef.h>

static void clear_timer (ir_persistence_t * p_timer);
static bool persist (ir_persistence_t * p_timer, bool b_condition,
                     uint16_t time_ms, uint16_t duration_ms);
static bool read_darkness (ir_processing_t *   p_line,
                           ir_sample_t const * p_sample, uint16_t * p_darkness);
static void classify (ir_processing_t * p_line, uint16_t const * p_darkness,
                      ir_permission_t permission, ir_result_t * p_result);

bool ir_processing_normalise (ir_calibration_t calibration, uint16_t raw,
                              uint16_t minimum_contrast, uint16_t * p_darkness)
{
    /* Twelve-bit endpoints yield span +/-4095 and product +/-16769025. */
    int32_t const span =
        (int32_t)calibration.black - (int32_t)calibration.white;
    int32_t const contrast = (span < 0) ? -span : span;
    int32_t       darkness = 0;

    if ((NULL == p_darkness) || (raw > IR_ADC_MAX) ||
        (0u == minimum_contrast) || (contrast < minimum_contrast))
    {
        return false;
    }

    darkness =
        (((int32_t)raw - (int32_t)calibration.white) * (int32_t)IR_ADC_MAX) /
        span;
    if (darkness < 0)
    {
        darkness = 0;
    }

    if (darkness > (int32_t)IR_ADC_MAX)
    {
        darkness = IR_ADC_MAX;
    }

    /* Clipping above proves the twelve-bit result fits. */
    *p_darkness = (uint16_t)darkness;
    return true;
}

void ir_processing_reset (ir_processing_t * p_line)
{
    if (NULL != p_line)
    {
        clear_timer(&p_line->loss);
        clear_timer(&p_line->recovery);
        clear_timer(&p_line->junction);
        p_line->dark_mask    = 0u;
        p_line->b_have_frame = false;
    }
}

bool ir_processing_init (ir_processing_t * p_line, ir_config_t const * p_config)
{
    ir_processing_t const empty    = {0};
    uint16_t              darkness = 0u;

    if (NULL == p_line)
    {
        return false;
    }

    *p_line = empty;
    if ((NULL == p_config) || (p_config->left_channel >= IR_CHANNEL_COUNT) ||
        (p_config->centre_channel >= IR_CHANNEL_COUNT) ||
        (p_config->barcode_channel >= IR_CHANNEL_COUNT) ||
        (p_config->left_channel == p_config->centre_channel) ||
        (p_config->left_channel == p_config->barcode_channel) ||
        (p_config->centre_channel == p_config->barcode_channel) ||
        (p_config->dark_off >= p_config->dark_on) ||
        (0u == p_config->minimum_signal) ||
        (p_config->minimum_signal > (2u * IR_ADC_MAX)) ||
        (0u == p_config->max_gap_ms) ||
        (p_config->max_gap_ms >= IR_TIME_HALF_RANGE) ||
        (0u == p_config->persistence_ms) ||
        (p_config->persistence_ms >= IR_TIME_HALF_RANGE))
    {
        return false;
    }

    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        if (!ir_processing_normalise(p_config->calibration[channel], 0u,
                                     p_config->minimum_contrast, &darkness))
        {
            return false;
        }
    }

    p_line->config        = *p_config;
    p_line->b_initialised = true;
    return true;
}

ir_result_t ir_processing_step (ir_processing_t *  p_line,
                                ir_frame_t const * p_frame,
                                ir_permission_t permission, uint16_t now_ms)
{
    ir_result_t result                     = {0};
    ir_sample_t sample                     = {0};
    uint16_t    darkness[IR_CHANNEL_COUNT] = {0u};
    bool        b_contiguous               = false;

    result.generation = permission.generation;
    if ((NULL == p_line) || (!p_line->b_initialised))
    {
        result.b_fault = true;
        return result;
    }

    if ((!permission.b_enabled) || (IR_MODE_DISABLED == permission.mode))
    {
        ir_processing_reset(p_line);
        return result;
    }

    if ((NULL == p_frame) ||
        (!ir_frame_unpack(p_frame->bytes, IR_FRAME_BYTES, &sample)) ||
        (sample.generation != permission.generation) ||
        (!ir_frame_age(now_ms, sample.time_ms, p_line->config.max_gap_ms)) ||
        (IR_VALID_ALL != sample.valid_mask))
    {
        ir_processing_reset(p_line);
        result.b_fault = true;
        return result;
    }

    b_contiguous =
        p_line->b_have_frame && (p_line->generation == permission.generation) &&
        (p_line->last_mode == permission.mode) &&
        (1u == ir_frame_elapsed(sample.sequence, p_line->last_sequence)) &&
        (0u != ir_frame_elapsed(sample.time_ms, p_line->last_time_ms)) &&
        ir_frame_age(sample.time_ms, p_line->last_time_ms,
                     p_line->config.max_gap_ms);
    if (!b_contiguous)
    {
        bool const b_same_context =
            p_line->b_have_frame &&
            (p_line->generation == permission.generation) &&
            (p_line->last_mode == permission.mode);

        ir_processing_reset(p_line);
        if (b_same_context)
        {
            result.b_fault = true;
            return result;
        }
    }

    p_line->generation    = permission.generation;
    p_line->last_mode     = permission.mode;
    p_line->last_time_ms  = sample.time_ms;
    p_line->last_sequence = sample.sequence;
    p_line->b_have_frame  = true;
    result.time_ms        = sample.time_ms;
    result.b_fault        = !read_darkness(p_line, &sample, darkness);
    if (!result.b_fault)
    {
        classify(p_line, darkness, permission, &result);
    }

    return result;
}

/** Reset one persistence timer. */
static void clear_timer (ir_persistence_t * p_timer)
{
    ir_persistence_t const empty = {0u, false, false};

    *p_timer = empty;
}

/** Emit once after continuous evidence, measured from its first sample. */
static bool persist (ir_persistence_t * p_timer, bool b_condition,
                     uint16_t time_ms, uint16_t duration_ms)
{
    if (!b_condition)
    {
        clear_timer(p_timer);
        return false;
    }

    if (!p_timer->b_active)
    {
        p_timer->since_ms = time_ms;
        p_timer->b_active = true;
    }

    if ((!p_timer->b_reported) &&
        (ir_frame_elapsed(time_ms, p_timer->since_ms) >= duration_ms))
    {
        p_timer->b_reported = true;
        return true;
    }

    return false;
}

/** Apply Schmitt thresholds without filtering away narrow barcode bars. */
static bool read_darkness (ir_processing_t *   p_line,
                           ir_sample_t const * p_sample, uint16_t * p_darkness)
{
    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        if (!ir_processing_normalise(
                p_line->config.calibration[channel], p_sample->raw[channel],
                p_line->config.minimum_contrast, &p_darkness[channel]))
        {
            return false;
        }

        if (p_darkness[channel] >= p_line->config.dark_on)
        {
            p_line->dark_mask |= 1u << channel;
        }

        if (p_darkness[channel] <= p_line->config.dark_off)
        {
            p_line->dark_mask &= ~(1u << channel);
        }
    }

    return true;
}

/** Classify tracker evidence; the barcode channel cannot steer the car. */
static void classify (ir_processing_t * p_line, uint16_t const * p_darkness,
                      ir_permission_t permission, ir_result_t * p_result)
{
    uint16_t const     left     = p_darkness[p_line->config.left_channel];
    uint16_t const     centre   = p_darkness[p_line->config.centre_channel];
    uint16_t const     total    = left + centre;
    unsigned int const trackers = (1u << p_line->config.left_channel) |
                                  (1u << p_line->config.centre_channel);
    bool const b_crossing = (p_line->dark_mask & trackers) == trackers;
    bool const b_line =
        (total >= p_line->config.minimum_signal) && (!b_crossing);
    bool const     b_tracking = IR_MODE_TRACKING == permission.mode;
    uint16_t const duration   = p_line->config.persistence_ms;

    p_result->b_lost = persist(&p_line->loss,
                               b_tracking && (!b_line) && (!b_crossing) &&
                                   (!permission.b_barcode_gap_expected),
                               p_result->time_ms, duration);
    p_result->b_reacquired =
        persist(&p_line->recovery,
                (IR_MODE_RECOVERY == permission.mode) &&
                    permission.b_recovery_expected && b_line,
                p_result->time_ms, duration);
    p_result->b_junction =
        persist(&p_line->junction,
                b_tracking && permission.b_junction_expected && b_crossing,
                p_result->time_ms, duration);

    if (b_tracking && b_line)
    {
        /* Difference * 100 fits signed 20 bits; quotient is -100..100. */
        int32_t const correction =
            (((int32_t)centre - (int32_t)left) * IR_STEERING_LIMIT) / total;

        /* Offset is 0..200, representable in the eight-bit field. */
        p_result->correction = (unsigned int)(correction + IR_STEERING_LIMIT);
        p_result->b_target_valid = true;
    }
}

/* End of file. */

