/** @file ir_frame.c
 * @brief Change Requirement 1: explicit bits, no structure wire images.
 */
#include "ir_frame.h"
#include <stddef.h>

static void     put_bits (uint8_t * p_bytes, uint8_t offset, uint8_t width,
                          uint16_t value);
static uint16_t get_bits (uint8_t const * p_bytes, uint8_t offset,
                          uint8_t width);

bool ir_frame_pack (ir_sample_t const * p_sample, ir_frame_t * p_frame)
{
    ir_frame_t frame = {{0u}};

    if ((NULL == p_sample) || (NULL == p_frame))
    {
        return false;
    }

    if (p_sample->valid_mask > IR_VALID_ALL)
    {
        return false;
    }

    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        if (p_sample->raw[channel] > IR_ADC_MAX)
        {
            return false;
        }

        put_bits(frame.bytes, channel * 12u, 12u, p_sample->raw[channel]);
    }

    put_bits(frame.bytes, 36u, 3u, p_sample->valid_mask);
    put_bits(frame.bytes, 39u, 16u, p_sample->time_ms);
    put_bits(frame.bytes, 55u, 16u, p_sample->sequence);
    put_bits(frame.bytes, 71u, 16u, p_sample->generation);
    *p_frame = frame;
    return true;
}

bool ir_frame_unpack (uint8_t const * p_bytes, uint32_t length,
                      ir_sample_t * p_sample)
{
    ir_sample_t sample = {{0u}, 0u, 0u, 0u, 0u};

    if ((NULL == p_bytes) || (NULL == p_sample) || (IR_FRAME_BYTES != length))
    {
        return false;
    }

    if (0u != (p_bytes[IR_FRAME_BYTES - 1u] & 0x80u))
    {
        return false;
    }

    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        sample.raw[channel] = get_bits(p_bytes, channel * 12u, 12u);
    }

    /* The three-bit field is representable in uint8_t. */
    sample.valid_mask = (uint8_t)get_bits(p_bytes, 36u, 3u);
    sample.time_ms    = get_bits(p_bytes, 39u, 16u);
    sample.sequence   = get_bits(p_bytes, 55u, 16u);
    sample.generation = get_bits(p_bytes, 71u, 16u);
    *p_sample         = sample;
    return true;
}

uint16_t ir_frame_elapsed (uint16_t now_ms, uint16_t then_ms)
{
    /* Conversion defines modulo 65536 after integer promotion. */
    return (uint16_t)(now_ms - then_ms);
}

bool ir_frame_age (uint16_t now_ms, uint16_t then_ms, uint16_t limit_ms)
{
    return (limit_ms < IR_TIME_HALF_RANGE) &&
           (ir_frame_elapsed(now_ms, then_ms) <= limit_ms);
}

bool ir_frame_acquire (ir_source_t * p_source, uint16_t generation,
                       uint16_t max_scan_ms, ir_frame_t * p_frame)
{
    ir_sample_t sample = {{0u}, 0u, 0u, 0u, 0u};

    if ((NULL == p_source) || (NULL == p_frame) ||
        (NULL == p_source->p_clock) || (NULL == p_source->p_read) ||
        (0u == max_scan_ms) || (max_scan_ms >= IR_TIME_HALF_RANGE))
    {
        return false;
    }

    sample.time_ms    = p_source->p_clock(p_source->p_context);
    sample.generation = generation;
    sample.sequence   = p_source->next_sequence;
    ++p_source->next_sequence;

    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        uint16_t   raw = 0u;
        bool const b_read =
            p_source->p_read(p_source->p_context, channel, &raw);

        if (b_read && (raw <= IR_ADC_MAX))
        {
            sample.raw[channel] = raw;
            sample.valid_mask |= 1u << channel;
        }
    }

    if (!ir_frame_age(p_source->p_clock(p_source->p_context), sample.time_ms,
                      max_scan_ms))
    {
        sample.valid_mask = 0u;
    }

    return ir_frame_pack(&sample, p_frame);
}

/** Insert a bounded field into private, zero-initialised storage. */
static void put_bits (uint8_t * p_bytes, uint8_t offset, uint8_t width,
                      uint16_t value)
{
    for (uint8_t bit = 0u; bit < width; ++bit)
    {
        uint8_t const position = offset + bit;
        /* The shift is 0..7, so the mask fits one byte. */
        uint8_t const mask = (uint8_t)(1u << (position % 8u));

        if (0u != (value & (1u << bit)))
        {
            p_bytes[position / 8u] |= mask;
        }
    }
}

/** Read at most sixteen bits from validated storage. */
static uint16_t get_bits (uint8_t const * p_bytes, uint8_t offset,
                          uint8_t width)
{
    uint16_t value = 0u;

    for (uint8_t bit = 0u; bit < width; ++bit)
    {
        uint8_t const position = offset + bit;

        if (0u != (p_bytes[position / 8u] & (1u << (position % 8u))))
        {
            value |= 1u << bit;
        }
    }

    return value;
}

/* End of file. */

