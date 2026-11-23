/** @file ir_frame.h
 * @brief Private, lossless frame format and injected acquisition seam.
 */
#ifndef IR_FRAME_H
#define IR_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define IR_CHANNEL_COUNT 3u
#define IR_ADC_MAX 4095u
#define IR_VALID_ALL 7u
#define IR_FRAME_BYTES 11u
#define IR_TIME_HALF_RANGE 32768u

/** Native scratch values only; never queue or transmit this structure. */
typedef struct
{
    uint16_t raw[IR_CHANNEL_COUNT];
    uint16_t time_ms;
    uint16_t sequence;
    uint16_t generation;
    uint8_t  valid_mask;
} ir_sample_t;

/** 87 meaningful bits, least significant bit first; bit 87 is reserved. */
typedef struct
{
    uint8_t bytes[IR_FRAME_BYTES];
} ir_frame_t;

typedef uint16_t (*ir_clock_fn_t)(void * p_context);
typedef bool (*ir_read_fn_t)(void * p_context, uint8_t channel,
                             uint16_t * p_raw);

typedef struct
{
    ir_clock_fn_t p_clock;
    ir_read_fn_t  p_read;
    void *        p_context;
    uint16_t      next_sequence;
} ir_source_t;

/** Encode atomically; reject invalid ranges and leave output unchanged. */
bool ir_frame_pack (ir_sample_t const * p_sample, ir_frame_t * p_frame);
/** Decode exact length and reserved bits before committing output. */
bool ir_frame_unpack (uint8_t const * p_bytes, uint32_t length,
                      ir_sample_t * p_sample);
/** Timestamp before the scan; mark every failed or out-of-range read. */
bool ir_frame_acquire (ir_source_t * p_source, uint16_t generation,
                       uint16_t max_scan_ms, ir_frame_t * p_frame);
/** Valid only if the external outage bound is less than half a wrap. */
bool ir_frame_age (uint16_t now_ms, uint16_t then_ms, uint16_t limit_ms);
/** Modular subtraction, including the 65535 to zero transition. */
uint16_t ir_frame_elapsed (uint16_t now_ms, uint16_t then_ms);

#endif
/* End of file. */

