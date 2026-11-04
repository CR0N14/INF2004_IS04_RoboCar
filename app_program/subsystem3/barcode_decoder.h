/** @file barcode_decoder.h
 * @brief Bounded encounter capture with default single-letter Code 39 decoding.
 */
#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include "ir_processing.h"

#define BARCODE_RUN_CAPACITY 32u
#define BARCODE_SYMBOL_COUNT 26u
#define BARCODE_ACTION_COUNT 4u
#define BARCODE_RUN_DARK 0x8000u
#define BARCODE_RUN_WIDTH 0x7fffu

typedef enum
{
    BARCODE_WAIT_CLEAR = 0,
    BARCODE_IDLE,
    BARCODE_CAPTURING
} barcode_state_t;

typedef enum
{
    BARCODE_NONE = 0,
    BARCODE_ACCEPTED,
    BARCODE_UNSUPPORTED_CODEBOOK,
    BARCODE_INVALID_CODE,
    BARCODE_UNBOUND,
    BARCODE_GAP,
    BARCODE_STALE,
    BARCODE_SENSOR_FAULT,
    BARCODE_TIMEOUT,
    BARCODE_OVERFLOW,
    BARCODE_DIRECTION,
    BARCODE_INTERRUPTED,
    BARCODE_INCOMPLETE
} barcode_outcome_t;

typedef enum
{
    BARCODE_DIRECTION_UNKNOWN = 0,
    BARCODE_FORWARD,
    BARCODE_REVERSE
} barcode_direction_t;

/** Snapshot already associated by Motion with this frame's acquisition time. */
typedef struct
{
    uint16_t     time_ms;
    uint16_t     generation;
    unsigned int direction : 2;
    bool         b_valid   : 1;
} barcode_motion_t;

/** Runs contain level in bit 15 and elapsed milliseconds in bits 0..14. */
typedef bool (*barcode_recognise_fn_t)(void *           p_context,
                                       uint16_t const * p_runs, uint8_t count,
                                       barcode_direction_t direction,
                                       uint8_t *           p_symbol);
typedef bool (*barcode_bind_fn_t)(void * p_context, uint8_t symbol,
                                  uint16_t binding_version, uint8_t * p_action);

typedef struct
{
    ir_calibration_t       calibration;
    barcode_recognise_fn_t p_recognise;
    barcode_bind_fn_t      p_bind;
    void *                 p_context;
    unsigned int           max_gap_ms         : 15;
    unsigned int           max_capture_ms     : 15;
    unsigned int           clearance_ms       : 15;
    unsigned int           max_motion_skew_ms : 15;
    unsigned int           channel            : 2;
    unsigned int           minimum_contrast   : 12;
    unsigned int           dark_on            : 12;
    unsigned int           dark_off           : 12;
    unsigned int           minimum_runs       : 6;
} barcode_config_t;

typedef struct
{
    uint16_t     generation;
    uint16_t     time_ms;
    uint16_t     binding_version;
    unsigned int outcome : 4;
    unsigned int symbol  : 5;
    unsigned int action  : 2;
} barcode_result_t;

typedef struct
{
    barcode_config_t config;
    uint16_t         runs[BARCODE_RUN_CAPACITY];
    uint16_t         generation;
    uint16_t         binding_version;
    uint16_t         last_time_ms;
    uint16_t         last_sequence;
    uint16_t         started_ms;
    uint16_t         edge_ms;
    uint16_t         clear_since_ms;
    unsigned int     state         : 2;
    unsigned int     count         : 6;
    unsigned int     direction     : 2;
    bool             b_initialised : 1;
    bool             b_have_frame  : 1;
    bool             b_dark        : 1;
    bool             b_clearing    : 1;
} barcode_decoder_t;

/** Initialise inhibited until sustained background has been observed. */
bool barcode_decoder_init (barcode_decoder_t *      p_decoder,
                           barcode_config_t const * p_config);
/** Call on stop/reset wake even if there are no arriving frames. */
void barcode_decoder_reset (barcode_decoder_t * p_decoder);
/** One bounded frame step; accepted means request, never permission to move. */
barcode_result_t
barcode_decoder_step (barcode_decoder_t * p_decoder, ir_frame_t const * p_frame,
                      ir_permission_t permission, uint16_t binding_version,
                      barcode_motion_t motion, uint16_t now_ms);

#endif
/* End of file. */

