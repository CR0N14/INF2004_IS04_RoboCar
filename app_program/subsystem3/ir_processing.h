/** @file ir_processing.h
 * @brief Single-owner, pure line processing; never writes motor PWM.
 */
#ifndef IR_PROCESSING_H
#define IR_PROCESSING_H

#include "ir_frame.h"

#define IR_STEERING_LIMIT 100

typedef enum
{
    IR_MODE_DISABLED = 0,
    IR_MODE_TRACKING,
    IR_MODE_SUSPENDED,
    IR_MODE_RECOVERY
} ir_mode_t;

typedef struct
{
    unsigned int white : 12;
    unsigned int black : 12;
} ir_calibration_t;

typedef struct
{
    ir_calibration_t calibration[IR_CHANNEL_COUNT];
    unsigned int     left_channel     : 2;
    unsigned int     centre_channel   : 2;
    unsigned int     barcode_channel  : 2;
    unsigned int     minimum_contrast : 12;
    unsigned int     minimum_signal   : 13;
    unsigned int     dark_on          : 12;
    unsigned int     dark_off         : 12;
    unsigned int     max_gap_ms       : 15;
    unsigned int     persistence_ms   : 15;
} ir_config_t;

/** The adapter must replace generation on stop, fault or route change. */
typedef struct
{
    uint16_t     generation;
    unsigned int mode                   : 2;
    bool         b_enabled              : 1;
    bool         b_junction_expected    : 1;
    bool         b_recovery_expected    : 1;
    bool         b_barcode_gap_expected : 1;
} ir_permission_t;

/** Offset encoding gives -100..100 exactly 201 values in eight bits. */
typedef struct
{
    uint16_t     generation;
    uint16_t     time_ms;
    unsigned int correction     : 8;
    bool         b_target_valid : 1;
    bool         b_fault        : 1;
    bool         b_lost         : 1;
    bool         b_reacquired   : 1;
    bool         b_junction     : 1;
} ir_result_t;

typedef struct
{
    uint16_t since_ms;
    bool     b_active   : 1;
    bool     b_reported : 1;
} ir_persistence_t;

typedef struct
{
    ir_config_t      config;
    ir_persistence_t loss;
    ir_persistence_t recovery;
    ir_persistence_t junction;
    uint16_t         generation;
    uint16_t         last_time_ms;
    uint16_t         last_sequence;
    unsigned int     dark_mask     : 3;
    unsigned int     last_mode     : 2;
    bool             b_initialised : 1;
    bool             b_have_frame  : 1;
} ir_processing_t;

/** Reject duplicate roles, unusable calibration and invalid timing. */
bool ir_processing_init (ir_processing_t *   p_line,
                         ir_config_t const * p_config);
/** Normalise either polarity; invalid output is never substituted by zero. */
bool ir_processing_normalise (ir_calibration_t calibration, uint16_t raw,
                              uint16_t minimum_contrast, uint16_t * p_darkness);
/** Invalidate all temporal evidence on a controller wake, even without ADC. */
void ir_processing_reset (ir_processing_t * p_line);
/** Evaluate one fresh frame; events are edges, not queued commands. */
ir_result_t ir_processing_step (ir_processing_t *  p_line,
                                ir_frame_t const * p_frame,
                                ir_permission_t permission, uint16_t now_ms);

#endif
/* End of file. */

