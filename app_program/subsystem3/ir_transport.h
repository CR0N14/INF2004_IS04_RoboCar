/** @file ir_transport.h
 * @brief Task-context transport prototype with injected SMP-safe locking.
 */
#ifndef IR_TRANSPORT_H
#define IR_TRANSPORT_H

#include "ir_processing.h"

#define IR_QUEUE_CAPACITY 4u

typedef bool (*ir_lock_fn_t)(void * p_context);
typedef void (*ir_unlock_fn_t)(void * p_context);

typedef struct
{
    ir_frame_t     frames[IR_QUEUE_CAPACITY];
    ir_result_t    latest;
    ir_lock_fn_t   p_lock;
    ir_unlock_fn_t p_unlock;
    void *         p_context;
    uint16_t       generation;
    uint16_t       fault_token;
    uint16_t       dropped;
    unsigned int   head              : 2;
    unsigned int   count             : 3;
    bool           b_inhibited       : 1;
    bool           b_fault           : 1;
    bool           b_token_exhausted : 1;
    bool           b_drop_saturated  : 1;
} ir_transport_t;

/** Create before tasks; the lock must cover both cores and never be an ISR. */
bool ir_transport_init (ir_transport_t * p_transport, ir_lock_fn_t p_lock,
                        ir_unlock_fn_t p_unlock, void * p_context);
/** Drop newest on full; sequence gaps expose that loss to the decoder. */
bool ir_transport_offer (ir_transport_t *   p_transport,
                         ir_frame_t const * p_frame);
/** Copy one whole frame under the same lock; an empty queue never blocks. */
bool ir_transport_take (ir_transport_t * p_transport, ir_frame_t * p_frame);
/** Stop dominates: drain frames and invalidate the target under one lock. */
bool ir_transport_stop (ir_transport_t * p_transport, uint16_t generation,
                        bool b_fault);
/** Only an explicit matching recovery may clear a fault; no automatic retry. */
bool ir_transport_resume (ir_transport_t * p_transport, uint16_t generation,
                          uint16_t fault_token, bool b_inputs_ready);
/** Publish only if permission still matches while holding the lock. */
bool ir_transport_publish (ir_transport_t *    p_transport,
                           ir_result_t const * p_result);
/** A failed or stale snapshot never leaves a caller with a valid target. */
bool ir_transport_snapshot (ir_transport_t * p_transport, uint16_t now_ms,
                            uint16_t expiry_ms, ir_result_t * p_result);

#endif
/* End of file. */

