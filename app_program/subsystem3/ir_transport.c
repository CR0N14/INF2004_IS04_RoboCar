/** @file ir_transport.c
 * @brief Tribunal C: inhibit and fault state do not depend on queue space.
 */
#include "ir_transport.h"
#include <stddef.h>

static void count_drop (ir_transport_t * p_transport);
static bool enter (ir_transport_t * p_transport);

bool ir_transport_init (ir_transport_t * p_transport, ir_lock_fn_t p_lock,
                        ir_unlock_fn_t p_unlock, void * p_context)
{
    ir_transport_t const empty = {0};

    if ((NULL == p_transport) || (NULL == p_lock) || (NULL == p_unlock))
    {
        return false;
    }

    *p_transport             = empty;
    p_transport->p_lock      = p_lock;
    p_transport->p_unlock    = p_unlock;
    p_transport->p_context   = p_context;
    p_transport->b_inhibited = true;
    return true;
}

bool ir_transport_offer (ir_transport_t *   p_transport,
                         ir_frame_t const * p_frame)
{
    ir_sample_t sample = {0};
    bool        b_sent = false;

    if ((NULL == p_frame) ||
        (!ir_frame_unpack(p_frame->bytes, IR_FRAME_BYTES, &sample)) ||
        (!enter(p_transport)))
    {
        return false;
    }

    if ((!p_transport->b_inhibited) &&
        (sample.generation == p_transport->generation))
    {
        if (p_transport->count < IR_QUEUE_CAPACITY)
        {
            unsigned int const tail =
                (p_transport->head + p_transport->count) % IR_QUEUE_CAPACITY;

            p_transport->frames[tail] = *p_frame;
            ++p_transport->count;
            b_sent = true;
        }
        else
        {
            count_drop(p_transport);
        }
    }

    p_transport->p_unlock(p_transport->p_context);
    return b_sent;
}

bool ir_transport_take (ir_transport_t * p_transport, ir_frame_t * p_frame)
{
    bool b_received = false;

    if ((NULL == p_frame) || (!enter(p_transport)))
    {
        return false;
    }

    if ((!p_transport->b_inhibited) && (0u != p_transport->count))
    {
        *p_frame          = p_transport->frames[p_transport->head];
        p_transport->head = (p_transport->head + 1u) % IR_QUEUE_CAPACITY;
        --p_transport->count;
        b_received = true;
    }

    p_transport->p_unlock(p_transport->p_context);
    return b_received;
}

bool ir_transport_stop (ir_transport_t * p_transport, uint16_t generation,
                        bool b_fault)
{
    if (!enter(p_transport))
    {
        return false;
    }

    p_transport->b_inhibited           = true;
    p_transport->latest.b_target_valid = false;
    p_transport->generation            = generation;
    p_transport->head                  = 0u;
    p_transport->count                 = 0u;
    if (b_fault)
    {
        p_transport->b_fault = true;
        if (UINT16_MAX != p_transport->fault_token)
        {
            ++p_transport->fault_token;
        }
        else
        {
            p_transport->b_token_exhausted = true;
        }
    }

    p_transport->p_unlock(p_transport->p_context);
    return true;
}

bool ir_transport_resume (ir_transport_t * p_transport, uint16_t generation,
                          uint16_t fault_token, bool b_inputs_ready)
{
    bool b_resumed = false;

    if (!enter(p_transport))
    {
        return false;
    }

    if (b_inputs_ready && (!p_transport->b_token_exhausted) &&
        (generation == p_transport->generation) &&
        (fault_token == p_transport->fault_token))
    {
        p_transport->b_inhibited = false;
        p_transport->b_fault     = false;
        b_resumed                = true;
    }

    p_transport->p_unlock(p_transport->p_context);
    return b_resumed;
}

bool ir_transport_publish (ir_transport_t *    p_transport,
                           ir_result_t const * p_result)
{
    bool b_published = false;

    if ((NULL == p_result) || (p_result->correction > 200u) ||
        (!enter(p_transport)))
    {
        return false;
    }

    if ((!p_transport->b_inhibited) &&
        (p_result->generation == p_transport->generation))
    {
        p_transport->latest = *p_result;
        b_published         = true;
    }

    p_transport->p_unlock(p_transport->p_context);
    return b_published;
}

bool ir_transport_snapshot (ir_transport_t * p_transport, uint16_t now_ms,
                            uint16_t expiry_ms, ir_result_t * p_result)
{
    ir_result_t const empty = {0};

    if (NULL == p_result)
    {
        return false;
    }

    *p_result = empty;
    if (!enter(p_transport))
    {
        return false;
    }

    *p_result = p_transport->latest;
    p_result->b_target_valid =
        p_result->b_target_valid && (!p_transport->b_inhibited) &&
        (p_result->generation == p_transport->generation) &&
        ir_frame_age(now_ms, p_result->time_ms, expiry_ms);
    p_transport->p_unlock(p_transport->p_context);
    return true;
}

/** A failed lock is a failed operation, never permission to continue. */
static bool enter (ir_transport_t * p_transport)
{
    return (NULL != p_transport) && (NULL != p_transport->p_lock) &&
           (NULL != p_transport->p_unlock) &&
           p_transport->p_lock(p_transport->p_context);
}

/** Saturate diagnostics instead of wrapping away evidence of loss. */
static void count_drop (ir_transport_t * p_transport)
{
    if (UINT16_MAX != p_transport->dropped)
    {
        ++p_transport->dropped;
    }
    else
    {
        p_transport->b_drop_saturated = true;
    }
}

/* End of file. */

