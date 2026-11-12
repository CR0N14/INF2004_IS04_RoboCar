/** @file ir_junction.c
 * @brief Both-black at startup does not establish a junction approach.
 */
#include "ir_junction.h"
#include "ir_config.h"
void ir_junction_reset (ir_junction_t * p_junction)
{
    ir_junction_t const empty = {0};
    *p_junction = empty;
}
void ir_junction_observe (ir_junction_t * p_junction, bool b_line,
                          bool b_lost, uint16_t time_ms)
{
    if (b_lost)
    {
        ir_junction_reset(p_junction);
    }
    else if (b_line)
    {
        if (!p_junction->b_active)
        {
            p_junction->b_active = true;
            p_junction->since_ms = time_ms;
        }
        if (ir_frame_elapsed(time_ms, p_junction->since_ms) >= IR_PERSIST_MS)
        {
            p_junction->b_qualified = true;
        }
    }
    else if (!p_junction->b_qualified)
    {
        p_junction->b_active = false;
    }
}
/* End of file. */

