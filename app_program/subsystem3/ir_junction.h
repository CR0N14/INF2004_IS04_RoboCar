/** @file ir_junction.h
 * @brief A new barcode needs a subsequent stable line approach.
 */
#ifndef IR_JUNCTION_H
#define IR_JUNCTION_H
#include "ir_frame.h"
typedef struct
{
    uint16_t since_ms;
    bool b_active : 1;
    bool b_qualified : 1;
} ir_junction_t;
void ir_junction_reset (ir_junction_t * p_junction);
void ir_junction_observe (ir_junction_t * p_junction, bool b_line,
                          bool b_lost, uint16_t time_ms);
#endif
/* End of file. */

