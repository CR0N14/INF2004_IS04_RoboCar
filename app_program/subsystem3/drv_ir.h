/** @file drv_ir.h
 * @brief ADC access exclusively owned by the IR acquisition task.
 */
#ifndef DRV_IR_H
#define DRV_IR_H
#include "../app.h"
#include "ir_frame.h"
typedef struct
{
    ID adc;
    bool b_clock_failed;
} drv_ir_t;
ER drv_ir_open (drv_ir_t * p_driver);
uint16_t drv_ir_clock (void * p_context);
bool drv_ir_read (void * p_context, uint8_t channel, uint16_t * p_raw);
#endif
/* End of file. */

