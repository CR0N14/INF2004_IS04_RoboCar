/** @file ir_config.h
 * @brief Integration settings; widths and scan speeds require car validation.
 */
#ifndef IR_CONFIG_H
#define IR_CONFIG_H
#include "barcode_decoder.h"
#define IR_PERIOD_MS 10u
#define IR_MAX_GAP_MS 60u
#define IR_CONTRAST 200u
#define IR_PERSIST_MS 50u
#define IR_RECOVERY_MS 3000u
void ir_config_line (ir_calibration_t const * p_calibration,
                      ir_config_t * p_line);
barcode_config_t ir_config_barcode (ir_calibration_t calibration);
#endif
/* End of file. */

