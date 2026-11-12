/** @file ir_config.c
 * @brief Shared production/replay configuration without movement assignments.
 */
#include "ir_config.h"
#include "code39.h"
void ir_config_line (ir_calibration_t const * p_calibration,
                      ir_config_t * p_line)
{
    ir_config_t line = {0};
    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        line.calibration[channel] = p_calibration[channel];
    }
    line.left_channel = 0u;
    line.centre_channel = 1u;
    line.barcode_channel = 2u;
    line.minimum_contrast = IR_CONTRAST;
    line.minimum_signal = 1000u;
    line.dark_on = 2400u;
    line.dark_off = 1600u;
    line.max_gap_ms = IR_MAX_GAP_MS;
    line.persistence_ms = IR_PERSIST_MS;
    *p_line = line;
}
barcode_config_t ir_config_barcode (ir_calibration_t calibration)
{
    barcode_config_t code = {0};
    code.calibration = calibration;
    code.channel = 2u;
    code.minimum_contrast = IR_CONTRAST;
    code.dark_on = 2400u;
    code.dark_off = 1600u;
    code.max_gap_ms = IR_MAX_GAP_MS;
    code.max_capture_ms = 15000u;
    code.clearance_ms = 600u;
    code.max_motion_skew_ms = 40u;
    code.minimum_runs = CODE39_RUN_COUNT;
    return code;
}
/* End of file. */

