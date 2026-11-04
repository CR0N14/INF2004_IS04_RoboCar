/** @file subsystem3_test.c
 * @brief Pure-core tests; never reconfigure the running application service.
 */
#include "subsystem3_test.h"
#include "../test_report.h"

#if APP_TEST
void subsystem3_test (void)
{
    ir_sample_t      sample      = {{0u, 2048u, 4095u}, 65530u, 65535u, 1u, 7u};
    ir_sample_t      decoded     = {0};
    ir_frame_t       frame       = {{0u}};
    ir_calibration_t calibration = {3500u, 500u};
    uint16_t         darkness    = 0u;

    TEST_INIT();
    TEST_CHECK("S3-U01",
               ir_frame_pack(&sample, &frame) &&
                   ir_frame_unpack(frame.bytes, IR_FRAME_BYTES, &decoded) &&
                   (2048u == decoded.raw[1]) && (65535u == decoded.sequence),
               "packed frame preserves ADC and sequence");
    TEST_CHECK("S3-U02",
               ir_processing_normalise(calibration, 0u, 100u, &darkness) &&
                   (4095u == darkness),
               "reversed calibration clips to black");
    calibration.black = calibration.white;
    TEST_CHECK("S3-U03",
               !ir_processing_normalise(calibration, 1000u, 100u, &darkness),
               "zero contrast rejected");
    TEST_CHECK("S3-U04",
               ir_frame_age(4u, 65530u, 10u) &&
                   (!ir_frame_age(0u, 32768u, 32767u)),
               "clock wrap and half-range bound");
    TEST_SUMMARY("subsystem3");
}
#endif

/* End of file. */

