/** @file integration_components.c
 * @brief Exercise measured configuration and temporal junction qualification.
 */
#include "integration_components.h"
#include "ir_config.h"
#include "ir_junction.h"
#include <stdio.h>

static unsigned int g_checks;
static unsigned int g_failures;
static void check (bool b_passed, char const * p_description);
static void test_configuration (void);
static void test_approach (void);

static void check (bool b_passed, char const * p_description)
{
    ++g_checks;
    if (!b_passed) { ++g_failures; }
    printf("%s %s\n", b_passed ? "PASS" : "FAIL", p_description);
}

static void test_configuration (void)
{
    ir_calibration_t calibration[IR_CHANNEL_COUNT] = {
        {180u, 3200u}, {3200u, 180u}, {200u, 3300u}};
    ir_config_t line = {0};
    ir_processing_t processor = {0};
    barcode_decoder_t decoder = {0};
    ir_config_line(calibration, &line);
    barcode_config_t code = ir_config_barcode(calibration[2]);
    check(ir_processing_init(&processor, &line) &&
              barcode_decoder_init(&decoder, &code),
          "IC01 measured three-sensor production configuration accepted");
    uint16_t darkness = 0u;
    check(ir_processing_normalise(line.calibration[1], 180u,
                                   IR_CONTRAST, &darkness) &&
              (IR_ADC_MAX == darkness),
          "IC02 factory preserves independently reversed centre polarity");
    check((0u == line.left_channel) && (1u == line.centre_channel) &&
              (2u == line.barcode_channel) && (2u == code.channel),
          "IC03 line and decoder share the right-IR role contract");
    calibration[1].white = 1000u;
    calibration[1].black = 1199u;
    ir_config_line(calibration, &line);
    check(!ir_processing_init(&processor, &line),
          "IC04 inadequate physical tracker contrast rejects configuration");
    calibration[2].white = 1000u;
    calibration[2].black = 1199u;
    code = ir_config_barcode(calibration[2]);
    check(!barcode_decoder_init(&decoder, &code),
          "IC05 inadequate right-sensor contrast rejects decoder readiness");
}

static void test_approach (void)
{
    ir_junction_t junction = {0};
    ir_junction_reset(&junction);
    ir_junction_observe(&junction, false, false, 100u);
    ir_junction_observe(&junction, false, false, 200u);
    check(!junction.b_qualified,
          "IJ01 both-black without a usable approach cannot qualify");
    ir_junction_observe(&junction, true, false, 300u);
    ir_junction_observe(&junction, true, false, 340u);
    check(!junction.b_qualified,
          "IJ02 a short usable-line approach remains unqualified");
    ir_junction_observe(&junction, false, false, 345u);
    ir_junction_observe(&junction, true, false, 350u);
    ir_junction_observe(&junction, true, false, 390u);
    check(!junction.b_qualified,
          "IJ03 interrupted approach must start a new persistence interval");
    ir_junction_observe(&junction, true, false, 400u);
    check(junction.b_qualified,
          "IJ04 stable usable approach qualifies after the full interval");
    ir_junction_observe(&junction, false, false, 410u);
    check(junction.b_qualified,
          "IJ05 a subsequent crossing preserves the qualified approach");
    ir_junction_observe(&junction, false, true, 470u);
    check(!junction.b_qualified,
          "IJ06 persistent line loss invalidates the approach");
    ir_junction_observe(&junction, true, false, 65520u);
    ir_junction_observe(&junction, true, false, 34u);
    check(junction.b_qualified,
          "IJ07 approach timing works across the millisecond wrap");
    ir_junction_reset(&junction);
    check((!junction.b_qualified) && (!junction.b_active),
          "IJ08 stop or new encounter explicitly clears qualification");
}

int main (void)
{
    test_configuration();
    test_approach();
    printf("Integration component checks: %u; failures: %u\n",
           g_checks, g_failures);
    return (0u == g_failures) ? 0 : 1;
}
/* End of file. */

