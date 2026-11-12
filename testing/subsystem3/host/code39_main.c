/** @file code39_main.c
 * @brief Fixtures generated independently with python-barcode, checksum off.
 */
#include "code39_main.h"
#include "code39.h"
#include <stdio.h>
#include <string.h>

static char const * const g_fixtures[BARCODE_SYMBOL_COUNT] = {
    "10001011101110101110101000101110100010111011101",
    "10001011101110101011101000101110100010111011101",
    "10001011101110101110111010001010100010111011101",
    "10001011101110101010111000101110100010111011101",
    "10001011101110101110101110001010100010111011101",
    "10001011101110101011101110001010100010111011101",
    "10001011101110101010100011101110100010111011101",
    "10001011101110101110101000111010100010111011101",
    "10001011101110101011101000111010100010111011101",
    "10001011101110101010111000111010100010111011101",
    "10001011101110101110101010001110100010111011101",
    "10001011101110101011101010001110100010111011101",
    "10001011101110101110111010100010100010111011101",
    "10001011101110101010111010001110100010111011101",
    "10001011101110101110101110100010100010111011101",
    "10001011101110101011101110100010100010111011101",
    "10001011101110101010101110001110100010111011101",
    "10001011101110101110101011100010100010111011101",
    "10001011101110101011101011100010100010111011101",
    "10001011101110101010111011100010100010111011101",
    "10001011101110101110001010101110100010111011101",
    "10001011101110101000111010101110100010111011101",
    "10001011101110101110001110101010100010111011101",
    "10001011101110101000101110101110100010111011101",
    "10001011101110101110001011101010100010111011101",
    "10001011101110101000111011101010100010111011101"};

static bool replay_symbol (char const * p_bits, bool b_reverse,
                           uint8_t expected);

static bool replay_symbol (char const * p_bits, bool b_reverse,
                           uint8_t expected)
{
    barcode_config_t  config     = {0};
    barcode_decoder_t decoder    = {0};
    ir_permission_t   permission = {0};
    barcode_motion_t  motion     = {0};
    unsigned int      results    = 0u;
    size_t const      length     = strlen(p_bits);

    config.calibration.black = IR_ADC_MAX;
    config.channel           = 2u;
    config.minimum_contrast  = 100u;
    config.dark_on           = 3000u;
    config.dark_off          = 1000u;
    config.max_gap_ms        = 2u;
    config.max_capture_ms    = 2000u;
    config.clearance_ms      = 100u;
    config.minimum_runs      = CODE39_RUN_COUNT;
    permission.generation    = 1u;
    permission.mode          = IR_MODE_TRACKING;
    permission.b_enabled     = true;
    motion.generation        = 1u;
    motion.direction         = BARCODE_FORWARD;
    motion.b_valid           = true;
    if (!barcode_decoder_init(&decoder, &config))
    {
        return false;
    }
    for (uint16_t time = 0u; time < 250u + length * 10u; ++time)
    {
        ir_sample_t      sample = {0};
        ir_frame_t       frame  = {0};
        barcode_result_t result = {0};
        sample.raw[0]           = IR_ADC_MAX;
        sample.time_ms          = time;
        sample.sequence         = time;
        sample.generation       = 1u;
        sample.valid_mask       = 7u;
        if ((time >= 110u) && (time < 110u + length * 10u))
        {
            size_t index = (time - 110u) / 10u;
            if (b_reverse)
            {
                index = length - 1u - index;
            }
            sample.raw[2] = ('1' == p_bits[index]) ? IR_ADC_MAX : 0u;
        }
        if (!ir_frame_pack(&sample, &frame))
        {
            return false;
        }
        motion.time_ms = time;
        result = barcode_decoder_step(&decoder, &frame, permission, 1u, motion,
                                      time);
        if (BARCODE_NONE != result.outcome)
        {
            if ((BARCODE_UNBOUND != result.outcome) ||
                (expected != result.symbol) ||
                (time != 110u + length * 10u))
            {
                return false;
            }
            ++results;
        }
    }
    return 1u == results;
}

int main (void)
{
    unsigned int failures = 0u;
    unsigned int checks   = 0u;

    for (uint8_t symbol = 0u; symbol < BARCODE_SYMBOL_COUNT; ++symbol)
    {
        for (uint8_t reverse = 0u; reverse < 2u; ++reverse)
        {
            uint16_t     runs[CODE39_RUN_COUNT] = {0u};
            uint8_t      count                  = 0u;
            uint8_t      decoded                = 255u;
            char const * p_bits                 = g_fixtures[symbol];
            size_t const length                 = strlen(p_bits);
            char         previous               = p_bits[0];
            uint16_t     width                  = 0u;

            for (size_t index = 0u; index <= length; ++index)
            {
                char const level = p_bits[index];
                if (level != previous)
                {
                    uint8_t const offset =
                        (0u == reverse) ? count : CODE39_RUN_COUNT - 1u - count;
                    runs[offset] =
                        width | (('1' == previous) ? BARCODE_RUN_DARK : 0u);
                    ++count;
                    width    = 0u;
                    previous = level;
                }
                width += 10u;
            }

            ++checks;
            if ((!code39_recognise(NULL, runs, count, BARCODE_FORWARD,
                                   &decoded)) ||
                (decoded != symbol))
            {
                ++failures;
            }
            ++checks;
            if (!replay_symbol(p_bits, 0u != reverse, symbol))
            {
                ++failures;
            }
            /* Corrupt a mandatory guard and require rejection. */
            runs[0] = BARCODE_RUN_DARK;
            ++checks;
            if (code39_recognise(NULL, runs, count, BARCODE_REVERSE, &decoded))
            {
                ++failures;
            }
        }
    }

    printf("Code39 A-Z bidirectional and final-edge checks: %u; failures: %u\n",
           checks, failures);
    return (0u == failures) ? 0 : 1;
}

/* End of file. */

