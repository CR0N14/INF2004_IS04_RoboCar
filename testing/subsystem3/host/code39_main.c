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
                (expected != result.symbol) || (time != 110u + length * 10u))
            {
                return false;
            }
            ++results;
        }
    }
    return 1u == results;
}

/* Hand scans of slide 2's 'A', stationary bench 2026-11-22 18:44 onwards,
   run widths in ms at 20 ms sampling.  Rows 0-3 must decode as A (rows 1
   and 3 were scanned in reverse); rows 4-5 hold 29 runs but do not match
   the printed pattern, so they must be rejected. */
static uint16_t const g_measured[6][CODE39_RUN_COUNT] = {
    {220, 640, 120, 340, 800, 300, 640, 200, 160, 220, 700, 300,  120, 240, 220,
     760, 400, 80,  800, 300, 160, 880, 380, 240, 660, 140, 1280, 60,  40},
    {300, 160, 840,  340,  780, 280, 160, 720, 160, 280,
     760, 500, 200,  1160, 200, 940, 180, 560, 640, 640,
     80,  420, 1200, 440,  620, 380, 260, 960, 260},
    {140, 480, 140, 260, 580, 400, 400, 200, 220, 280, 520, 240, 120, 240, 120,
     620, 100, 140, 580, 240, 80,  560, 100, 200, 460, 280, 520, 300, 200},
    {200, 300, 620, 340, 380, 220, 200, 500, 80,  360, 660, 400, 120, 820, 140,
     300, 240, 320, 720, 80,  100, 160, 620, 160, 660, 180, 100, 360, 100},
    {60,  440, 80,  880, 260, 360, 740, 280, 460, 180, 820, 140, 660, 280, 100,
     280, 200, 800, 380, 280, 580, 240, 160, 800, 320, 240, 740, 300, 760},
    {120, 140, 260, 160, 560, 40,  80,  20,  40,  760, 80,  240, 1060, 380, 180,
     120, 100, 740, 160, 200, 380, 140, 340, 260, 140, 560, 160, 280,  720}};

/* Ideal '*A*': 0 = narrow, 1 = wide, star gap A gap star. */
static uint8_t const g_ideal_a[CODE39_RUN_COUNT] = {
    0u, 1u, 0u, 0u, 1u, 0u, 1u, 0u, 0u, 0u, 1u, 0u, 0u, 0u, 0u,
    1u, 0u, 0u, 1u, 0u, 0u, 1u, 0u, 0u, 1u, 0u, 1u, 0u, 0u};

static bool recognise_widths (uint16_t const * p_widths, uint8_t * p_symbol);
static bool replay_widths (uint16_t const * p_widths, uint8_t count,
                           uint8_t expected);
static unsigned int measured_checks (unsigned int * p_checks);

/** Tag alternating dark/light runs, starting dark, and recognise them. */
static bool recognise_widths (uint16_t const * p_widths, uint8_t * p_symbol)
{
    uint16_t runs[CODE39_RUN_COUNT] = {0u};

    for (uint8_t index = 0u; index < CODE39_RUN_COUNT; ++index)
    {
        runs[index] =
            (uint16_t)(p_widths[index] |
                       ((0u == (index % 2u)) ? BARCODE_RUN_DARK : 0u));
    }
    return code39_recognise(NULL, runs, CODE39_RUN_COUNT, BARCODE_FORWARD,
                            p_symbol);
}

/** Replay run widths (ms) through the full decoder at 1 ms per sample,
 *  with the stationary bench timings (2 s clearance, 30 s capture). */
static bool replay_widths (uint16_t const * p_widths, uint8_t count,
                           uint8_t expected)
{
    barcode_config_t  config     = {0};
    barcode_decoder_t decoder    = {0};
    ir_permission_t   permission = {0};
    barcode_motion_t  motion     = {0};
    unsigned int      results    = 0u;
    uint32_t          total      = 0u;

    for (uint8_t index = 0u; index < count; ++index)
    {
        total += p_widths[index];
    }
    config.calibration.black = IR_ADC_MAX;
    config.channel           = 2u;
    config.minimum_contrast  = 100u;
    config.dark_on           = 3000u;
    config.dark_off          = 1000u;
    config.max_gap_ms        = 2u;
    config.max_capture_ms    = 30000u;
    config.clearance_ms      = 2000u;
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
    for (uint32_t time = 0u; time < 2310u + total; ++time)
    {
        ir_sample_t      sample = {0};
        ir_frame_t       frame  = {0};
        barcode_result_t result = {0};
        uint32_t         edge   = 2010u;
        uint8_t          run    = 0u;

        while ((run < count) && (time >= edge + p_widths[run]))
        {
            edge += p_widths[run];
            ++run;
        }
        sample.raw[0]     = IR_ADC_MAX;
        sample.time_ms    = (uint16_t)time;
        sample.sequence   = (uint16_t)time;
        sample.generation = 1u;
        sample.valid_mask = 7u;
        sample.raw[2] = ((time >= 2010u) && (run < count) && (0u == (run % 2u)))
                            ? IR_ADC_MAX
                            : 0u;
        if (!ir_frame_pack(&sample, &frame))
        {
            return false;
        }
        motion.time_ms = (uint16_t)time;
        result = barcode_decoder_step(&decoder, &frame, permission, 1u, motion,
                                      (uint16_t)time);
        if (BARCODE_NONE != result.outcome)
        {
            if ((BARCODE_UNBOUND != result.outcome) ||
                (expected != result.symbol) || (time != 2010u + total))
            {
                return false;
            }
            ++results;
        }
    }
    return 1u == results;
}

/** Measured hand scans, a stray leading pair, a speed ramp, equal widths. */
static unsigned int measured_checks (unsigned int * p_checks)
{
    unsigned int failures                      = 0u;
    uint8_t      symbol                        = 255u;
    uint16_t     widths[CODE39_RUN_COUNT + 2u] = {60u, 440u};

    for (uint8_t row = 0u; row < 6u; ++row)
    {
        bool const b_ok = recognise_widths(g_measured[row], &symbol);

        ++*p_checks;
        if ((row < 4u) ? ((!b_ok) || (0u != symbol)) : b_ok)
        {
            ++failures;
        }
    }

    ++*p_checks;
    if (!replay_widths(g_measured[0], CODE39_RUN_COUNT, 0u))
    {
        ++failures;
    }

    /* Stray 60 ms bar and 440 ms space before the first real bar. */
    for (uint8_t index = 0u; index < CODE39_RUN_COUNT; ++index)
    {
        widths[index + 2u] = g_measured[0][index];
    }
    ++*p_checks;
    if (!replay_widths(widths, CODE39_RUN_COUNT + 2u, 0u))
    {
        ++failures;
    }

    /* Ideal A, narrow 100 / wide 250 ms, hand speeding up 2.2x across it. */
    for (uint8_t index = 0u; index < CODE39_RUN_COUNT; ++index)
    {
        uint32_t const base  = (0u != g_ideal_a[index]) ? 250u : 100u;
        uint32_t const scale = 100u + ((uint32_t)index * 120u) / 28u;

        widths[index] = (uint16_t)((base * 100u) / scale);
    }
    ++*p_checks;
    if ((!recognise_widths(widths, &symbol)) || (0u != symbol))
    {
        ++failures;
    }

    /* All runs equal: nothing can be ranked, so it must be rejected. */
    for (uint8_t index = 0u; index < CODE39_RUN_COUNT; ++index)
    {
        widths[index] = 200u;
    }
    ++*p_checks;
    if (recognise_widths(widths, &symbol))
    {
        ++failures;
    }
    return failures;
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

    /* Measured from slide 2's embedded EMF, rendered at 1600x900.
       Excludes the outer white margins; exactly 29 alternating runs. */
    uint16_t slide_runs[29] = {33, 99, 33, 33, 99, 33, 99, 33, 33, 33,
                               99, 33, 33, 33, 33, 99, 33, 33, 99, 33,
                               33, 99, 33, 33, 99, 33, 99, 33, 33};
    uint8_t  slide_symbol   = 255u;
    for (unsigned i = 0; i < 29; i++)
        if (i % 2 == 0)
            slide_runs[i] |= BARCODE_RUN_DARK;
    ++checks;
    if (!code39_recognise(NULL, slide_runs, 29, BARCODE_FORWARD,
                          &slide_symbol) ||
        slide_symbol != 0u)
        ++failures;

    failures += measured_checks(&checks);

    printf("Code39 A-Z bidirectional and final-edge checks: %u; failures: %u\n",
           checks, failures);
    return (0u == failures) ? 0 : 1;
}

/* End of file. */

