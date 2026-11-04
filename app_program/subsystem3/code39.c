/** @file code39.c
 * @brief Standard Code 39 patterns; no character-to-motion bindings.
 */
#include "code39.h"
#include <stddef.h>

#define CODE39_ELEMENTS 9u
#define CODE39_GUARD 0x094u
#define CODE39_INVALID 0xffffu

/* Standard nine-element wide/narrow encodings, A through Z. */
static uint16_t const g_patterns[BARCODE_SYMBOL_COUNT] = {
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu, 0x04cu,
    0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u,
    0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};

static uint16_t pattern_at (uint16_t const * p_runs, uint8_t start,
                            bool b_reverse, uint16_t narrow);
static bool     orientation (uint16_t const * p_runs, bool b_reverse,
                             uint16_t narrow, uint8_t * p_symbol);

bool code39_recognise (void * p_context, uint16_t const * p_runs, uint8_t count,
                       barcode_direction_t direction, uint8_t * p_symbol)
{
    uint16_t narrow = BARCODE_RUN_WIDTH;
    uint8_t  symbol = 0u;

    /* Context is reserved; orientation is determined from the guards. */
    (void)p_context;
    if ((NULL == p_runs) || (NULL == p_symbol) || (CODE39_RUN_COUNT != count) ||
        ((BARCODE_FORWARD != direction) && (BARCODE_REVERSE != direction)))
    {
        return false;
    }

    for (uint8_t index = 0u; index < CODE39_RUN_COUNT; ++index)
    {
        uint16_t const width  = p_runs[index] & BARCODE_RUN_WIDTH;
        bool const     b_dark = 0u != (p_runs[index] & BARCODE_RUN_DARK);

        if ((0u == width) || (b_dark != (0u == (index % 2u))))
        {
            return false;
        }

        if (width < narrow)
        {
            narrow = width;
        }
    }

    if ((!orientation(p_runs, false, narrow, &symbol)) &&
        (!orientation(p_runs, true, narrow, &symbol)))
    {
        return false;
    }

    *p_symbol = symbol;
    return true;
}

/** Conservative timing bands; speed must remain approximately constant. */
static uint16_t pattern_at (uint16_t const * p_runs, uint8_t start,
                            bool b_reverse, uint16_t narrow)
{
    uint16_t pattern    = 0u;
    uint8_t  wide_count = 0u;

    for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
    {
        uint8_t const index = start + element;
        uint8_t const offset =
            b_reverse ? CODE39_RUN_COUNT - 1u - index : index;
        uint32_t const width = p_runs[offset] & BARCODE_RUN_WIDTH;

        pattern <<= 1u;
        if ((width * 2u) <= (narrow * 3u))
        {
            /* Narrow element. */
        }
        else if (((width * 5u) >= (narrow * 9u)) && (width <= (narrow * 4u)))
        {
            pattern |= 1u;
            ++wide_count;
        }
        else
        {
            return CODE39_INVALID;
        }
    }

    return (3u == wide_count) ? pattern : CODE39_INVALID;
}

/** Single-character framing is star, gap, character, gap, star. */
static bool orientation (uint16_t const * p_runs, bool b_reverse,
                         uint16_t narrow, uint8_t * p_symbol)
{
    if ((CODE39_GUARD != pattern_at(p_runs, 0u, b_reverse, narrow)) ||
        (CODE39_GUARD != pattern_at(p_runs, 20u, b_reverse, narrow)) ||
        (((p_runs[9] & BARCODE_RUN_WIDTH) * 2u) > (narrow * 3u)) ||
        (((p_runs[19] & BARCODE_RUN_WIDTH) * 2u) > (narrow * 3u)))
    {
        return false;
    }

    uint16_t const pattern = pattern_at(p_runs, 10u, b_reverse, narrow);

    for (uint8_t symbol = 0u; symbol < BARCODE_SYMBOL_COUNT; ++symbol)
    {
        if (g_patterns[symbol] == pattern)
        {
            *p_symbol = symbol;
            return true;
        }
    }

    return false;
}

/* End of file. */

