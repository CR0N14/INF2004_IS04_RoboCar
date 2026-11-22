/** @file code39.c
 * @brief Standard Code 39 patterns; no character-to-motion bindings.
 */
#include "code39.h"
#include <stddef.h>

#define CODE39_ELEMENTS 9u
#define CODE39_GUARD 0x094u
#define CODE39_INVALID 0xffffu
/* Within one character, the narrowest element classed wide must be at least
   this percentage of the widest same-colour element classed narrow.  Also
   bounds each inter-character gap against the neighbouring wide spaces. */
#define CODE39_MIN_RATIO_PCT 115u

/* Standard nine-element wide/narrow encodings, A through Z. */
static uint16_t const g_patterns[BARCODE_SYMBOL_COUNT] = {
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu, 0x04cu,
    0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u,
    0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};

static uint32_t run_width (uint16_t const * p_runs, uint8_t index,
                           bool b_reverse);
static uint16_t pattern_at (uint16_t const * p_runs, uint8_t start,
                            bool b_reverse, uint32_t * p_wide_space);
static bool     gap_narrow (uint32_t gap, uint32_t wide_before,
                            uint32_t wide_after);
static bool     orientation (uint16_t const * p_runs, bool b_reverse,
                             uint8_t * p_symbol);

bool code39_recognise (void * p_context, uint16_t const * p_runs, uint8_t count,
                       barcode_direction_t direction, uint8_t * p_symbol)
{
    uint8_t symbol = 0u;

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
    }

    if ((!orientation(p_runs, false, &symbol)) &&
        (!orientation(p_runs, true, &symbol)))
    {
        return false;
    }

    *p_symbol = symbol;
    return true;
}

/** Width of a run in reading order. */
static uint32_t run_width (uint16_t const * p_runs, uint8_t index,
                           bool b_reverse)
{
    uint8_t const offset =
        b_reverse ? (uint8_t)(CODE39_RUN_COUNT - 1u - index) : index;

    return p_runs[offset] & BARCODE_RUN_WIDTH;
}

/** Classify one character by ranking within it, not by a global width.
 *
 *  Every Code 39 letter, digit and the star guard has exactly two wide bars
 *  and one wide space.  The two widest bars and the widest space are marked
 *  wide, so hand speed may change between characters, and bars may read
 *  wider or narrower than spaces, without moving the classification.
 *  Ranking always yields a defined pattern, so validity relies on the ratio
 *  check here plus the guards, gaps and symbol table in orientation().
 */
static uint16_t pattern_at (uint16_t const * p_runs, uint8_t start,
                            bool b_reverse, uint32_t * p_wide_space)
{
    uint32_t width[CODE39_ELEMENTS];
    uint8_t  first        = 0u;
    uint8_t  second       = 2u;
    uint8_t  space        = 1u;
    uint32_t narrow_bar   = 0u;
    uint32_t narrow_space = 0u;
    uint16_t pattern      = 0u;

    for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
    {
        width[element] =
            run_width(p_runs, (uint8_t)(start + element), b_reverse);
    }

    if (width[second] > width[first])
    {
        first  = 2u;
        second = 0u;
    }

    for (uint8_t bar = 4u; bar < CODE39_ELEMENTS; bar += 2u)
    {
        if (width[bar] > width[first])
        {
            second = first;
            first  = bar;
        }
        else if (width[bar] > width[second])
        {
            second = bar;
        }
        else
        {
            /* Narrow bar. */
        }
    }

    for (uint8_t gap = 3u; gap < CODE39_ELEMENTS; gap += 2u)
    {
        if (width[gap] > width[space])
        {
            space = gap;
        }
    }

    for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
    {
        bool const b_wide =
            (element == first) || (element == second) || (element == space);

        pattern = (uint16_t)(pattern << 1u);
        if (b_wide)
        {
            pattern |= 1u;
        }
        else if ((0u == (element % 2u)) && (width[element] > narrow_bar))
        {
            narrow_bar = width[element];
        }
        else if ((0u != (element % 2u)) && (width[element] > narrow_space))
        {
            narrow_space = width[element];
        }
        else
        {
            /* Narrow element no wider than the current maximum. */
        }
    }

    /* Equal or near-equal widths cannot be ranked reliably: reject. */
    if (((width[second] * 100u) < (narrow_bar * CODE39_MIN_RATIO_PCT)) ||
        ((width[space] * 100u) < (narrow_space * CODE39_MIN_RATIO_PCT)))
    {
        return CODE39_INVALID;
    }

    *p_wide_space = width[space];
    return pattern;
}

/** True if a gap is clearly narrower than both neighbouring wide spaces. */
static bool gap_narrow (uint32_t gap, uint32_t wide_before, uint32_t wide_after)
{
    uint32_t const limit =
        (wide_before < wide_after) ? wide_before : wide_after;

    return (gap * CODE39_MIN_RATIO_PCT) <= (limit * 100u);
}

/** Single-character framing is star, gap, character, gap, star. */
static bool orientation (uint16_t const * p_runs, bool b_reverse,
                         uint8_t * p_symbol)
{
    uint32_t wide_start = 0u;
    uint32_t wide_char  = 0u;
    uint32_t wide_stop  = 0u;

    if ((CODE39_GUARD != pattern_at(p_runs, 0u, b_reverse, &wide_start)) ||
        (CODE39_GUARD != pattern_at(p_runs, 20u, b_reverse, &wide_stop)))
    {
        return false;
    }

    uint16_t const pattern = pattern_at(p_runs, 10u, b_reverse, &wide_char);

    if ((CODE39_INVALID == pattern) ||
        (!gap_narrow(run_width(p_runs, 9u, b_reverse), wide_start,
                     wide_char)) ||
        (!gap_narrow(run_width(p_runs, 19u, b_reverse), wide_char, wide_stop)))
    {
        return false;
    }

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

