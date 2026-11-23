/*
 * Host tests for demo_measure_width(): a simulated servo and ultrasonic sensor
 * looking at a flat-faced box. The sensor returns the nearest echo inside its
 * beam cone, with +/-1 cm of noise. No hardware, kernel or SDK is needed.
 */
#define _GNU_SOURCE
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/subsystem5/demo_profile.h"

#define BEAM_HALF_DEG   ((double)DEMO_BEAM_HALF_DEG)

static int g_failed;
static int g_passed;

#define TEST_CHECK(id, cond, msg) \
    do { \
        if (cond) { g_passed++; printf("[TEST] %s PASS  %s\n", (id), (msg)); } \
        else { g_failed++; printf("[TEST] %s FAIL  %s\n", (id), (msg)); } \
    } while (0)

typedef struct
{
    double depth_cm;    /* box face, straight ahead of the servo        */
    double width_cm;
    double centre_cm;   /* sideways offset of the box centre, + = right  */
    double wall_cm;     /* background wall depth; 0 = open space         */
    double dropout;     /* chance a reading returns no echo              */
    unsigned int seed;
} world_t;

static unsigned int
rnd(world_t *p_w)
{
    p_w->seed = (p_w->seed * 1664525u) + 1013904223u;
    return p_w->seed >> 8;
}

static double
ray_range(world_t const *p_w, double off_deg)
{
    double const a = off_deg * M_PI / 180.0;
    double const c = cos(a);
    double best = 1e9;

    if (c > 1e-6)
    {
        double const r = p_w->depth_cm / c;
        double const x = r * sin(a);

        if ((x >= p_w->centre_cm - (p_w->width_cm / 2.0)) &&
            (x <= p_w->centre_cm + (p_w->width_cm / 2.0)))
        {
            best = r;
        }
        if ((p_w->wall_cm > 0.0) && ((p_w->wall_cm / c) < best))
        {
            best = p_w->wall_cm / c;
        }
    }
    return best;
}

static uint16_t
read_world(void *p_ctx, unsigned int angle_deg)
{
    world_t *p_w = (world_t *)p_ctx;
    double const off = (double)angle_deg - 90.0;
    double best = 1e9;

    for (double d = -BEAM_HALF_DEG; d <= BEAM_HALF_DEG + 1e-9; d += 0.25)
    {
        double const r = ray_range(p_w, off + d);

        if (r < best)
        {
            best = r;
        }
    }
    if ((p_w->dropout > 0.0) && ((rnd(p_w) % 1000u) < (unsigned int)(p_w->dropout * 1000.0)))
    {
        return 511u;
    }
    if (best > 400.0)
    {
        return 511u;
    }
    return (uint16_t)lround(best + (double)((int)(rnd(p_w) % 3u) - 1));
}

int
main(void)
{
    static double const depths[] = { 15.0, 20.0, 25.0, 30.0 };
    static double const widths[] = { 6.0, 10.0, 14.0, 20.0 };
    static double const offsets[] = { 0.0, 2.0, -2.0 };
    double sum = 0.0;
    double worst = 0.0;
    unsigned int most = 0u;
    unsigned int n = 0u;
    unsigned int failed_cases = 0u;
    demo_width_t out;
    world_t w;

    /* S5-U22: accuracy over a grid of box sizes, distances and offsets */
    for (unsigned int di = 0u; di < 4u; di++)
    {
        for (unsigned int wi = 0u; wi < 4u; wi++)
        {
            for (unsigned int oi = 0u; oi < 3u; oi++)
            {
                w = (world_t){ depths[di], widths[wi], offsets[oi], 80.0, 0.0,
                               12345u + (di * 100u) + (wi * 10u) + oi };
                if (demo_measure_width(read_world, &w, &out))
                {
                    double const e = fabs((double)out.width_cm - widths[wi]);

                    sum += e;
                    worst = (e > worst) ? e : worst;
                    most = (out.readings > most) ? out.readings : most;
                    n++;
                }
                else
                {
                    failed_cases++;
                }
            }
        }
    }
    printf("grid: %u measured, %u failed, mean error %.2f cm, worst %.1f cm, "
           "most readings %u\n", n, failed_cases, n ? sum / n : 0.0, worst, most);
    TEST_CHECK("S5-U22", (0u == failed_cases) && (sum / n <= 0.8) && (worst <= 2.0),
               "48 box cases: all measured, mean error <= 0.8 cm, worst <= 2 cm");
    TEST_CHECK("S5-U23", most <= 24u, "no more than 24 sensor readings per scan");

    /* S5-U24: open space behind the box is background, not a failure */
    w = (world_t){ 20.0, 10.0, 0.0, 0.0, 0.0, 777u };
    TEST_CHECK("S5-U24", demo_measure_width(read_world, &w, &out) &&
               (fabs((double)out.width_cm - 10.0) <= 1.0),
               "no echo beyond the box: 10 cm box measured within 1 cm");

    /* S5-U25: corners, bearing and ahead range */
    w = (world_t){ 20.0, 14.0, 4.0, 80.0, 0.0, 999u };
    TEST_CHECK("S5-U25", demo_measure_width(read_world, &w, &out) &&
               (out.left_deg < 90u) && (out.right_deg > 90u) &&
               (out.bearing_deg > 90u) && (out.bearing_deg <= 100u) &&
               (out.ahead_cm >= 19u) && (out.ahead_cm <= 21u),
               "box 4 cm to the right: bearing right of 90, ahead about 20 cm");

    /* S5-U26: occasional dropped echoes are retried */
    w = (world_t){ 20.0, 10.0, 0.0, 80.0, 0.10, 4242u };
    TEST_CHECK("S5-U26", demo_measure_width(read_world, &w, &out) &&
               (fabs((double)out.width_cm - 10.0) <= 2.0),
               "10% dropped echoes: 10 cm box measured within 2 cm");

    /* S5-U27: refusals leave the output untouched */
    out = (demo_width_t){ .width_cm = 55u };
    w = (world_t){ 20.0, 200.0, 0.0, 80.0, 0.0, 1u };
    TEST_CHECK("S5-U27a", !demo_measure_width(read_world, &w, &out) && (55u == out.width_cm),
               "box wider than the sweep is refused");
    w = (world_t){ 20.0, 10.0, 0.0, 0.0, 1.0, 1u };
    TEST_CHECK("S5-U27b", !demo_measure_width(read_world, &w, &out) && (55u == out.width_cm),
               "no echo straight ahead is refused");
    w = (world_t){ 20.0, 10.0, 0.0, 80.0, 0.0, 1u };
    w.depth_cm = 60.0;
    TEST_CHECK("S5-U27c", !demo_measure_width(read_world, &w, &out) && (55u == out.width_cm),
               "box 60 cm away is refused");
    TEST_CHECK("S5-U27d", !demo_measure_width(NULL, &w, &out) &&
               !demo_measure_width(read_world, &w, NULL), "null arguments are refused");

    printf("[TEST] width SUMMARY %d passed, %d failed\n", g_passed, g_failed);
    return (0 == g_failed) ? 0 : 1;
}
