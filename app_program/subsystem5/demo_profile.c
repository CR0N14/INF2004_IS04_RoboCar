#include "demo_profile.h"

#include <stddef.h>

#define FORWARD_DEG     90
#define MAX_RANGE_CM    400u
#define MIN_RANGE_CM    2u
#define MAX_WIDTH_CM    127u    /* width_cm is a 7-bit telemetry field */

/* sin and cos of 0..60 degrees, scaled by 10000: no runtime trig on the M0+. */
static uint16_t const g_sin[DEMO_SCAN_HALF_DEG + 1u] =
{
        0,   175,   349,   523,   698,   872,  1045,  1219,  1392,  1564,
     1736,  1908,  2079,  2250,  2419,  2588,  2756,  2924,  3090,  3256,
     3420,  3584,  3746,  3907,  4067,  4226,  4384,  4540,  4695,  4848,
     5000,  5150,  5299,  5446,  5592,  5736,  5878,  6018,  6157,  6293,
     6428,  6561,  6691,  6820,  6947,  7071,  7193,  7314,  7431,  7547,
     7660,  7771,  7880,  7986,  8090,  8192,  8290,  8387,  8480,  8572,
     8660,
};

static uint16_t const g_cos[DEMO_SCAN_HALF_DEG + 1u] =
{
    10000,  9998,  9994,  9986,  9976,  9962,  9945,  9925,  9903,  9877,
     9848,  9816,  9781,  9744,  9703,  9659,  9613,  9563,  9511,  9455,
     9397,  9336,  9272,  9205,  9135,  9063,  8988,  8910,  8829,  8746,
     8660,  8572,  8480,  8387,  8290,  8192,  8090,  7986,  7880,  7771,
     7660,  7547,  7431,  7314,  7193,  7071,  6947,  6820,  6691,  6561,
     6428,  6293,  6157,  6018,  5878,  5736,  5592,  5446,  5299,  5150,
     5000,
};

typedef struct
{
    demo_range_fn read;
    void         *p_ctx;
    unsigned int  readings;
} scan_t;

typedef struct
{
    unsigned int off_deg;   /* corner angle from straight ahead */
    unsigned int range_cm;  /* range at the last reading on the obstacle */
} edge_t;

static bool
valid(uint16_t range_cm)
{
    return (range_cm >= MIN_RANGE_CM) && (range_cm <= MAX_RANGE_CM);
}

/* One reading, repeated up to twice if it returned no echo, so a single
   dropped echo is not mistaken for the edge of the obstacle. 0 means no echo. */
static uint16_t
sample(scan_t *p_s, int angle_deg)
{
    uint16_t range_cm = 0u;

    for (unsigned int attempt = 0u; attempt < 3u; attempt++)
    {
        range_cm = p_s->read(p_s->p_ctx, (unsigned int)angle_deg);
        p_s->readings++;
        if (valid(range_cm))
        {
            return range_cm;
        }
    }
    return 0u;
}

/* A flat face square to the car has the same depth at every angle. A reading
   with no echo is background, not obstacle. */
static bool
is_hit(uint16_t range_cm, unsigned int off_deg, uint16_t ahead_cm)
{
    unsigned int depth_cm;

    if ((0u == range_cm) || (off_deg > DEMO_SCAN_HALF_DEG))
    {
        return false;
    }
    depth_cm = ((range_cm * (unsigned int)g_cos[off_deg]) + 5000u) / 10000u;
    return depth_cm <= (ahead_cm + DEMO_HIT_TOL_CM);
}

/* Median of up to three readings straight ahead; 0 if fewer than two echo. */
static uint16_t
read_ahead(scan_t *p_s)
{
    uint16_t v[3];
    unsigned int n = 0u;

    for (unsigned int i = 0u; i < 3u; i++)
    {
        uint16_t const r = sample(p_s, FORWARD_DEG);

        if (0u != r)
        {
            v[n++] = r;
        }
    }
    if (n < 2u)
    {
        return 0u;
    }
    for (unsigned int i = 1u; i < n; i++)
    {
        uint16_t const key = v[i];
        unsigned int j = i;

        while ((j > 0u) && (v[j - 1u] > key))
        {
            v[j] = v[j - 1u];
            j--;
        }
        v[j] = key;
    }
    return v[(n - 1u) / 2u];
}

/*
 * Walk outward from straight ahead in coarse steps until a reading is no
 * longer on the obstacle, then step back through that gap in fine steps.
 * dir is -1 for the left and +1 for the right.
 */
static bool
find_edge(scan_t *p_s, uint16_t ahead_cm, int dir, edge_t *p_e)
{
    unsigned int hit_off = 0u;
    unsigned int miss_off = 0u;
    unsigned int off;
    unsigned int edge_off;
    uint16_t hit_range = ahead_cm;

    for (off = DEMO_COARSE_STEP_DEG; off <= DEMO_SCAN_HALF_DEG;
         off += DEMO_COARSE_STEP_DEG)
    {
        uint16_t const r = sample(p_s, FORWARD_DEG + (dir * (int)off));

        if (is_hit(r, off, ahead_cm))
        {
            hit_off = off;
            hit_range = r;
        }
        else
        {
            miss_off = off;
            break;
        }
    }
    if (0u == miss_off)
    {
        return false;       /* still on the obstacle at the sweep limit */
    }

    for (off = hit_off + DEMO_FINE_STEP_DEG; off < miss_off;
         off += DEMO_FINE_STEP_DEG)
    {
        uint16_t const r = sample(p_s, FORWARD_DEG + (dir * (int)off));

        if (is_hit(r, off, ahead_cm))
        {
            hit_off = off;
            hit_range = r;
        }
        else
        {
            miss_off = off;
            break;
        }
    }

    /* The corner lies between the last reading on the obstacle and the first
       off it, then back toward the axis by the beam half-angle. */
    edge_off = (hit_off + miss_off) / 2u;
    edge_off = (edge_off > DEMO_BEAM_HALF_DEG) ? (edge_off - DEMO_BEAM_HALF_DEG)
                                               : 0u;
    p_e->off_deg = edge_off;
    p_e->range_cm = hit_range;
    return true;
}

bool
demo_measure_width(demo_range_fn read, void *p_ctx, demo_width_t *p_out)
{
    scan_t scan = { read, p_ctx, 0u };
    edge_t left;
    edge_t right;
    uint16_t ahead_cm;
    unsigned int lateral;
    unsigned int width_cm;

    if ((NULL == read) || (NULL == p_out))
    {
        return false;
    }
    ahead_cm = read_ahead(&scan);
    if ((0u == ahead_cm) || (ahead_cm > DEMO_AHEAD_MAX_CM))
    {
        return false;
    }
    if (!find_edge(&scan, ahead_cm, -1, &left) ||
        !find_edge(&scan, ahead_cm, +1, &right))
    {
        return false;
    }

    lateral = (left.range_cm * (unsigned int)g_sin[left.off_deg]) +
              (right.range_cm * (unsigned int)g_sin[right.off_deg]);
    width_cm = (lateral + 5000u) / 10000u;
    if ((0u == width_cm) || (width_cm > MAX_WIDTH_CM))
    {
        return false;
    }

    p_out->width_cm = (uint16_t)width_cm;
    p_out->ahead_cm = ahead_cm;
    p_out->left_deg = (uint8_t)(FORWARD_DEG - (int)left.off_deg);
    p_out->right_deg = (uint8_t)(FORWARD_DEG + (int)right.off_deg);
    p_out->bearing_deg = (uint8_t)((p_out->left_deg + p_out->right_deg) / 2u);
    p_out->readings = (uint8_t)((scan.readings > 255u) ? 255u : scan.readings);
    return true;
}
