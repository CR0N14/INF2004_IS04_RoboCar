/*
 *  imu_fixmath.c
 *  Integer maths for Subsystem 4.  See imu_fixmath.h.
 */

#include "imu_fixmath.h"

/* atan(k / 32) in cdeg for k = 0..32 */
#define ATAN_STEPS          32u
#define ATAN_FRAC_BITS      10u     /* Q15 ratio = index:5 . frac:10     */
#define ATAN_RATIO_BITS     15u
#define ATAN_INPUT_MAX      0xFFFFuL

static int16_t const g_atan_tab[ATAN_STEPS + 1u] =
{
    0, 179, 358, 536, 713, 888, 1062, 1234, 1404, 1571, 1735,
    1897, 2056, 2211, 2363, 2511, 2657, 2798, 2936, 3070, 3201,
    3327, 3451, 3571, 3687, 3800, 3909, 4016, 4119, 4218, 4315,
    4409, 4500
};

uint32_t
imu_isqrt(uint32_t x)
{
    uint32_t r = 0u;
    uint32_t bit = 1uL << 30;

    while (bit > x)
    {
        bit >>= 2;
    }
    while (0u != bit)
    {
        if (x >= (r + bit))
        {
            x -= r + bit;
            r = (r >> 1) + bit;
        }
        else
        {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

/* atan(r) for a Q15 ratio 0 <= r <= 1, linear interpolation */
static int32_t
atan_q15(uint32_t r)
{
    uint32_t const idx = r >> ATAN_FRAC_BITS;
    uint32_t const frac = r & ((1uL << ATAN_FRAC_BITS) - 1u);
    int32_t lo;
    int32_t hi;

    if (idx >= ATAN_STEPS)
    {
        return g_atan_tab[ATAN_STEPS];
    }
    lo = g_atan_tab[idx];
    hi = g_atan_tab[idx + 1u];
    return lo + (int32_t)((((uint32_t)(hi - lo) * frac) +
                           (1uL << (ATAN_FRAC_BITS - 1u))) >> ATAN_FRAC_BITS);
}

static uint32_t
mag_u32(int32_t a)
{
    return (a < 0) ? (0u - (uint32_t)a) : (uint32_t)a;
}

int32_t
imu_atan2_cdeg(int32_t y, int32_t x)
{
    uint32_t ax = mag_u32(x);
    uint32_t ay = mag_u32(y);
    int32_t a;

    if ((0u == ax) && (0u == ay))
    {
        return 0;
    }
    while ((ax > ATAN_INPUT_MAX) || (ay > ATAN_INPUT_MAX))
    {
        ax >>= 1;
        ay >>= 1;
    }

    /* First octant via the table, then unfold */
    if (ay <= ax)
    {
        a = atan_q15((ay << ATAN_RATIO_BITS) / ax);
    }
    else
    {
        a = IMU_CDEG_90 - atan_q15((ax << ATAN_RATIO_BITS) / ay);
    }
    if (x < 0)
    {
        a = IMU_CDEG_180 - a;
    }
    if (y < 0)
    {
        a = -a;
    }
    return a;
}

int32_t
imu_atan2_cdeg64(int64_t y, int64_t x)
{
    uint64_t ax = (x < 0) ? (0u - (uint64_t)x) : (uint64_t)x;
    uint64_t ay = (y < 0) ? (0u - (uint64_t)y) : (uint64_t)y;
    int32_t sx;
    int32_t sy;

    while ((ax > ATAN_INPUT_MAX) || (ay > ATAN_INPUT_MAX))
    {
        ax >>= 1;
        ay >>= 1;
    }
    sx = (x < 0) ? -(int32_t)ax : (int32_t)ax;
    sy = (y < 0) ? -(int32_t)ay : (int32_t)ay;
    if ((0 == sx) && (0 == sy) && ((0 != x) || (0 != y)))
    {
        /* Tiny against the other: keep the sign of the larger one */
        sx = (x < 0) ? -1 : ((x > 0) ? 1 : 0);
        sy = (y < 0) ? -1 : ((y > 0) ? 1 : 0);
    }
    return imu_atan2_cdeg(sy, sx);
}

int32_t
imu_wrap_cdeg(int32_t a)
{
    while (a > IMU_CDEG_180)
    {
        a -= IMU_CDEG_360;
    }
    while (a <= -IMU_CDEG_180)
    {
        a += IMU_CDEG_360;
    }
    return a;
}

int32_t
imu_div_round(int32_t num, int32_t den)
{
    return (num >= 0) ? ((num + (den / 2)) / den) :
                        -(((-num) + (den / 2)) / den);
}

int64_t
imu_div_round64(int64_t num, int64_t den)
{
    return (num >= 0) ? ((num + (den / 2)) / den) :
                        -(((-num) + (den / 2)) / den);
}

int32_t
imu_abs(int32_t a)
{
    if (INT32_MIN == a)
    {
        return INT32_MAX;
    }
    return (a < 0) ? -a : a;
}

int32_t
imu_clamp(int32_t a, int32_t lo, int32_t hi)
{
    if (a < lo)
    {
        return lo;
    }
    return (a > hi) ? hi : a;
}
