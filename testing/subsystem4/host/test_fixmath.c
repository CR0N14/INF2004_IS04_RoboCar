/*
 *  test_fixmath.c
 *  S4-H01..H02: integer maths against the host's floating point.
 */

#include <math.h>
#include <stdint.h>
#include "test_host.h"
#include "imu_fixmath.h"

#define PI  3.14159265358979323846

void
test_fixmath(void)
{
    uint32_t x;
    uint32_t bad_sqrt = 0u;
    int32_t worst = 0;
    int a;
    int r;

    /* S4-H01 isqrt: floor(sqrt(x)) */
    for (x = 0u; x < 2000000u; x++)
    {
        uint32_t const s = imu_isqrt(x);

        if (((uint64_t)s * s > x) || ((uint64_t)(s + 1u) * (s + 1u) <= x))
        {
            bad_sqrt++;
        }
    }
    for (x = 0xFFFF0000u; x != 0u; x += 0x1001u)
    {
        uint32_t const s = imu_isqrt(x);

        if (((uint64_t)s * s > x) || ((uint64_t)(s + 1u) * (s + 1u) <= x))
        {
            bad_sqrt++;
        }
    }
    CHECK("S4-H01", 0u == bad_sqrt, "isqrt exact (%u wrong)", bad_sqrt);

    /* S4-H02 atan2 within 0.01 deg, every quadrant, several radii */
    for (r = 1; r <= 100000; r *= 10)
    {
        for (a = -17999; a <= 18000; a += 7)
        {
            double const rad = (a / 100.0) * PI / 180.0;
            int32_t const yy = (int32_t)lround(r * 30.0 * sin(rad));
            int32_t const xx = (int32_t)lround(r * 30.0 * cos(rad));
            double const ref = atan2((double)yy, (double)xx) * 18000.0 / PI;
            int32_t const got = imu_atan2_cdeg(yy, xx);
            int32_t err = imu_wrap_cdeg(got - (int32_t)lround(ref));

            err = (err < 0) ? -err : err;
            worst = (err > worst) ? err : worst;
        }
    }
    CHECK("S4-H02", worst <= 1, "atan2 worst error %d cdeg (limit 1)",
          (int)worst);
    CHECK("S4-H02", (imu_atan2_cdeg64(-4000000000LL, 1) == -9000) &&
          (imu_atan2_cdeg(0, -5) == 18000) && (imu_atan2_cdeg(0, 0) == 0),
          "atan2 edge cases");
    CHECK("S4-H02", (imu_wrap_cdeg(35900) == -100) &&
          (imu_wrap_cdeg(-18000) == 18000) && (imu_div_round(-7, 2) == -4) &&
          (imu_div_round(7, 2) == 4), "wrap and rounding");
}
