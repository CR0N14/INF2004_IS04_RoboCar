/*
 *  imu_fixmath.h
 *  Integer maths for Subsystem 4.  The Cortex-M0+ has no FPU and the
 *  build links no libm, so angles are integers in centidegrees (cdeg,
 *  0.01 deg) and square roots are integer square roots.
 *
 *  No kernel dependencies (host tests include it).
 */

#ifndef IMU_FIXMATH_H
#define IMU_FIXMATH_H

#include <stdint.h>

#define IMU_CDEG_90         9000
#define IMU_CDEG_180        18000
#define IMU_CDEG_360        36000

/* floor(sqrt(x)) */
uint32_t    imu_isqrt(uint32_t x);

/* atan2(y, x) in cdeg, -18000 < result <= 18000; 0 for (0, 0).
   Error below 0.01 deg. */
int32_t     imu_atan2_cdeg(int32_t y, int32_t x);
int32_t     imu_atan2_cdeg64(int64_t y, int64_t x);

/* Angle difference folded into -18000 < result <= 18000 */
int32_t     imu_wrap_cdeg(int32_t a);

/* num / den rounded half away from zero; den > 0 */
int32_t     imu_div_round(int32_t num, int32_t den);
int64_t     imu_div_round64(int64_t num, int64_t den);

/* |a| with INT32_MIN mapped to INT32_MAX */
int32_t     imu_abs(int32_t a);

int32_t     imu_clamp(int32_t a, int32_t lo, int32_t hi);

#endif /* IMU_FIXMATH_H */
