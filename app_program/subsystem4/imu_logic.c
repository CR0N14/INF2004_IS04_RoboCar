/*
 *  imu_logic.c
 *  IMU processing (design review section 6.4).  Pure logic: no kernel
 *  calls.  See imu_logic.h.
 *
 *  Each 10 ms cycle:
 *    1. mean of the cycle's samples, vibration, largest horizontal spike
 *    2. vertical acceleration = mean projected on the gravity direction
 *       of the previous cycle, minus 1 g and a slowly tracked bias
 *    3. gravity low-pass, then pitch, roll and the lever-arm correction
 *       (IMU height -> rear-axle height) from it
 *    4. heading and turn rate when a magnetometer sample arrived
 *    5. hump state machine, motion class, collision, tip-over
 *
 *  Hump height.  While the car is flat and steady the integration start
 *  ("anchor") keeps moving forward, ANCHOR_BACKOFF cycles behind the
 *  present, so when the castor meets the ramp the anchor is still on the
 *  flat.  The IMU (behind and above the rear axle) first dips while only
 *  the castor climbs, then rises once the rear wheels are on the ramp.
 *  CLIMBING starts when pitch is above +3 deg, the IMU has dipped, and
 *  the IMU itself has risen for 50 ms.  Forward acceleration tilts the
 *  gravity estimate (0.1 g reads as 5.7 deg) and adds a^2 / 2g to the
 *  vertical estimate, so it can look like pitch plus rise, but it never
 *  produces the dip, so it cannot start a hump.  At the end the
 *  history is integrated again from the anchor, a constant bias and a
 *  start-velocity error are removed so that height and vertical speed
 *  are zero at both ends, and the peak of the rear-axle height
 *  z_axle = z_imu + back * sin(pitch) + up * (1 - cos(pitch)) is the
 *  hump's height.
 */

#include <string.h>
#include "imu_logic.h"
#include "imu_fixmath.h"
#include "imu_params.h"

#define CYCLE_MS            10
#define MS_TO_CYCLES(ms)    ((uint32_t)(((ms) + CYCLE_MS - 1) / CYCLE_MS))

#define GRAV_FRAC           16      /* grav_f = q2 x 16                  */
#define VIB_FRAC            8       /* vib_f = q2 x 8                    */
#define VIB_SHIFT_DIV       8       /* vibration low-pass: 1/8 per cycle */
#define VBIAS_FRAC          256     /* vbias_f = q2 x 256, tau 2.56 s    */

#define RING_LEN            8u      /* v_ring                            */
#define PITCH_RING_LEN      32u     /* pitch_ring                        */
#define RAW_RING_LEN        32u     /* raw_ring (> ANCHOR_BACKOFF)       */
#define STEADY_CYCLES       20u     /* "steady": pitch change over 200 ms */
#define STEADY_CDEG         40      /*   below 0.4 deg (2 deg/s)         */
#define ANCHOR_BACKOFF      16u     /* anchor this far behind steady-flat */
#define PRE_MAX_CYCLES      200u    /* FLAT but never steady: re-anchor  */
#define SEED_CYCLES         16u     /* settle before looking for humps   */
#define HIST_MARGIN         16u
#define HEAD_STALE_CYCLES   10u     /* no heading for 100 ms: rate 0     */

#define HOLD_CYCLES         MS_TO_CYCLES(IMU_CLASS_HOLD_MS)
#define RISE_CYCLES         MS_TO_CYCLES(IMU_HUMP_RISE_MS)
#define LEVEL_CYCLES        MS_TO_CYCLES(IMU_HUMP_LEVEL_MS)
#define CLIMB_CYCLES        MS_TO_CYCLES(IMU_HUMP_CLIMB_MS)
#define DESC_CYCLES         MS_TO_CYCLES(IMU_HUMP_DESC_MS)
#define TURN_CYCLES         MS_TO_CYCLES(IMU_TURN_WINDOW_MS)
#define IMPACT_CYCLES       MS_TO_CYCLES(IMU_COLLISION_IGNORE_MS)
#define CONFIRM_CYCLES      MS_TO_CYCLES(IMU_COLLISION_CONFIRM_MS)
#define TIP_CYCLES          MS_TO_CYCLES(IMU_TIP_MS)

/*
 * Integration sums -> micrometres.  v_sum is in q2 x cycles, z_sum in
 * q2 x cycles^2.  1 q2 = 0.25 mg = 2.4516625 mm/s^2, one cycle = 0.01 s:
 * z_um = z_sum x 2.4516625 x 0.0001 x 1000 = z_sum x 980665 / 4000000.
 */
#define Z_NUM               980665
#define Z_DEN               4000000
#define RISE_SUM            ((int64_t)IMU_HUMP_RISE_UM * Z_DEN / Z_NUM)
#define DIP_SUM             ((int64_t)IMU_HUMP_DIP_UM * Z_DEN / Z_NUM)

#define CORR_UNIT_UM        10      /* hist_corr unit                    */
#define Q16_SHIFT           16u
#define Q16_ONE             65536
#define Z_UM_Q16            16067   /* 0.24516625 x 65536: z_sum -> um   */
#define FINISH_CHUNK        128u    /* history cycles processed per step */
#define UM_PER_MM           1000
#define CDPS_PER_CDEG_CYCLE 100     /* cdeg per cycle -> cdeg per s      */
/* LSM303D magnetometer at +-2 gauss: 0.080 mgauss/LSB on every axis
   (DocID023312 Table 3) = 2/25; full scale 25000 counts */
#define MAG_MG_NUM          2
#define MAG_MG_DEN          25
#define MAG_RAW_MAX         25000
#define MAG_SAMPLES_PER_S   50      /* read every second cycle           */
#define PERMIL              1000

#if (((IMU_HUMP_RISE_MS) + CYCLE_MS - 1) / CYCLE_MS) > 8
#error "IMU_HUMP_RISE_MS too long for the history ring"
#endif
#if (((IMU_COLLISION_CONFIRM_MS) + CYCLE_MS - 1) / CYCLE_MS) < 2
#error "IMU_COLLISION_CONFIRM_MS must cover at least two cycles"
#endif

/* ---- Small helpers ------------------------------------------------ */

static int16_t
sat16(int32_t a)
{
    return (int16_t)imu_clamp(a, INT16_MIN, INT16_MAX);
}

static uint16_t
satu16(int32_t a)
{
    return (uint16_t)imu_clamp(a, 0, UINT16_MAX);
}

static int32_t
grav(imu_ctx_t const *p_ctx, uint32_t k)
{
    return imu_div_round(p_ctx->grav_f[k], GRAV_FRAC);
}

/* Arguments within +-32767, so each square fits and the sum fits 32 bits */
static uint32_t
sq(int32_t a)
{
    return (uint32_t)(a * a);
}

static uint32_t
norm3(int32_t x, int32_t y, int32_t z)
{
    return imu_isqrt(sq(x) + sq(y) + sq(z));
}

/* a / 2^s rounded half away from zero, without a 64-bit division */
static int64_t
shr_round64(int64_t a, uint32_t s)
{
    int64_t const half = (int64_t)1 << (s - 1u);

    return (a >= 0) ? ((a + half) >> s) : -(((-a) + half) >> s);
}

/* Q14 fixed point: 16384 = 1.  Unit vectors are formed at Q12 and
   scaled up so the products stay inside 32 bits. */
#define Q14_ONE             16384
#define Q14_ONE_SMALL       4096
#define Q14_SCALE_UP        4
#define Q14_SHIFT2          28u     /* divide by Q14_ONE x Q14_ONE       */

static int32_t
q14_mul(int32_t a, int32_t b)
{
    return imu_div_round(a * (b / Q14_SCALE_UP), Q14_ONE / Q14_SCALE_UP);
}

static int32_t
median3(int32_t a, int32_t b, int32_t c)
{
    int32_t const lo = (a < b) ? a : b;
    int32_t const hi = (a < b) ? b : a;

    return (c < lo) ? lo : ((c > hi) ? hi : c);
}

static int32_t
pitch_ago(imu_ctx_t const *p_ctx, uint32_t ago)
{
    return p_ctx->pitch_ring[(p_ctx->cycle - ago) % PITCH_RING_LEN];
}

/* ---- Calibration -------------------------------------------------- */

void
imu_cal_reset(imu_cal_t *p_cal)
{
    (void)memset(p_cal, 0, sizeof(*p_cal));
}

void
imu_cal_add(imu_cal_t *p_cal, int16_t const (*p_acc)[3], uint8_t n)
{
    int32_t sum[3] = { 0, 0, 0 };
    uint32_t i;
    uint32_t k;

    if (0u == n)
    {
        return;
    }
    for (i = 0u; i < n; i++)
    {
        for (k = 0u; k < 3u; k++)
        {
            sum[k] += p_acc[i][k];
        }
    }
    for (k = 0u; k < 3u; k++)
    {
        int32_t const mean = imu_div_round(sum[k], (int32_t)n);

        p_cal->sum[k] += sum[k];
        if ((0u == p_cal->cycles) || (mean < p_cal->lo[k]))
        {
            p_cal->lo[k] = mean;
        }
        if ((0u == p_cal->cycles) || (mean > p_cal->hi[k]))
        {
            p_cal->hi[k] = mean;
        }
    }
    p_cal->n += n;
    p_cal->cycles++;
}

imu_cal_result_t
imu_cal_finish(imu_cal_t const *p_cal, uint32_t want_n, imu_align_t *p_align,
               int32_t *p_g_mg)
{
    int32_t mean[3];
    int32_t n[3];               /* unit "up" vector, Q14                 */
    int32_t mag;
    int32_t k;                  /* 1 / (1 + nz), Q14                     */
    uint32_t i;
    bool b_moved = false;

    *p_g_mg = 0;
    if ((0u == p_cal->n) || (p_cal->n < want_n))
    {
        return IMU_CAL_TOO_FEW;
    }
    for (i = 0u; i < 3u; i++)
    {
        mean[i] = imu_div_round(p_cal->sum[i], (int32_t)p_cal->n);
        if ((p_cal->hi[i] - p_cal->lo[i]) > (IMU_ST_STILL_MG * IMU_Q2_PER_MG))
        {
            b_moved = true;
        }
    }
    mag = (int32_t)norm3(mean[0], mean[1], mean[2]);
    *p_g_mg = imu_div_round(mag, IMU_Q2_PER_MG);
    if (b_moved)
    {
        return IMU_CAL_MOVED;
    }
    if ((imu_abs(*p_g_mg - (IMU_Q2_1G / IMU_Q2_PER_MG)) > IMU_ST_G_TOL_MG) ||
        (mean[2] <= 0))
    {
        return IMU_CAL_NOT_1G;
    }

    /* Rodrigues rotation taking n onto +z (rotation axis n x z):
         | 1 - k nx^2   -k nx ny    -nx |
         | -k nx ny     1 - k ny^2  -ny |
         | nx           ny           nz | */
    for (i = 0u; i < 3u; i++)
    {
        n[i] = imu_div_round(mean[i] * Q14_ONE_SMALL, mag) * Q14_SCALE_UP;
    }
    k = imu_div_round(Q14_ONE * Q14_ONE_SMALL, Q14_ONE + n[2]) * Q14_SCALE_UP;
    p_align->r[0][0] = sat16(Q14_ONE - q14_mul(k, q14_mul(n[0], n[0])));
    p_align->r[0][1] = sat16(-q14_mul(k, q14_mul(n[0], n[1])));
    p_align->r[0][2] = sat16(-n[0]);
    p_align->r[1][0] = p_align->r[0][1];
    p_align->r[1][1] = sat16(Q14_ONE - q14_mul(k, q14_mul(n[1], n[1])));
    p_align->r[1][2] = sat16(-n[1]);
    p_align->r[2][0] = sat16(n[0]);
    p_align->r[2][1] = sat16(n[1]);
    p_align->r[2][2] = sat16(n[2]);
    p_align->gain_q14 = imu_div_round(IMU_Q2_1G * Q14_ONE, mag);
    return IMU_CAL_OK;
}

void
imu_align_identity(imu_align_t *p_align)
{
    uint32_t i;
    uint32_t j;

    for (i = 0u; i < 3u; i++)
    {
        for (j = 0u; j < 3u; j++)
        {
            p_align->r[i][j] = (int16_t)((i == j) ? Q14_ONE : 0);
        }
    }
    p_align->gain_q14 = Q14_ONE;
}

static void
rotate(imu_align_t const *p_align, int16_t const *p_in, int16_t *p_out,
       int64_t gain_q14)
{
    uint32_t i;

    for (i = 0u; i < 3u; i++)
    {
        int64_t const dot = ((int64_t)p_align->r[i][0] * p_in[0]) +
                            ((int64_t)p_align->r[i][1] * p_in[1]) +
                            ((int64_t)p_align->r[i][2] * p_in[2]);

        p_out[i] = sat16((int32_t)shr_round64(dot * gain_q14, Q14_SHIFT2));
    }
}

void
imu_align_acc(imu_align_t const *p_align, int16_t const *p_in, int16_t *p_out)
{
    rotate(p_align, p_in, p_out, p_align->gain_q14);
}

void
imu_align_mag(imu_align_t const *p_align, int16_t const *p_in, int16_t *p_out)
{
    rotate(p_align, p_in, p_out, Q14_ONE);
}

/* ---- Axis mapping ------------------------------------------------- */

void
imu_map_acc(int16_t const *p_raw, int16_t *p_body_q2)
{
    static int32_t const zg_q2[3] =
    {
        IMU_ACC_ZG_X_MG * IMU_Q2_PER_MG,
        IMU_ACC_ZG_Y_MG * IMU_Q2_PER_MG,
        IMU_ACC_ZG_Z_MG * IMU_Q2_PER_MG
    };
    static int32_t const scale[3] =
    {
        IMU_ACC_SCALE_X_PERMIL, IMU_ACC_SCALE_Y_PERMIL, IMU_ACC_SCALE_Z_PERMIL
    };
    int32_t s[3];
    uint32_t k;

    /* counts -> q2, minus the zero-g offset, then the per-axis gain */
    for (k = 0u; k < 3u; k++)
    {
        s[k] = imu_div_round((int32_t)p_raw[k] * IMU_Q2_1G, IMU_ACC_RAW_PER_G) -
               zg_q2[k];
        s[k] = imu_div_round(s[k] * PERMIL, scale[k]);
    }
    p_body_q2[0] = sat16(IMU_ACC_BX_SIGN * s[IMU_ACC_BX_AXIS]);
    p_body_q2[1] = sat16(IMU_ACC_BY_SIGN * s[IMU_ACC_BY_AXIS]);
    p_body_q2[2] = sat16(IMU_ACC_BZ_SIGN * s[IMU_ACC_BZ_AXIS]);
}

bool
imu_map_mag(int16_t const *p_raw, int16_t *p_body_mg)
{
    int32_t s[3];
    int32_t b[3];
    uint32_t k;

    for (k = 0u; k < 3u; k++)
    {
        if (imu_abs(p_raw[k]) > MAG_RAW_MAX)
        {
            return false;
        }
    }
    for (k = 0u; k < 3u; k++)
    {
        s[k] = imu_div_round((int32_t)p_raw[k] * MAG_MG_NUM, MAG_MG_DEN);
    }
    b[0] = (IMU_MAG_BX_SIGN * s[IMU_MAG_BX_AXIS]) - IMU_MAG_OFF_BX_MG;
    b[1] = (IMU_MAG_BY_SIGN * s[IMU_MAG_BY_AXIS]) - IMU_MAG_OFF_BY_MG;
    b[2] = (IMU_MAG_BZ_SIGN * s[IMU_MAG_BZ_AXIS]) - IMU_MAG_OFF_BZ_MG;
    p_body_mg[0] = sat16(imu_div_round(b[0] * IMU_MAG_SCALE_BX_PERMIL, PERMIL));
    p_body_mg[1] = sat16(imu_div_round(b[1] * IMU_MAG_SCALE_BY_PERMIL, PERMIL));
    p_body_mg[2] = sat16(b[2]);
    return true;
}

/* ---- Init --------------------------------------------------------- */

void
imu_logic_init(imu_ctx_t *p_ctx, bool b_level_check)
{
    (void)memset(p_ctx, 0, sizeof(*p_ctx));
    p_ctx->b_level_check = b_level_check;
    p_ctx->view.cls = (uint8_t)IMU_CLASS_STATIONARY;
}

void
imu_logic_restart(imu_ctx_t *p_ctx)
{
    bool const b_level = p_ctx->b_level_check;
    uint8_t const last = p_ctx->view.last_peak_mm;
    uint8_t const max = p_ctx->view.max_peak_mm;
    bool const b_motors = p_ctx->b_motors_prev;

    imu_logic_init(p_ctx, b_level);
    p_ctx->view.last_peak_mm = last;
    p_ctx->view.max_peak_mm = max;
    p_ctx->b_motors_prev = b_motors;
}

uint16_t
imu_logic_take_spike(imu_ctx_t *p_ctx)
{
    uint16_t const spike = p_ctx->view.spike_mg;

    p_ctx->view.spike_mg = 0u;
    return spike;
}

/* ---- 1. Cycle statistics ------------------------------------------ */

/* Mean (q2), vibration and the largest horizontal deviation from the
   previous gravity estimate (q2). */
static void
cycle_stats(imu_ctx_t *p_ctx, imu_in_t const *p_in, int32_t *p_mean,
            int32_t *p_spike_q2)
{
    int32_t const gx = grav(p_ctx, 0u);
    int32_t const gy = grav(p_ctx, 1u);
    int32_t sum[3] = { 0, 0, 0 };
    int32_t diff = 0;
    int32_t spike = 0;
    uint32_t i;
    uint32_t k;

    for (i = 0u; i < p_in->n_acc; i++)
    {
        int16_t const *p_a = p_in->acc[i];
        int16_t const *p_prev = (0u == i) ? p_ctx->last_acc : p_in->acc[i - 1u];
        int32_t dx;
        int32_t dy;
        int32_t h;

        for (k = 0u; k < 3u; k++)
        {
            sum[k] += p_a[k];
            diff += imu_abs((int32_t)p_a[k] - p_prev[k]);
        }
        dx = imu_clamp((int32_t)p_a[0] - gx, -INT16_MAX, INT16_MAX);
        dy = imu_clamp((int32_t)p_a[1] - gy, -INT16_MAX, INT16_MAX);
        h = (int32_t)imu_isqrt(sq(dx) + sq(dy));
        spike = (h > spike) ? h : spike;
    }
    for (k = 0u; k < 3u; k++)
    {
        p_mean[k] = imu_div_round(sum[k], (int32_t)p_in->n_acc);
        p_ctx->last_acc[k] = p_in->acc[p_in->n_acc - 1u][k];
    }
    if (p_ctx->b_seeded)
    {
        int32_t const d = imu_div_round(diff, (int32_t)p_in->n_acc);

        p_ctx->vib_f += ((d * VIB_FRAC) - p_ctx->vib_f) / VIB_SHIFT_DIV;
    }
    *p_spike_q2 = spike;
}

/* ---- 4. Heading and turn rate ------------------------------------- */

static void
update_heading(imu_ctx_t *p_ctx, int16_t const *p_mag)
{
    int64_t const gx = grav(p_ctx, 0u);
    int64_t const gy = grav(p_ctx, 1u);
    int64_t const gz = grav(p_ctx, 2u);
    int64_t const a2 = (gx * gx) + (gy * gy) + (gz * gz);
    int64_t const a = (int64_t)imu_isqrt((uint32_t)a2);
    int64_t const mx = p_mag[0];
    int64_t const my = p_mag[1];
    int64_t const mz = p_mag[2];
    int64_t const mdota = (mx * gx) + (my * gy) + (mz * gz);
    int64_t c;
    int64_t s;
    int32_t head;
    int32_t u;
    uint32_t n;
    uint8_t idx;

    /* Horizontal field against the car's forward direction:
       c = forward . N_h, s = up . (forward x N_h), both x |a|^2 */
    c = (mx * a2) - (gx * mdota);
    s = a * ((gy * mz) - (gz * my));
    if ((0 == c) && (0 == s))
    {
        return;                         /* no horizontal field          */
    }
    head = imu_atan2_cdeg64(-s, c);
    if (head < 0)
    {
        head += IMU_CDEG_360;
    }
    p_ctx->view.heading_cdeg = head;
    p_ctx->view.b_heading = true;
    p_ctx->head_cycle = p_ctx->cycle;

    p_ctx->head_ring[p_ctx->head_idx] = head;
    p_ctx->head_idx = (uint8_t)((p_ctx->head_idx + 1u) % IMU_HEAD_HIST_LEN);
    if (p_ctx->head_n < IMU_HEAD_HIST_LEN)
    {
        p_ctx->head_n++;
    }

    /* Turn rate: least-squares slope of the last n headings (one every
       20 ms) over IMU_TURN_WINDOW_MS.  With u_k = 2k - (n - 1):
       slope = 2 sum(u d) / sum(u^2) per sample, x 50 samples/s.  The
       magnetometer's ~5 mG noise makes a plain two-point difference
       about 5 deg/s noisy; the slope over 300 ms is about 2 deg/s. */
    n = (TURN_CYCLES / 2u) + 1u;
    n = (n > IMU_HEAD_HIST_LEN) ? IMU_HEAD_HIST_LEN : n;
    if ((n >= 3u) && (p_ctx->head_n >= n))
    {
        int32_t sum_ud = 0;
        int32_t sum_uu = 0;
        uint32_t k;

        for (k = 0u; k < n; k++)
        {
            /* oldest (k = 0) to newest (k = n - 1), relative to newest */
            idx = (uint8_t)((p_ctx->head_idx + IMU_HEAD_HIST_LEN - n + k) %
                            IMU_HEAD_HIST_LEN);
            u = (int32_t)(2u * k) - (int32_t)(n - 1u);
            sum_ud += u * imu_wrap_cdeg(p_ctx->head_ring[idx] - head);
            sum_uu += u * u;
        }
        p_ctx->view.rate_cdps = imu_div_round(2 * MAG_SAMPLES_PER_S * sum_ud,
                                              sum_uu);
    }
}

/* ---- 5a. Humps ---------------------------------------------------- */

static void
reanchor(imu_ctx_t *p_ctx)
{
    uint32_t const now = p_ctx->cycle;
    uint32_t i;

    p_ctx->anchor = (now > ANCHOR_BACKOFF) ? (now - ANCHOR_BACKOFF) : 0u;
    p_ctx->v_sum = 0;
    p_ctx->z_sum = 0;
    p_ctx->z_min = 0;
    for (i = p_ctx->anchor + 1u; i <= now; i++)
    {
        p_ctx->v_sum += p_ctx->hist_av[i % IMU_HIST_LEN];
        p_ctx->z_sum += p_ctx->v_sum;
        p_ctx->v_ring[i % RING_LEN] = p_ctx->v_sum;
        p_ctx->z_min = (p_ctx->z_sum < p_ctx->z_min) ? p_ctx->z_sum :
                       p_ctx->z_min;
    }
}

static void
integrate(imu_ctx_t *p_ctx, int16_t av)
{
    p_ctx->v_sum += av;
    p_ctx->z_sum += p_ctx->v_sum;
    p_ctx->v_ring[p_ctx->cycle % RING_LEN] = p_ctx->v_sum;
    p_ctx->z_min = (p_ctx->z_sum < p_ctx->z_min) ? p_ctx->z_sum :
                   p_ctx->z_min;
}

/* Rise of the IMU over the last RISE_CYCLES, in z_sum units */
static int64_t
recent_rise(imu_ctx_t const *p_ctx)
{
    int64_t rise = 0;
    uint32_t i;

    for (i = 0u; i < RISE_CYCLES; i++)
    {
        rise += p_ctx->v_ring[(p_ctx->cycle - i) % RING_LEN];
    }
    return rise;
}

/*
 * End of a hump: integrate the history again from the anchor and remove
 * a constant bias b and a start-velocity error u so that height and
 * vertical speed are both zero at the end:  z_err(k) = u k + b k(k+1)/2.
 * With P = v_N N - z_N and D = N (N - 1):  b = 2P / D, u = v_N - 2P/(N-1),
 * so  z_corr(k) = z(k) - k v_N + (P / D) k (2N - k - 1).
 * v_N and z_N are the live sums (same anchor, same history).  P / D is
 * taken once as a Q16 factor, so the loop only multiplies, and the loop
 * is spread over FINISH_CHUNK cycles per 10 ms so no cycle runs long
 * (NFR20).  The peak of the rear-axle height is the hump's height.
 */
static void
finish_start(imu_ctx_t *p_ctx)
{
    uint32_t const n = p_ctx->cycle - p_ctx->anchor;
    int64_t const big_n = (int64_t)n;

    if (n < 3u)
    {
        return;
    }
    p_ctx->fin_active = true;
    p_ctx->fin_anchor = p_ctx->anchor;
    p_ctx->fin_n = n;
    p_ctx->fin_k = 0u;
    p_ctx->fin_vn = p_ctx->v_sum;
    p_ctx->fin_q16 = imu_div_round64((((int64_t)p_ctx->v_sum * big_n) -
                                      p_ctx->z_sum) * Q16_ONE,
                                     big_n * (big_n - 1));
    p_ctx->fin_v = 0;
    p_ctx->fin_z = 0;
    p_ctx->fin_peak = INT32_MIN;
    p_ctx->fin_hump.duration_ms = satu16((int32_t)(n * CYCLE_MS));
    p_ctx->fin_hump.max_pitch_cdeg = p_ctx->ep_max_pitch;
    p_ctx->fin_hump.min_pitch_cdeg = p_ctx->ep_min_pitch;
    p_ctx->fin_hump.max_spike_mg = p_ctx->ep_max_spike;
}

static void
finish_step(imu_ctx_t *p_ctx, imu_out_t *p_out)
{
    int32_t const corr0 = p_ctx->hist_corr[p_ctx->fin_anchor % IMU_HIST_LEN];
    int64_t const big_n = (int64_t)p_ctx->fin_n;
    uint32_t todo = FINISH_CHUNK;
    int32_t peak_mm;

    while ((todo > 0u) && (p_ctx->fin_k < p_ctx->fin_n))
    {
        uint32_t const k = p_ctx->fin_k + 1u;
        uint32_t const h = (p_ctx->fin_anchor + k) % IMU_HIST_LEN;
        int64_t const kk = (int64_t)k;
        int64_t zc;
        int32_t axle;

        p_ctx->fin_v += p_ctx->hist_av[h];
        p_ctx->fin_z += p_ctx->fin_v;
        zc = p_ctx->fin_z - (kk * p_ctx->fin_vn) +
             shr_round64(p_ctx->fin_q16 * (kk * ((2 * big_n) - kk - 1)),
                         Q16_SHIFT);
        axle = (int32_t)shr_round64(zc * Z_UM_Q16, Q16_SHIFT) +
               ((p_ctx->hist_corr[h] - corr0) * CORR_UNIT_UM);
        p_ctx->fin_peak = (axle > p_ctx->fin_peak) ? axle : p_ctx->fin_peak;
        p_ctx->fin_k = k;
        todo--;
    }
    if (p_ctx->fin_k < p_ctx->fin_n)
    {
        return;                         /* more next cycle               */
    }

    p_ctx->fin_active = false;
    peak_mm = imu_clamp(imu_div_round(p_ctx->fin_peak, UM_PER_MM), 0,
                        (int32_t)IMU_PEAK_MAX_MM);
    if (peak_mm < IMU_HUMP_MIN_MM)
    {
        p_out->b_hump_dropped = 1u;     /* too small: noise, not a hump */
        return;
    }
    p_out->b_hump = 1u;
    p_out->hump = p_ctx->fin_hump;
    p_out->hump.peak_mm = (uint8_t)peak_mm;
    p_out->hump.peak_um = p_ctx->fin_peak;
    p_ctx->view.last_peak_mm = (uint8_t)peak_mm;
    if (peak_mm > p_ctx->view.max_peak_mm)
    {
        p_ctx->view.max_peak_mm = (uint8_t)peak_mm;
    }
}

static void
hump_enter(imu_ctx_t *p_ctx, imu_hump_state_t state)
{
    p_ctx->hump_state = (uint8_t)state;
    p_ctx->state_cycle = p_ctx->cycle;
    p_ctx->level_cycles = 0u;
}

static void
hump_step(imu_ctx_t *p_ctx, int16_t av, int32_t pitch, uint16_t spike_mg,
          imu_out_t *p_out)
{
    uint32_t const now = p_ctx->cycle;
    bool const b_steady_flat =
        (imu_abs(pitch) < IMU_HUMP_LEVEL_CDEG) &&
        (imu_abs(pitch - pitch_ago(p_ctx, STEADY_CYCLES)) < STEADY_CDEG);

    if (IMU_HUMP_FLAT == p_ctx->hump_state)
    {
        if (b_steady_flat)
        {
            /* Vertical bias: learn only from the cycle ANCHOR_BACKOFF back,
               which is certainly on the flat.  Recent cycles may already
               hold the castor's kick onto a ramp (about -18 mg), which the
               tracker would otherwise absorb as bias. */
            int32_t const old_raw =
                p_ctx->raw_ring[(now - ANCHOR_BACKOFF) % RAW_RING_LEN];

            /* Running mean of the first samples, then tau 2.56 s */
            if (p_ctx->bias_n < VBIAS_FRAC)
            {
                p_ctx->bias_n++;
            }
            p_ctx->vbias_f += ((old_raw * VBIAS_FRAC) - p_ctx->vbias_f) /
                              (int32_t)p_ctx->bias_n;
            p_ctx->flat_cycle = now;
            reanchor(p_ctx);
        }
        else if ((now - p_ctx->flat_cycle) > PRE_MAX_CYCLES)
        {
            p_ctx->flat_cycle = now;    /* tilted for a long time        */
            reanchor(p_ctx);
        }
        else
        {
            integrate(p_ctx, av);
        }

        /* Climb: nose up, the IMU dipped first (only the castor was on
           the ramp), and now the IMU itself rises (rear wheels on it).
           Forward acceleration tilts the estimate and even makes the
           IMU seem to rise (a^2 / 2g), but never makes it dip. */
        if ((now >= SEED_CYCLES) && !p_ctx->fin_active &&
            (pitch > IMU_HUMP_UP_CDEG) && (p_ctx->z_min <= -DIP_SUM) &&
            (p_ctx->v_sum > 0) && (recent_rise(p_ctx) >= RISE_SUM))
        {
            hump_enter(p_ctx, IMU_HUMP_CLIMBING);
            p_ctx->ep_max_pitch = sat16(pitch);
            p_ctx->ep_min_pitch = sat16(pitch);
            p_ctx->ep_max_spike = spike_mg;
        }
        return;
    }

    /* On a hump */
    integrate(p_ctx, av);
    p_ctx->ep_max_pitch = (pitch > p_ctx->ep_max_pitch) ? sat16(pitch) :
                          p_ctx->ep_max_pitch;
    p_ctx->ep_min_pitch = (pitch < p_ctx->ep_min_pitch) ? sat16(pitch) :
                          p_ctx->ep_min_pitch;
    p_ctx->ep_max_spike = (spike_mg > p_ctx->ep_max_spike) ? spike_mg :
                          p_ctx->ep_max_spike;

    if ((now - p_ctx->anchor) >= (IMU_HIST_LEN - HIST_MARGIN))
    {
        p_out->b_hump_dropped = 1u;     /* history full                 */
        hump_enter(p_ctx, IMU_HUMP_FLAT);
    }
    else if (IMU_HUMP_CLIMBING == p_ctx->hump_state)
    {
        if (pitch < IMU_HUMP_DOWN_CDEG)
        {
            hump_enter(p_ctx, IMU_HUMP_DESCENDING);
        }
        else if ((now - p_ctx->state_cycle) > CLIMB_CYCLES)
        {
            p_out->b_hump_dropped = 1u; /* no descent within 3 s        */
            hump_enter(p_ctx, IMU_HUMP_FLAT);
        }
        else
        {
            /* still climbing */
        }
    }
    else
    {
        p_ctx->level_cycles = (imu_abs(pitch) < IMU_HUMP_END_CDEG) ?
                              (p_ctx->level_cycles + 1u) : 0u;
        if (p_ctx->level_cycles >= LEVEL_CYCLES)
        {
            finish_start(p_ctx);
            hump_enter(p_ctx, IMU_HUMP_FLAT);
        }
        else if ((now - p_ctx->state_cycle) > DESC_CYCLES)
        {
            p_out->b_hump_dropped = 1u;
            hump_enter(p_ctx, IMU_HUMP_FLAT);
        }
        else
        {
            /* still descending */
        }
    }

    if (IMU_HUMP_FLAT == p_ctx->hump_state)
    {
        p_ctx->flat_cycle = now;
        reanchor(p_ctx);
    }
}

/* ---- 5b. Collision and tip-over ----------------------------------- */

static void
collision(imu_ctx_t *p_ctx, uint16_t spike_mg, imu_out_t *p_out)
{
    p_out->b_collision = 1u;
    p_out->collision_mg = spike_mg;
    p_ctx->impact_until = p_ctx->cycle + IMPACT_CYCLES;
    p_ctx->ignore_until = p_ctx->impact_until;
}

static void
collision_step(imu_ctx_t *p_ctx, imu_in_t const *p_in, int32_t const *p_mean,
               uint16_t spike_mg, imu_out_t *p_out)
{
    uint32_t k;

    if (0u != p_ctx->confirm_left)
    {
        /* Level check after a spike: pitch from the means of the cycles
           after it, skipping the first, which still holds the end of the
           crash and the filter's tail */
        if (p_ctx->confirm_left < CONFIRM_CYCLES)
        {
            for (k = 0u; k < 3u; k++)
            {
                p_ctx->confirm_sum[k] += p_mean[k];
            }
        }
        p_ctx->confirm_left--;
        if (0u == p_ctx->confirm_left)
        {
            int32_t const n = (int32_t)CONFIRM_CYCLES - 1;
            int32_t const x = imu_div_round(p_ctx->confirm_sum[0], n);
            int32_t const y = imu_div_round(p_ctx->confirm_sum[1], n);
            int32_t const z = imu_div_round(p_ctx->confirm_sum[2], n);
            int32_t const after =
                imu_atan2_cdeg(x, (int32_t)imu_isqrt(sq(y) + sq(z)));

            if (imu_abs(after - p_ctx->confirm_pitch) <
                IMU_COLLISION_LEVEL_CDEG)
            {
                collision(p_ctx, p_ctx->confirm_spike, p_out);
            }
        }
        return;
    }

    /* "Outside a hump" (FR25): FLAT, and level just before the spike.
       The biggest hump kick comes as the rear wheels meet the ramp, when
       the car is already pitched by the slope; on our course only humps
       tilt the car. */
    if (p_in->b_motors && (IMU_HUMP_FLAT == p_ctx->hump_state) &&
        (imu_abs(pitch_ago(p_ctx, 1u)) < IMU_HUMP_LEVEL_CDEG) &&
        (p_ctx->cycle >= p_ctx->ignore_until) &&
        (spike_mg > IMU_COLLISION_MG))
    {
        if (!p_ctx->b_level_check)
        {
            collision(p_ctx, spike_mg, p_out);
        }
        else
        {
            p_ctx->confirm_left = (uint8_t)CONFIRM_CYCLES;
            p_ctx->confirm_spike = spike_mg;
            p_ctx->confirm_pitch = pitch_ago(p_ctx, 1u);
            for (k = 0u; k < 3u; k++)
            {
                p_ctx->confirm_sum[k] = 0;
            }
        }
    }
}

static void
tip_step(imu_ctx_t *p_ctx, imu_in_t const *p_in, imu_out_t *p_out)
{
    if (p_in->b_motors &&
        ((imu_abs(p_ctx->view.pitch_cdeg) > IMU_TIP_CDEG) ||
         (imu_abs(p_ctx->view.roll_cdeg) > IMU_TIP_CDEG)))
    {
        if (p_ctx->tip_cycles < UINT32_MAX)
        {
            p_ctx->tip_cycles++;
        }
        if ((p_ctx->tip_cycles >= TIP_CYCLES) && !p_ctx->b_tip_sent)
        {
            p_out->b_tipover = 1u;
            p_ctx->b_tip_sent = true;
        }
    }
    else
    {
        p_ctx->tip_cycles = 0u;
        p_ctx->b_tip_sent = false;
    }
}

/* ---- 5c. Motion class --------------------------------------------- */

static bool
hysteresis(bool b_on, int32_t value, int32_t on, int32_t off)
{
    return b_on ? (value > off) : (value > on);
}

static void
class_step(imu_ctx_t *p_ctx, imu_out_t *p_out)
{
    int32_t const rate = imu_abs(p_ctx->view.rate_cdps);
    int32_t const fwd = imu_abs(p_ctx->view.fwd_mg);
    int32_t const vib = p_ctx->view.vib_mg;
    uint8_t cand;

    p_ctx->b_turn = hysteresis(p_ctx->b_turn, rate, IMU_TURN_ON_CDPS,
                               IMU_TURN_OFF_CDPS);
    p_ctx->b_accel = hysteresis(p_ctx->b_accel, fwd, IMU_ACCEL_ON_MG,
                                IMU_ACCEL_OFF_MG);
    if (p_ctx->b_still)
    {
        p_ctx->b_still = (vib <= IMU_STILL_VIB_OFF_MG) &&
                         (rate <= IMU_STILL_RATE_OFF_CDPS) &&
                         (fwd <= IMU_ACCEL_ON_MG);
    }
    else
    {
        p_ctx->b_still = (vib < IMU_STILL_VIB_ON_MG) &&
                         (rate < IMU_STILL_RATE_ON_CDPS) &&
                         (fwd < IMU_ACCEL_OFF_MG);
    }

    if (p_ctx->cycle < p_ctx->impact_until)
    {
        cand = (uint8_t)IMU_CLASS_SUDDEN_IMPACT;
    }
    else if (IMU_HUMP_CLIMBING == p_ctx->hump_state)
    {
        cand = (uint8_t)IMU_CLASS_CLIMBING_HUMP;
    }
    else if (IMU_HUMP_DESCENDING == p_ctx->hump_state)
    {
        cand = (uint8_t)IMU_CLASS_DESCENDING_HUMP;
    }
    else if (p_ctx->b_turn)
    {
        cand = (uint8_t)IMU_CLASS_TURNING;
    }
    else if (p_ctx->b_accel)
    {
        cand = (uint8_t)IMU_CLASS_ACCELERATING;
    }
    else if (p_ctx->b_still)
    {
        cand = (uint8_t)IMU_CLASS_STATIONARY;
    }
    else
    {
        cand = (uint8_t)IMU_CLASS_MOVING_STRAIGHT;
    }

    if (cand == p_ctx->view.cls)
    {
        p_ctx->cand_cycles = 0u;
        return;
    }
    if (cand != p_ctx->cand)
    {
        p_ctx->cand = cand;
        p_ctx->cand_cycles = 0u;
    }
    p_ctx->cand_cycles++;

    /* Sudden impact at once; anything else after holding 100 ms */
    if (((uint8_t)IMU_CLASS_SUDDEN_IMPACT == cand) ||
        (p_ctx->cand_cycles >= HOLD_CYCLES))
    {
        p_ctx->view.cls = cand;
        p_ctx->cand_cycles = 0u;
        p_out->b_class = 1u;
        p_out->cls = cand;
    }
}

/* ---- Step --------------------------------------------------------- */

void
imu_logic_step(imu_ctx_t *p_ctx, imu_in_t const *p_in, imu_out_t *p_out)
{
    int32_t mean[3];
    int32_t g[3];
    int32_t spike_q2 = 0;
    uint16_t spike_mg = 0u;
    int32_t av = p_ctx->last_av;
    int64_t corr;
    uint32_t gn;
    uint32_t k;
    int32_t pitch;

    (void)memset(p_out, 0, sizeof(*p_out));
    if ((0u == p_in->n_acc) && !p_ctx->b_seeded)
    {
        return;                         /* nothing to start from yet     */
    }
    p_ctx->cycle++;

    if (0u != p_in->n_acc)
    {
        uint32_t const n = (p_in->n_acc > IMU_FIFO_MAX) ? IMU_FIFO_MAX :
                           p_in->n_acc;
        imu_in_t in = *p_in;

        in.n_acc = (uint8_t)n;
        if (!p_ctx->b_seeded)
        {
            for (k = 0u; k < 3u; k++)
            {
                p_ctx->last_acc[k] = in.acc[0][k];
            }
        }
        cycle_stats(p_ctx, &in, mean, &spike_q2);
        spike_mg = satu16(imu_div_round(spike_q2, IMU_Q2_PER_MG));

        if (!p_ctx->b_seeded)
        {
            for (k = 0u; k < 3u; k++)
            {
                p_ctx->grav_f[k] = mean[k] * GRAV_FRAC;
                p_ctx->mean_prev[0][k] = sat16(mean[k]);
                p_ctx->mean_prev[1][k] = sat16(mean[k]);
            }
            p_ctx->b_seeded = true;
            spike_mg = 0u;
        }

        /* 2. vertical acceleration on the previous gravity direction */
        for (k = 0u; k < 3u; k++)
        {
            g[k] = grav(p_ctx, k);
        }
        gn = norm3(g[0], g[1], g[2]);
        if (0u != gn)
        {
            int64_t const dot = ((int64_t)mean[0] * g[0]) +
                                ((int64_t)mean[1] * g[1]) +
                                ((int64_t)mean[2] * g[2]);
            int32_t const raw = (int32_t)imu_div_round64(dot, (int64_t)gn) -
                                IMU_Q2_1G;

            /* The bias is learnt in hump_step(), from this value once it
               is known to lie on flat ground */
            p_ctx->raw_ring[p_ctx->cycle % RAW_RING_LEN] = sat16(raw);
            av = raw - imu_div_round(p_ctx->vbias_f, VBIAS_FRAC);
        }

        /* 3. gravity low-pass, fed with the median of the last three
           cycle means so a single-cycle kick does not tilt it */
        for (k = 0u; k < 3u; k++)
        {
            int32_t const med = median3(mean[k], p_ctx->mean_prev[0][k],
                                        p_ctx->mean_prev[1][k]);

            p_ctx->mean_prev[1][k] = p_ctx->mean_prev[0][k];
            p_ctx->mean_prev[0][k] = sat16(mean[k]);
            p_ctx->grav_f[k] += ((med * GRAV_FRAC) - p_ctx->grav_f[k]) /
                                (1 << IMU_GRAV_LPF_SHIFT);
        }
    }
    else
    {
        for (k = 0u; k < 3u; k++)
        {
            mean[k] = grav(p_ctx, k);   /* no samples: hold              */
        }
    }
    p_ctx->last_av = sat16(av);

    /* 3. tilt and lever arm from the filtered gravity */
    for (k = 0u; k < 3u; k++)
    {
        g[k] = grav(p_ctx, k);
    }
    gn = norm3(g[0], g[1], g[2]);
    pitch = imu_atan2_cdeg(g[0], (int32_t)imu_isqrt(sq(g[1]) + sq(g[2])));
    p_ctx->view.pitch_cdeg = pitch;
    p_ctx->view.roll_cdeg = imu_atan2_cdeg(g[1], g[2]);
    p_ctx->view.fwd_mg = sat16(imu_div_round(g[0], IMU_Q2_PER_MG));
    p_ctx->view.vib_mg = satu16(imu_div_round(p_ctx->vib_f,
                                              VIB_FRAC * IMU_Q2_PER_MG));
    p_ctx->pitch_ring[p_ctx->cycle % PITCH_RING_LEN] = sat16(pitch);
    corr = 0;
    if (0u != gn)
    {
        corr = (((int64_t)IMU_LEVER_BACK_UM * g[0]) +
                ((int64_t)IMU_LEVER_UP_UM * ((int32_t)gn - g[2]))) /
               (int64_t)gn;
    }
    p_ctx->hist_av[p_ctx->cycle % IMU_HIST_LEN] = p_ctx->last_av;
    p_ctx->hist_corr[p_ctx->cycle % IMU_HIST_LEN] =
        sat16((int32_t)(corr / CORR_UNIT_UM));

    /* 4. heading and turn rate */
    if (p_in->b_mag)
    {
        update_heading(p_ctx, p_in->mag);
    }
    if ((p_ctx->cycle - p_ctx->head_cycle) > HEAD_STALE_CYCLES)
    {
        p_ctx->view.rate_cdps = 0;      /* magnetometer silent           */
        p_ctx->head_n = 0u;
    }
    p_ctx->view.spike_mg = (spike_mg > p_ctx->view.spike_mg) ? spike_mg :
                           p_ctx->view.spike_mg;

    /* 5. humps, collision, tip-over, motion class */
    if (p_in->b_motors && !p_ctx->b_motors_prev)
    {
        p_ctx->view.max_peak_mm = 0u;   /* new run (FR23)                */
    }
    p_ctx->b_motors_prev = p_in->b_motors;

    if (p_ctx->fin_active)
    {
        finish_step(p_ctx, p_out);
    }
    hump_step(p_ctx, p_ctx->last_av, pitch, spike_mg, p_out);
    collision_step(p_ctx, p_in, mean, spike_mg, p_out);
    tip_step(p_ctx, p_in, p_out);
    class_step(p_ctx, p_out);
    p_ctx->view.hump_state = p_ctx->hump_state;
}
