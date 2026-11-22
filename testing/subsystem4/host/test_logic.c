/*
 *  test_logic.c
 *  S4-H03..H16: imu_logic against the car/hump model and synthetic data.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "test_host.h"
#include "hump_model.h"
#include "imu_logic.h"
#include "imu_fixmath.h"
#include "imu_params.h"

#define MAX_EVT         32
#define CYCLES_PER_S    100

typedef struct
{
    int      humps;
    imu_hump_t hump[MAX_EVT];
    int      dropped;
    int      collisions;
    int      tipovers;
    int      n_class;
    uint8_t  cls[MAX_EVT];
    uint32_t cls_cycle[MAX_EVT];
    uint16_t pk_flat;           /* largest spike while FLAT, motors on   */
    uint16_t pk_hump_region;    /* ... near the hump                     */
    imu_view_t view;
    double   min_pitch_deg;
    double   max_pitch_deg;
    imu_cal_result_t cal_result;
} run_t;

static imu_ctx_t g_ctx;
static model_t   g_model;

/* Run one model scenario through the logic.  With b_cal the first
   still_s seconds go to the at-rest calibration (as the self-test does
   at boot) and every later sample is aligned before the logic sees it. */
static void
run_model_cal(model_cfg_t const *p_cfg, bool b_level_check, bool b_cal,
              run_t *p_r)
{
    imu_in_t in;
    imu_out_t out;
    imu_cal_t cal;
    imu_align_t align;
    uint32_t cycle = 0u;
    uint32_t const cal_cycles = (uint32_t)(p_cfg->still_s * CYCLES_PER_S);
    int32_t g_mg;

    (void)memset(p_r, 0, sizeof(*p_r));
    imu_logic_init(&g_ctx, b_level_check);
    imu_align_identity(&align);
    imu_cal_reset(&cal);
    model_start(&g_model, p_cfg);
    p_r->cal_result = IMU_CAL_OK;
    while (model_cycle(&g_model, &in))
    {
        uint16_t pk;
        int i;
        bool const b_near = (p_cfg->h_cm > 0.0) &&
                            (g_model.x > (p_cfg->start_cm - 12.0)) &&
                            (g_model.x < (p_cfg->start_cm + p_cfg->length_cm +
                                          3.0));

        cycle++;
        if (b_cal && (cycle <= cal_cycles))
        {
            imu_cal_add(&cal, (int16_t const (*)[3])in.acc, in.n_acc);
            if (cycle == cal_cycles)
            {
                p_r->cal_result = imu_cal_finish(&cal, (cal_cycles * 4u * 9u) /
                                                 10u, &align, &g_mg);
            }
            continue;
        }
        for (i = 0; i < in.n_acc; i++)
        {
            int16_t raw[3];

            (void)memcpy(raw, in.acc[i], sizeof(raw));
            imu_align_acc(&align, raw, in.acc[i]);
        }
        if (in.b_mag)
        {
            int16_t raw[3];

            (void)memcpy(raw, in.mag, sizeof(raw));
            imu_align_mag(&align, raw, in.mag);
        }

        imu_logic_step(&g_ctx, &in, &out);
        pk = imu_logic_take_spike(&g_ctx);
        if ((IMU_HUMP_FLAT == g_ctx.view.hump_state) &&
            (cycle > (cal_cycles + 20u)))
        {
            p_r->pk_flat = (pk > p_r->pk_flat) ? pk : p_r->pk_flat;
            if (b_near)
            {
                p_r->pk_hump_region = (pk > p_r->pk_hump_region) ? pk :
                                      p_r->pk_hump_region;
            }
        }
        if (out.b_hump && (p_r->humps < MAX_EVT))
        {
            p_r->hump[p_r->humps] = out.hump;
            p_r->humps++;
        }
        p_r->dropped += (int)out.b_hump_dropped;
        p_r->collisions += (int)out.b_collision;
        p_r->tipovers += (int)out.b_tipover;
        if (out.b_class && (p_r->n_class < MAX_EVT))
        {
            p_r->cls[p_r->n_class] = (uint8_t)out.cls;
            p_r->cls_cycle[p_r->n_class] = cycle;
            p_r->n_class++;
        }
        if (g_model.pitch_true_deg < p_r->min_pitch_deg)
        {
            p_r->min_pitch_deg = g_model.pitch_true_deg;
        }
        if (g_model.pitch_true_deg > p_r->max_pitch_deg)
        {
            p_r->max_pitch_deg = g_model.pitch_true_deg;
        }
    }
    p_r->view = g_ctx.view;
}

static void
run_model(model_cfg_t const *p_cfg, bool b_level_check, run_t *p_r)
{
    run_model_cal(p_cfg, b_level_check, p_cfg->still_s > 0.0, p_r);
}

static void
hump_cfg(model_cfg_t *p_cfg, bool b_big, double wheelbase, double speed)
{
    model_default(p_cfg);
    p_cfg->h_cm = b_big ? 2.3 : 1.0;
    p_cfg->length_cm = b_big ? 19.75 : 19.25;
    p_cfg->top_cm = b_big ? 0.0 : 0.5;
    p_cfg->wheelbase_cm = wheelbase;
    p_cfg->speed_cms = speed;
    p_cfg->start_cm = 30.0;
    p_cfg->still_s = 1.0;                       /* self-test at rest     */
    p_cfg->duration_s = p_cfg->still_s + 0.5 +
                        ((p_cfg->start_cm + p_cfg->length_cm + 30.0) / speed);
}

static bool
nfr15_ok(double got_mm, double true_mm)
{
    double const tol = (0.2 * true_mm > 5.0) ? (0.2 * true_mm) : 5.0;

    return fabs(got_mm - true_mm) <= tol;
}

/* ---- Synthetic samples -------------------------------------------- */

static void
fill_static(imu_in_t *p_in, double pitch_deg, double roll_deg, bool b_motors)
{
    double const th = pitch_deg * 3.14159265358979323846 / 180.0;
    double const ph = roll_deg * 3.14159265358979323846 / 180.0;
    int i;

    (void)memset(p_in, 0, sizeof(*p_in));
    for (i = 0; i < 4; i++)
    {
        p_in->acc[i][0] = (int16_t)lround(4000.0 * sin(th));
        p_in->acc[i][1] = (int16_t)lround(4000.0 * cos(th) * sin(ph));
        p_in->acc[i][2] = (int16_t)lround(4000.0 * cos(th) * cos(ph));
    }
    p_in->n_acc = 4u;
    p_in->b_motors = b_motors;
}

/* ---- Tests -------------------------------------------------------- */

static void
test_mapping_and_cal(void)
{
    int16_t raw[3];
    int16_t b[3];
    int16_t m[3];
    int16_t samples[4][3];
    imu_align_t align;
    imu_cal_t cal;
    imu_cal_result_t res;
    int32_t g_mg;
    int i;
    int c;

    /* S4-H03 axis mapping and scaling with the imu_params.h mapping,
       zero-g offsets and per-axis scale: the counts the sensor gives for
       1 g along body z must read as body z +1 g (within 0.5 mg) */
    {
        static int const zg_mg[3] =
        {
            IMU_ACC_ZG_X_MG, IMU_ACC_ZG_Y_MG, IMU_ACC_ZG_Z_MG
        };
        static int const scale[3] =
        {
            IMU_ACC_SCALE_X_PERMIL, IMU_ACC_SCALE_Y_PERMIL,
            IMU_ACC_SCALE_Z_PERMIL
        };

        for (i = 0; i < 3; i++)
        {
            double const true_mg = (IMU_ACC_BZ_AXIS == i) ?
                                   (IMU_ACC_BZ_SIGN * 1000.0) : 0.0;

            raw[i] = (int16_t)lround((zg_mg[i] +
                                      (scale[i] * true_mg / 1000.0)) *
                                     IMU_ACC_RAW_PER_G / 1000.0);
        }
    }
    imu_map_acc(raw, b);
    CHECK("S4-H03", (abs(b[0]) <= 2) && (abs(b[1]) <= 2) &&
          (abs(b[2] - 4000) <= 2),
          "1 g on the mapped z axis, with the zero-g offsets and scale -> "
          "body z +1 g (%d %d %d q2)", b[0], b[1], b[2]);
    /* 12500, -6250, 5000 counts = 1000, -500, 400 mG on the sensor
       axes; then the mapping and the hard-iron offsets of imu_params.h */
    raw[0] = 12500; raw[1] = -6250; raw[2] = 5000;
    {
        int32_t const s_mg[3] = { 1000, -500, 400 };
        int32_t const want_x = (IMU_MAG_BX_SIGN * s_mg[IMU_MAG_BX_AXIS]) -
                               IMU_MAG_OFF_BX_MG;
        int32_t const want_y = (IMU_MAG_BY_SIGN * s_mg[IMU_MAG_BY_AXIS]) -
                               IMU_MAG_OFF_BY_MG;
        int32_t const want_z = (IMU_MAG_BZ_SIGN * s_mg[IMU_MAG_BZ_AXIS]) -
                               IMU_MAG_OFF_BZ_MG;

        CHECK("S4-H03", imu_map_mag(raw, m) && (m[0] == want_x) &&
              (m[1] == want_y) && (m[2] == want_z),
              "magnetometer 0.080 mG/LSB, mapping and offsets (%d %d %d mG)",
              m[0], m[1], m[2]);
    }
    raw[0] = 25001;
    CHECK("S4-H03", !imu_map_mag(raw, m), "beyond +-2 gauss rejected");

    /* S4-H04 calibration at rest: board tilted 3 deg nose up and 2 deg
       left, |g| 2.5 % high -> aligned resting reading is (0, 0, 1 g) */
    imu_cal_reset(&cal);
    for (c = 0; c < 100; c++)
    {
        for (i = 0; i < 4; i++)
        {
            samples[i][0] = (int16_t)lround(4100.0 * sin(0.05236) +
                                            ((c + i) % 3) - 1);
            samples[i][1] = (int16_t)lround(4100.0 * cos(0.05236) *
                                            sin(0.03491));
            samples[i][2] = (int16_t)lround(4100.0 * cos(0.05236) *
                                            cos(0.03491));
        }
        imu_cal_add(&cal, samples, 4u);
    }
    res = imu_cal_finish(&cal, 360u, &align, &g_mg);
    imu_align_acc(&align, samples[0], b);
    CHECK("S4-H04", (IMU_CAL_OK == res) && (imu_abs(b[0]) <= 4) &&
          (imu_abs(b[1]) <= 4) && (imu_abs(b[2] - 4000) <= 4),
          "tilted board: aligned rest reading %d %d %d q2, |g| %d mg", b[0],
          b[1], b[2], (int)g_mg);

    imu_cal_reset(&cal);
    for (c = 0; c < 100; c++)
    {
        for (i = 0; i < 4; i++)
        {
            samples[i][0] = (int16_t)((c < 50) ? 0 : 800);      /* moved   */
            samples[i][1] = 0;
            samples[i][2] = 4000;
        }
        imu_cal_add(&cal, samples, 4u);
    }
    CHECK("S4-H04", IMU_CAL_MOVED == imu_cal_finish(&cal, 360u, &align, &g_mg),
          "moving car rejected");

    imu_cal_reset(&cal);
    for (c = 0; c < 100; c++)
    {
        for (i = 0; i < 4; i++)
        {
            samples[i][0] = 0;
            samples[i][1] = 0;
            samples[i][2] = 2000;                               /* 0.5 g   */
        }
        imu_cal_add(&cal, samples, 4u);
    }
    CHECK("S4-H04", IMU_CAL_NOT_1G == imu_cal_finish(&cal, 360u, &align, &g_mg),
          "0.5 g at rest rejected (|g| %d mg)", (int)g_mg);
    imu_cal_reset(&cal);
    imu_cal_add(&cal, samples, 4u);
    CHECK("S4-H04", IMU_CAL_TOO_FEW == imu_cal_finish(&cal, 360u, &align, &g_mg),
          "too few samples rejected");
}

static void
test_tilt_heading_rate(void)
{
    model_cfg_t cfg;
    run_t r;
    static double const heads[] = { 0.0, 90.0, 180.0, 270.0, 45.0 };
    unsigned int i;

    /* S4-H05 static tilt */
    model_default(&cfg);
    cfg.speed_cms = 0.0;
    cfg.duration_s = 1.0;
    cfg.pitch_deg = 10.0;
    cfg.roll_deg = -5.0;
    cfg.noise_mg = 2.0;
    cfg.b_motors = false;
    run_model(&cfg, false, &r);
    CHECK("S4-H05", (imu_abs(r.view.pitch_cdeg - 1000) <= 20) &&
          (imu_abs(r.view.roll_cdeg + 500) <= 20),
          "pitch %d, roll %d cdeg (want 1000, -500)", (int)r.view.pitch_cdeg,
          (int)r.view.roll_cdeg);

    /* S4-H06 heading, level and tilted */
    for (i = 0u; i < (sizeof(heads) / sizeof(heads[0])); i++)
    {
        int32_t err;

        cfg.pitch_deg = (i == 4u) ? 10.0 : 0.0;
        cfg.roll_deg = (i == 4u) ? 8.0 : 0.0;
        cfg.heading_deg = heads[i];
        cfg.mag_noise_mg = 5.0;             /* Table 3: 5 mgauss RMS      */
        run_model(&cfg, false, &r);
        err = imu_abs(imu_wrap_cdeg(r.view.heading_cdeg -
                                    (int32_t)lround(heads[i] * 100.0)));
        CHECK("S4-H06", r.view.b_heading && (err <= 150),
              "heading %d cdeg for %d deg%s (error %d)",
              (int)r.view.heading_cdeg, (int)heads[i],
              (i == 4u) ? " pitched 10, rolled 8" : "", (int)err);
    }

    /* S4-H07 turn rate and Turning class, anticlockwise 90 deg/s */
    model_default(&cfg);
    cfg.speed_cms = 0.0;
    cfg.turn_dps = -90.0;
    cfg.duration_s = 2.0;
    cfg.noise_mg = 2.0;
    cfg.mag_noise_mg = 5.0;
    run_model(&cfg, false, &r);
    CHECK("S4-H07", (imu_abs(r.view.rate_cdps + 9000) <= 900) &&
          (IMU_CLASS_TURNING == r.view.cls),
          "turn rate %d cdeg/s (want -9000), class %u", (int)r.view.rate_cdps,
          (unsigned int)r.view.cls);

    /* S4-H07 still: no turn, Stationary */
    cfg.turn_dps = 0.0;
    run_model(&cfg, false, &r);
    CHECK("S4-H07", (imu_abs(r.view.rate_cdps) < 300) &&
          (IMU_CLASS_STATIONARY == r.view.cls) && (0 == r.n_class),
          "at rest: rate %d cdeg/s, class %u, %d class events",
          (int)r.view.rate_cdps, (unsigned int)r.view.cls, r.n_class);
}

static void
test_humps(void)
{
    static double const wheelbases[] = { 8.0, 8.45 };
    static double const speeds[] = { 10.0, 20.0, 30.0 };
    static double const noises[] = { 5.0, 10.0 };
    model_cfg_t cfg;
    run_t r;
    unsigned int big;
    unsigned int w;
    unsigned int s;
    unsigned int n;

    /* S4-H08 NFR15, noise-free model */
    for (big = 0u; big < 2u; big++)
    {
        for (w = 0u; w < 2u; w++)
        {
            for (s = 0u; s < 3u; s++)
            {
                double truth;
                double got;

                hump_cfg(&cfg, 1u == big, wheelbases[w], speeds[s]);
                truth = model_true_peak_mm(&cfg);
                run_model(&cfg, false, &r);
                got = (r.humps > 0) ? (r.hump[0].peak_um / 1000.0) : -1.0;
                CHECK("S4-H08", (1 == r.humps) && (0 == r.dropped) &&
                      nfr15_ok(got, truth),
                      "%s hump, L %.2f, %2.0f cm/s: %.1f mm (true %.1f), "
                      "%d hump(s), pitch %.1f/%.1f deg",
                      big ? "big  " : "small", wheelbases[w], speeds[s], got,
                      truth, r.humps, r.max_pitch_deg, r.min_pitch_deg);
                INFO("    kick: largest spike near the hump while FLAT %u mg"
                     " (collision threshold %d mg), collisions %d",
                     (unsigned int)r.pk_hump_region, IMU_COLLISION_MG,
                     r.collisions);
            }
        }
    }

    /* S4-H09 NFR15 with sensor noise, mounting tilt and calibration
       error.  5 mg per 400 Hz sample is the datasheet noise (1.8 mg) plus
       margin for motor vibration; the 10 mg row shows where it breaks. */
    for (n = 0u; n < 2u; n++)
    {
        for (big = 0u; big < 2u; big++)
        {
            for (w = 0u; w < 2u; w++)
            {
                for (s = 0u; s < 3u; s++)
                {
                    int seed;
                    double worst = 0.0;
                    int fails = 0;
                    int missed = 0;

                    for (seed = 1; seed <= 6; seed++)
                    {
                        double truth;
                        double got;

                        hump_cfg(&cfg, 1u == big, wheelbases[w], speeds[s]);
                        cfg.noise_mg = noises[n];
                        cfg.mount_pitch_deg = (seed % 2) ? 2.0 : -2.0;
                        cfg.mount_roll_deg = (seed % 3) ? 1.0 : -1.5;
                        cfg.off_mg[0] = (seed % 2) ? 3.0 : -3.0;
                        cfg.off_mg[2] = (seed % 3) ? 3.0 : -3.0;
                        cfg.seed = (unsigned int)(1000 + seed);
                        truth = model_true_peak_mm(&cfg);
                        run_model(&cfg, false, &r);
                        if (1 != r.humps)
                        {
                            missed++;
                            continue;
                        }
                        got = r.hump[0].peak_um / 1000.0;
                        fails += nfr15_ok(got, truth) ? 0 : 1;
                        worst = (fabs(got - truth) > worst) ?
                                fabs(got - truth) : worst;
                    }
                    /* NFR15 for every measured hump; NFR16 allows 1 in 10
                       to be missed (5 of 6 here) */
                    if (0u == n)
                    {
                        CHECK("S4-H09", (0 == fails) && (missed <= 1),
                              "%s hump, L %.2f, %2.0f cm/s, %2.0f mg noise, "
                              "2 deg mount, 3 mg zero-g left: worst %.1f mm, "
                              "%d outside NFR15, %d of 6 missed",
                              big ? "big  " : "small", wheelbases[w],
                              speeds[s], noises[n], worst, fails, missed);
                    }
                    else
                    {
                        INFO("S4-H09 %s hump, L %.2f, %2.0f cm/s, %2.0f mg "
                             "noise: worst %.1f mm, %d outside NFR15, %d of "
                             "6 missed", big ? "big  " : "small",
                             wheelbases[w], speeds[s], noises[n], worst,
                             fails, missed);
                    }
                }
            }
        }
    }

    /* S4-H10 detection rate (NFR16: at least 9 of 10 per hump) */
    for (big = 0u; big < 2u; big++)
    {
        int seed;
        int found = 0;

        for (seed = 1; seed <= 10; seed++)
        {
            hump_cfg(&cfg, 1u == big, 8.0, 10.0 + (2.0 * seed));
            cfg.noise_mg = 5.0;
            cfg.seed = (unsigned int)(2000 + seed);
            run_model(&cfg, false, &r);
            found += (1 == r.humps) ? 1 : 0;
        }
        CHECK("S4-H10", found >= 9, "%s hump detected %d of 10 (12-30 cm/s)",
              big ? "big" : "small", found);
    }
}

/* S4-H18 how much uncorrected zero-g offset the at-rest calibration
   tolerates: slowest speed (worst case), board tilted 2 deg, 5 mg noise */
static void
test_offset_sensitivity(void)
{
    static double const offs[] = { 0.0, 5.0, 10.0, 20.0, 30.0 };
    model_cfg_t cfg;
    run_t r;
    unsigned int big;
    unsigned int i;

    for (big = 0u; big < 2u; big++)
    {
        for (i = 0u; i < (sizeof(offs) / sizeof(offs[0])); i++)
        {
            double truth;
            double got;

            hump_cfg(&cfg, 1u == big, 8.0, 10.0);
            cfg.noise_mg = 5.0;
            cfg.mount_pitch_deg = 2.0;
            cfg.off_mg[0] = offs[i];
            cfg.seed = (unsigned int)(3000 + i);
            truth = model_true_peak_mm(&cfg);
            run_model(&cfg, false, &r);
            got = (1 == r.humps) ? (r.hump[0].peak_um / 1000.0) : -99.0;
            if (offs[i] <= 10.0)
            {
                CHECK("S4-H18", nfr15_ok(got, truth),
                      "%s hump 10 cm/s, %2.0f mg zero-g x error left in: "
                      "%.1f mm (true %.1f)", big ? "big  " : "small",
                      offs[i], got, truth);
            }
            else
            {
                INFO("S4-H18 %s hump 10 cm/s, %2.0f mg zero-g x error left "
                     "in: %.1f mm (true %.1f) -> %s: the Stage 2 offset fit "
                     "(IMU_ACC_ZG_*) must bring the error under 10 mg",
                     big ? "big  " : "small", offs[i], got, truth,
                     nfr15_ok(got, truth) ? "inside NFR15" : "OUTSIDE NFR15");
            }
        }
    }
}

/* S4-H20 the second board's mounting: gravity 15 deg nose-down and
   6 deg rolled from the sensor's Z at rest.  The at-rest rotation must
   take it out completely.  S4-H21 (INFO): what the second board's
   per-axis gains (fitted in S4-M01b, imu_params.h) would cost if
   IMU_ACC_SCALE_* were left at 1000. */
static void
test_mounting(void)
{
    static double const speeds[] = { 10.0, 20.0, 30.0 };
    static double const g_gain[3] =     /* sensor axes X, Y, Z */
    {
        IMU_ACC_SCALE_X_PERMIL / 1000.0, IMU_ACC_SCALE_Y_PERMIL / 1000.0,
        IMU_ACC_SCALE_Z_PERMIL / 1000.0
    };
    model_cfg_t cfg;
    run_t r;
    unsigned int big;
    unsigned int sp;

    for (big = 0u; big < 2u; big++)
    {
        for (sp = 0u; sp < 3u; sp++)
        {
            double truth;
            double got;

            hump_cfg(&cfg, 1u == big, 8.0, speeds[sp]);
            cfg.noise_mg = 5.0;
            cfg.mount_pitch_deg = -15.0;
            cfg.mount_roll_deg = 6.0;
            cfg.off_mg[0] = 3.0;                /* zero-g left after fit */
            cfg.off_mg[2] = -3.0;
            cfg.seed = (unsigned int)(5000 + (10u * big) + sp);
            truth = model_true_peak_mm(&cfg);
            run_model(&cfg, true, &r);          /* level check on (H19)  */
            got = (1 == r.humps) ? (r.hump[0].peak_um / 1000.0) : -99.0;
            CHECK("S4-H20", (IMU_CAL_OK == r.cal_result) &&
                  nfr15_ok(got, truth) && (0 == r.collisions),
                  "%s hump %2.0f cm/s, board 15 deg nose-down 6 deg rolled: "
                  "%.1f mm (true %.1f), level after calibration %+.1f deg",
                  big ? "big  " : "small", speeds[sp], got, truth,
                  r.view.pitch_cdeg / 100.0);

            cfg.gain[0] = g_gain[IMU_ACC_BX_AXIS];  /* gains left in     */
            cfg.gain[1] = g_gain[IMU_ACC_BY_AXIS];
            cfg.gain[2] = g_gain[IMU_ACC_BZ_AXIS];
            run_model(&cfg, true, &r);
            got = (1 == r.humps) ? (r.hump[0].peak_um / 1000.0) : -99.0;
            INFO("S4-H21 same with the fitted gains (X %.3f Y %.3f Z %.3f) "
                 "uncorrected: %.1f mm (true %.1f) -> %s", g_gain[0],
                 g_gain[1], g_gain[2], got, truth,
                 nfr15_ok(got, truth) ? "inside NFR15" : "OUTSIDE NFR15");
        }
    }
}

/* S4-H19 hump kicks vs collisions for each anti-alias setting (CTRL2
   ABW): the largest horizontal spike near each hump while the logic is
   still FLAT, against a bump into a wall at 20 cm/s that stops the car
   in 5 ms or 20 ms.  Then, at the chosen 50 Hz, false collisions over
   both humps and bump detection, without and with the level check. */
static run_t g_bump;

static double
bump_run(double bw, double crush_s, bool b_level)
{
    model_cfg_t cfg;

    model_default(&cfg);
    cfg.h_cm = 0.0;
    cfg.speed_cms = 20.0;
    cfg.still_s = 1.0;
    cfg.duration_s = 2.5;
    cfg.bw_hz = bw;
    cfg.noise_mg = 5.0;
    cfg.n_phase = 2;
    cfg.phase[0].t_s = 2.0;
    cfg.phase[0].accel_cms2 = -20.0 / crush_s;
    cfg.phase[1].t_s = 2.0 + crush_s;
    cfg.phase[1].accel_cms2 = 0.0;
    run_model(&cfg, b_level, &g_bump);
    return g_bump.pk_flat;
}

static void
test_kick_bandwidth(void)
{
    static double const bws[] = { 50.0, 194.0, 362.0, 773.0 };
    model_cfg_t cfg;
    run_t r;
    unsigned int i;
    unsigned int big;
    unsigned int lv;

    for (i = 0u; i < (sizeof(bws) / sizeof(bws[0])); i++)
    {
        double kick[2][2];
        double hard;
        unsigned int sp;

        for (big = 0u; big < 2u; big++)
        {
            for (sp = 0u; sp < 2u; sp++)
            {
                hump_cfg(&cfg, 1u == big, 8.0, (0u == sp) ? 20.0 : 30.0);
                cfg.bw_hz = bws[i];
                run_model(&cfg, false, &r);
                kick[big][sp] = r.pk_hump_region;
            }
        }
        hard = bump_run(bws[i], 0.005, false);
        INFO("S4-H19 ABW %3.0f Hz: hump kick small %4.0f / %4.0f mg, big "
             "%4.0f / %4.0f mg (20 / 30 cm/s); bump at 20 cm/s %4.0f mg "
             "(5 ms stop), %4.0f mg (20 ms stop)", bws[i], kick[0][0],
             kick[0][1], kick[1][0], kick[1][1], hard,
             bump_run(bws[i], 0.020, false));
    }

    /* At 50 Hz: false collisions on humps, and bumps caught */
    for (lv = 0u; lv < 2u; lv++)
    {
        int false_coll = 0;
        int runs = 0;
        int seed;
        int caught_hard;
        int caught_soft;

        for (big = 0u; big < 2u; big++)
        {
            for (seed = 1; seed <= 4; seed++)
            {
                unsigned int sp;

                for (sp = 0u; sp < 3u; sp++)
                {
                    hump_cfg(&cfg, 1u == big, 8.0, 10.0 + (10.0 * sp));
                    cfg.noise_mg = 5.0 * (double)(1 + (seed % 2));
                    cfg.seed = (unsigned int)(4000 + seed);
                    run_model(&cfg, 1u == lv, &r);
                    false_coll += (r.collisions > 0) ? 1 : 0;
                    runs++;
                }
            }
        }
        (void)bump_run(50.0, 0.005, 1u == lv);
        caught_hard = g_bump.collisions;
        (void)bump_run(50.0, 0.020, 1u == lv);
        caught_soft = g_bump.collisions;
        /* The level check is the agreed fallback for kicks and bumps that
           overlap: it must remove the false collisions and still catch a
           hard bump.  Without it the result is reported, not judged. */
        if (0u == lv)
        {
            INFO("S4-H19 50 Hz, level check off: %d of %d hump runs with a "
                 "false collision; 5 ms bump %s, 20 ms bump %s", false_coll,
                 runs, caught_hard ? "caught" : "missed",
                 caught_soft ? "caught" : "missed (about 1.0 g)");
        }
        else
        {
            CHECK("S4-H19", (0 == false_coll) && (caught_hard > 0),
                  "50 Hz, level check on: %d of %d hump runs with a false "
                  "collision; 5 ms bump %s, 20 ms bump %s", false_coll, runs,
                  caught_hard ? "caught" : "missed",
                  caught_soft ? "caught" : "missed (about 1.0 g)");
        }
    }
}

static void
test_flat_track(void)
{
    model_cfg_t cfg;
    run_t r;

    /* S4-H11 NFR16: 5 m of flat track, starts, stops and speed changes */
    model_default(&cfg);
    cfg.h_cm = 0.0;
    cfg.speed_cms = 0.0;
    cfg.duration_s = 27.0;
    cfg.noise_mg = 10.0;
    cfg.n_phase = 6;
    cfg.phase[0].t_s = 0.5;  cfg.phase[0].accel_cms2 = 100.0;  /* 0.1 g    */
    cfg.phase[1].t_s = 0.7;  cfg.phase[1].accel_cms2 = 0.0;    /* 20 cm/s  */
    cfg.phase[2].t_s = 10.0; cfg.phase[2].accel_cms2 = -100.0; /* stop     */
    cfg.phase[3].t_s = 10.2; cfg.phase[3].accel_cms2 = 0.0;
    cfg.phase[4].t_s = 11.0; cfg.phase[4].accel_cms2 = 150.0;
    cfg.phase[5].t_s = 11.2; cfg.phase[5].accel_cms2 = 0.0;
    run_model(&cfg, false, &r);
    {
        int false_climb = 0;
        int i;

        for (i = 0; i < r.n_class; i++)
        {
            false_climb += ((IMU_CLASS_CLIMBING_HUMP == r.cls[i]) ||
                            (IMU_CLASS_DESCENDING_HUMP == r.cls[i])) ? 1 : 0;
        }
        CHECK("S4-H11", (0 == r.humps) && (0 == r.collisions) &&
              (0 == false_climb) && (0 == r.dropped),
              "flat %.1f m with 0.1-0.15 g starts and stops: %d humps, %d "
              "hump classes, %d aborted climbs, %d collisions, largest "
              "spike %u mg", g_model.x / 100.0, r.humps, false_climb,
              r.dropped, r.collisions, (unsigned int)r.pk_flat);
    }

    /* S4-H12 motion classes along the way */
    {
        bool b_acc = false;
        bool b_straight = false;
        int i;

        for (i = 0; i < r.n_class; i++)
        {
            b_acc = b_acc || (IMU_CLASS_ACCELERATING == r.cls[i]);
            b_straight = b_straight || (IMU_CLASS_MOVING_STRAIGHT == r.cls[i]);
        }
        CHECK("S4-H12", b_acc && b_straight && (r.n_class <= 12),
              "accelerating and moving straight reported, %d class events",
              r.n_class);
    }
}

static void
test_class_sequence(void)
{
    model_cfg_t cfg;
    run_t r;
    bool b_climb = false;
    bool b_desc = false;
    int i;
    uint32_t climb_at = 0u;

    /* S4-H13 hump classes in order, each after its 100 ms hold */
    hump_cfg(&cfg, true, 8.0, 20.0);
    cfg.noise_mg = 10.0;
    run_model(&cfg, false, &r);
    for (i = 0; i < r.n_class; i++)
    {
        if (IMU_CLASS_CLIMBING_HUMP == r.cls[i])
        {
            b_climb = true;
            climb_at = r.cls_cycle[i];
        }
        if ((IMU_CLASS_DESCENDING_HUMP == r.cls[i]) && b_climb &&
            (r.cls_cycle[i] > climb_at))
        {
            b_desc = true;
        }
    }
    {
        char seq[MAX_EVT * 2 + 1];
        int k;

        for (k = 0; (k < r.n_class) && (k < MAX_EVT); k++)
        {
            seq[2 * k] = (char)('0' + r.cls[k]);
            seq[(2 * k) + 1] = ' ';
        }
        seq[2 * k] = '\0';
        CHECK("S4-H13", b_climb && b_desc &&
              (IMU_CLASS_MOVING_STRAIGHT == r.view.cls),
              "climbing then descending, then moving straight (classes: %s)",
              seq);
    }
}

static void
test_collision_tip(void)
{
    imu_in_t in;
    imu_out_t out;
    int c;
    int coll = 0;
    int impact_at = -1;
    int tips = 0;

    /* S4-H14 collision: spike outside a hump, motors on */
    imu_logic_init(&g_ctx, false);
    for (c = 0; c < 200; c++)
    {
        fill_static(&in, 0.0, 0.0, true);
        if ((50 == c) || (70 == c) || (150 == c))
        {
            in.acc[1][0] = (int16_t)(-2000 * IMU_Q2_PER_MG);    /* 2 g     */
        }
        imu_logic_step(&g_ctx, &in, &out);
        if (out.b_collision)
        {
            coll++;
        }
        if (out.b_class && (IMU_CLASS_SUDDEN_IMPACT == out.cls) &&
            (impact_at < 0))
        {
            impact_at = c;
        }
    }
    CHECK("S4-H14", (2 == coll) && (50 == impact_at),
          "spikes at 0.5, 0.7, 1.5 s: %d collisions (want 2: the 0.7 s one "
          "is inside the 500 ms ignore window), Sudden impact at cycle %d",
          coll, impact_at);

    imu_logic_init(&g_ctx, false);
    coll = 0;
    for (c = 0; c < 100; c++)
    {
        fill_static(&in, 0.0, 0.0, false);
        if (50 == c)
        {
            in.acc[1][0] = (int16_t)(-2000 * IMU_Q2_PER_MG);
        }
        imu_logic_step(&g_ctx, &in, &out);
        coll += (int)out.b_collision;
    }
    CHECK("S4-H14", 0 == coll, "no collision with the motors off");

    /* S4-H15 level-check fallback: a bump keeps pitch, a hump kick not */
    imu_logic_init(&g_ctx, true);
    coll = 0;
    for (c = 0; c < 100; c++)
    {
        fill_static(&in, 0.0, 0.0, true);
        if (50 == c)
        {
            in.acc[1][0] = (int16_t)(-2000 * IMU_Q2_PER_MG);
        }
        imu_logic_step(&g_ctx, &in, &out);
        coll += (int)out.b_collision;
    }
    CHECK("S4-H15", 1 == coll, "level check: spike then level -> collision");

    imu_logic_init(&g_ctx, true);
    coll = 0;
    for (c = 0; c < 100; c++)
    {
        double const pitch = (c > 50) ? (0.3 * (c - 50)) : 0.0;  /* 30 deg/s */

        fill_static(&in, pitch, 0.0, true);
        if (50 == c)
        {
            in.acc[1][0] = (int16_t)(-2000 * IMU_Q2_PER_MG);
        }
        imu_logic_step(&g_ctx, &in, &out);
        coll += (int)out.b_collision;
    }
    CHECK("S4-H15", 0 == coll,
          "level check: spike then pitch rising 30 deg/s -> no collision");

    /* S4-H16 tip-over: 65 deg for 0.8 s -> one event; 0.4 s -> none */
    imu_logic_init(&g_ctx, false);
    for (c = 0; c < 150; c++)
    {
        fill_static(&in, ((c >= 20) && (c < 100)) ? 65.0 : 0.0, 0.0, true);
        imu_logic_step(&g_ctx, &in, &out);
        tips += (int)out.b_tipover;
    }
    CHECK("S4-H16", 1 == tips, "65 deg for 0.8 s: %d tip-over events", tips);
    imu_logic_init(&g_ctx, false);
    tips = 0;
    for (c = 0; c < 150; c++)
    {
        fill_static(&in, 0.0, ((c >= 20) && (c < 60)) ? -70.0 : 0.0, true);
        imu_logic_step(&g_ctx, &in, &out);
        tips += (int)out.b_tipover;
    }
    CHECK("S4-H16", 0 == tips, "roll 70 deg for 0.4 s: %d tip-over events",
          tips);
}

static void
test_run_peak(void)
{
    model_cfg_t cfg;
    run_t r;
    imu_in_t in;
    imu_out_t out;
    int c;
    uint8_t last;

    /* S4-H17 highest peak resets when a run starts (motors on edge) */
    hump_cfg(&cfg, false, 8.0, 20.0);
    run_model(&cfg, false, &r);
    last = g_ctx.view.last_peak_mm;
    for (c = 0; c < 20; c++)
    {
        fill_static(&in, 0.0, 0.0, c >= 10);
        imu_logic_step(&g_ctx, &in, &out);
    }
    CHECK("S4-H17", (r.view.max_peak_mm > 0u) && (0u == g_ctx.view.max_peak_mm) &&
          (g_ctx.view.last_peak_mm == last),
          "max peak %u -> %u at run start, last peak kept (%u)",
          (unsigned int)r.view.max_peak_mm,
          (unsigned int)g_ctx.view.max_peak_mm,
          (unsigned int)g_ctx.view.last_peak_mm);
}

void
test_logic(void)
{
    test_mapping_and_cal();
    test_tilt_heading_rate();
    test_humps();
    test_offset_sensitivity();
    test_mounting();
    test_kick_bandwidth();
    test_flat_track();
    test_class_sequence();
    test_collision_tip();
    test_run_peak();
}
