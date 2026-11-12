/*
 *  hump_model.c
 *  Host-only car and sensor model.  See hump_model.h.
 */

#include <math.h>
#include <string.h>
#include "hump_model.h"

#define WHEEL_R_CM      3.25
#define CASTOR_R_CM     0.75
#define IMU_BACK_CM     1.75
#define IMU_UP_CM       9.2
#define G_CMS2          980.665
#define DT_S            0.00025         /* 0.25 ms model step             */
#define STEPS_PER_SAMPLE 10             /* 2.5 ms = 400 Hz                */
#define SAMPLES_PER_CYCLE 4
#define LSB_MG          0.122           /* 16-bit at +-4 g (Table 3)      */
#define RANGE_MG        4000.0
#define B_H_MG          400.0           /* horizontal field, roughly SG   */
#define B_Z_MG          (-120.0)        /* vertical: inclination ~ -17 deg */
#define MAG_LSB_MG      0.080           /* +-2 gauss (Table 3)            */
#define DEG             (3.14159265358979323846 / 180.0)
#define FAR_CM          1.0e4
#define N_PTS           6

/* ---- Surface: polyline foot - crest - (flat top) - foot ----------- */

static void
hump_points(model_cfg_t const *p_c, double *p_x, double *p_z)
{
    double const ramp = (p_c->length_cm - p_c->top_cm) / 2.0;
    double const x0 = p_c->start_cm;

    p_x[0] = -FAR_CM;               p_z[0] = 0.0;
    p_x[1] = x0;                    p_z[1] = 0.0;
    p_x[2] = x0 + ramp;             p_z[2] = p_c->h_cm;
    p_x[3] = x0 + ramp + p_c->top_cm; p_z[3] = p_c->h_cm;
    p_x[4] = x0 + p_c->length_cm;   p_z[4] = 0.0;
    p_x[5] = FAR_CM;                p_z[5] = 0.0;
}

/* Height of the centre of a circle of radius r resting on the surface,
   exact for a polyline (segments and corners). */
static double
centre_z(model_cfg_t const *p_c, double xc, double r)
{
    double px[N_PTS];
    double pz[N_PTS];
    double best = r;
    int i;

    if (p_c->h_cm <= 0.0)
    {
        return r;
    }
    hump_points(p_c, px, pz);
    for (i = 0; i < (N_PTS - 1); i++)
    {
        double const m = (pz[i + 1] - pz[i]) / (px[i + 1] - px[i]);
        double const s = sqrt(1.0 + (m * m));
        double const xt = xc + ((r * m) / s);   /* tangent point        */

        if ((xt >= px[i]) && (xt <= px[i + 1]))
        {
            double const z = pz[i] + (m * (xc - px[i])) + (r * s);

            best = (z > best) ? z : best;
        }
    }
    for (i = 1; i < (N_PTS - 1); i++)
    {
        double const dx = xc - px[i];

        if (fabs(dx) <= r)
        {
            double const z = pz[i] + sqrt((r * r) - (dx * dx));

            best = (z > best) ? z : best;
        }
    }
    return best;
}

/* Chassis pose for rear axle at x: pitch (rad), axle height, IMU (x, z) */
static void
pose(model_t *p_m, double x, double *p_pitch, double *p_zh, double *p_imu)
{
    model_cfg_t const *p_c = &p_m->cfg;
    double const big_d = hypot(p_c->wheelbase_cm, WHEEL_R_CM - CASTOR_R_CM);
    double const lvl = atan2(CASTOR_R_CM - WHEEL_R_CM, p_c->wheelbase_cm);
    double const zh = centre_z(p_c, x, WHEEL_R_CM);
    double xc = (p_m->castor_x > x) ? p_m->castor_x : (x + p_c->wheelbase_cm);
    double zc = centre_z(p_c, xc, CASTOR_R_CM);
    double th;
    int it;

    /* Secant on |hub - castor| = D */
    for (it = 0; it < 60; it++)
    {
        double const f = hypot(xc - x, zc - zh) - big_d;
        double const h = 1.0e-7;
        double const f2 = hypot((xc + h) - x,
                                centre_z(p_c, xc + h, CASTOR_R_CM) - zh) -
                          big_d;
        double const df = (f2 - f) / h;

        if (fabs(f) < 1.0e-12)
        {
            break;
        }
        xc -= f / ((fabs(df) > 1.0e-9) ? df : 1.0);
        zc = centre_z(p_c, xc, CASTOR_R_CM);
    }
    p_m->castor_x = xc;
    th = atan2(zc - zh, xc - x) - lvl;
    *p_pitch = th;
    *p_zh = zh;
    p_imu[0] = x - (IMU_BACK_CM * cos(th)) - (IMU_UP_CM * sin(th));
    p_imu[1] = zh - (IMU_BACK_CM * sin(th)) + (IMU_UP_CM * cos(th));
}

/* ---- Noise -------------------------------------------------------- */

static double
uniform(model_t *p_m)
{
    p_m->rng ^= p_m->rng << 13;
    p_m->rng ^= p_m->rng >> 17;
    p_m->rng ^= p_m->rng << 5;
    return ((double)p_m->rng + 1.0) / 4294967297.0;
}

static double
gauss(model_t *p_m)
{
    double const u1 = uniform(p_m);
    double const u2 = uniform(p_m);

    return sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979323846 * u2);
}

/* Rotate a world vector (x fwd-north, y left, z up) into the body:
   yaw is handled by the caller; here pitch th (nose up +), roll ph
   (left side up +). */
static void
to_body(double const *p_w, double th, double ph, double *p_b)
{
    double const x1 = (p_w[0] * cos(th)) + (p_w[2] * sin(th));
    double const z1 = (-p_w[0] * sin(th)) + (p_w[2] * cos(th));
    double const y1 = p_w[1];

    p_b[0] = x1;
    p_b[1] = (y1 * cos(ph)) + (z1 * sin(ph));
    p_b[2] = (-y1 * sin(ph)) + (z1 * cos(ph));
}

static int16_t
quant_acc(double mg)
{
    double q = floor((mg / LSB_MG) + 0.5) * LSB_MG;

    q = (q > RANGE_MG) ? RANGE_MG : ((q < -RANGE_MG) ? -RANGE_MG : q);
    return (int16_t)lround(q * IMU_Q2_PER_MG);
}

/* ---- API ---------------------------------------------------------- */

void
model_default(model_cfg_t *p_cfg)
{
    (void)memset(p_cfg, 0, sizeof(*p_cfg));
    p_cfg->wheelbase_cm = 8.0;
    p_cfg->speed_cms = 20.0;
    p_cfg->start_cm = 30.0;
    p_cfg->duration_s = 4.0;
    p_cfg->noise_mg = 0.0;
    p_cfg->b_motors = true;
    p_cfg->seed = 12345u;
    p_cfg->start_accel_cms2 = 100.0;
    p_cfg->bw_hz = 50.0;
    p_cfg->gain[0] = 1.0;
    p_cfg->gain[1] = 1.0;
    p_cfg->gain[2] = 1.0;
}

void
model_start(model_t *p_m, model_cfg_t const *p_cfg)
{
    double th;
    double zh;
    double p[2];

    (void)memset(p_m, 0, sizeof(*p_m));
    p_m->cfg = *p_cfg;
    p_m->v = (p_cfg->still_s > 0.0) ? 0.0 : p_cfg->speed_cms;
    p_m->rng = (0u != p_cfg->seed) ? p_cfg->seed : 1u;
    pose(p_m, -(p_m->v * DT_S), &th, &zh, p);
    p_m->p_prev[0] = p[0];
    p_m->p_prev[1] = p[1];
    pose(p_m, 0.0, &th, &zh, p);
    p_m->p_cur[0] = p[0];
    p_m->p_cur[1] = p[1];
}

bool
model_cycle(model_t *p_m, imu_in_t *p_in)
{
    model_cfg_t const *p_c = &p_m->cfg;
    double const alpha = DT_S / (DT_S + (1.0 / (2.0 * 3.14159265358979323846 *
                                                p_c->bw_hz)));
    int step;
    int k;
    int n = 0;

    if (p_m->t >= p_c->duration_s)
    {
        return false;
    }
    (void)memset(p_in, 0, sizeof(*p_in));

    for (step = 1; step <= (SAMPLES_PER_CYCLE * STEPS_PER_SAMPLE); step++)
    {
        double acc = 0.0;
        double th;
        double zh;
        double p_next[2];
        double a_w[3];
        double f_b[3];
        double ph = p_c->roll_deg * DEG;
        double x_next;
        double p_tmp[2];
        int i;

        for (i = 0; i < p_c->n_phase; i++)
        {
            if (p_m->t >= p_c->phase[i].t_s)
            {
                acc = p_c->phase[i].accel_cms2;
            }
        }
        if (p_c->still_s > 0.0)
        {
            /* At rest for the self-test, then ramp up to speed */
            if (p_m->t < p_c->still_s)
            {
                p_m->v = 0.0;
            }
            else if (!p_m->b_at_speed)
            {
                acc = p_c->start_accel_cms2;
                if ((p_m->v + (acc * DT_S)) >= p_c->speed_cms)
                {
                    acc = (p_c->speed_cms - p_m->v) / DT_S;
                    p_m->b_at_speed = true;     /* phases rule from here */
                }
            }
            else
            {
                /* at speed */
            }
        }

        /* Centred second difference of the IMU path at x */
        p_m->v += acc * DT_S;
        if (p_m->v < 0.0)
        {
            p_m->v = 0.0;
        }
        x_next = p_m->x + (p_m->v * DT_S);
        pose(p_m, x_next, &th, &zh, p_next);
        a_w[0] = (p_next[0] - (2.0 * p_m->p_cur[0]) + p_m->p_prev[0]) /
                 (DT_S * DT_S);
        a_w[1] = 0.0;
        a_w[2] = ((p_next[1] - (2.0 * p_m->p_cur[1]) + p_m->p_prev[1]) /
                  (DT_S * DT_S)) + G_CMS2;

        pose(p_m, p_m->x, &th, &zh, p_tmp);
        th += p_c->pitch_deg * DEG;
        p_m->p_prev[0] = p_m->p_cur[0];
        p_m->p_prev[1] = p_m->p_cur[1];
        p_m->p_cur[0] = p_next[0];
        p_m->p_cur[1] = p_next[1];
        p_m->x = x_next;
        p_m->t += DT_S;
        p_m->pitch_true_deg = th / DEG;
        p_m->axle_z_cm = zh - WHEEL_R_CM;
        to_body(a_w, th, ph, f_b);
        if ((0.0 != p_c->mount_pitch_deg) || (0.0 != p_c->mount_roll_deg))
        {
            double const f_car[3] = { f_b[0], f_b[1], f_b[2] };

            to_body(f_car, p_c->mount_pitch_deg * DEG,
                    p_c->mount_roll_deg * DEG, f_b);
        }
        for (k = 0; k < 3; k++)
        {
            f_b[k] *= p_c->gain[k];     /* sensor-axis gain error        */
        }

        for (k = 0; k < 3; k++)
        {
            double const mg = (f_b[k] / G_CMS2) * 1000.0;

            if (!p_m->b_started)
            {
                p_m->lpf[k] = mg;
            }
            p_m->lpf[k] += alpha * (mg - p_m->lpf[k]);
        }
        p_m->b_started = true;

        if (0 == (step % STEPS_PER_SAMPLE))
        {
            for (k = 0; k < 3; k++)
            {
                p_in->acc[n][k] = quant_acc(p_m->lpf[k] + p_c->off_mg[k] +
                                            (p_c->noise_mg * gauss(p_m)));
            }
            n++;
        }
    }
    p_in->n_acc = (uint8_t)n;
    p_in->b_motors = p_c->b_motors;

    /* Magnetometer every second cycle (50 Hz) */
    if (0u == (p_m->cycle % 2u))
    {
        double const yaw = (p_c->heading_deg + (p_c->turn_dps * p_m->t)) * DEG;
        double const b_lvl[3] =
        {
            B_H_MG * cos(yaw), B_H_MG * sin(yaw), B_Z_MG
        };
        double b_b[3];

        to_body(b_lvl, p_m->pitch_true_deg * DEG, p_c->roll_deg * DEG, b_b);
        if ((0.0 != p_c->mount_pitch_deg) || (0.0 != p_c->mount_roll_deg))
        {
            double const b_car[3] = { b_b[0], b_b[1], b_b[2] };

            to_body(b_car, p_c->mount_pitch_deg * DEG,
                    p_c->mount_roll_deg * DEG, b_b);
        }
        for (k = 0; k < 3; k++)
        {
            double const mg = b_b[k] + (p_c->mag_noise_mg * gauss(p_m));

            p_in->mag[k] = (int16_t)lround(floor((mg / MAG_LSB_MG) + 0.5) *
                                           MAG_LSB_MG);
        }
        p_in->b_mag = true;
    }
    p_m->cycle++;
    return true;
}

double
model_true_peak_mm(model_cfg_t const *p_cfg)
{
    double peak = 0.0;
    double x;

    for (x = p_cfg->start_cm - 5.0; x < (p_cfg->start_cm + p_cfg->length_cm +
                                         5.0); x += 0.001)
    {
        double const z = centre_z(p_cfg, x, WHEEL_R_CM) - WHEEL_R_CM;

        peak = (z > peak) ? z : peak;
    }
    return peak * 10.0;
}
