/*
 *  hump_model.h
 *  Host-only model of the robot car for the Subsystem 4 unit tests.
 *
 *  A rigid car (rear wheels of radius 3.25 cm, ball castor 0.75 cm,
 *  measured wheelbase) drives over a triangular hump.  The wheel and
 *  castor centres rest exactly on the hump's surface (polyline), the
 *  chassis pitch follows from the two contacts, and the IMU sits
 *  1.75 cm behind and 9.2 cm above the rear axle.  The IMU's specific
 *  force is differentiated from its path every 0.25 ms, passed through
 *  the LSM303D's anti-alias filter (bw_hz, set by CTRL2 ABW; 50 Hz in
 *  our set-up, modelled as one pole since the datasheet gives no order),
 *  sampled at 400 Hz, given noise, quantised to 0.122 mg and clipped at
 *  +-4 g, as the real sensor would deliver it (body frame, after
 *  calibration).
 *
 *  Same geometry as the Python model used for the design notes
 *  (hump_geom.py), so the numbers can be compared.
 */

#ifndef HUMP_MODEL_H
#define HUMP_MODEL_H

#include <stdbool.h>
#include "imu_logic.h"

#define MODEL_MAX_PHASES    8

/* Speed profile phase: from t_s on, accelerate at accel_cms2 */
typedef struct
{
    double t_s;
    double accel_cms2;
} model_phase_t;

typedef struct
{
    /* Hump (all cm); h_cm = 0 means flat track */
    double h_cm;
    double length_cm;           /* whole hump, foot to foot              */
    double top_cm;              /* flat strip on the crest               */
    double start_cm;            /* where the hump begins                 */

    /* Car */
    double wheelbase_cm;
    double speed_cms;           /* initial speed                         */
    model_phase_t phase[MODEL_MAX_PHASES];
    int    n_phase;
    double duration_s;
    double turn_dps;            /* in-place turn rate, clockwise +       */
    double heading_deg;         /* initial heading                       */
    double pitch_deg;           /* static tilt (hand-held tests)         */
    double roll_deg;
    double still_s;             /* at rest first (self-test), then ramp  */
    double start_accel_cms2;    /* ... up to speed_cms at this rate      */
    double mount_pitch_deg;     /* GY-511 tilt on its bracket            */
    double mount_roll_deg;

    /* Sensor */
    double bw_hz;               /* anti-alias filter (CTRL2 ABW)         */
    double noise_mg;            /* white noise per 400 Hz sample         */
    double off_mg[3];           /* residual offset after calibration     */
    double gain[3];             /* per-axis gain error (1 = none)        */
    double mag_noise_mg;
    bool   b_motors;
    unsigned int seed;
} model_cfg_t;

typedef struct
{
    model_cfg_t cfg;
    double t;
    double x;                   /* rear axle position, cm                */
    double v;                   /* cm/s                                  */
    double castor_x;            /* last solution, cm                     */
    double p_prev[2];           /* IMU path, cm                          */
    double p_cur[2];
    double lpf[3];              /* sensor low-pass state, mg             */
    bool   b_started;
    bool   b_at_speed;              /* start ramp finished               */
    unsigned int rng;
    unsigned long cycle;
    double pitch_true_deg;      /* for checks                            */
    double axle_z_cm;           /* rear-axle height above the floor      */
} model_t;

void    model_default(model_cfg_t *p_cfg);
void    model_start(model_t *p_m, model_cfg_t const *p_cfg);

/* Fill one 10 ms cycle of input.  false once the run is over. */
bool    model_cycle(model_t *p_m, imu_in_t *p_in);

/* Peak rear-axle height the model itself reaches (truth), mm */
double  model_true_peak_mm(model_cfg_t const *p_cfg);

#endif /* HUMP_MODEL_H */
