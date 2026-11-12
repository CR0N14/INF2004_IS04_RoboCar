/*
 *  imu_logic.h
 *  IMU processing as a pure function: (context, one 10 ms cycle of
 *  samples) -> (context, outputs).  No kernel calls, so it runs and is
 *  unit-tested on a PC (testing/subsystem4/host/).  The IMU task
 *  (imu_task.c) feeds it samples from the driver and carries out the
 *  outputs (events, snapshot).
 *
 *  What it computes (design review section 6.4):
 *    tilt          pitch and roll from a low-passed gravity vector
 *    heading       tilt-compensated, from the magnetometer
 *    turn rate     heading change over IMU_TURN_WINDOW_MS (no gyro)
 *    motion class  Stationary / Moving straight / Accelerating / Turning
 *                  / Climbing hump / Descending hump / Sudden impact,
 *                  100 ms hold, Sudden impact at once (FR24, NFR18)
 *    humps         FLAT -> CLIMBING -> DESCENDING -> FLAT; height by
 *                  double integration of the vertical acceleration from
 *                  the last flat moment (pre-trigger history), drift
 *                  removed at both ends, then corrected from the IMU to
 *                  the rear axle (lever arm) (FR22, FR23)
 *    collision     horizontal spike outside a hump, motors on (FR25)
 *    tip-over      pitch or roll beyond the limit for 0.5 s (FR26)
 *
 *  Units: acceleration samples in 0.25 mg ("q2"), magnetic field in
 *  milligauss, angles in centidegrees, heights in micrometres.
 *  Body frame: x forward, y left, z up.
 *
 *  Private to subsystem 4.
 */

#ifndef IMU_LOGIC_H
#define IMU_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

#define IMU_Q2_PER_MG       4           /* 0.25 mg units                 */
#define IMU_Q2_1G           4000        /* 1 g after calibration         */
#define IMU_FIFO_MAX        12u         /* samples per cycle accepted    */
#define IMU_HIST_LEN        1024u       /* hump history, cycles (10.24 s) */
#define IMU_HEAD_HIST_LEN   32u         /* magnetometer headings kept    */
#define IMU_PEAK_MAX_MM     127u        /* 7-bit telemetry field         */

/* Same numbering as s4_motion_class_t (subsystem4.h); imu_task.c checks
   this at compile time. */
typedef enum
{
    IMU_CLASS_STATIONARY = 0,
    IMU_CLASS_MOVING_STRAIGHT,
    IMU_CLASS_ACCELERATING,
    IMU_CLASS_TURNING,
    IMU_CLASS_CLIMBING_HUMP,
    IMU_CLASS_DESCENDING_HUMP,
    IMU_CLASS_SUDDEN_IMPACT
} imu_class_t;

typedef enum
{
    IMU_HUMP_FLAT = 0,
    IMU_HUMP_CLIMBING,
    IMU_HUMP_DESCENDING
} imu_hump_state_t;

/* ---- Inputs ------------------------------------------------------- */

/* One 10 ms cycle */
typedef struct
{
    int16_t acc[IMU_FIFO_MAX][3];   /* body q2, calibration removed      */
    uint8_t n_acc;                  /* samples this cycle (about 4)      */
    bool    b_mag;                  /* a magnetometer sample this cycle  */
    int16_t mag[3];                 /* body mG, calibration removed      */
    bool    b_motors;               /* SM_FLG_MOTORS_ENABLED             */
} imu_in_t;

/* ---- Outputs ------------------------------------------------------ */

/* One measured hump */
typedef struct
{
    uint8_t  peak_mm;           /* reported: rounded, 0..127             */
    int32_t  peak_um;           /* before rounding                       */
    uint16_t duration_ms;       /* flat reference to end                 */
    int16_t  max_pitch_cdeg;
    int16_t  min_pitch_cdeg;
    uint16_t max_spike_mg;      /* largest horizontal spike on the hump  */
} imu_hump_t;

/* Produced by one step */
typedef struct
{
    unsigned int b_hump         : 1;    /* hump measured: see hump       */
    unsigned int b_class        : 1;    /* reported class changed        */
    unsigned int cls            : 3;    /* imu_class_t                   */
    unsigned int b_collision    : 1;
    unsigned int b_tipover      : 1;
    unsigned int b_hump_dropped : 1;    /* climb or descent timed out    */
    uint16_t     collision_mg;          /* the spike that caused it      */
    imu_hump_t   hump;
} imu_out_t;

/* Latest values, for the snapshot and the console */
typedef struct
{
    int32_t  pitch_cdeg;        /* nose up +                             */
    int32_t  roll_cdeg;         /* left side up + (leaning right)        */
    int32_t  heading_cdeg;      /* 0..35999 clockwise from magnetic N    */
    int32_t  rate_cdps;         /* clockwise (right turn) +              */
    bool     b_heading;         /* heading valid                         */
    uint8_t  cls;               /* imu_class_t (reported)                */
    uint8_t  hump_state;        /* imu_hump_state_t                      */
    uint8_t  last_peak_mm;
    uint8_t  max_peak_mm;       /* this run                              */
    uint16_t vib_mg;            /* vibration level                       */
    int16_t  fwd_mg;            /* forward acceleration (flat ground)    */
    uint16_t spike_mg;          /* largest spike since imu_logic_take_spike() */
} imu_view_t;

/* ---- Context (private state; static storage, about 4.5 KB) -------- */

typedef struct
{
    bool     b_level_check;     /* S4_CFG_COLLISION_LEVEL_CHECK          */
    bool     b_seeded;          /* gravity filter has a first value      */
    uint32_t cycle;             /* cycles since restart                  */

    int32_t  grav_f[3];         /* low-passed gravity, q2 x 16           */
    int16_t  mean_prev[2][3];   /* last two cycle means, q2              */
    int16_t  last_acc[3];       /* last sample of the previous cycle     */
    int32_t  vib_f;             /* q2 << 3                               */
    int32_t  vbias_f;           /* vertical bias, q2 x 256               */
    uint32_t bias_n;            /* flat samples learnt (saturates)       */
    int16_t  last_av;           /* vertical acceleration, last cycle     */
    int16_t  raw_ring[32];      /* vertical acc. before bias removal     */
    int16_t  pitch_ring[32];    /* recent pitch, cdeg                    */

    /* Humps */
    uint8_t  hump_state;
    uint32_t state_cycle;       /* cycle the state was entered           */
    uint32_t level_cycles;      /* consecutive level cycles (descent)    */
    uint32_t anchor;            /* cycle of the flat reference           */
    uint32_t flat_cycle;        /* last steady-flat cycle                */
    int32_t  v_sum;             /* sum of vertical acc. since anchor     */
    int64_t  z_sum;             /* sum of v_sum since anchor             */
    int64_t  z_min;             /* lowest z_sum since anchor             */
    int32_t  v_ring[8];         /* recent v_sum                          */
    int16_t  ep_max_pitch;
    int16_t  ep_min_pitch;
    uint16_t ep_max_spike;
    bool     fin_active;        /* end-of-hump maths in progress         */
    uint32_t fin_anchor;
    uint32_t fin_n;
    uint32_t fin_k;
    int32_t  fin_vn;
    int64_t  fin_q16;           /* P / D, Q16                            */
    int64_t  fin_v;
    int64_t  fin_z;
    int32_t  fin_peak;          /* um                                    */
    imu_hump_t fin_hump;
    int16_t  hist_av[IMU_HIST_LEN];     /* vertical acc., q2             */
    int16_t  hist_corr[IMU_HIST_LEN];   /* lever-arm correction, 10 um   */

    /* Heading and turn rate */
    int32_t  head_ring[IMU_HEAD_HIST_LEN];
    uint8_t  head_idx;
    uint8_t  head_n;
    uint32_t head_cycle;        /* cycle of the last heading             */

    /* Motion class */
    bool     b_turn;
    bool     b_accel;
    bool     b_still;
    uint8_t  cand;
    uint8_t  cand_cycles;
    uint32_t impact_until;      /* Sudden impact latched until this cycle */

    /* Collision and tip-over */
    uint32_t ignore_until;
    uint8_t  confirm_left;      /* level-check cycles still to go        */
    uint16_t confirm_spike;
    int32_t  confirm_pitch;
    int32_t  confirm_sum[3];
    uint32_t tip_cycles;
    bool     b_tip_sent;

    bool     b_motors_prev;
    imu_view_t view;
} imu_ctx_t;

/* ---- Calibration and self-test at rest (FR20) --------------------- */

typedef enum
{
    IMU_CAL_OK = 0,
    IMU_CAL_TOO_FEW,            /* not enough samples arrived            */
    IMU_CAL_MOVED,              /* car not still during the average      */
    IMU_CAL_NOT_1G              /* |a| not 1 g: sensor or scale wrong    */
} imu_cal_result_t;

typedef struct
{
    int32_t  sum[3];            /* q2                                    */
    int32_t  lo[3];             /* smallest 10 ms mean                   */
    int32_t  hi[3];
    uint32_t n;                 /* samples                               */
    uint32_t cycles;
} imu_cal_t;

/*
 * Mounting alignment from the at-rest average: the rotation that turns
 * the measured "down" into body -z (the board's tilt on its bracket),
 * and the gain that makes it exactly 1 g.  A tilt removed as a rotation
 * stays right on a slope; removed as an offset it would add a vertical
 * error of g x sin(tilt) x sin(pitch) on every ramp.
 */
typedef struct
{
    int16_t  r[3][3];           /* rotation, Q14 (16384 = 1)             */
    int32_t  gain_q14;          /* 1 g / measured |g|, Q14               */
} imu_align_t;

void    imu_cal_reset(imu_cal_t *p_cal);

/* Body q2 samples, before alignment */
void    imu_cal_add(imu_cal_t *p_cal, int16_t const (*p_acc)[3], uint8_t n);

/* Alignment that makes the resting reading (0, 0, 1 g); *p_g_mg = |a| */
imu_cal_result_t imu_cal_finish(imu_cal_t const *p_cal, uint32_t want_n,
                                imu_align_t *p_align, int32_t *p_g_mg);

void    imu_align_identity(imu_align_t *p_align);

/* Accelerometer: rotate and scale.  Magnetometer: rotate only. */
void    imu_align_acc(imu_align_t const *p_align, int16_t const *p_in,
                      int16_t *p_out);
void    imu_align_mag(imu_align_t const *p_align, int16_t const *p_in,
                      int16_t *p_out);

/* ---- Sensor -> body frame (axis mapping from imu_params.h) -------- */

/* Raw accelerometer counts -> body q2, the sensor's zero-g offsets
   (imu_params.h) removed.  Alignment is applied afterwards. */
void    imu_map_acc(int16_t const *p_raw, int16_t *p_body_q2);

/* Raw magnetometer counts -> body mG, hard/soft iron removed.  false if
   a channel is saturated. */
bool    imu_map_mag(int16_t const *p_raw, int16_t *p_body_mg);

/* ---- Processing --------------------------------------------------- */

/* Whole reset (boot): also clears the hump peaks. */
void    imu_logic_init(imu_ctx_t *p_ctx, bool b_level_check);

/* Entering MONITORING (from IDLE or after a fault): filters and states
   restart, the hump peaks are kept. */
void    imu_logic_restart(imu_ctx_t *p_ctx);

void    imu_logic_step(imu_ctx_t *p_ctx, imu_in_t const *p_in,
                       imu_out_t *p_out);

/* Largest horizontal spike since the last call (bench readout "pk") */
uint16_t imu_logic_take_spike(imu_ctx_t *p_ctx);

#endif /* IMU_LOGIC_H */
