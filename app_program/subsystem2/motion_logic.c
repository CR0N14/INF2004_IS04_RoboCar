/*
 *  motion_logic.c
 *  Motion Control maths.  Pure logic: see motion_logic.h.
 *
 *  Floats are used for readability.  The RP2040 has no FPU, so the compiler
 *  does float maths in software (a few microseconds per operation); at
 *  2 wheels x 100 Hz that is well under 1 % of one core.
 */

#include "motion_logic.h"

#define ML_PI           3.14159265f
#define ML_MS_PER_S     1000.0f

static float
clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static int32_t
round_to_int(float x)
{
    return (int32_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}

bool
ml_config_ok(void)
{
    return (ENC_COUNTS_PER_REV > 0) && (WHEEL_DIAMETER_MM > 0) &&
           (TRACK_WIDTH_MM > 0);
}

/* ---- Unit conversion ------------------------------------------------- */

float
ml_mm_per_count(void)
{
#if ENC_COUNTS_PER_REV > 0
    return (ML_PI * (float)WHEEL_DIAMETER_MM) / (float)ENC_COUNTS_PER_REV;
#else
    return 0.0f;        /* not measured yet: everything reads as 0 mm */
#endif
}

float
ml_spin_arc_mm(int32_t deg)
{
    return (ML_PI * (float)TRACK_WIDTH_MM * (float)deg) / 360.0f;
}

/* ---- Speed measurement ----------------------------------------------- */

#define ML_SPEED_SLOTS  (SPEED_WINDOW_TICKS + 1u)

void
ml_speed_reset(ml_speed_t *p_s, int32_t count)
{
    for (uint32_t i = 0u; i < ML_SPEED_SLOTS; i++)
    {
        p_s->history[i] = count;
    }
    p_s->newest = 0u;
    p_s->filled = 1u;
}

float
ml_speed_update(ml_speed_t *p_s, int32_t count)
{
    uint32_t span;
    uint32_t oldest;
    int32_t  delta;

    /* Ring buffer: overwrite the oldest slot with the newest count */
    p_s->newest = (uint8_t)((p_s->newest + 1u) % ML_SPEED_SLOTS);
    p_s->history[p_s->newest] = count;
    if (p_s->filled < ML_SPEED_SLOTS)
    {
        p_s->filled++;      /* just after a reset the window is shorter */
    }

    span = p_s->filled - 1u;                    /* ticks covered */
    oldest = (p_s->newest + ML_SPEED_SLOTS - span) % ML_SPEED_SLOTS;

    /* Subtract as unsigned so a counter wrap still gives the right
       difference (signed overflow is undefined in C). */
    delta = (int32_t)((uint32_t)count - (uint32_t)p_s->history[oldest]);

    if (0u == span)
    {
        return 0.0f;
    }
    return ((float)delta * ml_mm_per_count() * ML_MS_PER_S) /
           (float)(span * MOTION_PERIOD_MS);
}

/* ---- PI(D) speed controller ------------------------------------------ */

void
ml_pid_reset(ml_pid_t *p_pid)
{
    p_pid->i_term = 0.0f;
    p_pid->prev_error = 0.0f;
    p_pid->b_first = true;
}

float
ml_feed_forward(float target_mmps)
{
#if MOTOR_TOP_SPEED_MMPS > 0
    float const mag = (target_mmps < 0.0f) ? -target_mmps : target_mmps;
    float ff;

    if (mag < 1.0f)
    {
        return 0.0f;                /* asking for 0: no push */
    }
    ff = (float)MOTOR_START_DUTY_PCT +
         ((100.0f - (float)MOTOR_START_DUTY_PCT) * mag) /
         (float)MOTOR_TOP_SPEED_MMPS;
    ff = clampf(ff, 0.0f, 100.0f);
    return (target_mmps < 0.0f) ? -ff : ff;
#else
    (void)target_mmps;
    return 0.0f;                    /* not measured yet: pure PI */
#endif
}

int32_t
ml_pid_step(ml_pid_t *p_pid, float target_mmps, float measured_mmps,
            float dt_s)
{
    float const error = target_mmps - measured_mmps;
    float const ff = ml_feed_forward(target_mmps);
    float const p_term = PID_KP * error;
    float d_term = 0.0f;
    float i_new;
    float out;

    if (!p_pid->b_first && (dt_s > 0.0f))
    {
        d_term = (PID_KD * (error - p_pid->prev_error)) / dt_s;
    }
    p_pid->prev_error = error;
    p_pid->b_first = false;

    /* Integral: add this step's error, and limit it (anti-windup #1) */
    i_new = clampf(p_pid->i_term + (PID_KI * error * dt_s),
                   -PID_I_MAX, PID_I_MAX);
    out = ff + p_term + i_new + d_term;

    /* Anti-windup #2: if the output is already at its limit and the error
       would push it further, do not let the integral grow.  Otherwise it
       keeps growing while the wheel cannot go faster, and the wheel
       overshoots badly once it can. */
    if (((out > 100.0f) && (error > 0.0f)) ||
        ((out < -100.0f) && (error < 0.0f)))
    {
        out = ff + p_term + p_pid->i_term + d_term;
    }
    else
    {
        p_pid->i_term = i_new;
    }

    return round_to_int(clampf(out, -100.0f, 100.0f));
}

/* ---- Steering mix ---------------------------------------------------- */

void
ml_steer_mix(int32_t base_mmps, int32_t steer_pct,
             int32_t *p_left_mmps, int32_t *p_right_mmps)
{
    if (steer_pct > 100)
    {
        steer_pct = 100;
    }
    if (steer_pct < -100)
    {
        steer_pct = -100;
    }

    *p_left_mmps = base_mmps;
    *p_right_mmps = base_mmps;
    if (steer_pct > 0)
    {
        *p_right_mmps = (base_mmps * (100 - steer_pct)) / 100;  /* right */
    }
    else if (steer_pct < 0)
    {
        *p_left_mmps = (base_mmps * (100 + steer_pct)) / 100;   /* left  */
    }
}

/* ---- Stall detection ------------------------------------------------- */

void
ml_stall_reset(ml_stall_t *p_st)
{
    p_st->still_ticks = 0u;
}

bool
ml_stall_update(ml_stall_t *p_st, int32_t duty_pct, int32_t delta_counts)
{
    int32_t const mag = (duty_pct < 0) ? -duty_pct : duty_pct;

    if ((mag > STALL_DUTY_MIN_PCT) && (0 == delta_counts))
    {
        p_st->still_ticks++;    /* pushing hard, but not moving */
    }
    else
    {
        p_st->still_ticks = 0u; /* moving, or not pushing: no stall */
    }
    return (p_st->still_ticks * MOTION_PERIOD_MS) >= STALL_TIME_MS;
}

/* ---- Manoeuvre segments ---------------------------------------------- */

void
ml_seg_targets(ml_seg_t const *p_seg, int32_t speed_mmps,
               int32_t *p_left_mmps, int32_t *p_right_mmps)
{
    int32_t const dir = (p_seg->value >= 0) ? 1 : -1;

    if (ML_SEG_STRAIGHT == p_seg->kind)
    {
        *p_left_mmps = dir * speed_mmps;
        *p_right_mmps = dir * speed_mmps;
    }
    else
    {
        /* Spin left (anticlockwise): left wheel back, right wheel forward */
        *p_left_mmps = -dir * speed_mmps;
        *p_right_mmps = dir * speed_mmps;
    }
}

bool
ml_seg_done(ml_seg_t const *p_seg, float moved_l_mm, float moved_r_mm)
{
    float travelled;
    float goal;

    if (ML_SEG_STRAIGHT == p_seg->kind)
    {
        travelled = (moved_l_mm + moved_r_mm) / 2.0f;   /* car centre */
        goal = (float)p_seg->value;
    }
    else
    {
        travelled = (moved_r_mm - moved_l_mm) / 2.0f;   /* arc per wheel */
        goal = ml_spin_arc_mm(p_seg->value);
    }
    return (p_seg->value >= 0) ? (travelled >= goal) : (travelled <= goal);
}
