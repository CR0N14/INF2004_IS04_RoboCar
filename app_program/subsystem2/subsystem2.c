/*
 *  subsystem2.c
 *  Subsystem 2 -- Motion Control task.
 *
 *  One task, priority PRI_MOTION_CONTROL (11), woken every MOTION_PERIOD_MS
 *  (10 ms, NFR5) by a cyclic handler.  Each cycle it:
 *    1. measures: encoder counts -> wheel speeds and distance
 *    2. reads its inputs, in this order (see read_inputs() for why):
 *         navigation commands  (IF-04, mbf_nav_cmd, never waits)
 *         mode flags           (IF-02, SM_FLG_MOTORS_ENABLED)
 *         status word          (IF-03, state + nav mode)
 *    3. runs the motion statechart (Week 6 design):
 *
 *         DISABLED --[MOTORS_ENABLED set]--> DRIVING
 *         DRIVING  --[cleared, not ERROR]--> STOPPING --[speed 0]--> DISABLED
 *         DRIVING  --[cleared, ERROR]------> BRAKING  --[speed 0]--> DISABLED
 *         DRIVING  --[stall]---------------> STALL_FAULT --[cleared]--> DISABLED
 *
 *       Inside DRIVING, what the wheels do depends on the nav mode the
 *       State Machine publishes, plus any manoeuvre it commanded:
 *         TRACKING  line following: cruise speed +- S3's steering
 *         MANOEUVRE a commanded turn (FR13), then SM_EVT_TURN_DONE
 *         BYPASS    the obstacle bypass script, then SM_EVT_BYPASS_DONE
 *         RECOVERY  creep while S3 looks for the line
 *         HOLD      stand still (obstacle being scanned)
 *    4. runs the speed controller per wheel and writes the PWM (this task
 *       is the ONLY writer of the motor pins), checks for stalls (FR10)
 *    5. publishes the telemetry snapshot (IF-07) and any control events
 *
 *  NFR10: nothing here waits on WiFi/MQTT, and the control loop never
 *  blocks: every kernel call in the cycle uses TMO_POL (poll, no wait).
 */

#include "subsystem2.h"
#include "../subsystem1/state_machine.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_logic.h"
#include "../demo_config.h"
#if ENCODER_FREE_DEMO
#include "demo_motion.h"
#include "../subsystem3/demo_ir.h"
#include "../subsystem4/demo_imu.h"
#include "../subsystem5/demo_sonar.h"
#endif

/* ---- Constants ------------------------------------------------------- */

#define S2_DT_S             ((float)MOTION_PERIOD_MS / 1000.0f)
#define S2_MS_TO_TICKS(ms)  (((ms) + MOTION_PERIOD_MS - 1u) / MOTION_PERIOD_MS)

#define S2_MAX_SEGMENTS     4u      /* longest manoeuvre (obstacle bypass) */
#define S2_NAV_MSG_SIZE     ((INT)sizeof(uint8_t))

/* s2_tlm_t field limits (bits in subsystem2.h) */
#define S2_TLM_SPEED_MAX    1023u
#define S2_TLM_DIST_MAX     16383u

/* Control events still to be delivered to the State Machine.  One bit
 * each: if its queue is full we try again next cycle instead of losing
 * the event (the State Machine waits for TURN_DONE, for example). */
#define PEND_READY          0x01u
#define PEND_TURN_DONE      0x02u
#define PEND_BYPASS_DONE    0x04u
#define PEND_STALL          0x08u

/* ---- Types ----------------------------------------------------------- */

typedef enum
{
    MS_DISABLED = 0,    /* PWM braked, encoders still measured          */
    MS_DRIVING,         /* closed-loop driving                          */
    MS_STOPPING,        /* normal stop: ramp speed down to 0            */
    MS_BRAKING,         /* ERROR: hard brake until stopped              */
    MS_STALL_FAULT      /* stall seen: braked, wait for State Machine   */
} motion_state_t;

typedef enum
{
    MAN_NONE = 0,
    MAN_TURN,           /* barcode / junction turn  -> TURN_DONE   */
    MAN_AVOID           /* obstacle bypass          -> BYPASS_DONE */
} man_kind_t;

/* A manoeuvre is a short list of segments ("drive 50 mm", "spin 90 deg"),
 * done one after the other and measured with the encoders. */
typedef struct
{
    man_kind_t  kind;
    ml_seg_t    seg[S2_MAX_SEGMENTS];
    uint8_t     n_seg;
    uint8_t     idx;                    /* segment being driven         */
    int32_t     start[WHEEL_COUNT];     /* counts when it started       */
    int32_t     speed_mmps;
} manoeuvre_t;

/* Everything the task knows.  Used only by the Motion Control task. */
typedef struct
{
    motion_state_t  state;
    manoeuvre_t     man;

    ml_speed_t      spd[WHEEL_COUNT];
    ml_pid_t        pid[WHEEL_COUNT];
    ml_stall_t      stall[WHEEL_COUNT];

    int32_t         count[WHEEL_COUNT];     /* latest encoder count     */
    int32_t         delta[WHEEL_COUNT];     /* counts moved this cycle  */
    float           speed[WHEEL_COUNT];     /* measured, mm/s           */
    int32_t         target[WHEEL_COUNT];    /* wanted, mm/s             */
    int32_t         duty[WHEEL_COUNT];      /* applied, %               */
    float           distance_mm;            /* this run                 */

    uint32_t        stop_ticks;             /* time spent STOPPING      */
    uint32_t        debug_ticks;
    uint8_t         pending;                /* PEND_* bits              */
    bool            b_selftest_done;        /* for this INITIALISING    */
    bool            b_config_warned;
} motion_ctx_t;

/* ---- Kernel objects and shared data ---------------------------------- */

static ID   g_bench_flags;
static uint32_t g_bench_deadline;
static UINT g_bench_wheel;
static bool g_bench_active;
static ID   g_tid_motion;
static ID   g_cyc_motion;
static ID   g_mtx_tlm;      /* protects g_tlm                           */
static ID   g_mtx_steer;    /* protects g_steer_*                       */

static motion_ctx_t g_ctx;  /* task-only, so no lock needed             */

/* IF-07 snapshot: written only by the Motion Control task */
static s2_tlm_t g_tlm;
static uint32_t g_tlm_ms;

/* Steering input: written only through s2_set_steering() */
static int8_t   g_steer_pct;
static uint32_t g_steer_ms;     /* when it was set                       */
static bool     g_b_steer_set;  /* ever set                              */

/* Last steering value the task managed to read (used if the mutex is
 * busy for one cycle, so the loop never has to wait) */
static int8_t   g_steer_cached_pct;
static uint32_t g_steer_cached_ms;
static bool     g_b_steer_cached;

/* ---- Small helpers --------------------------------------------------- */

static uint32_t
now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);
    return (uint32_t)tim.lo;
}

static int32_t
abs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

static bool
both_stopped(motion_ctx_t const *p_c)
{
    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        float const s = p_c->speed[w];

        if ((s > (float)STOPPED_SPEED_MMPS) || (s < -(float)STOPPED_SPEED_MMPS))
        {
            return false;
        }
    }
    return true;
}

static void
set_targets(motion_ctx_t *p_c, int32_t left_mmps, int32_t right_mmps)
{
    p_c->target[WHEEL_LEFT] = left_mmps;
    p_c->target[WHEEL_RIGHT] = right_mmps;
}

static void
reset_control(motion_ctx_t *p_c)
{
    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        ml_pid_reset(&p_c->pid[w]);
        ml_stall_reset(&p_c->stall[w]);
    }
}

/* Latest steering, if fresh.  TMO_POL: if S3 holds the mutex right now we
 * use what we read last time rather than wait. */
static bool
read_steering(int32_t *p_steer, uint32_t now)
{
    if (E_OK == tk_loc_mtx(g_mtx_steer, TMO_POL))
    {
        g_steer_cached_pct = g_steer_pct;
        g_steer_cached_ms = g_steer_ms;
        g_b_steer_cached = g_b_steer_set;
        (void)tk_unl_mtx(g_mtx_steer);
    }
    *p_steer = g_steer_cached_pct;
    return g_b_steer_cached && ((now - g_steer_cached_ms) <= 30u);
}

/* ---- Statechart transitions ------------------------------------------ */

static void
enter_disabled(motion_ctx_t *p_c)
{
    drv_motor_brake_all();
    p_c->state = MS_DISABLED;
    p_c->man.kind = MAN_NONE;
    set_targets(p_c, 0, 0);
    reset_control(p_c);
}

static void
enter_driving(motion_ctx_t *p_c)
{
    p_c->state = MS_DRIVING;
    p_c->man.kind = MAN_NONE;
    p_c->distance_mm = 0.0f;        /* a new run starts */
    set_targets(p_c, 0, 0);
    reset_control(p_c);
}

static void
enter_stopping(motion_ctx_t *p_c)
{
    p_c->state = MS_STOPPING;
    p_c->man.kind = MAN_NONE;
    p_c->stop_ticks = 0u;
    /* Ramp down from the speed the wheels really have, not from the old
       target, so the stop is smooth.  The PIDs keep their state. */
    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        p_c->target[w] = (int32_t)p_c->speed[w];
    }
}

static void
enter_braking(motion_ctx_t *p_c)
{
    drv_motor_brake_all();          /* FR12: at once, before anything else */
    p_c->state = MS_BRAKING;
    p_c->man.kind = MAN_NONE;
    set_targets(p_c, 0, 0);
}

static void
enter_stall_fault(motion_ctx_t *p_c, wheel_t wheel)
{
    drv_motor_brake_all();
    p_c->state = MS_STALL_FAULT;
    p_c->man.kind = MAN_NONE;
    set_targets(p_c, 0, 0);
    p_c->pending |= PEND_STALL;     /* -> SM_EVT_FAULT(SM_FAULT_MOTOR_STALL) */
    tm_printf((UB *)"s2: STALL on %s wheel (duty %d %%)\n",
              (WHEEL_LEFT == wheel) ? "left" : "right",
              (int)p_c->duty[wheel]);
}

/* ---- Manoeuvres (FR13 turns, obstacle bypass) ------------------------ */

static void
add_seg(manoeuvre_t *p_m, ml_seg_kind_t kind, int32_t value)
{
    if (p_m->n_seg < S2_MAX_SEGMENTS)
    {
        p_m->seg[p_m->n_seg].kind = kind;
        p_m->seg[p_m->n_seg].value = (int16_t)value;
        p_m->n_seg++;
    }
}

static void
start_manoeuvre(motion_ctx_t *p_c, man_kind_t kind, int32_t speed_mmps)
{
    p_c->man.kind = kind;
    p_c->man.idx = 0u;
    p_c->man.speed_mmps = speed_mmps;
    p_c->man.start[WHEEL_LEFT] = p_c->count[WHEEL_LEFT];
    p_c->man.start[WHEEL_RIGHT] = p_c->count[WHEEL_RIGHT];
    reset_control(p_c);
}

/* SM_NAV_CMD_TURN.  Positive spin = left (anticlockwise). */
static void
begin_turn(motion_ctx_t *p_c, sm_action_t action)
{
    manoeuvre_t *p_m = &p_c->man;

    p_m->n_seg = 0u;
    switch (action)
    {
        case SM_ACTION_LEFT:
            add_seg(p_m, ML_SEG_STRAIGHT, TURN_PRE_DRIVE_MM);
            add_seg(p_m, ML_SEG_SPIN, TURN_SIDE_DEG);
            break;

        case SM_ACTION_RIGHT:
            add_seg(p_m, ML_SEG_STRAIGHT, TURN_PRE_DRIVE_MM);
            add_seg(p_m, ML_SEG_SPIN, -TURN_SIDE_DEG);
            break;

        case SM_ACTION_STRAIGHT:
            add_seg(p_m, ML_SEG_STRAIGHT, JUNCTION_CLEAR_MM);
            break;

        case SM_ACTION_UTURN:
        default:
            add_seg(p_m, ML_SEG_SPIN, TURN_UTURN_DEG);
            break;
    }
    start_manoeuvre(p_c, MAN_TURN, TURN_SPEED_MMPS);
}

/* SM_NAV_CMD_AVOID.  DRAFT script, see motion_config.h. */
static void
begin_avoid(motion_ctx_t *p_c, sm_avoid_t rec)
{
    manoeuvre_t *p_m = &p_c->man;
    int32_t side;

    p_m->n_seg = 0u;
    switch (rec)
    {
        case SM_AVOID_LEFT:
        case SM_AVOID_RIGHT:
            side = (SM_AVOID_LEFT == rec) ? 1 : -1;     /* + = left */
            add_seg(p_m, ML_SEG_SPIN, side * AVOID_OUT_DEG);
            add_seg(p_m, ML_SEG_STRAIGHT, AVOID_SIDE_MM);
            add_seg(p_m, ML_SEG_SPIN, -side * 2 * AVOID_OUT_DEG);
            add_seg(p_m, ML_SEG_STRAIGHT, AVOID_SIDE_MM);
            break;

        case SM_AVOID_REVERSE:
            add_seg(p_m, ML_SEG_STRAIGHT, -AVOID_REVERSE_MM);
            break;

        default:
            return;     /* CONTINUE / STOP are not moves: nothing to do */
    }
    start_manoeuvre(p_c, MAN_AVOID, AVOID_SPEED_MMPS);
}

/* Drive the current segment; move to the next one when it is done. */
static void
run_manoeuvre(motion_ctx_t *p_c)
{
    manoeuvre_t *p_m = &p_c->man;
    float const mmpc = ml_mm_per_count();
    int32_t left;
    int32_t right;

    /* "while", not "if": a 0 mm segment (e.g. TURN_PRE_DRIVE_MM = 0) is
       done at once and is skipped in the same cycle. */
    while (p_m->idx < p_m->n_seg)
    {
        float const moved_l = (float)(p_c->count[WHEEL_LEFT] -
                                      p_m->start[WHEEL_LEFT]) * mmpc;
        float const moved_r = (float)(p_c->count[WHEEL_RIGHT] -
                                      p_m->start[WHEEL_RIGHT]) * mmpc;

        if (!ml_seg_done(&p_m->seg[p_m->idx], moved_l, moved_r))
        {
            break;
        }
        p_m->idx++;
        p_m->start[WHEEL_LEFT] = p_c->count[WHEEL_LEFT];
        p_m->start[WHEEL_RIGHT] = p_c->count[WHEEL_RIGHT];
        reset_control(p_c);     /* wheels may now go the other way */
    }

    if (p_m->idx >= p_m->n_seg)
    {
        /* Finished: tell the State Machine and stand still until it
           changes the nav mode (it runs at a higher priority, so that is
           normally the very next cycle). */
        p_c->pending |= (MAN_TURN == p_m->kind) ? PEND_TURN_DONE
                                                : PEND_BYPASS_DONE;
        p_m->kind = MAN_NONE;
        set_targets(p_c, 0, 0);
        return;
    }

    ml_seg_targets(&p_m->seg[p_m->idx], p_m->speed_mmps, &left, &right);
    set_targets(p_c, left, right);
}

/* ---- DRIVING: choose wheel targets ----------------------------------- */

static void
apply_nav_cmd(motion_ctx_t *p_c, uint8_t msg)
{
    uint8_t const arg = SM_NAV_CMD_ARG(msg);

    switch (SM_NAV_CMD_CODE(msg))
    {
        case SM_NAV_CMD_TURN:
            begin_turn(p_c, (sm_action_t)arg);
            break;

        case SM_NAV_CMD_AVOID:
            begin_avoid(p_c, (sm_avoid_t)arg);
            break;

        case SM_NAV_CMD_HOLD:
            p_c->man.kind = MAN_NONE;   /* nav mode HOLD stops the wheels */
            break;

        default:
            break;
    }
}

static void
drive(motion_ctx_t *p_c, sm_nav_mode_t nav, uint32_t now)
{
    int32_t steer;
    int32_t left;
    int32_t right;

    /* A manoeuvre only makes sense in its own nav mode.  If the State
       Machine has moved on (e.g. the line was found during a bypass, or
       an obstacle appeared mid-turn), drop it and follow the new mode. */
    if (((MAN_TURN == p_c->man.kind) && (SM_NAV_MANOEUVRE != nav)) ||
        ((MAN_AVOID == p_c->man.kind) && (SM_NAV_BYPASS != nav)))
    {
        p_c->man.kind = MAN_NONE;
    }

    if (MAN_NONE != p_c->man.kind)
    {
        run_manoeuvre(p_c);
        return;
    }

    switch (nav)
    {
        case SM_NAV_TRACKING:
            if (read_steering(&steer, now))
            {
                ml_steer_mix(CRUISE_SPEED_MMPS, steer, &left, &right);
                set_targets(p_c, left, right);
            }
            else if (MOTION_BRINGUP_IGNORE_STEERING)
            {
                set_targets(p_c, CRUISE_SPEED_MMPS, CRUISE_SPEED_MMPS);
            }
            else
            {
                set_targets(p_c, 0, 0);     /* no fresh steering: wait */
            }
            break;

        case SM_NAV_RECOVERY:
            /* Creep slowly; steer if S3 has a hint, else straight.
               DRAFT: the search pattern is an open item with S3. */
            if (read_steering(&steer, now))
            {
                ml_steer_mix(RECOVERY_SPEED_MMPS, steer, &left, &right);
                set_targets(p_c, left, right);
            }
            else
            {
                set_targets(p_c, 0, 0); /* Await an authorised fresh search target. */
            }
            break;

        case SM_NAV_HOLD:
        case SM_NAV_MANOEUVRE:  /* turn finished, waiting for TRACKING  */
        case SM_NAV_BYPASS:     /* bypass finished, waiting for RECOVERY */
        case SM_NAV_DISABLED:
        default:
            set_targets(p_c, 0, 0);
            break;
    }
}

/* ---- STOPPING: ramp targets towards 0 -------------------------------- */

static int32_t
ramp_to_zero(int32_t target)
{
    if (target > 0)
    {
        target = (target > STOP_DECEL_MMPS_PER_TICK)
                 ? (target - STOP_DECEL_MMPS_PER_TICK) : 0;
    }
    else if (target < 0)
    {
        target = (target < -STOP_DECEL_MMPS_PER_TICK)
                 ? (target + STOP_DECEL_MMPS_PER_TICK) : 0;
    }
    return target;
}

/* ---- One control cycle ----------------------------------------------- */

static void
measure(motion_ctx_t *p_c)
{
    float const mmpc = ml_mm_per_count();
    int32_t moved = 0;

    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        int32_t const now = drv_encoder_count((wheel_t)w);

        /* Unsigned subtraction: correct even when the counter wraps */
        p_c->delta[w] = (int32_t)((uint32_t)now - (uint32_t)p_c->count[w]);
        p_c->count[w] = now;
        p_c->speed[w] = ml_speed_update(&p_c->spd[w], now);
        moved += abs32(p_c->delta[w]);
    }
    /* Distance travelled by the car's centre = average wheel travel */
    p_c->distance_mm += ((float)moved * mmpc) / 2.0f;
}

/*
 * Read the State Machine's outputs.  The ORDER matters:
 *   - Commands first, then the status word: the State Machine writes the
 *     new nav mode BEFORE it sends a command, so a status word read after
 *     the command is at least as new as the command.
 *   - Flags before the status word: the State Machine writes the status
 *     word BEFORE it changes the flags, so if MOTORS_ENABLED has just been
 *     cleared, the status word read after it already says whether it was
 *     an ERROR (brake) or a stop command (ramp down).
 */
static INT
read_inputs(uint8_t *p_cmds, INT max_cmds, uint32_t *p_flags,
            uint32_t *p_status)
{
    INT n = 0;
    uint8_t msg;

    while (S2_NAV_MSG_SIZE == tk_rcv_mbf(mbf_nav_cmd, &msg, TMO_POL))
    {
        if (n < max_cmds)
        {
            p_cmds[n++] = msg;
        }
    }
    *p_flags = sm_mode_flags();
    *p_status = sm_status_word();
    return n;
}

/* Self-test once per INITIALISING (the State Machine re-enters it after an
 * ERROR retry).  There is nothing to test without moving the wheels, so it
 * checks that the measured values are filled in. */
static void
self_test(motion_ctx_t *p_c, uint32_t flags)
{
    if (0u == (flags & SM_FLG_INITIALISING))
    {
        p_c->b_selftest_done = false;
        return;
    }
    if (p_c->b_selftest_done)
    {
        return;
    }
    p_c->b_selftest_done = true;

    if (ml_config_ok())
    {
        p_c->pending |= PEND_READY;
    }
    else
    {
        /* Not reported as SELFTEST_FAIL on purpose: that would put the
           whole car into ERROR and block the rest of the team's testing
           until S2 has measured.  Motion just refuses to drive. */
        tm_printf((UB *)"s2: NOT READY: measure ENC_COUNTS_PER_REV, "
                  "TRACK_WIDTH_MM (motion_config.h)\n");
    }
}

static void
update_state(motion_ctx_t *p_c, bool b_enabled, sm_state_t sm_state)
{
    switch (p_c->state)
    {
        case MS_DISABLED:
            if (b_enabled)
            {
                if (ml_config_ok())
                {
                    enter_driving(p_c);
                }
                else if (!p_c->b_config_warned)
                {
                    p_c->b_config_warned = true;
                    tm_printf((UB *)"s2: motors enabled but motion_config.h "
                              "is not measured yet: staying DISABLED\n");
                }
            }
            else
            {
                p_c->b_config_warned = false;
            }
            break;

        case MS_DRIVING:
            if (!b_enabled)
            {
                if (SM_STATE_ERROR == sm_state)
                {
                    enter_braking(p_c);
                }
                else
                {
                    enter_stopping(p_c);
                }
            }
            break;

        case MS_STOPPING:
            if (SM_STATE_ERROR == sm_state)
            {
                enter_braking(p_c);     /* error during a stop: brake now */
            }
            else if (b_enabled)
            {
                enter_driving(p_c);     /* stopped and started again */
            }
            else if (((0 == p_c->target[WHEEL_LEFT]) &&
                      (0 == p_c->target[WHEEL_RIGHT]) && both_stopped(p_c)) ||
                     (p_c->stop_ticks >= S2_MS_TO_TICKS(STOP_TIMEOUT_MS)))
            {
                enter_disabled(p_c);
            }
            break;

        case MS_BRAKING:
            if (both_stopped(p_c))
            {
                enter_disabled(p_c);
            }
            break;

        case MS_STALL_FAULT:
        default:
            if (!b_enabled)
            {
                enter_disabled(p_c);
            }
            break;
    }
}

/* Speed controller and PWM output for both wheels */
static void
control_wheels(motion_ctx_t *p_c)
{
    bool const b_closed_loop = (MS_DRIVING == p_c->state) ||
                               (MS_STOPPING == p_c->state);

    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        wheel_t const wheel = (wheel_t)w;

        if (!b_closed_loop)
        {
            /* DISABLED / BRAKING / STALL_FAULT: hold the brake.  Writing
               it every cycle costs nothing and means nothing can leave a
               motor running by mistake. */
            drv_motor_brake(wheel);
            p_c->duty[w] = 0;
            continue;
        }

        if ((0 == p_c->target[w]) &&
            (p_c->speed[w] <= (float)STOPPED_SPEED_MMPS) &&
            (p_c->speed[w] >= -(float)STOPPED_SPEED_MMPS))
        {
            /* Asked to stand still and already still: brake instead of
               letting the PID hum around 0 %. */
            drv_motor_brake(wheel);
            ml_pid_reset(&p_c->pid[w]);
            p_c->duty[w] = 0;
        }
        else
        {
            p_c->duty[w] = ml_pid_step(&p_c->pid[w], (float)p_c->target[w],
                                       p_c->speed[w], S2_DT_S);
            drv_motor_set(wheel, p_c->duty[w]);
        }

        /* FR10 */
        if ((MS_DRIVING == p_c->state) &&
            ml_stall_update(&p_c->stall[w], p_c->duty[w], p_c->delta[w]))
        {
            enter_stall_fault(p_c, wheel);
            return;     /* both wheels are braked now */
        }
    }
}

/* Deliver pending control events (IF-01); keep any that did not fit */
static void
send_events(motion_ctx_t *p_c)
{
    if ((0u != (p_c->pending & PEND_STALL)) &&
        (E_OK == sm_post_event(SM_EVT_FAULT, SM_FAULT_MOTOR_STALL)))
    {
        p_c->pending &= (uint8_t)~PEND_STALL;
    }
    if ((0u != (p_c->pending & PEND_TURN_DONE)) &&
        (E_OK == sm_post_event(SM_EVT_TURN_DONE, 0u)))
    {
        p_c->pending &= (uint8_t)~PEND_TURN_DONE;
    }
    if ((0u != (p_c->pending & PEND_BYPASS_DONE)) &&
        (E_OK == sm_post_event(SM_EVT_BYPASS_DONE, 0u)))
    {
        p_c->pending &= (uint8_t)~PEND_BYPASS_DONE;
    }
    if ((0u != (p_c->pending & PEND_READY)) &&
        (E_OK == sm_post_event(SM_EVT_READY, SM_SUBSYS_MOTION)))
    {
        p_c->pending &= (uint8_t)~PEND_READY;
    }
}

static s2_motion_state_t
tlm_state(motion_ctx_t const *p_c)
{
    switch (p_c->state)
    {
        case MS_DRIVING:
            if (MAN_TURN == p_c->man.kind)
            {
                return S2_MOTION_TURNING;
            }
            if (MAN_AVOID == p_c->man.kind)
            {
                return S2_MOTION_AVOIDING;
            }
            return S2_MOTION_LINE_FOLLOWING;

        case MS_STALL_FAULT:
            return S2_MOTION_STALL_FAULT;

        /* STOPPING / BRAKING are reported as DISABLED (the telemetry enum
           has no value for them); the speed fields show it still moving. */
        default:
            return S2_MOTION_DISABLED;
    }
}

static void
speed_field(float speed, unsigned int *p_mag, unsigned int *p_b_rev)
{
    int32_t const mmps = (int32_t)((speed >= 0.0f) ? (speed + 0.5f)
                                                   : (speed - 0.5f));
    uint32_t mag = (uint32_t)abs32(mmps);

    if (mag > S2_TLM_SPEED_MAX)
    {
        mag = S2_TLM_SPEED_MAX;
    }
    *p_mag = mag;
    *p_b_rev = (mmps < 0) ? 1u : 0u;
}

/* IF-07: build the snapshot locally, then copy it in under the mutex, so
 * the mutex is held only for one struct copy.  TMO_POL: if Networking is
 * reading right now, skip this update (the next one is 10 ms away). */
static void
publish(motion_ctx_t const *p_c)
{
    s2_tlm_t t = {0};
    unsigned int mag;
    unsigned int b_rev;
    uint32_t dist_cm = (uint32_t)(p_c->distance_mm / 10.0f);

    speed_field(p_c->speed[WHEEL_LEFT], &mag, &b_rev);
    t.speed_l_mms = mag;
    t.b_rev_l = b_rev;
    speed_field(p_c->speed[WHEEL_RIGHT], &mag, &b_rev);
    t.speed_r_mms = mag;
    t.b_rev_r = b_rev;
    t.motion_state = (unsigned int)tlm_state(p_c);
    t.enc_l = (uint16_t)p_c->count[WHEEL_LEFT];     /* low 16 bits, wraps */
    t.enc_r = (uint16_t)p_c->count[WHEEL_RIGHT];
    t.distance_cm = (dist_cm > S2_TLM_DIST_MAX) ? S2_TLM_DIST_MAX : dist_cm;

    if (E_OK == tk_loc_mtx(g_mtx_tlm, TMO_POL))
    {
        g_tlm = t;
        g_tlm_ms = now_ms();
        (void)tk_unl_mtx(g_mtx_tlm);
    }
}

static void
debug_print(motion_ctx_t *p_c)
{
#if MOTION_DEBUG_PRINT_MS > 0
    if (++p_c->debug_ticks >= S2_MS_TO_TICKS(MOTION_DEBUG_PRINT_MS))
    {
        p_c->debug_ticks = 0u;
        tm_printf((UB *)"s2: st %d  L cnt %d err %d v %d/%d duty %d  "
                  "R cnt %d err %d v %d/%d duty %d  dist %d mm\n",
                  (int)p_c->state,
                  (int)p_c->count[WHEEL_LEFT],
                  (int)drv_encoder_errors(WHEEL_LEFT),
                  (int)p_c->speed[WHEEL_LEFT], (int)p_c->target[WHEEL_LEFT],
                  (int)p_c->duty[WHEEL_LEFT],
                  (int)p_c->count[WHEEL_RIGHT],
                  (int)drv_encoder_errors(WHEEL_RIGHT),
                  (int)p_c->speed[WHEEL_RIGHT], (int)p_c->target[WHEEL_RIGHT],
                  (int)p_c->duty[WHEEL_RIGHT],
                  (int)p_c->distance_mm);
    }
#else
    (void)p_c;
#endif
}

/* Request bits are atomically consumed by the sole motor owner. */
static bool
bench_cycle(uint32_t status, uint32_t now)
{
    UINT requests = 0u;
    (void)tk_wai_flg(g_bench_flags, 7u, TWF_ORW | TWF_BITCLR,
                     &requests, TMO_POL);
    if ((0u != (requests & 4u)) ||
        (SM_STATE_STANDBY != SM_STATUS_STATE(status)) ||
        (0u != (SM_STATUS_FAULTS(status) & ~SM_FAULT_NON_FATAL_MASK)))
    {
        if (g_bench_active) { drv_motor_brake_all(); }
        g_bench_active = false;
        return false;
    }
    if (g_bench_active && ((int32_t)(now - g_bench_deadline) >= 0))
    {
        g_bench_active = false;
        drv_motor_brake_all();
        return false;
    }
    if (!g_bench_active && ((requests == 1u) || (requests == 2u)))
    {
        g_bench_wheel = (requests == 1u) ? 0u : 1u;
        g_bench_deadline = now + 500u;
        g_bench_active = true;
    }
    if (g_bench_active)
    {
        drv_motor_brake((wheel_t)(1u - g_bench_wheel));
        drv_motor_set((wheel_t)g_bench_wheel, 20);
    }
    return g_bench_active;
}

EXPORT ER
s2_bench_request(UINT wheel)
{
    if (ENCODER_FREE_DEMO || !MOTION_MQTT_BENCH || wheel > 1u ||
        SM_STATUS_STATE(sm_status_word()) != SM_STATE_STANDBY)
    {
        return E_PAR;
    }
    return tk_set_flg(g_bench_flags, 1u << wheel);
}

EXPORT ER
s2_bench_cancel(void)
{
    return tk_set_flg(g_bench_flags, 4u);
}

static int g_demo_left, g_demo_right;
static uint32_t g_demo_stamp;
ER s2_demo_duty(int *left,int *right,uint32_t *stamp){
 ER e=tk_loc_mtx(g_mtx_tlm,TMO_POL);
 if(e==E_OK){*left=g_demo_left;*right=g_demo_right;*stamp=g_demo_stamp;tk_unl_mtx(g_mtx_tlm);}return e;
}
#if ENCODER_FREE_DEMO
static void demo_cycle(void){
 static demo_motion_t ctx;
 static sm_evt_code_t pending;static uint16_t arg;
 static sm_nav_mode_t previous_nav;
 static demo_line_t line;
 static demo_imu_t imu;
 uint8_t cmds[SM_NAV_QUEUE_LEN]={0};uint32_t flags,status;
 INT n=read_inputs(cmds,SM_NAV_QUEUE_LEN,&flags,&status);
 uint32_t now=now_ms();
 sm_nav_mode_t nav=SM_STATUS_NAV(status);
 if(nav!=previous_nav&&pending!=SM_EVT_FAULT)pending=SM_EVT_NONE;
 previous_nav=nav;
 (void)demo_ir_read(&line);
 (void)demo_imu_read(&imu);
 now=now_ms(); /* Snapshots may have been published during the reads. */
 bool pickup=!DEMO_IGNORE_IMU && imu.latched;
 /* Bypass inputs: S4 heading and the latest S5 reading with its angle. */
 demo_sense_t sense={0};
 sense.heading_ok=imu.heading_ok&&now-imu.time_ms<=100u;
 sense.heading_cdeg=imu.heading_cdeg;
 sense.sonar_ok=s5_demo_reading(&sense.sonar_cm,&sense.sonar_deg,&sense.sonar_time)==E_OK;
 ctx.sense=sense;
 demo_output_t o=demo_motion_step(&ctx,now,status,&line,n?cmds[n-1]:0,pickup);
 s5_demo_set_aim(o.aim?o.aim_deg:90u);
 if(!DEMO_IGNORE_IMU && !pickup && (!imu.ready||now-imu.time_ms>100u)) {
  o.left=0;o.right=0;
  if(SM_STATUS_STATE(status)==SM_STATE_NAVIGATING ||
     SM_STATUS_STATE(status)==SM_STATE_OBSTACLE_AVOIDANCE) {
   ctx.failed=true;ctx.failure=SM_FAULT_IMU;
   o.event=SM_EVT_FAULT;o.arg=SM_FAULT_IMU;
  }
 }
 /* STOP/error/HOLD decisions take effect at the sole motor owner. */
 if(!(flags&SM_FLG_MOTORS_ENABLED)){o.left=0;o.right=0;pending=SM_EVT_NONE;}
 if(o.event!=SM_EVT_NONE){pending=o.event;arg=o.arg;}
 if(o.left)drv_motor_set(WHEEL_LEFT,o.left);else drv_motor_brake(WHEEL_LEFT);
 if(o.right)drv_motor_set(WHEEL_RIGHT,o.right);else drv_motor_brake(WHEEL_RIGHT);
 if(pending!=SM_EVT_NONE&&sm_post_event(pending,arg)==E_OK)pending=SM_EVT_NONE;
 if(tk_loc_mtx(g_mtx_tlm,TMO_POL)==E_OK){g_demo_left=o.left;g_demo_right=o.right;g_demo_stamp=now;tk_unl_mtx(g_mtx_tlm);}
}
#endif
static void
motion_cycle(motion_ctx_t *p_c)
{
    uint8_t  cmds[SM_NAV_QUEUE_LEN];
    uint32_t flags;
    uint32_t status;
    uint32_t const now = now_ms();

#if ENCODER_FREE_DEMO
    (void)p_c;(void)cmds;(void)flags;(void)status;(void)now;
    demo_cycle();return;
#endif
    /* 1. Measure */
    measure(p_c);

    /* 2. Inputs */
    INT const n_cmds = read_inputs(cmds, (INT)SM_NAV_QUEUE_LEN, &flags,
                                   &status);
    self_test(p_c, flags);

    /* 3. Statechart */
    update_state(p_c, 0u != (flags & SM_FLG_MOTORS_ENABLED),
                 SM_STATUS_STATE(status));

    switch (p_c->state)
    {
        case MS_DRIVING:
            /* Commands only matter while driving; otherwise they are
               dropped (the queue was emptied above either way). */
            for (INT i = 0; i < n_cmds; i++)
            {
                apply_nav_cmd(p_c, cmds[i]);
            }
            drive(p_c, SM_STATUS_NAV(status), now);
            break;

        case MS_STOPPING:
            p_c->stop_ticks++;
            p_c->target[WHEEL_LEFT] = ramp_to_zero(p_c->target[WHEEL_LEFT]);
            p_c->target[WHEEL_RIGHT] = ramp_to_zero(p_c->target[WHEEL_RIGHT]);
            break;

        default:
            break;
    }

    /* Bench override remains owned by this task and expires after 500 ms. */
    if (!bench_cycle(status, now))
    {
        control_wheels(p_c);
    }

    /* 5. Outputs */
    send_events(p_c);
    publish(p_c);
    debug_print(p_c);
}

/* ---- Task and cyclic handler ----------------------------------------- */

/* Runs every MOTION_PERIOD_MS in the kernel's timer interrupt: it only
 * wakes the task, the work is done in task context. */
static void
motion_cyclic_handler(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(g_tid_motion);
}

static void
motion_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        /* If a cycle ever overran, more than one wake-up is queued.  Drop
           the extras: running several cycles back to back would not make
           up for the lost time, it would just feed the PID bad dt values. */
        (void)tk_can_wup(TSK_SELF);
        motion_cycle(&g_ctx);
    }
}

/* ---- Public interface ------------------------------------------------ */

EXPORT ER
s2_get_tlm(s2_tlm_t *p_tlm)
{
#if ENCODER_FREE_DEMO
    (void)p_tlm; return E_NOSPT; /* No fabricated speed, distance or counts. */
#endif
    ER er;

    if (NULL == p_tlm)
    {
        return E_PAR;
    }
    er = tk_loc_mtx(g_mtx_tlm, TMO_FEVR);
    if (E_OK == er)
    {
        *p_tlm = g_tlm;
        (void)tk_unl_mtx(g_mtx_tlm);
    }
    return er;
}

EXPORT ER
s2_set_steering(INT steer_pct)
{
    ER er;

    if (steer_pct > S2_STEER_MAX_PCT)
    {
        steer_pct = S2_STEER_MAX_PCT;
    }
    if (steer_pct < -S2_STEER_MAX_PCT)
    {
        steer_pct = -S2_STEER_MAX_PCT;
    }

    er = tk_loc_mtx(g_mtx_steer, TMO_FEVR);
    if (E_OK == er)
    {
        g_steer_pct = (int8_t)steer_pct;
        g_steer_ms = now_ms();
        g_b_steer_set = true;
        (void)tk_unl_mtx(g_mtx_steer);
    }
    return er;
}

EXPORT ER
s2_set_line_target(INT steer_pct, uint32_t acquired_ms, BOOL valid)
{
    ER er = tk_loc_mtx(g_mtx_steer, TMO_POL);
    if (E_OK != er) { return er; }
    g_b_steer_set = valid && ((now_ms() - acquired_ms) <= 30u);
    g_steer_pct = (int8_t)((steer_pct > 100) ? 100 :
                          ((steer_pct < -100) ? -100 : steer_pct));
    g_steer_ms = acquired_ms;
    (void)tk_unl_mtx(g_mtx_steer);
    return E_OK;
}

EXPORT ER
s2_get_motion_sample(s2_tlm_t *p_tlm, uint32_t *p_acquired_ms)
{
    if ((NULL == p_tlm) || (NULL == p_acquired_ms)) { return E_PAR; }
    ER er = tk_loc_mtx(g_mtx_tlm, TMO_POL);
    if (E_OK != er) { return er; }
    *p_tlm = g_tlm;
    *p_acquired_ms = g_tlm_ms;
    (void)tk_unl_mtx(g_mtx_tlm);
    return E_OK;
}

EXPORT ER
subsystem2_init(void)
{
    T_CMTX  cmtx = {0};
    T_CCYC  ccyc = {0};
    ER      er;

    T_CFLG bench = {0};
    bench.flgatr = TA_TFIFO;
    g_bench_flags = tk_cre_flg(&bench);
    if (g_bench_flags < E_OK) { return g_bench_flags; }

    /* Motors first: hold them braked as early as possible */
    drv_motor_init();

    er = E_OK;
#if !ENCODER_FREE_DEMO
    er = drv_encoder_init();
#endif
    if (er < E_OK)
    {
        return er;
    }

    /* Priority inheritance: if low-priority Networking holds the mutex
       when we want it, it is raised to our priority until it lets go. */
    cmtx.mtxatr = TA_INHERIT;
    g_mtx_tlm = tk_cre_mtx(&cmtx);
    if (g_mtx_tlm < E_OK)
    {
        return g_mtx_tlm;
    }
    g_mtx_steer = tk_cre_mtx(&cmtx);
    if (g_mtx_steer < E_OK)
    {
        return g_mtx_steer;
    }

    /* Created stopped; subsystem2_start() starts it once the task exists */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)motion_cyclic_handler;
    ccyc.cyctim = MOTION_PERIOD_MS;
    ccyc.cycphs = MOTION_PERIOD_MS;
    g_cyc_motion = tk_cre_cyc(&ccyc);
    if (g_cyc_motion < E_OK)
    {
        return g_cyc_motion;
    }

    /* Start the speed windows from the current counts */
    for (INT w = 0; w < (INT)WHEEL_COUNT; w++)
    {
        g_ctx.count[w] = drv_encoder_count((wheel_t)w);
        ml_speed_reset(&g_ctx.spd[w], g_ctx.count[w]);
    }
    enter_disabled(&g_ctx);
    return E_OK;
}

EXPORT ER
subsystem2_start(void)
{
    g_tid_motion = app_start_task(motion_task, S2_PRI_MAIN, 0u,
                                  "motion_task");
    if (g_tid_motion < E_OK)
    {
        return g_tid_motion;
    }
    return tk_sta_cyc(g_cyc_motion);
}
