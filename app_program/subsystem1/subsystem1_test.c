/*
 *  subsystem1_test.c
 *  On-target tests for subsystem 1.  Built only with: make TEST=1
 *  Record results in testing/subsystem1/TEST_LOG.md.  IDs: S1-U01, ...
 *
 *  The statechart tests drive a private copy of the statechart logic
 *  (sm_logic.c), so they never move the real car.
 */

#include "subsystem1.h"
#include "sm_logic.h"
#include "../test_report.h"

#if APP_TEST

static sm_actions_t g_act;

static void
step(sm_ctx_t *p_ctx, sm_evt_code_t code, uint16_t arg)
{
    sm_logic_step(p_ctx, code, arg, &g_act);
}

/* A context that has finished initialising and sits in STANDBY */
static void
make_standby(sm_ctx_t *p_ctx)
{
    sm_logic_init(p_ctx, &g_act);
    step(p_ctx, SM_EVT_NET_UP, 0u);
    step(p_ctx, SM_EVT_READY, SM_SUBSYS_MOTION);
    step(p_ctx, SM_EVT_READY, SM_SUBSYS_LINE);
    step(p_ctx, SM_EVT_READY, SM_SUBSYS_IMU);
    step(p_ctx, SM_EVT_READY, SM_SUBSYS_ULTRASOUND);
    step(p_ctx, SM_EVT_CHECK, 0u);
}

static void
make_navigating(sm_ctx_t *p_ctx)
{
    make_standby(p_ctx);
    step(p_ctx, SM_EVT_CMD_START, 0u);
}

EXPORT void
subsystem1_test(void)
{
    sm_ctx_t    ctx;
    sm_action_t action;
    uint32_t    i;
    bool        b_ok;

    TEST_INIT();

    /* S1-U01: the packed types fit their storage */
    TEST_CHECK("S1-U01", (sizeof(sm_ctx_t) == 4u) &&
               (sizeof(sm_actions_t) == 4u) &&
               (SM_EVT_COUNT <= (1u << SM_EVT_CODE_BITS)) &&
               (SM_FAULT_COUNT <= SM_FAULT_BITS),
               "statechart context and events are packed");

    /* S1-U02: power-on enters INITIALISING with only the INIT flag */
    sm_logic_init(&ctx, &g_act);
    TEST_CHECK("S1-U02", (SM_STATE_INITIALISING == ctx.state) &&
               (SM_TIMER_START_INIT == g_act.timer) &&
               (SM_FLG_INITIALISING ==
                sm_logic_mode_flags(SM_STATE_INITIALISING)),
               "power-on enters INITIALISING");

    /* S1-U03: required subsystems ready -> STANDBY, sensors only */
    make_standby(&ctx);
    TEST_CHECK("S1-U03", (SM_STATE_STANDBY == ctx.state) &&
               (SM_FLG_SENSORS_ENABLED ==
                sm_logic_mode_flags(SM_STATE_STANDBY)),
               "ready subsystems move INITIALISING to STANDBY");

    /* S1-U04: start in INITIALISING is rejected and does not fire
       later (design review 8.3 gap) */
    sm_logic_init(&ctx, &g_act);
    step(&ctx, SM_EVT_CMD_START, 0u);
    b_ok = (0u != g_act.b_ack) && (0u == g_act.b_ack_ok) &&
           (SM_STATE_INITIALISING == ctx.state);
    step(&ctx, SM_EVT_NET_UP, 0u);
    for (i = 0u; i < SM_SUBSYS_COUNT; i++)
    {
        step(&ctx, SM_EVT_READY, (uint16_t)i);
    }
    TEST_CHECK("S1-U04", b_ok && (SM_STATE_STANDBY == ctx.state),
               "start rejected outside STANDBY, never replayed");

    /* S1-U05: start -> NAVIGATING with sensors and motors enabled */
    make_navigating(&ctx);
    TEST_CHECK("S1-U05", (SM_STATE_NAVIGATING == ctx.state) &&
               (0u != g_act.b_ack_ok) &&
               (SM_NAV_TRACKING == ctx.nav_mode) &&
               ((SM_FLG_SENSORS_ENABLED | SM_FLG_MOTORS_ENABLED) ==
                sm_logic_mode_flags(SM_STATE_NAVIGATING)),
               "start moves STANDBY to NAVIGATING");

    /* S1-U06: stop -> STANDBY from NAVIGATING and OBSTACLE_AVOIDANCE */
    step(&ctx, SM_EVT_CMD_STOP, 0u);
    b_ok = (SM_STATE_STANDBY == ctx.state);
    make_navigating(&ctx);
    step(&ctx, SM_EVT_OBSTACLE, 0u);
    step(&ctx, SM_EVT_CMD_STOP, 0u);
    TEST_CHECK("S1-U06", b_ok && (SM_STATE_STANDBY == ctx.state) &&
               (0u != g_act.b_ack_ok),
               "stop returns to STANDBY from both running states");

    /* S1-U07: losing WiFi while navigating does not stop the car */
    make_navigating(&ctx);
    step(&ctx, SM_EVT_NET_DOWN, 0u);
    b_ok = (SM_STATE_NAVIGATING == ctx.state) &&
           (0u != (ctx.fault_mask & SM_FAULT_BIT(SM_FAULT_WIFI_LOSS)));
    step(&ctx, SM_EVT_NET_UP, 0u);
    TEST_CHECK("S1-U07", b_ok && (0u == ctx.fault_mask),
               "WiFi loss is reported but does not stop the car (FR6)");

    /* S1-U08: an IMU fault is non-fatal, a stall is fatal */
    step(&ctx, SM_EVT_FAULT, SM_FAULT_IMU);
    b_ok = (SM_STATE_NAVIGATING == ctx.state);
    step(&ctx, SM_EVT_FAULT, SM_FAULT_MOTOR_STALL);
    TEST_CHECK("S1-U08", b_ok && (SM_STATE_ERROR == ctx.state) &&
               (0u == sm_logic_mode_flags(SM_STATE_ERROR)) &&
               (0u != (ctx.fault_mask &
                       SM_FAULT_BIT(SM_FAULT_MOTOR_STALL))),
               "fatal fault enters ERROR, IMU fault does not");

    /* S1-U09: ERROR retries INITIALISING SM_MAX_RETRIES times */
    sm_logic_init(&ctx, &g_act);
    for (i = 0u; i <= SM_MAX_RETRIES; i++)
    {
        step(&ctx, SM_EVT_TIMEOUT, 0u);     /* INITIALISING timeout */
        b_ok = (SM_STATE_ERROR == ctx.state);
        step(&ctx, SM_EVT_TIMEOUT, 0u);     /* retry delay          */
    }
    TEST_CHECK("S1-U09", b_ok && (SM_STATE_ERROR == ctx.state) &&
               (SM_MAX_RETRIES == ctx.retries) &&
               (SM_TIMER_KEEP == g_act.timer),
               "ERROR retries initialisation then stays in ERROR");

    /* S1-U10: obstacle avoidance sequence */
    make_navigating(&ctx);
    step(&ctx, SM_EVT_OBSTACLE, 0u);
    b_ok = (SM_STATE_OBSTACLE_AVOIDANCE == ctx.state) &&
           (SM_NAV_CMD_HOLD == SM_NAV_CMD_CODE(g_act.nav_cmd));
    step(&ctx, SM_EVT_PROFILE_READY, SM_AVOID_LEFT);
    b_ok = b_ok && (SM_NAV_BYPASS == ctx.nav_mode) &&
           (SM_NAV_CMD_PACK(SM_NAV_CMD_AVOID, SM_AVOID_LEFT) ==
            g_act.nav_cmd);
    step(&ctx, SM_EVT_BYPASS_DONE, 0u);
    b_ok = b_ok && (SM_NAV_RECOVERY == ctx.nav_mode);
    step(&ctx, SM_EVT_LINE_REACQUIRED, 0u);
    TEST_CHECK("S1-U10", b_ok && (SM_STATE_NAVIGATING == ctx.state) &&
               (SM_NAV_TRACKING == ctx.nav_mode),
               "obstacle: hold, bypass, recover, back to NAVIGATING");

    /* S1-U11: barcode action is executed at the next junction */
    make_navigating(&ctx);
    step(&ctx, SM_EVT_BARCODE,
         SM_BARCODE_ARG(SM_BARCODE_SYMBOL('B'), SM_ACTION_RIGHT));
    b_ok = (0u == g_act.b_nav_cmd);
    step(&ctx, SM_EVT_JUNCTION, 0u);
    b_ok = b_ok && (SM_NAV_MANOEUVRE == ctx.nav_mode) &&
           (SM_NAV_CMD_PACK(SM_NAV_CMD_TURN, SM_ACTION_RIGHT) ==
            g_act.nav_cmd);
    step(&ctx, SM_EVT_TURN_DONE, 0u);
    TEST_CHECK("S1-U11", b_ok && (SM_NAV_TRACKING == ctx.nav_mode),
               "barcode turn waits for the junction");

    /* S1-U12: bindings are only accepted in STANDBY */
    step(&ctx, SM_EVT_CMD_BIND, 0u);
    b_ok = (0u == g_act.b_commit_bindings) && (0u == g_act.b_ack_ok);
    make_standby(&ctx);
    step(&ctx, SM_EVT_CMD_BIND, 0u);
    TEST_CHECK("S1-U12", b_ok && (0u != g_act.b_commit_bindings),
               "barcode bindings commit only in STANDBY");

    /* S1-U13: the live State Machine task is running */
    TEST_CHECK("S1-U13", (SM_STATUS_STATE(sm_status_word()) <
                          SM_STATE_COUNT) &&
               (E_PAR == sm_post_event(SM_EVT_TIMEOUT, 0u)),
               "State Machine task publishes a valid status");

    /* S1-U14: default bindings A..D, other symbols unbound */
    b_ok = sm_barcode_lookup(SM_BARCODE_SYMBOL('A'), &action) &&
           (SM_ACTION_LEFT == action);
    b_ok = b_ok && sm_barcode_lookup(SM_BARCODE_SYMBOL('D'), &action) &&
           (SM_ACTION_UTURN == action);
    TEST_CHECK("S1-U14", b_ok &&
               !sm_barcode_lookup(SM_BARCODE_SYMBOL('E'), &action),
               "default barcode bindings A-D");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
