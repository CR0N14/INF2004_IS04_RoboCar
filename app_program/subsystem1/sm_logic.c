/*
 *  sm_logic.c
 *  System-level statechart (design review section 5).
 *
 *  INITIALISING --all ready + MQTT up--> STANDBY --start--> NAVIGATING
 *  NAVIGATING --obstacle--> OBSTACLE_AVOIDANCE --line reacquired-->
 *  NAVIGATING.  stop: NAVIGATING / OBSTACLE_AVOIDANCE -> STANDBY.
 *  A fatal fault in any state, a failed self-test or an INITIALISING
 *  timeout -> ERROR.  ERROR retries INITIALISING up to SM_MAX_RETRIES
 *  times, SM_ERROR_RETRY_MS apart, then stays in ERROR.
 *
 *  Pure logic: no kernel calls.  See sm_logic.h.
 */

#include <string.h>
#include "sm_logic.h"

static char const * const g_state_names[SM_STATE_COUNT] =
{
    "INITIALISING", "STANDBY", "NAVIGATING", "OBSTACLE_AVOIDANCE", "ERROR"
};

const char *
sm_state_name(sm_state_t state)
{
    return (state < SM_STATE_COUNT) ? g_state_names[state] : "?";
}

static void
clear_actions(sm_actions_t *p_act)
{
    (void)memset(p_act, 0, sizeof(*p_act));
}

static void
send_nav_cmd(sm_actions_t *p_act, sm_nav_cmd_t cmd, uint8_t arg)
{
    p_act->b_nav_cmd = 1u;
    p_act->nav_cmd = SM_NAV_CMD_PACK(cmd, arg);
}

static void
ack(sm_actions_t *p_act, sm_ack_cmd_t cmd, bool b_ok)
{
    p_act->b_ack = 1u;
    p_act->ack_cmd = (unsigned int)cmd;
    p_act->b_ack_ok = b_ok ? 1u : 0u;
}

/* Run the exit/entry actions of a transition to new_state */
static void
enter_state(sm_ctx_t *p_ctx, sm_state_t new_state, sm_actions_t *p_act)
{
    sm_state_t const old_state = (sm_state_t)p_ctx->state;

    if (0u == p_act->b_state_changed)
    {
        p_act->b_state_changed = 1u;
        p_act->prev_state = (unsigned int)old_state;
    }
    p_ctx->state = (unsigned int)new_state;

    switch (new_state)
    {
        case SM_STATE_INITIALISING:
            /* Every subsystem re-runs its self-test and reports again.
               Faults from the previous attempt were already published
               on entry to ERROR; keep only the live link status. */
            p_ctx->ready_mask = 0u;
            p_ctx->fault_mask &= SM_FAULT_BIT(SM_FAULT_WIFI_LOSS);
            p_ctx->nav_mode = SM_NAV_DISABLED;
            p_ctx->b_pending = 0u;
            p_act->timer = SM_TIMER_START_INIT;
            break;

        case SM_STATE_STANDBY:
            p_ctx->nav_mode = SM_NAV_DISABLED;
            p_ctx->b_pending = 0u;
            if (SM_STATE_INITIALISING == old_state)
            {
                p_ctx->retries = 0u;    /* initialisation succeeded */
            }
            p_act->timer = SM_TIMER_CANCEL;
            break;

        case SM_STATE_NAVIGATING:
            if (SM_STATE_STANDBY == old_state)
            {
                p_ctx->b_pending = 0u;  /* new run */
            }
            p_ctx->nav_mode = SM_NAV_TRACKING;
            p_act->timer = SM_TIMER_CANCEL;
            break;

        case SM_STATE_OBSTACLE_AVOIDANCE:
            /* Stop in front of the obstacle while it is profiled */
            p_ctx->nav_mode = SM_NAV_HOLD;
            send_nav_cmd(p_act, SM_NAV_CMD_HOLD, 0u);
            break;

        case SM_STATE_ERROR:
        default:
            p_ctx->nav_mode = SM_NAV_DISABLED;
            p_ctx->b_pending = 0u;
            p_act->timer = (p_ctx->retries < SM_MAX_RETRIES)
                           ? SM_TIMER_START_RETRY : SM_TIMER_CANCEL;
            break;
    }
}

static bool
is_operational(sm_state_t state)
{
    return (SM_STATE_STANDBY == state) || (SM_STATE_NAVIGATING == state) ||
           (SM_STATE_OBSTACLE_AVOIDANCE == state);
}

static bool
is_running(sm_state_t state)
{
    return (SM_STATE_NAVIGATING == state) ||
           (SM_STATE_OBSTACLE_AVOIDANCE == state);
}

/* INITIALISING -> STANDBY once every required subsystem is ready */
static void
check_init_done(sm_ctx_t *p_ctx, sm_actions_t *p_act)
{
    uint8_t ready = (uint8_t)p_ctx->ready_mask;

    if (0u != p_ctx->b_net_up)
    {
        ready |= SM_READY_BIT(SM_SUBSYS_NET);
    }
    if ((SM_STATE_INITIALISING == p_ctx->state) &&
        ((ready & SM_REQUIRED_READY_MASK) == SM_REQUIRED_READY_MASK))
    {
        enter_state(p_ctx, SM_STATE_STANDBY, p_act);
    }
}

static void
raise_fault(sm_ctx_t *p_ctx, sm_fault_t fault, sm_actions_t *p_act)
{
    sm_state_t const state = (sm_state_t)p_ctx->state;
    uint16_t const bit = SM_FAULT_BIT(fault);

    p_ctx->fault_mask |= bit;

    if ((0u == (bit & SM_FAULT_NON_FATAL_MASK)) &&
        (SM_STATE_ERROR != state))
    {
        enter_state(p_ctx, SM_STATE_ERROR, p_act);
    }
}

static void
handle_navigating(sm_ctx_t *p_ctx, sm_evt_code_t code, uint16_t arg,
                  sm_actions_t *p_act)
{
    switch (code)
    {
        case SM_EVT_OBSTACLE:
            enter_state(p_ctx, SM_STATE_OBSTACLE_AVOIDANCE, p_act);
            break;

        case SM_EVT_BARCODE:
        {
            sm_action_t const action = SM_BARCODE_ARG_ACTION(arg);

            /* Every demo movement waits for the next cross-junction. */
            p_ctx->pending_symbol = SM_BARCODE_ARG_SYMBOL(arg);
            p_ctx->b_pending = 1u;
            p_ctx->pending = (unsigned int)action;
            break;
        }

        case SM_EVT_JUNCTION:
            if ((0u != p_ctx->b_pending) &&
                (SM_NAV_TRACKING == p_ctx->nav_mode))
            {
                p_ctx->b_pending = 0u;
                p_ctx->nav_mode = SM_NAV_MANOEUVRE;
                send_nav_cmd(p_act, SM_NAV_CMD_TURN,
                             (uint8_t)p_ctx->pending);
            }
            else
            {
                p_act->b_ignored = 1u;  /* no command: go straight on */
            }
            break;

        case SM_EVT_TURN_DONE:
            p_ctx->nav_mode = SM_NAV_TRACKING;
            break;

        case SM_EVT_LINE_LOST:
            if (SM_NAV_TRACKING == p_ctx->nav_mode)
            {
                p_ctx->nav_mode = SM_NAV_RECOVERY;  /* authorise search */
            }
            else
            {
                p_act->b_ignored = 1u;  /* expected during a manoeuvre */
            }
            break;

        case SM_EVT_LINE_REACQUIRED:
            if (SM_NAV_RECOVERY == p_ctx->nav_mode)
            {
                p_ctx->nav_mode = SM_NAV_TRACKING;
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;

        default:
            p_act->b_ignored = 1u;
            break;
    }
}

static void
handle_avoidance(sm_ctx_t *p_ctx, sm_evt_code_t code, uint16_t arg,
                 sm_actions_t *p_act)
{
    switch (code)
    {
        case SM_EVT_PROFILE_READY:
        {
            sm_avoid_t const rec = (sm_avoid_t)(arg &
                                   SM_FIELD_MASK(SM_AVOID_BITS));

            if (SM_AVOID_CONTINUE == rec)
            {
                enter_state(p_ctx, SM_STATE_NAVIGATING, p_act);
            }
            else if (SM_AVOID_STOP == rec)
            {
                p_ctx->nav_mode = SM_NAV_HOLD;  /* wait for a re-scan */
                send_nav_cmd(p_act, SM_NAV_CMD_HOLD, 0u);
            }
            else if (rec < SM_AVOID_COUNT)
            {
                p_ctx->nav_mode = SM_NAV_BYPASS;
                send_nav_cmd(p_act, SM_NAV_CMD_AVOID, (uint8_t)rec);
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;
        }

        case SM_EVT_BYPASS_DONE:
            p_ctx->nav_mode = SM_NAV_RECOVERY;
            break;

        case SM_EVT_LINE_REACQUIRED:
            if ((SM_NAV_BYPASS == p_ctx->nav_mode) ||
                (SM_NAV_RECOVERY == p_ctx->nav_mode))
            {
                enter_state(p_ctx, SM_STATE_NAVIGATING, p_act);
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;

        default:
            p_act->b_ignored = 1u;
            break;
    }
}

void
sm_logic_init(sm_ctx_t *p_ctx, sm_actions_t *p_act)
{
    (void)memset(p_ctx, 0, sizeof(*p_ctx));
    clear_actions(p_act);
    p_ctx->state = SM_STATE_INITIALISING;
    enter_state(p_ctx, SM_STATE_INITIALISING, p_act);
}

void
sm_logic_step(sm_ctx_t *p_ctx, sm_evt_code_t code, uint16_t arg,
              sm_actions_t *p_act)
{
    sm_state_t const state = (sm_state_t)p_ctx->state;

    clear_actions(p_act);

    switch (code)
    {
        /* ---- Commands from the base station ---- */
        case SM_EVT_CMD_START:
            if (SM_STATE_STANDBY == state)
            {
                enter_state(p_ctx, SM_STATE_NAVIGATING, p_act);
                ack(p_act, SM_ACK_START, true);
            }
            else
            {
                ack(p_act, SM_ACK_START, false);
            }
            break;

        case SM_EVT_CMD_STOP:
            if (is_running(state))
            {
                enter_state(p_ctx, SM_STATE_STANDBY, p_act);
            }
            ack(p_act, SM_ACK_STOP, is_operational(state));
            break;

        case SM_EVT_CMD_BIND:
            if (SM_STATE_STANDBY == state)
            {
                p_act->b_commit_bindings = 1u;
                ack(p_act, SM_ACK_BIND, true);
            }
            else
            {
                ack(p_act, SM_ACK_BIND, false);
            }
            break;

        /* ---- Link status: never a reason to stop (FR6) ---- */
        case SM_EVT_NET_UP:
            p_ctx->b_net_up = 1u;
            p_ctx->fault_mask &= ~(unsigned int)SM_FAULT_BIT(SM_FAULT_WIFI_LOSS);
            check_init_done(p_ctx, p_act);
            break;

        case SM_EVT_NET_DOWN:
            p_ctx->b_net_up = 0u;
            p_ctx->fault_mask |= SM_FAULT_BIT(SM_FAULT_WIFI_LOSS);
            break;

        /* ---- Initialisation ---- */
        case SM_EVT_READY:
            if (arg < SM_SUBSYS_COUNT)
            {
                p_ctx->ready_mask |= SM_READY_BIT(arg);
                check_init_done(p_ctx, p_act);
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;

        case SM_EVT_SELFTEST_FAIL:
            raise_fault(p_ctx, SM_FAULT_SELF_TEST, p_act);
            break;

        case SM_EVT_CHECK:
            check_init_done(p_ctx, p_act);
            break;

        case SM_EVT_TIMEOUT:
            if (SM_STATE_INITIALISING == state)
            {
                raise_fault(p_ctx, SM_FAULT_INIT_TIMEOUT, p_act);
            }
            else if ((SM_STATE_ERROR == state) &&
                     (p_ctx->retries < SM_MAX_RETRIES))
            {
                p_ctx->retries++;
                enter_state(p_ctx, SM_STATE_INITIALISING, p_act);
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;

        /* ---- Faults ---- */
        case SM_EVT_FAULT:
            if (arg < SM_FAULT_COUNT)
            {
                raise_fault(p_ctx, (sm_fault_t)arg, p_act);
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;

        /* ---- Navigation events: only meaningful while running ---- */
        default:
            if (SM_STATE_NAVIGATING == state)
            {
                handle_navigating(p_ctx, code, arg, p_act);
            }
            else if (SM_STATE_OBSTACLE_AVOIDANCE == state)
            {
                handle_avoidance(p_ctx, code, arg, p_act);
            }
            else
            {
                p_act->b_ignored = 1u;
            }
            break;
    }
}

uint32_t
sm_logic_mode_flags(sm_state_t state)
{
    uint32_t flags;

    switch (state)
    {
        case SM_STATE_INITIALISING:
            flags = SM_FLG_INITIALISING;
            break;

        case SM_STATE_STANDBY:
            flags = SM_FLG_SENSORS_ENABLED;
            break;

        case SM_STATE_NAVIGATING:
        case SM_STATE_OBSTACLE_AVOIDANCE:
            flags = SM_FLG_SENSORS_ENABLED | SM_FLG_MOTORS_ENABLED;
            break;

        case SM_STATE_ERROR:
        default:
            flags = 0u;
            break;
    }
    return flags;
}

uint32_t
sm_logic_status_word(sm_ctx_t const *p_ctx)
{
    return ((uint32_t)p_ctx->state << SM_STATUS_STATE_POS) |
           ((uint32_t)p_ctx->nav_mode << SM_STATUS_NAV_POS) |
           ((uint32_t)p_ctx->fault_mask << SM_STATUS_FAULT_POS) |
           ((uint32_t)p_ctx->retries << SM_STATUS_RETRY_POS);
}
