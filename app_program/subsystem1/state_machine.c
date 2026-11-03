/*
 *  state_machine.c
 *  State Machine task: owns the system state.
 *
 *  The task blocks on the control-event message buffer (IF-01).  Each
 *  event is passed to the statechart in sm_logic.c, and the task then
 *  carries out the actions it returns, in this order:
 *    1. publish the new status word (IF-03)
 *    2. update the mode flags (IF-02)
 *    3. send a navigation command to Motion Control (IF-04)
 *    4. commit staged barcode bindings
 *    5. report the state change / command result as telemetry events
 *  The INITIALISING timeout and the ERROR retry delay are the receive
 *  timeout of the same message buffer, so no separate timer object is
 *  needed.
 */

#include "state_machine.h"
#include "sm_logic.h"
#include "networking.h"

#define SM_EVT_MSG_SIZE     ((INT)sizeof(uint16_t))
#define SM_NAV_MSG_SIZE     ((INT)sizeof(uint8_t))

/* Each message buffer entry costs a 4-byte kernel header plus the
   message rounded up to 4 bytes. */
#define SM_MBF_SLOT_BYTES   8u

/* Default barcode bindings (design assumption 1): A left, B right,
   C straight, D U-turn, as in the project write-up. */
#define SM_DEFAULT_BOUND    0x0000000Fu
#define SM_DEFAULT_ACTIONS  (((uint64_t)SM_ACTION_LEFT     << 0u) | \
                             ((uint64_t)SM_ACTION_RIGHT    << 2u) | \
                             ((uint64_t)SM_ACTION_STRAIGHT << 4u) | \
                             ((uint64_t)SM_ACTION_UTURN    << 6u))
#define SM_ALL_SYMBOLS_MASK ((1u << SM_BARCODE_SYMBOLS) - 1u)

/* Command opcode reported in an acknowledgement, per sm_ack_cmd_t */
static uint8_t const g_ack_ops[] = { NET_OP_START, NET_OP_STOP, NET_OP_BIND };

EXPORT ID   flg_mode;
EXPORT ID   mbf_nav_cmd;

static ID   g_mbf_ctrl;
static ID   g_mtx_bindings;

/* IF-03: written only by the State Machine task, with one store */
static volatile uint32_t g_status_word;

/* Barcode binding tables, protected by g_mtx_bindings */
typedef struct
{
    uint64_t actions;       /* 2 bits per symbol                        */
    uint32_t bound_mask;    /* 1 bit per symbol                         */
} sm_bindings_t;

static sm_bindings_t g_bindings_active;
static sm_bindings_t g_bindings_staged;

/* Timer: deadline in ms (low 32 bits of the system time) */
static uint32_t g_deadline_ms;
static bool     g_b_timer_running;

static uint32_t
now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);
    return (uint32_t)tim.lo;
}

static TMO
receive_timeout(void)
{
    TMO tmo = TMO_FEVR;

    if (g_b_timer_running)
    {
        int32_t const left = (int32_t)(g_deadline_ms - now_ms());

        tmo = (left > 0) ? (TMO)left : TMO_POL;
    }
    return tmo;
}

static void
apply_timer(sm_timer_req_t req)
{
    switch (req)
    {
        case SM_TIMER_START_INIT:
            g_deadline_ms = now_ms() + SM_INIT_TIMEOUT_MS;
            g_b_timer_running = true;
            break;

        case SM_TIMER_START_RETRY:
            g_deadline_ms = now_ms() + SM_ERROR_RETRY_MS;
            g_b_timer_running = true;
            break;

        case SM_TIMER_CANCEL:
            g_b_timer_running = false;
            break;

        case SM_TIMER_KEEP:
        default:
            break;
    }
}

static void
apply_mode_flags(sm_state_t state)
{
    uint32_t const flags = sm_logic_mode_flags(state);

    /* tk_clr_flg() ANDs the pattern in: keep only the new bits, then
       set them.  Clearing first means MOTORS_ENABLED is never seen set
       together with a state that forbids it. */
    (void)tk_clr_flg(flg_mode, (UINT)flags);
    (void)tk_set_flg(flg_mode, (UINT)flags);
}

static void
commit_bindings(void)
{
    if (E_OK == tk_loc_mtx(g_mtx_bindings, TMO_FEVR))
    {
        g_bindings_active = g_bindings_staged;
        (void)tk_unl_mtx(g_mtx_bindings);
    }
}

static void
log_step(sm_evt_code_t code, sm_actions_t const *p_act, sm_ctx_t const *p_ctx)
{
    static char const * const ack_names[] = { "start", "stop", "bind" };

    if (0u != p_act->b_state_changed)
    {
        tm_printf((UB *)"sm: %s -> %s (evt %d, faults 0x%04x)\n",
                  sm_state_name((sm_state_t)p_act->prev_state),
                  sm_state_name((sm_state_t)p_ctx->state), (int)code,
                  (unsigned int)p_ctx->fault_mask);
    }
    if (0u != p_act->b_ack)
    {
        tm_printf((UB *)"sm: command %s %s\n", ack_names[p_act->ack_cmd],
                  (0u != p_act->b_ack_ok) ? "accepted" : "rejected");
    }
}

static void
apply_actions(sm_evt_code_t code, sm_ctx_t const *p_ctx,
              sm_actions_t const *p_act)
{
    /* 1. Status word first: readers of the mode flags rely on it */
    g_status_word = sm_logic_status_word(p_ctx);

    /* 2. Mode flags */
    if (0u != p_act->b_state_changed)
    {
        apply_mode_flags((sm_state_t)p_ctx->state);
    }

    /* 3. Navigation command */
    if (0u != p_act->b_nav_cmd)
    {
        uint8_t const msg = (uint8_t)p_act->nav_cmd;

        if (E_OK != tk_snd_mbf(mbf_nav_cmd, &msg, SM_NAV_MSG_SIZE, TMO_POL))
        {
            tm_printf((UB *)"sm: nav command queue full\n");
        }
    }

    /* 4. Bindings */
    if (0u != p_act->b_commit_bindings)
    {
        commit_bindings();
    }

    /* 5. Telemetry: state change (with the fault code, FR5) and the
          result of a command (FR4) */
    if (0u != p_act->b_state_changed)
    {
        (void)net_post_event(NET_EVT_STATE,
                             NET_EVT_STATE_PAYLOAD(p_act->prev_state,
                                                   p_ctx->state,
                                                   p_ctx->fault_mask));
    }
    if (0u != p_act->b_ack)
    {
        (void)net_post_event(NET_EVT_CMD_ACK,
                             NET_EVT_ACK_PAYLOAD(g_ack_ops[p_act->ack_cmd],
                                                 0u != p_act->b_ack_ok));
    }

    apply_timer((sm_timer_req_t)p_act->timer);
    log_step(code, p_act, p_ctx);
}

static void
sm_task(INT stacd, void *exinf)
{
    sm_ctx_t        ctx;
    sm_actions_t    act;
    uint16_t        msg;
    sm_evt_code_t   code;

    (void)stacd;
    (void)exinf;

    sm_logic_init(&ctx, &act);
    apply_actions(SM_EVT_NONE, &ctx, &act);

    for (;;)
    {
        /* An INITIALISING state with nothing left to wait for moves on
           without needing another event. */
        if (SM_STATE_INITIALISING == ctx.state)
        {
            sm_logic_step(&ctx, SM_EVT_CHECK, 0u, &act);
            if (0u != act.b_state_changed)
            {
                apply_actions(SM_EVT_CHECK, &ctx, &act);
            }
        }

        INT const len = tk_rcv_mbf(g_mbf_ctrl, &msg, receive_timeout());

        if (SM_EVT_MSG_SIZE == len)
        {
            code = SM_EVT_CODE(msg);
            sm_logic_step(&ctx, code, SM_EVT_ARG(msg), &act);
        }
        else if (E_TMOUT == len)
        {
            g_b_timer_running = false;
            code = SM_EVT_TIMEOUT;
            sm_logic_step(&ctx, code, 0u, &act);
        }
        else
        {
            continue;
        }
        apply_actions(code, &ctx, &act);
    }
}

EXPORT ER
sm_post_event(sm_evt_code_t code, uint16_t arg)
{
    uint16_t const msg = SM_EVT_PACK(code, arg);

    if ((SM_EVT_NONE == code) || (code >= SM_EVT_TIMEOUT))
    {
        return E_PAR;
    }
    return tk_snd_mbf(g_mbf_ctrl, &msg, SM_EVT_MSG_SIZE, TMO_POL);
}

EXPORT uint32_t
sm_mode_flags(void)
{
    T_RFLG rflg;

    if (E_OK != tk_ref_flg(flg_mode, &rflg))
    {
        return 0u;
    }
    return (uint32_t)rflg.flgptn & SM_FLG_ALL;
}

EXPORT uint32_t
sm_status_word(void)
{
    return g_status_word;
}

EXPORT bool
sm_barcode_lookup(uint8_t symbol, sm_action_t *p_action)
{
    bool b_bound = false;

    if ((symbol < SM_BARCODE_SYMBOLS) && (NULL != p_action) &&
        (E_OK == tk_loc_mtx(g_mtx_bindings, TMO_FEVR)))
    {
        if (0u != (g_bindings_active.bound_mask & (1u << symbol)))
        {
            *p_action = (sm_action_t)((g_bindings_active.actions >>
                        (symbol * SM_ACTION_BITS)) &
                        SM_FIELD_MASK(SM_ACTION_BITS));
            b_bound = true;
        }
        (void)tk_unl_mtx(g_mtx_bindings);
    }
    return b_bound;
}

EXPORT ER
sm_stage_bindings(uint32_t bound_mask, uint64_t actions)
{
    ER er;

    if (0u != (bound_mask & ~SM_ALL_SYMBOLS_MASK))
    {
        return E_PAR;
    }
    er = tk_loc_mtx(g_mtx_bindings, TMO_FEVR);
    if (E_OK == er)
    {
        g_bindings_staged.bound_mask = bound_mask;
        g_bindings_staged.actions = actions;
        (void)tk_unl_mtx(g_mtx_bindings);
    }
    return er;
}

EXPORT void
sm_get_bindings(uint32_t *p_bound_mask, uint64_t *p_actions)
{
    if (E_OK == tk_loc_mtx(g_mtx_bindings, TMO_FEVR))
    {
        *p_bound_mask = g_bindings_active.bound_mask;
        *p_actions = g_bindings_active.actions;
        (void)tk_unl_mtx(g_mtx_bindings);
    }
}

EXPORT ER
sm_init(void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};
    T_CMTX cmtx = {0};

    cflg.flgatr = TA_TFIFO | TA_WMUL;   /* every subsystem may wait */
    cflg.iflgptn = 0u;
    flg_mode = tk_cre_flg(&cflg);
    if (flg_mode < E_OK)
    {
        return flg_mode;
    }

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)(SM_CTRL_QUEUE_LEN * SM_MBF_SLOT_BYTES);
    cmbf.maxmsz = SM_EVT_MSG_SIZE;
    g_mbf_ctrl = tk_cre_mbf(&cmbf);
    if (g_mbf_ctrl < E_OK)
    {
        return g_mbf_ctrl;
    }

    cmbf.bufsz = (SZ)(SM_NAV_QUEUE_LEN * SM_MBF_SLOT_BYTES);
    cmbf.maxmsz = SM_NAV_MSG_SIZE;
    mbf_nav_cmd = tk_cre_mbf(&cmbf);
    if (mbf_nav_cmd < E_OK)
    {
        return mbf_nav_cmd;
    }

    cmtx.mtxatr = TA_INHERIT;
    g_mtx_bindings = tk_cre_mtx(&cmtx);
    if (g_mtx_bindings < E_OK)
    {
        return g_mtx_bindings;
    }

    g_bindings_active.bound_mask = SM_DEFAULT_BOUND;
    g_bindings_active.actions = SM_DEFAULT_ACTIONS;
    g_bindings_staged = g_bindings_active;
    g_status_word = ((uint32_t)SM_STATE_INITIALISING << SM_STATUS_STATE_POS);
    return E_OK;
}

EXPORT ER
sm_start(void)
{
    ID const tid = app_start_task(sm_task, PRI_STATE_MACHINE, 0u,
                                  "sm_task");

    return (tid < E_OK) ? tid : E_OK;
}
