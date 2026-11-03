/*
 *  sm_logic.h
 *  System statechart as a pure function: (context, event) -> (context,
 *  actions).  No kernel calls, so the transitions can be unit-tested on
 *  the target and on a PC.  The State Machine task (state_machine.c)
 *  feeds it events and carries out the actions it returns.
 *
 *  Private to subsystem 1.
 */

#ifndef SM_LOGIC_H
#define SM_LOGIC_H

#include "sm_types.h"

/* Whole statechart context: 30 bits, one 32-bit word */
typedef struct
{
    unsigned int state      : 3;    /* sm_state_t                        */
    unsigned int nav_mode   : 3;    /* sm_nav_mode_t                     */
    unsigned int b_pending  : 1;    /* a barcode action waits for a      */
    unsigned int pending    : 2;    /*   junction (sm_action_t)          */
    unsigned int retries    : 2;    /* ERROR -> INITIALISING retries     */
    unsigned int ready_mask : 5;    /* SM_READY_BIT() of ready subsystems */
    unsigned int b_net_up   : 1;    /* MQTT connected                    */
    unsigned int fault_mask : 13;   /* SM_FAULT_BIT() of active faults   */
} sm_ctx_t;

/* What the timer should do after this step */
typedef enum
{
    SM_TIMER_KEEP = 0,      /* leave it as it is                         */
    SM_TIMER_START_INIT,    /* start the INITIALISING timeout            */
    SM_TIMER_START_RETRY,   /* start the ERROR retry delay               */
    SM_TIMER_CANCEL
} sm_timer_req_t;

/* Command acknowledgement */
typedef enum
{
    SM_ACK_START = 0,
    SM_ACK_STOP,
    SM_ACK_BIND
} sm_ack_cmd_t;

/* Actions produced by one step: 21 bits, one 32-bit word */
typedef struct
{
    unsigned int b_state_changed    : 1;
    unsigned int prev_state         : 3;    /* sm_state_t                */
    unsigned int b_nav_cmd          : 1;
    unsigned int nav_cmd            : 6;    /* SM_NAV_CMD_PACK() byte    */
    unsigned int b_ack              : 1;
    unsigned int ack_cmd            : 2;    /* sm_ack_cmd_t              */
    unsigned int b_ack_ok           : 1;
    unsigned int timer              : 2;    /* sm_timer_req_t            */
    unsigned int b_commit_bindings  : 1;
    unsigned int b_ignored          : 1;    /* event had no effect       */
} sm_actions_t;

/* Power-on: enter INITIALISING */
void sm_logic_init(sm_ctx_t *p_ctx, sm_actions_t *p_act);

/* Handle one control event */
void sm_logic_step(sm_ctx_t *p_ctx, sm_evt_code_t code, uint16_t arg,
                   sm_actions_t *p_act);

/* Mode flags (SM_FLG_*) that belong to a state */
uint32_t sm_logic_mode_flags(sm_state_t state);

/* Status word (see sm_types.h) for a context */
uint32_t sm_logic_status_word(sm_ctx_t const *p_ctx);

#endif /* SM_LOGIC_H */
