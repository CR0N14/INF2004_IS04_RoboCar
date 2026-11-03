/*
 *  state_machine.h
 *  State Machine task: public interface for every subsystem.
 *
 *  Include as "../subsystem1/state_machine.h".
 *
 *  How a subsystem talks to the State Machine (TEAM.md section 5):
 *
 *  IF-01  Control events, any task -> State Machine
 *         sm_post_event(SM_EVT_..., arg).  Never blocks; returns
 *         E_QOVR if the queue is full.  Codes and args: sm_types.h.
 *
 *  IF-02  Mode flags, State Machine -> all tasks
 *         Event flag flg_mode, bits SM_FLG_SENSORS_ENABLED,
 *         SM_FLG_MOTORS_ENABLED and SM_FLG_INITIALISING.  Wait with
 *         tk_wai_flg(flg_mode, SM_FLG_..., TWF_ORW, &ptn, tmo) or test
 *         with sm_mode_flags().  Never set or clear them yourself.
 *
 *  IF-03  Status word, State Machine -> all tasks
 *         sm_status_word() returns state, nav mode and fault mask in
 *         one consistent read (decode with SM_STATUS_*()).  It is
 *         updated BEFORE the mode flags change, so when Motion Control
 *         sees SM_FLG_MOTORS_ENABLED clear it can tell an ERROR (hard
 *         brake) from a stop command (coast to a stop) by the state.
 *
 *  IF-04  Navigation commands, State Machine -> Motion Control
 *         Message buffer mbf_nav_cmd, one byte per command,
 *         SM_NAV_CMD_PACK(cmd, arg).  Motion Control receives with
 *         TMO_POL in its control cycle.
 *
 *  IF-05  Barcode bindings, State Machine -> Barcode Decoder
 *         sm_barcode_lookup(symbol, &action) maps a decoded symbol to
 *         its action using the committed binding table.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "../app.h"
#include "sm_types.h"

/* Queue sizes */
#define SM_CTRL_QUEUE_LEN       16u     /* control events in flight     */
#define SM_NAV_QUEUE_LEN        4u      /* navigation commands          */

/* Kernel objects created by the State Machine */
IMPORT ID   flg_mode;       /* IF-02 */
IMPORT ID   mbf_nav_cmd;    /* IF-04 */

/* IF-01: post a control event.  Task context only; never blocks. */
ER          sm_post_event(sm_evt_code_t code, uint16_t arg);

/* IF-02: current mode flags (SM_FLG_*) */
uint32_t    sm_mode_flags(void);

/* IF-03: current status word (decode with SM_STATUS_*()) */
uint32_t    sm_status_word(void);

/* IF-05: action bound to a barcode symbol (0..25 = 'A'..'Z').
   Returns false if the symbol is not bound to any action. */
bool        sm_barcode_lookup(uint8_t symbol, sm_action_t *p_action);

/*
 * Stage a new binding table, to be committed by the State Machine when
 * it accepts SM_EVT_CMD_BIND (only in STANDBY).  Used by Networking.
 *   bound_mask  bit n set = symbol n is bound
 *   actions     2 bits per symbol: symbol n at bits 2n..2n+1
 */
ER          sm_stage_bindings(uint32_t bound_mask, uint64_t actions);

/* Committed binding table (for telemetry) */
void        sm_get_bindings(uint32_t *p_bound_mask, uint64_t *p_actions);

/* Created / started by subsystem1.c */
ER          sm_init(void);
ER          sm_start(void);

#endif /* STATE_MACHINE_H */
