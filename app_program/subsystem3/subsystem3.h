/*
 *  subsystem3.h
 *  Subsystem 3 -- Barcode Decoding and IR Line Following.  Student ID:
 *2303564.
 *
 * Public application interface; core processing remains RTOS-independent.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../subsystem1/sm_types.h"
#include "barcode_decoder.h"

#define S3_PRI_MAIN PRI_LINE_FOLLOWING /* app.h priority map */

/* ---------------------------------------------------------------------
 *  IF-07 telemetry snapshot, S3 -> S1 Networking
 *
 *  The latest values only.  s3_get_tlm() copies the snapshot out under
 *  this subsystem's own mutex and returns E_OK; Networking calls it every
 *  200 ms. The strong S3 getter overrides the existing weak fallback.
 *  Until configuration succeeds it returns E_NOSPT, omitting the block.
 *
 *  Each field holds only the bits its range needs (Change Requirement 1).
 *  Signed quantities are stored as magnitude + direction bit.  If a field
 *  or range changes, tell S1: subsystem1/net_codec.c packs these fields
 *  for MQTT and docs/MQTT.md documents them.
 * --------------------------------------------------------------------- */

typedef enum
{
    S3_LINE_SELFTEST = 0,
    S3_LINE_IDLE,
    S3_LINE_TRACKING,
    S3_LINE_SUSPENDED, /* manoeuvre or bypass: steering suspended   */
    S3_LINE_LOST,
    S3_LINE_REACQUIRING,
    S3_LINE_FAULT
} s3_line_state_t;

typedef enum
{
    S3_DECODER_IDLE = 0,
    S3_DECODER_SEARCHING,
    S3_DECODER_CAPTURING,
    S3_DECODER_VALIDATING,
    S3_DECODER_WAIT_CLEAR
} s3_decoder_state_t;

#define S3_IR_LEFT 0x4u /* ir_pattern bits: sensor sees black    */
#define S3_IR_CENTRE 0x2u
#define S3_IR_RIGHT 0x1u

/* 26 bits in 1 word */
typedef struct
{
    unsigned int line_state    : 3; /* s3_line_state_t               */
    unsigned int b_line_valid  : 1; /* line position is valid        */
    unsigned int line_pos      : 7; /* |offset|, 0..100 % half-width */
    unsigned int b_line_left   : 1; /* line is left of centre        */
    unsigned int ir_pattern    : 3; /* S3_IR_* bits                  */
    unsigned int decoder_state : 3; /* s3_decoder_state_t            */
    unsigned int b_barcode     : 1; /* a barcode was accepted        */
    unsigned int last_symbol   : 5; /* last barcode, 0..25 = 'A'..'Z' */
    unsigned int last_action   : 2; /* its sm_action_t               */
} s3_tlm_t;

/** Copy the existing IF-07 layout; E_NOSPT until calibrated configuration. */
ER s3_get_tlm (s3_tlm_t * p_tlm);
/** Configure after init and before start; no fabricated hardware defaults. */
ER subsystem3_configure (ir_config_t const *      p_line,
                         barcode_config_t const * p_barcode);
/** Controller-only update; context changes atomically invalidate both cores. */
ER subsystem3_set_context (ir_permission_t permission,
                           uint16_t        binding_version);
/** Line owner submits an acquired frame; events are returned for routing. */
ER subsystem3_line_step (ir_frame_t const * p_frame, uint16_t now_ms,
                         ir_result_t * p_result);
/** Decoder owner submits a copied frame and acquisition-associated Motion. */
ER subsystem3_barcode_step (ir_frame_t const * p_frame, barcode_motion_t motion,
                            uint16_t now_ms, barcode_result_t * p_result);
/** Motion reads an expiring target; mode/generation must also match at use. */
ER subsystem3_get_target (uint16_t now_ms, ir_result_t * p_result);

#endif /* SUBSYSTEM3_H */
/* End of file. */

