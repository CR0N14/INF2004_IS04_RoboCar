/*
 *  net_codec.h
 *  Encoding of telemetry, heartbeat and event frames, and decoding of
 *  commands, for MQTT.  Pure functions: no kernel calls, no I/O.
 *  The wire format is documented in docs/MQTT.md.
 *
 *  Private to subsystem 1.
 */

#ifndef NET_CODEC_H
#define NET_CODEC_H

#include "net_types.h"
#include "bitpack.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

/* Which subsystem blocks a telemetry frame carries */
#define NET_HAS_S2      0x8u
#define NET_HAS_S3      0x4u
#define NET_HAS_S4      0x2u
#define NET_HAS_S5      0x1u
#define NET_HAS_BITS    4u

/* Everything a telemetry frame is built from */
typedef struct
{
    s2_tlm_t    s2;
    s3_tlm_t    s3;
    s4_tlm_t    s4;
    s5_tlm_t    s5;
    uint32_t    sm_status;      /* sm_status_word()                      */
    uint8_t     present;        /* NET_HAS_* of valid blocks             */
    uint8_t     seq;            /* frame counter, wraps                  */
} net_tlm_in_t;

/* One queued event */
typedef struct
{
    uint32_t    payload;
    uint16_t    time_ds;        /* uptime in 0.1 s, wraps                */
    uint8_t     type;           /* net_evt_t                             */
} net_evt_rec_t;

/* A validated command */
typedef struct
{
    uint64_t    actions;        /* BIND: 2 bits per symbol               */
    uint32_t    bound_mask;     /* BIND: 1 bit per symbol                */
    uint8_t     op;             /* net_op_t                              */
} net_cmd_t;

#define NET_TLM_MAX_BYTES   32u
#define NET_HB_BYTES        6u

/* Payload width of each event type on the wire (bits) */
uint8_t     net_codec_evt_bits(net_evt_t type);

/* Telemetry frame; returns its length in bytes */
uint16_t    net_codec_tlm(net_tlm_in_t const *p_in, uint8_t *p_buf,
                          uint16_t cap);

/* Heartbeat; returns its length in bytes */
uint16_t    net_codec_hb(uint32_t uptime_s, uint32_t sm_status,
                         uint8_t dropped, uint8_t *p_buf, uint16_t cap);

/* Append one event to a batch; false (and nothing written) if it does
   not fit */
bool        net_codec_evt_put(bitpack_t *p_bp, net_evt_rec_t const *p_evt);

/* Validate and decode a command; false if it is malformed.  p_cmd->op
   is set to the opcode whenever the first byte could be read, so the
   rejection can still be acknowledged. */
bool        net_codec_cmd(uint8_t const *p_buf, uint16_t len,
                          net_cmd_t *p_cmd);

#endif /* NET_CODEC_H */
