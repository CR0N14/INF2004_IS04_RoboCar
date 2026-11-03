/*
 *  net_types.h
 *  Networking: telemetry event types and command opcodes shared with
 *  every subsystem.  No kernel dependency.  Tasks include networking.h.
 *
 *  The MQTT wire format built from these is documented in docs/MQTT.md.
 */

#ifndef NET_TYPES_H
#define NET_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "sm_types.h"

/* ---------------------------------------------------------------------
 *  Telemetry events (any task -> Networking -> <root>/evt)
 *
 *  One-off events that must not be lost.  Each has a payload of at most
 *  NET_EVT_PAYLOAD_BITS bits, laid out as listed (first field in the
 *  most significant bits).  Use the NET_EVT_*() helpers below to build
 *  a payload.
 * --------------------------------------------------------------------- */
typedef enum
{
    NET_EVT_NONE = 0,       /* end of a batch on the wire                */
    NET_EVT_STATE,          /* S1 [from:3][to:3][faults:13]              */
    NET_EVT_CMD_ACK,        /* S1 [opcode:3][accepted:1]                 */
    NET_EVT_RUN_SUMMARY,    /* S1 [max_peak_mm:7][distance_cm:14]        */
                            /*    [duration_s:11] -- built by Networking */
    NET_EVT_HUMP,           /* S4 [peak_mm:7]                            */
    NET_EVT_MOTION_CLASS,   /* S4 [class:3]  (s4_motion_class_t)         */
    NET_EVT_IMU_FAULT,      /* S4 -                                      */
    NET_EVT_BARCODE_OK,     /* S3 [symbol:5][action:2]                   */
    NET_EVT_BARCODE_REJECT, /* S3 [reason:3] (net_barcode_reject_t)      */
    NET_EVT_LINE_LOST,      /* S3 -                                      */
    NET_EVT_LINE_FOUND,     /* S3 -                                      */
    NET_EVT_IR_FAULT,       /* S3 -                                      */
    NET_EVT_OBSTACLE,       /* S5 [distance_cm:9]                        */
    NET_EVT_PROFILE,        /* S5 [rec:3][bearing_deg:8][width_cm:7]     */
                            /*    [closest_cm:9]                         */
    NET_EVT_AVOID_DONE,     /* S5 -  obstacle bypassed, line recovered   */
    NET_EVT_DROPPED,        /* S1 [count:8] events lost before this one  */
    NET_EVT_COUNT
} net_evt_t;

#define NET_EVT_TYPE_BITS       5u
#define NET_EVT_PAYLOAD_BITS    27u     /* type + payload = one 32-bit word */
#define NET_EVT_TIME_BITS       16u     /* wire timestamp, 0.1 s units    */

typedef enum
{
    NET_BARCODE_INVALID = 0,    /* pattern failed the encoding checks    */
    NET_BARCODE_INCOMPLETE,     /* capture timed out                     */
    NET_BARCODE_UNBOUND,        /* valid symbol with no binding          */
    NET_BARCODE_SAMPLE_GAP,     /* missing IR samples during capture     */
    NET_BARCODE_OVERFLOW        /* sample buffer overflow                */
} net_barcode_reject_t;

/* Payload helpers */
#define NET_FIELD(value, bits)  ((uint32_t)(value) & ((1u << (bits)) - 1u))

#define NET_EVT_STATE_PAYLOAD(from, to, faults) \
    ((NET_FIELD(from, 3u) << 16u) | (NET_FIELD(to, 3u) << 13u) | \
     NET_FIELD(faults, 13u))
#define NET_EVT_ACK_PAYLOAD(opcode, b_ok) \
    ((NET_FIELD(opcode, 3u) << 1u) | ((b_ok) ? 1u : 0u))
#define NET_EVT_HUMP_PAYLOAD(peak_mm)       NET_FIELD(peak_mm, 7u)
#define NET_EVT_CLASS_PAYLOAD(cls)          NET_FIELD(cls, 3u)
#define NET_EVT_BARCODE_OK_PAYLOAD(symbol, action) \
    ((NET_FIELD(symbol, 5u) << 2u) | NET_FIELD(action, 2u))
#define NET_EVT_REJECT_PAYLOAD(reason)      NET_FIELD(reason, 3u)
#define NET_EVT_OBSTACLE_PAYLOAD(dist_cm)   NET_FIELD(dist_cm, 9u)
#define NET_EVT_PROFILE_PAYLOAD(rec, bearing, width, closest) \
    ((NET_FIELD(rec, 3u) << 24u) | (NET_FIELD(bearing, 8u) << 16u) | \
     (NET_FIELD(width, 7u) << 9u) | NET_FIELD(closest, 9u))

/* ---------------------------------------------------------------------
 *  Commands (<root>/cmd -> Networking), first 3 bits of the payload
 * --------------------------------------------------------------------- */
typedef enum
{
    NET_OP_NONE = 0,
    NET_OP_START,           /* start a run (STANDBY only)                */
    NET_OP_STOP,            /* stop / park the car                       */
    NET_OP_BIND,            /* [count:5] then count x [symbol:5][act:2]  */
    NET_OP_PING             /* acknowledged at once, for latency checks  */
} net_op_t;

#define NET_OP_BITS             3u
#define NET_BIND_COUNT_BITS     5u

#endif /* NET_TYPES_H */
