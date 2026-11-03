/*
 *  net_codec.c
 *  MQTT frame encoding / command decoding.  See net_codec.h and
 *  docs/MQTT.md.  Every field is written with exactly the bits its
 *  range needs, most significant bit first.
 */

#include "net_codec.h"

/* Telemetry header */
#define SEQ_BITS            8u

/* Heartbeat */
#define UPTIME_BITS         17u     /* seconds, wraps after ~36 h        */
#define DROPPED_BITS        8u

/* Field widths of the subsystem blocks (must match sN_tlm_t) */
#define S2_SPEED_BITS       10u
#define S2_STATE_BITS       3u
#define S2_ENC_BITS         16u
#define S2_DIST_BITS        14u

#define S3_STATE_BITS       3u
#define S3_POS_BITS         7u
#define S3_IR_BITS          3u
#define S3_DECODER_BITS     3u

#define S4_ANGLE_BITS       8u
#define S4_HEADING_BITS     9u
#define S4_STATUS_BITS      2u
#define S4_RATE_BITS        9u
#define S4_CLASS_BITS       3u
#define S4_PEAK_BITS        7u

#define S5_STATE_BITS       3u
#define S5_DIST_BITS        9u
#define S5_ANGLE_BITS       8u
#define S5_CLEAR_BITS       8u
#define S5_WIDTH_BITS       7u

#define FLAG_BITS           1u
#define BITS_PER_BYTE       8u

static uint8_t const g_evt_bits[NET_EVT_COUNT] =
{
    0u,     /* NET_EVT_NONE           */
    19u,    /* NET_EVT_STATE          */
    4u,     /* NET_EVT_CMD_ACK        */
    32u,    /* NET_EVT_RUN_SUMMARY    */
    7u,     /* NET_EVT_HUMP           */
    3u,     /* NET_EVT_MOTION_CLASS   */
    0u,     /* NET_EVT_IMU_FAULT      */
    7u,     /* NET_EVT_BARCODE_OK     */
    3u,     /* NET_EVT_BARCODE_REJECT */
    0u,     /* NET_EVT_LINE_LOST      */
    0u,     /* NET_EVT_LINE_FOUND     */
    0u,     /* NET_EVT_IR_FAULT       */
    9u,     /* NET_EVT_OBSTACLE       */
    27u,    /* NET_EVT_PROFILE        */
    0u,     /* NET_EVT_AVOID_DONE     */
    8u      /* NET_EVT_DROPPED        */
};

uint8_t
net_codec_evt_bits(net_evt_t type)
{
    return (type < NET_EVT_COUNT) ? g_evt_bits[type] : 0u;
}

static void
put_flag(bitpack_t *p_bp, unsigned int b_flag)
{
    bitpack_put(p_bp, (0u != b_flag) ? 1u : 0u, FLAG_BITS);
}

static void
put_s2(bitpack_t *p_bp, s2_tlm_t const *p)
{
    bitpack_put(p_bp, p->speed_l_mms, S2_SPEED_BITS);
    put_flag(p_bp, p->b_rev_l);
    bitpack_put(p_bp, p->speed_r_mms, S2_SPEED_BITS);
    put_flag(p_bp, p->b_rev_r);
    bitpack_put(p_bp, p->motion_state, S2_STATE_BITS);
    bitpack_put(p_bp, p->enc_l, S2_ENC_BITS);
    bitpack_put(p_bp, p->enc_r, S2_ENC_BITS);
    bitpack_put(p_bp, p->distance_cm, S2_DIST_BITS);
}

static void
put_s3(bitpack_t *p_bp, s3_tlm_t const *p)
{
    bitpack_put(p_bp, p->line_state, S3_STATE_BITS);
    put_flag(p_bp, p->b_line_valid);
    bitpack_put(p_bp, p->line_pos, S3_POS_BITS);
    put_flag(p_bp, p->b_line_left);
    bitpack_put(p_bp, p->ir_pattern, S3_IR_BITS);
    bitpack_put(p_bp, p->decoder_state, S3_DECODER_BITS);
    put_flag(p_bp, p->b_barcode);
    bitpack_put(p_bp, p->last_symbol, SM_BARCODE_SYMBOL_BITS);
    bitpack_put(p_bp, p->last_action, SM_ACTION_BITS);
}

static void
put_s4(bitpack_t *p_bp, s4_tlm_t const *p)
{
    bitpack_put(p_bp, p->pitch_half_deg, S4_ANGLE_BITS);
    put_flag(p_bp, p->b_pitch_down);
    bitpack_put(p_bp, p->roll_half_deg, S4_ANGLE_BITS);
    put_flag(p_bp, p->b_roll_left);
    bitpack_put(p_bp, p->heading_deg, S4_HEADING_BITS);
    bitpack_put(p_bp, p->imu_status, S4_STATUS_BITS);
    bitpack_put(p_bp, p->turn_rate_dps, S4_RATE_BITS);
    put_flag(p_bp, p->b_turn_left);
    bitpack_put(p_bp, p->motion_class, S4_CLASS_BITS);
    bitpack_put(p_bp, p->last_peak_mm, S4_PEAK_BITS);
    bitpack_put(p_bp, p->max_peak_mm, S4_PEAK_BITS);
}

static void
put_s5(bitpack_t *p_bp, s5_tlm_t const *p)
{
    bitpack_put(p_bp, p->scan_state, S5_STATE_BITS);
    bitpack_put(p_bp, p->distance_cm, S5_DIST_BITS);
    bitpack_put(p_bp, p->servo_deg, S5_ANGLE_BITS);
    bitpack_put(p_bp, p->recommendation, SM_AVOID_BITS);
    put_flag(p_bp, p->b_profile);
    bitpack_put(p_bp, p->left_clear_cm, S5_CLEAR_BITS);
    bitpack_put(p_bp, p->bearing_deg, S5_ANGLE_BITS);
    bitpack_put(p_bp, p->width_cm, S5_WIDTH_BITS);
    bitpack_put(p_bp, p->closest_cm, S5_DIST_BITS);
    bitpack_put(p_bp, p->right_clear_cm, S5_CLEAR_BITS);
}

static void
put_sm_status(bitpack_t *p_bp, uint32_t status)
{
    bitpack_put(p_bp, SM_STATUS_STATE(status), SM_STATE_BITS);
    bitpack_put(p_bp, SM_STATUS_NAV(status), SM_NAV_BITS);
    bitpack_put(p_bp, SM_STATUS_FAULTS(status), SM_FAULT_BITS);
}

uint16_t
net_codec_tlm(net_tlm_in_t const *p_in, uint8_t *p_buf, uint16_t cap)
{
    bitpack_t bp;

    bitpack_init(&bp, p_buf, cap);
    bitpack_put(&bp, p_in->seq, SEQ_BITS);
    put_sm_status(&bp, p_in->sm_status);
    bitpack_put(&bp, p_in->present, NET_HAS_BITS);

    if (0u != (p_in->present & NET_HAS_S2))
    {
        put_s2(&bp, &p_in->s2);
    }
    if (0u != (p_in->present & NET_HAS_S3))
    {
        put_s3(&bp, &p_in->s3);
    }
    if (0u != (p_in->present & NET_HAS_S4))
    {
        put_s4(&bp, &p_in->s4);
    }
    if (0u != (p_in->present & NET_HAS_S5))
    {
        put_s5(&bp, &p_in->s5);
    }
    return bp.b_overflow ? 0u : bitpack_bytes(&bp);
}

uint16_t
net_codec_hb(uint32_t uptime_s, uint32_t sm_status, uint8_t dropped,
             uint8_t *p_buf, uint16_t cap)
{
    bitpack_t bp;

    bitpack_init(&bp, p_buf, cap);
    bitpack_put(&bp, uptime_s, UPTIME_BITS);
    bitpack_put(&bp, SM_STATUS_STATE(sm_status), SM_STATE_BITS);
    bitpack_put(&bp, SM_STATUS_FAULTS(sm_status), SM_FAULT_BITS);
    bitpack_put(&bp, dropped, DROPPED_BITS);
    return bp.b_overflow ? 0u : bitpack_bytes(&bp);
}

bool
net_codec_evt_put(bitpack_t *p_bp, net_evt_rec_t const *p_evt)
{
    uint8_t const payload_bits = net_codec_evt_bits((net_evt_t)p_evt->type);
    uint16_t const need = (uint16_t)(NET_EVT_TYPE_BITS + NET_EVT_TIME_BITS +
                                     payload_bits);

    if ((NET_EVT_NONE == p_evt->type) || (p_evt->type >= NET_EVT_COUNT) ||
        (need > bitpack_free_bits(p_bp)))
    {
        return false;
    }
    bitpack_put(p_bp, p_evt->type, NET_EVT_TYPE_BITS);
    bitpack_put(p_bp, p_evt->time_ds, NET_EVT_TIME_BITS);
    if (payload_bits > 0u)
    {
        bitpack_put(p_bp, p_evt->payload, payload_bits);
    }
    return true;
}

/* BIND: [count:5] then count x [symbol:5][action:2]; each symbol at
   most once.  Symbols not listed become unbound. */
static bool
decode_bind(bitread_t *p_br, net_cmd_t *p_cmd)
{
    uint32_t const count = bitread_get(p_br, NET_BIND_COUNT_BITS);
    uint32_t i;

    if ((0u == count) || (count > SM_BARCODE_SYMBOLS))
    {
        return false;
    }
    for (i = 0u; i < count; i++)
    {
        uint32_t const symbol = bitread_get(p_br, SM_BARCODE_SYMBOL_BITS);
        uint32_t const action = bitread_get(p_br, SM_ACTION_BITS);

        if (p_br->b_underflow || (symbol >= SM_BARCODE_SYMBOLS) ||
            (0u != (p_cmd->bound_mask & (1u << symbol))))
        {
            return false;
        }
        p_cmd->bound_mask |= (1u << symbol);
        p_cmd->actions |= ((uint64_t)action << (symbol * SM_ACTION_BITS));
    }
    return true;
}

bool
net_codec_cmd(uint8_t const *p_buf, uint16_t len, net_cmd_t *p_cmd)
{
    bitread_t   br;
    bool        b_ok;

    p_cmd->op = NET_OP_NONE;
    p_cmd->bound_mask = 0u;
    p_cmd->actions = 0u;

    if (0u == len)
    {
        return false;
    }
    bitread_init(&br, p_buf, len);
    p_cmd->op = (uint8_t)bitread_get(&br, NET_OP_BITS);

    switch ((net_op_t)p_cmd->op)
    {
        case NET_OP_START:
        case NET_OP_STOP:
        case NET_OP_PING:
            b_ok = true;
            break;

        case NET_OP_BIND:
            b_ok = decode_bind(&br, p_cmd);
            break;

        case NET_OP_NONE:
        default:
            b_ok = false;
            break;
    }

    /* Exactly the bytes the fields need, padding bits zero */
    if (b_ok && (bitread_left(&br) < BITS_PER_BYTE))
    {
        uint8_t const pad = (uint8_t)bitread_left(&br);

        b_ok = (0u == pad) || (0u == bitread_get(&br, pad));
    }
    else
    {
        b_ok = false;
    }
    return b_ok && !br.b_underflow;
}
