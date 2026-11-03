/*
 *  networking.c
 *  Networking task: telemetry, heartbeat, events and commands over MQTT.
 *
 *  Woken every NET_CYCLE_MS by a cyclic handler.  Each cycle it:
 *    1. follows the MQTT connection and reports NET_UP / NET_DOWN to the
 *       State Machine (losing the link is never a reason to stop, FR6)
 *    2. validates commands from the base station and forwards them to
 *       the State Machine as control events
 *    3. moves telemetry events from other tasks into a bounded local
 *       buffer that drops the oldest when full
 *    4. while online: publishes a telemetry frame every
 *       NET_TLM_PERIOD_MS, a heartbeat every NET_HB_PERIOD_MS, and any
 *       queued events.
 *
 *  lwIP may only be called from the kernel's WiFi task, so frames are
 *  handed to the MQTT bridge (lib/liblwip/.../mqtt_utk.c) through a TX
 *  message buffer, and commands come back through an RX message
 *  buffer.  Nothing here ever waits on the network.
 */

#include "networking.h"
#include "state_machine.h"
#include "net_codec.h"

#if TM_MQTT
#include "mqtt_utk.h"
#include "cyw43_utk.h"
#else
#define MQTT_UTK_TOPIC_TLM      0u
#define MQTT_UTK_TOPIC_HB       1u
#define MQTT_UTK_TOPIC_EVT      2u
#define MQTT_UTK_MAX_PAYLOAD    64u
#endif

#define NET_TLM_CYCLES      (NET_TLM_PERIOD_MS / NET_CYCLE_MS)
#define NET_HB_CYCLES       (NET_HB_PERIOD_MS / NET_CYCLE_MS)

/* Event message: [type:5][payload:27] in one 32-bit word */
#define NET_EVT_MSG_SIZE    ((INT)sizeof(uint32_t))
#define NET_EVT_PAYLOAD_MASK ((1u << NET_EVT_PAYLOAD_BITS) - 1u)

/* Queue sizes.  A message buffer entry costs a 4-byte kernel header
   plus the message rounded up to 4 bytes. */
#define NET_MBF_HEADER      4u
#define NET_EVT_QUEUE_LEN   16u
#define NET_EVT_RING_LEN    32u     /* kept while offline, oldest dropped */
#define NET_TX_MSG_MAX      (MQTT_UTK_MAX_PAYLOAD + 1u)
#define NET_TX_QUEUE_BYTES  256u
#define NET_RX_QUEUE_LEN    2u
#define NET_LOST_MAX        255u    /* semaphore used as a loss counter  */

#define MS_PER_DS           100u
#define MS_PER_S            1000u
#define COUNT_MAX_8         255u

static ID   g_tid_net;
static ID   g_cyc_net;
static ID   g_mbf_evt;      /* IF-08: other tasks -> Networking          */
static ID   g_sem_lost;     /* events producers could not queue          */
static ID   g_mbf_tx;       /* Networking -> MQTT bridge                 */
static ID   g_mbf_rx;       /* MQTT bridge -> Networking                 */

/* Task-private state (only the Networking task touches it) */
static net_evt_rec_t g_ring[NET_EVT_RING_LEN];
static uint8_t  g_ring_head;        /* oldest entry                      */
static uint8_t  g_ring_count;
static uint8_t  g_lost_pending;     /* not yet reported, saturating      */
static uint8_t  g_lost_total;       /* since boot, saturating            */
static uint8_t  g_tlm_seq;
static bool     g_b_online;
static bool     g_b_radio_checked;
static bool     g_b_run_active;
static uint32_t g_run_start_ms;

static uint8_t  g_tx_buf[NET_TX_MSG_MAX];
static uint8_t  g_rx_buf[MQTT_UTK_MAX_PAYLOAD];

static uint32_t
now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);
    return (uint32_t)tim.lo;
}

static uint8_t
add_saturating(uint8_t count, uint32_t add)
{
    uint32_t const sum = (uint32_t)count + add;

    return (uint8_t)((sum > COUNT_MAX_8) ? COUNT_MAX_8 : sum);
}

/* ---- Local event buffer ------------------------------------------- */

static void
ring_push(net_evt_t type, uint32_t payload, uint32_t now)
{
    uint8_t idx;

    if (NET_EVT_RING_LEN == g_ring_count)
    {
        /* Full: drop the oldest, keep the newest */
        g_ring_head = (uint8_t)((g_ring_head + 1u) % NET_EVT_RING_LEN);
        g_ring_count--;
        g_lost_pending = add_saturating(g_lost_pending, 1u);
        g_lost_total = add_saturating(g_lost_total, 1u);
    }
    idx = (uint8_t)((g_ring_head + g_ring_count) % NET_EVT_RING_LEN);
    g_ring[idx].type = (uint8_t)type;
    g_ring[idx].payload = payload;
    g_ring[idx].time_ds = (uint16_t)(now / MS_PER_DS);
    g_ring_count++;
}

/* ---- Run summary (FR5) -------------------------------------------- */

static void
track_run(uint32_t payload, uint32_t now)
{
    sm_state_t const from = (sm_state_t)((payload >> 16u) &
                            SM_FIELD_MASK(SM_STATE_BITS));
    sm_state_t const to = (sm_state_t)((payload >> 13u) &
                          SM_FIELD_MASK(SM_STATE_BITS));

    if ((SM_STATE_NAVIGATING == to) && (SM_STATE_STANDBY == from))
    {
        g_b_run_active = true;
        g_run_start_ms = now;
    }
    else if (g_b_run_active &&
             ((SM_STATE_STANDBY == to) || (SM_STATE_ERROR == to)))
    {
        s2_tlm_t    s2;
        s4_tlm_t    s4;
        uint32_t    peak = 0u;
        uint32_t    dist = 0u;
        uint32_t    secs = (now - g_run_start_ms) / MS_PER_S;

        if (E_OK == s4_get_tlm(&s4))
        {
            peak = s4.max_peak_mm;
        }
        if (E_OK == s2_get_tlm(&s2))
        {
            dist = s2.distance_cm;
        }
        if (secs > SM_FIELD_MASK(11u))
        {
            secs = SM_FIELD_MASK(11u);
        }
        g_b_run_active = false;
        ring_push(NET_EVT_RUN_SUMMARY, (peak << 25u) | (dist << 11u) | secs,
                  now);
        tm_printf((UB *)"net: run summary: peak %u mm, %u cm, %u s\n",
                  (unsigned int)peak, (unsigned int)dist,
                  (unsigned int)secs);
    }
    else
    {
        /* not a run boundary */
    }
}

static void
collect_events(uint32_t now)
{
    uint32_t msg;
    T_RSEM   rsem;

    while (NET_EVT_MSG_SIZE == tk_rcv_mbf(g_mbf_evt, &msg, TMO_POL))
    {
        net_evt_t const type = (net_evt_t)(msg >> NET_EVT_PAYLOAD_BITS);
        uint32_t const payload = msg & NET_EVT_PAYLOAD_MASK;

        ring_push(type, payload, now);
        if (NET_EVT_STATE == type)
        {
            track_run(payload, now);
        }
    }

    /* Events their producers could not queue */
    if ((E_OK == tk_ref_sem(g_sem_lost, &rsem)) && (rsem.semcnt > 0))
    {
        if (E_OK == tk_wai_sem(g_sem_lost, rsem.semcnt, TMO_POL))
        {
            g_lost_pending = add_saturating(g_lost_pending,
                                            (uint32_t)rsem.semcnt);
            g_lost_total = add_saturating(g_lost_total,
                                          (uint32_t)rsem.semcnt);
        }
    }
}

/* ---- Publishing --------------------------------------------------- */

static void
queue_tx(uint8_t topic, uint16_t payload_len)
{
    g_tx_buf[0] = topic;
    if ((payload_len > 0u) &&
        (E_OK != tk_snd_mbf(g_mbf_tx, g_tx_buf, (INT)(payload_len + 1u),
                            TMO_POL)))
    {
        tm_printf((UB *)"net: TX queue full, frame dropped\n");
    }
}

static void
send_telemetry(void)
{
    net_tlm_in_t in;

    in.sm_status = sm_status_word();
    in.seq = g_tlm_seq++;
    in.present = 0u;
    if (E_OK == s2_get_tlm(&in.s2))
    {
        in.present |= NET_HAS_S2;
    }
    if (E_OK == s3_get_tlm(&in.s3))
    {
        in.present |= NET_HAS_S3;
    }
    if (E_OK == s4_get_tlm(&in.s4))
    {
        in.present |= NET_HAS_S4;
    }
    if (E_OK == s5_get_tlm(&in.s5))
    {
        in.present |= NET_HAS_S5;
    }
    queue_tx(MQTT_UTK_TOPIC_TLM,
             net_codec_tlm(&in, &g_tx_buf[1], NET_TLM_MAX_BYTES));
}

static void
send_heartbeat(uint32_t now)
{
    queue_tx(MQTT_UTK_TOPIC_HB,
             net_codec_hb(now / MS_PER_S, sm_status_word(), g_lost_total,
                          &g_tx_buf[1], NET_HB_BYTES));
}

/* Pack as many queued events as fit into one message, oldest first.
   They leave the buffer only once the message is queued for sending. */
static void
send_events(uint32_t now)
{
    while ((g_ring_count > 0u) || (g_lost_pending > 0u))
    {
        bitpack_t       bp;
        net_evt_rec_t   lost;
        uint8_t         used = 0u;

        bitpack_init(&bp, &g_tx_buf[1], MQTT_UTK_MAX_PAYLOAD);

        if (g_lost_pending > 0u)
        {
            lost.type = NET_EVT_DROPPED;
            lost.payload = g_lost_pending;
            lost.time_ds = (uint16_t)(now / MS_PER_DS);
            (void)net_codec_evt_put(&bp, &lost);
        }
        while ((used < g_ring_count) &&
               net_codec_evt_put(&bp, &g_ring[(g_ring_head + used) %
                                              NET_EVT_RING_LEN]))
        {
            used++;
        }

        g_tx_buf[0] = MQTT_UTK_TOPIC_EVT;
        if (E_OK != tk_snd_mbf(g_mbf_tx, g_tx_buf,
                               (INT)(bitpack_bytes(&bp) + 1u), TMO_POL))
        {
            break;      /* bridge busy: try again next cycle */
        }
        g_lost_pending = 0u;
        g_ring_head = (uint8_t)((g_ring_head + used) % NET_EVT_RING_LEN);
        g_ring_count = (uint8_t)(g_ring_count - used);
    }
}

/* ---- Link and commands -------------------------------------------- */

static void
update_link(void)
{
    bool b_up = false;

#if TM_MQTT
    if (!g_b_radio_checked)
    {
        T_CYW43_UTK_STATUS st;

        cyw43_utk_get_status(&st);
        if (0u != st.init_complete)
        {
            g_b_radio_checked = true;
            if (0 != st.init_result)
            {
                tm_printf((UB *)"net: WiFi chip failed to start\n");
                (void)sm_post_event(SM_EVT_FAULT, SM_FAULT_RADIO);
            }
        }
    }
    b_up = (0 != mqtt_utk_is_connected());
#else
    g_b_radio_checked = true;
#endif

    if ((b_up != g_b_online) &&
        (E_OK == sm_post_event(b_up ? SM_EVT_NET_UP : SM_EVT_NET_DOWN, 0u)))
    {
        g_b_online = b_up;
        tm_printf((UB *)"net: MQTT %s\n", b_up ? "online" : "offline");
    }
}

static void
reply(uint8_t op, bool b_ok, uint32_t now)
{
    ring_push(NET_EVT_CMD_ACK, NET_EVT_ACK_PAYLOAD(op, b_ok), now);
}

static void
handle_commands(uint32_t now)
{
    INT len;

    while ((len = tk_rcv_mbf(g_mbf_rx, g_rx_buf, TMO_POL)) > 0)
    {
        net_cmd_t   cmd;
        ER          er = E_OK;

        if (!net_codec_cmd(g_rx_buf, (uint16_t)len, &cmd))
        {
            tm_printf((UB *)"net: malformed command (op %u, %d bytes)\n",
                      (unsigned int)cmd.op, (int)len);
            reply(cmd.op, false, now);
            continue;
        }

        /* The State Machine accepts or rejects START/STOP/BIND for the
           current state and acknowledges them itself. */
        switch ((net_op_t)cmd.op)
        {
            case NET_OP_START:
                er = sm_post_event(SM_EVT_CMD_START, 0u);
                break;

            case NET_OP_STOP:
                er = sm_post_event(SM_EVT_CMD_STOP, 0u);
                break;

            case NET_OP_BIND:
                er = sm_stage_bindings(cmd.bound_mask, cmd.actions);
                if (E_OK == er)
                {
                    er = sm_post_event(SM_EVT_CMD_BIND, 0u);
                }
                break;

            case NET_OP_PING:
            default:
                reply(cmd.op, true, now);
                break;
        }
        if (E_OK != er)
        {
            reply(cmd.op, false, now);
        }
    }
}

/* ---- Task --------------------------------------------------------- */

static void
net_cyclic_handler(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(g_tid_net);
}

static void
net_task(INT stacd, void *exinf)
{
    uint8_t tlm_cycles = 0u;
    uint8_t hb_cycles = 0u;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        uint32_t now;

        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);     /* never "catch up" in a burst */
        now = now_ms();

        update_link();
        handle_commands(now);
        collect_events(now);

        if (++tlm_cycles >= NET_TLM_CYCLES)
        {
            tlm_cycles = 0u;
            if (g_b_online)
            {
                send_telemetry();
            }
        }
        if (++hb_cycles >= NET_HB_CYCLES)
        {
            hb_cycles = 0u;
            if (g_b_online)
            {
                send_heartbeat(now);
            }
        }
        if (g_b_online)
        {
            send_events(now);
        }
    }
}

EXPORT ER
net_post_event(net_evt_t type, uint32_t payload)
{
    uint32_t const msg = ((uint32_t)type << NET_EVT_PAYLOAD_BITS) | payload;
    ER er;

    if ((NET_EVT_NONE == type) || (type >= NET_EVT_COUNT) ||
        (0u != (payload & ~NET_EVT_PAYLOAD_MASK)))
    {
        return E_PAR;
    }
    er = tk_snd_mbf(g_mbf_evt, &msg, NET_EVT_MSG_SIZE, TMO_POL);
    if (E_OK != er)
    {
        (void)tk_sig_sem(g_sem_lost, 1);    /* counted, reported later */
    }
    return er;
}

EXPORT ER
net_init(void)
{
    T_CMBF cmbf = {0};
    T_CSEM csem = {0};
    T_CCYC ccyc = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)(NET_EVT_QUEUE_LEN *
                      (NET_MBF_HEADER + (uint32_t)NET_EVT_MSG_SIZE));
    cmbf.maxmsz = NET_EVT_MSG_SIZE;
    g_mbf_evt = tk_cre_mbf(&cmbf);
    if (g_mbf_evt < E_OK)
    {
        return g_mbf_evt;
    }

    cmbf.bufsz = (SZ)NET_TX_QUEUE_BYTES;
    cmbf.maxmsz = (INT)NET_TX_MSG_MAX;
    g_mbf_tx = tk_cre_mbf(&cmbf);
    if (g_mbf_tx < E_OK)
    {
        return g_mbf_tx;
    }

    cmbf.bufsz = (SZ)(NET_RX_QUEUE_LEN *
                      (NET_MBF_HEADER + MQTT_UTK_MAX_PAYLOAD));
    cmbf.maxmsz = (INT)MQTT_UTK_MAX_PAYLOAD;
    g_mbf_rx = tk_cre_mbf(&cmbf);
    if (g_mbf_rx < E_OK)
    {
        return g_mbf_rx;
    }

    csem.sematr = TA_TFIFO | TA_FIRST;
    csem.isemcnt = 0;
    csem.maxsem = (INT)NET_LOST_MAX;
    g_sem_lost = tk_cre_sem(&csem);
    if (g_sem_lost < E_OK)
    {
        return g_sem_lost;
    }

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)net_cyclic_handler;
    ccyc.cyctim = NET_CYCLE_MS;
    ccyc.cycphs = NET_CYCLE_MS;
    g_cyc_net = tk_cre_cyc(&ccyc);
    if (g_cyc_net < E_OK)
    {
        return g_cyc_net;
    }

#if TM_MQTT
    mqtt_utk_attach((int)g_mbf_tx, (int)g_mbf_rx);
#endif
    return E_OK;
}

EXPORT ER
net_start(void)
{
    g_tid_net = app_start_task(net_task, PRI_NETWORKING, 0u, "net_task");
    if (g_tid_net < E_OK)
    {
        return g_tid_net;
    }
    return tk_sta_cyc(g_cyc_net);
}
