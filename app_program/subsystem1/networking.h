/*
 *  networking.h
 *  Networking task: public interface for every subsystem.
 *
 *  Include as "../subsystem1/networking.h".
 *
 *  The Networking task is the only application task that deals with
 *  WiFi/MQTT.  Other subsystems never wait on the network:
 *
 *  IF-07  Telemetry snapshots (latest values), S2..S5 -> Networking
 *         Each subsystem implements sN_get_tlm() (see its header).
 *         Networking reads them every NET_TLM_PERIOD_MS.
 *
 *  IF-08  Telemetry events (must not be lost), any task -> Networking
 *         net_post_event(NET_EVT_..., payload).  Never blocks.  While
 *         MQTT is down events are kept in a bounded buffer that drops
 *         the oldest; losses are counted and reported.
 *
 *  Commands from the base station arrive here, are validated, and
 *  reach the State Machine as control events (IF-01).
 */

#ifndef NETWORKING_H
#define NETWORKING_H

#include "../app.h"
#include "net_types.h"

#define NET_CYCLE_MS            10u     /* task wake-up period           */
#define NET_TLM_PERIOD_MS       200u    /* telemetry frame (NFR1)        */
#define NET_HB_PERIOD_MS        1000u   /* heartbeat (NFR1)              */

/* IF-08: post a telemetry event.  Task context only; never blocks.
   payload must fit in NET_EVT_PAYLOAD_BITS. */
ER  net_post_event(net_evt_t type, uint32_t payload);

/* Created / started by subsystem1.c */
ER  net_init(void);
ER  net_start(void);

#endif /* NETWORKING_H */
