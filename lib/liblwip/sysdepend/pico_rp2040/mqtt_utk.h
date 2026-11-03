/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	lwIP and the MQTT client are only ever called from cyw43_utk_task()
 *	(docs/RELEASE.md safety rule 6).  Application tasks never call them:
 *	they exchange whole MQTT payloads with this code through two message
 *	buffers that the application creates and attaches with
 *	mqtt_utk_attach():
 *
 *	  TX  application -> broker   message = [topic index:1 byte][payload]
 *	  RX  broker -> application   message = [payload] from the command
 *	                              topic only
 *
 *	The service task drains TX with TMO_POL (never blocking) and posts
 *	RX with TMO_POL, so a slow or stuck application task cannot stall
 *	the radio.
 *
 *	Topics are MQTT_TOPIC_ROOT (config/wifi_secrets.h) followed by:
 *	  /tlm     telemetry          (TX index MQTT_UTK_TOPIC_TLM)
 *	  /hb      heartbeat          (TX index MQTT_UTK_TOPIC_HB)
 *	  /evt     events             (TX index MQTT_UTK_TOPIC_EVT)
 *	  /status  1 byte, retained: 0x01 online, 0x00 offline (last will)
 *	  /cmd     commands           (subscribed, delivered on RX)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

/* Outbound topic index: first byte of every TX message */
#define MQTT_UTK_TOPIC_TLM	0u
#define MQTT_UTK_TOPIC_HB	1u
#define MQTT_UTK_TOPIC_EVT	2u
#define MQTT_UTK_TOPIC_COUNT	3u

/* Largest payload either way (bytes) */
#define MQTT_UTK_MAX_PAYLOAD	64u

/* Status payloads on <root>/status */
#define MQTT_UTK_STATUS_OFFLINE	0x00u
#define MQTT_UTK_STATUS_ONLINE	0x01u

/*
 * Called from cyw43_utk_task() while the link is up. Starts the MQTT
 * connection to MQTT_BROKER_IP:MQTT_BROKER_PORT. Safe to call more than
 * once -- a call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);

/*
 * Called from cyw43_utk_task() on every poll: updates the connected
 * flag and publishes queued TX messages (dropping them while offline).
 */
void mqtt_utk_poll(int link_up);

/*
 * Called once by the application, before its tasks start, with the IDs
 * of the TX and RX message buffers it created (maxmsz at least
 * MQTT_UTK_MAX_PAYLOAD + 1 for TX and MQTT_UTK_MAX_PAYLOAD for RX).
 */
void mqtt_utk_attach(int tx_mbf, int rx_mbf);

/* Safe from any task: 1 while connected to the broker */
int mqtt_utk_is_connected(void);

#endif /* MQTT_UTK_H */
