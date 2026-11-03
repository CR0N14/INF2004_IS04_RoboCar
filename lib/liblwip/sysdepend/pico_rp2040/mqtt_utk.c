/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() and
 *	mqtt_utk_poll() on every poll iteration while the link is up.
 *	mqtt.c's own keepalive/ping/retry timers ride the same
 *	sys_check_timeouts() polling that task already does for DHCP and
 *	TCP -- nothing extra runs on any other task or core.
 *
 *	Application data is exchanged through two message buffers (see
 *	mqtt_utk.h).  This file does not include tk/tkernel.h or any
 *	pico-sdk/cyw43 header, so the two kernel calls it needs are declared
 *	below with their exact micro T-Kernel types (ID = INT = int,
 *	TMO = W = long), the same way cyw43_utk.c declares tk_dly_tsk().
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <string.h>
#include <tm/tmonitor.h>

extern int tk_snd_mbf(int mbfid, const void *msg, int msgsz, long tmout);
extern int tk_rcv_mbf(int mbfid, void *msg, long tmout);

#define MQTT_UTK_TMO_POL	0L
#define MQTT_UTK_KEEPALIVE_S	10
#define MQTT_UTK_RETRY_MS	3000

/* Publishes per poll: bounded so one poll never holds the radio task
   for long, and below MQTT_REQ_MAX_IN_FLIGHT (4). */
#define MQTT_UTK_TX_PER_POLL	3

/* Older wifi_secrets.h files only define MQTT_TOPIC */
#ifndef MQTT_TOPIC_ROOT
#define MQTT_TOPIC_ROOT		"is04/car"
#endif

#define MQTT_UTK_TOPIC_STATUS	MQTT_TOPIC_ROOT "/status"
#define MQTT_UTK_TOPIC_CMD	MQTT_TOPIC_ROOT "/cmd"

static const char * const tx_topics[MQTT_UTK_TOPIC_COUNT] = {
	MQTT_TOPIC_ROOT "/tlm",		/* MQTT_UTK_TOPIC_TLM */
	MQTT_TOPIC_ROOT "/hb",		/* MQTT_UTK_TOPIC_HB  */
	MQTT_TOPIC_ROOT "/evt",		/* MQTT_UTK_TOPIC_EVT */
};

static const u8_t status_online  = MQTT_UTK_STATUS_ONLINE;
static const char status_offline = (char)MQTT_UTK_STATUS_OFFLINE;

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

/* Read by application tasks: plain word, single writer (this task) */
static volatile int	connected;

/* Attached by the application before its tasks start */
static volatile int	tx_mbf;
static volatile int	rx_mbf;

/* One outbound message held over when lwIP's output buffer is full */
static u8_t		tx_buf[MQTT_UTK_MAX_PAYLOAD + 1u];
static int		tx_len;

/* Inbound message staging. A publish arrives as one topic callback
 * followed by one or more data callbacks, so whether it is a command
 * has to be remembered until the fragment carrying MQTT_DATA_FLAG_LAST
 * shows up. Oversized payloads are dropped, never truncated: a cut
 * binary command could still look valid. */
static u8_t		rx_is_cmd;
static u8_t		rx_data[MQTT_UTK_MAX_PAYLOAD];
static u16_t		rx_len;
static u8_t		rx_too_long;

void mqtt_utk_attach(int tx, int rx)
{
	tx_mbf = tx;
	rx_mbf = rx;
}

int mqtt_utk_is_connected(void)
{
	return connected;
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is
 * only valid for the duration of this call. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;

	rx_is_cmd = (strcmp(topic, MQTT_UTK_TOPIC_CMD) == 0)? 1u: 0u;
	rx_len = 0;
	rx_too_long = (tot_len > MQTT_UTK_MAX_PAYLOAD)? 1u: 0u;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	(void)arg;

	if(rx_too_long == 0u && (u32_t)rx_len + len <= MQTT_UTK_MAX_PAYLOAD) {
		memcpy(&rx_data[rx_len], data, len);
		rx_len = (u16_t)(rx_len + len);
	} else {
		rx_too_long = 1u;
	}

	if((flags & MQTT_DATA_FLAG_LAST) == 0) {
		return;
	}

	if(rx_is_cmd == 0u) {
		return;
	}
	if(rx_too_long != 0u || rx_len == 0u) {
		tm_printf((UB*)"mqtt: command dropped (length)\n");
		return;
	}
	if(rx_mbf <= 0 ||
	   tk_snd_mbf(rx_mbf, rx_data, (int)rx_len, MQTT_UTK_TMO_POL) != 0) {
		tm_printf((UB*)"mqtt: command dropped (queue full)\n");
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_UTK_TOPIC_CMD "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks must be registered before subscribing, or the
		 * first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_UTK_TOPIC_CMD, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Retained, so a base station that connects later still sees
		 * that the car is online. The broker replaces it with the
		 * last-will "offline" byte if the connection is lost. */
		(void)mqtt_publish(client, MQTT_UTK_TOPIC_STATUS, &status_online,
				   1u, 1 /* qos */, 1 /* retain */, NULL, NULL);
	} else {
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		connecting = 1;	/* hold off mqtt_utk_start() until the retry */
		sys_untimeout(retry_timeout, NULL);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;
	client_info.will_topic = MQTT_UTK_TOPIC_STATUS;
	client_info.will_msg = &status_offline;
	client_info.will_msg_len = 1u;
	client_info.will_qos = 1u;
	client_info.will_retain = 1u;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}

void mqtt_utk_poll(int link_up)
{
	int	i;
	err_t	err;

	connected = (link_up && client != NULL &&
		     mqtt_client_is_connected(client))? 1: 0;

	if(tx_mbf <= 0) {
		return;
	}

	for(i = 0; i < MQTT_UTK_TX_PER_POLL; i++) {
		if(tx_len == 0) {
			int const len = tk_rcv_mbf(tx_mbf, tx_buf, MQTT_UTK_TMO_POL);

			if(len < 2) {
				break;			/* queue empty (or junk) */
			}
			tx_len = len;
		}

		if(connected == 0 || tx_buf[0] >= MQTT_UTK_TOPIC_COUNT) {
			tx_len = 0;			/* offline: drop it */
			continue;
		}

		err = mqtt_publish(client, tx_topics[tx_buf[0]], &tx_buf[1],
				   (u16_t)(tx_len - 1), 0 /* qos */,
				   0 /* retain */, NULL, NULL);
		if(err == ERR_MEM) {
			break;		/* output full: keep it, retry next poll */
		}
		tx_len = 0;
	}
}
