# MQTT interface — IS04 robot car

Owner: Buddy 1 (subsystem 1). Firmware: `app_program/subsystem1/` (Networking
task, `net_codec.c`) and `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` (MQTT
bridge). Base-station tool: `tools/base_station.py`.

All payloads are **compact binary** (Change Requirement 1), not text. Fields
are packed **most-significant bit first**, back to back with no padding, and
the last byte is padded with zero bits. Each field uses only the bits its
valid range needs.

## Topics

`<root>` is `MQTT_TOPIC_ROOT` in `config/wifi_secrets.h` (default `is04/car`).

| Topic | Direction | QoS | Retained | Rate | Payload |
|---|---|---|---|---|---|
| `<root>/status` | car → base | 1 | yes | on connect / last will | 1 byte: `0x01` online, `0x00` offline |
| `<root>/tlm` | car → base | 0 | no | every 200 ms while online | telemetry frame, 4–31 bytes |
| `<root>/hb` | car → base | 0 | no | every 1 s while online | heartbeat, 6 bytes |
| `<root>/evt` | car → base | 0 | no | as events occur | batch of events, ≤ 64 bytes |
| `<root>/cmd` | base → car | 0 | no | on demand | command, 1+ bytes |

The car's MQTT client ID is `MQTT_CLIENT_ID` (default `is04-car`). The
keepalive is 10 s, so the broker publishes the `offline` last will within
about 15 s of the car disappearing.

## Telemetry frame — `<root>/tlm`

Header (31 bits), then one block per subsystem whose bit is set in `present`.
A subsystem that has not implemented `sN_get_tlm()` yet is simply left out.

| Field | Bits | Range / meaning |
|---|---|---|
| seq | 8 | frame counter, wraps; a gap means frames were lost |
| state | 3 | 0 INITIALISING, 1 STANDBY, 2 NAVIGATING, 3 OBSTACLE_AVOIDANCE, 4 ERROR |
| nav_mode | 3 | 0 DISABLED, 1 TRACKING, 2 MANOEUVRE, 3 HOLD, 4 BYPASS, 5 RECOVERY |
| faults | 13 | fault bitmask, see [Faults](#faults) |
| present | 4 | bit 3 S2, bit 2 S3, bit 1 S4, bit 0 S5 |

**S2 Motion** (71 bits): speed_l_mms 10 (0–1023 mm/s), rev_l 1, speed_r_mms
10, rev_r 1, motion_state 3 (0 DISABLED, 1 LINE_FOLLOWING, 2 TURNING,
3 AVOIDING, 4 STALL_FAULT), enc_l 16 (pulses, wraps), enc_r 16, distance_cm 14
(0–16383 cm this run).

**S3 Line + barcode** (26 bits): line_state 3 (0 SELFTEST, 1 IDLE,
2 TRACKING, 3 SUSPENDED, 4 LOST, 5 REACQUIRING, 6 FAULT), line_valid 1,
line_pos 7 (0–100 % of half-width), line_left 1, ir_pattern 3 (bit 2 left,
bit 1 centre, bit 0 right; 1 = black), decoder_state 3 (0 IDLE, 1 SEARCHING,
2 CAPTURING, 3 VALIDATING, 4 WAIT_CLEAR), barcode_seen 1, last_symbol 5
(0–25 = `A`–`Z`), last_action 2.

**S4 IMU** (56 bits): pitch 8 (0–180 × 0.5°), pitch_down 1, roll 8
(0–180 × 0.5°), roll_left 1, heading 9 (0–359°), imu_status 2 (0 OK,
1 SELFTEST, 2 FAULT, 3 IDLE), turn_rate 9 (0–511 °/s), turn_left 1,
motion_class 3 (0 Stationary, 1 Moving straight, 2 Accelerating, 3 Turning,
4 Climbing hump, 5 Descending hump, 6 Sudden impact), last_peak_mm 7
(0–127 mm), max_peak_mm 7 (highest hump peak this run).

**S5 Ultrasound** (64 bits): scan_state 3 (0 IDLE, 1 MONITORING, 2 SCANNING,
3 PROFILING, 4 AVOIDANCE, 5 RECOVERY, 6 ERROR), distance_cm 9 (0–400,
511 = no echo), servo_deg 8 (0–180, 90 ahead), recommendation 3 (see
[Avoidance](#avoidance-recommendation)), profile_valid 1, left_clear_cm 8,
bearing_deg 8, width_cm 7, closest_cm 9, right_clear_cm 8.

Size: 4 bytes with no subsystem blocks, 31 bytes with all four.

## Heartbeat — `<root>/hb`

| Field | Bits | Meaning |
|---|---|---|
| uptime_s | 17 | seconds since boot, wraps after ~36 h |
| state | 3 | as in telemetry |
| faults | 13 | as in telemetry |
| events_lost | 8 | events lost since boot (saturates at 255) |

41 bits → 6 bytes.

## Events — `<root>/evt`

One message carries one or more events, oldest first. Each event is:

| Field | Bits | Meaning |
|---|---|---|
| type | 5 | event type below; `0` (or fewer than 5 bits left) ends the batch |
| time | 16 | uptime in 0.1 s when Networking received it, wraps after ~109 min |
| payload | 0–32 | depends on type |

| Type | Name | From | Payload (bits) |
|---|---|---|---|
| 1 | STATE | S1 | from 3, to 3, faults 13 — the fault code on entering ERROR (FR5) |
| 2 | CMD_ACK | S1 | opcode 3, accepted 1 |
| 3 | RUN_SUMMARY | S1 | max_peak_mm 7, distance_cm 14, duration_s 11 — on leaving a run (FR5) |
| 4 | HUMP | S4 | peak_mm 7 |
| 5 | MOTION_CLASS | S4 | class 3 |
| 6 | IMU_FAULT | S4 | — |
| 7 | BARCODE_OK | S3 | symbol 5, action 2 |
| 8 | BARCODE_REJECT | S3 | reason 3: 0 invalid, 1 incomplete, 2 unbound, 3 sample gap, 4 overflow |
| 9 | LINE_LOST | S3 | — |
| 10 | LINE_FOUND | S3 | — |
| 11 | IR_FAULT | S3 | — |
| 12 | OBSTACLE | S5 | distance_cm 9 |
| 13 | PROFILE | S5 | recommendation 3, bearing_deg 8, width_cm 7, closest_cm 9 |
| 14 | AVOID_DONE | S5 | — |
| 15 | DROPPED | S1 | count 8 — events lost just before this batch |

While MQTT is down, events wait in a 32-entry buffer on the car that drops
the oldest when full; the number dropped is sent as a `DROPPED` event when the
link returns. Telemetry and heartbeats are not buffered: only the latest
values matter.

Other subsystems post events with `net_post_event()` (`subsystem1/networking.h`),
using the `NET_EVT_*_PAYLOAD()` helpers. The call never blocks.

## Commands — `<root>/cmd`

The first 3 bits are the opcode. A command must be exactly as long as its
fields need, with zero padding bits; anything else is rejected with a
`CMD_ACK` (accepted = 0) and changes nothing.

| Opcode | Name | Fields after the opcode | Bytes | Example |
|---|---|---|---|---|
| 1 | START | — | 1 | `20` |
| 2 | STOP (park) | — | 1 | `40` |
| 3 | BIND | count 5 (1–26), then count × (symbol 5, action 2) | 1 + ⌈7·count/8⌉ | A→right, B→left: `62 02 10` |
| 4 | PING | — | 1 | `80` |

- START is accepted only in STANDBY. STOP is accepted in STANDBY, NAVIGATING
  and OBSTACLE_AVOIDANCE. BIND is accepted only in STANDBY and replaces the
  whole table: symbols not listed become unbound, so the car ignores them.
- The State Machine decides; its `CMD_ACK` event reports the result. PING is
  acknowledged by Networking at once and is useful for measuring command
  latency (NFR2).
- Barcode actions: 0 left, 1 right, 2 straight, 3 U-turn. Default bindings at
  power-on: A left, B right, C straight, D U-turn.

## Faults

| Bit | Fault | Stops the car |
|---|---|---|
| 0 | MOTOR_STALL | yes |
| 1 | ENCODER | yes |
| 2 | IMU | no (FR27) |
| 3 | NO_ECHO | yes |
| 4 | WIFI_LOSS | no (FR6) |
| 5 | COLLISION | yes |
| 6 | TIP_OVER | yes |
| 7 | IR_SENSOR | yes |
| 8 | LINE_RECOVERY | yes |
| 9 | SCAN | yes |
| 10 | INIT_TIMEOUT | yes |
| 11 | SELF_TEST | yes |
| 12 | RADIO | yes |

## Avoidance recommendation

0 continue, 1 stop, 2 left, 3 right, 4 reverse and re-attempt.

## Data representation (Change Requirement 1)

Application data owned by subsystem 1, with the bits each field uses.

| Data | Valid range | Bits | Representation |
|---|---|---|---|
| System state | 5 states | 3 | `sm_ctx_t.state` bit-field; status word bits 0–2; wire 3 |
| Navigation mode | 6 modes | 3 | bit-field; status word bits 3–5; wire 3 |
| Fault conditions | 13 independent faults | 13 | one bit each in a shared mask (`SM_FAULT_BIT()`) |
| ERROR retries | 0–3 | 2 | bit-field; status word bits 19–20 |
| Ready subsystems | 5 subsystems | 5 | bitmask bit-field |
| Pending barcode action | 4 actions + valid | 2 + 1 | bit-fields |
| Whole statechart context | — | 30 | `sm_ctx_t`: one 32-bit word |
| State snapshot shared with all tasks | — | 21 | one 32-bit word, single writer, one store: safe to read from any core without a lock |
| Control event (inter-task) | 19 codes, 11-bit argument | 16 | `uint16_t` `[code:5][arg:11]` |
| Navigation command (inter-task) | 3 commands, 3-bit argument | 6 | 1 byte `[cmd:3][arg:3]` |
| Telemetry event (inter-task) | 15 types, ≤ 27-bit payload | 32 | one `uint32_t` `[type:5][payload:27]` |
| Barcode binding table | 26 symbols × 4 actions | 26 + 52 | 26-bit bound mask + 2 bits per symbol in a `uint64_t` |
| Barcode symbol | `A`–`Z` | 5 | 0–25 |
| Command opcode | 4 commands | 3 | wire, first 3 bits |
| Uptime in heartbeat | 0–36 h | 17 | wire |
| Event timestamp | 0–109 min | 16 | wire, 0.1 s units |
| Events-lost counter | 0–255 | 8 | saturating |

Packed data shared between tasks is either written by one task with a single
aligned 32-bit store (the status word), copied under a mutex (binding table,
`sN_get_tlm()` snapshots), or passed by value through a kernel message buffer
(events, commands). No task does a read-modify-write on packed data another
task writes. The one shared counter (events that could not be queued) is a
kernel semaphore, whose count the kernel updates atomically.

For MQTT, fields are packed with explicit shifts (`bitpack.c`), not by sending
C structs, because bit-field layout is compiler-defined and would add padding.
