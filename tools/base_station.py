#!/usr/bin/env python3
"""
base_station.py - IS04 robot car MQTT base station.

Decodes the car's binary telemetry and sends commands. The wire format is
documented in docs/MQTT.md and must stay in step with
app_program/subsystem1/net_codec.c.

    pip install paho-mqtt

    python tools/base_station.py monitor               # print everything
    python tools/base_station.py monitor --sizes       # + binary vs JSON size
    python tools/base_station.py ping                  # command round trip
    python tools/base_station.py start
    python tools/base_station.py stop
    python tools/base_station.py bind A=left B=right C=straight D=uturn
    python tools/base_station.py raw 20                # send raw hex bytes
    python tools/base_station.py decode tlm 1f2c00...  # decode offline

Options: --host (default localhost), --port (1883), --root (is04/car).
"""

import argparse
import json
import sys
import threading
import time

STATES = ["INITIALISING", "STANDBY", "NAVIGATING", "OBSTACLE_AVOIDANCE",
          "ERROR"]
NAV_MODES = ["DISABLED", "TRACKING", "MANOEUVRE", "HOLD", "BYPASS",
             "RECOVERY"]
FAULTS = ["MOTOR_STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI_LOSS",
          "COLLISION", "TIP_OVER", "IR_SENSOR", "LINE_RECOVERY", "SCAN",
          "INIT_TIMEOUT", "SELF_TEST", "RADIO"]
ACTIONS = ["left", "right", "straight", "uturn"]
AVOID = ["continue", "stop", "left", "right", "reverse"]
OPS = {1: "START", 2: "STOP", 3: "BIND", 4: "PING"}
MOTION_STATES = ["DISABLED", "LINE_FOLLOWING", "TURNING", "AVOIDING",
                 "STALL_FAULT"]
LINE_STATES = ["SELFTEST", "IDLE", "TRACKING", "SUSPENDED", "LOST",
               "REACQUIRING", "FAULT"]
DECODER_STATES = ["IDLE", "SEARCHING", "CAPTURING", "VALIDATING",
                  "WAIT_CLEAR"]
IMU_STATUS = ["OK", "SELFTEST", "FAULT", "IDLE"]
MOTION_CLASSES = ["Stationary", "Moving straight", "Accelerating",
                  "Turning", "Climbing hump", "Descending hump",
                  "Sudden impact"]
SCAN_STATES = ["IDLE", "MONITORING", "SCANNING", "PROFILING", "AVOIDANCE",
               "RECOVERY", "ERROR"]
REJECT_REASONS = ["invalid", "incomplete", "unbound", "sample gap",
                  "overflow"]

# Event types: (name, [(field, bits), ...])
EVENTS = {
    1: ("STATE", [("from", 3), ("to", 3), ("faults", 13)]),
    2: ("CMD_ACK", [("opcode", 3), ("accepted", 1)]),
    3: ("RUN_SUMMARY", [("max_peak_mm", 7), ("distance_cm", 14),
                        ("duration_s", 11)]),
    4: ("HUMP", [("peak_mm", 7)]),
    5: ("MOTION_CLASS", [("class", 3)]),
    6: ("IMU_FAULT", []),
    7: ("BARCODE_OK", [("symbol", 5), ("action", 2)]),
    8: ("BARCODE_REJECT", [("reason", 3)]),
    9: ("LINE_LOST", []),
    10: ("LINE_FOUND", []),
    11: ("IR_FAULT", []),
    12: ("OBSTACLE", [("distance_cm", 9)]),
    13: ("PROFILE", [("recommendation", 3), ("bearing_deg", 8),
                     ("width_cm", 7), ("closest_cm", 9)]),
    14: ("AVOID_DONE", []),
    15: ("DROPPED", [("count", 8)]),
}

S2_FIELDS = [("speed_l_mms", 10), ("rev_l", 1), ("speed_r_mms", 10),
             ("rev_r", 1), ("motion_state", 3), ("enc_l", 16),
             ("enc_r", 16), ("distance_cm", 14)]
S3_FIELDS = [("line_state", 3), ("line_valid", 1), ("line_pos", 7),
             ("line_left", 1), ("ir_pattern", 3), ("decoder_state", 3),
             ("barcode_seen", 1), ("last_symbol", 5), ("last_action", 2)]
S4_FIELDS = [("pitch_half_deg", 8), ("pitch_down", 1),
             ("roll_half_deg", 8), ("roll_left", 1), ("heading_deg", 9),
             ("imu_status", 2), ("turn_rate_dps", 9), ("turn_left", 1),
             ("motion_class", 3), ("last_peak_mm", 7), ("max_peak_mm", 7)]
S5_FIELDS = [("scan_state", 3), ("distance_cm", 9), ("servo_deg", 8),
             ("recommendation", 3), ("profile_valid", 1),
             ("left_clear_cm", 8), ("bearing_deg", 8), ("width_cm", 7),
             ("closest_cm", 9), ("right_clear_cm", 8)]

OP_BITS = 3
BIND_COUNT_BITS = 5
SYMBOL_BITS = 5
ACTION_BITS = 2
EVT_TYPE_BITS = 5
EVT_TIME_BITS = 16


class BitReader:
    """Reads MSB-first bit fields, as written by bitpack.c."""

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def left(self):
        return len(self.data) * 8 - self.pos

    def get(self, bits):
        if bits > self.left():
            raise ValueError("frame too short")
        value = 0
        for _ in range(bits):
            byte = self.data[self.pos // 8]
            value = (value << 1) | ((byte >> (7 - self.pos % 8)) & 1)
            self.pos += 1
        return value

    def fields(self, spec):
        return {name: self.get(bits) for name, bits in spec}


class BitWriter:
    """Writes MSB-first bit fields, zero-padded to whole bytes."""

    def __init__(self):
        self.bits = []

    def put(self, value, bits):
        for i in range(bits - 1, -1, -1):
            self.bits.append((value >> i) & 1)

    def to_bytes(self):
        bits = self.bits + [0] * (-len(self.bits) % 8)
        return bytes(int("".join(map(str, bits[i:i + 8])), 2)
                     for i in range(0, len(bits), 8))


def name(table, index):
    return table[index] if index < len(table) else str(index)


def fault_names(mask):
    return [f for i, f in enumerate(FAULTS) if mask & (1 << i)] or ["none"]


def decode_tlm(data):
    r = BitReader(data)
    out = {"seq": r.get(8), "state": name(STATES, r.get(3)),
           "nav_mode": name(NAV_MODES, r.get(3)),
           "faults": fault_names(r.get(13))}
    present = r.get(4)
    if present & 0x8:
        s2 = r.fields(S2_FIELDS)
        s2["motion_state"] = name(MOTION_STATES, s2["motion_state"])
        out["motion"] = s2
    if present & 0x4:
        s3 = r.fields(S3_FIELDS)
        s3["line_state"] = name(LINE_STATES, s3["line_state"])
        s3["decoder_state"] = name(DECODER_STATES, s3["decoder_state"])
        s3["last_symbol"] = chr(ord("A") + s3["last_symbol"])
        s3["last_action"] = name(ACTIONS, s3["last_action"])
        out["line"] = s3
    if present & 0x2:
        s4 = r.fields(S4_FIELDS)
        s4["imu_status"] = name(IMU_STATUS, s4["imu_status"])
        s4["motion_class"] = name(MOTION_CLASSES, s4["motion_class"])
        out["imu"] = s4
    if present & 0x1:
        s5 = r.fields(S5_FIELDS)
        s5["scan_state"] = name(SCAN_STATES, s5["scan_state"])
        s5["recommendation"] = name(AVOID, s5["recommendation"])
        out["ultrasound"] = s5
    return out


def decode_hb(data):
    r = BitReader(data)
    return {"uptime_s": r.get(17), "state": name(STATES, r.get(3)),
            "faults": fault_names(r.get(13)), "events_lost": r.get(8)}


def describe_event(ev_type, f):
    if ev_type == 1:
        return "%s -> %s, faults %s" % (name(STATES, f["from"]),
                                        name(STATES, f["to"]),
                                        ",".join(fault_names(f["faults"])))
    if ev_type == 2:
        return "%s %s" % (OPS.get(f["opcode"], "op%d" % f["opcode"]),
                          "accepted" if f["accepted"] else "rejected")
    if ev_type == 5:
        return name(MOTION_CLASSES, f["class"])
    if ev_type == 7:
        return "%s -> %s" % (chr(ord("A") + f["symbol"]),
                             name(ACTIONS, f["action"]))
    if ev_type == 8:
        return name(REJECT_REASONS, f["reason"])
    if ev_type == 13:
        f = dict(f, recommendation=name(AVOID, f["recommendation"]))
    return " ".join("%s=%s" % kv for kv in f.items())


def decode_evt(data):
    r = BitReader(data)
    events = []
    while r.left() >= EVT_TYPE_BITS:
        ev_type = r.get(EVT_TYPE_BITS)
        if ev_type == 0:
            break
        ev_name, spec = EVENTS.get(ev_type, ("type%d" % ev_type, []))
        time_s = r.get(EVT_TIME_BITS) / 10.0
        fields = r.fields(spec)
        events.append({"t": time_s, "type": ev_name, "fields": fields,
                       "text": describe_event(ev_type, fields)})
    return events


def encode_command(words):
    """Build a command payload from command-line words."""
    op = words[0].lower()
    w = BitWriter()
    if op in ("start", "stop", "ping"):
        w.put({"start": 1, "stop": 2, "ping": 4}[op], OP_BITS)
        return w.to_bytes()
    if op == "bind":
        pairs = []
        for item in words[1:]:
            sym, _, act = item.partition("=")
            pairs.append((ord(sym.upper()) - ord("A"),
                          ACTIONS.index(act.lower())))
        w.put(3, OP_BITS)
        w.put(len(pairs), BIND_COUNT_BITS)
        for sym, act in pairs:
            w.put(sym, SYMBOL_BITS)
            w.put(act, ACTION_BITS)
        return w.to_bytes()
    if op == "raw":
        return bytes.fromhex("".join(words[1:]))
    raise ValueError("unknown command " + op)


def show(topic_tail, payload, sizes):
    stamp = time.strftime("%H:%M:%S")
    if topic_tail == "status":
        text = "online" if payload[:1] == b"\x01" else "OFFLINE"
        print("%s status %s" % (stamp, text))
        return
    decoders = {"tlm": decode_tlm, "hb": decode_hb, "evt": decode_evt}
    if topic_tail not in decoders:
        return
    try:
        decoded = decoders[topic_tail](payload)
    except ValueError as err:
        print("%s %s undecodable (%s): %s" % (stamp, topic_tail, err,
                                              payload.hex()))
        return
    if topic_tail == "evt":
        for ev in decoded:
            print("%s evt  [%7.1f] %-14s %s" % (stamp, ev["t"], ev["type"],
                                                ev["text"]))
    else:
        print("%s %-4s %s" % (stamp, topic_tail, json.dumps(decoded)))
    if sizes:
        as_json = len(json.dumps(decoded, separators=(",", ":")))
        print("         size: %d bytes binary vs %d bytes as JSON"
              % (len(payload), as_json))


def make_client(args, on_message):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("paho-mqtt is missing: pip install paho-mqtt")
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:          # paho-mqtt 1.x
        client = mqtt.Client()
    client.on_message = on_message
    client.connect(args.host, args.port, keepalive=30)
    client.subscribe(args.root + "/#")
    client.loop_start()
    return client


def cmd_monitor(args):
    prefix = args.root + "/"

    def on_message(_client, _userdata, msg):
        if msg.topic.startswith(prefix):
            show(msg.topic[len(prefix):], msg.payload, args.sizes)

    make_client(args, on_message)
    print("listening on %s:%d %s/# (Ctrl-C to quit)"
          % (args.host, args.port, args.root))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


def cmd_send(args):
    payload = encode_command(args.words)
    op = payload[0] >> 5
    acked = threading.Event()
    result = {}

    def on_message(_client, _userdata, msg):
        if msg.topic != args.root + "/evt":
            return
        for ev in decode_evt(msg.payload):
            if ev["type"] == "CMD_ACK" and ev["fields"]["opcode"] == op:
                result["ok"] = ev["fields"]["accepted"]
                acked.set()
            else:
                print("evt %-14s %s" % (ev["type"], ev["text"]))

    client = make_client(args, on_message)
    time.sleep(0.3)                 # let the subscription settle
    sent = time.monotonic()
    client.publish(args.root + "/cmd", payload)
    print("sent %s" % payload.hex())
    if acked.wait(args.timeout):
        ms = (time.monotonic() - sent) * 1000.0
        print("%s %s in %.0f ms" % (OPS.get(op, "op%d" % op),
                                    "accepted" if result["ok"]
                                    else "REJECTED", ms))
    else:
        print("no acknowledgement within %.1f s" % args.timeout)
    client.loop_stop()


def cmd_decode(args):
    show(args.topic, bytes.fromhex("".join(args.hex)), args.sizes)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--root", default="is04/car")
    p.add_argument("--timeout", type=float, default=2.0)
    sub = p.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("monitor")
    m.add_argument("--sizes", action="store_true")
    for c in ("start", "stop", "ping"):
        sub.add_parser(c)
    b = sub.add_parser("bind")
    b.add_argument("pairs", nargs="+", help="e.g. A=left B=right")
    r = sub.add_parser("raw")
    r.add_argument("hex", nargs="+")
    d = sub.add_parser("decode")
    d.add_argument("topic", choices=["tlm", "hb", "evt", "status"])
    d.add_argument("hex", nargs="+")
    d.add_argument("--sizes", action="store_true")
    args = p.parse_args()

    if args.cmd == "monitor":
        cmd_monitor(args)
    elif args.cmd == "decode":
        cmd_decode(args)
    else:
        args.words = [args.cmd] + getattr(args, "pairs", []) + \
            getattr(args, "hex", [])
        cmd_send(args)


if __name__ == "__main__":
    main()
