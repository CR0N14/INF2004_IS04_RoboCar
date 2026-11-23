#!/usr/bin/env python3
"""Record and decode the S2 encoder diagnostic (MOTION_MQTT_BENCH=1 builds).

The firmware publishes <root>/diag about every telemetry cycle: 40 bytes,
ten little-endian uint32 words:
  0 schema (1)   1 GPIO_IN (all pins)   2 encoder IRQ calls
  3 left count   4 right count          5 left valid steps   6 right valid steps
  7 left errors  8 right errors         9 bit0 IRQ enabled, bit1 enabled on core 1

    python tools/encoder_monitor.py                      # watch until Ctrl+C
    python tools/encoder_monitor.py --bench left --repeat 5
Every MQTT message is saved to local-only/buddy3/demo/logs/<time>_encoder.jsonl.
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import struct
import threading
import time

PINS = (("L_A", 4), ("L_B", 5), ("R_A", 16), ("R_B", 17))
BENCH = {"left": b"\xa0", "right": b"\xa1"}


def decode_encoder_diag(data):
    if len(data) < 40:
        raise ValueError("encoder diagnostic frame must be at least 40 bytes")
    w = struct.unpack_from("<10I", data)
    if w[0] not in (1, 2):
        raise ValueError("unknown encoder diagnostic schema %d" % w[0])
    s32 = lambda v: v - (1 << 32) if v & 0x80000000 else v
    return dict(schema=w[0], gpio_in="%08x" % w[1],
                pins={n: (w[1] >> p) & 1 for n, p in PINS},
                irq_calls=w[2], count_left=s32(w[3]), count_right=s32(w[4]),
                steps_left=w[5], steps_right=w[6],
                errors_left=w[7], errors_right=w[8],
                irq_enabled=bool(w[9] & 1), irq_core=1 if w[9] & 2 else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.20.10.6")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--root", default="is04/car")
    ap.add_argument("--logs", type=Path,
                    default=Path(__file__).resolve().parents[2] / "local-only/buddy3/demo/logs")
    ap.add_argument("--seconds", type=float, default=0, help="0: until Ctrl+C")
    ap.add_argument("--bench", choices=BENCH, help="drive this wheel at 20%% for 500 ms (STANDBY only)")
    ap.add_argument("--repeat", type=int, default=1, help="bench pulses, 2 s apart")
    args = ap.parse_args()
    import paho.mqtt.client as mqtt

    args.logs.mkdir(parents=True, exist_ok=True)
    path = args.logs / (dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_encoder.jsonl")
    lock = threading.Lock()
    ready = threading.Event()
    first, last = {}, {}
    pin_edges = {n: 0 for n, _ in PINS}
    frames = [0]

    with path.open("x", buffering=1) as log:
        def received(client, userdata, msg):
            row = dict(topic=msg.topic, hex=msg.payload.hex(), retained=bool(msg.retain),
                       utc=dt.datetime.now(dt.timezone.utc).isoformat())
            if msg.topic.endswith("/diag") and msg.payload[:4] in (b"\x01\0\0\0", b"\x02\0\0\0"):
                try:
                    d = decode_encoder_diag(msg.payload)
                    row["decoded"] = d
                    with lock:
                        if not first:
                            first.update(d)
                        if last:
                            for n in pin_edges:
                                pin_edges[n] += d["pins"][n] != last["pins"][n]
                        last.clear(); last.update(d)
                        frames[0] += 1
                    p = d["pins"]
                    print("%s pins L=%d%d R=%d%d  irq=%-6d  count L=%-6d R=%-6d  err L=%d R=%d  irq_en=%d core=%d"
                          % (row["utc"][11:23], p["L_A"], p["L_B"], p["R_A"], p["R_B"],
                             d["irq_calls"], d["count_left"], d["count_right"],
                             d["errors_left"], d["errors_right"], d["irq_enabled"], d["irq_core"]),
                          flush=True)
                except ValueError as exc:
                    row["error"] = str(exc)
            with lock:
                log.write(json.dumps(row) + "\n")

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = lambda c, u, f, rc, p: c.subscribe(args.root + "/#")
        client.on_subscribe = lambda c, u, mid, rcs, p: ready.set()
        client.on_message = received
        client.connect(args.host, args.port, 30)
        client.loop_start()
        try:
            if not ready.wait(10):
                raise SystemExit("MQTT subscription failed")
            print("recording to", path, flush=True)
            if args.bench:
                time.sleep(2)
                for i in range(args.repeat):
                    print("BENCH %s pulse %d/%d" % (args.bench, i + 1, args.repeat), flush=True)
                    client.publish(args.root + "/cmd", BENCH[args.bench], qos=0).wait_for_publish(3)
                    time.sleep(2)
                if not args.seconds:
                    time.sleep(2)
                    return
            deadline = time.monotonic() + args.seconds if args.seconds else float("inf")
            while time.monotonic() < deadline:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            client.loop_stop()
            client.disconnect()
            with lock:
                if not first:
                    print("SUMMARY no encoder diagnostic frames received "
                          "(is the MOTION_MQTT_BENCH build flashed?)")
                else:
                    print("SUMMARY frames=%d  irq_calls +%d  count L %+d R %+d  errors L +%d R +%d  "
                          "pin changes seen %s"
                          % (frames[0], last["irq_calls"] - first["irq_calls"],
                             last["count_left"] - first["count_left"],
                             last["count_right"] - first["count_right"],
                             last["errors_left"] - first["errors_left"],
                             last["errors_right"] - first["errors_right"],
                             json.dumps(pin_edges)))
                print("log:", path)


if __name__ == "__main__":
    main()
