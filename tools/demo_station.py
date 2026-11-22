#!/usr/bin/env python3
"""Encoder-free demo controls and JSONL capture. Commands never retained."""
import argparse
import datetime as dt
import json
from pathlib import Path
import struct
import threading
import time
import base_station


def decode_demo(data):
    if len(data) != 64:
        raise ValueError("Demo payload must be 64 bytes")
    w = struct.unpack("<16I", data)
    if w[0] != 1:
        raise ValueError("Unsupported demo schema")
    signed = lambda n: n - 65536 if n & 32768 else n
    return dict(mode="encoder_free_open_loop", encoder_available=False,
                imu_ignored=bool(w[1] & 128),
                ir_calibrated=bool(w[1] & 2), ir_fresh=bool(w[1] & 4),
                imu_ready=bool(w[1] & 8), pickup_latched=bool(w[1] & 16),
                height_valid=bool(w[1] & 32), commanded_motion=bool(w[1] & 64),
                uptime_ms=w[2], dark_mask=w[3], raw=list(w[4:7]),
                white=list(w[7:10]), black=list(w[10:13]),
                duty_left=signed(w[13] & 65535), duty_right=signed(w[13] >> 16),
                last_hump_mm=(w[14] & 65535) if w[1] & 32 else None,
                calibration_state={0:"waiting",1:"capturing_white",2:"capturing_black",3:"ready",4:"rejected"}.get(w[14] >> 16,"unknown"),
                motion_timestamp_ms=w[15], speed_mm_s=None, distance_cm=None)


def start_blockers(demo, tlm):
    reasons = []
    if not demo:
        reasons.append("No live demo telemetry")
    else:
        for key, message in (("ir_calibrated", "IR calibration incomplete"),
                             ("ir_fresh", "IR readings stale/invalid"),
                             ("imu_ready", "IMU not ready (check calibration, orientation and faults)")):
            if key == "imu_ready" and demo.get("imu_ignored"): continue
            if not demo.get(key): reasons.append(message)
        if demo.get("pickup_latched") and not demo.get("imu_ignored"): reasons.append("Pickup stop latched; place flat and run imu-zero")
    if not tlm:
        reasons.append("No live state telemetry")
    elif tlm.get("state") != "STANDBY":
        reasons.append("State is " + str(tlm.get("state")) + "; STOP first")
    return reasons


def status_line(demo, tlm):
    imu = tlm.get("imu", {})
    sonar = tlm.get("ultrasound", {})
    distance = sonar.get("distance_cm")
    sonar_text = "no echo/pending" if distance == 511 else (f"{distance}cm" if distance is not None else "?")
    return (f"state={tlm.get('state', '?')}/{tlm.get('nav_mode', '?')} "
            f"IR={demo.get('calibration_state', '?')} fresh={demo.get('ir_fresh', '?')} "
            f"mask={demo.get('dark_mask', '?')} "
            f"IMU_ignored={demo.get('imu_ignored', False)} "
            f"IMU_ready={demo.get('imu_ready', '?')} IMU={imu.get('imu_status', '?')} "
            f"pickup={demo.get('pickup_latched', '?')} "
            f"PWM={demo.get('duty_left', '?')}/{demo.get('duty_right', '?')} "
            f"sonar={sonar_text} faults={tlm.get('faults', '?')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="172.20.10.6")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--root", default="is04/car")
    ap.add_argument("--logs", type=Path, default=Path(__file__).resolve().parents[2] / "local-only/buddy3/demo/logs")
    sub = ap.add_subparsers(dest="command", required=True)
    mon = sub.add_parser("monitor")
    mon.add_argument("--raw", action="store_true", help="Print every telemetry frame")
    mon.add_argument("--interval", type=float, default=2, help="Summary interval in seconds (minimum 0.2)")
    sub.add_parser("status", help="Read readiness without sending commands")
    mon.add_argument("--seconds", type=float, default=0, help="0: until Ctrl+C")
    for name in ("white", "black", "imu-zero", "start", "stop", "ping"):
        sub.add_parser(name)
    b = sub.add_parser("bind")
    b.add_argument("pairs", nargs="+")
    args = ap.parse_args()
    import paho.mqtt.client as mqtt
    args.logs.mkdir(parents=True, exist_ok=True)
    path = args.logs / (dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + args.command + ".jsonl")
    ready, acked = threading.Event(), threading.Event()
    result = {}
    latest = {}
    sent = threading.Event()
    exit_code = 0
    log_lock = threading.Lock()
    expected = 5 if args.command in ("white","black","imu-zero") else {"start":1,"stop":2,"bind":3,"ping":4}.get(args.command)
    with path.open("x", buffering=1) as log:
        def save(row):
            row["utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
            with log_lock:
                log.write(json.dumps(row) + "\n")
        def connect(client, userdata, flags, reason, properties):
            if reason == 0: client.subscribe(args.root + "/#")
        def subscribed(client, userdata, mid, reasons, properties):
            if all(not r.is_failure for r in reasons): ready.set()
        def received(client, userdata, msg):
            row = dict(topic=msg.topic, hex=msg.payload.hex(), retained=bool(msg.retain))
            tail = msg.topic.rsplit("/",1)[-1]
            try:
                if tail == "demo": row["decoded"] = decode_demo(msg.payload)
                elif tail in ("tlm","hb","evt"): row["decoded"] = getattr(base_station,"decode_"+tail)(msg.payload)
                if tail in ("demo", "tlm") and not msg.retain:
                    latest[tail] = (time.monotonic(), row["decoded"])
                if tail == "evt" and not msg.retain and sent.is_set():
                    for event in row.get("decoded",[]):
                        f = event.get("fields",{})
                        if event.get("type") == "CMD_ACK" and f.get("opcode") == expected:
                            result["accepted"] = bool(f["accepted"]); acked.set()
                if tail == "evt" or (args.command == "monitor" and args.raw and tail in ("demo","tlm")):
                    print(tail, json.dumps(row.get("decoded",row)), flush=True)
            except (ValueError, IndexError, KeyError) as exc: row["error"] = str(exc)
            save(row)
        def snapshot():
            now = time.monotonic()
            return tuple(value[1] if value and now - value[0] < 2 else {}
                         for value in (latest.get("demo"), latest.get("tlm")))
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect, client.on_subscribe, client.on_message = connect, subscribed, received
        client.connect(args.host,args.port,30);client.loop_start()
        try:
            if not ready.wait(10): raise RuntimeError("MQTT subscription failed")
            if args.command == "monitor":
                deadline = time.monotonic() + args.seconds if args.seconds else float("inf")
                next_summary = 0
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if not args.raw and now >= next_summary:
                        d, t = snapshot()
                        print(status_line(d, t), flush=True)
                        reasons = start_blockers(d, t)
                        print("START readiness: " + ("; ".join(reasons) if reasons else "ready"), flush=True)
                        next_summary = now + max(.2, args.interval)
                    time.sleep(.1)
            elif args.command == "status":
                time.sleep(1)
                d, t = snapshot()
                print(status_line(d, t))
                print("START readiness: " + ("; ".join(start_blockers(d, t)) or "ready"))
                print(json.dumps(dict(demo=d, telemetry=t), indent=2))
            else:
                codes = {"white":b"\xb1","black":b"\xb2","imu-zero":b"\xb3"}
                payload = codes.get(args.command)
                if payload is None: payload = base_station.encode_command([args.command] + getattr(args,"pairs",[]))
                save(dict(command=args.command,payload_hex=payload.hex()))
                sent.set()
                client.publish(args.root+"/cmd",payload,qos=0,retain=False).wait_for_publish(3)
                if not acked.wait(5): raise RuntimeError("No acknowledgement; inspect monitor before retrying")
                print("ACCEPTED" if result["accepted"] else "REJECTED")
                if not result["accepted"]:
                    time.sleep(.5)
                    d, t = snapshot()
                    if args.command == "start":
                        print("Observed blockers: " + ("; ".join(start_blockers(d, t)) or "none in latest snapshot; inspect event log"))
                    else:
                        print("Check state/readiness with: python tools/demo_station.py status")
                    exit_code = 2
                if result["accepted"] and args.command in codes: print("Acceptance is only request delivery. Wait for calibration/IMU readiness in monitor.")
        except KeyboardInterrupt: pass
        except (RuntimeError, TimeoutError) as exc:
            print("ERROR:", exc)
            exit_code = 1
        finally:
            client.disconnect();client.loop_stop()
            print("Saved",path)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
