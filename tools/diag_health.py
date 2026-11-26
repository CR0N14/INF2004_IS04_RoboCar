#!/usr/bin/env python3
"""Report IR diagnostic loss and delay in a monitor log.

    python tools/diag_health.py LOG_monitor.jsonl

Each IRB1 packet carries 3 consecutive samples. A sequence jump between
packets is lost samples. Delay is how far arrival falls behind the car's
clock, relative to the least-delayed packet; a delay that climbs towards
5 s means the car's 256-sample buffer is filling and will drop samples.
"""
import datetime as dt
import json
import sys

for path in sys.argv[1:]:
    rows = []
    for line in open(path):
        r = json.loads(line)
        if r.get("topic", "").endswith("/diag") and r.get("hex", "").startswith("49524231") and isinstance(r.get("decoded"), list) and r["decoded"]:
            d = r["decoded"]
            rows.append((dt.datetime.fromisoformat(r["utc"]).timestamp(),
                         d[0]["sequence"], d[-1]["sequence"], d[-1]["time_ms"]))
    if not rows:
        print(path, ": no IRB1 diagnostics")
        continue
    lost = [b[1] - a[2] - 1 for a, b in zip(rows, rows[1:]) if b[1] - a[2] != 1]
    offsets = [t - c / 1000.0 for t, _, _, c in rows]
    base = min(offsets)
    delay = sorted(o - base for o in offsets)
    span = rows[-1][3] - rows[0][3]
    print(path)
    print("  packets %d over %.0f s, samples expected %d"
          % (len(rows), span / 1000.0, rows[-1][2] - rows[0][1] + 1))
    print("  gaps %d, samples lost %d" % (len(lost), sum(x for x in lost if x > 0)))
    print("  delay median %.2f s, 99th percentile %.2f s, max %.2f s"
          % (delay[len(delay) // 2], delay[int(len(delay) * 0.99)], delay[-1]))
    print("  verdict:", "OK" if not lost and delay[-1] < 1.0 else
          "LOSS" if lost else "DELAY BUILDING")
