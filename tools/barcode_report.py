#!/usr/bin/env python3
"""Summarize buffered IRB1 diagnostics; write CSV beside the input JSONL."""
import argparse,csv,json
from pathlib import Path
from demo_station import decode_ir_diag
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument("log",type=Path)
a=ap.parse_args();samples=[]
for line in a.log.read_text().splitlines():
    try:
        row=json.loads(line)
        if row.get("topic", "").endswith("/diag"):
            data=bytes.fromhex(row.get("hex", ""))
            if data[:4]==b"IRB1":samples.extend(decode_ir_diag(data))
    except (ValueError,KeyError) as exc: print("Skipped invalid record:",exc)
if not samples:raise SystemExit("No IRB1 diagnostics. Flash the diagnostic firmware and record monitor again.")
out=a.log.with_suffix(".barcode.csv")
with out.open("w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=samples[0].keys());w.writeheader();w.writerows(samples)
print("Saved",out,"samples",len(samples))
previous=None;edge=None;dark=None
for s in samples:
    if previous and (s["sequence"]!=previous["sequence"]+1 or s["time_ms"]<previous["time_ms"]):
        print("GAP/RESET",previous["sequence"],"->",s["sequence"]);edge=None;dark=None
    if s["state_before"]==2 or s["state_after"]==2:
        level=bool(s["mask"]&4)
        if dark is not None and level!=dark and edge is not None:
            print(s["time_ms"],"black" if dark else "white","run",s["time_ms"]-edge,"ms")
        if dark is None or level!=dark:edge=s["time_ms"]
        dark=level
    else:edge=None;dark=None
    if s["outcome"] not in ("none","not_processed") and (s["state_before"]==2 or s["outcome"]=="accepted"):
        print(s["time_ms"],s["outcome"],"runs",s["runs_before"],"->",s["runs_after"],"ADC",s["raw_right"],"PWM",s["duty_left"],s["duty_right"])
    previous=s
