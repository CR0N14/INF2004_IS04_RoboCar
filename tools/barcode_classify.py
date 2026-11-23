#!/usr/bin/env python3
"""Explain each barcode capture using the firmware's per-character classifier.

Reads the CSV that barcode_report.py writes beside a monitor log and, for every
capture of 29 or more runs, prints run widths, the narrow/wide pattern, the
per-character ratio margins, the decoded symbol and where it differs from the
expected '*A*'. Mirrors app_program/subsystem3/code39.c (CODE39_MIN_RATIO_PCT)
and the decoder's sliding window (up to 3 retries, 2 runs each).

    python tools/barcode_report.py LOG_monitor.jsonl     # writes the CSV
    python tools/barcode_classify.py LOG_monitor.barcode.csv
"""
import csv
import sys

RATIO = 1.15
SLIDES = 3
GUARD = 0x094
TABLE = dict(zip(
    [0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d, 0x10c, 0x04c, 0x01c,
     0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
     0x181, 0x0c1, 0x1c0, 0x091, 0x190, 0x0d0], "ABCDEFGHIJKLMNOPQRSTUVWXYZ"))
EXPECTED = "nWnnWnWnn" + "n" + "WnnnnWnnW" + "n" + "nWnnWnWnn"


def captures(path):
    caps, cur = [], None
    for r in csv.DictReader(open(path)):
        t, dark = int(r["time_ms"]), bool(int(r["mask"]) & 4)
        if r["state_before"] == "2" or r["state_after"] == "2":
            if cur is None:
                cur = dict(start=t, edge=t, dark=dark, runs=[], outcome="")
            elif dark != cur["dark"]:
                cur["runs"].append(t - cur["edge"])
                cur["edge"], cur["dark"] = t, dark
            if r["outcome"] not in ("none", "not_processed"):
                cur["outcome"] = r["outcome"]
        elif cur is not None:
            caps.append(cur)
            cur = None
    return caps


def char(w):
    bars, spaces = w[0::2], w[1::2]
    wb = sorted(range(5), key=lambda i: -bars[i])[:2]
    ws = max(range(4), key=lambda i: spaces[i])
    rb = min(bars[i] for i in wb) / max(bars[i] for i in range(5) if i not in wb)
    rs = spaces[ws] / max(spaces[i] for i in range(4) if i != ws)
    bits = "".join("W" if (i % 2 == 0 and i // 2 in wb) or
                   (i % 2 == 1 and i // 2 == ws) else "n" for i in range(9))
    return bits, rb, rs, spaces[ws]


def frame(w):
    parts = [char(w[s:s + 9]) for s in (0, 10, 20)]
    ok = all(rb >= RATIO and rs >= RATIO for _, rb, rs, _ in parts)
    gaps = (w[9] * RATIO <= min(parts[0][3], parts[1][3]) and
            w[19] * RATIO <= min(parts[1][3], parts[2][3]))
    bits = [p[0] for p in parts]
    val = lambda b: int(b.replace("W", "1").replace("n", "0"), 2)
    sym = None
    if ok and gaps and val(bits[0]) == GUARD and val(bits[2]) == GUARD:
        sym = TABLE.get(val(bits[1]))
    pattern = bits[0] + ("n" if gaps else "?") + bits[1] + ("n" if gaps else "?") + bits[2]
    return sym, pattern, [(round(rb, 2), round(rs, 2)) for _, rb, rs, _ in parts]


def main():
    for path in sys.argv[1:]:
        for c in captures(path):
            runs = c["runs"]
            print("\ncapture at %d ms: %d runs, firmware outcome %s"
                  % (c["start"], len(runs), c["outcome"] or "-"))
            if len(runs) < 29:
                print("  too few runs for a frame (needs 29)")
                continue
            print("  widths ms:", runs)
            for slide in range(min(SLIDES, (len(runs) - 29) // 2) + 1):
                w = runs[2 * slide:2 * slide + 29]
                results = [(name, *frame(seq))
                           for name, seq in (("forward", w), ("reverse", w[::-1]))]
                for name, sym, pattern, margins in results:
                    print("  slide %d %-7s %s -> %s  margins(bar,space) %s"
                          % (slide, name, pattern, sym or "reject", margins))
                if all(sym is None for _, sym, _, _ in results):
                    for name, _, pattern, _ in results:
                        diff = "".join("^" if a != b else " "
                                       for a, b in zip(pattern, EXPECTED))
                        print("  %-15s vs expected %s" % (name, EXPECTED))
                        print("  %-15s             %s" % ("", diff))
                    print("  margins below %.2f mean that character could not be"
                          " ranked reliably" % RATIO)
            chars = [sum(runs[s:s + 9]) for s in (0, 10, 20)]
            print("  character durations ms:", chars,
                  " speed change x%.2f" % (max(chars) / min(chars)))


if __name__ == "__main__":
    main()
