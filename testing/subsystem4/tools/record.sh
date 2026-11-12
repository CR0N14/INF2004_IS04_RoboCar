#!/bin/bash
#
# record.sh - record the Pico's USB console to a log, from before it boots.
#
#   testing/subsystem4/tools/record.sh NAME
#       -> testing/subsystem4/logs/YYYY-MM-DD_NAME.txt (today's date)
#
# Start it FIRST, then flash or reset the Pico.  It waits for the Pico's
# serial port (/dev/cu.usbmodem*), connects within 0.1 s of it appearing,
# and connects again by itself whenever the Pico reboots (flashing, reset,
# unplugging).  Each connection is marked in the log with the time, so a
# late start shows.  The text is also shown on screen.  Ctrl-C stops it
# and prints a short summary.
#
# Why: while nothing reads the port the Pico keeps only its first 4415
# bytes of output and drops the rest (usb_console.c), and three S4 logs
# lost text that way.
#
# Only one program can usefully read the port: close any other serial
# monitor first.  It never overwrites a log; pick a new NAME instead.

set -u

[ $# -eq 1 ] || { echo "usage: record.sh NAME   (e.g. s4-u_unit_tests)" >&2
                  exit 1; }
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
LOG="$ROOT/testing/subsystem4/logs/$(date +%F)_$1.txt"
MARK="--- record.sh:"

if [ -e "$LOG" ]; then
    echo "record.sh: $LOG already exists; use another NAME" >&2
    exit 1
fi

summary() {
    local n_test n_fail
    echo
    echo "record.sh: stopped.  Saved $LOG"
    echo "  $(wc -l < "$LOG" | tr -d ' ') lines," \
         "$(grep -c "^$MARK connected" "$LOG") connection(s)"
    n_test=$(grep -c "\[TEST\]" "$LOG")
    if [ "$n_test" -gt 0 ]; then
        n_fail=$(grep "\[TEST\]" "$LOG" | grep -c " FAIL ")
        echo "  $n_test [TEST] lines, $n_fail FAIL"
    fi
    exit 0
}
trap summary INT TERM

# One connection: open the port once, switch it to raw mode (no echo
# back to the Pico, line ends kept as sent), copy until the Pico goes away
connect() {
    exec 3<"$1" 2>/dev/null || return 1
    echo "$MARK connected to $1 at $(date +%T) ---"
    stty raw -echo <&3 2>/dev/null
    cat <&3 2>/dev/null
    echo "$MARK $1 closed at $(date +%T) ---"
}

echo "record.sh: recording to $LOG"
echo "record.sh: waiting for the Pico (/dev/cu.usbmodem*); flash or reset" \
     "it now.  Ctrl-C to stop."
: > "$LOG"
shopt -s nullglob
while true; do
    for p in /dev/cu.usbmodem*; do
        ( connect "$p" )
    done
    sleep 0.1
done | tee -a "$LOG"
