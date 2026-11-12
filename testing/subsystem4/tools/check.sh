#!/bin/bash
#
# check.sh - the Subsystem 4 checks to run before every report.
#
#   testing/subsystem4/tools/check.sh
#
#   1. Lint: every Subsystem 4 .c file, compiled with the default build's
#      own command line (taken from "make -n", so the defines and include
#      paths cannot drift) plus the stricter warnings below, once with
#      APP_TEST=0 and once with APP_TEST=1.  The kernel and Pico SDK
#      headers are passed as -isystem, so only Subsystem 4's code is
#      judged.  Any warning fails.
#   2. The four builds: default, NET=lwip MQTT=1, TEST=1 and
#      CONSOLE=uart WIFI=none.  Each must build with no warning in
#      Subsystem 4's files (other warnings are counted).
#   3. The host unit tests (testing/subsystem4/host, "make").
#
# Left out of the lint on purpose:
#   -Wcast-qual          tm_printf() takes a non-const UB *, so every
#                        format string is cast (the kernel's idiom, also
#                        in the shared test_report.h)
#   -Wbad-function-cast  imu_task.c stores the imu_cal_result_t from
#                        imu_cal_finish() as a uint8_t on purpose
#   -Wconversion         GCC's version warns on every bit-field store in
#                        publish(), although each value is clamped to its
#                        field first; the host build compiles the logic
#                        with clang's -Wconversion instead
#
# Builds go to build_make/ as usual; the S4 objects are deleted before
# each build (make 3.81 keeps one-second timestamps).  Nothing is
# committed.

set -e -o pipefail

ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
BUILD="$ROOT/build_make"
SRC="$ROOT/app_program/subsystem4"
HOST="$ROOT/testing/subsystem4/host"
OBJ_DIR="$BUILD/mtkernel_3/app_program/subsystem4"
JOBS=${JOBS:-8}

LINT_WARN="-Wall -Wextra -Wpedantic -Wshadow -Wsign-conversion \
-Wcast-align -Wpointer-arith -Wstrict-prototypes -Wmissing-prototypes \
-Wmissing-declarations -Wredundant-decls -Wswitch-default -Wswitch-enum \
-Wundef -Wfloat-equal -Wdouble-promotion -Wformat=2 -Wlogical-op \
-Wduplicated-cond -Wduplicated-branches -Wnull-dereference \
-Wjump-misses-init -Wvla -Wwrite-strings -Wold-style-definition \
${LINT_EXTRA:-}"

TMP=$(mktemp -d "${TMPDIR:-/tmp}/s4chk.XXXXXX")
trap 'rm -rf "$TMP"' EXIT
fail=0

echo "=== Lint (Subsystem 4 sources, default build flags + stricter warnings)"
# The compile line make would use, minus dependency output and the
# object file
base=$(cd "$BUILD" &&
       make -n -B mtkernel_3/app_program/subsystem4/imu_logic.o 2>/dev/null |
       grep -m1 'arm-none-eabi-gcc' |
       sed -E 's/ -MMD -MP//; s/ -MF"[^"]*"//; s/ -MT"[^"]*"//;
               s/ -c -o "[^"]*" "[^"]*"$//; s/ -I"/ -isystem "/g')
case "$base" in
*-DAPP_TEST=0*) ;;
*) echo "cannot read the compile line from make -n"; exit 1 ;;
esac
# A diagnostic counts if it points into app_program/subsystem4/ itself.
# Teammates' headers that S4 includes by a relative path (e.g.
# ../subsystem1/sm_types.h) cannot be made -isystem; their warnings are
# listed, not counted.
S4_DIAG='^[^ ]*subsystem4/[^/ ]+:[0-9]+:[0-9]+: (warning|error)'
n_files=0
: > "$TMP/other.txt"
for t in 0 1; do
    cmd=${base/-DAPP_TEST=0/-DAPP_TEST=$t}
    for f in "$SRC"/*.c; do
        n_files=$((n_files + 1))
        rc=0
        (cd "$BUILD" && eval "$cmd -fsyntax-only $LINT_WARN \"$f\"") \
            > "$TMP/lint.txt" 2>&1 || rc=$?
        if [ $rc -ne 0 ] || grep -qE "$S4_DIAG" "$TMP/lint.txt"; then
            echo "APP_TEST=$t $(basename "$f"):"
            cat "$TMP/lint.txt"
            fail=1
        fi
        grep -E ": (warning|error):" "$TMP/lint.txt" |
            grep -vE "$S4_DIAG" | sed -E 's|^.*/app_program/||' \
            >> "$TMP/other.txt" || true
    done
done
[ $fail -eq 0 ] && echo "clean: $n_files compiles ($((n_files / 2)) files x" \
                        "APP_TEST 0/1), 0 warnings in Subsystem 4"
if [ -s "$TMP/other.txt" ]; then
    echo "not judged, outside Subsystem 4:"
    sort "$TMP/other.txt" | uniq -c | sed 's/^/  /'
fi

for args in "" "NET=lwip MQTT=1" "TEST=1" "CONSOLE=uart WIFI=none"; do
    echo "=== make ${args:-(default)}"
    rm -f "$OBJ_DIR"/*.o "$OBJ_DIR"/*.d
    # shellcheck disable=SC2086
    if ! (cd "$BUILD" && make $args -j"$JOBS") > "$TMP/build.txt" 2>&1; then
        tail -30 "$TMP/build.txt"
        echo "BUILD FAILED"
        fail=1
        continue
    fi
    warn=$(grep -E "warning:" "$TMP/build.txt" || true)
    s4=$(echo "$warn" | grep -c "subsystem4/" || true)
    all=$( [ -z "$warn" ] && echo 0 || echo "$warn" | wc -l | tr -d ' ')
    img=$(grep -oE "mtk3pico_[a-z0-9_]+\.uf2" "$TMP/build.txt" | tail -1)
    echo "built ${img:-?}: $all warnings, $s4 in Subsystem 4"
    if [ "$s4" -ne 0 ]; then
        echo "$warn" | grep "subsystem4/"
        fail=1
    fi
done

echo "=== Host tests"
if ! make -C "$HOST" > "$TMP/host.txt" 2>&1; then
    fail=1
fi
grep -E "FAIL|passed|failed" "$TMP/host.txt" | tail -5

[ $fail -eq 0 ] && echo "=== ALL CHECKS PASSED" || echo "=== CHECKS FAILED"
exit $fail
