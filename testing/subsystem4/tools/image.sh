#!/bin/bash
#
# image.sh - build the Subsystem 4 firmware images.
#
#   testing/subsystem4/tools/image.sh all
#       the four images listed in S4_plan.md:
#         s4_accel_diag      default build, S4_CFG_CONSOLE S4_CONSOLE_ACCDIAG
#         s4_stage2_lsm303d  default build, S4_CFG_CONSOLE S4_CONSOLE_RAW
#         s4_bench_readout   default build, S4_CFG_CONSOLE S4_CONSOLE_BENCH,
#                            S4_CFG_BENCH_MOTORS 1, S4_CFG_REPORT_ONLY 1
#         s4_unit_tests      make TEST=1, imu_cfg.h defaults
#   testing/subsystem4/tools/image.sh s4_bench_readout
#       one of the four
#   testing/subsystem4/tools/image.sh NAME [SWITCH=VALUE ...] [-- MAKE_ARGS]
#       anything else, e.g. "my_image S4_CFG_I2C_100KHZ=1 -- TEST=1"
#
# The switches cannot be passed to make ("make CFLAGS+=-D..." replaces the
# makefile's own CFLAGS), so each build edits imu_cfg.h, builds, and puts
# imu_cfg.h back byte for byte, also when the build fails or is stopped.
#
# make 3.81 (macOS) keeps one-second timestamps.  A switch changed in the
# same second as the last compile can leave the old object in place, and
# once gave a bench image built from stale objects.  So the S4 objects
# and the image are deleted before every build, and again after imu_cfg.h
# is restored, so the next plain "make" cannot reuse them either.
#
# Each build must finish with no warning in Subsystem 4's files, and the
# script prints the console functions the image actually linked: proof of
# which readout is in it.
#
# Images go to $S4_IMAGE_DIR (default: ~/Documents/INF2004 docs/s4_images).
# Nothing is committed.

set -e -o pipefail

ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
BUILD="$ROOT/build_make"
CFG="$ROOT/app_program/subsystem4/imu_cfg.h"
OBJ_DIR="$BUILD/mtkernel_3/app_program/subsystem4"
OUT=${S4_IMAGE_DIR:-"$HOME/Documents/INF2004 docs/s4_images"}
JOBS=${JOBS:-8}

die() { echo "image.sh: $*" >&2; exit 1; }

[ -f "$CFG" ] || die "no $CFG"
[ -d "$OUT" ] || die "no output folder '$OUT' (set S4_IMAGE_DIR)"
NM=$(command -v arm-none-eabi-nm) || die "arm-none-eabi-nm not on PATH"

TMP=$(mktemp -d "${TMPDIR:-/tmp}/s4img.XXXXXX")
cp -p "$CFG" "$TMP/imu_cfg.h.orig"

clean_objects() {
    # $1: the image's base name (EXE_FILE)
    rm -f "$OBJ_DIR"/*.o "$OBJ_DIR"/*.d
    rm -f "$BUILD/$1.elf" "$BUILD/$1.map" "$BUILD/$1.uf2"
}

# A plain copy gives imu_cfg.h a new timestamp, so anything built from
# the edited copy is older than it
restore() {
    cmp -s "$TMP/imu_cfg.h.orig" "$CFG" || cp "$TMP/imu_cfg.h.orig" "$CFG"
    cmp -s "$TMP/imu_cfg.h.orig" "$CFG"
}

finish() {
    local st=$?
    if restore; then
        [ -z "$EXE" ] || clean_objects "$EXE"
        rm -rf "$TMP"
    else
        echo "image.sh: WARNING: could not restore $CFG" \
             "(original in $TMP)" >&2
        st=1
    fi
    exit $st
}
trap finish EXIT
trap 'exit 130' INT TERM

preset() {
    case "$1" in
    s4_accel_diag)      echo "S4_CFG_CONSOLE=S4_CONSOLE_ACCDIAG" ;;
    s4_stage2_lsm303d)  echo "S4_CFG_CONSOLE=S4_CONSOLE_RAW" ;;
    s4_bench_readout)   echo "S4_CFG_CONSOLE=S4_CONSOLE_BENCH" \
                             "S4_CFG_BENCH_MOTORS=1 S4_CFG_REPORT_ONLY=1" ;;
    s4_unit_tests)      echo "-- TEST=1" ;;
    *)                  return 1 ;;
    esac
}

# Set one "#define NAME VALUE" in imu_cfg.h; exactly one such line must
# exist, and it must read VALUE afterwards
set_switch() {
    local name=${1%%=*}
    local value=${1#*=}
    local n
    n=$(grep -cE "^#define[[:space:]]+$name[[:space:]]" "$CFG" || true)
    [ "$n" = 1 ] || die "$name: $n definitions in imu_cfg.h, want 1"
    perl -pi -e "s/^(#define\\s+$name\\s+)\\S+/\${1}$value/" "$CFG"
    grep -qE "^#define[[:space:]]+$name[[:space:]]+$value([[:space:]]|\$)" \
        "$CFG" || die "$name did not change to $value"
}

# Value of a switch as imu_cfg.h now has it
switch_value() {
    sed -nE "s/^#define[[:space:]]+$1[[:space:]]+([^[:space:]]+).*/\\1/p" \
        "$CFG"
}

build_one() {
    local name=$1
    shift
    local switches=()
    local make_args=()
    while [ $# -gt 0 ]; do
        if [ "$1" = "--" ]; then
            shift
            make_args=("$@")
            break
        fi
        switches+=("$1")
        shift
    done

    echo "=== $name: ${switches[*]:-imu_cfg.h defaults}" \
         "${make_args[*]:+(make ${make_args[*]})}"
    cp "$TMP/imu_cfg.h.orig" "$CFG"
    for s in ${switches[@]+"${switches[@]}"}; do
        set_switch "$s"
    done
    local console
    console=$(switch_value S4_CFG_CONSOLE)

    EXE=$(cd "$BUILD" && make ${make_args[@]+"${make_args[@]}"} -pn \
              2>/dev/null | sed -n 's/^EXE_FILE := //p' | tail -1)
    [ -n "$EXE" ] || die "cannot work out the image name"
    clean_objects "$EXE"

    if ! (cd "$BUILD" && make ${make_args[@]+"${make_args[@]}"} -j"$JOBS") \
            > "$TMP/$name.log" 2>&1; then
        tail -30 "$TMP/$name.log"
        die "$name: build failed (log $TMP/$name.log)"
    fi
    local warn
    warn=$(grep -E "warning:" "$TMP/$name.log" || true)
    if echo "$warn" | grep -q "subsystem4/"; then
        echo "$warn" | grep "subsystem4/"
        die "$name: warnings in Subsystem 4"
    fi
    [ -z "$warn" ] ||
        echo "  note: $(echo "$warn" | wc -l | tr -d ' ') warnings" \
             "outside Subsystem 4"

    # Which readout did it link?  (--gc-sections drops uncalled ones; the
    # bench lines are compiled out of imu_console_status otherwise)
    local syms hex status_size
    syms=$("$NM" -S "$BUILD/$EXE.elf" |
           awk '$4 ~ /^imu_console_/ { print $4, $2 }')
    echo "  console functions linked:" \
         "$(echo "$syms" | awk '{ printf "%s ", $1 }')"
    hex=$(echo "$syms" | awk '$1 == "imu_console_status" { print $2 }')
    status_size=$((16#${hex:-0}))
    has() { echo "$syms" | awk '{ print $1 }' | grep -qx "$1"; }
    case "$console" in
    S4_CONSOLE_RAW)
        has imu_console_raw && ! has imu_console_accdiag ||
            die "$name: Stage 2 readout not linked" ;;
    S4_CONSOLE_ACCDIAG)
        has imu_console_accdiag && ! has imu_console_raw ||
            die "$name: diagnosis not linked" ;;
    S4_CONSOLE_BENCH)
        ! has imu_console_raw && ! has imu_console_accdiag &&
            [ "$status_size" -gt 64 ] ||
            die "$name: bench readout not linked" ;;
    *)
        ! has imu_console_raw && ! has imu_console_accdiag &&
            [ "$status_size" -le 64 ] ||
            die "$name: a readout is linked into a console-off build" ;;
    esac
    echo "  check: $console build links the right readout" \
         "(imu_console_status ${status_size} bytes)"

    cp "$BUILD/$EXE.uf2" "$OUT/$name.uf2"
    echo "  $OUT/$name.uf2  $(wc -c < "$OUT/$name.uf2" | tr -d ' ') bytes" \
         " sha256 $(shasum -a 256 "$OUT/$name.uf2" | cut -c1-16)" \
         " from $EXE.uf2"

    restore || die "could not restore imu_cfg.h (original in $TMP)"
    clean_objects "$EXE"
    EXE=
}

EXE=
[ $# -ge 1 ] || die "usage: image.sh all | NAME [SWITCH=VALUE ...] [-- MAKE_ARGS]"
if [ "$1" = "all" ]; then
    for name in s4_accel_diag s4_stage2_lsm303d s4_bench_readout s4_unit_tests
    do
        # shellcheck disable=SC2046
        build_one "$name" $(preset "$name")
    done
elif [ $# -eq 1 ] && preset "$1" > /dev/null; then
    # shellcheck disable=SC2046
    build_one "$1" $(preset "$1")
else
    build_one "$@"
fi
restore && echo "imu_cfg.h restored unchanged"
