#!/bin/sh
# Host test of the obstacle width scan against a simulated sensor.
set -eu
s5_test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
s5_repo_root=$(CDPATH= cd -- "$s5_test_dir/../.." && pwd)
s5_test_binary=$(mktemp "${TMPDIR:-/tmp}/s5-width-tests.XXXXXX")
trap 'rm -f "$s5_test_binary"' EXIT HUP INT TERM
"${CC:-cc}" -std=gnu11 -Wall -Wextra -Werror \
    "$s5_test_dir/width_tests.c" \
    "$s5_repo_root/app_program/subsystem5/demo_profile.c" \
    -lm -o "$s5_test_binary"
"$s5_test_binary"
