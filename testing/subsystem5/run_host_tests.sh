#!/bin/sh
# No hardware, kernel session, Arm toolchain or Pico SDK is needed.
set -eu
s5_test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
s5_repo_root=$(CDPATH= cd -- "$s5_test_dir/../.." && pwd)
s5_test_binary=$(mktemp "${TMPDIR:-/tmp}/s5-host-tests.XXXXXX")
trap 'rm -f "$s5_test_binary"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -I "$s5_repo_root/include" \
    "$s5_test_dir/host_tests.c" -o "$s5_test_binary"
"$s5_test_binary"
