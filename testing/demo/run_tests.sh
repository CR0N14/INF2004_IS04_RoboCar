#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
binary=$(mktemp /tmp/robocar-demo-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g testing/demo/test_demo.c app_program/subsystem2/demo_motion.c app_program/subsystem1/sm_logic.c app_program/subsystem5/demo_profile.c app_program/subsystem4/demo_pickup.c -o "$binary"
"$binary"

clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g testing/demo/test_sonar_capture.c -o "$binary"
"$binary"
echo "Ultrasonic capture: boundary, delayed-task, invalid-edge and recovery checks passed"
