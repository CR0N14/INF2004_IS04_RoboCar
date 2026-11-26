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

clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -DENCODER_FREE_DEMO=1 -DDEMO_BARCODE_BENCH=1 testing/demo/test_barcode_bench.c app_program/subsystem2/demo_motion.c -o "$binary"
"$binary"
echo "Stationary barcode: motor inhibit checks passed"

clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -DENCODER_FREE_DEMO=1 -DDEMO_BARCODE_DRIVE_MS=3000u testing/demo/test_barcode_drive.c app_program/subsystem2/demo_motion.c -o "$binary"
"$binary"

clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -DENCODER_FREE_DEMO=1 testing/demo/test_bypass.c app_program/subsystem2/demo_motion.c app_program/subsystem5/demo_profile.c -o "$binary"
"$binary"

clang -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -DENCODER_FREE_DEMO=1 -DDEMO_SMOOTH_TRACK=1 testing/demo/test_smooth_track.c app_program/subsystem2/demo_motion.c -o "$binary"
"$binary"

clang -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined -DENCODER_FREE_DEMO=1 testing/demo/test_sonar_profile.c app_program/subsystem5/demo_profile.c -lm -o "$binary"
"$binary"
