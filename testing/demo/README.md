# Encoder-free demo validation

Run from the project root:

```sh
bash testing/demo/run_tests.sh
python3 testing/demo/test_station.py
python3 testing/subsystem3/host/run_tests.py
```

The C runner uses Clang with address/undefined-behavior sanitizers. It checks
motion and state transitions, pending instruction consumption, sonar capture,
stationary motor inhibition and timed-drive stopping. Generated binaries are
created in the temporary directory. These are software tests, not evidence of
successful physical barcode decoding or junction alignment.

## Build profiles

From `build_make`, the ordinary encoder-free demo is:

```sh
make PICO_SDK_PATH=/home/koh/INF2004/pico-sdk NET=lwip MQTT=1 TEST=0 DEMO=1 -j8
```

Adjust the SDK path for your machine. Defaults are `DEMO_DUTY=14` and
`IR_PERIOD_MS=10` (cyclic acquisition). Left-only correction is 7%/14%;
turn duty is 20%, recovery duty 18%. Actual wheel speed is not measured.
IMU enforcement remains on by default; explicitly add `DEMO_NO_IMU=1` only
for the current IMU-disabled demonstration. Pickup stopping is then unavailable.

Optional, mutually exclusive test profiles:

| Option | Behavior |
|---|---|
| `BARCODE_BENCH=1` | Stationary hand scan; START rejected and motors inhibited |
| `BARCODE_DRIVE_MS=3000` | Timed straight pass; decoded actions logged only |
| `JUNCTION_TEST=1` | Inject mapped A on START; execute once at confirmed junction |

All default to zero. Junction test requires A bound before START. It bypasses
optical decoding and keeps barcode_seen false. Centre-only black for 100 ms
qualifies approach, then both tracking sensors black for 100 ms confirms the
junction. Completion resumes tracking; use MQTT STOP to end the run. This rule
does not measure heading and physical exit alignment still needs validation.
Profile changes invalidate the shared build object tree automatically.

## MQTT tools and evidence

Install paho-mqtt in a separate virtual environment. `tools/demo_station.py`
provides monitor, status, white, black, bind, start and stop. Calibration is lost
on reset; accepted calibration ACK means request delivery, not completed capture.
Use `--host` before the subcommand to override the broker address.

`tools/barcode_report.py LOG.jsonl` writes a CSV beside the recording.
`tools/barcode_classify.py LOG.barcode.csv` explains recorded widths.
`tools/diag_health.py LOG.jsonl` reports diagnostic loss and delay.
`tools/encoder_monitor.py` is for a separate encoder-diagnostic firmware.
IRB1 diagnostic records contain raw/normalized ADC, mask, decoder state/count,
outcome, symbol/action, PWM and sequence. Missing sequences mean the received
trace is incomplete. Buffers are bounded; MQTT capture is not guaranteed lossless.

Store personal logs, UF2s and experiment notes in the sibling
`../local-only/buddy3/` directory, not under `testing/`.
