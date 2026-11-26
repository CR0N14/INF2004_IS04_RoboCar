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
`IR_PERIOD_MS=20` (cyclic acquisition). Either tracking sensor black commands straight 14%/14%;
turn duty follows forward duty (14%), recovery duty 10%. Actual wheel speed is not measured.
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

The diagnostic ring holds 256 records (5 KB), about 5.12 seconds at the default
20 ms sample interval. MQTT buffers remain enlarged. Check for inner-wheel stalling before floor tests.

Encoder-free forward monitoring treats no echo as unknown distance and resets
the consecutive-close counter without raising NO_ECHO. Valid close readings
still trigger obstacle handling. Scan-analysis failures can still stop the car.

Tracking drives straight at 14%/14% while either left or centre sees black.
Both-white continues straight for 20 ms, then a fresh white sample begins a
forward arc (9% inner wheel, 14% outer wheel) in the last known direction.
Left-only during that arc keeps correcting; centre-only returns immediately to
straight travel. Both-black retains junction confirmation. The arc is bounded
to 300 ms; failure then brakes and invokes the existing 10% pivot search, with
its 3-second timeout. Stale IR always stops motion. Barcode capture continues
during forward tracking arcs, but pivot recovery/stops invalidate capture.
IMU-ignore and no-echo settings are unchanged. Validate on the floor.

Temporary boot calibration (left/centre/right ADC counts): white 169/177/231,
black 3082/3120/3563. These are rounded midpoints of the two most recent sets
in 20261123_090720_407883_monitor.jsonl; earlier centre-black values varied.
MQTT white/black calibration remains available and overrides RAM values until
reboot. Verify stationary white/black recognition if sensor mounting changes.

### Staged corner recovery (14% forward demo)

After the existing 20 ms loss confirmation and 300 ms forward arc, recovery
uses a 350 ms pivot at 10%, then a wide search at 18% in the same direction.
Viewed facing forward, the code-labelled left sensor is physically right:
losing it selects a right search; losing centre selects a left search.
Direction stays latched throughout the arc and both pivot stages. The wide phase
has a 3000 ms total budget; failure stops the car. Either single tracking sensor can confirm recovery with fresh IR frames
20 ms apart. Both-black remains a crossing; junction turns and U-turns
still require centre-only confirmation. Recovery updates direction memory
from the confirming sensor before resuming straight tracking. Forward duty remains 14%.

U-turn: 250 ms forward clearance, 1600 ms left rotation at 14%, then the
wide search continuing left (the commanded U-turn direction). It stays MANOEUVRE until centre confirmation emits
TURN_DONE; no state-machine or MQTT protocol changes. 1600 ms is a floor-test
starting value, not a guaranteed 180 degrees without encoders/heading.

Validate on the corner in both approach directions, then bind A=uturn and
test at a junction. Check completion and direction physically, as crossing
the wrong branch can also produce centre detection. Test no-line timeout
and MQTT STOP during both gentle and wide phases. Record the monitor log.

### Barcode priority window

A decoder CAPTURING candidate (first eligible dark bar after clear background,
not a verified start character) allows straight 14% driving for at most 4000 ms.
It ends early when capture ends. Bars and repeated frames do not extend it.
Line-loss time continues accumulating; expired scans can enter recovery
immediately. Obstacle/state changes, stale IR, pickup and faults take priority.
Test a candidate with both tracking sensors white: straight for at most four
seconds, then recovery; repeat with an early decode/reject and with MQTT STOP.
False candidates can also cause this straight window; validate on the floor.

Steering polarity correction: code-left darkness now steers right, and a
fading centre steers left. The signed smooth controller, diagnostic steering,
barcode-window direction memory and recovery use this mapping consistently.
Commanded junction turns and U-turns keep their original motor directions.

Corner torque adjustment: smooth tracking now keeps base duty at 14%
(DEMO_TRACK_MIN_DUTY=14). Steering ratios are unchanged; the 70% steering
limit commands outer/inner 14%/4%, previously 11%/3%. Validate on the
floor without nudging; commanded duty does not establish wheel movement.

Current demo build uses IR_PERIOD_MS=10, 4000 ms barcode priority and
1600 ms initial U-turn rotation. Diagnostic ring capacity is unchanged;
inspect sequence gaps in the next monitor capture before resizing it.
Loss/reacquisition debounce remains 20 ms; steering gains are unchanged.
