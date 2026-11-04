# Subsystem 5 — Week 10 test log

**Owner:** Buddy 5, student 2303571. **Scope:** servo, ultrasonic scan, telemetry,
basic State Machine events and three-point demo profile.

## How these tests work

`subsystem5_test.c` compiles a private copy of the production S5 implementation
with mocked hardware, mutex, task, delay and event calls. The same tests run on
host and in the `TEST=1` image. They never change the running S5 snapshot, touch
GPIO/PWM, start another owning task, or enqueue real State Machine events.
The copy tests software control flow, not real scheduler or peripheral behavior.
Injected `ERROR S5 ...` lines in the private tests are expected; assess the
`[TEST] S5-Uxx PASS|FAIL` lines and final summary.

Run from the repository root:

```sh
testing/subsystem5/run_host_tests.sh
cd build_make
PICO_SDK_PATH=/Users/cayden/pico/pico-sdk make -j8
PICO_SDK_PATH=/Users/cayden/pico/pico-sdk make TEST=1 -j8
```

Use your local SDK path if different. Flash the `_test.uf2` to run all subsystem
tests on target. The normal task still owns all S5 hardware in either firmware.
Capture USB output before resetting; save it under `testing/subsystem5/logs/`.
A successful build is not an on-target test result. No on-target run is claimed
by the host results below. Follow `testing/README.md` for evidence and append-only
results. The user requested no commits, so this check-in records the baseline
commit plus dirty source SHA-256 hashes rather than claiming a committed test
revision. Commit and re-run before pushing when authorized.

## Automated cases

S5-U01 is **retired**: its original `tk_get_tid() > 0` check proved only that the
shared test runner was a task. Its ID is not reused. S5-U02 was an empty manual
placeholder and is now the servo case below.

| ID | What is tested | Method / input | Expected result |
|---|---|---|---|
| S5-U05 | Task/startup readiness | Mock mutex creation failure/success and task-start callback; initialize both mock drivers; inject each init failure | Start rejects absent mutex; priority 14/any core; READY(ULTRASOUND) only after success; each failure publishes ERROR and SELFTEST_FAIL(ULTRASOUND), then exits |
| S5-U06 | Telemetry getter | Private snapshot with mocked mutex; null pointer, missing mutex, lock/unlock failures; modify returned copy | E_PAR/E_OBJ and kernel errors propagated; IDLE/no-echo/STOP defaults; copy independent of stored snapshot; no unlock after failed lock |
| S5-U07 | Scan sequence/states | Call real private scan-step helper for 45/90/135/90 with mocked drivers/delays; inject no echo and servo-command error | SCANNING at angle command; 500 ms settle; MONITORING before read/after hold; two 500 ms delays per successful position; no echo stored as 511; command error gives ERROR without reading |
| S5-U08 | Obstacle threshold/re-arm | Complete one profile; exercise forward 24/25/35/36 cm and side 10 cm samples | Only forward <25 posts OBSTACLE; remains latched at 35; >35 re-arms; next <25 starts another episode |
| S5-U09 | Consecutive no echo | Feed forward 511 readings interleaved with side readings; valid forward readings break streaks | First two do not fault; third posts FAULT(NO_ECHO); count saturates and repeated failures do not spam; a valid forward reading resets the episode |
| S5-U10 | Recommendation rules | Table: left/right 100/60, 60/100, 39/39, 40/40, 39/40, 40/39; forward clears to 36 before completion | LEFT, RIGHT, STOP, LEFT, RIGHT, LEFT; CONTINUE if forward >35; PROFILING is published during calculation, then MONITORING |
| S5-U11 | Profile freshness/no echo | Seed side samples before OBSTACLE; supply missing/no-echo points, then fresh valid samples | No profile from old data or missing points; a no-echo point invalidates its freshness; profile only with all three fresh valid points |
| S5-U12 | Duplicate PROFILE_READY suppression | Complete profile, repeat samples for four scans; clear/re-arm and detect again | One accepted profile per episode; next obstacle can generate a second profile |
| S5-U13 | Profile telemetry persistence | Left 300/right 60/forward 20; normal SCANNING update, clear/re-arm, new episode; closest side at 10 cm | Side clamp to 255; minimum/bearing correct; width 0; profile persists through normal updates and re-arm; clears at next episode |
| S5-U14 | Nonblocking event rejection | Mock E_QOVR for OBSTACLE, PROFILE_READY and NO_ECHO fault | Failed OBSTACLE does not start profiling; later qualifying sample retries; successful events latch; no duplicate success |

All automated cases use `TEST_CHECK` with these stable IDs. The on-target test
runner executes them after startup; they are deterministic even if no S5
hardware is connected because all test-copy I/O is mocked.

## Manual cases — actual robot required

Before connecting: GP12 servo signal, GP1 TRIG, GP0 ECHO; common ground and a
stable suitable supply. ECHO must be 3.3 V safe at GP0 (level-shift a 5 V output).
Use USB console and `CRASH_REPORT=0`; UART0 conflicts with GP0/GP1.
Physically disable motor drive for bench tests: S5 posts events but does not
control motors, and the integrated State Machine can command S2.

| ID | What is tested | Steps / evidence | Expected result |
|---|---|---|---|
| S5-U02 | Servo sweep / PWM | Flash normal image; observe at least three 45/90/135/90 cycles; capture USB log and video or scope signal on GP12 | Repeated order without binding/jitter; about 500 ms settle + 500 ms hold per point; 50 Hz PWM with nominal 1000/1500/2000 us pulses; physical calibration recorded |
| S5-U03 | Ultrasonic distance / finite timeout | Use a broad flat target normal to the sensor at measured 20, 50 and 100 cm; collect ten forward readings per position; remove target and separately exercise a safely disconnected/pulled-low or stuck-high ECHO fixture; save distances/logs/scope | Record actual error/spread (demo target: within max(3 cm, 10%)); normal loop continues on missing echo, prints no echo and stores 511; rise/high waits each limited to 30 ms plus scheduler delays; never drive GP0 externally above 3.3 V |
| S5-U04 | Physical left/right mapping | Observe where the sensor faces at 45 and 135; place target on each physical side and inspect profile values/recommendation; photograph mounting | Confirm or correct private PROFILE_LEFT_ANGLE_DEG=45 / PROFILE_RIGHT_ANGLE_DEG=135 mapping before trusting LEFT/RIGHT; repository mounting convention is unconfirmed |
| S5-U15 | Real task/readiness, mutex and mode integration | Capture boot and S1 state/event evidence with both devices connected; inspect S5 telemetry through existing S1 Networking/debugger while scans run; verify start priority/core; disconnect sensor only with suitable power/wiring procedure | Owning task runs at priority 14; normal startup attempts READY; real mutex copies are coherent; SCANNING/MONITORING visible, brief PROFILING can require debugger; init ERROR/SELFTEST requires controlled driver-error injection (missing target alone is not init failure) |
| S5-U16 | Real obstacle/no-echo/profile event flow | With motor drive disabled, place forward target below 25 cm; set side targets above/below 40; inspect S1 event queue/log/debugger and S5 profile line; hold obstacle for several scans; clear >35 and reintroduce; test three forward no echoes | One OBSTACLE and accepted PROFILE_READY per episode, matching recommendations; no duplicates; third consecutive forward no echo faults once per episode; side no echoes do not increment forward count |
| S5-U17 | Timing/load and persistent profile telemetry | Repeat scans with full team firmware/network load; compare scope ECHO pulse and USB distance; observe completed profile fields across normal updates and next episode | Quantify preemption error and servo-settle adequacy; no hangs; profile remains coherent and persists until new episode; 40 cm side threshold must be calibrated for robot width/turn clearance |

Important limits: these are three discrete bearings, not a guaranteed safe path.
`bearing_deg` is the nearest measured angle, not an inferred obstacle centre;
`width_cm` stays zero. A profile waits for three valid episode samples: persistent
side no echo can leave it pending. A pending profile delays clear re-arm so it
can report CONTINUE. Accepted STOP is sent once per episode; there is no automatic
re-profile of the same uncleared obstacle. READY/SELFTEST posting is attempted once
at startup; queue rejection is reported but not retried. S1 can ignore OBSTACLE
outside NAVIGATING; S5 currently does not gate scans/events on S1 mode flags or
repeat startup self-tests after S1 error recovery. Verify these integration limits
with the team; this check-in does not change the agreed interfaces or algorithms.

## Results

Actual run results are appended below. Manual and on-target tests remain **NOT RUN**
until evidence is captured on the robot; host PASS does not substitute for them.

| Date | Test ID / check | Revision | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-04 | S5-U05–S5-U14 | 516b52c + dirty sources | Codex (host) | PASS: 10/10 | [Host output](logs/2026-11-04_week10_host.txt), [source hashes](logs/2026-11-04_week10_sources.txt) | Mocked software tests only; no physical hardware or on-target execution |
| 2026-11-04 | Normal build | 516b52c + dirty sources | Codex | PASS | [Build output](logs/2026-11-04_week10_normal_build.txt) | make -j8, SDK path supplied; UF2 generated |
| 2026-11-04 | TEST=1 build | 516b52c + dirty sources | Codex | PASS | [Build output](logs/2026-11-04_week10_test_build.txt) | make TEST=1 -j8; UF2 generated, not flashed |
