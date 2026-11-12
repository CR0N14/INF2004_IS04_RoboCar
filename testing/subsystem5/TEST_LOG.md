# Subsystem 5 — Week 10 test log

**Owner:** Buddy 5, student 2303571. **Scope:** servo, ultrasonic scan, telemetry,
basic State Machine events and three-point demo profile.

## How these tests work

`subsystem5_test.c` compiles a private copy of the production S5 implementation
with mocked hardware, mutex, task, delay, status/time and event calls.
Successful mock events are applied to a private instance of S1's actual pure
state-machine logic; no live State Machine or shared data is changed. The servo
driver is also included privately with captured MMIO/BSP operations. The same tests run on
host and in the `TEST=1` image. They never change the running S5 snapshot, touch
GPIO/PWM, start another owning task, or enqueue real State Machine events.
The copies test software control flow and register arguments, not real scheduler,
peripheral readback, electrical waveforms or physical movement.
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
| S5-U08 | Obstacle threshold/re-arm | Complete one profile; simulate S1 line recovery; exercise forward 24/25/35/36 cm and side 10 cm samples | Only NAVIGATING forward <25 posts OBSTACLE; no duplicate during active avoidance; after observed recovery and a clear sample, next <25 starts another episode |
| S5-U09 | Consecutive no echo | Feed forward 511 readings interleaved with side readings; valid forward readings break streaks | First two do not fault; third posts FAULT(NO_ECHO); count saturates and repeated failures do not spam; a valid forward reading resets the episode |
| S5-U10 | Recommendation rules | Table: left/right 100/60, 60/100, 39/39, 40/40, 39/40, 40/39; forward clears to 36 before completion | LEFT, RIGHT, STOP, LEFT, RIGHT, LEFT; CONTINUE immediately on a valid forward >35 in S1 HOLD, even without side samples; PROFILING during full calculation, then MONITORING |
| S5-U11 | Profile freshness/no echo | Seed side samples before OBSTACLE; supply missing/no-echo points, then fresh valid samples | No profile from old data or missing points; a no-echo point invalidates its freshness; complete three-point profile only with all three fresh valid points |
| S5-U12 | Duplicate PROFILE_READY suppression | Complete profile, repeat samples for four scans; clear/re-arm and detect again | No repeat of a completed bypass recommendation; a new S1 navigation episode can report again; changed recommendations after STOP are tested separately |
| S5-U13 | Profile telemetry persistence | Left 300/right 60/forward 20; normal SCANNING update, clear/re-arm, new episode; closest side at 10 cm | Side clamp to 255; minimum/bearing correct; width 0; profile persists through normal updates and re-arm; clears at next episode |
| S5-U14 | Nonblocking event rejection | Mock E_QOVR for OBSTACLE, PROFILE_READY and NO_ECHO fault | Failed OBSTACLE does not start profiling; later qualifying sample retries; successful events latch; no duplicate success |
| S5-U18 | PWM reset release and unchanged pulse calculations | Real private servo-driver copy; capture reset clear mask, done wait, all direct writes and BSP arguments; test already-released PWM and stuck reset-done across timer wrap | Clear only PWM bit 14; never assert reset; wait before configuring only slice 6; timeout returns E_TMOUT after 10 ms; divider 125.0, TOP 19999, 50 Hz, pulses 1000/1500/2000 us; invalid angle rejected |
| S5-U19 | S1 episode gating and observation | Actual private S1 model in STANDBY, NAVIGATING, avoidance and stop/start; delay event consumption | No OBSTACLE in STANDBY; no profile until published avoidance/HOLD; unconfirmed obstacle can retry once per forward sample; a rejected retry retains the earlier queued event; confirmed episodes suppress duplicates; stop/start permits re-arm |
| S5-U20 | STOP recovery and post retry | Real S1 model stays HOLD on repeated blocked scans; reject then accept clear CONTINUE; separately improve side clearance using fresh samples | Unchanged STOP is not repeated; valid forward >35 leaves HOLD via CONTINUE; failed post remains retryable; fresh safer side after STOP enters BYPASS once |
| S5-U21 | Missing-side recovery / bounded profiling | Keep side echoes missing; first clear forward, then separately hold forward at 20 cm until 10 s; reject first fault attempt | Clear forward leaves HOLD without claiming a complete profile; otherwise one accepted FAULT(SCAN) reaches S1 ERROR; rejection retries; existing S1 retry timeout reaches INITIALISING |

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
| S5-U16 | Real obstacle/no-echo/profile event flow | With motor drive disabled, place forward target below 25 cm; set side targets above/below 40; inspect S1 event queue/log/debugger and S5 profile line; hold obstacle for several scans; clear >35 and reintroduce; test three forward no echoes | OBSTACLE only in NAVIGATING; profile only in avoidance/HOLD; no repeated unchanged recommendation; STOP can update to CONTINUE or a fresh safer route; third forward no echo faults once; missing sides fault SCAN after a 10 s incomplete profile unless forward clears |
| S5-U17 | Timing/load and persistent profile telemetry | Repeat scans with full team firmware/network load; compare scope ECHO pulse and USB distance; observe completed profile fields across normal updates and next episode | Quantify preemption error and servo-settle adequacy; no hangs; profile remains coherent and persists until new episode; 40 cm side threshold must be calibrated for robot width/turn clearance |

## Focused fixes re-check — 2026-11-12

The current source still had USE_PTMR=0 and PWM reset release guarded by that
option; no other active release path existed. S5 now atomically clears only
RESETS_RESET_PWM (bit 14, 0x4000) in RESETS_RESET (0x4000C000; clear alias
0x4000F000), waits for the same bit in RESETS_RESET_DONE (0x4000C008), and returns
E_TMOUT if the read-only 1 MHz timer observes 10 ms without readiness. It never
asserts PWM reset, so other already-running slices are not reset. Configuration
of owned slice 6 and the existing 45/90/135 pulse calculations are unchanged.

The S1 model still accepts OBSTACLE only in NAVIGATING, and PROFILE_READY only in
OBSTACLE_AVOIDANCE. STOP sets HOLD; CONTINUE returns to NAVIGATING; SCAN is an
existing fatal fault with S1's existing ERROR/retry handling. These were genuine
protocol risks, addressed without changing S1:

- OBSTACLE is attempted only while S1 reports NAVIGATING, at most once per forward
  reading (~2 s in the unchanged scan loop). A queued-but-unconfirmed episode can
  retry on the next qualifying forward reading; a rejected retry retains the
  earlier queued event until its disposition can be observed. Profile publication requires
  observing obstacle avoidance with navigation HOLD. Leaving the episode through
  STOP/ERROR or a return to NAVIGATING resets local episode state.
- STOP allows another fresh three-point set. An unchanged STOP updates telemetry
  but is not re-posted; a changed safe-side recommendation can be sent once.
- A valid current forward >35 cm while in HOLD sends CONTINUE even with missing
  sides. `b_profile` remains false if no complete profile exists. An existing
  completed profile is preserved with the updated recommendation.
- A confirmed pending profile/re-profile with no complete valid set within
  10,000 ms reports FAULT(SCAN), rather than waiting indefinitely. This new private
  protocol deadline is checked at sample boundaries, so scheduling/scan delays
  add latency. Rejected posts retry on later samples; accepted faults are latched.
  A completed fresh blocked set may safely keep HOLD; it renews the deadline.

**Unchanged:** pins, 25 cm obstacle / 35 cm clear / 40 cm side-clearance thresholds,
three forward-no-echo fault rule, servo/ultrasonic scan timing, public interfaces,
no motor access, and the provisional 45 LEFT / 135 RIGHT mapping.

**Remaining limits to validate/document:**

- S1 status is an observation, not an event-ID acknowledgment. A command can race
  the status read/post, or a rapid stop/start can occur between observations.
  Existing APIs cannot make acceptance atomic. S5 retries unconfirmed obstacle
  posts and cancels observed inactive episodes; no new shared API was invented.
- Persistent event-queue rejection prevents delivery of any recovery event; S5
  reports errors and retries but cannot force S1 to accept a message.
- No echo is never confirmed clear space. Three missing forward echoes retain
  the existing fatal NO_ECHO behavior, including for open-space targets. A timed
  out side profile safely faults instead of guessing a route.
- READY/SELFTEST posts are still one startup attempt; they are not retried, and S5
  still does not re-run startup self-tests during S1 recovery or suspend the demo
  sweep in ERROR. The shared app_start_task start-error reporting weakness was
  rechecked but left untouched because it is outside this focused S5 fix.
- `bearing_deg` is the nearest measured angle; width stays zero. No motion sensor
  confirms servo position. Correct MMIO arguments and passing mock tests cannot
  prove actual pin pulses, power integrity, servo movement or mounting direction.

Tomorrow's bench run must cold-boot, verify PWM reset/done and GP12's 50 Hz
1/1.5/2 ms waveform, then check the connector/ground/supply and unloaded movement.
Validate STOP/clear/side-no-echo recovery with motor drive physically disabled.
No physical servo success is claimed by these software tests.

## Results

Actual run results are appended below. Manual and on-target tests remain **NOT RUN**
until evidence is captured on the robot; host PASS does not substitute for them.

| Date | Test ID / check | Revision | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-04 | S5-U05–S5-U14 | 516b52c + dirty sources | Codex (host) | PASS: 10/10 | [Host output](logs/2026-11-04_week10_host.txt), [source hashes](logs/2026-11-04_week10_sources.txt) | Mocked software tests only; no physical hardware or on-target execution |
| 2026-11-04 | Normal build | 516b52c + dirty sources | Codex | PASS | [Build output](logs/2026-11-04_week10_normal_build.txt) | make -j8, SDK path supplied; UF2 generated |
| 2026-11-04 | TEST=1 build | 516b52c + dirty sources | Codex | PASS | [Build output](logs/2026-11-04_week10_test_build.txt) | make TEST=1 -j8; UF2 generated, not flashed |
| 2026-11-12 | S5-U19 delayed-consumption regression (development) | a993319 + intermediate dirty snapshot | Codex (host) | FAIL: 13/14 total | [Regression output](logs/2026-11-12_retry_regression.txt) | Rejected retry erased earlier queued OBSTACLE; corrected locally, final suite below passes |
| 2026-11-12 | S5-U05–U14, U18–U21 | a993319 + dirty sources | Codex (host) | PASS: 14/14 | [Host output](logs/2026-11-12_fixes_host.txt), [source hashes](logs/2026-11-12_fixes_sources.txt) | Private real servo implementation and S1 statechart, mocked I/O only; no hardware claim |
| 2026-11-12 | Normal full build | a993319 + dirty sources | Codex | PASS | [Build output](logs/2026-11-12_fixes_normal_build.txt) | make -j8 with installed SDK path; no warnings/errors |
| 2026-11-12 | TEST=1 full build | a993319 + dirty sources | Codex | PASS | [Build output](logs/2026-11-12_fixes_test_build.txt) | make TEST=1 -j8 with installed SDK path; no warnings/errors; not flashed |
