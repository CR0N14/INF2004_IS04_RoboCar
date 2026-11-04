/* Hardware-independent tests of a private copy of the production S5 code.
   No mock is installed into the running Ultrasound task. No real hardware,
   telemetry snapshot, kernel object or State Machine queue is changed here. */
#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../subsystem1/state_machine.h"
#include "../test_report.h"

#if APP_TEST

typedef struct
{
    bool ok, locked;
    ER create_error, lock_error, unlock_error, post_error;
    ER servo_error, ultrasonic_error, angle_error;
    UINT events, delays, reads, starts, exits, unlocks;
    UINT reading, commanded_angle;
    sm_evt_code_t codes[32];
    uint16_t args[32];
    UINT states[64], state_count;
} s5_mock_t;

static s5_mock_t mock;
static ID mock_create(const T_CMTX *cmtx);
static ER mock_lock(ID id, TMO timeout);
static ER mock_unlock(ID id);
static ID mock_start(FP entry, PRI priority, UW core, const char *name);
static void mock_exit(void);
static ER mock_delay(RELTIM delay);
static ER mock_servo_init(void);
static ER mock_ultrasonic_init(void);
static ER mock_angle(UINT angle);
static UINT mock_read(void);
static ER mock_post(sm_evt_code_t code, uint16_t arg);

/* Compile exactly the production implementation with private globals and
   test-only dependencies, instead of duplicating its detection algorithms. */
#define s5_get_tlm s5_copy_get_tlm
#define subsystem5_init s5_copy_init
#define subsystem5_start s5_copy_start
#define tk_cre_mtx mock_create
#define tk_loc_mtx mock_lock
#define tk_unl_mtx mock_unlock
#define app_start_task mock_start
#define tk_ext_tsk mock_exit
#define tk_dly_tsk mock_delay
#define s5_servo_init mock_servo_init
#define s5_ultrasonic_init mock_ultrasonic_init
#define s5_servo_set_angle mock_angle
#define s5_ultrasonic_read_cm mock_read
#define sm_post_event mock_post
#undef EXPORT
#define EXPORT static
#include "subsystem5.c"
#undef EXPORT
#define EXPORT
#undef s5_get_tlm
#undef subsystem5_init
#undef subsystem5_start
#undef tk_cre_mtx
#undef tk_loc_mtx
#undef tk_unl_mtx
#undef app_start_task
#undef tk_ext_tsk
#undef tk_dly_tsk
#undef s5_servo_init
#undef s5_ultrasonic_init
#undef s5_servo_set_angle
#undef s5_ultrasonic_read_cm
#undef sm_post_event

static void reset_mock(void)
{
    mock = (s5_mock_t){0};
    mock.ok = true;
    mock.reading = 54u;
    g_mtx_tlm = 0;
    g_tlm = (s5_tlm_t){ .scan_state = S5_SCAN_IDLE,
        .distance_cm = S5_NO_ECHO_CM, .servo_deg = 90u,
        .recommendation = SM_AVOID_STOP };
}

static ID mock_create(const T_CMTX *cmtx)
{
    mock.ok = mock.ok && cmtx->mtxatr == TA_INHERIT;
    return mock.create_error ? mock.create_error : 7;
}
static ER mock_lock(ID id, TMO timeout)
{
    mock.ok = mock.ok && id == 7 && timeout == TMO_FEVR && !mock.locked;
    if (mock.lock_error) return mock.lock_error;
    mock.locked = true;
    return E_OK;
}
static ER mock_unlock(ID id)
{
    mock.ok = mock.ok && id == 7 && mock.locked;
    mock.locked = false;
    ++mock.unlocks;
    if (mock.state_count < 64u) mock.states[mock.state_count++] = g_tlm.scan_state;
    return mock.unlock_error;
}
static ID mock_start(FP entry, PRI priority, UW core, const char *name)
{
    mock.ok = mock.ok && entry == (FP)ultrasound_task &&
        priority == S5_PRI_MAIN && core == 0u && name != NULL;
    ++mock.starts;
    return 8; /* Never run the infinite task from a test. */
}
static void mock_exit(void) { ++mock.exits; }
static ER mock_delay(RELTIM delay)
{
    mock.ok = mock.ok && !mock.locked && delay == 500u;
    ++mock.delays;
    return E_OK;
}
static ER mock_servo_init(void)
{
    mock.ok = mock.ok && !mock.locked;
    return mock.servo_error;
}
static ER mock_ultrasonic_init(void)
{
    mock.ok = mock.ok && !mock.locked;
    return mock.ultrasonic_error;
}
static ER mock_angle(UINT angle)
{
    mock.ok = mock.ok && !mock.locked && g_tlm.scan_state == S5_SCAN_SCANNING;
    mock.commanded_angle = angle;
    return mock.angle_error;
}
static UINT mock_read(void)
{
    mock.ok = mock.ok && !mock.locked && g_tlm.scan_state == S5_SCAN_MONITORING;
    ++mock.reads;
    return mock.reading;
}
static ER mock_post(sm_evt_code_t code, uint16_t arg)
{
    mock.ok = mock.ok && !mock.locked;
    if (mock.events >= 32u) { mock.ok = false; return E_QOVR; }
    mock.codes[mock.events] = code;
    mock.args[mock.events++] = arg;
    return mock.post_error;
}
static void feed(s5_detection_t *d, UINT angle, UINT cm)
{
    check_forward(d, angle, cm);
    profile_sample(d, angle, cm);
}
static void begin_episode(s5_detection_t *d)
{
    reset_mock();
    (void)s5_copy_init();
    *d = (s5_detection_t){0};
    feed(d, 45u, 300u); /* These old side samples must not qualify. */
    feed(d, 135u, 300u);
    feed(d, 90u, 20u);
}
static bool has_state(UINT state)
{
    UINT i;
    for (i = 0u; i < mock.state_count; ++i)
        if (mock.states[i] == state) return true;
    return false;
}

EXPORT void subsystem5_test(void)
{
    s5_detection_t d;
    s5_tlm_t tlm;
    bool ok;
    UINT i, previous_unlocks;
    static UINT const sequence[] = {45u, 90u, 135u, 90u};
    static struct { UINT left, right; sm_avoid_t recommendation; } const cases[] = {
        {100u, 60u, SM_AVOID_LEFT}, {60u, 100u, SM_AVOID_RIGHT},
        {39u, 39u, SM_AVOID_STOP}, {40u, 40u, SM_AVOID_LEFT},
        {39u, 40u, SM_AVOID_RIGHT}, {40u, 39u, SM_AVOID_LEFT}
    };
    TEST_INIT();
    tm_printf((UB*)"[S5 test] private mocks only; injected ERROR messages below are expected\n");

    reset_mock();
    ok = s5_copy_start() == E_OBJ && mock.starts == 0u;
    mock.create_error = E_NOMEM;
    ok = ok && s5_copy_init() == E_NOMEM && s5_copy_start() == E_OBJ;
    mock.create_error = E_OK;
    ok = ok && s5_copy_init() == E_OK && s5_copy_start() == E_OK && mock.starts == 1u;
    ok = ok && initialize_hardware() == E_OK && mock.events == 1u &&
        mock.codes[0] == SM_EVT_READY && mock.args[0] == SM_SUBSYS_ULTRASOUND;
    ok = ok && mock.ok;
    for (i = 0u; i < 2u; ++i) {
        reset_mock(); (void)s5_copy_init();
        if (i == 0u) mock.servo_error = E_SYS; else mock.ultrasonic_error = E_SYS;
        ultrasound_task(0, NULL); /* Failure path returns; all I/O is mocked. */
        ok = ok && mock.exits == 1u && g_tlm.scan_state == S5_SCAN_ERROR &&
            mock.events == 1u && mock.codes[0] == SM_EVT_SELFTEST_FAIL &&
            mock.args[0] == SM_SUBSYS_ULTRASOUND && mock.ok;
    }
    TEST_CHECK("S5-U05", ok, "mock task creation, READY, both SELFTEST_FAIL paths");

    reset_mock();
    ok = s5_copy_get_tlm(NULL) == E_PAR && s5_copy_get_tlm(&tlm) == E_OBJ;
    (void)s5_copy_init();
    ok = ok && s5_copy_get_tlm(&tlm) == E_OK && tlm.scan_state == S5_SCAN_IDLE &&
        tlm.distance_cm == S5_NO_ECHO_CM && !tlm.b_profile &&
        tlm.recommendation == SM_AVOID_STOP;
    publish_tlm(S5_SCAN_MONITORING, 90u, 54u);
    ok = ok && s5_copy_get_tlm(&tlm) == E_OK && tlm.distance_cm == 54u;
    tlm.distance_cm = 1u;
    ok = ok && g_tlm.distance_cm == 54u; /* Copy, not alias. */
    previous_unlocks = mock.unlocks;
    mock.lock_error = E_TMOUT;
    ok = ok && s5_copy_get_tlm(&tlm) == E_TMOUT && tlm.distance_cm == 1u &&
        mock.unlocks == previous_unlocks;
    mock.lock_error = E_OK; mock.unlock_error = E_SYS;
    ok = ok && s5_copy_get_tlm(&tlm) == E_SYS && !mock.locked;
    TEST_CHECK("S5-U06", ok && mock.ok, "snapshot copy, defaults, null/lock/unlock errors");

    reset_mock(); (void)s5_copy_init(); d = (s5_detection_t){0};
    ok = g_tlm.scan_state == S5_SCAN_IDLE;
    for (i = 0u; i < 4u; ++i) {
        scan_position(&d, sequence[i]);
        ok = ok && mock.commanded_angle == sequence[i] &&
            g_tlm.scan_state == S5_SCAN_MONITORING && g_tlm.distance_cm == 54u;
    }
    ok = ok && mock.reads == 4u && mock.delays == 8u && has_state(S5_SCAN_SCANNING);
    mock.reading = S5_NO_ECHO_CM;
    scan_position(&d, 45u);
    ok = ok && g_tlm.distance_cm == S5_NO_ECHO_CM;
    mock.angle_error = E_SYS;
    scan_position(&d, 45u);
    TEST_CHECK("S5-U07", ok && mock.ok && g_tlm.scan_state == S5_SCAN_ERROR &&
        mock.reads == 5u, "scan sequence, settle/monitor/error states, no-echo snapshot");

    begin_episode(&d);
    ok = mock.events == 1u && d.profile_pending && d.fresh_mask == 2u;
    feed(&d, 45u, 80u); feed(&d, 135u, 60u); /* Complete before re-arm. */
    feed(&d, 90u, 25u); feed(&d, 90u, 35u); feed(&d, 90u, 24u);
    ok = ok && mock.events == 2u && d.obstacle_reported;
    feed(&d, 90u, 36u); feed(&d, 90u, 25u);
    ok = ok && mock.events == 2u && !d.obstacle_reported;
    feed(&d, 45u, 10u); feed(&d, 135u, 10u); feed(&d, 90u, 24u);
    TEST_CHECK("S5-U08", ok && mock.ok && mock.events == 3u &&
        mock.codes[2] == SM_EVT_OBSTACLE, "forward <25, re-arm >35, sides ignored");

    reset_mock(); (void)s5_copy_init(); d = (s5_detection_t){0};
    feed(&d, 90u, S5_NO_ECHO_CM); feed(&d, 45u, 80u);
    feed(&d, 90u, S5_NO_ECHO_CM); feed(&d, 135u, S5_NO_ECHO_CM);
    ok = mock.events == 0u && d.no_echo_count == 2u;
    feed(&d, 90u, S5_NO_ECHO_CM);
    for (i = 0u; i < 8u; ++i) feed(&d, 90u, S5_NO_ECHO_CM);
    ok = ok && mock.events == 1u && mock.codes[0] == SM_EVT_FAULT &&
        mock.args[0] == SM_FAULT_NO_ECHO && d.no_echo_count == 3u;
    feed(&d, 90u, 50u); feed(&d, 90u, S5_NO_ECHO_CM);
    feed(&d, 90u, 50u); feed(&d, 90u, S5_NO_ECHO_CM);
    feed(&d, 90u, S5_NO_ECHO_CM);
    ok = ok && mock.events == 1u;
    feed(&d, 90u, S5_NO_ECHO_CM);
    TEST_CHECK("S5-U09", ok && mock.ok && mock.events == 2u,
        "three forward no-echo readings, saturation, valid-reading reset");

    ok = true;
    for (i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        begin_episode(&d);
        feed(&d, PROFILE_LEFT_ANGLE_DEG, cases[i].left);
        feed(&d, PROFILE_RIGHT_ANGLE_DEG, cases[i].right);
        ok = ok && mock.ok && mock.events == 2u &&
            mock.codes[1] == SM_EVT_PROFILE_READY &&
            mock.args[1] == cases[i].recommendation &&
            g_tlm.recommendation == cases[i].recommendation &&
            g_tlm.b_profile && has_state(S5_SCAN_PROFILING) &&
            g_tlm.scan_state == S5_SCAN_MONITORING;
    }
    begin_episode(&d); feed(&d, 90u, 36u);
    feed(&d, 45u, 20u); feed(&d, 135u, 20u);
    TEST_CHECK("S5-U10", ok && mock.ok && mock.events == 2u &&
        mock.args[1] == SM_AVOID_CONTINUE && !d.obstacle_reported,
        "LEFT/RIGHT/STOP/CONTINUE, 40 cm boundary, safe tie prefers LEFT");

    begin_episode(&d);
    ok = !g_tlm.b_profile && d.fresh_mask == 2u;
    feed(&d, 45u, S5_NO_ECHO_CM); feed(&d, 135u, 90u);
    ok = ok && mock.events == 1u && !g_tlm.b_profile;
    feed(&d, 90u, S5_NO_ECHO_CM); feed(&d, 45u, 80u);
    ok = ok && mock.events == 1u;
    feed(&d, 90u, 20u);
    TEST_CHECK("S5-U11", ok && mock.ok && mock.events == 2u && g_tlm.b_profile,
        "old samples/no echo cannot complete a fresh three-point profile");

    begin_episode(&d); feed(&d, 45u, 80u); feed(&d, 135u, 60u);
    for (i = 0u; i < 4u; ++i) {
        feed(&d, 90u, 20u); feed(&d, 45u, 80u); feed(&d, 135u, 60u);
    }
    ok = mock.events == 2u;
    feed(&d, 90u, 36u); feed(&d, 90u, 20u);
    feed(&d, 45u, 80u); feed(&d, 135u, 60u);
    TEST_CHECK("S5-U12", ok && mock.ok && mock.events == 4u &&
        mock.codes[3] == SM_EVT_PROFILE_READY,
        "one accepted profile per episode, next episode can report again");

    begin_episode(&d);
    feed(&d, PROFILE_LEFT_ANGLE_DEG, 300u);
    feed(&d, PROFILE_RIGHT_ANGLE_DEG, 60u);
    publish_tlm(S5_SCAN_SCANNING, 90u, S5_NO_ECHO_CM);
    (void)s5_copy_get_tlm(&tlm);
    ok = tlm.b_profile && tlm.left_clear_cm == 255u && tlm.right_clear_cm == 60u &&
        tlm.closest_cm == 20u && tlm.bearing_deg == 90u && tlm.width_cm == 0u &&
        tlm.recommendation == SM_AVOID_LEFT;
    feed(&d, 90u, 36u);
    ok = ok && g_tlm.b_profile;
    feed(&d, 90u, 20u);
    ok = ok && !g_tlm.b_profile && g_tlm.recommendation == SM_AVOID_STOP;
    feed(&d, 45u, 10u); feed(&d, 135u, 100u);
    TEST_CHECK("S5-U13", ok && mock.ok && g_tlm.closest_cm == 10u &&
        g_tlm.bearing_deg == 45u, "profile persistence, clamping, nearest bearing, new-episode reset");

    begin_episode(&d); feed(&d, 45u, 80u); mock.post_error = E_QOVR;
    feed(&d, 135u, 60u);
    ok = d.profile_pending && !d.profile_reported;
    mock.post_error = E_OK; feed(&d, 90u, 20u); feed(&d, 90u, 20u);
    ok = ok && mock.events == 3u && d.profile_reported;
    reset_mock(); (void)s5_copy_init(); d = (s5_detection_t){0};
    mock.post_error = E_QOVR; feed(&d, 90u, 20u);
    ok = ok && !d.profile_pending;
    mock.post_error = E_OK; feed(&d, 90u, 20u);
    ok = ok && d.profile_pending && mock.events == 2u;
    reset_mock(); (void)s5_copy_init(); d = (s5_detection_t){0};
    feed(&d, 90u, S5_NO_ECHO_CM); feed(&d, 90u, S5_NO_ECHO_CM);
    mock.post_error = E_QOVR; feed(&d, 90u, S5_NO_ECHO_CM);
    ok = ok && !d.no_echo_reported;
    mock.post_error = E_OK; feed(&d, 90u, S5_NO_ECHO_CM);
    feed(&d, 90u, S5_NO_ECHO_CM);
    TEST_CHECK("S5-U14", ok && mock.ok && mock.events == 2u && d.no_echo_reported,
        "rejected OBSTACLE/PROFILE_READY/FAULT posts retry without duplicate success");

    TEST_SUMMARY("subsystem5");
}
#endif /* APP_TEST */
