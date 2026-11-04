/* Subsystem 5, student 2303571: basic servo scan and ultrasonic demo. */

#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../subsystem1/state_machine.h"

#define SERVO_SETTLE_MS  500
#define SCAN_PAUSE_MS    500
#define FORWARD_ANGLE_DEG       90u
#define OBSTACLE_THRESHOLD_CM   25u
#define OBSTACLE_CLEAR_CM       35u
#define FORWARD_NO_ECHO_LIMIT   3u

/* Three-point Week 10/demo profile, to be calibrated on the robot.
   The repository establishes only 90 = forward. PROVISIONAL mounting
   mapping: swap these two constants if hardware testing shows the reverse. */
#define PROFILE_LEFT_ANGLE_DEG   45u
#define PROFILE_RIGHT_ANGLE_DEG  135u
#define PROFILE_SAFE_CLEAR_CM   40u
#define PROFILE_ALL_POINTS      7u

/* Private to the owning task; sideways samples never affect these latches. */
typedef struct
{
    bool obstacle_reported;
    bool no_echo_reported;
    UINT no_echo_count;
    UINT latest_valid_cm[3];       /* indices: 45, 90, 135 degrees */
    UINT fresh_mask;               /* valid samples from this episode only */
    bool profile_pending;
    bool profile_new;
    bool profile_reported;
} s5_detection_t;

static void check_forward(s5_detection_t *detection, UINT angle, UINT distance_cm)
{
    ER er;

    if (angle != FORWARD_ANGLE_DEG) {
        return;
    }
    if (distance_cm == S5_NO_ECHO_CM) {
        if (detection->no_echo_count < FORWARD_NO_ECHO_LIMIT) {
            ++detection->no_echo_count;
        }
        if (detection->no_echo_count == FORWARD_NO_ECHO_LIMIT &&
            !detection->no_echo_reported) {
            er = sm_post_event(SM_EVT_FAULT, SM_FAULT_NO_ECHO);
            CHECK(er, "S5 post NO_ECHO fault");
            detection->no_echo_reported = (er == E_OK);
        }
        /* Missing data cannot establish that an obstacle has cleared. */
        return;
    }
    if (distance_cm == 0u || distance_cm > 400u) {
        return;
    }

    /* A valid forward reading ends this no-echo episode. */
    detection->no_echo_count = 0u;
    detection->no_echo_reported = false;
    if (distance_cm > OBSTACLE_CLEAR_CM) {
        /* Finish a pending profile (possibly CONTINUE) before re-arming. */
        if (!detection->profile_pending) {
            detection->obstacle_reported = false;
            detection->profile_reported = false;
            detection->fresh_mask = 0u;
        }
    } else if (distance_cm < OBSTACLE_THRESHOLD_CM &&
               !detection->obstacle_reported) {
        er = sm_post_event(SM_EVT_OBSTACLE, 0u);
        CHECK(er, "S5 post OBSTACLE");
        detection->obstacle_reported = (er == E_OK);
        if (er == E_OK) {
            detection->profile_pending = true;
            detection->profile_new = true;
            detection->profile_reported = false;
            detection->fresh_mask = 0u;
        }
    }
    /* Failed nonblocking posts remain eligible at the next forward sample. */
}

static ID g_mtx_tlm;
/* Safe snapshot available before the owning task initializes hardware.
   Only ultrasound_task (through publish_tlm) changes it at runtime. */
static s5_tlm_t g_tlm = {
    .scan_state = S5_SCAN_IDLE,
    .distance_cm = S5_NO_ECHO_CM,
    .servo_deg = 90u,
    .recommendation = SM_AVOID_STOP
};

EXPORT ER s5_get_tlm(s5_tlm_t *p_tlm)
{
    ER er;

    if (p_tlm == NULL) {
        return E_PAR;
    }
    if (g_mtx_tlm <= 0) {
        return E_OBJ;
    }
    er = tk_loc_mtx(g_mtx_tlm, TMO_FEVR);
    if (er != E_OK) {
        return er;
    }
    *p_tlm = g_tlm;
    return tk_unl_mtx(g_mtx_tlm);
}

/* Called only by the Ultrasound task. Never hold the mutex during hardware
   access, delays or printing. Normal updates preserve the completed profile. */
static void publish_tlm(s5_scan_state_t state, UINT angle, UINT distance_cm)
{
    ER er;

    er = tk_loc_mtx(g_mtx_tlm, TMO_FEVR);
    if (er != E_OK) {
        CHECK(er, "S5 telemetry lock");
        return;
    }
    g_tlm.scan_state = state;
    g_tlm.servo_deg = angle;
    g_tlm.distance_cm = distance_cm;
    er = tk_unl_mtx(g_mtx_tlm);
    CHECK(er, "S5 telemetry unlock");
}

/* The same owning task publishes or clears just the profile fields. */
static void publish_profile(s5_tlm_t const *profile)
{
    ER er = tk_loc_mtx(g_mtx_tlm, TMO_FEVR);

    if (er != E_OK) {
        CHECK(er, "S5 profile telemetry lock");
        return;
    }
    g_tlm.b_profile = profile->b_profile;
    g_tlm.recommendation = profile->recommendation;
    g_tlm.left_clear_cm = profile->left_clear_cm;
    g_tlm.right_clear_cm = profile->right_clear_cm;
    g_tlm.closest_cm = profile->closest_cm;
    g_tlm.bearing_deg = profile->bearing_deg;
    g_tlm.width_cm = 0u; /* Three samples cannot support a precise width. */
    er = tk_unl_mtx(g_mtx_tlm);
    CHECK(er, "S5 profile telemetry unlock");
}

static UINT profile_index(UINT angle)
{
    return (angle == 45u) ? 0u : ((angle == 90u) ? 1u : 2u);
}

static void profile_sample(s5_detection_t *detection, UINT angle, UINT distance_cm)
{
    static UINT const point_angles[] = {45u, 90u, 135u};
    s5_tlm_t profile = {0};
    UINT index, left, forward, right, closest;
    sm_avoid_t recommendation;
    char const *name;
    ER er;

    if (angle != 45u && angle != 90u && angle != 135u) {
        return;
    }
    if (detection->profile_new) {
        profile.recommendation = SM_AVOID_STOP;
        publish_profile(&profile);
        detection->profile_new = false;
    }
    index = profile_index(angle);
    if (distance_cm > 0u && distance_cm <= 400u) {
        detection->latest_valid_cm[index] = distance_cm;
        if (detection->profile_pending) {
            detection->fresh_mask |= 1u << index;
        }
    } else if (detection->profile_pending) {
        /* Keep the latest valid value, but do not use a stale/no-echo point. */
        detection->fresh_mask &= ~(1u << index);
    }
    if (!detection->profile_pending || detection->profile_reported ||
        detection->fresh_mask != PROFILE_ALL_POINTS) {
        return;
    }

    publish_tlm(S5_SCAN_PROFILING, angle, distance_cm);
    left = detection->latest_valid_cm[profile_index(PROFILE_LEFT_ANGLE_DEG)];
    forward = detection->latest_valid_cm[1];
    right = detection->latest_valid_cm[profile_index(PROFILE_RIGHT_ANGLE_DEG)];
    if (forward > OBSTACLE_CLEAR_CM) {
        recommendation = SM_AVOID_CONTINUE;
        name = "CONTINUE";
    } else if (left >= PROFILE_SAFE_CLEAR_CM && left >= right) {
        recommendation = SM_AVOID_LEFT; /* Equal safe clearances: prefer left. */
        name = "LEFT";
    } else if (right >= PROFILE_SAFE_CLEAR_CM) {
        recommendation = SM_AVOID_RIGHT;
        name = "RIGHT";
    } else {
        recommendation = SM_AVOID_STOP;
        name = "STOP";
    }

    closest = 0u;
    for (index = 1u; index < 3u; ++index) {
        if (detection->latest_valid_cm[index] < detection->latest_valid_cm[closest]) {
            closest = index;
        }
    }
    profile.b_profile = 1u;
    profile.left_clear_cm = (left > 255u) ? 255u : left;
    profile.right_clear_cm = (right > 255u) ? 255u : right;
    profile.closest_cm = detection->latest_valid_cm[closest];
    /* Measured nearest point, not an inferred obstacle centre. */
    profile.bearing_deg = point_angles[closest];
    profile.recommendation = recommendation;
    publish_profile(&profile);

    er = sm_post_event(SM_EVT_PROFILE_READY, recommendation);
    CHECK(er, "S5 post PROFILE_READY");
    if (er == E_OK) {
        detection->profile_reported = true;
        detection->profile_pending = false;
        tm_printf((UB*)"[S5] profile: left=%u forward=%u right=%u recommend=%s\n",
                  left, forward, right, name);
        if (forward > OBSTACLE_CLEAR_CM) {
            detection->obstacle_reported = false;
            detection->profile_reported = false;
            detection->fresh_mask = 0u;
        }
    }
    /* Retry a rejected nonblocking post on a later complete set of readings.
       No avoidance state: S5 has recommended a route, not executed a bypass. */
    publish_tlm(S5_SCAN_MONITORING, angle, distance_cm);
}

/* Private helpers allow tests to exercise startup and a complete scan step
   with a separate copy and mocked I/O. Production calls stay in the owner task. */
static ER initialize_hardware(void)
{
    ER er;

    /* This task owns all servo hardware access, including initialization. */
    er = s5_servo_init();
    if (er < E_OK) {
        ER init_er = er;
        CHECK(er, "S5 servo init");
        publish_tlm(S5_SCAN_ERROR, 90u, S5_NO_ECHO_CM);
        er = sm_post_event(SM_EVT_SELFTEST_FAIL, SM_SUBSYS_ULTRASOUND);
        CHECK(er, "S5 post SELFTEST_FAIL (servo)");
        return init_er;
    }
    er = s5_ultrasonic_init();
    if (er < E_OK) {
        ER init_er = er;
        CHECK(er, "S5 ultrasonic init (requires GP0/GP1 free from UART0)");
        publish_tlm(S5_SCAN_ERROR, 90u, S5_NO_ECHO_CM);
        er = sm_post_event(SM_EVT_SELFTEST_FAIL, SM_SUBSYS_ULTRASOUND);
        CHECK(er, "S5 post SELFTEST_FAIL (ultrasonic)");
        return init_er;
    }

    er = sm_post_event(SM_EVT_READY, SM_SUBSYS_ULTRASOUND);
    CHECK(er, "S5 post READY");

    return E_OK;
}

static void scan_position(s5_detection_t *detection, UINT angle)
{
    UINT distance_cm;
    ER er;

    /* Discard the previous angle's distance while moving. */
    publish_tlm(S5_SCAN_SCANNING, angle, S5_NO_ECHO_CM);
    er = s5_servo_set_angle(angle);
    CHECK(er, "S5 servo set angle");
    if (er < E_OK) {
        publish_tlm(S5_SCAN_ERROR, angle, S5_NO_ECHO_CM);
        tk_dly_tsk(SCAN_PAUSE_MS);
        return;
    }
    tk_dly_tsk(SERVO_SETTLE_MS);
    publish_tlm(S5_SCAN_MONITORING, angle, S5_NO_ECHO_CM);
    distance_cm = s5_ultrasonic_read_cm();
    publish_tlm(S5_SCAN_MONITORING, angle, distance_cm);
    if (distance_cm == S5_NO_ECHO_CM) {
        tm_printf((UB*)"[S5] angle=%u deg no echo\n", angle);
    } else {
        tm_printf((UB*)"[S5] angle=%u deg distance=%u cm\n",
                  angle, distance_cm);
    }
    check_forward(detection, angle, distance_cm);
    profile_sample(detection, angle, distance_cm);
    /* Also guarantees more than 60 ms between ultrasonic triggers. */
    tk_dly_tsk(SCAN_PAUSE_MS);
}

static void ultrasound_task(INT stacd, void *exinf)
{
    static UINT const angles[] = {45u, 90u, 135u, 90u};
    s5_detection_t detection = {0};
    UINT i;

    (void)stacd;
    (void)exinf;
    if (initialize_hardware() < E_OK) {
        tk_ext_tsk();
        return;
    }
    for (;;) {
        for (i = 0u; i < sizeof(angles) / sizeof(angles[0]); ++i) {
            scan_position(&detection, angles[i]);
        }
    }
}

EXPORT ER subsystem5_init(void)
{
    T_CMTX cmtx = {0};

    cmtx.mtxatr = TA_INHERIT;
    g_mtx_tlm = tk_cre_mtx(&cmtx);
    return (g_mtx_tlm < E_OK) ? g_mtx_tlm : E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid;

    if (g_mtx_tlm <= 0) {
        return E_OBJ;
    }
    tid = app_start_task((FP)ultrasound_task, S5_PRI_MAIN, 0, "Ultrasound");
    return (tid < E_OK) ? tid : E_OK;
}
