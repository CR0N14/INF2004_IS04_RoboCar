/* Subsystem 5, student 2303571: basic servo scan and ultrasonic demo. */

#include "subsystem5.h"
#include "../demo_config.h"
#if ENCODER_FREE_DEMO
#include "demo_profile.h"
#endif
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
#define PROFILE_TIMEOUT_MS      10000u /* Protocol liveness, not a clearance. */

/* Core 1 (TP_PRC2): the CYW43 WiFi service task is pinned to core 0 at a
   higher priority and polls every 2 ms.  The echo IRQ is enabled on the core
   that runs s5_ultrasonic_init(), so "any core" let boot timing decide
   whether echo timing shared core 0 with WiFi (logged as whole boots of no
   echo after cold power-up, never after a warm reflash). */
#if TK_SUPPORT_SMP
#define S5_TASK_CORE            TP_PRC2
#else
#define S5_TASK_CORE            0u
#endif

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
    bool obstacle_confirmed;      /* Observed S1 in obstacle avoidance. */
    bool scan_fault_reported;
    sm_avoid_t last_recommendation;
    UW profile_started_ms;
} s5_detection_t;

static UW now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);
    return (UW)tim.lo;
}

static void reset_episode(s5_detection_t *detection)
{
    detection->obstacle_reported = false;
    detection->obstacle_confirmed = false;
    detection->profile_pending = false;
    detection->profile_new = false;
    detection->profile_reported = false;
    detection->scan_fault_reported = false;
    detection->fresh_mask = 0u;
}

static void check_forward(s5_detection_t *detection, UINT angle, UINT distance_cm)
{
    ER er;
    uint32_t status = sm_status_word();
    sm_state_t state = SM_STATUS_STATE(status);

    /* Queue success is not an acknowledgment. Discard episodes when S1
       stops/faults, or returns from a confirmed avoidance episode. */
    if ((state != SM_STATE_NAVIGATING && state != SM_STATE_OBSTACLE_AVOIDANCE) ||
        (state == SM_STATE_NAVIGATING && detection->obstacle_confirmed)) {
        reset_episode(detection);
    }
    if (state == SM_STATE_OBSTACLE_AVOIDANCE && detection->obstacle_reported &&
        !detection->obstacle_confirmed) {
        detection->obstacle_confirmed = true;
        detection->profile_started_ms = now_ms();
    }

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
    if (state != SM_STATE_NAVIGATING) {
        return; /* HOLD/clear handling belongs to profile_sample below. */
    }
    if (distance_cm > OBSTACLE_CLEAR_CM) {
        reset_episode(detection);
    } else if (distance_cm < OBSTACLE_THRESHOLD_CM &&
               !detection->obstacle_confirmed) {
        /* At most one attempt per forward sample (~2 s). If S1 stayed in
           NAVIGATING after a queued event, allow a later sample to retry. */
        er = sm_post_event(SM_EVT_OBSTACLE, 0u);
        CHECK(er, "S5 post OBSTACLE");
        if (er == E_OK) {
            detection->obstacle_reported = true;
            detection->profile_pending = true;
            detection->profile_new = true;
            detection->profile_reported = false;
            detection->fresh_mask = 0u;
            detection->profile_started_ms = now_ms();
        }
        /* A rejected retry must not forget an earlier queued event that
           S1 may still consume after this status observation. */
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
#if ENCODER_FREE_DEMO
    g_tlm.width_cm = profile->width_cm;
#else
    g_tlm.width_cm = 0u; /* Three samples cannot support a precise width. */
#endif
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
    uint32_t status;

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
    status = sm_status_word();
    if (!detection->obstacle_reported || detection->scan_fault_reported ||
        SM_STATUS_STATE(status) != SM_STATE_OBSTACLE_AVOIDANCE ||
        SM_STATUS_NAV(status) != SM_NAV_HOLD) {
        return; /* Never send a profile into STANDBY, ERROR or a bypass. */
    }
    if (!detection->obstacle_confirmed) {
        detection->obstacle_confirmed = true;
        detection->profile_started_ms = now_ms();
    }

    /* A valid clear forward sample is sufficient to release HOLD. Missing
       side echoes must not prevent this, and are never interpreted as clear. */
    if (angle == FORWARD_ANGLE_DEG && distance_cm > OBSTACLE_CLEAR_CM &&
        distance_cm <= 400u && (!detection->profile_reported ||
        detection->last_recommendation == SM_AVOID_STOP)) {
        if (s5_get_tlm(&profile) == E_OK) {
            profile.recommendation = SM_AVOID_CONTINUE;
            publish_profile(&profile); /* b_profile stays false if incomplete. */
        }
        er = sm_post_event(SM_EVT_PROFILE_READY, SM_AVOID_CONTINUE);
        CHECK(er, "S5 post clear PROFILE_READY");
        if (er == E_OK) {
            detection->profile_reported = true;
            detection->profile_pending = false;
            detection->last_recommendation = SM_AVOID_CONTINUE;
            tm_printf((UB*)"[S5] profile: forward=%u recommend=CONTINUE\n", distance_cm);
        }
        return;
    }
    if (!detection->profile_pending) {
        return;
    }
    if (detection->fresh_mask != PROFILE_ALL_POINTS) {
        if ((UW)(now_ms() - detection->profile_started_ms) >= PROFILE_TIMEOUT_MS) {
            er = sm_post_event(SM_EVT_FAULT, SM_FAULT_SCAN);
            CHECK(er, "S5 post profile timeout fault");
            if (er == E_OK) {
                detection->scan_fault_reported = true;
                detection->profile_pending = false;
                publish_tlm(S5_SCAN_ERROR, angle, distance_cm);
            }
        }
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

    /* STOP means keep holding and inspect another fresh set. Publish only a
       changed recommendation, rather than repeating STOP every scan. */
    if (detection->profile_reported &&
        recommendation == detection->last_recommendation) {
        detection->fresh_mask = 0u;
        detection->profile_started_ms = now_ms();
        publish_tlm(S5_SCAN_MONITORING, angle, distance_cm);
        return;
    }
    er = sm_post_event(SM_EVT_PROFILE_READY, recommendation);
    CHECK(er, "S5 post PROFILE_READY");
    if (er == E_OK) {
        detection->profile_reported = true;
        detection->profile_pending = (recommendation == SM_AVOID_STOP);
        detection->last_recommendation = recommendation;
        detection->fresh_mask = 0u;
        detection->profile_started_ms = now_ms();
        tm_printf((UB*)"[S5] profile: left=%u forward=%u right=%u recommend=%s\n",
                  left, forward, right, name);
        /* Keep the latch until S1's published state leaves this episode. */
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

#if ENCODER_FREE_DEMO
static bool demo_hold(void){uint32_t s=sm_status_word();return SM_STATUS_STATE(s)==SM_STATE_OBSTACLE_AVOIDANCE&&SM_STATUS_NAV(s)==SM_NAV_HOLD;}
typedef struct
{
    UINT angle;     /* where the servo is now */
    bool aborted;   /* S1 left HOLD during the scan */
    bool fault;     /* the servo rejected an angle */
} demo_scan_t;

/* Range callback for demo_measure_width(). The settle time grows with the
   move, and every reading waits at least 80 ms so triggers stay over 60 ms
   apart. */
static uint16_t demo_scan_read(void *p_ctx, unsigned int angle)
{
    demo_scan_t *p_s = (demo_scan_t *)p_ctx;
    uint16_t d;

    if (p_s->aborted || p_s->fault) {
        return S5_NO_ECHO_CM;
    }
    if (!demo_hold()) {
        p_s->aborted = true;
        return S5_NO_ECHO_CM;
    }
    if (angle != p_s->angle) {
        UINT moved = (angle > p_s->angle) ? (angle - p_s->angle) : (p_s->angle - angle);

        if (s5_servo_set_angle(angle) != E_OK) {
            p_s->fault = true;
            return S5_NO_ECHO_CM;
        }
        p_s->angle = angle;
        publish_tlm(S5_SCAN_SCANNING, angle, S5_NO_ECHO_CM);
        tk_dly_tsk(80u + 3u * moved);
    } else {
        tk_dly_tsk(80u);
    }
    d = s5_ultrasonic_read_cm();
    publish_tlm(S5_SCAN_SCANNING, angle, d);
    return d;
}

/* Missing data is never read as clear: no echo after a retry gives 0. */
static UINT demo_clearance(demo_scan_t *p_s, unsigned int angle)
{
    uint16_t d = demo_scan_read(p_s, angle);

    if (d == S5_NO_ECHO_CM) {
        d = demo_scan_read(p_s, angle);
    }
    return (d == S5_NO_ECHO_CM) ? 0u : d;
}

static void demo_scan(void){
 demo_scan_t scan={FORWARD_ANGLE_DEG,false,false};
 demo_width_t w={0};
 s5_tlm_t p={0};
 UINT left=0u,right=0u;
 bool ok;

 p.recommendation=SM_AVOID_STOP;publish_profile(&p);
 ok=demo_measure_width(demo_scan_read,&scan,&w);
 if(ok&&!scan.aborted&&!scan.fault){
  left=demo_clearance(&scan,PROFILE_LEFT_ANGLE_DEG);
  right=demo_clearance(&scan,PROFILE_RIGHT_ANGLE_DEG);
 }
 if(!scan.aborted){
  if(!ok||scan.fault){
   if(demo_hold())sm_post_event(SM_EVT_FAULT,SM_FAULT_SCAN);
  }else{
   p.b_profile=1;p.width_cm=w.width_cm;p.closest_cm=w.ahead_cm;p.bearing_deg=w.bearing_deg;
   p.left_clear_cm=left>255u?255u:left;p.right_clear_cm=right>255u?255u:right;
   if(left>=PROFILE_SAFE_CLEAR_CM&&left>=right)p.recommendation=SM_AVOID_LEFT;
   else if(right>=PROFILE_SAFE_CLEAR_CM)p.recommendation=SM_AVOID_RIGHT;
   publish_profile(&p);
   tm_printf((UB*)"[S5] width: %u cm, corners %u..%u deg, ahead %u cm, clear L%u R%u, %u readings\n",
             (unsigned int)w.width_cm,(unsigned int)w.left_deg,(unsigned int)w.right_deg,
             (unsigned int)w.ahead_cm,(unsigned int)left,(unsigned int)right,(unsigned int)w.readings);
   if(demo_hold())sm_post_event(SM_EVT_PROFILE_READY,p.recommendation);
  }
 }
 s5_servo_set_angle(90);tk_dly_tsk(500);
}
static void demo_ultrasound_loop(void){
 unsigned near=0,missing=0;bool scan_done=false;
 s5_servo_set_angle(90);tk_dly_tsk(500);
 for(;;){
  uint32_t st=sm_status_word();
  if(demo_hold()){
   if(!scan_done){demo_scan();scan_done=true;}
   else { /* STOP profiles remain stopped; rescan after a bounded pause. */
    tk_dly_tsk(500);if(demo_hold())demo_scan();
   }
  }else{
   scan_done=false;UINT d=s5_ultrasonic_read_cm();publish_tlm(S5_SCAN_MONITORING,90,d);
   bool moving=SM_STATUS_STATE(st)==SM_STATE_NAVIGATING||SM_STATUS_STATE(st)==SM_STATE_OBSTACLE_AVOIDANCE;
   if(moving){
    if(d==S5_NO_ECHO_CM){if(++missing>=3)sm_post_event(SM_EVT_FAULT,SM_FAULT_NO_ECHO);}
    else {missing=0;if(d<25){if(++near>=2){
      if(SM_STATUS_NAV(st)==SM_NAV_BYPASS||SM_STATUS_NAV(st)==SM_NAV_RECOVERY)sm_post_event(SM_EVT_FAULT,SM_FAULT_SCAN);
      else sm_post_event(SM_EVT_OBSTACLE,0);
    }}else near=0;}
   }else {near=0;missing=0;}
   tk_dly_tsk(100);
  }
 }
}
#endif
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
#if ENCODER_FREE_DEMO
    demo_ultrasound_loop();
#endif
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
    tid = app_start_task((FP)ultrasound_task, S5_PRI_MAIN, S5_TASK_CORE, "Ultrasound");
    return (tid < E_OK) ? tid : E_OK;
}
