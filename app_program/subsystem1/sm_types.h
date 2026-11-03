/*
 *  sm_types.h
 *  System State Machine: states, events, faults and commands shared by
 *  every subsystem.
 *
 *  This header has no kernel dependencies so that pure logic (and host
 *  unit tests) can use it.  Tasks include state_machine.h instead, which
 *  adds the kernel objects and the API.
 *
 *  Every value is sized to its valid range (Change Requirement 1): the
 *  *_BITS constants give the number of bits each field occupies wherever
 *  it is stored or sent.
 */

#ifndef SM_TYPES_H
#define SM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 *  System states (system-level statechart, design review section 5)
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_STATE_INITIALISING = 0,      /* self-tests, WiFi + MQTT connect   */
    SM_STATE_STANDBY,               /* motors off, sensors on, idle      */
    SM_STATE_NAVIGATING,            /* following the line                */
    SM_STATE_OBSTACLE_AVOIDANCE,    /* profiling and bypassing obstacle  */
    SM_STATE_ERROR,                 /* motors braked, fault published    */
    SM_STATE_COUNT
} sm_state_t;

#define SM_STATE_BITS           3u

/* ---------------------------------------------------------------------
 *  Navigation mode: what the line-following and motion subsystems are
 *  allowed to do inside the current state.
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_NAV_DISABLED = 0,    /* motors must not run                       */
    SM_NAV_TRACKING,        /* normal line following                     */
    SM_NAV_MANOEUVRE,       /* commanded turn, line steering suspended   */
    SM_NAV_HOLD,            /* stopped at an obstacle while it is scanned */
    SM_NAV_BYPASS,          /* driving round an obstacle, steering off   */
    SM_NAV_RECOVERY,        /* searching for the original line           */
    SM_NAV_COUNT
} sm_nav_mode_t;

#define SM_NAV_BITS             3u

/* ---------------------------------------------------------------------
 *  Barcode navigation actions
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_ACTION_LEFT = 0,
    SM_ACTION_RIGHT,
    SM_ACTION_STRAIGHT,
    SM_ACTION_UTURN,
    SM_ACTION_COUNT
} sm_action_t;

#define SM_ACTION_BITS          2u

/* Barcode symbols are the letters 'A'..'Z', stored as 0..25 */
#define SM_BARCODE_SYMBOLS      26u
#define SM_BARCODE_SYMBOL_BITS  5u
#define SM_BARCODE_SYMBOL(c)    ((uint8_t)((c) - 'A'))

/* ---------------------------------------------------------------------
 *  Obstacle avoidance recommendation (Ultrasound -> State Machine)
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_AVOID_CONTINUE = 0,  /* false alarm: carry on along the line      */
    SM_AVOID_STOP,          /* no safe route: stay stopped, re-scan      */
    SM_AVOID_LEFT,          /* bypass on the left                        */
    SM_AVOID_RIGHT,         /* bypass on the right                       */
    SM_AVOID_REVERSE,       /* reverse and re-attempt                    */
    SM_AVOID_COUNT
} sm_avoid_t;

#define SM_AVOID_BITS           3u

/* ---------------------------------------------------------------------
 *  Subsystems that report readiness during INITIALISING
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_SUBSYS_NET = 0,      /* S1 Networking: WiFi + MQTT up             */
    SM_SUBSYS_MOTION,       /* S2 Motion Control                         */
    SM_SUBSYS_LINE,         /* S3 Line Following + Barcode               */
    SM_SUBSYS_IMU,          /* S4 IMU                                    */
    SM_SUBSYS_ULTRASOUND,   /* S5 Ultrasound                             */
    SM_SUBSYS_COUNT
} sm_subsys_t;

#define SM_SUBSYS_BITS          3u
#define SM_READY_BIT(subsys)    ((uint8_t)(1u << (subsys)))

/*
 * Subsystems that must report ready before INITIALISING may move to
 * STANDBY.  Networking is "ready" while MQTT is connected.  Add a
 * subsystem's bit here once its task sends SM_EVT_READY, or override
 * from the build: make ... CFLAGS+=-DSM_REQUIRED_READY_MASK=0x1F
 */
#ifndef SM_REQUIRED_READY_MASK
#if TM_MQTT
#define SM_REQUIRED_READY_MASK  (SM_READY_BIT(SM_SUBSYS_NET))
#else
#define SM_REQUIRED_READY_MASK  (0u)
#endif
#endif

/* ---------------------------------------------------------------------
 *  Fault bitmask: one bit per fault condition (13 bits)
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_FAULT_MOTOR_STALL = 0,   /* S2: duty > 20 % and no pulses 300 ms */
    SM_FAULT_ENCODER,           /* S2: encoder fault                    */
    SM_FAULT_IMU,               /* S4: IMU not responding (non-fatal)   */
    SM_FAULT_NO_ECHO,           /* S5: ultrasonic gives no echo         */
    SM_FAULT_WIFI_LOSS,         /* S1: WiFi/MQTT down (non-fatal)       */
    SM_FAULT_COLLISION,         /* S4: horizontal spike > 1.5 g         */
    SM_FAULT_TIP_OVER,          /* S4: pitch/roll > 60 deg for 0.5 s    */
    SM_FAULT_IR_SENSOR,         /* S3: IR calibration / sensor fault    */
    SM_FAULT_LINE_RECOVERY,     /* S3/S5: line not reacquired in limit  */
    SM_FAULT_SCAN,              /* S5: scan timeout / invalid profile   */
    SM_FAULT_INIT_TIMEOUT,      /* S1: INITIALISING took too long       */
    SM_FAULT_SELF_TEST,         /* any: self-test failed                */
    SM_FAULT_RADIO,             /* S1: WiFi chip failed to initialise   */
    SM_FAULT_COUNT
} sm_fault_t;

#define SM_FAULT_BITS           13u
#define SM_FAULT_BIT(fault)     ((uint16_t)(1u << (fault)))
#define SM_FAULT_INDEX_BITS     4u

/* Faults that do not stop the car (design assumptions 3 and FR27) */
#define SM_FAULT_NON_FATAL_MASK (SM_FAULT_BIT(SM_FAULT_IMU) | \
                                 SM_FAULT_BIT(SM_FAULT_WIFI_LOSS))

/* ---------------------------------------------------------------------
 *  Control events (any task -> State Machine)
 *
 *  A control event is one 16-bit message: [code:5][arg:11].
 *  Codes and the meaning of arg:
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_EVT_NONE = 0,
    SM_EVT_CMD_START,       /* base station: start a run     arg: -      */
    SM_EVT_CMD_STOP,        /* base station: stop / park     arg: -      */
    SM_EVT_CMD_BIND,        /* base station: commit staged   arg: -      */
                            /*   barcode bindings                        */
    SM_EVT_NET_UP,          /* Networking: MQTT connected    arg: -      */
    SM_EVT_NET_DOWN,        /* Networking: WiFi/MQTT lost    arg: -      */
    SM_EVT_READY,           /* self-test passed       arg: sm_subsys_t   */
    SM_EVT_SELFTEST_FAIL,   /* self-test failed       arg: sm_subsys_t   */
    SM_EVT_FAULT,           /* fault detected         arg: sm_fault_t    */
    SM_EVT_OBSTACLE,        /* S5: obstacle ahead            arg: -      */
    SM_EVT_PROFILE_READY,   /* S5: profile done       arg: sm_avoid_t    */
    SM_EVT_BARCODE,         /* S3: barcode decoded           arg: see    */
                            /*   SM_BARCODE_ARG()                        */
    SM_EVT_JUNCTION,        /* S3: junction confirmed        arg: -      */
    SM_EVT_LINE_LOST,       /* S3: persistent line loss      arg: -      */
    SM_EVT_LINE_REACQUIRED, /* S3: stable line found again   arg: -      */
    SM_EVT_TURN_DONE,       /* S2: commanded turn finished   arg: -      */
    SM_EVT_BYPASS_DONE,     /* S5/S2: obstacle bypassed      arg: -      */
    SM_EVT_TIMEOUT,         /* internal to the State Machine             */
    SM_EVT_CHECK,           /* internal to the State Machine             */
    SM_EVT_COUNT
} sm_evt_code_t;

#define SM_EVT_CODE_BITS        5u
#define SM_EVT_ARG_BITS         11u
#define SM_EVT_ARG_MASK         ((1u << SM_EVT_ARG_BITS) - 1u)

#define SM_EVT_PACK(code, arg)  ((uint16_t)(((uint16_t)(code) << \
                                 SM_EVT_ARG_BITS) | ((arg) & SM_EVT_ARG_MASK)))
#define SM_EVT_CODE(evt)        ((sm_evt_code_t)((evt) >> SM_EVT_ARG_BITS))
#define SM_EVT_ARG(evt)         ((uint16_t)((evt) & SM_EVT_ARG_MASK))

/* SM_EVT_BARCODE arg: [symbol:5][action:2] */
#define SM_BARCODE_ARG(symbol, action) \
    ((uint16_t)(((uint16_t)(symbol) << SM_ACTION_BITS) | \
                ((uint16_t)(action) & ((1u << SM_ACTION_BITS) - 1u))))
#define SM_BARCODE_ARG_SYMBOL(arg)  ((uint8_t)((arg) >> SM_ACTION_BITS))
#define SM_BARCODE_ARG_ACTION(arg)  \
    ((sm_action_t)((arg) & ((1u << SM_ACTION_BITS) - 1u)))

/* ---------------------------------------------------------------------
 *  Navigation commands (State Machine -> Motion Control)
 *
 *  One byte: [cmd:3][arg:3].
 * --------------------------------------------------------------------- */
typedef enum
{
    SM_NAV_CMD_NONE = 0,
    SM_NAV_CMD_TURN,        /* arg: sm_action_t (left/right/straight/U)  */
    SM_NAV_CMD_HOLD,        /* stop (or slow) in front of an obstacle    */
    SM_NAV_CMD_AVOID        /* arg: sm_avoid_t (left/right/reverse)      */
} sm_nav_cmd_t;

#define SM_NAV_CMD_BITS         3u
#define SM_NAV_CMD_ARG_BITS     3u
#define SM_NAV_CMD_ARG_MASK     ((1u << SM_NAV_CMD_ARG_BITS) - 1u)
#define SM_NAV_CMD_PACK(cmd, arg) ((uint8_t)(((uint8_t)(cmd) << \
                                   SM_NAV_CMD_ARG_BITS) | \
                                   ((arg) & SM_NAV_CMD_ARG_MASK)))
#define SM_NAV_CMD_CODE(b)      ((sm_nav_cmd_t)((b) >> SM_NAV_CMD_ARG_BITS))
#define SM_NAV_CMD_ARG(b)       ((uint8_t)((b) & SM_NAV_CMD_ARG_MASK))

/* ---------------------------------------------------------------------
 *  Status word: the State Machine's current-state snapshot.
 *
 *  Packed into one aligned 32-bit word that only the State Machine
 *  writes, with a single store, so any task or core reads a consistent
 *  value without a mutex.
 *
 *    bits  0..2   state         (sm_state_t)
 *    bits  3..5   nav mode      (sm_nav_mode_t)
 *    bits  6..18  fault mask    (SM_FAULT_BIT(...))
 *    bits 19..20  error retries used
 * --------------------------------------------------------------------- */
#define SM_STATUS_STATE_POS     0u
#define SM_STATUS_NAV_POS       3u
#define SM_STATUS_FAULT_POS     6u
#define SM_STATUS_RETRY_POS     19u
#define SM_RETRY_BITS           2u

#define SM_FIELD_MASK(bits)     ((1u << (bits)) - 1u)

#define SM_STATUS_STATE(w)      ((sm_state_t)(((w) >> SM_STATUS_STATE_POS) \
                                 & SM_FIELD_MASK(SM_STATE_BITS)))
#define SM_STATUS_NAV(w)        ((sm_nav_mode_t)(((w) >> SM_STATUS_NAV_POS) \
                                 & SM_FIELD_MASK(SM_NAV_BITS)))
#define SM_STATUS_FAULTS(w)     ((uint16_t)(((w) >> SM_STATUS_FAULT_POS) \
                                 & SM_FIELD_MASK(SM_FAULT_BITS)))
#define SM_STATUS_RETRIES(w)    ((uint8_t)(((w) >> SM_STATUS_RETRY_POS) \
                                 & SM_FIELD_MASK(SM_RETRY_BITS)))

/* ---------------------------------------------------------------------
 *  Mode flags (event flag flg_mode): level bits written only by the
 *  State Machine.  Subsystems wait on or test them.
 *
 *    INITIALISING   STANDBY   NAVIGATING / AVOIDANCE   ERROR
 *    INIT           SENSORS   SENSORS | MOTORS         (none)
 * --------------------------------------------------------------------- */
#define SM_FLG_SENSORS_ENABLED  0x0001u
#define SM_FLG_MOTORS_ENABLED   0x0002u
#define SM_FLG_INITIALISING     0x0004u     /* run (or re-run) self-test */
#define SM_FLG_ALL              (SM_FLG_SENSORS_ENABLED | \
                                 SM_FLG_MOTORS_ENABLED | SM_FLG_INITIALISING)

/* Timing */
#define SM_INIT_TIMEOUT_MS      30000u  /* INITIALISING -> ERROR          */
#define SM_ERROR_RETRY_MS       2000u   /* ERROR -> INITIALISING retry    */
#define SM_MAX_RETRIES          3u      /* then stay in ERROR             */

/* Short names for the console and tests */
const char *sm_state_name(sm_state_t state);

#endif /* SM_TYPES_H */
