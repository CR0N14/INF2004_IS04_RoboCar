/** @file adapter_main.c
 * @brief Real S3/S1 headers and codec with deterministic kernel call doubles.
 */
#include "adapter_main.h"
#include <stdio.h>
#include "subsystem3.h"
#include "../../../app_program/subsystem1/state_machine.h"
#include "../../../app_program/subsystem1/net_codec.h"

#define TEST_CAPTURE_SAMPLES 8u

static unsigned int g_checks        = 0u;
static unsigned int g_failures      = 0u;
static unsigned int g_locks         = 0u;
static unsigned int g_unlocks       = 0u;
static bool         g_b_fail_create = false;
static bool         g_b_fail_lock   = false;
static bool         g_b_locked      = false;
static uint16_t     g_time_ms       = 0u;

static void             check (bool b_passed, char const * p_name);
static ir_frame_t       frame_at (uint16_t time_ms, uint16_t sequence,
                                  uint16_t generation, uint16_t barcode);
static ir_config_t      line_config (void);
static barcode_config_t decoder_config (void);
static bool recognise (void * p_context, uint16_t const * p_runs, uint8_t count,
                       barcode_direction_t direction, uint8_t * p_symbol);
static bool bind_symbol (void * p_context, uint8_t symbol, uint16_t version,
                         uint8_t * p_action);
static void test_default_start (void);
static void test_configured (void);
static void test_decoder (ir_permission_t permission);
static void test_network (void);

/** Return errors exactly as the kernel creation interface does. */
ID tk_cre_mtx (T_CMTX const * p_attributes)
{
    if (g_b_fail_create || (TA_INHERIT != p_attributes->mtxatr))
    {
        return E_NOMEM;
    }

    return 1;
}

/** A poll can fail; the adapter must never access shared state afterward. */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    if ((1 != h_mutex) || (TMO_POL != timeout))
    {
        return E_PAR;
    }

    if (g_b_fail_lock || g_b_locked)
    {
        return E_TMOUT;
    }

    g_b_locked = true;
    ++g_locks;
    return E_OK;
}

/** Detect unbalanced ownership instead of silently accepting it. */
ER tk_unl_mtx (ID h_mutex)
{
    if ((1 != h_mutex) || (!g_b_locked))
    {
        return E_OBJ;
    }

    g_b_locked = false;
    ++g_unlocks;
    return E_OK;
}

/** Use the same clock domain as the synthetic frame timestamps. */
ER tk_get_otm (SYSTIM * p_time)
{
    p_time->hi = 0;
    p_time->lo = g_time_ms;
    return E_OK;
}

/** The host adapter checks S3 calls without a running kernel queue. */
ER sm_post_event (sm_evt_code_t code, uint16_t arg)
{
    return ((SM_EVT_BARCODE == code) || (SM_EVT_JUNCTION == code)) &&
                   (arg <= SM_EVT_ARG_MASK)
               ? E_OK
               : E_PAR;
}

/** No instructor table exists in this test process. */
bool sm_barcode_lookup (uint8_t symbol, sm_action_t * p_action)
{
    (void)symbol;
    (void)p_action;
    return false;
}

int main (int argument_count, char * p_arguments[])
{
    if ((argument_count > 1) && (NULL != p_arguments[1]))
    {
        test_default_start();
    }
    else
    {
        test_configured();
    }

    check((g_locks == g_unlocks) && (!g_b_locked),
          "I99 all successful locks released");
    printf("Adapter checks: %u; failures: %u\n", g_checks, g_failures);
    return (0u == g_failures) ? 0 : 1;
}

/** Keep all test failures visible in a single deterministic transcript. */
static void check (bool b_passed, char const * p_name)
{
    ++g_checks;
    if (!b_passed)
    {
        ++g_failures;
    }

    printf("%s %s\n", b_passed ? "PASS" : "FAIL", p_name);
}

/** ADC0 is black and ADC1 is white; barcode uses ADC2 only. */
static ir_frame_t frame_at (uint16_t time_ms, uint16_t sequence,
                            uint16_t generation, uint16_t barcode)
{
    ir_sample_t const sample = {
        {4095u, 0u, barcode}, time_ms, sequence, generation, 7u};
    ir_frame_t frame    = {{0u}};
    bool const b_packed = ir_frame_pack(&sample, &frame);

    if (!b_packed)
    {
        ++g_failures;
    }

    return frame;
}

/** Synthetic calibration is installed only by this test executable. */
static ir_config_t line_config (void)
{
    ir_config_t config = {0};

    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        config.calibration[channel].black = 4095u;
    }

    config.left_channel     = 0u;
    config.centre_channel   = 1u;
    config.barcode_channel  = 2u;
    config.minimum_contrast = 100u;
    config.minimum_signal   = 500u;
    config.dark_on          = 3000u;
    config.dark_off         = 1000u;
    config.max_gap_ms       = 20u;
    config.persistence_ms   = 50u;
    return config;
}

/** Accept only one synthetic twenty-millisecond dark run. */
static bool recognise (void * p_context, uint16_t const * p_runs, uint8_t count,
                       barcode_direction_t direction, uint8_t * p_symbol)
{
    *p_symbol = 25u;
    return (NULL == p_context) && (1u == count) &&
           ((BARCODE_RUN_DARK | 20u) == p_runs[0]) &&
           (BARCODE_FORWARD == direction);
}

/** Use the maximum valid symbol/action to test IF-07 field preservation. */
static bool bind_symbol (void * p_context, uint8_t symbol, uint16_t version,
                         uint8_t * p_action)
{
    *p_action = 3u;
    return (NULL == p_context) && (25u == symbol) && (1u == version);
}

static barcode_config_t decoder_config (void)
{
    barcode_config_t config = {0};

    config.calibration.black  = 4095u;
    config.channel            = 2u;
    config.minimum_contrast   = 100u;
    config.dark_on            = 3000u;
    config.dark_off           = 1000u;
    config.max_gap_ms         = 20u;
    config.max_capture_ms     = 500u;
    config.clearance_ms       = 20u;
    config.max_motion_skew_ms = 5u;
    config.minimum_runs       = 1u;
    config.p_recognise        = recognise;
    config.p_bind             = bind_symbol;
    return config;
}

/** Existing usermain may initialise and start without measured calibration. */
static void test_default_start (void)
{
    s3_tlm_t    telemetry = {0};
    ir_result_t target    = {0};

    check(E_OK == subsystem3_init(), "D01 initialise unconfigured service");
    check(E_OK == subsystem3_start(), "D02 default startup remains compatible");
    check(E_NOSPT == s3_get_tlm(&telemetry),
          "D03 unconfigured S3 remains omitted by Networking");
    check((E_OBJ == subsystem3_get_target(0u, &target)) &&
              (!target.b_target_valid),
          "D04 no invented steering or readiness");
}

/** Exercise lifecycle failures, snapshot mapping and generation invalidation.
 */
static void test_configured (void)
{
    ir_config_t      line       = line_config();
    barcode_config_t decoder    = decoder_config();
    ir_permission_t  permission = {1u,   IR_MODE_TRACKING, true, false, false,
                                   false};
    ir_result_t      result     = {0};
    s3_tlm_t         telemetry  = {0};
    ir_frame_t       frame      = frame_at(0u, 0u, 1u, 0u);

    check(E_OBJ == subsystem3_start(), "I01 start before init rejected");
    g_b_fail_create = true;
    check(E_NOMEM == subsystem3_init(),
          "I02 mutex creation failure propagated");
    g_b_fail_create = false;
    check(E_OK == subsystem3_init(), "I03 failed init can retry");
    check(E_OBJ == subsystem3_init(), "I04 repeated init rejected");
    check(E_NOSPT == s3_get_tlm(&telemetry),
          "I05 unconfigured telemetry absent");
    decoder.channel = 0u;
    check(E_PAR == subsystem3_configure(&line, &decoder),
          "I06 inconsistent barcode role rejected transactionally");
    decoder.channel = 2u;
    check(E_OK == subsystem3_configure(&line, &decoder), "I07 valid configure");
    check(E_OK == subsystem3_start(), "I08 existing start entry works");
    check(E_OBJ == subsystem3_configure(&line, &decoder),
          "I09 running configuration cannot be replaced");
    check(E_OBJ == subsystem3_start(), "I10 repeated start rejected");
    check(E_OK == subsystem3_set_context(permission, 1u),
          "I11 explicit context");
    check((E_OK == subsystem3_line_step(&frame, 0u, &result)) &&
              result.b_target_valid,
          "I12 application reaches production core");
    check((E_OK == s3_get_tlm(&telemetry)) && telemetry.b_line_valid &&
              (100u == telemetry.line_pos) && telemetry.b_line_left &&
              (S3_IR_LEFT == telemetry.ir_pattern),
          "I13 S1 magnitude/sign/pin map");
    g_time_ms = 21u;
    check((E_OK == s3_get_tlm(&telemetry)) && (!telemetry.b_line_valid),
          "I14 stale telemetry does not claim valid line");
    check((E_OK == subsystem3_get_target(21u, &result)) &&
              (!result.b_target_valid),
          "I15 Motion target expires");
    g_b_fail_lock         = true;
    result.b_target_valid = true;
    check((E_TMOUT == subsystem3_get_target(0u, &result)) &&
              (!result.b_target_valid),
          "I16 lock failure returns invalid target");
    check(E_TMOUT == subsystem3_set_context(permission, 1u),
          "I17 failed context update is not reported as success");
    g_b_fail_lock = false;
    check((E_OK == subsystem3_line_step(&frame, 0u, &result)) && result.b_fault,
          "I18 duplicate frame latches application fault");
    check(E_OBJ == subsystem3_set_context(permission, 1u),
          "I19 same-generation restart cannot clear fault");
    permission.generation = 2u;
    check(E_OK == subsystem3_set_context(permission, 1u),
          "I20 controller recovery requires fresh generation");
    frame = frame_at(0u, 0u, 2u, 0u);
    check((E_OK == subsystem3_line_step(&frame, 0u, &result)) &&
              result.b_target_valid,
          "I21 fresh-run steering resumes");
    test_decoder(permission);
    permission.b_enabled = false;
    check(E_OK == subsystem3_set_context(permission, 1u),
          "I22 stop resets both processors without ADC input");
    check((E_OK == subsystem3_get_target(0u, &result)) &&
              (!result.b_target_valid),
          "I23 stopped target cannot survive");
    check((E_OK == s3_get_tlm(&telemetry)) && (!telemetry.b_barcode),
          "I24 pending barcode cleared on stop");
    check((E_PAR == s3_get_tlm(NULL)) &&
              (E_PAR == subsystem3_get_target(0u, NULL)),
          "I25 null outputs rejected");
}

/** Decode through the application adapter, then encode with unmodified S1. */
static void test_decoder (ir_permission_t permission)
{
    barcode_result_t result    = {0};
    barcode_motion_t motion    = {0u, 2u, BARCODE_FORWARD, true};
    s3_tlm_t         telemetry = {0};

    for (uint16_t index = 0u; index < TEST_CAPTURE_SAMPLES; ++index)
    {
        uint16_t const   stamp  = index * 10u;
        bool const       b_dark = (index >= 3u) && (index < 5u);
        ir_frame_t const frame =
            frame_at(stamp, index, permission.generation, b_dark ? 4095u : 0u);

        motion.time_ms = stamp;
        check(E_OK == subsystem3_barcode_step(&frame, motion, stamp, &result),
              "I26 decoder frame routed");
    }

    check((BARCODE_ACCEPTED == result.outcome) && (25u == result.symbol) &&
              (3u == result.action),
          "I27 synthetic result preserves domains");
    check((E_OK == s3_get_tlm(&telemetry)) && telemetry.b_barcode &&
              (25u == telemetry.last_symbol) && (3u == telemetry.last_action) &&
              (S3_DECODER_WAIT_CLEAR == telemetry.decoder_state),
          "I28 private decoder states translated to public IF-07 states");
    test_network();
}

/** Read the actual S1 bitstream, including every existing S3 field. */
static void test_network (void)
{
    net_tlm_in_t input                    = {0};
    uint8_t      bytes[NET_TLM_MAX_BYTES] = {0u};
    bitread_t    reader                   = {0};

    g_time_ms = 0u;
    check(E_OK == s3_get_tlm(&input.s3), "I29 Networking gets strong snapshot");
    input.present         = NET_HAS_S3;
    uint16_t const length = net_codec_tlm(&input, bytes, NET_TLM_MAX_BYTES);

    bitread_init(&reader, bytes, length);
    check((8u == length) && (0u == bitread_get(&reader, 8u)) &&
              (0u == bitread_get(&reader, 19u)) &&
              (NET_HAS_S3 == bitread_get(&reader, 4u)),
          "I30 S1 header unchanged");
    check((S3_LINE_TRACKING == bitread_get(&reader, 3u)) &&
              (1u == bitread_get(&reader, 1u)) &&
              (100u == bitread_get(&reader, 7u)) &&
              (1u == bitread_get(&reader, 1u)) &&
              (S3_IR_LEFT == bitread_get(&reader, 3u)) &&
              (S3_DECODER_WAIT_CLEAR == bitread_get(&reader, 3u)) &&
              (1u == bitread_get(&reader, 1u)) &&
              (25u == bitread_get(&reader, 5u)) &&
              (3u == bitread_get(&reader, 2u)) && (!reader.b_underflow),
          "I31 complete 26-bit S3 payload remains compatible");
}

/* End of file. */

