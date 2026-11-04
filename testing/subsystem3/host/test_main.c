/** @file test_main.c
 * @brief Synthetic fixtures only; no claim of physical recognition accuracy.
 */
#include "test_main.h"
#include "barcode_decoder.h"
#include "ir_transport.h"
#include <stdio.h>
#include <string.h>

#define TEST_PERIOD_MS 10u
#define TEST_CLEAR_MS 30u
#define TEST_PERSIST_MS 50u
#define TEST_PERSIST_SAMPLES 8u
#define TEST_CODE_END_MS 130u
#define TEST_CAPTURE_START_INDEX 4u
#define TEST_NEXT_SAMPLE_INDEX 5u
#define TEST_TIMEOUT_INDEX 8u
#define TEST_OVERFLOW_INDEX 37u
#define TEST_UNCLEAR_SAMPLES 10u

static unsigned int g_checks   = 0u;
static unsigned int g_failures = 0u;

typedef struct
{
    uint16_t time_ms;
    uint16_t raw[IR_CHANNEL_COUNT];
    uint8_t  failed_mask;
    uint16_t scan_ms;
} fake_adc_t;

typedef struct
{
    bool         b_locked;
    bool         b_fail;
    unsigned int acquisitions;
    unsigned int releases;
} fake_lock_t;

static void     check (bool b_passed, char const * p_name);
static uint16_t fake_clock (void * p_context);
static bool     fake_read (void * p_context, uint8_t channel, uint16_t * p_raw);
static ir_frame_t      make_frame (uint16_t time_ms, uint16_t sequence,
                                   uint16_t generation, uint16_t left,
                                   uint16_t barcode, uint16_t right);
static ir_config_t     line_config (void);
static ir_permission_t permission (void);
static void            test_codec (void);
static void            test_acquisition (void);
static void            test_line_math (void);
static void            test_persistence (void);
static bool synthetic_recognise (void * p_context, uint16_t const * p_runs,
                                 uint8_t count, barcode_direction_t direction,
                                 uint8_t * p_symbol);
static bool synthetic_bind (void * p_context, uint8_t symbol,
                            uint16_t binding_version, uint8_t * p_action);
static barcode_config_t barcode_config (void);
static barcode_result_t play_code (barcode_decoder_t * p_decoder,
                                   uint16_t base_ms, uint16_t base_sequence,
                                   bool b_reverse);
static void             test_barcode (void);
static void             begin_capture (barcode_decoder_t *      p_decoder,
                                       barcode_config_t const * p_config);
static void             test_barcode_rejection (void);
static bool             fake_lock (void * p_context);
static void             fake_unlock (void * p_context);
static void             test_transport (void);
static void             test_additional_guards (void);

static bool oversized_symbol (void * p_context, uint16_t const * p_runs,
                              uint8_t count, barcode_direction_t direction,
                              uint8_t * p_symbol);
static bool oversized_action (void * p_context, uint8_t symbol,
                              uint16_t binding_version, uint8_t * p_action);

int main (int argument_count, char * p_arguments[])
{
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "frame")))
    {
        test_codec();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "frame")))
    {
        test_acquisition();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "line")))
    {
        test_line_math();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "line")))
    {
        test_persistence();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "barcode")))
    {
        test_barcode();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "barcode")))
    {
        test_barcode_rejection();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "transport")))
    {
        test_transport();
    }
    if ((1 == argument_count) || (0 == strcmp(p_arguments[1], "guards")))
    {
        test_additional_guards();
    }
    if (0u == g_checks)
    {
        return 1;
    }

    printf("Checks: %u; failures: %u\n", g_checks, g_failures);
    printf("Storage bytes: frame=%zu line=%zu decoder=%zu transport=%zu\n",
           sizeof(ir_frame_t), sizeof(ir_processing_t),
           sizeof(barcode_decoder_t), sizeof(ir_transport_t));
    return (0u == g_failures) ? 0 : 1;
}

/** Print each independently meaningful result without aborting the suite. */
static void check (bool b_passed, char const * p_name)
{
    ++g_checks;
    if (!b_passed)
    {
        ++g_failures;
    }

    printf("%s %s\n", b_passed ? "PASS" : "FAIL", p_name);
}

/** Supply an injectable monotonic clock, including wrap. */
static uint16_t fake_clock (void * p_context)
{
    fake_adc_t const * p_adc = p_context;

    return p_adc->time_ms;
}

/** Advance time only after the last conversion to model scan skew. */
static bool fake_read (void * p_context, uint8_t channel, uint16_t * p_raw)
{
    fake_adc_t * p_adc = p_context;

    *p_raw = p_adc->raw[channel];
    if ((IR_CHANNEL_COUNT - 1u) == channel)
    {
        p_adc->time_ms += p_adc->scan_ms;
    }

    return 0u == (p_adc->failed_mask & (1u << channel));
}

/** Build a fixture through the same codec used by acquisition. */
static ir_frame_t make_frame (uint16_t time_ms, uint16_t sequence,
                              uint16_t generation, uint16_t left,
                              uint16_t barcode, uint16_t right)
{
    ir_sample_t const sample = {
        {left, right, barcode}, time_ms, sequence, generation, IR_VALID_ALL};
    ir_frame_t frame    = {{0u}};
    bool const b_packed = ir_frame_pack(&sample, &frame);

    if (!b_packed)
    {
        ++g_failures;
    }

    return frame;
}

/** Fixture roles are logical, not a statement about the assembled car. */
static ir_config_t line_config (void)
{
    ir_config_t config = {0};

    for (uint8_t channel = 0u; channel < IR_CHANNEL_COUNT; ++channel)
    {
        config.calibration[channel].white = 0u;
        config.calibration[channel].black = IR_ADC_MAX;
    }

    config.left_channel     = 0u;
    config.barcode_channel  = 2u;
    config.centre_channel   = 1u;
    config.minimum_contrast = 100u;
    config.minimum_signal   = 500u;
    config.dark_on          = 3000u;
    config.dark_off         = 1000u;
    config.max_gap_ms       = 20u;
    config.persistence_ms   = TEST_PERSIST_MS;
    return config;
}

/** Enable only the synthetic run used by each test. */
static ir_permission_t permission (void)
{
    ir_permission_t result = {0};

    result.generation = 1u;
    result.mode       = IR_MODE_TRACKING;
    result.b_enabled  = true;
    return result;
}

/** Golden bytes independently calculated from the field ledger. */
static void test_codec (void)
{
    uint8_t const golden[IR_FRAME_BYTES] = {0xbcu, 0x3au, 0x12u, 0xffu,
                                            0x7fu, 0x1au, 0x09u, 0x3cu,
                                            0x2bu, 0x5eu, 0x4du};
    ir_sample_t   sample                 = {
        {0xabcu, 0x123u, 0xfffu}, 0x1234u, 0x5678u, 0x9abcu, 7u};
    ir_sample_t decoded = {0};
    ir_frame_t  frame   = {{0u}};
    bool        b_exact = true;

    check(ir_frame_pack(&sample, &frame) &&
              (0 == memcmp(frame.bytes, golden, IR_FRAME_BYTES)),
          "C01 golden 87-bit frame");
    check(ir_frame_unpack(golden, sizeof(golden), &decoded) &&
              (decoded.generation == sample.generation) &&
              (decoded.time_ms == sample.time_ms) &&
              (decoded.sequence == sample.sequence),
          "C02 independent decode");

    for (uint16_t raw = 0u; raw <= IR_ADC_MAX; ++raw)
    {
        sample.raw[0]     = raw;
        sample.raw[1]     = IR_ADC_MAX - raw;
        sample.raw[2]     = raw;
        sample.valid_mask = raw % 8u;
        b_exact           = b_exact && ir_frame_pack(&sample, &frame) &&
                  ir_frame_unpack(frame.bytes, sizeof(frame.bytes), &decoded) &&
                  (sample.raw[0] == decoded.raw[0]) &&
                  (sample.raw[1] == decoded.raw[1]) &&
                  (sample.raw[2] == decoded.raw[2]) &&
                  (sample.valid_mask == decoded.valid_mask);
    }

    check(b_exact, "C03 exhaustive 4096 ADC values and validity masks");
    sample.raw[0] = IR_ADC_MAX + 1u;
    check(!ir_frame_pack(&sample, &frame), "C04 reject over-range ADC");
    sample.raw[0]     = 0u;
    sample.valid_mask = 8u;
    check(!ir_frame_pack(&sample, &frame), "C05 reject invalid validity mask");
    decoded.generation = 77u;
    frame.bytes[IR_FRAME_BYTES - 1u] |= 0x80u;
    check((!ir_frame_unpack(frame.bytes, IR_FRAME_BYTES, &decoded)) &&
              (77u == decoded.generation),
          "C06 reserved bits no partial commit");
    check((!ir_frame_unpack(golden, IR_FRAME_BYTES - 1u, &decoded)) &&
              (!ir_frame_unpack(golden, IR_FRAME_BYTES + 1u, &decoded)) &&
              (!ir_frame_unpack(NULL, IR_FRAME_BYTES, &decoded)),
          "C07 malformed lengths and null input");
    check(ir_frame_age(4u, 65530u, 10u) && (!ir_frame_age(4u, 65530u, 9u)) &&
              (!ir_frame_age(0u, 32768u, 32767u)) &&
              (!ir_frame_age(1u, 0u, 32768u)),
          "C08 wrap and half-range bounds");
}

/** Read errors never masquerade as legitimate zero-count samples. */
static void test_acquisition (void)
{
    fake_adc_t  adc    = {65530u, {0u, 2048u, 4095u}, 0u, 2u};
    ir_source_t source = {fake_clock, fake_read, &adc, 65535u};
    ir_frame_t  frame  = {{0u}};
    ir_sample_t sample = {0};

    check(ir_frame_acquire(&source, 1u, 5u, &frame) &&
              ir_frame_unpack(frame.bytes, sizeof(frame.bytes), &sample) &&
              (65535u == sample.sequence) && (0u == source.next_sequence) &&
              (65530u == sample.time_ms) && (7u == sample.valid_mask),
          "A01 acquisition timestamp and sequence wrap");
    adc.failed_mask = 2u;
    adc.raw[2]      = 4096u;
    check(ir_frame_acquire(&source, 1u, 5u, &frame) &&
              ir_frame_unpack(frame.bytes, sizeof(frame.bytes), &sample) &&
              (1u == sample.valid_mask) && (0u == sample.raw[0]),
          "A02 failed channel and out-of-range read, zero remains valid");
    adc.scan_ms = 6u;
    check(ir_frame_acquire(&source, 1u, 5u, &frame) &&
              ir_frame_unpack(frame.bytes, sizeof(frame.bytes), &sample) &&
              (0u == sample.valid_mask) && (2u == source.next_sequence),
          "A03 late scan invalidated, attempted sequence still advances");
}

/** Compare sign, clipping, role isolation and unusable calibration. */
static void test_line_math (void)
{
    ir_config_t      config      = line_config();
    ir_processing_t  line        = {0};
    ir_permission_t  control     = permission();
    ir_frame_t       frame       = make_frame(0u, 0u, 1u, 4095u, 0u, 0u);
    ir_result_t      result      = {0};
    ir_calibration_t calibration = {3500u, 500u};
    uint16_t         darkness    = 123u;

    check(ir_processing_normalise(calibration, 0u, 100u, &darkness) &&
              (4095u == darkness),
          "L01 reversed polarity clips to black");
    check(ir_processing_normalise(calibration, 4095u, 100u, &darkness) &&
              (0u == darkness),
          "L02 reversed polarity clips to white");
    calibration.black = calibration.white;
    darkness          = 123u;
    check((!ir_processing_normalise(calibration, 1000u, 100u, &darkness)) &&
              (123u == darkness),
          "L03 zero contrast leaves output unchanged");
    calibration.black = 3499u;
    check(!ir_processing_normalise(calibration, 1000u, 100u, &darkness),
          "L04 insufficient contrast rejected");
    check(ir_processing_init(&line, &config), "L05 valid role configuration");
    result = ir_processing_step(&line, &frame, control, 0u);
    check(result.b_target_valid && (0u == result.correction),
          "L06 left-only gives minus 100");
    frame  = make_frame(10u, 1u, 1u, 0u, 4095u, 4095u);
    result = ir_processing_step(&line, &frame, control, 10u);
    check(result.b_target_valid && (200u == result.correction),
          "L07 centre-only gives plus 100, barcode cannot steer");
    frame = make_frame(20u, 2u, 1u, 1500u, 0u, 1500u);
    ir_processing_reset(&line);
    result = ir_processing_step(&line, &frame, control, 20u);
    check(result.b_target_valid && (100u == result.correction),
          "L08 symmetric signal gives zero correction");
    frame  = make_frame(30u, 3u, 1u, 0u, 4095u, 0u);
    result = ir_processing_step(&line, &frame, control, 30u);
    check(!result.b_target_valid, "L09 absent line is invalid, not centred");
    control.mode = IR_MODE_SUSPENDED;
    frame        = make_frame(40u, 4u, 1u, 4095u, 0u, 0u);
    result       = ir_processing_step(&line, &frame, control, 40u);
    check(!result.b_target_valid, "L10 turn or bypass inhibits steering");
    control.mode = IR_MODE_TRACKING;
    result       = ir_processing_step(&line, &frame, control, 61u);
    check(result.b_fault && (!result.b_target_valid),
          "L11 stale target rejected");
    control.generation = 2u;
    result             = ir_processing_step(&line, &frame, control, 40u);
    check(result.b_fault && (!result.b_target_valid),
          "L12 old run cannot steer a new run");
    config.barcode_channel = config.left_channel;
    check((!ir_processing_init(&line, &config)) && (!line.b_initialised),
          "L13 duplicate roles inhibit initialisation");
}

/** Persistence is elapsed continuous time, not a count of task invocations. */
static void test_persistence (void)
{
    ir_config_t     config  = line_config();
    ir_processing_t line    = {0};
    ir_permission_t control = permission();
    ir_result_t     result  = {0};
    bool            b_early = false;
    unsigned int    events  = 0u;

    check(ir_processing_init(&line, &config), "P01 initialise persistence");
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const   stamp = index * TEST_PERIOD_MS;
        ir_frame_t const frame = make_frame(stamp, index, 1u, 0u, 0u, 0u);

        result = ir_processing_step(&line, &frame, control, stamp);
        if (result.b_lost)
        {
            ++events;
            b_early = b_early || (stamp < TEST_PERSIST_MS);
        }
    }

    check((1u == events) && (!b_early), "P02 loss once after full 50 ms");
    ir_processing_reset(&line);
    control.b_barcode_gap_expected = true;
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const   stamp = index * TEST_PERIOD_MS;
        ir_frame_t const frame = make_frame(stamp, index, 1u, 0u, 0u, 0u);

        result  = ir_processing_step(&line, &frame, control, stamp);
        b_early = b_early || result.b_lost;
    }

    check(!b_early, "P03 authorised barcode gap does not report loss");
    control.b_barcode_gap_expected = false;
    control.b_junction_expected    = true;
    ir_processing_reset(&line);
    events = 0u;
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const   stamp = index * TEST_PERIOD_MS;
        ir_frame_t const frame = make_frame(stamp, index, 1u, 4095u, 0u, 4095u);

        result = ir_processing_step(&line, &frame, control, stamp);
        if (result.b_junction)
        {
            ++events;
        }
    }

    check((1u == events) && (!result.b_target_valid),
          "P04 crossing consumed once and not used as position");
    control.mode                = IR_MODE_RECOVERY;
    control.b_recovery_expected = true;
    ir_processing_reset(&line);
    events = 0u;
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const   stamp = index * TEST_PERIOD_MS;
        ir_frame_t const frame = make_frame(stamp, index, 1u, 4095u, 0u, 0u);

        result = ir_processing_step(&line, &frame, control, stamp);
        if (result.b_reacquired)
        {
            ++events;
        }
    }

    check((1u == events) && (!result.b_target_valid),
          "P05 stable recovery reports candidate but cannot authorise motion");
    ir_processing_reset(&line);
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const   stamp = index * TEST_PERIOD_MS;
        ir_frame_t const frame =
            make_frame(stamp, index * 2u, 1u, 4095u, 0u, 0u);

        result = ir_processing_step(&line, &frame, control, stamp);
    }

    check(!result.b_reacquired, "P06 missing sequence breaks persistence");
    control.b_enabled = false;
    result            = ir_processing_step(&line, NULL, control, 100u);
    check((!result.b_target_valid) && (!line.b_have_frame),
          "P07 stop without ADC input resets evidence");
}

/** Synthetic vocabulary: dark 20, light 10, dark 30, or its reverse. */
static bool synthetic_recognise (void * p_context, uint16_t const * p_runs,
                                 uint8_t count, barcode_direction_t direction,
                                 uint8_t * p_symbol)
{
    uint16_t const first = (BARCODE_FORWARD == direction) ? 20u : 30u;
    uint16_t const last  = (BARCODE_FORWARD == direction) ? 30u : 20u;

    if (NULL != p_context)
    {
        return false;
    }

    if ((3u != count) || (p_runs[0] != (BARCODE_RUN_DARK | first)) ||
        (10u != p_runs[1]) || (p_runs[2] != (BARCODE_RUN_DARK | last)))
    {
        return false;
    }

    *p_symbol = 0u;
    return true;
}

/** Synthetic action mapping is not the course mapping. */
static bool synthetic_bind (void * p_context, uint8_t symbol,
                            uint16_t binding_version, uint8_t * p_action)
{
    if ((NULL != p_context) || (0u != symbol) || (1u != binding_version))
    {
        return false;
    }

    *p_action = 2u;
    return true;
}

/** Timing constants describe only these synthetic fixtures. */
static barcode_config_t barcode_config (void)
{
    barcode_config_t config = {0};

    config.calibration.black  = IR_ADC_MAX;
    config.p_recognise        = synthetic_recognise;
    config.p_bind             = synthetic_bind;
    config.max_gap_ms         = 20u;
    config.max_capture_ms     = 500u;
    config.clearance_ms       = TEST_CLEAR_MS;
    config.max_motion_skew_ms = 5u;
    config.channel            = 2u;
    config.minimum_contrast   = 100u;
    config.dark_on            = 3000u;
    config.dark_off           = 1000u;
    config.minimum_runs       = 3u;
    return config;
}

/** Replay a full bidirectional encounter with a separate Motion double. */
static barcode_result_t play_code (barcode_decoder_t * p_decoder,
                                   uint16_t base_ms, uint16_t base_sequence,
                                   bool b_reverse)
{
    barcode_result_t      result  = {0};
    ir_permission_t const control = permission();
    unsigned int const    direction =
        b_reverse ? BARCODE_REVERSE : BARCODE_FORWARD;
    uint16_t const first_end    = b_reverse ? 70u : 60u;
    uint16_t const second_start = first_end + 10u;

    for (uint16_t offset = 0u; offset <= TEST_CODE_END_MS;
         offset += TEST_PERIOD_MS)
    {
        bool const b_dark = ((offset >= 40u) && (offset < first_end)) ||
                            ((offset >= second_start) && (offset < 100u));
        /* Fixture clock and sequence deliberately wrap in the wrap test. */
        uint16_t const stamp = (uint16_t)(base_ms + offset);
        uint16_t const sequence =
            (uint16_t)(base_sequence + (offset / TEST_PERIOD_MS));
        ir_frame_t const frame =
            make_frame(stamp, sequence, 1u, 4095u, b_dark ? 4095u : 0u, 0u);
        barcode_motion_t motion = {0};

        motion.time_ms    = stamp;
        motion.generation = 1u;
        motion.direction  = direction;
        motion.b_valid    = true;
        result =
            barcode_decoder_step(p_decoder, &frame, control, 1u, motion, stamp);
    }

    return result;
}

/** Accepted and unsupported outcomes both require a new clear encounter. */
static void test_barcode (void)
{
    barcode_config_t  config  = barcode_config();
    barcode_decoder_t decoder = {0};
    barcode_result_t  result  = {0};

    check(barcode_decoder_init(&decoder, &config), "B01 initialise decoder");
    result = play_code(&decoder, 0u, 0u, false);
    check((BARCODE_ACCEPTED == result.outcome) && (0u == result.symbol) &&
              (2u == result.action) && (BARCODE_WAIT_CLEAR == decoder.state),
          "B02 forward synthetic code accepted once then waits for clearance");
    result = play_code(&decoder, 140u, 14u, true);
    check(BARCODE_ACCEPTED == result.outcome,
          "B03 reverse synthetic code, same symbol on a later encounter");
    barcode_decoder_reset(&decoder);
    result = play_code(&decoder, 65500u, 65530u, false);
    check(BARCODE_ACCEPTED == result.outcome,
          "B04 continuous capture across clock and sequence wrap");
    config.p_recognise = NULL;
    check(barcode_decoder_init(&decoder, &config), "B05 no codebook is legal");
    result = play_code(&decoder, 0u, 0u, false);
    check(BARCODE_INVALID_CODE == result.outcome,
          "B06 default Code 39 rejects the synthetic vocabulary");
    config.p_recognise = synthetic_recognise;
    config.p_bind      = NULL;
    check(barcode_decoder_init(&decoder, &config), "B07 no binding is legal");
    result = play_code(&decoder, 0u, 0u, false);
    check(BARCODE_UNBOUND == result.outcome, "B08 unbound code rejected");
    config.p_bind    = synthetic_bind;
    config.p_context = &decoder;
    check(barcode_decoder_init(&decoder, &config), "B09 invalid code setup");
    result = play_code(&decoder, 0u, 0u, false);
    check(BARCODE_INVALID_CODE == result.outcome, "B10 invalid code rejected");
    config.p_context   = NULL;
    config.p_recognise = oversized_symbol;
    check(barcode_decoder_init(&decoder, &config), "B11 symbol bound setup");
    result = play_code(&decoder, 0u, 0u, false);
    check(BARCODE_INVALID_CODE == result.outcome,
          "B12 out-of-range symbol rejected before five-bit storage");
    config.p_recognise = synthetic_recognise;
    config.p_bind      = oversized_action;
    check(barcode_decoder_init(&decoder, &config), "B13 action bound setup");
    result = play_code(&decoder, 0u, 0u, false);
    check(BARCODE_UNBOUND == result.outcome,
          "B14 out-of-range action rejected before two-bit storage");
}

/** Prepare an active capture without bypassing the clearance state. */
static void begin_capture (barcode_decoder_t *      p_decoder,
                           barcode_config_t const * p_config)
{
    bool const            b_ready = barcode_decoder_init(p_decoder, p_config);
    ir_permission_t const control = permission();
    barcode_motion_t      motion  = {0u, 1u, BARCODE_FORWARD, true};

    if (!b_ready)
    {
        ++g_failures;
    }

    for (uint16_t index = 0u; index <= TEST_CAPTURE_START_INDEX; ++index)
    {
        uint16_t const   stamp = index * TEST_PERIOD_MS;
        ir_frame_t const frame =
            make_frame(stamp, index, 1u, 4095u, (4u == index) ? 4095u : 0u, 0u);
        barcode_result_t result = {0};

        motion.time_ms = stamp;
        result =
            barcode_decoder_step(p_decoder, &frame, control, 1u, motion, stamp);
        if (BARCODE_NONE != result.outcome)
        {
            ++g_failures;
        }
    }
}

/** Missing, interrupted and ambiguous evidence consumes the whole capture. */
static void test_barcode_rejection (void)
{
    barcode_config_t  config  = barcode_config();
    barcode_decoder_t decoder = {0};
    ir_permission_t   control = permission();
    barcode_motion_t  motion  = {50u, 1u, BARCODE_FORWARD, true};
    ir_frame_t        frame   = make_frame(50u, 6u, 1u, 4095u, 4095u, 0u);
    barcode_result_t  result  = {0};

    begin_capture(&decoder, &config);
    result = barcode_decoder_step(&decoder, &frame, control, 1u, motion, 50u);
    check((BARCODE_GAP == result.outcome) &&
              (BARCODE_WAIT_CLEAR == decoder.state),
          "R01 dropped frame rejects");
    begin_capture(&decoder, &config);
    frame          = make_frame(40u, 4u, 1u, 4095u, 4095u, 0u);
    motion.time_ms = 40u;
    result = barcode_decoder_step(&decoder, &frame, control, 1u, motion, 40u);
    check(BARCODE_GAP == result.outcome, "R02 duplicate frame rejects");
    begin_capture(&decoder, &config);
    frame            = make_frame(50u, 5u, 1u, 4095u, 4095u, 0u);
    motion.time_ms   = 50u;
    motion.direction = BARCODE_REVERSE;
    result = barcode_decoder_step(&decoder, &frame, control, 1u, motion, 50u);
    check(BARCODE_DIRECTION == result.outcome, "R03 mid-code reversal rejects");
    begin_capture(&decoder, &config);
    motion.direction = BARCODE_FORWARD;
    motion.time_ms   = 20u;
    result = barcode_decoder_step(&decoder, &frame, control, 1u, motion, 50u);
    check(BARCODE_DIRECTION == result.outcome, "R04 stale Motion rejects");
    begin_capture(&decoder, &config);
    control.b_enabled = false;
    result = barcode_decoder_step(&decoder, NULL, control, 1u, motion, 50u);
    check(BARCODE_INTERRUPTED == result.outcome,
          "R05 stop without ADC interrupts capture");
    control.b_enabled  = true;
    control.generation = 2u;
    result = barcode_decoder_step(&decoder, &frame, control, 1u, motion, 50u);
    check(BARCODE_STALE == result.outcome, "R06 queued old-run frame rejected");
    control.generation = 1u;
    begin_capture(&decoder, &config);
    motion.time_ms = 50u;
    result = barcode_decoder_step(&decoder, &frame, control, 2u, motion, 50u);
    check(BARCODE_INTERRUPTED == result.outcome,
          "R07 binding version change interrupts capture");
    config.max_capture_ms = 40u;
    begin_capture(&decoder, &config);
    for (uint16_t index = TEST_NEXT_SAMPLE_INDEX; index <= TEST_TIMEOUT_INDEX;
         ++index)
    {
        uint16_t const stamp = index * TEST_PERIOD_MS;

        frame          = make_frame(stamp, index, 1u, 4095u, 4095u, 0u);
        motion.time_ms = stamp;
        result =
            barcode_decoder_step(&decoder, &frame, control, 1u, motion, stamp);
    }

    check(BARCODE_TIMEOUT == result.outcome, "R08 stuck-dark capture timeout");
    config.max_capture_ms = 500u;
    begin_capture(&decoder, &config);
    for (uint16_t index = TEST_NEXT_SAMPLE_INDEX; index <= TEST_OVERFLOW_INDEX;
         ++index)
    {
        uint16_t const stamp = index * TEST_PERIOD_MS;

        frame          = make_frame(stamp, index, 1u, 4095u,
                           (0u == (index % 2u)) ? 4095u : 0u, 0u);
        motion.time_ms = stamp;
        result =
            barcode_decoder_step(&decoder, &frame, control, 1u, motion, stamp);
    }

    check(BARCODE_OVERFLOW == result.outcome,
          "R09 run history overflow rejects instead of truncating");
}

/** Model lock acquisition failure and detect accidental nested access. */
static bool fake_lock (void * p_context)
{
    fake_lock_t * p_lock = p_context;

    if (p_lock->b_locked || p_lock->b_fail)
    {
        return false;
    }

    p_lock->b_locked = true;
    ++p_lock->acquisitions;
    return true;
}

/** Verify every successful acquisition is released. */
static void fake_unlock (void * p_context)
{
    fake_lock_t * p_lock = p_context;

    p_lock->b_locked = false;
    ++p_lock->releases;
}

/** Model queue-full fault delivery and stale recovery after a second fault. */
static void test_transport (void)
{
    fake_lock_t    lock      = {false, false, 0u, 0u};
    ir_transport_t transport = {0};
    ir_frame_t     frame     = make_frame(0u, 0u, 1u, 4095u, 0u, 0u);
    ir_frame_t     received  = {{0u}};
    ir_result_t    target    = {1u, 0u, 100u, true, false, false, false, false};
    ir_result_t    snapshot  = {0};
    bool           b_filled  = true;

    check(ir_transport_init(&transport, fake_lock, fake_unlock, &lock) &&
              (!ir_transport_offer(&transport, &frame)),
          "T01 transport begins inhibited");
    check(ir_transport_stop(&transport, 1u, false) &&
              ir_transport_resume(&transport, 1u, 0u, true),
          "T02 explicit readiness and generation enable transport");
    for (uint8_t index = 0u; index < IR_QUEUE_CAPACITY; ++index)
    {
        frame    = make_frame(index * TEST_PERIOD_MS, index, 1u, 4095u, 0u, 0u);
        b_filled = b_filled && ir_transport_offer(&transport, &frame);
    }

    check(b_filled && (!ir_transport_offer(&transport, &frame)) &&
              (1u == transport.dropped),
          "T03 bounded queue drops newest");
    check(ir_transport_take(&transport, &received) && (3u == transport.count),
          "T04 consumer removes exactly one frame");
    check(ir_transport_publish(&transport, &target) &&
              ir_transport_snapshot(&transport, 10u, 20u, &snapshot) &&
              snapshot.b_target_valid,
          "T05 coherent fresh target snapshot");
    check(ir_transport_snapshot(&transport, 21u, 20u, &snapshot) &&
              (!snapshot.b_target_valid),
          "T06 stale snapshot invalidates target");
    check(ir_transport_offer(&transport, &frame) &&
              (IR_QUEUE_CAPACITY == transport.count),
          "T12 refill queue before fault injection");
    check(ir_transport_stop(&transport, 2u, true) && transport.b_fault &&
              transport.b_inhibited && (0u == transport.count) &&
              (!ir_transport_publish(&transport, &target)),
          "T07 fault latch drains backlog and prevents old publication");
    check(ir_transport_resume(&transport, 2u, 1u, true) &&
              ir_transport_stop(&transport, 3u, true) &&
              (!ir_transport_resume(&transport, 3u, 1u, true)),
          "T08 stale recovery cannot clear a newer fault");
    lock.b_fail             = true;
    snapshot.b_target_valid = true;
    check((!ir_transport_snapshot(&transport, 0u, 20u, &snapshot)) &&
              (!snapshot.b_target_valid),
          "T09 failed lock leaves no valid target");
    lock.b_fail           = false;
    transport.fault_token = UINT16_MAX;
    check(ir_transport_stop(&transport, 4u, true) &&
              (!ir_transport_resume(&transport, 4u, UINT16_MAX, true)),
          "T10 exhausted fault token cannot alias an old recovery");
    check((lock.acquisitions == lock.releases) && (!lock.b_locked),
          "T11 every successful lock released");
}

/** Reject malformed evidence and require clearance after restart. */
static void test_additional_guards (void)
{
    barcode_config_t  config        = barcode_config();
    barcode_decoder_t decoder       = {0};
    ir_permission_t   control       = permission();
    barcode_motion_t  motion        = {50u, 1u, BARCODE_FORWARD, true};
    barcode_result_t  decoded       = {0};
    ir_frame_t        frame         = make_frame(50u, 5u, 1u, 4095u, 0u, 0u);
    ir_sample_t       sample        = {0};
    ir_config_t       line_settings = line_config();
    ir_processing_t   line          = {0};
    ir_result_t       result        = {0};
    unsigned int      accepted      = 0u;

    begin_capture(&decoder, &config);
    check(ir_frame_unpack(frame.bytes, sizeof(frame.bytes), &sample),
          "G01 invalid-sensor fixture decodes");
    sample.valid_mask = 3u;
    check(ir_frame_pack(&sample, &frame), "G02 invalid-sensor fixture packs");
    decoded = barcode_decoder_step(&decoder, &frame, control, 1u, motion, 50u);
    check(BARCODE_SENSOR_FAULT == decoded.outcome,
          "G03 invalid barcode channel rejects active capture");
    begin_capture(&decoder, &config);
    frame = make_frame(50u, 5u, 1u, 4095u, 0u, 0u);
    for (uint16_t index = TEST_NEXT_SAMPLE_INDEX; index <= TEST_TIMEOUT_INDEX;
         ++index)
    {
        uint16_t const stamp = index * TEST_PERIOD_MS;

        frame          = make_frame(stamp, index, 1u, 4095u, 0u, 0u);
        motion.time_ms = stamp;
        decoded =
            barcode_decoder_step(&decoder, &frame, control, 1u, motion, stamp);
    }

    check(BARCODE_INCOMPLETE == decoded.outcome,
          "G04 truncated one-run code cannot reach recogniser");
    barcode_decoder_reset(&decoder);
    for (uint16_t index = 0u; index < TEST_UNCLEAR_SAMPLES; ++index)
    {
        uint16_t const stamp = index * TEST_PERIOD_MS;

        frame          = make_frame(stamp, index, 1u, 4095u, 2000u, 0u);
        motion.time_ms = stamp;
        decoded =
            barcode_decoder_step(&decoder, &frame, control, 1u, motion, stamp);
        if (BARCODE_ACCEPTED == decoded.outcome)
        {
            ++accepted;
        }
    }

    check((BARCODE_WAIT_CLEAR == decoder.state) && (0u == accepted),
          "G05 restart on ambiguous reflectance never proves clearance");
    begin_capture(&decoder, &config);
    control.mode = IR_MODE_SUSPENDED;
    decoded = barcode_decoder_step(&decoder, NULL, control, 1u, motion, 50u);
    check(BARCODE_INTERRUPTED == decoded.outcome,
          "G06 manoeuvre invalidates capture without waiting for a frame");
    control = permission();
    check(ir_processing_init(&line, &line_settings), "G07 line guard setup");
    frame  = make_frame(0u, 0u, 1u, 4095u, 0u, 0u);
    result = ir_processing_step(&line, &frame, control, 0u);
    result = ir_processing_step(&line, &frame, control, 0u);
    check(result.b_fault && (!result.b_target_valid),
          "G08 duplicate frame cannot refresh steering");
    ir_processing_reset(&line);
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const stamp = index * TEST_PERIOD_MS;

        frame  = make_frame(stamp, index, 1u, 4095u, 0u, 4095u);
        result = ir_processing_step(&line, &frame, control, stamp);
        if (result.b_junction)
        {
            ++accepted;
        }
    }

    check(0u == accepted, "G09 crossing without route context is not junction");
    ir_processing_reset(&line);
    control.mode = IR_MODE_RECOVERY;
    for (uint16_t index = 0u; index < TEST_PERSIST_SAMPLES; ++index)
    {
        uint16_t const stamp = index * TEST_PERIOD_MS;

        frame  = make_frame(stamp, index, 1u, 4095u, 0u, 0u);
        result = ir_processing_step(&line, &frame, control, stamp);
        if (result.b_reacquired)
        {
            ++accepted;
        }
    }

    check(0u == accepted, "G10 recovery requires external route evidence");
}

/** Deliberately return the first reserved symbol encoding. */
static bool oversized_symbol (void * p_context, uint16_t const * p_runs,
                              uint8_t count, barcode_direction_t direction,
                              uint8_t * p_symbol)
{
    *p_symbol = BARCODE_SYMBOL_COUNT;
    return (NULL == p_context) && (NULL != p_runs) && (0u != count) &&
           (BARCODE_FORWARD == direction);
}

/** Deliberately return an action that would truncate to a valid value. */
static bool oversized_action (void * p_context, uint8_t symbol,
                              uint16_t binding_version, uint8_t * p_action)
{
    *p_action = BARCODE_ACTION_COUNT;
    return (NULL == p_context) && (0u == symbol) && (1u == binding_version);
}

/* End of file. */

