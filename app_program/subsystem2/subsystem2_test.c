/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 *
 *	Run with the car's WHEELS OFF THE GROUND and nobody touching them: the
 *	tests expect the wheels to be still.  None of these tests drives the
 *	motors (the Motion Control task is the only PWM writer); driving tests
 *	are manual, see TEST_LOG.md.
 */

#include "subsystem2.h"
#include "../test_report.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_logic.h"

#if APP_TEST

#define PWM_SLICE_REG(slice, off)   (PWM_BASE + (off) + ((slice) * 0x14u))
#define PWM_SLICE_OF(gpio)          (((gpio) >> 1) & 0x7u)
#define EXPECTED_PWM_TOP            6249u   /* 125 MHz / 20 kHz - 1 */
#define EXPECTED_PWM_DIV            0x10u   /* integer divider 1    */

static BOOL
slice_ok(UINT gpio)
{
	UINT const s = PWM_SLICE_OF(gpio);

	return (in_w(PWM_SLICE_REG(s, PWM_CHx_TOP)) == EXPECTED_PWM_TOP) &&
	       (in_w(PWM_SLICE_REG(s, PWM_CHx_DIV)) == EXPECTED_PWM_DIV) &&
	       (0u != (in_w(PWM_SLICE_REG(s, PWM_CHx_CSR)) & PWM_CH_CSR_EN));
}

static BOOL
slice_braked(UINT gpio)
{
	/* CC holds both channels' compare values: 0 = both pins LOW = brake */
	return 0u == in_w(PWM_SLICE_REG(PWM_SLICE_OF(gpio), PWM_CHx_CC));
}

/* Sum of quadrature steps over a sequence of 2-bit states */
static INT
qdec_sum(uint8_t const *p_states, INT n)
{
	INT sum = 0;

	for (INT i = 1; i < n; i++) {
		sum += drv_encoder_qdec(p_states[i - 1], p_states[i]);
	}
	return sum;
}

EXPORT void subsystem2_test(void)
{
	static uint8_t const fwd[] = { 0x0, 0x2, 0x3, 0x1, 0x0 };  /* 00 10 11 01 00 */
	static uint8_t const rev[] = { 0x0, 0x1, 0x3, 0x2, 0x0 };  /* 00 01 11 10 00 */
	int32_t   l, r;
	ml_pid_t  pid;
	ml_stall_t st;
	s2_tlm_t  tlm;
	BOOL      b_ok;

	TEST_INIT();

	/* Let the Motion Control task run a few cycles first */
	(void)tk_dly_tsk(50);

	/* ---- Hardware set-up ---- */
	TEST_CHECK("S2-U01",
		   (drv_motor_pwm_top() == EXPECTED_PWM_TOP) &&
		   slice_ok(MOTOR_L_PIN_FWD) && slice_ok(MOTOR_R_PIN_FWD),
		   "PWM slices 4+5 at 20 kHz (TOP 6249, DIV 1), enabled (NFR9)");

	TEST_CHECK("S2-U02",
		   slice_braked(MOTOR_L_PIN_FWD) && slice_braked(MOTOR_R_PIN_FWD),
		   "both motors braked while motors are not enabled (FR11)");

	TEST_CHECK("S2-U03", drv_encoder_irq_enabled(),
		   "encoder edge interrupts enabled on GP4/5/16/17");

	/* ---- Pure logic ---- */
	TEST_CHECK("S2-U04",
		   (qdec_sum(fwd, 5) == 4) && (qdec_sum(rev, 5) == -4) &&
		   (drv_encoder_qdec(0x0, 0x3) == QDEC_ERROR) &&
		   (drv_encoder_qdec(0x1, 0x2) == QDEC_ERROR) &&
		   (drv_encoder_qdec(0x2, 0x2) == 0),
		   "quadrature table: +4 fwd cycle, -4 rev cycle, skips = error");

	b_ok = TRUE;
	ml_steer_mix(200, 0, &l, &r);
	b_ok = b_ok && (l == 200) && (r == 200);
	ml_steer_mix(200, 50, &l, &r);
	b_ok = b_ok && (l == 200) && (r == 100);
	ml_steer_mix(200, -100, &l, &r);
	b_ok = b_ok && (l == 0) && (r == 200);
	ml_steer_mix(200, 250, &l, &r);		/* clamped to +100 */
	b_ok = b_ok && (l == 200) && (r == 0);
	TEST_CHECK("S2-U05", b_ok,
		   "steering mix: 0 straight, +50 right slows, -100 pivots left, clamps");

	b_ok = TRUE;
	ml_pid_reset(&pid);
	b_ok = b_ok && (ml_pid_step(&pid, 0.0f, 0.0f, 0.01f) == 0);
	ml_pid_reset(&pid);
	b_ok = b_ok && (ml_pid_step(&pid, 200.0f, 0.0f, 0.01f) > 0);
	ml_pid_reset(&pid);
	b_ok = b_ok && (ml_pid_step(&pid, -200.0f, 0.0f, 0.01f) < 0);
	ml_pid_reset(&pid);
	b_ok = b_ok && (ml_pid_step(&pid, 100000.0f, 0.0f, 0.01f) == 100);
	TEST_CHECK("S2-U06", b_ok,
		   "PID: 0 err -> 0 %, sign follows error, output limited to 100 %");

	b_ok = TRUE;
	ml_stall_reset(&st);
	for (UINT i = 1u; i < (STALL_TIME_MS / MOTION_PERIOD_MS); i++) {
		b_ok = b_ok && !ml_stall_update(&st, 30, 0);	/* 290 ms: not yet */
	}
	b_ok = b_ok && ml_stall_update(&st, 30, 0);		/* 300 ms: stall   */
	ml_stall_reset(&st);
	for (UINT i = 0u; i < 100u; i++) {
		b_ok = b_ok && !ml_stall_update(&st, STALL_DUTY_MIN_PCT, 0);
	}
	b_ok = b_ok && !ml_stall_update(&st, 30, 1);		/* moving: reset   */
	TEST_CHECK("S2-U07", b_ok,
		   "stall: duty > 20 % and no pulse for 300 ms, not before (FR10)");

	/* ---- Interface ---- */
	TEST_CHECK("S2-U08",
		   (E_OK == s2_get_tlm(&tlm)) &&
		   (S2_MOTION_DISABLED == tlm.motion_state) &&
		   (0u == tlm.speed_l_mms) && (0u == tlm.speed_r_mms),
		   "IF-07 snapshot: E_OK, DISABLED, speed 0 at rest");

	/* ---- Configuration: FAILS until the values are measured ---- */
	TEST_CHECK("S2-U09", ml_config_ok(),
		   "motion_config.h measured: ENC_COUNTS_PER_REV, WHEEL_DIAMETER_MM, TRACK_WIDTH_MM");

	b_ok = TRUE;
	{
		ml_seg_t const fwd100 = { ML_SEG_STRAIGHT, 100 };
		ml_seg_t const back100 = { ML_SEG_STRAIGHT, -100 };
		ml_seg_t const spin_l = { ML_SEG_SPIN, 90 };

		b_ok = b_ok && !ml_seg_done(&fwd100, 50.0f, 50.0f);
		b_ok = b_ok && ml_seg_done(&fwd100, 99.0f, 101.0f);
		b_ok = b_ok && !ml_seg_done(&back100, -50.0f, -50.0f);
		b_ok = b_ok && ml_seg_done(&back100, -100.0f, -100.0f);
		ml_seg_targets(&spin_l, 100, &l, &r);
		b_ok = b_ok && (l == -100) && (r == 100);
		ml_seg_targets(&back100, 80, &l, &r);
		b_ok = b_ok && (l == -80) && (r == -80);
	}
	TEST_CHECK("S2-U10", b_ok,
		   "manoeuvre segments: distance done-check, spin left = L back / R fwd");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
