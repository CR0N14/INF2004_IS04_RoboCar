/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record results in testing/subsystem3/TEST_LOG.md.  IDs: S3-U01, ...
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	TEST_INIT();

	/* Replace with real checks of this subsystem's behaviour */
	TEST_CHECK("S3-U01", tk_get_tid() > 0, "example: running in a task context");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
