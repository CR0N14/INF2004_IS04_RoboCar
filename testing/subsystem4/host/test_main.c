/*
 *  test_main.c
 *  Subsystem 4 host unit tests: imu_fixmath and imu_logic, driven by the
 *  car/hump model.  Build and run with "make" in this folder.
 */

#include "test_host.h"

int g_pass = 0;
int g_fail = 0;

int
main(void)
{
    printf("[HOST] === subsystem 4 host tests ===\n");
    test_fixmath();
    test_logic();
    printf("[HOST] subsystem4 SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
