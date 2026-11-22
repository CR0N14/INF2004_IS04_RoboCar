/*
 *  test_host.h
 *  Minimal host test helpers for Subsystem 4, same line format as the
 *  on-target tests (app_program/test_report.h) so logs grep alike:
 *
 *      [HOST] S4-H07 PASS  big hump, L 8.0, 20 cm/s: 23 mm (true 23.0)
 */

#ifndef TEST_HOST_H
#define TEST_HOST_H

#include <stdio.h>

extern int g_pass;
extern int g_fail;

#define CHECK(id, cond, ...)                                        \
    do {                                                            \
        if (cond) {                                                 \
            g_pass++;                                               \
            printf("[HOST] %s PASS  ", (id));                       \
        } else {                                                    \
            g_fail++;                                               \
            printf("[HOST] %s FAIL  ", (id));                       \
        }                                                           \
        printf(__VA_ARGS__);                                        \
        if (!(cond)) {                                              \
            printf("   (%s:%d)", __FILE__, __LINE__);               \
        }                                                           \
        printf("\n");                                               \
    } while (0)

#define INFO(...)                                                   \
    do {                                                            \
        printf("[INFO] ");                                          \
        printf(__VA_ARGS__);                                        \
        printf("\n");                                               \
    } while (0)

void    test_fixmath(void);
void    test_logic(void);

#endif /* TEST_HOST_H */
