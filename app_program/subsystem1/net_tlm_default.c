/*
 *  net_tlm_default.c
 *  Placeholder telemetry snapshot getters (IF-07).
 *
 *  Weak definitions: each subsystem replaces its own by defining
 *  sN_get_tlm() in its folder, and the linker then uses that one.
 *  Until then Networking gets E_NOSPT and leaves that subsystem's
 *  values out of the telemetry frame.
 */

#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define NET_WEAK    __attribute__((weak))

NET_WEAK ER
s2_get_tlm(s2_tlm_t *p_tlm)
{
    (void)p_tlm;
    return E_NOSPT;
}

NET_WEAK ER
s3_get_tlm(s3_tlm_t *p_tlm)
{
    (void)p_tlm;
    return E_NOSPT;
}

NET_WEAK ER
s4_get_tlm(s4_tlm_t *p_tlm)
{
    (void)p_tlm;
    return E_NOSPT;
}

NET_WEAK ER
s5_get_tlm(s5_tlm_t *p_tlm)
{
    (void)p_tlm;
    return E_NOSPT;
}
