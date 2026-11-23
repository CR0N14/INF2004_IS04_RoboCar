/** @file navigation_main.c
 * @brief Exercise real S1 codecs and state logic with changing mappings.
 */
#include "navigation_main.h"
#include "../../../app_program/subsystem1/net_codec.h"
#include "../../../app_program/subsystem1/sm_logic.h"
#include <stdio.h>

#define TEST_MAPPING_COUNT 4u
#define TEST_BIND_BYTES 5u

int main (void)
{
    unsigned int failures = 0u;
    unsigned int checks   = 0u;

    for (uint8_t rotation = 0u; rotation < TEST_MAPPING_COUNT; ++rotation)
    {
        uint8_t   bytes[TEST_BIND_BYTES] = {0u};
        bitpack_t writer                 = {0};
        net_cmd_t command                = {0};
        bitpack_init(&writer, bytes, TEST_BIND_BYTES);
        bitpack_put(&writer, NET_OP_BIND, NET_OP_BITS);
        bitpack_put(&writer, TEST_MAPPING_COUNT, 5u);
        for (uint8_t entry = 0u; entry < TEST_MAPPING_COUNT; ++entry)
        {
            bitpack_put(&writer, 22u + entry, 5u);
            bitpack_put(&writer, (entry + rotation) % 4u, 2u);
        }
        ++checks;
        if (!net_codec_cmd(bytes, TEST_BIND_BYTES, &command))
        {
            ++failures;
        }
        for (uint8_t entry = 0u; entry < TEST_MAPPING_COUNT; ++entry)
        {
            uint8_t const symbol = 22u + entry;
            /* Two-bit MQTT action is representable in uint8_t. */
            uint8_t const action =
                (uint8_t)((command.actions >> (symbol * 2u)) & 3u);
            sm_ctx_t     context = {0};
            sm_actions_t effects = {0};
            context.state        = SM_STATE_NAVIGATING;
            context.nav_mode     = SM_NAV_TRACKING;
            sm_logic_step(&context, SM_EVT_BARCODE,
                          (uint16_t)((symbol * 4u) + action), &effects);
            ++checks;
            if (effects.b_nav_cmd || (!context.b_pending) ||
                (context.pending_symbol != symbol))
            {
                ++failures;
            }
            sm_logic_step(&context, SM_EVT_JUNCTION, 0u, &effects);
            ++checks;
            if ((!effects.b_nav_cmd) || context.b_pending ||
                (SM_NAV_CMD_ARG(effects.nav_cmd) != action))
            {
                ++failures;
            }
            sm_logic_step(&context, SM_EVT_TURN_DONE, 0u, &effects);
            sm_logic_step(&context, SM_EVT_JUNCTION, 0u, &effects);
            ++checks;
            if (effects.b_nav_cmd)
            {
                ++failures;
            }
        }
    }
    printf("MQTT mapping / stored symbol / junction checks: %u; failures: %u\n",
           checks, failures);
    return (0u == failures) ? 0 : 1;
}

/* End of file. */

