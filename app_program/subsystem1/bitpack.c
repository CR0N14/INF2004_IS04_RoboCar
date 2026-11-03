/*
 *  bitpack.c
 *  Bit-level writer and reader.  See bitpack.h.
 */

#include "bitpack.h"

#define BITS_PER_BYTE   8u
#define BYTE_MSB        0x80u

void
bitpack_init(bitpack_t *p_bp, uint8_t *p_buf, uint16_t cap_bytes)
{
    uint16_t i;

    for (i = 0u; i < cap_bytes; i++)
    {
        p_buf[i] = 0u;
    }
    p_bp->p_buf = p_buf;
    p_bp->cap_bits = (uint16_t)(cap_bytes * BITS_PER_BYTE);
    p_bp->pos = 0u;
    p_bp->b_overflow = false;
}

void
bitpack_put(bitpack_t *p_bp, uint32_t value, uint8_t bits)
{
    uint8_t i;

    if ((0u == bits) || (bits > BITPACK_MAX_FIELD_BITS) ||
        (bits > bitpack_free_bits(p_bp)))
    {
        p_bp->b_overflow = true;
        return;
    }

    for (i = bits; i > 0u; i--)
    {
        if (0u != ((value >> (i - 1u)) & 1u))
        {
            p_bp->p_buf[p_bp->pos / BITS_PER_BYTE] |=
                (uint8_t)(BYTE_MSB >> (p_bp->pos % BITS_PER_BYTE));
        }
        p_bp->pos++;
    }
}

uint16_t
bitpack_free_bits(bitpack_t const *p_bp)
{
    return (uint16_t)(p_bp->cap_bits - p_bp->pos);
}

uint16_t
bitpack_bytes(bitpack_t const *p_bp)
{
    return (uint16_t)((p_bp->pos + (BITS_PER_BYTE - 1u)) / BITS_PER_BYTE);
}

void
bitread_init(bitread_t *p_br, uint8_t const *p_buf, uint16_t len_bytes)
{
    p_br->p_buf = p_buf;
    p_br->len_bits = (uint16_t)(len_bytes * BITS_PER_BYTE);
    p_br->pos = 0u;
    p_br->b_underflow = false;
}

uint32_t
bitread_get(bitread_t *p_br, uint8_t bits)
{
    uint32_t value = 0u;
    uint8_t  i;

    if ((0u == bits) || (bits > BITPACK_MAX_FIELD_BITS) ||
        (bits > bitread_left(p_br)))
    {
        p_br->b_underflow = true;
        return 0u;
    }

    for (i = 0u; i < bits; i++)
    {
        uint8_t const byte = p_br->p_buf[p_br->pos / BITS_PER_BYTE];

        value = (value << 1u) |
                (uint32_t)((byte >> ((BITS_PER_BYTE - 1u) -
                             (p_br->pos % BITS_PER_BYTE))) & 1u);
        p_br->pos++;
    }
    return value;
}

uint16_t
bitread_left(bitread_t const *p_br)
{
    return (uint16_t)(p_br->len_bits - p_br->pos);
}
