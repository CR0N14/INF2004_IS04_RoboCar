/*
 *  bitpack.h
 *  Bit-level writer and reader for compact binary messages.
 *
 *  Fields are written most-significant bit first, one after another
 *  with no padding between them; the last byte is padded with zero
 *  bits.  This gives every field exactly the number of bits its range
 *  needs, independent of compiler bit-field layout (Change
 *  Requirement 1).
 *
 *  Private to subsystem 1.  No kernel dependency.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>
#include <stdbool.h>

#define BITPACK_MAX_FIELD_BITS  32u

typedef struct
{
    uint8_t     *p_buf;
    uint16_t    cap_bits;
    uint16_t    pos;            /* next bit to write                     */
    bool        b_overflow;     /* a field did not fit                   */
} bitpack_t;

typedef struct
{
    uint8_t const   *p_buf;
    uint16_t        len_bits;
    uint16_t        pos;        /* next bit to read                      */
    bool            b_underflow;    /* a field ran past the end          */
} bitread_t;

/* Start writing into p_buf (cleared to zero) of cap_bytes bytes */
void        bitpack_init(bitpack_t *p_bp, uint8_t *p_buf, uint16_t cap_bytes);

/* Append the low 'bits' bits of value (1..32).  Sets b_overflow and
   writes nothing if it does not fit. */
void        bitpack_put(bitpack_t *p_bp, uint32_t value, uint8_t bits);

/* Bits still free */
uint16_t    bitpack_free_bits(bitpack_t const *p_bp);

/* Bytes used so far (last byte zero-padded) */
uint16_t    bitpack_bytes(bitpack_t const *p_bp);

/* Start reading len_bytes bytes */
void        bitread_init(bitread_t *p_br, uint8_t const *p_buf,
                         uint16_t len_bytes);

/* Read the next 'bits' bits (1..32).  Sets b_underflow and returns 0
   if they run past the end. */
uint32_t    bitread_get(bitread_t *p_br, uint8_t bits);

/* Bits not read yet */
uint16_t    bitread_left(bitread_t const *p_br);

#endif /* BITPACK_H */
