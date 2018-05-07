/** \file
    \brief GF(256) Main C API Source
    \copyright Copyright (c) 2017 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of GF256 nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#include "gf256.h"

#ifdef LINUX_ARM
#include <unistd.h>
#include <fcntl.h>
#include <elf.h>
#include <linux/auxvec.h>
#endif

//------------------------------------------------------------------------------
// Self-Test
//
// This is executed during initialization to make sure the library is working

static const unsigned kTestBufferBytes = 32 + 16 + 8 + 4 + 2 + 1;
static const unsigned kTestBufferAllocated = 64;
struct SelfTestBuffersT
{
    GF256_ALIGNED uint8_t A[kTestBufferAllocated];
    GF256_ALIGNED uint8_t B[kTestBufferAllocated];
    GF256_ALIGNED uint8_t C[kTestBufferAllocated];
};
static GF256_ALIGNED SelfTestBuffersT m_SelfTestBuffers;

static bool gf256_self_test()
{
    if ((uintptr_t)m_SelfTestBuffers.A % GF256_ALIGN_BYTES != 0)
        return false;
    if ((uintptr_t)m_SelfTestBuffers.A % GF256_ALIGN_BYTES != 0)
        return false;
    if ((uintptr_t)m_SelfTestBuffers.B % GF256_ALIGN_BYTES != 0)
        return false;
    if ((uintptr_t)m_SelfTestBuffers.C % GF256_ALIGN_BYTES != 0)
        return false;

    // Check multiplication/division
    for (unsigned i = 0; i < 256; ++i)
    {
        for (unsigned j = 0; j < 256; ++j)
        {
            uint8_t prod = gf256_mul((uint8_t)i, (uint8_t)j);
            if (i != 0 && j != 0)
            {
                uint8_t div1 = gf256_div(prod, (uint8_t)i);
                if (div1 != j)
                    return false;
                uint8_t div2 = gf256_div(prod, (uint8_t)j);
                if (div2 != i)
                    return false;
            }
            else if (prod != 0)
                return false;
            if (j == 1 && prod != i)
                return false;
        }
    }

    // Check for overruns
    m_SelfTestBuffers.A[kTestBufferBytes] = 0x5a;
    m_SelfTestBuffers.B[kTestBufferBytes] = 0x5a;
    m_SelfTestBuffers.C[kTestBufferBytes] = 0x5a;

    // Test gf256_add_mem()
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
    {
        m_SelfTestBuffers.A[i] = 0x1f;
        m_SelfTestBuffers.B[i] = 0xf7;
    }
    gf256_add_mem(m_SelfTestBuffers.A, m_SelfTestBuffers.B, kTestBufferBytes);
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
        if (m_SelfTestBuffers.A[i] != (0x1f ^ 0xf7))
            return false;

    // Test gf256_add2_mem()
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
    {
        m_SelfTestBuffers.A[i] = 0x1f;
        m_SelfTestBuffers.B[i] = 0xf7;
        m_SelfTestBuffers.C[i] = 0x71;
    }
    gf256_add2_mem(m_SelfTestBuffers.A, m_SelfTestBuffers.B, m_SelfTestBuffers.C, kTestBufferBytes);
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
        if (m_SelfTestBuffers.A[i] != (0x1f ^ 0xf7 ^ 0x71))
            return false;

    // Test gf256_addset_mem()
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
    {
        m_SelfTestBuffers.A[i] = 0x55;
        m_SelfTestBuffers.B[i] = 0xaa;
        m_SelfTestBuffers.C[i] = 0x6c;
    }
    gf256_addset_mem(m_SelfTestBuffers.A, m_SelfTestBuffers.B, m_SelfTestBuffers.C, kTestBufferBytes);
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
        if (m_SelfTestBuffers.A[i] != (0xaa ^ 0x6c))
            return false;

    // Test gf256_muladd_mem()
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
    {
        m_SelfTestBuffers.A[i] = 0xff;
        m_SelfTestBuffers.B[i] = 0xaa;
    }
    const uint8_t expectedMulAdd = gf256_mul(0xaa, 0x6c);
    gf256_muladd_mem(m_SelfTestBuffers.A, 0x6c, m_SelfTestBuffers.B, kTestBufferBytes);
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
        if (m_SelfTestBuffers.A[i] != (expectedMulAdd ^ 0xff))
            return false;

    // Test gf256_mul_mem()
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
    {
        m_SelfTestBuffers.A[i] = 0xff;
        m_SelfTestBuffers.B[i] = 0x55;
    }
    const uint8_t expectedMul = gf256_mul(0xa2, 0x55);
    gf256_mul_mem(m_SelfTestBuffers.A, m_SelfTestBuffers.B, 0xa2, kTestBufferBytes);
    for (unsigned i = 0; i < kTestBufferBytes; ++i)
        if (m_SelfTestBuffers.A[i] != expectedMul)
            return false;

    if (m_SelfTestBuffers.A[kTestBufferBytes] != 0x5a)
        return false;
    if (m_SelfTestBuffers.B[kTestBufferBytes] != 0x5a)
        return false;
    if (m_SelfTestBuffers.C[kTestBufferBytes] != 0x5a)
        return false;

    return true;
}

//------------------------------------------------------------------------------
// Context Object

// Context object for GF(2^^8) math
GF256_ALIGNED gf256_ctx GF256Ctx;
static bool Initialized = false;


//------------------------------------------------------------------------------
// Generator Polynomial

// There are only 16 irreducible polynomials for GF(2^^8)
static const int GF256_GEN_POLY_COUNT = 16;
static const uint8_t GF256_GEN_POLY[GF256_GEN_POLY_COUNT] = {
    0x8e, 0x95, 0x96, 0xa6, 0xaf, 0xb1, 0xb2, 0xb4,
    0xb8, 0xc3, 0xc6, 0xd4, 0xe1, 0xe7, 0xf3, 0xfa
};

static const int kDefaultPolynomialIndex = 3;

// Select which polynomial to use
static void gf256_poly_init(int polynomialIndex)
{
    if (polynomialIndex < 0 || polynomialIndex >= GF256_GEN_POLY_COUNT)
        polynomialIndex = kDefaultPolynomialIndex;

    GF256Ctx.Polynomial = (GF256_GEN_POLY[polynomialIndex] << 1) | 1;
}


//------------------------------------------------------------------------------
// Exponential and Log Tables

// Construct EXP and LOG tables from polynomial
static void gf256_explog_init()
{
    unsigned poly = GF256Ctx.Polynomial;
    uint8_t* exptab = GF256Ctx.GF256_EXP_TABLE;
    uint16_t* logtab = GF256Ctx.GF256_LOG_TABLE;

    logtab[0] = 512;
    exptab[0] = 1;
    for (unsigned jj = 1; jj < 255; ++jj)
    {
        unsigned next = (unsigned)exptab[jj - 1] * 2;
        if (next >= 256)
            next ^= poly;

        exptab[jj] = static_cast<uint8_t>( next );
        logtab[exptab[jj]] = static_cast<uint16_t>( jj );
    }
    exptab[255] = exptab[0];
    logtab[exptab[255]] = 255;
    for (unsigned jj = 256; jj < 2 * 255; ++jj)
        exptab[jj] = exptab[jj % 255];
    exptab[2 * 255] = 1;
    for (unsigned jj = 2 * 255 + 1; jj < 4 * 255; ++jj)
        exptab[jj] = 0;
}


//------------------------------------------------------------------------------
// Multiply and Divide Tables

// Initialize MUL and DIV tables using LOG and EXP tables
static void gf256_muldiv_init()
{
    // Allocate table memory 65KB x 2
    uint8_t* m = GF256Ctx.GF256_MUL_TABLE;
    uint8_t* d = GF256Ctx.GF256_DIV_TABLE;

    // Unroll y = 0 subtable
    for (int x = 0; x < 256; ++x)
        m[x] = d[x] = 0;

    // For each other y value:
    for (int y = 1; y < 256; ++y)
    {
        // Calculate log(y) for mult and 255 - log(y) for div
        const uint8_t log_y = static_cast<uint8_t>(GF256Ctx.GF256_LOG_TABLE[y]);
        const uint8_t log_yn = 255 - log_y;

        // Next subtable
        m += 256, d += 256;

        // Unroll x = 0
        m[0] = 0, d[0] = 0;

        // Calculate x * y, x / y
        for (int x = 1; x < 256; ++x)
        {
            uint16_t log_x = GF256Ctx.GF256_LOG_TABLE[x];

            m[x] = GF256Ctx.GF256_EXP_TABLE[log_x + log_y];
            d[x] = GF256Ctx.GF256_EXP_TABLE[log_x + log_yn];
        }
    }
}


//------------------------------------------------------------------------------
// Inverse Table

// Initialize INV table using DIV table
static void gf256_inv_init()
{
    for (int x = 0; x < 256; ++x)
        GF256Ctx.GF256_INV_TABLE[x] = gf256_div(1, static_cast<uint8_t>(x));
}


//------------------------------------------------------------------------------
// Square Table

// Initialize SQR table using MUL table
static void gf256_sqr_init()
{
    for (int x = 0; x < 256; ++x)
        GF256Ctx.GF256_SQR_TABLE[x] = gf256_mul(static_cast<uint8_t>(x), static_cast<uint8_t>(x));
}


//------------------------------------------------------------------------------
// Multiply and Add Memory Tables

/*
    Fast algorithm to compute m[1..8] = a[1..8] * b in GF(256)
    using SSE3 SIMD instruction set:

    Consider z = x * y in GF(256).
    This operation can be performed bit-by-bit.  Usefully, the partial product
    of each bit is combined linearly with the rest.  This means that the 8-bit
    number x can be split into its high and low 4 bits, and partial products
    can be formed from each half.  Then the halves can be linearly combined:

        z = x[0..3] * y + x[4..7] * y

    The multiplication of each half can be done efficiently via table lookups,
    and the addition in GF(256) is XOR.  There must be two tables that map 16
    input elements for the low or high 4 bits of x to the two partial products.
    Each value for y has a different set of two tables:

        z = TABLE_LO_y(x[0..3]) xor TABLE_HI_y(x[4..7])

    This means that we need 16 * 2 * 256 = 8192 bytes for precomputed tables.

    Computing z[] = x[] * y can be performed 16 bytes at a time by using the
    128-bit register operations supported by modern processors.

    This is efficiently realized in SSE3 using the _mm_shuffle_epi8() function
    provided by Visual Studio 2010 or newer in <tmmintrin.h>.  This function
    uses the low bits to do a table lookup on each byte.  Unfortunately the
    high bit of each mask byte has the special feature that it clears the
    output byte when it is set, so we need to make sure it's cleared by masking
    off the high bit of each byte before using it:

        clr_mask = _mm_set1_epi8(0x0f) = 0x0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f

    For the low half of the partial product, clear the high bit of each byte
    and perform the table lookup:

        p_lo = _mm_and_si128(x, clr_mask)
        p_lo = _mm_shuffle_epi8(p_lo, TABLE_LO_y)

    For the high half of the partial product, shift the high 4 bits of each
    byte into the low 4 bits and clear the high bit of each byte, and then
    perform the table lookup:

        p_hi = _mm_srli_epi64(x, 4)
        p_hi = _mm_and_si128(p_hi, clr_mask)
        p_hi = _mm_shuffle_epi8(p_hi, TABLE_HI_y)

    Finally add the two partial products to form the product, recalling that
    addition is XOR in a Galois field:

        result = _mm_xor_si128(p_lo, p_hi)

    This crunches 16 bytes of x at a time, and the result can be stored in z.
*/

/*
    Intrinsic reference:

    SSE3, VS2010+, tmmintrin.h:

    GF256_M128 _mm_shuffle_epi8(GF256_M128 a, GF256_M128 mask);
        Emits the Supplemental Streaming SIMD Extensions 3 (SSSE3) instruction pshufb. This instruction shuffles 16-byte parameters from a 128-bit parameter.

        Pseudo-code for PSHUFB (with 128 bit operands):

            for i = 0 to 15 {
                 if (SRC[(i * 8)+7] = 1 ) then
                      DEST[(i*8)+7..(i*8)+0] <- 0;
                  else
                      index[3..0] <- SRC[(i*8)+3 .. (i*8)+0];
                      DEST[(i*8)+7..(i*8)+0] <- DEST[(index*8+7)..(index*8+0)];
                 endif
            }

    SSE2, VS2008+, emmintrin.h:

    GF256_M128 _mm_slli_epi64 (GF256_M128 a, int count);
        Shifts the 2 signed or unsigned 64-bit integers in a left by count bits while shifting in zeros.
    GF256_M128 _mm_srli_epi64 (GF256_M128 a, int count);
        Shifts the 2 signed or unsigned 64-bit integers in a right by count bits while shifting in zeros.
    GF256_M128 _mm_set1_epi8 (char b);
        Sets the 16 signed 8-bit integer values to b.
    GF256_M128 _mm_and_si128 (GF256_M128 a, GF256_M128 b);
        Computes the bitwise AND of the 128-bit value in a and the 128-bit value in b.
    GF256_M128 _mm_xor_si128 ( GF256_M128 a, GF256_M128 b);
        Computes the bitwise XOR of the 128-bit value in a and the 128-bit value in b.
*/

// Initialize the multiplication tables using gf256_mul()
static void gf256_mul_mem_init()
{
    // Reuse aligned self test buffers to load table data
    uint8_t* lo = m_SelfTestBuffers.A;
    uint8_t* hi = m_SelfTestBuffers.B;

    for (int y = 0; y < 256; ++y)
    {
        // TABLE_LO_Y maps 0..15 to 8-bit partial product based on y.
        for (unsigned char x = 0; x < 16; ++x)
        {
            lo[x] = gf256_mul(x, static_cast<uint8_t>( y ));
            hi[x] = gf256_mul(x << 4, static_cast<uint8_t>( y ));
        }
    }
}


//------------------------------------------------------------------------------
// Initialization

static unsigned char kLittleEndianTestData[4] = { 4, 3, 2, 1 };

union UnionType
{
    uint32_t IntValue;
    char CharArray[4];
};

static bool IsLittleEndian()
{
    UnionType type;
    for (unsigned i = 0; i < 4; ++i)
        type.CharArray[i] = kLittleEndianTestData[i];
    return 0x01020304 == type.IntValue;
}

extern "C" int gf256_init_(int version)
{
    if (version != GF256_VERSION)
        return -1; // User's header does not match library version.

    // Avoid multiple initialization
    if (Initialized)
        return 0;
    Initialized = true;

    if (!IsLittleEndian())
        return -2; // Architecture is not supported (code won't work without mods).

    gf256_poly_init(kDefaultPolynomialIndex);
    gf256_explog_init();
    gf256_muldiv_init();
    gf256_inv_init();
    gf256_sqr_init();
    gf256_mul_mem_init();

    if (!gf256_self_test())
        return -3; // Self-test failed (perhaps untested configuration)

    return 0;
}


//------------------------------------------------------------------------------
// Operations

extern "C" void gf256_add_mem(void * GF256_RESTRICT vx,
                              const void * GF256_RESTRICT vy, int bytes)
{
    uint8_t * GF256_RESTRICT x1 = reinterpret_cast<uint8_t *>(vx);
    const uint8_t * GF256_RESTRICT y1 = reinterpret_cast<const uint8_t *>(vy);

    for (int i = 0; i < bytes; i++) {
        x1[i] ^= y1[i];
    }
}

extern "C" void gf256_add2_mem(void * GF256_RESTRICT vz, const void * GF256_RESTRICT vx,
                               const void * GF256_RESTRICT vy, int bytes)
{
    uint8_t * GF256_RESTRICT z1 = reinterpret_cast<uint8_t *>(vz);
    const uint8_t * GF256_RESTRICT x1 = reinterpret_cast<const uint8_t *>(vx);
    const uint8_t * GF256_RESTRICT y1 = reinterpret_cast<const uint8_t *>(vy);

    for (int i = 0; i < bytes; i++) {
        z1[i] ^= x1[i] ^ y1[i];
    }
}

extern "C" void gf256_addset_mem(void * GF256_RESTRICT vz, const void * GF256_RESTRICT vx,
                                 const void * GF256_RESTRICT vy, int bytes)
{
    uint8_t * GF256_RESTRICT z1 = reinterpret_cast<uint8_t *>(vz);
    const uint8_t * GF256_RESTRICT x1 = reinterpret_cast<const uint8_t *>(vx);
    const uint8_t * GF256_RESTRICT y1 = reinterpret_cast<const uint8_t *>(vy);

    for (int i = 0; i < bytes; i++) {
        z1[i] = x1[i] ^ y1[i];
    }
}

extern "C" void gf256_mul_mem(void * GF256_RESTRICT vz, const void * GF256_RESTRICT vx, uint8_t y, int bytes)
{
    // Use a single if-statement to handle special cases
    if (y <= 1)
    {
        if (y == 0)
            memset(vz, 0, bytes);
        else if (vz != vx)
            memcpy(vz, vx, bytes);
        return;
    }

    uint8_t * GF256_RESTRICT z1 = reinterpret_cast<uint8_t*>(vz);
    const uint8_t * GF256_RESTRICT x1 = reinterpret_cast<const uint8_t*>(vx);
    const uint8_t * GF256_RESTRICT table = GF256Ctx.GF256_MUL_TABLE + ((unsigned)y << 8);

    // Handle blocks of 8 bytes
    while (bytes >= 8)
    {
        uint64_t * GF256_RESTRICT z8 = reinterpret_cast<uint64_t *>(z1);
        uint64_t word = table[x1[0]];
        word |= (uint64_t)table[x1[1]] << 8;
        word |= (uint64_t)table[x1[2]] << 16;
        word |= (uint64_t)table[x1[3]] << 24;
        word |= (uint64_t)table[x1[4]] << 32;
        word |= (uint64_t)table[x1[5]] << 40;
        word |= (uint64_t)table[x1[6]] << 48;
        word |= (uint64_t)table[x1[7]] << 56;
        *z8 = word;

        bytes -= 8, x1 += 8, z1 += 8;
    }

    // Handle a block of 4 bytes
    const int four = bytes & 4;
    if (four)
    {
        uint32_t * GF256_RESTRICT z4 = reinterpret_cast<uint32_t *>(z1);
        uint32_t word = table[x1[0]];
        word |= (uint32_t)table[x1[1]] << 8;
        word |= (uint32_t)table[x1[2]] << 16;
        word |= (uint32_t)table[x1[3]] << 24;
        *z4 = word;
    }

    // Handle single bytes
    const int offset = four;
    switch (bytes & 3)
    {
    case 3: z1[offset + 2] = table[x1[offset + 2]];
    case 2: z1[offset + 1] = table[x1[offset + 1]];
    case 1: z1[offset] = table[x1[offset]];
    default:
        break;
    }
}

extern "C" void gf256_muladd_mem(void * GF256_RESTRICT vz, uint8_t y,
                                 const void * GF256_RESTRICT vx, int bytes)
{
    // Use a single if-statement to handle special cases
    if (y <= 1)
    {
        if (y == 1)
            gf256_add_mem(vz, vx, bytes);
        return;
    }

    uint8_t * GF256_RESTRICT z1 = reinterpret_cast<uint8_t*>(vz);
    const uint8_t * GF256_RESTRICT x1 = reinterpret_cast<const uint8_t*>(vx);
    const uint8_t * GF256_RESTRICT table = GF256Ctx.GF256_MUL_TABLE + ((unsigned)y << 8);

    // Handle blocks of 8 bytes
    while (bytes >= 8)
    {
        uint64_t * GF256_RESTRICT z8 = reinterpret_cast<uint64_t *>(z1);
        uint64_t word = table[x1[0]];
        word |= (uint64_t)table[x1[1]] << 8;
        word |= (uint64_t)table[x1[2]] << 16;
        word |= (uint64_t)table[x1[3]] << 24;
        word |= (uint64_t)table[x1[4]] << 32;
        word |= (uint64_t)table[x1[5]] << 40;
        word |= (uint64_t)table[x1[6]] << 48;
        word |= (uint64_t)table[x1[7]] << 56;
        *z8 ^= word;

        bytes -= 8, x1 += 8, z1 += 8;
    }

    // Handle a block of 4 bytes
    const int four = bytes & 4;
    if (four)
    {
        uint32_t * GF256_RESTRICT z4 = reinterpret_cast<uint32_t *>(z1);
        uint32_t word = table[x1[0]];
        word |= (uint32_t)table[x1[1]] << 8;
        word |= (uint32_t)table[x1[2]] << 16;
        word |= (uint32_t)table[x1[3]] << 24;
        *z4 ^= word;
    }

    // Handle single bytes
    const int offset = four;
    switch (bytes & 3)
    {
    case 3: z1[offset + 2] ^= table[x1[offset + 2]];
    case 2: z1[offset + 1] ^= table[x1[offset + 1]];
    case 1: z1[offset] ^= table[x1[offset]];
    default:
        break;
    }
}

extern "C" void gf256_memswap(void * GF256_RESTRICT vx, void * GF256_RESTRICT vy, int bytes)
{
    uint8_t * GF256_RESTRICT x1 = reinterpret_cast<uint8_t *>(vx);
    uint8_t * GF256_RESTRICT y1 = reinterpret_cast<uint8_t *>(vy);

    for (int i = 0; i < bytes; i++)
    {
        const uint64_t temp = x1[i];
        x1[i] = y1[i];
        y1[i] = temp;
    }
}
