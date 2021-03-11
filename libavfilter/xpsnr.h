/*
File: xpsnr.h - public declarations for XPSNR measurement filter plug-in for FFmpeg
Authors: Christian Helmrich and Christian Stoffers, Fraunhofer HHI, Berlin, Germany

License:

The copyright in this software is being made available under this Software Copyright
License. This software may be subject to other third-party and contributor rights,
including patent rights, and no such rights are granted under this license.

Copyright (c) 2019 - 2021 Fraunhofer-Gesellschaft zur Förderung der angewandten
Forschung e.V. (Fraunhofer). All rights reserved.

Redistribution and use of this software in source and binary forms, with or without
modification, are permitted for non-commercial purposes of evaluation, testing, and
academic research (internal use) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list
  of conditions, and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions, and the following disclaimer in the documentation and/or other
  materials provided with the distribution.
* Neither the names of the copyright holders nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. NO EXPRESS OR IMPLIED LICENSES TO ANY PATENT CLAIMS, INCLUDING WITHOUT
LIMITATION THE PATENTS OF THE COPYRIGHT HOLDERS AND CONTRIBUTORS, ARE GRANTED BY
THIS SOFTWARE LICENSE. THE COPYRIGHT HOLDERS AND CONTRIBUTORS PROVIDE NO WARRANTY OF
PATENT NON-INFRINGEMENT WITH RESPECT TO THIS SOFTWARE.
*/

#ifndef AVFILTER_XPSNR_H
#define AVFILTER_XPSNR_H

#include <stddef.h>
#include <stdint.h>
#include "libavutil/x86/cpu.h"

/* public XPSNR DSP structure definition */

typedef struct XPSNRDSPContext
{
  uint64_t (*sse_line) (const uint8_t *buf, const uint8_t *ref, int w);
  uint64_t (*highds_func) (const int xAct, const int yAct, const int wAct, const int hAct, const int16_t *o, const int O);
  uint64_t (*diff2nd_func) (const uint32_t wAct, const uint32_t hAct, const int16_t *o, int16_t *oM1, int16_t *oM2, const int O);
}
PSNRDSPContext;

void ff_psnr_init_x86 (PSNRDSPContext *dsp, int bpp);

/* SIMD functions included here */
#include <x86intrin.h>

#define _mm_storeu_si16(p, a) (void)(*(short*)(p) = (short)_mm_cvtsi128_si32((a)))

#define MULADD(p1, p2, scale, tmp1, tmp2, sum) \
        tmp1 = _mm_madd_epi16 (p1, scale); \
        tmp2 = _mm_madd_epi16 (p2, scale); \
        tmp1 = _mm_hadd_epi32 (tmp1, tmp2); \
        tmp1 = _mm_hadd_epi32 (tmp1, tmp1); \
        tmp1 = _mm_hadd_epi32 (tmp1, tmp1); \
        sum += _mm_extract_epi32 (tmp1, 0);

/* XPSNR function definitions */
static uint64_t highds (const int xAct, const int yAct, const int wAct, const int hAct, const int16_t *o, const int O);
static uint64_t diff2nd (const uint32_t wAct, const uint32_t hAct, const int16_t *o, int16_t *oM1, int16_t *oM2, const int O);

#ifdef __AVX2__
static uint64_t highds_SIMD (const int xAct, const int yAct, const int wAct, const int hAct, const int16_t *o, const int O);
static uint64_t diff2nd_SIMD (const uint32_t wAct, const uint32_t hAct, const int16_t *o, int16_t *oM1, int16_t *oM2, const int O);

static uint64_t highds_SIMD (const int xAct, const int yAct, const int wAct, const int hAct, const int16_t *o, const int O)
{
  uint64_t saAct = 0;

  if (wAct > 12)
  {
    const __m128i scale1 = _mm_set_epi16 (0, 0,-1,-2,-3,-3,-2,-1);
    const __m128i scale2 = _mm_set_epi16 (0, 0,-1,-3,12,12,-3,-1);
    const __m128i scale3 = _mm_set_epi16 (0, 0, 0,-1,-1,-1,-1, 0);
    __m128i tmp1, tmp2;
    __m128i l0, lP1, lM1, lP2, lM2, lP3;
    int sum;

    for (int y = yAct; y < hAct; y += 2)
    {
      for (int x = xAct; x < wAct; x += 12)
      {
        __m256i lineM2 = _mm256_lddqu_si256 ((__m256i*) &o[(y-2)*O + x-2]);
        __m256i lineM1 = _mm256_lddqu_si256 ((__m256i*) &o[(y-1)*O + x-2]);
        __m256i line0  = _mm256_lddqu_si256 ((__m256i*) &o[ y   *O + x-2]);
        __m256i lineP1 = _mm256_lddqu_si256 ((__m256i*) &o[(y+1)*O + x-2]);
        __m256i lineP2 = _mm256_lddqu_si256 ((__m256i*) &o[(y+2)*O + x-2]);
        __m256i lineP3 = _mm256_lddqu_si256 ((__m256i*) &o[(y+3)*O + x-2]);

        for (int xx = 0; xx < 3; xx++)
        {
          if ((xx << 2) + x < wAct)
          {
            sum = 0;
            l0  = _mm256_castsi256_si128 (line0 );
            lP1 = _mm256_castsi256_si128 (lineP1);
            MULADD (l0 , lP1, scale2, tmp1, tmp2, sum)
            lM1 = _mm256_castsi256_si128 (lineM1);
            lP2 = _mm256_castsi256_si128 (lineP2);
            MULADD (lM1, lP2, scale1, tmp1, tmp2, sum)
            lM2 = _mm256_castsi256_si128 (lineM2);
            lP3 = _mm256_castsi256_si128 (lineP3);
            MULADD (lM2, lP3, scale3, tmp1, tmp2, sum)
            saAct += (uint64_t) abs (sum);
          }
          if ((xx << 2) + x + 2 < wAct)
          {
            sum = 0;
            l0  = _mm_bsrli_si128 (l0 , 4);
            lP1 = _mm_bsrli_si128 (lP1, 4);
            MULADD (l0 , lP1, scale2, tmp1, tmp2, sum)
            lM1 = _mm_bsrli_si128 (lM1, 4);
            lP2 = _mm_bsrli_si128 (lP2, 4);
            MULADD (lM1, lP2, scale1, tmp1, tmp2, sum)
            lM2 = _mm_bsrli_si128 (lM2, 4);
            lP3 = _mm_bsrli_si128 (lP3, 4);
            MULADD (lM2, lP3, scale3, tmp1, tmp2, sum)
            saAct += (uint64_t) abs (sum);

            /* 4 byte to the right */
            lineM2 = _mm256_permute4x64_epi64 (lineM2, 0x39);
            lineM1 = _mm256_permute4x64_epi64 (lineM1, 0x39);
            line0  = _mm256_permute4x64_epi64 (line0 , 0x39);
            lineP1 = _mm256_permute4x64_epi64 (lineP1, 0x39);
            lineP2 = _mm256_permute4x64_epi64 (lineP2, 0x39);
            lineP3 = _mm256_permute4x64_epi64 (lineP3, 0x39);
          }
        }
      }
    }
  }
  else
  {
    saAct = highds (xAct, yAct, wAct, hAct, o, O);
  }
  return saAct;
}

static uint64_t diff2nd_SIMD (const uint32_t wAct, const uint32_t hAct, const int16_t *o, int16_t *oM1, int16_t *oM2, const int O)
{
  uint64_t taAct = 0;
  uint16_t act = 0;

  for (uint32_t y = 0; y < hAct; y += 2)
  {
    for (uint32_t x = 0; x < wAct; x += 8)
    {
      __m128i lineM0u = _mm_lddqu_si128 ((__m128i*) &o  [ y   *O + x]); /* load 8 16-bit values */
      __m128i lineM0d = _mm_lddqu_si128 ((__m128i*) &o  [(y+1)*O + x]);
      __m128i lineM1u = _mm_lddqu_si128 ((__m128i*) &oM1[ y   *O + x]);
      __m128i lineM1d = _mm_lddqu_si128 ((__m128i*) &oM1[(y+1)*O + x]);
      __m128i lineM2u = _mm_lddqu_si128 ((__m128i*) &oM2[ y   *O + x]);
      __m128i lineM2d = _mm_lddqu_si128 ((__m128i*) &oM2[(y+1)*O + x]);

      __m128i M0 = _mm_add_epi16 (lineM0u, lineM0d);
      __m128i M1 = _mm_add_epi16 (lineM1u, lineM1d);
      __m128i M2 = _mm_add_epi16 (lineM2u, lineM2d);

      M0 = _mm_add_epi16 (M0, M2);
      M0 = _mm_hadd_epi16 (M0, M1);
      M1 = _mm_shuffle_epi32 (M0, 0xee);
      M1 = _mm_slli_epi16 (M1, 0x1);
      M1 = _mm_sub_epi16 (M0, M1);
      M1 = _mm_abs_epi16 (M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);

      _mm_storeu_si16 (&act, M1);
      taAct += (uint64_t) act;

      _mm_storeu_si128 ((__m128i*) &oM2[ y   *O + x], lineM1u);
      _mm_storeu_si128 ((__m128i*) &oM2[(y+1)*O + x], lineM1d);
      _mm_storeu_si128 ((__m128i*) &oM1[ y   *O + x], lineM0u);
      _mm_storeu_si128 ((__m128i*) &oM1[(y+1)*O + x], lineM0d);
    }
  }
  return (taAct << 1); /* * XPSNR_GAMMA */
}
#endif /* __AVX2__ */

#endif /* AVFILTER_XPSNR_H */
