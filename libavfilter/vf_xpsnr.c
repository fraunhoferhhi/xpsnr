/*
File: vf_xpsnr.c - code definitions for XPSNR measurement filter plug-in for FFmpeg
Authors: Christian Helmrich and Christian Stoffers, Fraunhofer HHI, Berlin, Germany

License:

The copyright in this software is being made available under this Software Copyright
License. This software may be subject to other third-party and contributor rights,
including patent rights, and no such rights are granted under this license.

Copyright (c) 2019 - 2024 Fraunhofer-Gesellschaft zur Förderung der angewandten
Forschung e.V. (Fraunhofer). All rights reserved.

Redistribution and use of this software in source and binary forms, with or without
modification, are permitted for non-commercial and commercial purposes provided that
the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list
  of conditions, and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions, and the following disclaimer in the documentation and/or other
  materials provided with the distribution.
* Neither the names of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

NO PATENTS GRANTED

NO EXPRESS OR IMPLIED LICENSES TO ANY PATENT CLAIMS, INCLUDING WITHOUT LIMITATION
THE PATENTS OF THE COPYRIGHT HOLDER AND CONTRIBUTORS, ARE GRANTED BY THIS SOFTWARE
LICENSE. THE COPYRIGHT HOLDER AND CONTRIBUTORS PROVIDE NO WARRANTY OF PATENT NON-
INFRINGEMENT WITH RESPECT TO THIS SOFTWARE.
*/

/*
 * @file
 * Calculate the extended perceptually weighted PSNR (XPSNR) between two input videos.
 */

#include <stdbool.h>
#include "libavutil/avstring.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "drawutils.h"
#include "formats.h"
#include "framesync.h"
#include "internal.h"
#include "video.h"
#include "xpsnr.h"

/* XPSNR structure definition */

typedef struct XPSNRContext
{
  /* required basic variables */
  const AVClass   *class;
  int             bpp; /* unpacked */
  int             depth; /* packed */
  char            comps[4];
  int             numComps;
  uint64_t        numFrames64;
  unsigned        frameRate;
  FFFrameSync     fs;
  int             lineSizes[4];
  int             planeHeight[4];
  int             planeWidth[4];
  uint8_t         rgbaMap[4];
  FILE            *statsFile;
  char            *statsFileStr;
  /* XPSNR specific variables */
  double          *sseLuma;
  double          *weights;
  AVBufferRef*    bufOrg  [3];
  AVBufferRef*    bufOrgM1[3];
  AVBufferRef*    bufOrgM2[3];
  AVBufferRef*    bufRec  [3];
  uint64_t        maxError64;
  double          sumWDist[3];
  double          sumXPSNR[3];
  bool            andIsInf[3];
  bool            isRGB;
  PSNRDSPContext  dsp;
}
XPSNRContext;

/* required macro definitions */

#define FLAGS     AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#define OFFSET(x) offsetof(XPSNRContext, x)
#define XPSNR_GAMMA 2

static const AVOption xpsnr_options[] =
{
  {"stats_file", "Set file where to store per-frame difference information", OFFSET (statsFileStr), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS},
  {"f",          "Set file where to store per-frame difference information", OFFSET (statsFileStr), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS},
  { NULL }
};

FRAMESYNC_DEFINE_CLASS (xpsnr, XPSNRContext, fs);

/* XPSNR function definitions */
static uint64_t highds (const int xAct, const int yAct, const int wAct, const int hAct, const int16_t *o, const int O)
{
  uint64_t saAct = 0;

  for (int y = yAct; y < hAct; y += 2)
  {
    for (int x = xAct; x < wAct; x += 2)
    {
      const int f = 12 * ((int)o[ y   *O + x  ] + (int)o[ y   *O + x+1] + (int)o[(y+1)*O + x  ] + (int)o[(y+1)*O + x+1])
                   - 3 * ((int)o[(y-1)*O + x  ] + (int)o[(y-1)*O + x+1] + (int)o[(y+2)*O + x  ] + (int)o[(y+2)*O + x+1])
                   - 3 * ((int)o[ y   *O + x-1] + (int)o[ y   *O + x+2] + (int)o[(y+1)*O + x-1] + (int)o[(y+1)*O + x+2])
                   - 2 * ((int)o[(y-1)*O + x-1] + (int)o[(y-1)*O + x+2] + (int)o[(y+2)*O + x-1] + (int)o[(y+2)*O + x+2])
                       - ((int)o[(y-2)*O + x-1] + (int)o[(y-2)*O + x  ] + (int)o[(y-2)*O + x+1] + (int)o[(y-2)*O + x+2]
                        + (int)o[(y+3)*O + x-1] + (int)o[(y+3)*O + x  ] + (int)o[(y+3)*O + x+1] + (int)o[(y+3)*O + x+2]
                        + (int)o[(y-1)*O + x-2] + (int)o[ y   *O + x-2] + (int)o[(y+1)*O + x-2] + (int)o[(y+2)*O + x-2]
                        + (int)o[(y-1)*O + x+3] + (int)o[ y   *O + x+3] + (int)o[(y+1)*O + x+3] + (int)o[(y+2)*O + x+3]);
      saAct += (uint64_t) abs(f);
    }
  }
  return saAct;
}

static uint64_t diff1st (const uint32_t wAct, const uint32_t hAct, const int16_t *o, int16_t *oM1, const int O)
{
  uint64_t taAct = 0;

  for (uint32_t y = 0; y < hAct; y += 2)
  {
    for (uint32_t x = 0; x < wAct; x += 2)
    {
      const int t = (int)o  [y*O + x] + (int)o  [y*O + x+1] + (int)o  [(y+1)*O + x] + (int)o  [(y+1)*O + x+1]
                 - ((int)oM1[y*O + x] + (int)oM1[y*O + x+1] + (int)oM1[(y+1)*O + x] + (int)oM1[(y+1)*O + x+1]);
      taAct += (uint64_t) abs(t);
      oM1[y*O + x  ] = o  [y*O + x  ];  oM1[(y+1)*O + x  ] = o  [(y+1)*O + x  ];
      oM1[y*O + x+1] = o  [y*O + x+1];  oM1[(y+1)*O + x+1] = o  [(y+1)*O + x+1];
    }
  }
  return (taAct * XPSNR_GAMMA);
}

static uint64_t diff2nd (const uint32_t wAct, const uint32_t hAct, const int16_t *o, int16_t *oM1, int16_t *oM2, const int O)
{
  uint64_t taAct = 0;

  for (uint32_t y = 0; y < hAct; y += 2)
  {
    for (uint32_t x = 0; x < wAct; x += 2)
    {
      const int t = (int)o  [y*O + x] + (int)o  [y*O + x+1] + (int)o  [(y+1)*O + x] + (int)o  [(y+1)*O + x+1]
             - 2 * ((int)oM1[y*O + x] + (int)oM1[y*O + x+1] + (int)oM1[(y+1)*O + x] + (int)oM1[(y+1)*O + x+1])
                  + (int)oM2[y*O + x] + (int)oM2[y*O + x+1] + (int)oM2[(y+1)*O + x] + (int)oM2[(y+1)*O + x+1];
      taAct += (uint64_t) abs(t);
      oM2[y*O + x  ] = oM1[y*O + x  ];  oM2[(y+1)*O + x  ] = oM1[(y+1)*O + x  ];
      oM2[y*O + x+1] = oM1[y*O + x+1];  oM2[(y+1)*O + x+1] = oM1[(y+1)*O + x+1];
      oM1[y*O + x  ] = o  [y*O + x  ];  oM1[(y+1)*O + x  ] = o  [(y+1)*O + x  ];
      oM1[y*O + x+1] = o  [y*O + x+1];  oM1[(y+1)*O + x+1] = o  [(y+1)*O + x+1];
    }
  }
  return (taAct * XPSNR_GAMMA);
}

static uint64_t sseLine16bit (const uint8_t *blkOrg8, const uint8_t *blkRec8, int blockWidth)
{
  const uint16_t *blkOrg = (const uint16_t*) blkOrg8;
  const uint16_t *blkRec = (const uint16_t*) blkRec8;
  uint64_t lSSE = 0; /* data for 1 pixel line */

  for (int x = 0; x < blockWidth; x++)
  {
    const int64_t error = (int64_t) blkOrg[x] - (int64_t) blkRec[x];

    lSSE += error * error;
  }

  /* sum of squared errors for the pixel line */
  return lSSE;
}

static inline uint64_t calcSquaredError(XPSNRContext const *s,
                                        const int16_t *blkOrg,     const uint32_t strideOrg,
                                        const int16_t *blkRec,     const uint32_t strideRec,
                                        const uint32_t blockWidth, const uint32_t blockHeight)
{
  uint64_t uSSE = 0; /* sum of squared errors */

  for (uint32_t y = 0; y < blockHeight; y++)
  {
    uSSE += s->dsp.sse_line ((const uint8_t*) blkOrg, (const uint8_t*) blkRec, (int) blockWidth);
    blkOrg += strideOrg;
    blkRec += strideRec;
  }

  /* return nonweighted sum of squared errors */
  return uSSE;
}

static inline double calcSquaredErrorAndWeight (XPSNRContext const *s,
                                                const int16_t *picOrg,     const uint32_t strideOrg,
                                                int16_t       *picOrgM1,   int16_t       *picOrgM2,
                                                const int16_t *picRec,     const uint32_t strideRec,
                                                const uint32_t offsetX,    const uint32_t offsetY,
                                                const uint32_t blockWidth, const uint32_t blockHeight,
                                                const uint32_t bitDepth,   const uint32_t intFrameRate, double *msAct)
{
  const int      O = (int) strideOrg;
  const int      R = (int) strideRec;
  const int16_t *o = picOrg   + offsetY*O + offsetX;
  int16_t     *oM1 = picOrgM1 + offsetY*O + offsetX;
  int16_t     *oM2 = picOrgM2 + offsetY*O + offsetX;
  const int16_t *r = picRec   + offsetY*R + offsetX;
  const int   bVal = (s->planeWidth[0] * s->planeHeight[0] > 2048 * 1152 ? 2 : 1); /* threshold is a bit more than HD resolution */
  const int   xAct = (offsetX > 0 ? 0 : bVal);
  const int   yAct = (offsetY > 0 ? 0 : bVal);
  const int   wAct = (offsetX + blockWidth  < (uint32_t) s->planeWidth [0] ? (int) blockWidth  : (int) blockWidth  - bVal);
  const int   hAct = (offsetY + blockHeight < (uint32_t) s->planeHeight[0] ? (int) blockHeight : (int) blockHeight - bVal);

  const double sse = (double) calcSquaredError (s, o, strideOrg,
                                                r, strideRec,
                                                blockWidth, blockHeight);
  uint64_t saAct = 0;  /* spatial abs. activity */
  uint64_t taAct = 0; /* temporal abs. activity */

  if (wAct <= xAct || hAct <= yAct) /* too tiny */
  {
    return sse;
  }

  if (bVal > 1) /* highpass with downsampling */
  {
    saAct = s->dsp.highds_func (xAct, yAct, wAct, hAct, o, O);
  }
  else /* <=HD, highpass without downsampling */
  {
    for (int y = yAct; y < hAct; y++)
    {
      for (int x = xAct; x < wAct; x++)
      {
        const int f = 12 * (int)o[y*O + x] - 2 * ((int)o[y*O + x-1] + (int)o[y*O + x+1] + (int)o[(y-1)*O + x] + (int)o[(y+1)*O + x])
                        - ((int)o[(y-1)*O + x-1] + (int)o[(y-1)*O + x+1] + (int)o[(y+1)*O + x-1] + (int)o[(y+1)*O + x+1]);
        saAct += (uint64_t) abs(f);
      }
    }
  }

  /* calculate weight (mean squared activity) */
  *msAct = (double) saAct / ((double)(wAct - xAct) * (double)(hAct - yAct));

  if (bVal > 1) /* highpass with downsampling */
  {
    if (intFrameRate <= 32) /* 1st-order diff */
    {
      taAct = s->dsp.diff1st_func (blockWidth, blockHeight, o, oM1, O);
    }
    else  /* 2nd-order diff (diff of 2 diffs) */
    {
      taAct = s->dsp.diff2nd_func (blockWidth, blockHeight, o, oM1, oM2, O);
    }
  }
  else /* <=HD, highpass without downsampling */
  {
    if (intFrameRate <= 32) /* 1st-order diff */
    {
      for (uint32_t y = 0; y < blockHeight; y++)
      {
        for (uint32_t x = 0; x < blockWidth; x++)
        {
          const int t = (int)o[y*O + x] - (int)oM1[y*O + x];

          taAct += XPSNR_GAMMA * (uint64_t) abs(t);
          oM1[y*O + x] = o  [y*O + x];
        }
      }
    }
    else  /* 2nd-order diff (diff of 2 diffs) */
    {
      for (uint32_t y = 0; y < blockHeight; y++)
      {
        for (uint32_t x = 0; x < blockWidth; x++)
        {
          const int t = (int)o[y*O + x] - 2 * (int)oM1[y*O + x] + (int)oM2[y*O + x];

          taAct += XPSNR_GAMMA * (uint64_t) abs(t);
          oM2[y*O + x] = oM1[y*O + x];
          oM1[y*O + x] = o  [y*O + x];
        }
      }
    }
  }

  /* weight += mean squared temporal activity */
  *msAct += (double) taAct / ((double) blockWidth * (double) blockHeight);

  /* lower limit, accounts for high-pass gain */
  if (*msAct < (double)(1 << (bitDepth - 6))) *msAct = (double)(1 << (bitDepth - 6));

  *msAct *= *msAct; /* because SSE is squared */

  /* return nonweighted sum of squared errors */
  return sse;
}

static inline double getAvgXPSNR (const double sqrtWSSEData, const double sumXPSNRData,
                                  const uint32_t imageWidth, const uint32_t imageHeight,
                                  const uint64_t maxError64, const uint64_t numFrames64)
{
  if (numFrames64 == 0) return INFINITY;

  if (sqrtWSSEData >= (double) numFrames64) /* sq.-mean-root dist averaging */
  {
    const double meanDist = sqrtWSSEData / (double) numFrames64;
    const uint64_t  num64 = (uint64_t) imageWidth * (uint64_t) imageHeight * maxError64;

    return 10.0 * log10 ((double) num64 / ((double) meanDist * (double) meanDist));
  }

  return sumXPSNRData / (double) numFrames64; /* older log-domain averaging */
}

static int getWSSE (AVFilterContext *ctx, int16_t **org, int16_t **orgM1, int16_t **orgM2, int16_t **rec, uint64_t* const wsse64)
{
  XPSNRContext* const s = ctx->priv;
  const uint32_t      W = s->planeWidth [0];  /* luma image width in pixels */
  const uint32_t      H = s->planeHeight[0]; /* luma image height in pixels */
  const double        R = (double)(W * H) / (3840.0 * 2160.0); /* UHD ratio */
  const uint32_t      B = MAX (0, 4 * (int32_t)(32.0 * sqrt (R) + 0.5)); /* block size, integer multiple of 4 for SIMD */
  const uint32_t   WBlk = (W + B - 1) / B; /* luma width in units of blocks */
  const double   avgAct = sqrt (16.0 * (double)(1 << (2 * s->depth - 9)) / sqrt (MAX (0.00001, R))); /* = sqrt (a_pic) */
  const int*  strideOrg = (s->bpp == 1 ? s->planeWidth : s->lineSizes);
  uint32_t x, y, idxBlk = 0; /* the "16.0" above is due to fixed-point code */
  double* const sseLuma = s->sseLuma;
  double* const weights = s->weights;
  int c;

  if ((wsse64 == NULL) || (s->depth < 6) || (s->depth > 16) || (s->numComps <= 0) || (s->numComps > 3) || (W == 0) || (H == 0))
  {
    av_log (ctx, AV_LOG_ERROR, "Error in XPSNR routine: invalid argument(s).\n");

    return AVERROR (EINVAL);
  }

  if ((weights == NULL) || (B >= 4 && sseLuma == NULL))
  {
    av_log (ctx, AV_LOG_ERROR, "Failed to allocate temporary block memory.\n");

    return AVERROR (ENOMEM);
  }

  if (B >= 4)
  {
    const bool blockWeightSmoothing = (W * H <= 640u * 480u); /* JITU paper */
    const int16_t *pOrg = org[0];
    const uint32_t sOrg = strideOrg[0] / s->bpp;
    const int16_t *pRec = rec[0];
    const uint32_t sRec = s->planeWidth[0];
    int16_t     *pOrgM1 = orgM1[0]; /* pixel  */
    int16_t     *pOrgM2 = orgM2[0]; /* memory */
    double wsseLuma = 0.0;

    for (y = 0; y < H; y += B) /* calculate block SSE and perceptual weight */
    {
      const uint32_t blockHeight = (y + B > H ? H - y : B);

      for (x = 0; x < W; x += B, idxBlk++)
      {
        const uint32_t blockWidth = (x + B > W ? W - x : B);
        double msAct = 1.0, msActPrev = 0.0;

        sseLuma[idxBlk] = calcSquaredErrorAndWeight(s, pOrg, sOrg,
                                                    pOrgM1, pOrgM2,
                                                    pRec, sRec,
                                                    x, y,
                                                    blockWidth, blockHeight,
                                                    s->depth, s->frameRate, &msAct);
        weights[idxBlk] = 1.0 / sqrt (msAct);

        if (blockWeightSmoothing) /* inline "minimum-smoothing" as in paper */
        {
          if (x == 0) /* first column */
          {
            msActPrev = (idxBlk > 1 ? weights[idxBlk - 2] : 0);
          }
          else  /* after first column */
          {
            msActPrev = (x > B ? MAX (weights[idxBlk - 2], weights[idxBlk]) : weights[idxBlk]);
          }
          if (idxBlk > WBlk) /* after first row and first column */
          {
            msActPrev = MAX (msActPrev, weights[idxBlk - 1 - WBlk]); /* min (left, top) */
          }
          if ((idxBlk > 0) && (weights[idxBlk - 1] > msActPrev))
          {
            weights[idxBlk - 1] = msActPrev;
          }
          if ((x + B >= W) && (y + B >= H) && (idxBlk > WBlk)) /* last block in picture */
          {
            msActPrev = MAX (weights[idxBlk - 1], weights[idxBlk - WBlk]);
            if (weights[idxBlk] > msActPrev)
            {
              weights[idxBlk] = msActPrev;
            }
          }
        }
      } /* for x */
    } /* for y */

    for (y = idxBlk = 0; y < H; y += B) /* calculate sum for luma (Y) XPSNR */
    {
      for (x = 0; x < W; x += B, idxBlk++)
      {
        wsseLuma += sseLuma[idxBlk] * weights[idxBlk];
      }
    }
    wsse64[0] = (wsseLuma <= 0.0 ? 0 : (uint64_t)(wsseLuma * avgAct + 0.5));
  } /* B >= 4 */

  for (c = 0; c < s->numComps; c++) /* finalize SSE data for all components */
  {
    const int16_t *pOrg = org[c];
    const uint32_t sOrg = strideOrg[c] / s->bpp;
    const int16_t *pRec = rec[c];
    const uint32_t sRec = s->planeWidth[c];
    const uint32_t WPln = s->planeWidth[c];
    const uint32_t HPln = s->planeHeight[c];

    if (B < 4) /* picture is too small for XPSNR, calculate unweighted PSNR */
    {
      wsse64[c] = calcSquaredError (s, pOrg, sOrg,
                                    pRec, sRec,
                                    WPln, HPln);
    }
    else if (c > 0) /* B >= 4, so Y XPSNR has already been calculated above */
    {
      const uint32_t Bx = (B * WPln) / W;
      const uint32_t By = (B * HPln) / H; /* up to chroma downsampling by 4 */
      double wsseChroma = 0.0;

      for (y = idxBlk = 0; y < HPln; y += By) /* calc. chroma (Cb/Cr) XPSNR */
      {
        const uint32_t blockHeight = (y + By > HPln ? HPln - y : By);

        for (x = 0; x < WPln; x += Bx, idxBlk++)
        {
          const uint32_t blockWidth = (x + Bx > WPln ? WPln - x : Bx);

          wsseChroma += (double) calcSquaredError(s, pOrg + y*sOrg + x, sOrg,
                                                  pRec + y*sRec + x, sRec,
                                                  blockWidth, blockHeight) * weights[idxBlk];
        }
      }
      wsse64[c] = (wsseChroma <= 0.0 ? 0 : (uint64_t)(wsseChroma * avgAct + 0.5));
    }
  } /* for c */

  return 0;
}

static int do_xpsnr (FFFrameSync *fs)
{
  AVFilterContext  *ctx = fs->parent;
  XPSNRContext* const s = ctx->priv;
  const uint32_t      W = s->planeWidth [0];  /* luma image width in pixels */
  const uint32_t      H = s->planeHeight[0]; /* luma image height in pixels */
  const uint32_t      B = MAX (0, 4 * (int32_t)(32.0 * sqrt ((double)(W * H) / (3840.0 * 2160.0)) + 0.5)); /* block size */
  const uint32_t   WBlk = (W + B - 1) / B; /* luma width in units of blocks */
  const uint32_t   HBlk = (H + B - 1) / B;/* luma height in units of blocks */
  AVFrame *master, *ref = NULL;
  int16_t *pOrg  [3];
  int16_t *pOrgM1[3];
  int16_t *pOrgM2[3];
  int16_t *pRec  [3];
  uint64_t wsse64[3] = {0, 0, 0};
  double curXPSNR[3] = {INFINITY, INFINITY, INFINITY};
  int c, retValue;

  if ((retValue = ff_framesync_dualinput_get (fs, &master, &ref)) < 0) return retValue;
  if (ref == NULL) return ff_filter_frame (ctx->outputs[0], master);

  /* prepare XPSNR calculation: allocate temporary picture and block memory */
  if (s->sseLuma == NULL) s->sseLuma = (double*) av_malloc_array (WBlk * HBlk, sizeof (double));
  if (s->weights == NULL) s->weights = (double*) av_malloc_array (WBlk * HBlk, sizeof (double));

  for (c = 0; c < s->numComps; c++)  /* allocate temporal org buffer memory */
  {
    s->lineSizes[c] = master->linesize[c];

    if (c == 0) /* luma ch. */
    {
      const int strideOrgBpp = (s->bpp == 1 ? s->planeWidth[c] : s->lineSizes[c] / s->bpp);

      if (s->bufOrgM1[c] == NULL) s->bufOrgM1[c] = av_buffer_allocz (strideOrgBpp * s->planeHeight[c] * sizeof (int16_t));
      if (s->bufOrgM2[c] == NULL) s->bufOrgM2[c] = av_buffer_allocz (strideOrgBpp * s->planeHeight[c] * sizeof (int16_t));

      pOrgM1[c] = (int16_t*) s->bufOrgM1[c]->data;
      pOrgM2[c] = (int16_t*) s->bufOrgM2[c]->data;
    }
  }

  if (s->bpp == 1) /* 8 bit */
  {
    for (c = 0; c < s->numComps; c++) /* allocate the org/rec buffer memory */
    {
      const int M = s->lineSizes[c]; /* master stride */
      const int R = ref->linesize[c]; /* ref/c stride */
      const int O = s->planeWidth[c]; /* XPSNR stride */

      if (s->bufOrg[c] == NULL) s->bufOrg[c] = av_buffer_allocz (s->planeWidth[c] * s->planeHeight[c] * sizeof (int16_t));
      if (s->bufRec[c] == NULL) s->bufRec[c] = av_buffer_allocz (s->planeWidth[c] * s->planeHeight[c] * sizeof (int16_t));

      pOrg[c] = (int16_t*) s->bufOrg[c]->data;
      pRec[c] = (int16_t*) s->bufRec[c]->data;

      for (int y = 0; y < s->planeHeight[c]; y++)
      {
        for (int x = 0; x < s->planeWidth[c]; x++)
        {
          pOrg[c][y*O + x] = (int16_t) master->data[c][y*M + x];
          pRec[c][y*O + x] = (int16_t)    ref->data[c][y*R + x];
        }
      }
    }
  }
  else /* 10, 12, or 14 bit */
  {
    for (c = 0; c < s->numComps; c++)
    {
      pOrg[c] = (int16_t*) master->data[c];
      pRec[c] = (int16_t*)    ref->data[c];
    }
  }

  /* extended perceptually weighted peak signal-to-noise ratio (XPSNR) data */

  if ((retValue = getWSSE (ctx, (int16_t **)&pOrg, (int16_t **)&pOrgM1, (int16_t **)&pOrgM2, (int16_t **)&pRec, wsse64)) < 0)
  {
    return retValue; /* an error here implies something went wrong earlier! */
  }

  for (c = 0; c < s->numComps; c++)
  {
    const double sqrtWSSE = sqrt ((double) wsse64[c]);

    curXPSNR[c] = getAvgXPSNR (sqrtWSSE, INFINITY,
                               s->planeWidth[c], s->planeHeight[c],
                               s->maxError64, 1 /* single frame */);
    s->sumWDist[c] += sqrtWSSE;
    s->sumXPSNR[c] += curXPSNR[c];
    s->andIsInf[c] &= isinf (curXPSNR[c]);
  }
  s->numFrames64++;

  if (s->statsFile != NULL) /* print out frame- and component-wise averages */
  {
    fprintf (s->statsFile, "n: %4"PRId64"", s->numFrames64);

    for (c = 0; c < s->numComps; c++)
    {
      fprintf (s->statsFile, "  XPSNR %c: %3.4f", s->comps[c], curXPSNR[c]);
    }
    fprintf (s->statsFile, "\n");
  }

  return ff_filter_frame (ctx->outputs[0], master);
}

static av_cold int init (AVFilterContext *ctx)
{
  XPSNRContext* const s = ctx->priv;
  int c;

  if (s->statsFileStr != NULL)
  {
    if (!strcmp (s->statsFileStr, "-"))  /* no statistics file, take stdout */
    {
      s->statsFile = stdout;
    }
    else
    {
      s->statsFile = fopen (s->statsFileStr, "w");

      if (s->statsFile == NULL)
      {
        const int err = AVERROR (errno);
        char buf[128];

        av_strerror (err, buf, sizeof (buf));
        av_log (ctx, AV_LOG_ERROR, "Could not open statistics file %s: %s\n", s->statsFileStr, buf);

        return err;
      }
    }
  }

  s->sseLuma = NULL;
  s->weights = NULL;

  for (c = 0; c < 3; c++) /* initialize XPSNR data of every color component */
  {
    s->bufOrg  [c] = NULL;
    s->bufOrgM1[c] = NULL;
    s->bufOrgM2[c] = NULL;
    s->bufRec  [c] = NULL;
    s->sumWDist[c] = 0.0;
    s->sumXPSNR[c] = 0.0;
    s->andIsInf[c] = true;
  }

  s->fs.on_event = do_xpsnr;

  return 0;
}

static const enum AVPixelFormat xpsnr_formats[] =
{
  AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9, AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16,
#define PF_NOALPHA(suf) AV_PIX_FMT_YUV420##suf,  AV_PIX_FMT_YUV422##suf,  AV_PIX_FMT_YUV444##suf
#define PF_ALPHA(suf)   AV_PIX_FMT_YUVA420##suf, AV_PIX_FMT_YUVA422##suf, AV_PIX_FMT_YUVA444##suf
#define PF(suf)         PF_NOALPHA(suf), PF_ALPHA(suf)
  PF(P), PF(P9), PF(P10), PF_NOALPHA(P12), PF_NOALPHA(P14), PF(P16),
  AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
  AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
  AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ444P,
  AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10,
  AV_PIX_FMT_GBRP12, AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
  AV_PIX_FMT_GBRAP, AV_PIX_FMT_GBRAP10, AV_PIX_FMT_GBRAP12, AV_PIX_FMT_GBRAP16,
  AV_PIX_FMT_NONE
};

static int config_input_ref (AVFilterLink *inLink)
{
  const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get (inLink->format);
  AVFilterContext  *ctx = inLink->dst;
  XPSNRContext* const s = ctx->priv;
  int cpu_flags;

  if ((ctx->inputs[0]->w != ctx->inputs[1]->w) ||
      (ctx->inputs[0]->h != ctx->inputs[1]->h))
  {
    av_log (ctx, AV_LOG_ERROR, "Width and height of the input videos must match.\n");

    return AVERROR (EINVAL);
  }

  if (ctx->inputs[0]->format != ctx->inputs[1]->format)
  {
    av_log (ctx, AV_LOG_ERROR, "The input videos must be of the same pixel format.\n");

    return AVERROR (EINVAL);
  }

  s->bpp =  (desc->comp[0].depth <= 8 ? 1 : 2);
  s->depth = desc->comp[0].depth;
#if 1
  s->maxError64 = (1 << s->depth) - 1; /* conventional limit */
#else
  s->maxError64 = 255 * (1 << (s->depth - 8)); /* JVET style */
#endif
  s->maxError64 *= s->maxError64;

  s->frameRate = inLink->frame_rate.num / inLink->frame_rate.den;

  s->numComps = (desc->nb_components > 3 ? 3 : desc->nb_components);

  s->isRGB = (ff_fill_rgba_map (s->rgbaMap, inLink->format) >= 0);
  s->comps[0] = (s->isRGB ? 'R' : 'Y');
  s->comps[1] = (s->isRGB ? 'G' : 'U');
  s->comps[2] = (s->isRGB ? 'B' : 'V');
  s->comps[3] = 'A';

  s->planeWidth [1] = s->planeWidth [2] = AV_CEIL_RSHIFT (inLink->w, desc->log2_chroma_w);
  s->planeWidth [0] = s->planeWidth [3] = inLink->w;
  s->planeHeight[1] = s->planeHeight[2] = AV_CEIL_RSHIFT (inLink->h, desc->log2_chroma_h);
  s->planeHeight[0] = s->planeHeight[3] = inLink->h;

  s->dsp.sse_line = sseLine16bit; /* initialize SIMD routine */
  if (ARCH_X86) ff_psnr_init_x86 (&s->dsp, 15); /* from PSNR */

  s->dsp.highds_func = highds; /* initialize customized AVX2 */
  s->dsp.diff1st_func = diff1st; /* SIMD routines from XPSNR */
  s->dsp.diff2nd_func = diff2nd;
  cpu_flags = av_get_cpu_flags();
  if (EXTERNAL_AVX2 (cpu_flags))
  {
#ifdef __AVX2__
    s->dsp.highds_func = highds_SIMD;
    s->dsp.diff1st_func = diff1st_SIMD;
    s->dsp.diff2nd_func = diff2nd_SIMD;
#endif
  }
  return 0;
}

static int config_output (AVFilterLink *outLink)
{
  AVFilterContext *ctx = outLink->src;
  AVFilterLink *mainLink = ctx->inputs[0];
  XPSNRContext *s = ctx->priv;
  int retValue;

  if ((retValue = ff_framesync_init_dualinput (&s->fs, ctx)) < 0) return retValue;

  outLink->w = mainLink->w;
  outLink->h = mainLink->h;
  outLink->frame_rate = mainLink->frame_rate;
  outLink->sample_aspect_ratio = mainLink->sample_aspect_ratio;
  outLink->time_base = mainLink->time_base;

  if ((retValue = ff_framesync_configure (&s->fs)) < 0) return retValue;

  return 0;
}

static int activate (AVFilterContext *ctx)
{
  XPSNRContext *s = ctx->priv;

  return ff_framesync_activate (&s->fs);
}

static av_cold void uninit (AVFilterContext *ctx)
{
  XPSNRContext* const s = ctx->priv;
  int c;

  if (s->numFrames64 > 0) /* print out overall component-wise XPSNR average */
  {
    const double xpsnrLuma = getAvgXPSNR (s->sumWDist[0], s->sumXPSNR[0],
                                          s->planeWidth[0], s->planeHeight[0],
                                          s->maxError64, s->numFrames64);
    double xpsnrMin = xpsnrLuma;

    /* luma */
    av_log (ctx, AV_LOG_INFO, "XPSNR  %c: %3.4f", s->comps[0], xpsnrLuma);
    if (s->statsFile != NULL)
    {
      fprintf (s->statsFile, "\nXPSNR average, %"PRId64" frames", s->numFrames64);
      fprintf (s->statsFile, "  %c: %3.4f", s->comps[0], xpsnrLuma);
    }
    /* chroma */
    for (c = 1; c < s->numComps; c++)
    {
      const double xpsnrChroma = getAvgXPSNR (s->sumWDist[c], s->sumXPSNR[c],
                                              s->planeWidth[c], s->planeHeight[c],
                                              s->maxError64, s->numFrames64);
      if (xpsnrMin > xpsnrChroma) xpsnrMin = xpsnrChroma;

      av_log (ctx, AV_LOG_INFO, "  %c: %3.4f", s->comps[c], xpsnrChroma);
      if (s->statsFile != NULL)
      {
        fprintf (s->statsFile, "  %c: %3.4f", s->comps[c], xpsnrChroma);
      }
    }
    /* print out line break (and minimum XPSNR across the color components) */
    if (s->numComps > 1)
    {
      av_log (ctx, AV_LOG_INFO, "  (minimum: %3.4f)\n", xpsnrMin);
      if (s->statsFile != NULL) fprintf (s->statsFile, "  (minimum: %3.4f)\n", xpsnrMin);
    }
    else
    {
      av_log (ctx, AV_LOG_INFO, "\n");
      if (s->statsFile != NULL) fprintf (s->statsFile, "\n");
    }
  }

  ff_framesync_uninit (&s->fs);  /* free temporary picture and block memory */

  if (s->sseLuma != NULL) av_freep (&s->sseLuma);
  if (s->weights != NULL) av_freep (&s->weights);

  for (c = 0; c < s->numComps; c++) /* free addl temporal org buffer memory */
  {
    if (s->bufOrgM1[c] != NULL) av_freep (&s->bufOrgM1[c]);
    if (s->bufOrgM2[c] != NULL) av_freep (&s->bufOrgM2[c]);
  }
  if (s->bpp == 1) /* 8 bit */
  {
    for (c = 0; c < s->numComps; c++) /* free org/rec picture buffer memory */
    {
      if (&s->bufOrg[c] != NULL) av_freep (&s->bufOrg[c]);
      if (&s->bufRec[c] != NULL) av_freep (&s->bufRec[c]);
    }
  }
}

static const AVFilterPad xpsnr_inputs[] =
{
  {
    .name         = "main",
    .type         = AVMEDIA_TYPE_VIDEO,
  },
  {
    .name         = "reference",
    .type         = AVMEDIA_TYPE_VIDEO,
    .config_props = config_input_ref,
  }
};

static const AVFilterPad xpsnr_outputs[] =
{
  {
    .name         = "default",
    .type         = AVMEDIA_TYPE_VIDEO,
    .config_props = config_output,
  }
};

AVFilter ff_vf_xpsnr =
{
  .name           = "xpsnr",
  .description    = NULL_IF_CONFIG_SMALL ("Calculate the extended perceptually weighted peak signal-to-noise ratio (XPSNR) between two video streams."),
  .preinit        = xpsnr_framesync_preinit,
  .init           = init,
  .uninit         = uninit,
  .activate       = activate,
  .priv_size      = sizeof (XPSNRContext),
  .priv_class     = &xpsnr_class,
  FILTER_INPUTS(xpsnr_inputs),
  FILTER_OUTPUTS(xpsnr_outputs),
  FILTER_PIXFMTS_ARRAY(xpsnr_formats),
  .flags          = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                    AVFILTER_FLAG_SLICE_THREADS             |
                    AVFILTER_FLAG_METADATA_ONLY
};
