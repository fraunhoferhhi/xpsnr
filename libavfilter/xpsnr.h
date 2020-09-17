/*
File: xpsnr.h - public declarations for XPSNR measurement filter plug-in for FFmpeg
Authors: Christian Helmrich and Christian Stoffers, Fraunhofer HHI, Berlin, Germany

License:

The copyright in this software is being made available under this Software Copyright
License. This software may be subject to other third-party and contributor rights,
including patent rights, and no such rights are granted under this license.

Copyright (c) 2019 - 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten
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

/* public XPSNR DSP structure definition */

typedef struct XPSNRDSPContext
{
  uint64_t (*sse_line) (const uint8_t *buf, const uint8_t *ref, int w);
}
XPSNRDSPContext;

void ff_xpsnr_init_x86 (XPSNRDSPContext *dsp, int bpp);

#endif /* AVFILTER_XPSNR_H */
