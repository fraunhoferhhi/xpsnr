XPSNR Filter Plug-in for FFmpeg
===============================

The Extended Perceptually Weighted Peak Signal-to-Noise Ratio XPSNR
is a low-complexity psychovisually motivated distortion measurement
algorithm for assessing the difference between two video streams or
images. This is particularly useful for objectively quantifying the
distortions caused by video or image codecs, as an alternative to a
formal subjective test.  The logarithmic XPSNR output values are in
a similar range as those of traditional PSNR assessments but better
reflect human impressions of visual coding quality. More details on
the XPSNR method can be found in the following scientific papers:

*   C. R. Helmrich, M. Siekmann, S. Becker, S. Bosse, D. Marpe, and
 T. Wiegand, “XPSNR: A Low-Complexity Extension of the Perceptually
 Weighted Peak Signal-to-Noise Ratio for High-Resolution Video Qua-
 lity Assessment,” in Proc. IEEE Int. Conf. Acoustics, Speech, Sig.
 Process. (ICASSP), virt./online, May 2020. www.ecodis.de/xpsnr.htm

*   C. R. Helmrich, S. Bosse, H. Schwarz, D. Marpe, and T. Wiegand,
“A Study of the Extended Perceptually Weighted Peak Signal-to-Noise
 Ratio (XPSNR) for Video Compression with Different Resolutions and
 Bit Depths,” ITU Journal: ICT Discoveries, vol. 3, no. 1, pp. 65 -
 72, May 2020. http://handle.itu.int/11.1002/pub/8153d78b-en

This software allows to determine XPSNR output values, per-frame or
averaged across all assessed frames, between two video streams (the
reference, or input, video and the reconstructed, or output, video)
using the *FFmpeg* software suite, available at https://ffmpeg.org/

When publishing the results of XPSNR assessments (which the license
of this XPSNR implementation allows, see below), a reference to the
above papers as a means of documentation is strongly encouraged.

___________________________________________________________________


Copyright
---------

© 2019 - 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten
Forschung e.V. (Fraunhofer). All rights reserved.


License
-------

The copyright in this software implementation of the XPSNR model is
being made available under the following Software Copyright License
which is similar to the 3-clause BSD license but altered to address
specific aspects dictated by the nature and use of this application.
Please note that this software may be subject to other third-party
and contributor rights, including patent rights, and no such rights
are granted under this license.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted for non-commercial pur-
poses of evaluation, testing, and academic research (often referred
to as internal use) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions, and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions, and the following disclaimer
  in the documentation and/or other materials provided with the
  distribution.
* Neither the names of the copyright holders nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
“AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
NO EXPRESS OR IMPLIED LICENSES TO ANY PATENT CLAIMS, INCLUDING WITH-
OUT LIMITATION THE PATENTS OF THE COPYRIGHT HOLDERS AND CONTRIBUTORS,
ARE GRANTED BY THIS SOFTWARE COPYRIGHT LICENSE. THE COPYRIGHT HOLDERS
AND CONTRIBUTORS PROVIDE NO WARRANTY OF PATENT NON-INFRINGEMENT WITH
RESPECT TO THIS SOFTWARE.


Compilation
-----------

This section describes how to compile the source code of this XPSNR
implementation into the FFmpeg executable application as A/V filter
under Linux. Instructions for Microsoft Windows may be added later.
Please also see https://trac.ffmpeg.org/wiki/CompilationGuide for a
more detailed explanation of the steps required to compile FFmpeg.

First, you need to obtain the latest revision of the *FFmpeg 4.3.x*
source code from its Git repository:

`git clone https://git.ffmpeg.org/ffmpeg.git ffmpeg`

Then, you need to copy the files inside the `libavfilter` directory
of this source distribution into FFmpeg's `libavfilter` directory:

`cp xpsnr/libavfilter/* ffmpeg/libavfilter/`

Note that the `allfilters.c` and `Makefile` files already exist in
the FFmpeg source distribution and will be replaced (XPSNR related
lines have been added in each of these source files). The `xpsnr.h`
and `vf_xpsnr.c` files will be added to the FFmpeg distribution.

Now, you can configure and compile FFmpeg to generate the `ffmpeg`
executable with integrated XPSNR support:

`cd ffmpeg`

`./configure`

`make`


Usage
-----

This section describes how to calculate XPSNR output values between
two video streams using a compiled FFmpeg executable which includes
this plug-in (see the “Compilation” section above). Since the XPSNR
filter works similarly to FFmpeg's existing PSNR filter, it is also
worth taking a look at https://trac.ffmpeg.org/wiki/FilteringGuide,
or http://ffmpeg.org/ffmpeg-filters.html#psnr specifically.

### Simple command-line examples:

The following examples assume an assessment of two 50-Hz input YUVs
(uncompressed video files), inRef.yuv and inTest.yuv, under a Linux
OS. For usage under Windows, replace the `./ffmpeg` by `ffmpeg.exe`.

Two output files are created when using these command-lines, a .log
file with frame-wise XPSNR statistics and a .yuv file (identical to
the inRef.yuv file), which demonstrates that the XPSNR plug-in does
not change the video input. Use of both output files can be omitted
by employing `stats_file=-` instead of `stats_file=(x)psnr.log` and
`-f null -` instead of `-y out.yuv`, respectively.


*8-bit HD reference and test YUVs, first 100 frames:*

`
./ffmpeg -s 1920x1080 -framerate 50 -i inRef.yuv
  -s 1920x1080 -framerate 50 -i inTest.yuv
  -lavfi xpsnr="stats_file=xpsnr.log" -vframes 100 -y out.yuv
`

*10-bit UHD reference and test YUVs, first 100 frames:*

`
./ffmpeg -s 3840x2160 -framerate 50 -pix_fmt yuv420p10le -i inRef.yuv
  -s 3840x2160 -framerate 50 -pix_fmt yuv420p10le -i inTest.yuv
  -lavfi xpsnr="stats_file=xpsnr.log" -vframes 100 -y out.yuv
`

*8-bit HD reference, 10-bit HD test YUV, all frames:*

`
./ffmpeg -s 1920x1080 -framerate 50 -pix_fmt yuv420p -i inRef.yuv
  -s 1920x1080 -framerate 50 -pix_fmt yuv420p10le -i inTest.yuv
  -lavfi xpsnr="stats_file=xpsnr.log" -y out.yuv
`

*8-bit HD reference, 10-bit HD test YUV, PSNR for comparison:*

`
./ffmpeg -s 1920x1080 -framerate 50 -pix_fmt yuv420p -i inRef.yuv
  -s 1920x1080 -framerate 50 -pix_fmt yuv420p10le -i inTest.yuv
  -lavfi psnr="stats_file=psnr.log" -y out.yuv
`


Development
-----------

This section addresses two aspects related to future development of
the XPSNR plug-in for FFmpeg (but not the XPSNR algorithm itself).

### Code contribution:

If you are interested in contributing to this implementation of the
XPSNR algorithm, please contact Fraunhofer HHI or the contributors.
Pull requests with bugfixes and/or speedups are highly appreciated.

### Feature roadmap:

The following functionality is planned to be integrated in a future
revision of this XPSNR implementation. Support is kindly requested.

* support for RGB input (via RGB-to-YCbCr pre-conversion upon read)
* support for SIMD (SSE) and multithreading during visual filtering
* support for metadata as with FFmpeg's existing PSNR, SSIM filters
* direct XPSNR integration into FFmpeg codebase if widely requested


Contact Information
-------------------

Christian Helmrich and Christian Stoffers,
Fraunhofer Heinrich Hertz Institute (HHI),
Video Coding & Analytics (VCA) Department,
Einsteinufer 37, 10587 Berlin, Germany.

For detailed contact information please see “People and Contact” at
https://www.hhi.fraunhofer.de/en/departments/vca.html
