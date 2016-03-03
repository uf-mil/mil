/* -*- mode: C++ -*- */
/* $Id$ */
/*
 * Copyright (C) 2000-2004 Damien Douxchamps  <ddouxchamps@users.sf.net>
 *                         Dan Dennedy  <dan@dennedy.org>
 *
 * NOTE: On 4 Jan. 2011, this file was re-licensed under the GNU LGPL
 * with permission of the original GPL authors: Damien Douxchamps and
 * Dan Dennedy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

/* Copyright (C) 2010 Jack O'Quin
 *
 * Minor changes for use with the ROS camera1394 driver:
 *
 *  * generate RGB rather than BGR
 *  * repackage as a separate file
 *  * use yuv namespace
 */

#ifndef _YUV_H_
#define _YUV_H_

#include <dc1394/dc1394.h>

/** @file

    @brief YUV to RGB conversion functions
 */

namespace yuv
{

  /** unpack yuv444 to rgb8 */
  void inline uyv2rgb(const unsigned char *src, unsigned char *dest,
                      unsigned long long int NumPixels)
  {
    register int i = NumPixels + (NumPixels << 1) - 1;
    register int j = NumPixels + (NumPixels << 1) - 1;
    register int y, u, v;
    register int r, g, b;

    while (i > 0) {
      v = src[i--] - 128;
      y = src[i--];
      u = src[i--] - 128;
      YUV2RGB(y, u, v, r, g, b);
      dest[j--] = b;
      dest[j--] = g;
      dest[j--] = r;
    }
  }

  /** unpack yuv422 to rgb8 */
  void inline uyvy2rgb(unsigned char *src, unsigned char *dest,
                       unsigned long long int NumPixels)
  {
    register int i = (NumPixels << 1)-1;
    register int j = NumPixels + ( NumPixels << 1 ) -1;
    register int y0, y1, u, v;
    register int r, g, b;

    while (i > 0)
      {
        y1 = (unsigned char) src[i--];
        v  = (unsigned char) src[i--] - 128;
        y0 = (unsigned char) src[i--];
        u  = (unsigned char) src[i--] - 128;
        YUV2RGB (y1, u, v, r, g, b);
        dest[j--] = b;
        dest[j--] = g;
        dest[j--] = r;
        YUV2RGB (y0, u, v, r, g, b);
        dest[j--] = b;
        dest[j--] = g;
        dest[j--] = r;
      }
  }

  /** unpack yuv411 to rgb8 */
  void inline uyyvyy2rgb(const unsigned char *src, unsigned char *dest,
                         unsigned long long int NumPixels)
  {
    register int i = NumPixels + (NumPixels >> 1) - 1;
    register int j = NumPixels + (NumPixels << 1) - 1;
    register int y0, y1, y2, y3, u, v;
    register int r, g, b;

    while (i > 0) {
      y3 = src[i--];
      y2 = src[i--];
      v = src[i--] - 128;
      y1 = src[i--];
      y0 = src[i--];
      u = src[i--] - 128;
      YUV2RGB(y3, u, v, r, g, b);
      dest[j--] = b;
      dest[j--] = g;
      dest[j--] = r;
      YUV2RGB(y2, u, v, r, g, b);
      dest[j--] = b;
      dest[j--] = g;
      dest[j--] = r;
      YUV2RGB(y1, u, v, r, g, b);
      dest[j--] = b;
      dest[j--] = g;
      dest[j--] = r;
      YUV2RGB(y0, u, v, r, g, b);
      dest[j--] = b;
      dest[j--] = g;
      dest[j--] = r;
    }
  }
}

#endif // _YUV_H_
