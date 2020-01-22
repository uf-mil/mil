// This code is based on the original FAST corner detector by Edward Rosten.
// Below is the original copyright and references

/*
Copyright (c) 2006, 2008 Edward Rosten
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    *Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

    *Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

    *Neither the name of the University of Cambridge nor the names of 
     its contributors may be used to endorse or promote products derived 
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
The references are:
 * Machine learning for high-speed corner detection, 
   E. Rosten and T. Drummond, ECCV 2006
 * Faster and better: A machine learning approach to corner detection
   E. Rosten, R. Porter and T. Drummond, PAMI, 2009
*/

#ifndef SPARSESTEREO_FAST9_INL_H
#define SPARSESTEREO_FAST9_INL_H

#include "fast9.h"

namespace sparsestereo {
	template <typename T>
	const int FAST9<T>::pixelOffsetsX[16] = {0, 1, 2, 3, 3,  3,  2,  1,  0, -1, -2, -3, -3, -3, -2, -1};
	
	template <typename T>
	const int FAST9<T>::pixelOffsetsY[16] = {3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1,  0,  1,  2,  3};

	template <typename T>
	FAST9<T>::FAST9():step(-1) {
		setStep(640);
	}

	template <typename T>
	void FAST9<T>::setStep(int s) {
		if(s == step)
			return; // noting changed
			
		step = s;
		for(int i=0; i<16; i++)
			pixel[i] = pixelOffsetsX[i] + step * pixelOffsetsY[i];
	}
	
	/* This defines non-strict maxima */
#define SPARSESTEREO_NONMAX_COMPARE(X, Y) ((X)>=(Y))

	/* This is a fast, integer only, sparse nonmaximal suppression. */
	/* probably only useful for FAST corner detection */
	template <typename T> template<typename T2>
	void FAST9<T>::nonMaxSuppression(const std::vector<cv::Point2i>& corners, const std::vector<T2>& scores, std::vector<int>& ret_nonmax) const {
		int i, j, num_corners = (int)corners.size(); 

		// Point above points (roughly) to the pixel above the one of interest, if there is a feature there.
		int point_above = 0;
		int point_below = 0;

		ret_nonmax.clear();
		if(num_corners < 1)
			return;

		/* Find where each row begins
		   (the corners are output in raster scan order). A beginning of -1 signifies
		   that there are no corners on that row. */
		int last_row = corners[num_corners-1].y;
		std::vector<int> row_start(last_row+1);

		for(i=0; i < last_row+1; i++)
			row_start[i] = -1;

		int prev_row = -1;
		for(i=0; i< num_corners; i++)
			if(corners[i].y != prev_row)
			{
				row_start[corners[i].y] = i;
				prev_row = corners[i].y;
			}
						

		ret_nonmax.reserve(num_corners);
		for(i = 0; i < num_corners; i++) {

			T2 score = scores[i];
			cv::Point pos = corners[i];
		
			// Check left
			if(i > 0 && corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y) {
				if(SPARSESTEREO_NONMAX_COMPARE(scores[i-1], score))
					continue;
			}
		
			// Check right
			if(i < num_corners-1 && corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y) {
				if(SPARSESTEREO_NONMAX_COMPARE(scores[i+1], score))
					continue;
			}

			bool suppressed = false;
			// Check above (if there is a valid row above)
			if(pos.y != 0 && row_start[pos.y - 1] != -1)  {
				// Make sure that current point_above is one row above.
				if(corners[point_above].y < pos.y - 1)
					point_above = row_start[pos.y-1];
		
				// Make point_above point to the first of the pixels above the current point, if it exists.
				for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
					;
		
				for(j=point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++) {
					int x = corners[j].x;
					if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1)) {
						if(SPARSESTEREO_NONMAX_COMPARE(scores[j], score)) {
							suppressed = true;
							break;
						}
					}
				}
				if( suppressed )
					continue;
			}
		
			// Check below (if there is anything below)
			if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < num_corners) { // Nothing below
				if(corners[point_below].y < pos.y + 1)
					point_below = row_start[pos.y+1];
		
				// Make point below point to one of the pixels belowthe current point, if it exists.
				for(; point_below < num_corners && corners[point_below].y == pos.y+1
					&& corners[point_below].x < pos.x - 1; point_below++)
					;

				for(j=point_below; j < num_corners && corners[j].y == pos.y+1 && corners[j].x <= pos.x + 1; j++) {
					int x = corners[j].x;
					if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1)) {
						if(SPARSESTEREO_NONMAX_COMPARE(scores[j],score)) {
							suppressed = true;
							break;
						}
					}
				}
				if( suppressed )
					continue;
			}

			ret_nonmax.push_back(i);
		}
	}
	
	template <typename T>
	int FAST9<T>::cornerScore(const T* p, T c, int bstart) const {
        int bmin = bstart;
        int bmax = 255;
        int b = (bmax + bmin)/2;
        
        /*Compute the score using binary search*/
        for(;;)
        {
            int cb = c + b;
            int c_b= c - b;

            if( p[pixel[0]] > cb) {
             if( p[pixel[1]] > cb) {
              if( p[pixel[2]] > cb) {
               if( p[pixel[3]] > cb) {
                if( p[pixel[4]] > cb) {
                 if( p[pixel[5]] > cb) {
                  if( p[pixel[6]] > cb) {
                   if( p[pixel[7]] > cb) {
                    if( p[pixel[8]] > cb) {
                     goto is_a_corner;
                    } else
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else if( p[pixel[7]] < c_b) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else if( p[pixel[14]] < c_b) {
                     if( p[pixel[8]] < c_b) {
                      if( p[pixel[9]] < c_b) {
                       if( p[pixel[10]] < c_b) {
                        if( p[pixel[11]] < c_b) {
                         if( p[pixel[12]] < c_b) {
                          if( p[pixel[13]] < c_b) {
                           if( p[pixel[15]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else if( p[pixel[6]] < c_b) {
                   if( p[pixel[15]] > cb) {
                    if( p[pixel[13]] > cb) {
                     if( p[pixel[14]] > cb) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else if( p[pixel[13]] < c_b) {
                     if( p[pixel[7]] < c_b) {
                      if( p[pixel[8]] < c_b) {
                       if( p[pixel[9]] < c_b) {
                        if( p[pixel[10]] < c_b) {
                         if( p[pixel[11]] < c_b) {
                          if( p[pixel[12]] < c_b) {
                           if( p[pixel[14]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    if( p[pixel[7]] < c_b) {
                     if( p[pixel[8]] < c_b) {
                      if( p[pixel[9]] < c_b) {
                       if( p[pixel[10]] < c_b) {
                        if( p[pixel[11]] < c_b) {
                         if( p[pixel[12]] < c_b) {
                          if( p[pixel[13]] < c_b) {
                           if( p[pixel[14]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else if( p[pixel[13]] < c_b) {
                    if( p[pixel[7]] < c_b) {
                     if( p[pixel[8]] < c_b) {
                      if( p[pixel[9]] < c_b) {
                       if( p[pixel[10]] < c_b) {
                        if( p[pixel[11]] < c_b) {
                         if( p[pixel[12]] < c_b) {
                          if( p[pixel[14]] < c_b) {
                           if( p[pixel[15]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else if( p[pixel[5]] < c_b) {
                  if( p[pixel[14]] > cb) {
                   if( p[pixel[12]] > cb) {
                    if( p[pixel[13]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          if( p[pixel[10]] > cb) {
                           if( p[pixel[11]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else if( p[pixel[12]] < c_b) {
                    if( p[pixel[6]] < c_b) {
                     if( p[pixel[7]] < c_b) {
                      if( p[pixel[8]] < c_b) {
                       if( p[pixel[9]] < c_b) {
                        if( p[pixel[10]] < c_b) {
                         if( p[pixel[11]] < c_b) {
                          if( p[pixel[13]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else if( p[pixel[14]] < c_b) {
                   if( p[pixel[7]] < c_b) {
                    if( p[pixel[8]] < c_b) {
                     if( p[pixel[9]] < c_b) {
                      if( p[pixel[10]] < c_b) {
                       if( p[pixel[11]] < c_b) {
                        if( p[pixel[12]] < c_b) {
                         if( p[pixel[13]] < c_b) {
                          if( p[pixel[6]] < c_b) {
                           goto is_a_corner;
                          } else
                           if( p[pixel[15]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   if( p[pixel[6]] < c_b) {
                    if( p[pixel[7]] < c_b) {
                     if( p[pixel[8]] < c_b) {
                      if( p[pixel[9]] < c_b) {
                       if( p[pixel[10]] < c_b) {
                        if( p[pixel[11]] < c_b) {
                         if( p[pixel[12]] < c_b) {
                          if( p[pixel[13]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          if( p[pixel[10]] > cb) {
                           if( p[pixel[11]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else if( p[pixel[12]] < c_b) {
                   if( p[pixel[7]] < c_b) {
                    if( p[pixel[8]] < c_b) {
                     if( p[pixel[9]] < c_b) {
                      if( p[pixel[10]] < c_b) {
                       if( p[pixel[11]] < c_b) {
                        if( p[pixel[13]] < c_b) {
                         if( p[pixel[14]] < c_b) {
                          if( p[pixel[6]] < c_b) {
                           goto is_a_corner;
                          } else
                           if( p[pixel[15]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else if( p[pixel[4]] < c_b) {
                 if( p[pixel[13]] > cb) {
                  if( p[pixel[11]] > cb) {
                   if( p[pixel[12]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          if( p[pixel[10]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          if( p[pixel[10]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else if( p[pixel[11]] < c_b) {
                   if( p[pixel[5]] < c_b) {
                    if( p[pixel[6]] < c_b) {
                     if( p[pixel[7]] < c_b) {
                      if( p[pixel[8]] < c_b) {
                       if( p[pixel[9]] < c_b) {
                        if( p[pixel[10]] < c_b) {
                         if( p[pixel[12]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else if( p[pixel[13]] < c_b) {
                  if( p[pixel[7]] < c_b) {
                   if( p[pixel[8]] < c_b) {
                    if( p[pixel[9]] < c_b) {
                     if( p[pixel[10]] < c_b) {
                      if( p[pixel[11]] < c_b) {
                       if( p[pixel[12]] < c_b) {
                        if( p[pixel[6]] < c_b) {
                         if( p[pixel[5]] < c_b) {
                          goto is_a_corner;
                         } else
                          if( p[pixel[14]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                        } else
                         if( p[pixel[14]] < c_b) {
                          if( p[pixel[15]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  if( p[pixel[5]] < c_b) {
                   if( p[pixel[6]] < c_b) {
                    if( p[pixel[7]] < c_b) {
                     if( p[pixel[8]] < c_b) {
                      if( p[pixel[9]] < c_b) {
                       if( p[pixel[10]] < c_b) {
                        if( p[pixel[11]] < c_b) {
                         if( p[pixel[12]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          if( p[pixel[10]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          if( p[pixel[10]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else if( p[pixel[11]] < c_b) {
                  if( p[pixel[7]] < c_b) {
                   if( p[pixel[8]] < c_b) {
                    if( p[pixel[9]] < c_b) {
                     if( p[pixel[10]] < c_b) {
                      if( p[pixel[12]] < c_b) {
                       if( p[pixel[13]] < c_b) {
                        if( p[pixel[6]] < c_b) {
                         if( p[pixel[5]] < c_b) {
                          goto is_a_corner;
                         } else
                          if( p[pixel[14]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                        } else
                         if( p[pixel[14]] < c_b) {
                          if( p[pixel[15]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
               } else if( p[pixel[3]] < c_b) {
                if( p[pixel[10]] > cb) {
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else if( p[pixel[10]] < c_b) {
                 if( p[pixel[7]] < c_b) {
                  if( p[pixel[8]] < c_b) {
                   if( p[pixel[9]] < c_b) {
                    if( p[pixel[11]] < c_b) {
                     if( p[pixel[6]] < c_b) {
                      if( p[pixel[5]] < c_b) {
                       if( p[pixel[4]] < c_b) {
                        goto is_a_corner;
                       } else
                        if( p[pixel[12]] < c_b) {
                         if( p[pixel[13]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                      } else
                       if( p[pixel[12]] < c_b) {
                        if( p[pixel[13]] < c_b) {
                         if( p[pixel[14]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[12]] < c_b) {
                       if( p[pixel[13]] < c_b) {
                        if( p[pixel[14]] < c_b) {
                         if( p[pixel[15]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                if( p[pixel[10]] > cb) {
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         if( p[pixel[9]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else if( p[pixel[10]] < c_b) {
                 if( p[pixel[7]] < c_b) {
                  if( p[pixel[8]] < c_b) {
                   if( p[pixel[9]] < c_b) {
                    if( p[pixel[11]] < c_b) {
                     if( p[pixel[12]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[5]] < c_b) {
                        if( p[pixel[4]] < c_b) {
                         goto is_a_corner;
                        } else
                         if( p[pixel[13]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                       } else
                        if( p[pixel[13]] < c_b) {
                         if( p[pixel[14]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                      } else
                       if( p[pixel[13]] < c_b) {
                        if( p[pixel[14]] < c_b) {
                         if( p[pixel[15]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
              } else if( p[pixel[2]] < c_b) {
               if( p[pixel[9]] > cb) {
                if( p[pixel[10]] > cb) {
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else if( p[pixel[9]] < c_b) {
                if( p[pixel[7]] < c_b) {
                 if( p[pixel[8]] < c_b) {
                  if( p[pixel[10]] < c_b) {
                   if( p[pixel[6]] < c_b) {
                    if( p[pixel[5]] < c_b) {
                     if( p[pixel[4]] < c_b) {
                      if( p[pixel[3]] < c_b) {
                       goto is_a_corner;
                      } else
                       if( p[pixel[11]] < c_b) {
                        if( p[pixel[12]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[11]] < c_b) {
                       if( p[pixel[12]] < c_b) {
                        if( p[pixel[13]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[11]] < c_b) {
                      if( p[pixel[12]] < c_b) {
                       if( p[pixel[13]] < c_b) {
                        if( p[pixel[14]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[11]] < c_b) {
                     if( p[pixel[12]] < c_b) {
                      if( p[pixel[13]] < c_b) {
                       if( p[pixel[14]] < c_b) {
                        if( p[pixel[15]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               if( p[pixel[9]] > cb) {
                if( p[pixel[10]] > cb) {
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        if( p[pixel[8]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else if( p[pixel[9]] < c_b) {
                if( p[pixel[7]] < c_b) {
                 if( p[pixel[8]] < c_b) {
                  if( p[pixel[10]] < c_b) {
                   if( p[pixel[11]] < c_b) {
                    if( p[pixel[6]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[4]] < c_b) {
                       if( p[pixel[3]] < c_b) {
                        goto is_a_corner;
                       } else
                        if( p[pixel[12]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                      } else
                       if( p[pixel[12]] < c_b) {
                        if( p[pixel[13]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[12]] < c_b) {
                       if( p[pixel[13]] < c_b) {
                        if( p[pixel[14]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[12]] < c_b) {
                      if( p[pixel[13]] < c_b) {
                       if( p[pixel[14]] < c_b) {
                        if( p[pixel[15]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
             } else if( p[pixel[1]] < c_b) {
              if( p[pixel[8]] > cb) {
               if( p[pixel[9]] > cb) {
                if( p[pixel[10]] > cb) {
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[2]] > cb) {
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else if( p[pixel[8]] < c_b) {
               if( p[pixel[7]] < c_b) {
                if( p[pixel[9]] < c_b) {
                 if( p[pixel[6]] < c_b) {
                  if( p[pixel[5]] < c_b) {
                   if( p[pixel[4]] < c_b) {
                    if( p[pixel[3]] < c_b) {
                     if( p[pixel[2]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[10]] < c_b) {
                       if( p[pixel[11]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[10]] < c_b) {
                      if( p[pixel[11]] < c_b) {
                       if( p[pixel[12]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[10]] < c_b) {
                     if( p[pixel[11]] < c_b) {
                      if( p[pixel[12]] < c_b) {
                       if( p[pixel[13]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[10]] < c_b) {
                    if( p[pixel[11]] < c_b) {
                     if( p[pixel[12]] < c_b) {
                      if( p[pixel[13]] < c_b) {
                       if( p[pixel[14]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[10]] < c_b) {
                   if( p[pixel[11]] < c_b) {
                    if( p[pixel[12]] < c_b) {
                     if( p[pixel[13]] < c_b) {
                      if( p[pixel[14]] < c_b) {
                       if( p[pixel[15]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               goto is_not_a_corner;
             } else
              if( p[pixel[8]] > cb) {
               if( p[pixel[9]] > cb) {
                if( p[pixel[10]] > cb) {
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[15]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[2]] > cb) {
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[7]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else if( p[pixel[8]] < c_b) {
               if( p[pixel[7]] < c_b) {
                if( p[pixel[9]] < c_b) {
                 if( p[pixel[10]] < c_b) {
                  if( p[pixel[6]] < c_b) {
                   if( p[pixel[5]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[3]] < c_b) {
                      if( p[pixel[2]] < c_b) {
                       goto is_a_corner;
                      } else
                       if( p[pixel[11]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[11]] < c_b) {
                       if( p[pixel[12]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[11]] < c_b) {
                      if( p[pixel[12]] < c_b) {
                       if( p[pixel[13]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[11]] < c_b) {
                     if( p[pixel[12]] < c_b) {
                      if( p[pixel[13]] < c_b) {
                       if( p[pixel[14]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[11]] < c_b) {
                    if( p[pixel[12]] < c_b) {
                     if( p[pixel[13]] < c_b) {
                      if( p[pixel[14]] < c_b) {
                       if( p[pixel[15]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               goto is_not_a_corner;
            } else if( p[pixel[0]] < c_b) {
             if( p[pixel[1]] > cb) {
              if( p[pixel[8]] > cb) {
               if( p[pixel[7]] > cb) {
                if( p[pixel[9]] > cb) {
                 if( p[pixel[6]] > cb) {
                  if( p[pixel[5]] > cb) {
                   if( p[pixel[4]] > cb) {
                    if( p[pixel[3]] > cb) {
                     if( p[pixel[2]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[10]] > cb) {
                       if( p[pixel[11]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[10]] > cb) {
                      if( p[pixel[11]] > cb) {
                       if( p[pixel[12]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[10]] > cb) {
                     if( p[pixel[11]] > cb) {
                      if( p[pixel[12]] > cb) {
                       if( p[pixel[13]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[10]] > cb) {
                    if( p[pixel[11]] > cb) {
                     if( p[pixel[12]] > cb) {
                      if( p[pixel[13]] > cb) {
                       if( p[pixel[14]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[10]] > cb) {
                   if( p[pixel[11]] > cb) {
                    if( p[pixel[12]] > cb) {
                     if( p[pixel[13]] > cb) {
                      if( p[pixel[14]] > cb) {
                       if( p[pixel[15]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else if( p[pixel[8]] < c_b) {
               if( p[pixel[9]] < c_b) {
                if( p[pixel[10]] < c_b) {
                 if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[2]] < c_b) {
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               goto is_not_a_corner;
             } else if( p[pixel[1]] < c_b) {
              if( p[pixel[2]] > cb) {
               if( p[pixel[9]] > cb) {
                if( p[pixel[7]] > cb) {
                 if( p[pixel[8]] > cb) {
                  if( p[pixel[10]] > cb) {
                   if( p[pixel[6]] > cb) {
                    if( p[pixel[5]] > cb) {
                     if( p[pixel[4]] > cb) {
                      if( p[pixel[3]] > cb) {
                       goto is_a_corner;
                      } else
                       if( p[pixel[11]] > cb) {
                        if( p[pixel[12]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[11]] > cb) {
                       if( p[pixel[12]] > cb) {
                        if( p[pixel[13]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[11]] > cb) {
                      if( p[pixel[12]] > cb) {
                       if( p[pixel[13]] > cb) {
                        if( p[pixel[14]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[11]] > cb) {
                     if( p[pixel[12]] > cb) {
                      if( p[pixel[13]] > cb) {
                       if( p[pixel[14]] > cb) {
                        if( p[pixel[15]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else if( p[pixel[9]] < c_b) {
                if( p[pixel[10]] < c_b) {
                 if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else if( p[pixel[2]] < c_b) {
               if( p[pixel[3]] > cb) {
                if( p[pixel[10]] > cb) {
                 if( p[pixel[7]] > cb) {
                  if( p[pixel[8]] > cb) {
                   if( p[pixel[9]] > cb) {
                    if( p[pixel[11]] > cb) {
                     if( p[pixel[6]] > cb) {
                      if( p[pixel[5]] > cb) {
                       if( p[pixel[4]] > cb) {
                        goto is_a_corner;
                       } else
                        if( p[pixel[12]] > cb) {
                         if( p[pixel[13]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                      } else
                       if( p[pixel[12]] > cb) {
                        if( p[pixel[13]] > cb) {
                         if( p[pixel[14]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[12]] > cb) {
                       if( p[pixel[13]] > cb) {
                        if( p[pixel[14]] > cb) {
                         if( p[pixel[15]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else if( p[pixel[10]] < c_b) {
                 if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else if( p[pixel[3]] < c_b) {
                if( p[pixel[4]] > cb) {
                 if( p[pixel[13]] > cb) {
                  if( p[pixel[7]] > cb) {
                   if( p[pixel[8]] > cb) {
                    if( p[pixel[9]] > cb) {
                     if( p[pixel[10]] > cb) {
                      if( p[pixel[11]] > cb) {
                       if( p[pixel[12]] > cb) {
                        if( p[pixel[6]] > cb) {
                         if( p[pixel[5]] > cb) {
                          goto is_a_corner;
                         } else
                          if( p[pixel[14]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                        } else
                         if( p[pixel[14]] > cb) {
                          if( p[pixel[15]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else if( p[pixel[13]] < c_b) {
                  if( p[pixel[11]] > cb) {
                   if( p[pixel[5]] > cb) {
                    if( p[pixel[6]] > cb) {
                     if( p[pixel[7]] > cb) {
                      if( p[pixel[8]] > cb) {
                       if( p[pixel[9]] > cb) {
                        if( p[pixel[10]] > cb) {
                         if( p[pixel[12]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else if( p[pixel[11]] < c_b) {
                   if( p[pixel[12]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          if( p[pixel[10]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          if( p[pixel[10]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  if( p[pixel[5]] > cb) {
                   if( p[pixel[6]] > cb) {
                    if( p[pixel[7]] > cb) {
                     if( p[pixel[8]] > cb) {
                      if( p[pixel[9]] > cb) {
                       if( p[pixel[10]] > cb) {
                        if( p[pixel[11]] > cb) {
                         if( p[pixel[12]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else if( p[pixel[4]] < c_b) {
                 if( p[pixel[5]] > cb) {
                  if( p[pixel[14]] > cb) {
                   if( p[pixel[7]] > cb) {
                    if( p[pixel[8]] > cb) {
                     if( p[pixel[9]] > cb) {
                      if( p[pixel[10]] > cb) {
                       if( p[pixel[11]] > cb) {
                        if( p[pixel[12]] > cb) {
                         if( p[pixel[13]] > cb) {
                          if( p[pixel[6]] > cb) {
                           goto is_a_corner;
                          } else
                           if( p[pixel[15]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else if( p[pixel[14]] < c_b) {
                   if( p[pixel[12]] > cb) {
                    if( p[pixel[6]] > cb) {
                     if( p[pixel[7]] > cb) {
                      if( p[pixel[8]] > cb) {
                       if( p[pixel[9]] > cb) {
                        if( p[pixel[10]] > cb) {
                         if( p[pixel[11]] > cb) {
                          if( p[pixel[13]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else if( p[pixel[12]] < c_b) {
                    if( p[pixel[13]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          if( p[pixel[10]] < c_b) {
                           if( p[pixel[11]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   if( p[pixel[6]] > cb) {
                    if( p[pixel[7]] > cb) {
                     if( p[pixel[8]] > cb) {
                      if( p[pixel[9]] > cb) {
                       if( p[pixel[10]] > cb) {
                        if( p[pixel[11]] > cb) {
                         if( p[pixel[12]] > cb) {
                          if( p[pixel[13]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else if( p[pixel[5]] < c_b) {
                  if( p[pixel[6]] > cb) {
                   if( p[pixel[15]] < c_b) {
                    if( p[pixel[13]] > cb) {
                     if( p[pixel[7]] > cb) {
                      if( p[pixel[8]] > cb) {
                       if( p[pixel[9]] > cb) {
                        if( p[pixel[10]] > cb) {
                         if( p[pixel[11]] > cb) {
                          if( p[pixel[12]] > cb) {
                           if( p[pixel[14]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else if( p[pixel[13]] < c_b) {
                     if( p[pixel[14]] < c_b) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    if( p[pixel[7]] > cb) {
                     if( p[pixel[8]] > cb) {
                      if( p[pixel[9]] > cb) {
                       if( p[pixel[10]] > cb) {
                        if( p[pixel[11]] > cb) {
                         if( p[pixel[12]] > cb) {
                          if( p[pixel[13]] > cb) {
                           if( p[pixel[14]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else if( p[pixel[6]] < c_b) {
                   if( p[pixel[7]] > cb) {
                    if( p[pixel[14]] > cb) {
                     if( p[pixel[8]] > cb) {
                      if( p[pixel[9]] > cb) {
                       if( p[pixel[10]] > cb) {
                        if( p[pixel[11]] > cb) {
                         if( p[pixel[12]] > cb) {
                          if( p[pixel[13]] > cb) {
                           if( p[pixel[15]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else if( p[pixel[7]] < c_b) {
                    if( p[pixel[8]] < c_b) {
                     goto is_a_corner;
                    } else
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[13]] > cb) {
                    if( p[pixel[7]] > cb) {
                     if( p[pixel[8]] > cb) {
                      if( p[pixel[9]] > cb) {
                       if( p[pixel[10]] > cb) {
                        if( p[pixel[11]] > cb) {
                         if( p[pixel[12]] > cb) {
                          if( p[pixel[14]] > cb) {
                           if( p[pixel[15]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[12]] > cb) {
                   if( p[pixel[7]] > cb) {
                    if( p[pixel[8]] > cb) {
                     if( p[pixel[9]] > cb) {
                      if( p[pixel[10]] > cb) {
                       if( p[pixel[11]] > cb) {
                        if( p[pixel[13]] > cb) {
                         if( p[pixel[14]] > cb) {
                          if( p[pixel[6]] > cb) {
                           goto is_a_corner;
                          } else
                           if( p[pixel[15]] > cb) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          if( p[pixel[10]] < c_b) {
                           if( p[pixel[11]] < c_b) {
                            goto is_a_corner;
                           } else
                            goto is_not_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 if( p[pixel[11]] > cb) {
                  if( p[pixel[7]] > cb) {
                   if( p[pixel[8]] > cb) {
                    if( p[pixel[9]] > cb) {
                     if( p[pixel[10]] > cb) {
                      if( p[pixel[12]] > cb) {
                       if( p[pixel[13]] > cb) {
                        if( p[pixel[6]] > cb) {
                         if( p[pixel[5]] > cb) {
                          goto is_a_corner;
                         } else
                          if( p[pixel[14]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                        } else
                         if( p[pixel[14]] > cb) {
                          if( p[pixel[15]] > cb) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          if( p[pixel[10]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          if( p[pixel[10]] < c_b) {
                           goto is_a_corner;
                          } else
                           goto is_not_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
               } else
                if( p[pixel[10]] > cb) {
                 if( p[pixel[7]] > cb) {
                  if( p[pixel[8]] > cb) {
                   if( p[pixel[9]] > cb) {
                    if( p[pixel[11]] > cb) {
                     if( p[pixel[12]] > cb) {
                      if( p[pixel[6]] > cb) {
                       if( p[pixel[5]] > cb) {
                        if( p[pixel[4]] > cb) {
                         goto is_a_corner;
                        } else
                         if( p[pixel[13]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                       } else
                        if( p[pixel[13]] > cb) {
                         if( p[pixel[14]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                      } else
                       if( p[pixel[13]] > cb) {
                        if( p[pixel[14]] > cb) {
                         if( p[pixel[15]] > cb) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else if( p[pixel[10]] < c_b) {
                 if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         if( p[pixel[9]] < c_b) {
                          goto is_a_corner;
                         } else
                          goto is_not_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
              } else
               if( p[pixel[9]] > cb) {
                if( p[pixel[7]] > cb) {
                 if( p[pixel[8]] > cb) {
                  if( p[pixel[10]] > cb) {
                   if( p[pixel[11]] > cb) {
                    if( p[pixel[6]] > cb) {
                     if( p[pixel[5]] > cb) {
                      if( p[pixel[4]] > cb) {
                       if( p[pixel[3]] > cb) {
                        goto is_a_corner;
                       } else
                        if( p[pixel[12]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                      } else
                       if( p[pixel[12]] > cb) {
                        if( p[pixel[13]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[12]] > cb) {
                       if( p[pixel[13]] > cb) {
                        if( p[pixel[14]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[12]] > cb) {
                      if( p[pixel[13]] > cb) {
                       if( p[pixel[14]] > cb) {
                        if( p[pixel[15]] > cb) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else if( p[pixel[9]] < c_b) {
                if( p[pixel[10]] < c_b) {
                 if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        if( p[pixel[8]] < c_b) {
                         goto is_a_corner;
                        } else
                         goto is_not_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
             } else
              if( p[pixel[8]] > cb) {
               if( p[pixel[7]] > cb) {
                if( p[pixel[9]] > cb) {
                 if( p[pixel[10]] > cb) {
                  if( p[pixel[6]] > cb) {
                   if( p[pixel[5]] > cb) {
                    if( p[pixel[4]] > cb) {
                     if( p[pixel[3]] > cb) {
                      if( p[pixel[2]] > cb) {
                       goto is_a_corner;
                      } else
                       if( p[pixel[11]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                     } else
                      if( p[pixel[11]] > cb) {
                       if( p[pixel[12]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[11]] > cb) {
                      if( p[pixel[12]] > cb) {
                       if( p[pixel[13]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[11]] > cb) {
                     if( p[pixel[12]] > cb) {
                      if( p[pixel[13]] > cb) {
                       if( p[pixel[14]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[11]] > cb) {
                    if( p[pixel[12]] > cb) {
                     if( p[pixel[13]] > cb) {
                      if( p[pixel[14]] > cb) {
                       if( p[pixel[15]] > cb) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else if( p[pixel[8]] < c_b) {
               if( p[pixel[9]] < c_b) {
                if( p[pixel[10]] < c_b) {
                 if( p[pixel[11]] < c_b) {
                  if( p[pixel[12]] < c_b) {
                   if( p[pixel[13]] < c_b) {
                    if( p[pixel[14]] < c_b) {
                     if( p[pixel[15]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[2]] < c_b) {
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[4]] < c_b) {
                     if( p[pixel[5]] < c_b) {
                      if( p[pixel[6]] < c_b) {
                       if( p[pixel[7]] < c_b) {
                        goto is_a_corner;
                       } else
                        goto is_not_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               goto is_not_a_corner;
            } else
             if( p[pixel[7]] > cb) {
              if( p[pixel[8]] > cb) {
               if( p[pixel[9]] > cb) {
                if( p[pixel[6]] > cb) {
                 if( p[pixel[5]] > cb) {
                  if( p[pixel[4]] > cb) {
                   if( p[pixel[3]] > cb) {
                    if( p[pixel[2]] > cb) {
                     if( p[pixel[1]] > cb) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[10]] > cb) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[10]] > cb) {
                      if( p[pixel[11]] > cb) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[10]] > cb) {
                     if( p[pixel[11]] > cb) {
                      if( p[pixel[12]] > cb) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[10]] > cb) {
                    if( p[pixel[11]] > cb) {
                     if( p[pixel[12]] > cb) {
                      if( p[pixel[13]] > cb) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[10]] > cb) {
                   if( p[pixel[11]] > cb) {
                    if( p[pixel[12]] > cb) {
                     if( p[pixel[13]] > cb) {
                      if( p[pixel[14]] > cb) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 if( p[pixel[10]] > cb) {
                  if( p[pixel[11]] > cb) {
                   if( p[pixel[12]] > cb) {
                    if( p[pixel[13]] > cb) {
                     if( p[pixel[14]] > cb) {
                      if( p[pixel[15]] > cb) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               goto is_not_a_corner;
             } else if( p[pixel[7]] < c_b) {
              if( p[pixel[8]] < c_b) {
               if( p[pixel[9]] < c_b) {
                if( p[pixel[6]] < c_b) {
                 if( p[pixel[5]] < c_b) {
                  if( p[pixel[4]] < c_b) {
                   if( p[pixel[3]] < c_b) {
                    if( p[pixel[2]] < c_b) {
                     if( p[pixel[1]] < c_b) {
                      goto is_a_corner;
                     } else
                      if( p[pixel[10]] < c_b) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                    } else
                     if( p[pixel[10]] < c_b) {
                      if( p[pixel[11]] < c_b) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                   } else
                    if( p[pixel[10]] < c_b) {
                     if( p[pixel[11]] < c_b) {
                      if( p[pixel[12]] < c_b) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                  } else
                   if( p[pixel[10]] < c_b) {
                    if( p[pixel[11]] < c_b) {
                     if( p[pixel[12]] < c_b) {
                      if( p[pixel[13]] < c_b) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                 } else
                  if( p[pixel[10]] < c_b) {
                   if( p[pixel[11]] < c_b) {
                    if( p[pixel[12]] < c_b) {
                     if( p[pixel[13]] < c_b) {
                      if( p[pixel[14]] < c_b) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                } else
                 if( p[pixel[10]] < c_b) {
                  if( p[pixel[11]] < c_b) {
                   if( p[pixel[12]] < c_b) {
                    if( p[pixel[13]] < c_b) {
                     if( p[pixel[14]] < c_b) {
                      if( p[pixel[15]] < c_b) {
                       goto is_a_corner;
                      } else
                       goto is_not_a_corner;
                     } else
                      goto is_not_a_corner;
                    } else
                     goto is_not_a_corner;
                   } else
                    goto is_not_a_corner;
                  } else
                   goto is_not_a_corner;
                 } else
                  goto is_not_a_corner;
               } else
                goto is_not_a_corner;
              } else
               goto is_not_a_corner;
             } else
              goto is_not_a_corner;

            is_a_corner:
                bmin=b;
                goto end_if;

            is_not_a_corner:
                bmax=b;
                goto end_if;

            end_if:

            if(bmin == bmax - 1 || bmin == bmax)
                return bmin;
            b = (bmin + bmax) / 2;
        }
	}
	
	template <typename T>
	__always_inline int FAST9<T>::cornerTest(const T* p, T c, unsigned char b) const {
        int cb = c + b;
        int c_b= c - b;
            
        if(p[pixel[0]] > cb) {
         if(p[pixel[1]] > cb) {
          if(p[pixel[2]] > cb) {
           if(p[pixel[3]] > cb) {
            if(p[pixel[4]] > cb) {
             if(p[pixel[5]] > cb) {
              if(p[pixel[6]] > cb) {
               if(p[pixel[7]] > cb) {
                if(p[pixel[8]] > cb) {
                 return 0x108; //pixel 0 - 8
                } else
                 if(p[pixel[15]] > cb) {
                  return 0x1f7; //pixel 15 - 7
                 } else
                  return 0;
               } else if(p[pixel[7]] < c_b) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1e6; //pixel 14 - 6
                 } else
                  return 0;
                } else if(p[pixel[14]] < c_b) {
                 if(p[pixel[8]] < c_b) {
                  if(p[pixel[9]] < c_b) {
                   if(p[pixel[10]] < c_b) {
                    if(p[pixel[11]] < c_b) {
                     if(p[pixel[12]] < c_b) {
                      if(p[pixel[13]] < c_b) {
                       if(p[pixel[15]] < c_b) {
                        return 0x07f; //pixel 7 - 15
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1e6; //pixel 14 - 6
                 } else
                  return 0;
                } else
                 return 0;
              } else if(p[pixel[6]] < c_b) {
               if(p[pixel[15]] > cb) {
                if(p[pixel[13]] > cb) {
                 if(p[pixel[14]] > cb) {
                  return 0x1d5; //pixel 13 - 5
                 } else
                  return 0;
                } else if(p[pixel[13]] < c_b) {
                 if(p[pixel[7]] < c_b) {
                  if(p[pixel[8]] < c_b) {
                   if(p[pixel[9]] < c_b) {
                    if(p[pixel[10]] < c_b) {
                     if(p[pixel[11]] < c_b) {
                      if(p[pixel[12]] < c_b) {
                       if(p[pixel[14]] < c_b) {
                        return 0x06e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                if(p[pixel[7]] < c_b) {
                 if(p[pixel[8]] < c_b) {
                  if(p[pixel[9]] < c_b) {
                   if(p[pixel[10]] < c_b) {
                    if(p[pixel[11]] < c_b) {
                     if(p[pixel[12]] < c_b) {
                      if(p[pixel[13]] < c_b) {
                       if(p[pixel[14]] < c_b) {
                        return 0x06e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1d5; //pixel 13 - 5
                 } else
                  return 0;
                } else
                 return 0;
               } else if(p[pixel[13]] < c_b) {
                if(p[pixel[7]] < c_b) {
                 if(p[pixel[8]] < c_b) {
                  if(p[pixel[9]] < c_b) {
                   if(p[pixel[10]] < c_b) {
                    if(p[pixel[11]] < c_b) {
                     if(p[pixel[12]] < c_b) {
                      if(p[pixel[14]] < c_b) {
                       if(p[pixel[15]] < c_b) {
                        return 0x07f; //pixel 7 - 15
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else if(p[pixel[5]] < c_b) {
              if(p[pixel[14]] > cb) {
               if(p[pixel[12]] > cb) {
                if(p[pixel[13]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1c4; //pixel 12 - 4
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      if(p[pixel[10]] > cb) {
                       if(p[pixel[11]] > cb) {
                        return 0x16e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 return 0;
               } else if(p[pixel[12]] < c_b) {
                if(p[pixel[6]] < c_b) {
                 if(p[pixel[7]] < c_b) {
                  if(p[pixel[8]] < c_b) {
                   if(p[pixel[9]] < c_b) {
                    if(p[pixel[10]] < c_b) {
                     if(p[pixel[11]] < c_b) {
                      if(p[pixel[13]] < c_b) {
                       return 0x05d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else if(p[pixel[14]] < c_b) {
               if(p[pixel[7]] < c_b) {
                if(p[pixel[8]] < c_b) {
                 if(p[pixel[9]] < c_b) {
                  if(p[pixel[10]] < c_b) {
                   if(p[pixel[11]] < c_b) {
                    if(p[pixel[12]] < c_b) {
                     if(p[pixel[13]] < c_b) {
                      if(p[pixel[6]] < c_b) {
                       return 0x05e; //pixel 5 - 14
                      } else
                       if(p[pixel[15]] < c_b) {
                        return 0x07f; //pixel 7 - 15
                       } else
                        return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               if(p[pixel[6]] < c_b) {
                if(p[pixel[7]] < c_b) {
                 if(p[pixel[8]] < c_b) {
                  if(p[pixel[9]] < c_b) {
                   if(p[pixel[10]] < c_b) {
                    if(p[pixel[11]] < c_b) {
                     if(p[pixel[12]] < c_b) {
                      if(p[pixel[13]] < c_b) {
                       return 0x05d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1c4; //pixel 12 - 4
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      if(p[pixel[10]] > cb) {
                       if(p[pixel[11]] > cb) {
                        return 0x16e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 return 0;
               } else
                return 0;
              } else if(p[pixel[12]] < c_b) {
               if(p[pixel[7]] < c_b) {
                if(p[pixel[8]] < c_b) {
                 if(p[pixel[9]] < c_b) {
                  if(p[pixel[10]] < c_b) {
                   if(p[pixel[11]] < c_b) {
                    if(p[pixel[13]] < c_b) {
                     if(p[pixel[14]] < c_b) {
                      if(p[pixel[6]] < c_b) {
                       return 0x06e; //pixel 6 - 14
                      } else
                       if(p[pixel[15]] < c_b) {
                        return 0x07f; //pixel 7 - 15
                       } else
                        return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else if(p[pixel[4]] < c_b) {
             if(p[pixel[13]] > cb) {
              if(p[pixel[11]] > cb) {
               if(p[pixel[12]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1b3; //pixel 11 - 3
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      if(p[pixel[10]] > cb) {
                       return 0x16e; //pixel 6 - 14
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      if(p[pixel[10]] > cb) {
                       return 0x15d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                return 0;
              } else if(p[pixel[11]] < c_b) {
               if(p[pixel[5]] < c_b) {
                if(p[pixel[6]] < c_b) {
                 if(p[pixel[7]] < c_b) {
                  if(p[pixel[8]] < c_b) {
                   if(p[pixel[9]] < c_b) {
                    if(p[pixel[10]] < c_b) {
                     if(p[pixel[12]] < c_b) {
                      return 0x04c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else if(p[pixel[13]] < c_b) {
              if(p[pixel[7]] < c_b) {
               if(p[pixel[8]] < c_b) {
                if(p[pixel[9]] < c_b) {
                 if(p[pixel[10]] < c_b) {
                  if(p[pixel[11]] < c_b) {
                   if(p[pixel[12]] < c_b) {
                    if(p[pixel[6]] < c_b) {
                     if(p[pixel[5]] < c_b) {
                      return 0x04d; //pixel 4 - 13
                     } else
                      if(p[pixel[14]] < c_b) {
                       return 0x06e; //pixel 6 - 14
                      } else
                       return 0;
                    } else
                     if(p[pixel[14]] < c_b) {
                      if(p[pixel[15]] < c_b) {
                       return 0x07f; //pixel 7 - 15
                      } else
                       return 0;
                     } else
                      return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              if(p[pixel[5]] < c_b) {
               if(p[pixel[6]] < c_b) {
                if(p[pixel[7]] < c_b) {
                 if(p[pixel[8]] < c_b) {
                  if(p[pixel[9]] < c_b) {
                   if(p[pixel[10]] < c_b) {
                    if(p[pixel[11]] < c_b) {
                     if(p[pixel[12]] < c_b) {
                      return 0x04c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1b3; //pixel 11 - 3
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      if(p[pixel[10]] > cb) {
                       return 0x16e; //pixel 6 - 14
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      if(p[pixel[10]] > cb) {
                       return 0x15d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                return 0;
              } else
               return 0;
             } else if(p[pixel[11]] < c_b) {
              if(p[pixel[7]] < c_b) {
               if(p[pixel[8]] < c_b) {
                if(p[pixel[9]] < c_b) {
                 if(p[pixel[10]] < c_b) {
                  if(p[pixel[12]] < c_b) {
                   if(p[pixel[13]] < c_b) {
                    if(p[pixel[6]] < c_b) {
                     if(p[pixel[5]] < c_b) {
                      return 0x05d; //pixel 5 - 13
                     } else
                      if(p[pixel[14]] < c_b) {
                       return 0x06e; //pixel 6 - 14
                      } else
                       return 0;
                    } else
                     if(p[pixel[14]] < c_b) {
                      if(p[pixel[15]] < c_b) {
                       return 0x07f; //pixel 7 - 15
                      } else
                       return 0;
                     } else
                      return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
           } else if(p[pixel[3]] < c_b) {
            if(p[pixel[10]] > cb) {
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1a2; //pixel 10 - 2
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      return 0x16e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      return 0x15d; //pixel 5 - 13
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      return 0x14c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               return 0;
             } else
              return 0;
            } else if(p[pixel[10]] < c_b) {
             if(p[pixel[7]] < c_b) {
              if(p[pixel[8]] < c_b) {
               if(p[pixel[9]] < c_b) {
                if(p[pixel[11]] < c_b) {
                 if(p[pixel[6]] < c_b) {
                  if(p[pixel[5]] < c_b) {
                   if(p[pixel[4]] < c_b) {
                    return 0x03b; //pixel 3 - 11
                   } else
                    if(p[pixel[12]] < c_b) {
                     if(p[pixel[13]] < c_b) {
                      return 0x05d; //pixel 5 - 13
                     } else
                      return 0;
                    } else
                     return 0;
                  } else
                   if(p[pixel[12]] < c_b) {
                    if(p[pixel[13]] < c_b) {
                     if(p[pixel[14]] < c_b) {
                      return 0x06e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  if(p[pixel[12]] < c_b) {
                   if(p[pixel[13]] < c_b) {
                    if(p[pixel[14]] < c_b) {
                     if(p[pixel[15]] < c_b) {
                      return 0x07f; //pixel 7 - 15
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            if(p[pixel[10]] > cb) {
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x1a2; //pixel 10 - 2
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      return 0x16e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      return 0x15d; //pixel 5 - 13
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     if(p[pixel[9]] > cb) {
                      return 0x14c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               return 0;
             } else
              return 0;
            } else if(p[pixel[10]] < c_b) {
             if(p[pixel[7]] < c_b) {
              if(p[pixel[8]] < c_b) {
               if(p[pixel[9]] < c_b) {
                if(p[pixel[11]] < c_b) {
                 if(p[pixel[12]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[5]] < c_b) {
                    if(p[pixel[4]] < c_b) {
                     return 0x04c; //pixel 4 - 12
                    } else
                     if(p[pixel[13]] < c_b) {
                      return 0x05d; //pixel 5 - 13
                     } else
                      return 0;
                   } else
                    if(p[pixel[13]] < c_b) {
                     if(p[pixel[14]] < c_b) {
                      return 0x06e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                  } else
                   if(p[pixel[13]] < c_b) {
                    if(p[pixel[14]] < c_b) {
                     if(p[pixel[15]] < c_b) {
                      return 0x07f; //pixel 7 - 15
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
          } else if(p[pixel[2]] < c_b) {
           if(p[pixel[9]] > cb) {
            if(p[pixel[10]] > cb) {
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x191; //pixel 9 - 1
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x16e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x15d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x14c; //pixel 4 - 12
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x13b; //pixel 3 - 11
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              return 0;
            } else
             return 0;
           } else if(p[pixel[9]] < c_b) {
            if(p[pixel[7]] < c_b) {
             if(p[pixel[8]] < c_b) {
              if(p[pixel[10]] < c_b) {
               if(p[pixel[6]] < c_b) {
                if(p[pixel[5]] < c_b) {
                 if(p[pixel[4]] < c_b) {
                  if(p[pixel[3]] < c_b) {
                   return 0x02a; //pixel 2 - 10
                  } else
                   if(p[pixel[11]] < c_b) {
                    if(p[pixel[12]] < c_b) {
                     return 0x04c; //pixel 4 - 12
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  if(p[pixel[11]] < c_b) {
                   if(p[pixel[12]] < c_b) {
                    if(p[pixel[13]] < c_b) {
                     return 0x05d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[11]] < c_b) {
                  if(p[pixel[12]] < c_b) {
                   if(p[pixel[13]] < c_b) {
                    if(p[pixel[14]] < c_b) {
                     return 0x06e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[11]] < c_b) {
                 if(p[pixel[12]] < c_b) {
                  if(p[pixel[13]] < c_b) {
                   if(p[pixel[14]] < c_b) {
                    if(p[pixel[15]] < c_b) {
                     return 0x07f; //pixel 7 - 15
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            return 0;
          } else
           if(p[pixel[9]] > cb) {
            if(p[pixel[10]] > cb) {
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x191; //pixel 9 - 1
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x16e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x15d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x14c; //pixel 4 - 12
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    if(p[pixel[8]] > cb) {
                     return 0x13b; //pixel 3 - 11
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              return 0;
            } else
             return 0;
           } else if(p[pixel[9]] < c_b) {
            if(p[pixel[7]] < c_b) {
             if(p[pixel[8]] < c_b) {
              if(p[pixel[10]] < c_b) {
               if(p[pixel[11]] < c_b) {
                if(p[pixel[6]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[4]] < c_b) {
                   if(p[pixel[3]] < c_b) {
                    return 0x03b; //pixel 3 - 11
                   } else
                    if(p[pixel[12]] < c_b) {
                     return 0x04c; //pixel 4 - 12
                    } else
                     return 0;
                  } else
                   if(p[pixel[12]] < c_b) {
                    if(p[pixel[13]] < c_b) {
                     return 0x05d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  if(p[pixel[12]] < c_b) {
                   if(p[pixel[13]] < c_b) {
                    if(p[pixel[14]] < c_b) {
                     return 0x06e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[12]] < c_b) {
                  if(p[pixel[13]] < c_b) {
                   if(p[pixel[14]] < c_b) {
                    if(p[pixel[15]] < c_b) {
                     return 0x07f; //pixel 7 - 15
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            return 0;
         } else if(p[pixel[1]] < c_b) {
          if(p[pixel[8]] > cb) {
           if(p[pixel[9]] > cb) {
            if(p[pixel[10]] > cb) {
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x180; //pixel 8 - 0
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x16e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x15d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x14c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x13b; //pixel 3 - 11
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[2]] > cb) {
               if(p[pixel[3]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x12a; //pixel 2 - 10
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             return 0;
           } else
            return 0;
          } else if(p[pixel[8]] < c_b) {
           if(p[pixel[7]] < c_b) {
            if(p[pixel[9]] < c_b) {
             if(p[pixel[6]] < c_b) {
              if(p[pixel[5]] < c_b) {
               if(p[pixel[4]] < c_b) {
                if(p[pixel[3]] < c_b) {
                 if(p[pixel[2]] < c_b) {
                  return 0x019; //pixel 1 - 9
                 } else
                  if(p[pixel[10]] < c_b) {
                   if(p[pixel[11]] < c_b) {
                    return 0x03b; //pixel 3 - 11
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[10]] < c_b) {
                  if(p[pixel[11]] < c_b) {
                   if(p[pixel[12]] < c_b) {
                    return 0x04c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[10]] < c_b) {
                 if(p[pixel[11]] < c_b) {
                  if(p[pixel[12]] < c_b) {
                   if(p[pixel[13]] < c_b) {
                    return 0x05d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[10]] < c_b) {
                if(p[pixel[11]] < c_b) {
                 if(p[pixel[12]] < c_b) {
                  if(p[pixel[13]] < c_b) {
                   if(p[pixel[14]] < c_b) {
                    return 0x06e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[10]] < c_b) {
               if(p[pixel[11]] < c_b) {
                if(p[pixel[12]] < c_b) {
                 if(p[pixel[13]] < c_b) {
                  if(p[pixel[14]] < c_b) {
                   if(p[pixel[15]] < c_b) {
                    return 0x07f; //pixel 7 - 15
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             return 0;
           } else
            return 0;
          } else
           return 0;
         } else
          if(p[pixel[8]] > cb) {
           if(p[pixel[9]] > cb) {
            if(p[pixel[10]] > cb) {
             if(p[pixel[11]] > cb) {
              if(p[pixel[12]] > cb) {
               if(p[pixel[13]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[15]] > cb) {
                  return 0x180; //pixel 8 - 0
                 } else
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x16e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x15d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x14c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x13b; //pixel 3 - 11
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[2]] > cb) {
               if(p[pixel[3]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[7]] > cb) {
                    return 0x12a; //pixel 2 - 10
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             return 0;
           } else
            return 0;
          } else if(p[pixel[8]] < c_b) {
           if(p[pixel[7]] < c_b) {
            if(p[pixel[9]] < c_b) {
             if(p[pixel[10]] < c_b) {
              if(p[pixel[6]] < c_b) {
               if(p[pixel[5]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[3]] < c_b) {
                  if(p[pixel[2]] < c_b) {
                   return 0x02a; //pixel 2 - 10
                  } else
                   if(p[pixel[11]] < c_b) {
                    return 0x03b; //pixel 3 - 11
                   } else
                    return 0;
                 } else
                  if(p[pixel[11]] < c_b) {
                   if(p[pixel[12]] < c_b) {
                    return 0x04c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[11]] < c_b) {
                  if(p[pixel[12]] < c_b) {
                   if(p[pixel[13]] < c_b) {
                    return 0x05d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[11]] < c_b) {
                 if(p[pixel[12]] < c_b) {
                  if(p[pixel[13]] < c_b) {
                   if(p[pixel[14]] < c_b) {
                    return 0x06e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[11]] < c_b) {
                if(p[pixel[12]] < c_b) {
                 if(p[pixel[13]] < c_b) {
                  if(p[pixel[14]] < c_b) {
                   if(p[pixel[15]] < c_b) {
                    return 0x07f; //pixel 7 - 15
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            return 0;
          } else
           return 0;
        } else if(p[pixel[0]] < c_b) {
         if(p[pixel[1]] > cb) {
          if(p[pixel[8]] > cb) {
           if(p[pixel[7]] > cb) {
            if(p[pixel[9]] > cb) {
             if(p[pixel[6]] > cb) {
              if(p[pixel[5]] > cb) {
               if(p[pixel[4]] > cb) {
                if(p[pixel[3]] > cb) {
                 if(p[pixel[2]] > cb) {
                  return 0x119; //pixel 1 - 9
                 } else
                  if(p[pixel[10]] > cb) {
                   if(p[pixel[11]] > cb) {
                    return 0x13b; //pixel 3 - 11
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[10]] > cb) {
                  if(p[pixel[11]] > cb) {
                   if(p[pixel[12]] > cb) {
                    return 0x14c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[10]] > cb) {
                 if(p[pixel[11]] > cb) {
                  if(p[pixel[12]] > cb) {
                   if(p[pixel[13]] > cb) {
                    return 0x15d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[10]] > cb) {
                if(p[pixel[11]] > cb) {
                 if(p[pixel[12]] > cb) {
                  if(p[pixel[13]] > cb) {
                   if(p[pixel[14]] > cb) {
                    return 0x16e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[10]] > cb) {
               if(p[pixel[11]] > cb) {
                if(p[pixel[12]] > cb) {
                 if(p[pixel[13]] > cb) {
                  if(p[pixel[14]] > cb) {
                   if(p[pixel[15]] > cb) {
                    return 0x17f; //pixel 7 - 15
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             return 0;
           } else
            return 0;
          } else if(p[pixel[8]] < c_b) {
           if(p[pixel[9]] < c_b) {
            if(p[pixel[10]] < c_b) {
             if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x080; //pixel 8 - 0
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x06e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x05d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x04c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x03b; //pixel 3 - 11
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[2]] < c_b) {
               if(p[pixel[3]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x02a; //pixel 2 - 10
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             return 0;
           } else
            return 0;
          } else
           return 0;
         } else if(p[pixel[1]] < c_b) {
          if(p[pixel[2]] > cb) {
           if(p[pixel[9]] > cb) {
            if(p[pixel[7]] > cb) {
             if(p[pixel[8]] > cb) {
              if(p[pixel[10]] > cb) {
               if(p[pixel[6]] > cb) {
                if(p[pixel[5]] > cb) {
                 if(p[pixel[4]] > cb) {
                  if(p[pixel[3]] > cb) {
                   return 0x12a; //pixel 2 - 10
                  } else
                   if(p[pixel[11]] > cb) {
                    if(p[pixel[12]] > cb) {
                     return 0x14c; //pixel 4 - 12
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  if(p[pixel[11]] > cb) {
                   if(p[pixel[12]] > cb) {
                    if(p[pixel[13]] > cb) {
                     return 0x15d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[11]] > cb) {
                  if(p[pixel[12]] > cb) {
                   if(p[pixel[13]] > cb) {
                    if(p[pixel[14]] > cb) {
                     return 0x16e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[11]] > cb) {
                 if(p[pixel[12]] > cb) {
                  if(p[pixel[13]] > cb) {
                   if(p[pixel[14]] > cb) {
                    if(p[pixel[15]] > cb) {
                     return 0x17f; //pixel 7 - 15
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
           } else if(p[pixel[9]] < c_b) {
            if(p[pixel[10]] < c_b) {
             if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x091; //pixel 9 - 1
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x06e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x05d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x04c; //pixel 4 - 12
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x03b; //pixel 3 - 11
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            return 0;
          } else if(p[pixel[2]] < c_b) {
           if(p[pixel[3]] > cb) {
            if(p[pixel[10]] > cb) {
             if(p[pixel[7]] > cb) {
              if(p[pixel[8]] > cb) {
               if(p[pixel[9]] > cb) {
                if(p[pixel[11]] > cb) {
                 if(p[pixel[6]] > cb) {
                  if(p[pixel[5]] > cb) {
                   if(p[pixel[4]] > cb) {
                    return 0x13b; //pixel 3 - 11
                   } else
                    if(p[pixel[12]] > cb) {
                     if(p[pixel[13]] > cb) {
                      return 0x15d; //pixel 5 - 13
                     } else
                      return 0;
                    } else
                     return 0;
                  } else
                   if(p[pixel[12]] > cb) {
                    if(p[pixel[13]] > cb) {
                     if(p[pixel[14]] > cb) {
                      return 0x16e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  if(p[pixel[12]] > cb) {
                   if(p[pixel[13]] > cb) {
                    if(p[pixel[14]] > cb) {
                     if(p[pixel[15]] > cb) {
                      return 0x17f; //pixel 7 - 15
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
            } else if(p[pixel[10]] < c_b) {
             if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0a2; //pixel 10 - 2
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      return 0x06e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      return 0x05d; //pixel 5 - 13
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      return 0x04c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
           } else if(p[pixel[3]] < c_b) {
            if(p[pixel[4]] > cb) {
             if(p[pixel[13]] > cb) {
              if(p[pixel[7]] > cb) {
               if(p[pixel[8]] > cb) {
                if(p[pixel[9]] > cb) {
                 if(p[pixel[10]] > cb) {
                  if(p[pixel[11]] > cb) {
                   if(p[pixel[12]] > cb) {
                    if(p[pixel[6]] > cb) {
                     if(p[pixel[5]] > cb) {
                      return 0x14d; //pixel 4 - 13
                     } else
                      if(p[pixel[14]] > cb) {
                       return 0x16e; //pixel 6 - 14
                      } else
                       return 0;
                    } else
                     if(p[pixel[14]] > cb) {
                      if(p[pixel[15]] > cb) {
                       return 0x17f; //pixel 7 - 15
                      } else
                       return 0;
                     } else
                      return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else if(p[pixel[13]] < c_b) {
              if(p[pixel[11]] > cb) {
               if(p[pixel[5]] > cb) {
                if(p[pixel[6]] > cb) {
                 if(p[pixel[7]] > cb) {
                  if(p[pixel[8]] > cb) {
                   if(p[pixel[9]] > cb) {
                    if(p[pixel[10]] > cb) {
                     if(p[pixel[12]] > cb) {
                      return 0x14c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else if(p[pixel[11]] < c_b) {
               if(p[pixel[12]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0b3; //pixel 11 - 3
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      if(p[pixel[10]] < c_b) {
                       return 0x06e; //pixel 6 - 14
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      if(p[pixel[10]] < c_b) {
                       return 0x05d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              if(p[pixel[5]] > cb) {
               if(p[pixel[6]] > cb) {
                if(p[pixel[7]] > cb) {
                 if(p[pixel[8]] > cb) {
                  if(p[pixel[9]] > cb) {
                   if(p[pixel[10]] > cb) {
                    if(p[pixel[11]] > cb) {
                     if(p[pixel[12]] > cb) {
                      return 0x14c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else if(p[pixel[4]] < c_b) {
             if(p[pixel[5]] > cb) {
              if(p[pixel[14]] > cb) {
               if(p[pixel[7]] > cb) {
                if(p[pixel[8]] > cb) {
                 if(p[pixel[9]] > cb) {
                  if(p[pixel[10]] > cb) {
                   if(p[pixel[11]] > cb) {
                    if(p[pixel[12]] > cb) {
                     if(p[pixel[13]] > cb) {
                      if(p[pixel[6]] > cb) {
                       return 0x15e; //pixel 5 - 14
                      } else
                       if(p[pixel[15]] > cb) {
                        return 0x17f; //pixel 7 - 15
                       } else
                        return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else if(p[pixel[14]] < c_b) {
               if(p[pixel[12]] > cb) {
                if(p[pixel[6]] > cb) {
                 if(p[pixel[7]] > cb) {
                  if(p[pixel[8]] > cb) {
                   if(p[pixel[9]] > cb) {
                    if(p[pixel[10]] > cb) {
                     if(p[pixel[11]] > cb) {
                      if(p[pixel[13]] > cb) {
                       return 0x15d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else if(p[pixel[12]] < c_b) {
                if(p[pixel[13]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0c4; //pixel 12 - 4
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      if(p[pixel[10]] < c_b) {
                       if(p[pixel[11]] < c_b) {
                        return 0x06e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               if(p[pixel[6]] > cb) {
                if(p[pixel[7]] > cb) {
                 if(p[pixel[8]] > cb) {
                  if(p[pixel[9]] > cb) {
                   if(p[pixel[10]] > cb) {
                    if(p[pixel[11]] > cb) {
                     if(p[pixel[12]] > cb) {
                      if(p[pixel[13]] > cb) {
                       return 0x15d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else if(p[pixel[5]] < c_b) {
              if(p[pixel[6]] > cb) {
               if(p[pixel[15]] < c_b) {
                if(p[pixel[13]] > cb) {
                 if(p[pixel[7]] > cb) {
                  if(p[pixel[8]] > cb) {
                   if(p[pixel[9]] > cb) {
                    if(p[pixel[10]] > cb) {
                     if(p[pixel[11]] > cb) {
                      if(p[pixel[12]] > cb) {
                       if(p[pixel[14]] > cb) {
                        return 0x16e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else if(p[pixel[13]] < c_b) {
                 if(p[pixel[14]] < c_b) {
                  return 0x0d5; //pixel 13 - 5
                 } else
                  return 0;
                } else
                 return 0;
               } else
                if(p[pixel[7]] > cb) {
                 if(p[pixel[8]] > cb) {
                  if(p[pixel[9]] > cb) {
                   if(p[pixel[10]] > cb) {
                    if(p[pixel[11]] > cb) {
                     if(p[pixel[12]] > cb) {
                      if(p[pixel[13]] > cb) {
                       if(p[pixel[14]] > cb) {
                        return 0x16e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else if(p[pixel[6]] < c_b) {
               if(p[pixel[7]] > cb) {
                if(p[pixel[14]] > cb) {
                 if(p[pixel[8]] > cb) {
                  if(p[pixel[9]] > cb) {
                   if(p[pixel[10]] > cb) {
                    if(p[pixel[11]] > cb) {
                     if(p[pixel[12]] > cb) {
                      if(p[pixel[13]] > cb) {
                       if(p[pixel[15]] > cb) {
                        return 0x17f; //pixel 7 - 15
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0e6; //pixel 14 - 6
                 } else
                  return 0;
                } else
                 return 0;
               } else if(p[pixel[7]] < c_b) {
                if(p[pixel[8]] < c_b) {
                 return 0x008; //pixel 0 - 8
                } else
                 if(p[pixel[15]] < c_b) {
                  return 0x0f7; //pixel 15 - 7
                 } else
                  return 0;
               } else
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0e6; //pixel 14 - 6
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[13]] > cb) {
                if(p[pixel[7]] > cb) {
                 if(p[pixel[8]] > cb) {
                  if(p[pixel[9]] > cb) {
                   if(p[pixel[10]] > cb) {
                    if(p[pixel[11]] > cb) {
                     if(p[pixel[12]] > cb) {
                      if(p[pixel[14]] > cb) {
                       if(p[pixel[15]] > cb) {
                        return 0x17f; //pixel 7 - 15
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0d5; //pixel 13 - 5
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[12]] > cb) {
               if(p[pixel[7]] > cb) {
                if(p[pixel[8]] > cb) {
                 if(p[pixel[9]] > cb) {
                  if(p[pixel[10]] > cb) {
                   if(p[pixel[11]] > cb) {
                    if(p[pixel[13]] > cb) {
                     if(p[pixel[14]] > cb) {
                      if(p[pixel[6]] > cb) {
                       return 0x16e; //pixel 6 - 14
                      } else
                       if(p[pixel[15]] > cb) {
                        return 0x17f; //pixel 7 - 15
                       } else
                        return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0c4; //pixel 12 - 4
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      if(p[pixel[10]] < c_b) {
                       if(p[pixel[11]] < c_b) {
                        return 0x06e; //pixel 6 - 14
                       } else
                        return 0;
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             if(p[pixel[11]] > cb) {
              if(p[pixel[7]] > cb) {
               if(p[pixel[8]] > cb) {
                if(p[pixel[9]] > cb) {
                 if(p[pixel[10]] > cb) {
                  if(p[pixel[12]] > cb) {
                   if(p[pixel[13]] > cb) {
                    if(p[pixel[6]] > cb) {
                     if(p[pixel[5]] > cb) {
                      return 0x15d; //pixel 5 - 13
                     } else
                      if(p[pixel[14]] > cb) {
                       return 0x16e; //pixel 6 - 14
                      } else
                       return 0;
                    } else
                     if(p[pixel[14]] > cb) {
                      if(p[pixel[15]] > cb) {
                       return 0x17f; //pixel 7 - 15
                      } else
                       return 0;
                     } else
                      return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0b3; //pixel 11 - 3
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      if(p[pixel[10]] < c_b) {
                       return 0x06e; //pixel 6 - 14
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      if(p[pixel[10]] < c_b) {
                       return 0x05d; //pixel 5 - 13
                      } else
                       return 0;
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
           } else
            if(p[pixel[10]] > cb) {
             if(p[pixel[7]] > cb) {
              if(p[pixel[8]] > cb) {
               if(p[pixel[9]] > cb) {
                if(p[pixel[11]] > cb) {
                 if(p[pixel[12]] > cb) {
                  if(p[pixel[6]] > cb) {
                   if(p[pixel[5]] > cb) {
                    if(p[pixel[4]] > cb) {
                     return 0x14c; //pixel 4 - 12
                    } else
                     if(p[pixel[13]] > cb) {
                      return 0x15d; //pixel 5 - 13
                     } else
                      return 0;
                   } else
                    if(p[pixel[13]] > cb) {
                     if(p[pixel[14]] > cb) {
                      return 0x16e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                  } else
                   if(p[pixel[13]] > cb) {
                    if(p[pixel[14]] > cb) {
                     if(p[pixel[15]] > cb) {
                      return 0x17f; //pixel 7 - 15
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
            } else if(p[pixel[10]] < c_b) {
             if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x0a2; //pixel 10 - 2
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      return 0x06e; //pixel 6 - 14
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      return 0x05d; //pixel 5 - 13
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     if(p[pixel[9]] < c_b) {
                      return 0x04c; //pixel 4 - 12
                     } else
                      return 0;
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
          } else
           if(p[pixel[9]] > cb) {
            if(p[pixel[7]] > cb) {
             if(p[pixel[8]] > cb) {
              if(p[pixel[10]] > cb) {
               if(p[pixel[11]] > cb) {
                if(p[pixel[6]] > cb) {
                 if(p[pixel[5]] > cb) {
                  if(p[pixel[4]] > cb) {
                   if(p[pixel[3]] > cb) {
                    return 0x13b; //pixel 3 - 11
                   } else
                    if(p[pixel[12]] > cb) {
                     return 0x14c; //pixel 4 - 12
                    } else
                     return 0;
                  } else
                   if(p[pixel[12]] > cb) {
                    if(p[pixel[13]] > cb) {
                     return 0x15d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                 } else
                  if(p[pixel[12]] > cb) {
                   if(p[pixel[13]] > cb) {
                    if(p[pixel[14]] > cb) {
                     return 0x16e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[12]] > cb) {
                  if(p[pixel[13]] > cb) {
                   if(p[pixel[14]] > cb) {
                    if(p[pixel[15]] > cb) {
                     return 0x17f; //pixel 7 - 15
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
            } else
             return 0;
           } else if(p[pixel[9]] < c_b) {
            if(p[pixel[10]] < c_b) {
             if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x091; //pixel 9 - 1
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x06e; //pixel 6 - 14
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x05d; //pixel 5 - 13
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x04c; //pixel 4 - 12
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    if(p[pixel[8]] < c_b) {
                     return 0x03b; //pixel 3 - 11
                    } else
                     return 0;
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            return 0;
         } else
          if(p[pixel[8]] > cb) {
           if(p[pixel[7]] > cb) {
            if(p[pixel[9]] > cb) {
             if(p[pixel[10]] > cb) {
              if(p[pixel[6]] > cb) {
               if(p[pixel[5]] > cb) {
                if(p[pixel[4]] > cb) {
                 if(p[pixel[3]] > cb) {
                  if(p[pixel[2]] > cb) {
                   return 0x12a; //pixel 2 - 10
                  } else
                   if(p[pixel[11]] > cb) {
                    return 0x13b; //pixel 3 - 11
                   } else
                    return 0;
                 } else
                  if(p[pixel[11]] > cb) {
                   if(p[pixel[12]] > cb) {
                    return 0x14c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[11]] > cb) {
                  if(p[pixel[12]] > cb) {
                   if(p[pixel[13]] > cb) {
                    return 0x15d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[11]] > cb) {
                 if(p[pixel[12]] > cb) {
                  if(p[pixel[13]] > cb) {
                   if(p[pixel[14]] > cb) {
                    return 0x16e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[11]] > cb) {
                if(p[pixel[12]] > cb) {
                 if(p[pixel[13]] > cb) {
                  if(p[pixel[14]] > cb) {
                   if(p[pixel[15]] > cb) {
                    return 0x17f; //pixel 7 - 15
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              return 0;
            } else
             return 0;
           } else
            return 0;
          } else if(p[pixel[8]] < c_b) {
           if(p[pixel[9]] < c_b) {
            if(p[pixel[10]] < c_b) {
             if(p[pixel[11]] < c_b) {
              if(p[pixel[12]] < c_b) {
               if(p[pixel[13]] < c_b) {
                if(p[pixel[14]] < c_b) {
                 if(p[pixel[15]] < c_b) {
                  return 0x080; //pixel 8 - 0
                 } else
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x06e; //pixel 6 - 14
                   } else
                    return 0;
                  } else
                   return 0;
                } else
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x05d; //pixel 5 - 13
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x04c; //pixel 4 - 12
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[3]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x03b; //pixel 3 - 11
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[2]] < c_b) {
               if(p[pixel[3]] < c_b) {
                if(p[pixel[4]] < c_b) {
                 if(p[pixel[5]] < c_b) {
                  if(p[pixel[6]] < c_b) {
                   if(p[pixel[7]] < c_b) {
                    return 0x02a; //pixel 2 - 10
                   } else
                    return 0;
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             return 0;
           } else
            return 0;
          } else
           return 0;
        } else
         if(p[pixel[7]] > cb) {
          if(p[pixel[8]] > cb) {
           if(p[pixel[9]] > cb) {
            if(p[pixel[6]] > cb) {
             if(p[pixel[5]] > cb) {
              if(p[pixel[4]] > cb) {
               if(p[pixel[3]] > cb) {
                if(p[pixel[2]] > cb) {
                 if(p[pixel[1]] > cb) {
                  return 0x119; //pixel 1 - 9
                 } else
                  if(p[pixel[10]] > cb) {
                   return 0x12a; //pixel 2 - 10
                  } else
                   return 0;
                } else
                 if(p[pixel[10]] > cb) {
                  if(p[pixel[11]] > cb) {
                   return 0x13b; //pixel 3 - 11
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[10]] > cb) {
                 if(p[pixel[11]] > cb) {
                  if(p[pixel[12]] > cb) {
                   return 0x14c; //pixel 4 - 12
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[10]] > cb) {
                if(p[pixel[11]] > cb) {
                 if(p[pixel[12]] > cb) {
                  if(p[pixel[13]] > cb) {
                   return 0x15d; //pixel 5 - 13
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[10]] > cb) {
               if(p[pixel[11]] > cb) {
                if(p[pixel[12]] > cb) {
                 if(p[pixel[13]] > cb) {
                  if(p[pixel[14]] > cb) {
                   return 0x16e; //pixel 6 - 14
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             if(p[pixel[10]] > cb) {
              if(p[pixel[11]] > cb) {
               if(p[pixel[12]] > cb) {
                if(p[pixel[13]] > cb) {
                 if(p[pixel[14]] > cb) {
                  if(p[pixel[15]] > cb) {
                   return 0x17f; //pixel 7 - 15
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
           } else
            return 0;
          } else
           return 0;
         } else if(p[pixel[7]] < c_b) {
          if(p[pixel[8]] < c_b) {
           if(p[pixel[9]] < c_b) {
            if(p[pixel[6]] < c_b) {
             if(p[pixel[5]] < c_b) {
              if(p[pixel[4]] < c_b) {
               if(p[pixel[3]] < c_b) {
                if(p[pixel[2]] < c_b) {
                 if(p[pixel[1]] < c_b) {
                  return 0x019; //pixel 1 - 9
                 } else
                  if(p[pixel[10]] < c_b) {
                   return 0x02a; //pixel 2 - 10
                  } else
                   return 0;
                } else
                 if(p[pixel[10]] < c_b) {
                  if(p[pixel[11]] < c_b) {
                   return 0x03b; //pixel 3 - 11
                  } else
                   return 0;
                 } else
                  return 0;
               } else
                if(p[pixel[10]] < c_b) {
                 if(p[pixel[11]] < c_b) {
                  if(p[pixel[12]] < c_b) {
                   return 0x04c; //pixel 4 - 12
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
              } else
               if(p[pixel[10]] < c_b) {
                if(p[pixel[11]] < c_b) {
                 if(p[pixel[12]] < c_b) {
                  if(p[pixel[13]] < c_b) {
                   return 0x05d; //pixel 5 - 13
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
             } else
              if(p[pixel[10]] < c_b) {
               if(p[pixel[11]] < c_b) {
                if(p[pixel[12]] < c_b) {
                 if(p[pixel[13]] < c_b) {
                  if(p[pixel[14]] < c_b) {
                   return 0x06e; //pixel 6 - 14
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
            } else
             if(p[pixel[10]] < c_b) {
              if(p[pixel[11]] < c_b) {
               if(p[pixel[12]] < c_b) {
                if(p[pixel[13]] < c_b) {
                 if(p[pixel[14]] < c_b) {
                  if(p[pixel[15]] < c_b) {
                   return 0x07f; //pixel 7 - 15
                  } else
                   return 0;
                 } else
                  return 0;
                } else
                 return 0;
               } else
                return 0;
              } else
               return 0;
             } else
              return 0;
           } else
            return 0;
          } else
           return 0;
         } else
          return 0;
          
        return 1;
    }
}

#endif
