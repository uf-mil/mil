/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_CENSUSWINDOW_H
#define SPARSESTEREO_CENSUSWINDOW_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "hammingdistance.h"
#include "simd.h"

//#define SPARSESTEREO_NO_POPCNT //Defined by CMake

#if defined(SPARSESTEREO_NO_POPCNT)
#define SPARSESTEREO_SSE_OPTIMIZE
#endif

namespace sparsestereo {
	// Matches over a set of census windows
	template <int SIZE>
	class CensusWindow {
	public:
		CensusWindow() {
#ifdef SPARSESTEREO_SSE_OPTIMIZE
			lookupTablePtr = hammingDist.getLookupTable();
			lookupTableVec = SIMD::scalar4((int)hammingDist.getLookupTable());
			zero = SIMD::scalar16(0);
#endif
		}
		
		void setReferenceImage(const cv::Mat_<unsigned int>& image) {
			refImage = image;
		}
		
		void setComparisonImage(const cv::Mat_<unsigned int>& image) {
			compImage = image;
		}
		
		const cv::Mat_<unsigned int>& getReferenceImage() const {return refImage;}
		const cv::Mat_<unsigned int>& getComparisonImage() const {return compImage;}
		const int getWindowSize() const {return SIZE;}
		
		// Sets the position of the reference window
		void setReferencePoint(const cv::Point2i& point);
		
		// Performs a window matching using census transformed images
		__always_inline short match(cv::Point2i point) const;
		
	private:
		v4si lookupTableVec;
		v4si refWindow[6];
		v16qi zero;
	
		const unsigned char* lookupTablePtr;
		HammingDistance hammingDist;
		cv::Mat_<unsigned int> refImage;
		cv::Mat_<unsigned int> compImage;
		cv::Point2i refPoint;
		
		// Stores a window in two SSE vectors
		void loadWindow(const cv::Mat_<unsigned int>& image, const cv::Point2i& point, v4si* dstWindow) const;
	} __attribute__ ((aligned (16)));
	
	template <int SIZE>
	void CensusWindow<SIZE>::loadWindow(const cv::Mat_<unsigned int>& image, const cv::Point2i& point, v4si* dstWindow) const {}
	
#ifdef SPARSESTEREO_SSE_OPTIMIZE
	template <>
	void CensusWindow<5>::loadWindow(const cv::Mat_<unsigned int>& image, const cv::Point2i& point, v4si* dstWindow) const {
		dstWindow[0] = (v4si)__builtin_ia32_loaddqu((char*)&image(point.y-2, point.x-2));
		dstWindow[1] = (v4si)__builtin_ia32_loaddqu((char*)&image(point.y-1, point.x-2));
		dstWindow[2] = (v4si)__builtin_ia32_loaddqu((char*)&image(point.y, point.x-2));
		dstWindow[3] = (v4si)__builtin_ia32_loaddqu((char*)&image(point.y+1, point.x-2));
		dstWindow[4] = (v4si)__builtin_ia32_loaddqu((char*)&image(point.y+2, point.x-2));
		
		// Unfortunately, the rest cannot be loaded aligned
		v4si buffer = {image(point.y-2, point.x+2), image(point.y-1, point.x+2), image(point.y, point.x+2), image(point.y+1, point.x+2)};
		dstWindow[5] = buffer;
	}
#endif
	
	template <int SIZE>
	void CensusWindow<SIZE>::setReferencePoint(const cv::Point2i& point) {
		refPoint = point;
	}
	
#ifdef SPARSESTEREO_SSE_OPTIMIZE
	template <>
	void CensusWindow<5>::setReferencePoint(const cv::Point2i& point) {
		loadWindow(refImage, point, refWindow);
	}
#endif
	
	template <int SIZE>
	__always_inline short CensusWindow<SIZE>::match(cv::Point2i point) const {
		int costs = 0;
		
#ifndef SPARSESTEREO_NO_POPCNT
		for(int y=-SIZE/2; y<=SIZE/2; y++) {
			unsigned long long* ptr1 = (unsigned long long*)&refImage(refPoint.y + y, refPoint.x -SIZE/2);
			unsigned long long* ptr2 = (unsigned long long*)&compImage(point.y + y, point.x -SIZE/2);
			
			for(int x=0; x<=SIZE/2;x++)
				costs += __builtin_popcountll(ptr1[x] ^ ptr2[x]);
		}
#else
		for(int y=-SIZE/2; y<=SIZE/2; y++)
			for(int x=-SIZE/2; x<=SIZE/2; x++)
				costs += hammingDist.calculate(refImage(refPoint.y + y, refPoint.x + x),
					compImage(point.y + y, point.x + x));
#endif
		
		return costs;
	}
	
#ifdef SPARSESTEREO_SSE_OPTIMIZE
#ifdef __LP64__
	// SSE2 optimized implementation for 64-bit systems.
	template <>
	__always_inline short CensusWindow<5>::match(cv::Point2i point) const {
		v8hi xorRes;
		unsigned int sum;
		
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y-2, point.x-2)), (v2di)refWindow[0]);
		sum = lookupTableVec[(unsigned short)SIMD::element8(xorRes, 0)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 1)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 2)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 3)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 4)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 5)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 6)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 7)];
			
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y-1, point.x-2)), (v2di)refWindow[1]);
		sum += lookupTableVec[(unsigned short)SIMD::element8(xorRes, 0)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 1)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 2)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 3)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 4)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 5)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 6)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 7)];
		
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y, point.x-2)), (v2di)refWindow[2]);
		sum += lookupTableVec[(unsigned short)SIMD::element8(xorRes, 0)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 1)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 2)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 3)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 4)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 5)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 6)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 7)];
		
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y+1, point.x-2)), (v2di)refWindow[3]);
		sum += lookupTableVec[(unsigned short)SIMD::element8(xorRes, 0)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 1)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 2)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 3)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 4)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 5)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 6)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 7)];
		
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y+2, point.x-2)), (v2di)refWindow[4]);
		sum += lookupTableVec[(unsigned short)SIMD::element8(xorRes, 0)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 1)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 2)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 3)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 4)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 5)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 6)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 7)];
		
		v4si buffer1 = {compImage(point.y-2, point.x+2), compImage(point.y-1, point.x+2), compImage(point.y, point.x+2), compImage(point.y+1, point.x+2)};
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)buffer1, (v2di)refWindow[5]);
		sum += lookupTableVec[(unsigned short)SIMD::element8(xorRes, 0)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 1)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 2)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 3)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 4)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 5)]
			+ lookupTableVec[(unsigned short)SIMD::element8(xorRes, 6)] + lookupTableVec[(unsigned short)SIMD::element8(xorRes, 7)];
		
		unsigned short lastXor = compImage(point.y+2, point.x+2) ^ refImage(refPoint.y + 2, refPoint.x+2);
		sum += lookupTablePtr[(unsigned short)lastXor] + lookupTablePtr[((unsigned short*)&lastXor)[1]];
		
		return sum;
	}

#else

	// SSE2 optimized implementation for 32-bit systems.
	template <>
	__always_inline short CensusWindow<5>::match(cv::Point2i point) const {
		
		v8hi xorRes;
		v4si lookupPtr;
		unsigned int sum;
		
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y-2, point.x-2)), (v2di)refWindow[0]);
		lookupPtr = (v4si)__builtin_ia32_punpcklwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum = *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		lookupPtr = (v4si)__builtin_ia32_punpckhwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
			
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y-1, point.x-2)), (v2di)refWindow[1]);
		lookupPtr = (v4si)__builtin_ia32_punpcklwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		lookupPtr = (v4si)__builtin_ia32_punpckhwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
			
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y, point.x-2)), (v2di)refWindow[2]);
		lookupPtr = (v4si)__builtin_ia32_punpcklwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		lookupPtr = (v4si)__builtin_ia32_punpckhwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
			
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y+1, point.x-2)), (v2di)refWindow[3]);
		lookupPtr = (v4si)__builtin_ia32_punpcklwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		lookupPtr = (v4si)__builtin_ia32_punpckhwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
			
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)__builtin_ia32_loaddqu((char*)&compImage(point.y+2, point.x-2)), (v2di)refWindow[4]);
		lookupPtr = (v4si)__builtin_ia32_punpcklwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		lookupPtr = (v4si)__builtin_ia32_punpckhwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
			
		v4si buffer1 = {compImage(point.y-2, point.x+2), compImage(point.y-1, point.x+2), compImage(point.y, point.x+2), compImage(point.y+1, point.x+2)};
		xorRes = (v8hi)__builtin_ia32_pxor128((v2di)buffer1, (v2di)refWindow[5]);
		lookupPtr = (v4si)__builtin_ia32_punpcklwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		lookupPtr = (v4si)__builtin_ia32_punpckhwd128(xorRes, (v8hi)zero) + lookupTableVec;
		sum += *((unsigned char*)SIMD::element4(lookupPtr, 0)) + *((unsigned char*)SIMD::element4(lookupPtr, 1))
			+ *((unsigned char*)SIMD::element4(lookupPtr, 2)) + *((unsigned char*)SIMD::element4(lookupPtr, 3)); 
		
		unsigned short lastXor = compImage(point.y+2, point.x+2) ^ refImage(refPoint.y + 2, refPoint.x+2);
		sum += lookupTablePtr[(unsigned short)lastXor] + lookupTablePtr[((unsigned short*)&lastXor)[1]];
		
		return sum;
	}
#endif
#endif
}

#endif
