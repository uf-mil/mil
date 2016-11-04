/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "census-inl.h"

namespace sparsestereo {
	using namespace std;
	using namespace cv;

	template <>
	void Census::transform5x5<char>(const Mat_<char>& input, Mat_<unsigned int>* output) {
		transform5x5SSE(input, output);
	}
	
	void Census::transform5x5SSE(const Mat_<char>& input, Mat_<unsigned int>* output) {
		// Predeclare required constants
		const v16qi const01 = SIMD::scalar16(0x01);

		// We skip one row at the beginning and end to avoid range checking
		for(int y=2; y<input.rows-2; y++)
			for(int x=0; x<input.cols; x+=16) {
				// Get 16 centoids
				v16qi centoid = (v16qi)_mm_load_si128((__m128i*)&(input)(y,x));
				
				// Row 5
				const char* rowPtr = &input(y+2, x-2);
				v16qi bitConst = const01;
				v16qi byte1 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x02
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x04
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Alinged
				bitConst += bitConst; // 0x08
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x10
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 4
				rowPtr -= input.step;
				bitConst += bitConst; // 0x20
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x40
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x80
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Aligned
				// Byte 2 starts
				bitConst = const01;
				v16qi byte2 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x02
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 3
				rowPtr -= input.step;
				bitConst += bitConst; // 0x04
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x08
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x10
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x20
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 2
				rowPtr -= input.step;
				bitConst += bitConst; // 0x40
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x80
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				// Byte 3 starts
				bitConst = const01;
				v16qi byte3 = __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Aligned
				bitConst += bitConst; // 0x02
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x04
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 1
				rowPtr -= input.step;
				bitConst += bitConst; // 0x08
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x10
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x20
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Aligned
				bitConst += bitConst; // 0x40
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x80
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
												
				storeSSEVec(byte3^byte3 /*0*/, byte3, byte2, byte1, (char*) &(*output)(y,x));
			}
	}
	
	__always_inline void Census::storeSSEVec(const v16qi& byte4, const v16qi& byte3, const v16qi& byte2, const v16qi& byte1, char* dst) {
		// Combine bytes to shorts
		v8hi high1 = (v8hi) __builtin_ia32_punpcklbw128(byte3, byte4);
		v8hi high2 = (v8hi) __builtin_ia32_punpckhbw128(byte3, byte4);
		v8hi low1 = (v8hi) __builtin_ia32_punpcklbw128(byte1, byte2);
		v8hi low2 = (v8hi) __builtin_ia32_punpckhbw128(byte1, byte2);
		
		// Combine shorts to ints
		__builtin_ia32_storedqu(dst, (v16qi)__builtin_ia32_punpcklwd128(low1, high1));
		__builtin_ia32_storedqu(dst + 4*4, (v16qi)__builtin_ia32_punpckhwd128(low1, high1));
		__builtin_ia32_storedqu(dst + 8*4, (v16qi)__builtin_ia32_punpcklwd128(low2, high2));
		__builtin_ia32_storedqu(dst + 12*4, (v16qi)__builtin_ia32_punpckhwd128(low2, high2));
	}
}
