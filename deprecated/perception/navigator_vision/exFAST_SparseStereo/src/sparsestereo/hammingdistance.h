/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_HAMMINGDISTANCE_H
#define SPARSESTEREO_HAMMINGDISTANCE_H

namespace sparsestereo {
	// Calculates the bitwise hamming distance between two integers
	// or counts the number of set bits.
	// NOTE: Prefer popcnt instruction if available
	class HammingDistance {
	public:
		HammingDistance();
	
		// Calculates the hamming distance for 32-bit integers
		unsigned char calculate(unsigned int x, unsigned int y) const {
			return countBits(x^y);
		}
		
		// Calculates the hamming distance for 16-bit integers
		unsigned char calculate(unsigned short x, unsigned short y) const {
			return countBits((unsigned short)(x^y));
		}
		
		// Determines the number of set bits for a 64-bit integer
 		unsigned char countBits(unsigned long long x) const {
			// Count using precomputed table:
			int buf = lookupTable[x & 0xFFFFU];
			x = x >> 16;
			buf += lookupTable[x & 0xFFFFU];
			x = x >> 16;
			return buf + lookupTable[x & 0xFFFFU] + lookupTable[x >> 16];
		}
		
		// Determines the number of set bits for a 32-bit integer
 		unsigned char countBits(unsigned int x) const {
			// Count using precomputed table:
			return lookupTable[x & 0xFFFFU] +  lookupTable[x >> 16];
		}
		
		// Determines the number of set bits for a 16-bit integer
		unsigned char countBits(unsigned short x) const {
			return lookupTable[x];
		}
		
		// Returns a pointer to the static lookup table
		const unsigned char* getLookupTable() const {return lookupTable;}
		
	private:
		static bool tableInitialized;
		static unsigned char lookupTable[1U<<16];
	
		// Initializes the lookup table
		void initTable();
	
		// Methods for parallel bit counting algorithm
		unsigned int getBit(int i) {return 0x1U << i;}
		unsigned int getMask(int i) {
			return (((unsigned int)(-1)) / (getBit(getBit(i)) + 1U));
        }
		unsigned int countBlocks(int x, int level) {
		    return ((x) & getMask(level)) + (((x) >> (getBit(level))) & getMask(level));
		}
	
		// Performs a parallel bit counting without using the precomputed table
		unsigned char countNotPrecomputed(unsigned int i) {
			i = countBlocks(i, 0);
			i = countBlocks(i, 1);
			i = countBlocks(i, 2);
			i = countBlocks(i, 3);
			i = countBlocks(i, 4);
			// i = countBlocks(i, 5); //For 64 Bits
			return i;
		}
	};
}

#endif
