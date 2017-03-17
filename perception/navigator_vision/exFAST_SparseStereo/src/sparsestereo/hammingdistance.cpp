/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "hammingdistance.h"

namespace sparsestereo {
	bool HammingDistance::tableInitialized = false;
	unsigned char HammingDistance::lookupTable[1U<<16] __attribute__ ((aligned (16)));
	
	HammingDistance::HammingDistance() {
		if(!tableInitialized)
			initTable();
	}
	
	void HammingDistance::initTable() {
		// This is threadsafe because we always write the same values and
		// set the initialization flag at the end
		for(unsigned int i=0; i < sizeof(lookupTable)/sizeof(*lookupTable); i++)
			lookupTable[i] = countNotPrecomputed(i);
		tableInitialized = true;
	}
}