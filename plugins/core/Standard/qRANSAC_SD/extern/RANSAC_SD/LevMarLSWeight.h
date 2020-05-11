#ifndef LEVMARLSWEIGHT_HEADER
#define LEVMARLSWEIGHT_HEADER
#include <algorithm>
#include <cmath>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE LevMarLSWeight
{
public:
	float Weigh(float d) const { return d; }
	template< unsigned int N >
	void DerivWeigh(float d, float *gradient) const {}
};

#endif
