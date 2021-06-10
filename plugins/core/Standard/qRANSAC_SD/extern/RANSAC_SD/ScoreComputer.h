#ifndef SCORECOMPUTER_HEADER
#define SCORECOMPUTER_HEADER
#include <GfxTL/NullClass.h>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include "PointCloud.h"

inline float weigh(const float d, const float eps)
{
	return std::exp(-d * d / (2.f / 9.f * eps * eps)); // gaussian weighting eps / 3 == sigma
	// return 1.f - (d / eps) * (d / eps);	// anti-quadratic weighting
	// return 1.f - d / eps;			// linear weighting
	//return d * d; // SSE
}

template< class BaseT >
class LocalScoreComputer
: public BaseT
{
public:
	template< class ShapeT >
	static void Score(const ShapeT &shape, const PointCloud &pc, float epsilon,
		float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end,
		std::pair< size_t, float > *score,
		MiscLib::Vector< size_t > *indices = NULL)
	{
		Vec3f n;
		size_t size = end - begin;
		for(size_t iter = 0; iter < size; ++iter)
		{
			float d = shape.DistanceAndNormal(pc[begin[iter]].pos, &n);
			float nd = n.dot(pc[begin[iter]].normal);
			if(d < epsilon && fabs(nd) > normalThresh)
			{
				++score->first;
				score->second += weigh(d,epsilon);
			}
		}
	}
};

#endif
