#ifndef BASEPRIMITIVESHAPE_HEADER
#define BASEPRIMITIVESHAPE_HEADER
#include "PrimitiveShape.h"
#include "ScoreComputer.h"
#include <MiscLib/Random.h>
#include <GfxTL/MathHelper.h>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE BasePrimitiveShape
: public PrimitiveShape
{
protected:
	template< class ShapeT >
	unsigned int ConfidenceTests(unsigned int numTests, float epsilon,
		float normalThresh, float rms, const PointCloud &pc,
		const MiscLib::Vector< size_t > &indices) const
	{
		unsigned int numFailures = 0;
		// estimate shapes
		const unsigned int numSamples = ShapeT::RequiredSamples;
		if(numSamples >= indices.size())
			return numTests;
		MiscLib::Vector< ShapeT > shapes;
		for(unsigned int i = 0; i < numTests; ++i)
		{
			MiscLib::Vector< size_t > sampleIndices;
			for(unsigned int j = 0; j < numSamples; ++j)
			{
				size_t idx;
				do
				{
					idx = indices[MiscLib::rn_rand() % indices.size()];
				}
				while(std::find(sampleIndices.begin(), sampleIndices.end(), idx)
					!= sampleIndices.end());
				sampleIndices.push_back(idx);
			}
			MiscLib::Vector< Vec3f > samples(numSamples << 1);
			for(unsigned int j = 0; j < numSamples; ++j)
			{
				samples[j] = pc[sampleIndices[j]].pos;
				samples[j + numSamples] = pc[sampleIndices[j]].normal;
			}
			ShapeT shape;
			if(shape.Init(samples))
				shapes.push_back(shape);
			else
				++numFailures;
		}

		MiscLib::Vector< size_t > scores(shapes.size(), 0);
		//MiscLib::Vector< float > sse(shapes.size(), 0.f);
		Vec3f n;
		for(size_t i = 0; i < indices.size(); ++i)
		{
			for(size_t j = 0; j < shapes.size(); ++j)
			{
				//float d = shapes[j].Distance(pc[indices[i]].pos);
				//sse[j] += d * d;
				float d = shapes[j].DistanceAndNormal(pc[indices[i]].pos, &n);
				if(d > epsilon)
					continue;
				float dn = n.dot(pc[indices[i]].normal);
				if(fabs(dn) > normalThresh)
					++scores[j];
			}
		}

		for(size_t i = 0; i < /*sse.size()*/scores.size(); ++i)
		{
			if(scores[i] < .9f * indices.size())
			//if(fabs(std::sqrt(sse[i] / indices.size()) - rms) > 1e-2)
				++numFailures;
		}

		return numFailures;
	}
};

#endif
