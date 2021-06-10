#ifndef FLATNORMALTHRESHPOINTCOMPATIBILITYFUNC_HEADER
#define FLATNORMALTHRESHPOINTCOMPATIBILITYFUNC_HEADER
#include <GfxTL/MathHelper.h>

class FlatNormalThreshPointCompatibilityFunc
{
public:
	FlatNormalThreshPointCompatibilityFunc() {}
	FlatNormalThreshPointCompatibilityFunc(float distThresh,
		float normalThresh)
	: m_distThresh(distThresh)
	, m_normalThresh(normalThresh)
	{}
	template< class ShapeT, class OctreeT >
	bool operator()(const ShapeT &shape, const OctreeT &oct,
		unsigned int i) const
	{
		Vec3f n;
		float distance = shape.DistanceAndNormal(oct.at(i), &n);
		if(distance < m_distThresh)
			return fabs(n.dot(oct.at(i).normal)) >= m_normalThresh;
		return false;
	}
	bool operator()(float distance, float normalDeviation) const
	{
		return distance < m_distThresh
			&& fabs(normalDeviation) >= m_normalThresh;
	}
	float DistanceThresh() const { return m_distThresh; }
	float NormalThresh() const { return m_normalThresh; }

private:
	float m_distThresh;
	float m_normalThresh;
};

#endif
