#ifndef LOWSTRETCHTORUSPARAMETRIZATION_HEADER
#define LOWSTRETCHTORUSPARAMETRIZATION_HEADER
#include "Torus.h"
#include "basic.h"
#include <GfxTL/VectorXD.h>
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/AABox.h>
#include <GfxTL/Frame.h>
#include <utility>
#include <MiscLib/Vector.h>

class LowStretchTorusParametrization
{
public:
	LowStretchTorusParametrization(const Torus &torus);
	void Shape(const Torus &torus);
	const Torus &Shape() const { return *m_torus; }
	inline void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const;
	inline bool InSpace(float u, float v, Vec3f *p) const;
	inline bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const;
	void WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, bool *uwrap, bool *vwrap) const { *uwrap = false; *vwrap = false; }
	void WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< int > *componentImg,
		MiscLib::Vector< std::pair< int, size_t > > *labels) const;
	template< class IteratorT >
	void Optimize(IteratorT begin, IteratorT end, float epsilon);
	static size_t SerializedSize();
	void Serialize(std::ostream *o, bool binary) const;
	void Deserialize(std::istream *i, bool binary);

private:
	float MinorFrameRotation() const;
	float MajorFrameRotation() const;

private:
	const Torus *m_torus;
	GfxTL::HyperplaneCoordinateSystem< float, 3 > m_hcs;
	GfxTL::Frame< 2, float > m_minorFrame;
};

void LowStretchTorusParametrization::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	Vec3f s = p - m_torus->Center();
	float planex = s.dot(m_hcs[0].Data());
	float planey = s.dot(m_hcs[1].Data());
	param->first = std::atan2(planey, planex); // major angle
	GfxTL::Vector2Df minorVec, minorL;
	minorVec[1] = s.dot(m_torus->AxisDirection());
	minorVec[0] = std::sqrt(planex * planex + planey * planey) - m_torus->MajorRadius();
	float majorRadiusAngle = std::atan2(minorVec[1], minorVec[0]);
	m_minorFrame.ToLocal(minorVec, &minorL);
	param->second = std::atan2(minorL[1], minorL[0]); // minor angle
	if(m_torus->IsAppleShaped())
	{
		if(fabs(param->second) > m_torus->AppleCutOffAngle())
			param->second = GfxTL::Math< float >::Sign(param->second)
				* m_torus->AppleCutOffAngle();
	}
	// multiply major angle with radius
	float majorRadius = m_torus->MajorRadius()
		+ std::cos(majorRadiusAngle) * m_torus->MinorRadius();
	param->first = param->first * majorRadius;
	param->second *= m_torus->MinorRadius();
}

bool LowStretchTorusParametrization::InSpace(float u, float v, Vec3f *p) const
{
	float vangle = v / m_torus->MinorRadius();
	GfxTL::Vector2Df minorVec, minorL;
	minorL[0] = std::cos(vangle);
	minorL[1] = std::sin(vangle);
	m_minorFrame.ToGlobal(minorL, &minorVec);
	float majorRadiusAngle = std::atan2(minorVec[1], minorVec[0]);
	minorVec[0] = m_torus->MinorRadius() * minorVec[0] + m_torus->MajorRadius();
	minorVec[1] *= m_torus->MinorRadius();
	Vec3f pp = minorVec[0] * Vec3f(m_hcs[0].Data()) + minorVec[1] * m_torus->AxisDirection();
	GfxTL::Quaternion< float > q;
	float majorRadius = m_torus->MajorRadius()
		+ std::cos(majorRadiusAngle) * m_torus->MinorRadius();
	float uangle = u / majorRadius;
	q.RotationRad(uangle, m_torus->AxisDirection()[0], m_torus->AxisDirection()[1],
		m_torus->AxisDirection()[2]);
	q.Rotate(pp, p);
	*p += m_torus->Center();
	return true;
}

bool LowStretchTorusParametrization::InSpace(float u, float v, Vec3f *p,
	Vec3f *n) const
{
	float vangle = v / m_torus->MinorRadius();
	GfxTL::Vector2Df minorVec, minorL;
	minorL[0] = std::cos(vangle);
	minorL[1] = std::sin(vangle);
	m_minorFrame.ToGlobal(minorL, &minorVec);
	float majorRadiusAngle = std::atan2(minorVec[1], minorVec[0]);
	Vec3f nn = minorVec[0] * Vec3f(m_hcs[0].Data()) + minorVec[1] * m_torus->AxisDirection();
	minorVec[0] = m_torus->MinorRadius() * minorVec[0] + m_torus->MajorRadius();
	minorVec[1] *= m_torus->MinorRadius();
	Vec3f pp = minorVec[0] * Vec3f(m_hcs[0].Data()) + minorVec[1] * m_torus->AxisDirection();
	GfxTL::Quaternion< float > q;
	float majorRadius = m_torus->MajorRadius()
		+ std::cos(majorRadiusAngle) * m_torus->MinorRadius();
	float uangle = u / majorRadius;
	q.RotationRad(uangle, m_torus->AxisDirection()[0], m_torus->AxisDirection()[1],
		m_torus->AxisDirection()[2]);
	q.Rotate(pp, p);
	q.Rotate(nn, n);
	*p += m_torus->Center();
	return true;
}

template< class IteratorT >
void LowStretchTorusParametrization::Optimize(IteratorT begin, IteratorT end,
	float epsilon)
{
	// tries to find a rotation of the parametrization that does not cut
	// the points
	// find a rotation along the major radius (u-direction) and minor radius (v-direction)
	GfxTL::Frame< 3, float > nframe;
	nframe.FromNormal(m_torus->AxisDirection());
	MiscLib::Vector< float > uangles(end - begin), vangles(end - begin);
	size_t j = 0;
	for(IteratorT i = begin; i != end; ++i, ++j)
	{
		Vec3f s = *i - m_torus->Center();
		float planex = s.dot(nframe[0].Data());
		float planey = s.dot(nframe[1].Data());
		uangles[j] = std::atan2(planey, planex); // major angle
		GfxTL::Vector2Df minorVec;
		minorVec[1] = s.dot(m_torus->AxisDirection());
		minorVec[0] = std::sqrt(planex * planex + planey * planey) - m_torus->MajorRadius();
		vangles[j] = std::atan2(minorVec[1], minorVec[0]); // minor angle
		if(m_torus->IsAppleShaped())
		{
			if(fabs(uangles[j]) > m_torus->AppleCutOffAngle())
				uangles[j] = GfxTL::Math< float >::Sign(uangles[j])
					* m_torus->AppleCutOffAngle();
		}
	}
	std::sort(vangles.begin(), vangles.end());
	std::sort(uangles.begin(), uangles.end());
	// try to find a large gap along u direction
	float maxGap = uangles.front() + 2 * float(M_PI) - uangles.back();
	float lower = uangles.back(), upper = uangles.front() + 2 * float(M_PI);
	for(size_t i = 1; i < uangles.size(); ++i)
	{
		float gap = uangles[i] - uangles[i - 1];
		if(gap > maxGap)
		{
			maxGap = gap;
			lower = uangles[i - 1];
			upper = uangles[i];
		}
	}
	// rotate m_hcs into gap
	float rotationAngle = (lower + upper) / 2;
	nframe.RotateOnNormal(rotationAngle + float(M_PI));
	m_hcs[0] = nframe[0];
	m_hcs[1] = nframe[1];
	// try to find a large gap along v direction
	maxGap = vangles.front() + 2 * float(M_PI) - vangles.back();
	lower = vangles.back(); upper = vangles.front() + 2 * float(M_PI);
	for(size_t i = 1; i < vangles.size(); ++i)
	{
		float gap = vangles[i] - vangles[i - 1];
		if(gap > maxGap)
		{
			maxGap = gap;
			lower = vangles[i - 1];
			upper = vangles[i];
		}
	}
	rotationAngle = (lower + upper) / 2;
	m_minorFrame.Canonical();
	m_minorFrame.RotateFrame(rotationAngle + float(M_PI));
}

#endif
