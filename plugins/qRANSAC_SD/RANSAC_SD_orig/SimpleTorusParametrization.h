#ifndef SIMPLETORUSPARAMETRIZATION_HEADER
#define SIMPLETORUSPARAMETRIZATION_HEADER
#include "Torus.h"
#include "basic.h"
#include <GfxTL/VectorXD.h>
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/AABox.h>
#include <utility>
#include <MiscLib/Vector.h>
#include <iostream>

class SimpleTorusParametrization
{
public:
	SimpleTorusParametrization(const Torus &torus);
	void Shape(const Torus &torus);
	const Torus &Shape() const { return *m_torus; }
	inline void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const;
	inline bool InSpace(float u, float v, Vec3f *p) const;
	inline bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const;
	void WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, bool *uwrap, bool *vwrap) const;
	void WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< int > *componentImg,
		MiscLib::Vector< std::pair< int, size_t > > *labels) const {}
	static size_t SerializedSize();
	void Serialize(std::ostream *o, bool binary) const;
	void Deserialize(std::istream *i, bool binary);

private:
	const Torus *m_torus;
	GfxTL::HyperplaneCoordinateSystem< float, 3 > m_hcs;
};

void SimpleTorusParametrization::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	Vec3f s = p - m_torus->Center();
	float planex = s.dot(m_hcs[0].Data());
	float planey = s.dot(m_hcs[1].Data());
	param->first = std::atan2(planey, planex); // major angle
	float minory = s.dot(m_torus->AxisDirection());
	float minorx = std::sqrt(planex * planex + planey * planey) - m_torus->MajorRadius();
	param->second = std::atan2(minory, minorx); // minor angle
	if(m_torus->IsAppleShaped())
	{
		if(fabs(param->second) > m_torus->AppleCutOffAngle())
			param->second = GfxTL::Math< float >::Sign(param->second)
				* m_torus->AppleCutOffAngle();
	}
	if(m_torus->MajorRadius() < m_torus->MinorRadius() * 2)
		param->first *= m_torus->MajorRadius() + m_torus->MinorRadius();
	else
		param->first *= m_torus->MajorRadius();
	param->second *= m_torus->MinorRadius();
}

bool SimpleTorusParametrization::InSpace(float u, float v, Vec3f *p) const
{
	float vangle = v / m_torus->MinorRadius();
	float minorx = std::cos(vangle);
	float minory = std::sin(vangle);
	minorx = m_torus->MinorRadius() * minorx + m_torus->MajorRadius();
	minory *= m_torus->MinorRadius();
	Vec3f pp = minorx * Vec3f(m_hcs[0].Data()) + minory * m_torus->AxisDirection();
	GfxTL::Quaternion< float > q;
	float majorRadius = (m_torus->MajorRadius() < m_torus->MinorRadius() * 2)?
		m_torus->MajorRadius() + m_torus->MinorRadius() : m_torus->MajorRadius();
	float uangle = u / majorRadius;
	q.RotationRad(uangle, m_torus->AxisDirection()[0], m_torus->AxisDirection()[1],
		m_torus->AxisDirection()[2]);
	q.Rotate(pp, p);
	*p += m_torus->Center();
	return true;
}

bool SimpleTorusParametrization::InSpace(float u, float v, Vec3f *p,
	Vec3f *n) const
{
	float vangle = v / m_torus->MinorRadius();
	float minorx = std::cos(vangle);
	float minory = std::sin(vangle);
	Vec3f nn = minorx * Vec3f(m_hcs[0].Data()) + minory * m_torus->AxisDirection();
	minorx = m_torus->MinorRadius() * minorx + m_torus->MajorRadius();
	minory *= m_torus->MinorRadius();
	Vec3f pp = minorx * Vec3f(m_hcs[0].Data()) + minory * m_torus->AxisDirection();
	GfxTL::Quaternion< float > q;
	float majorRadius = (m_torus->MajorRadius() < m_torus->MinorRadius() * 2)?
		m_torus->MajorRadius() + m_torus->MinorRadius() : m_torus->MajorRadius();
	float uangle = u / majorRadius;
	q.RotationRad(uangle, m_torus->AxisDirection()[0], m_torus->AxisDirection()[1],
		m_torus->AxisDirection()[2]);
	q.Rotate(pp, p);
	q.Rotate(nn, n);
	*p += m_torus->Center();
	return true;
}

#endif
