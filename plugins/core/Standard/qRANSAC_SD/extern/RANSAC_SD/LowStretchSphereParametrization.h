#ifndef LOWSTRETCHSPHEREPARAMETRIZATION_HEADER
#define LOWSTRETCHSPHEREPARAMETRIZATION_HEADER
#include "Sphere.h"
#include "basic.h"
#include <GfxTL/VectorXD.h>
#include <GfxTL/Frame.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/AABox.h>
#include <utility>
#include <MiscLib/Vector.h>

class LowStretchSphereParametrization
{
public:
	LowStretchSphereParametrization(const Sphere &sphere);
	void Shape(const Sphere &sphere);
	const Sphere &Shape() const { return *m_sphere; }
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
	const Sphere *m_sphere;
	GfxTL::Frame< 3, float > m_frame;
};

void LowStretchSphereParametrization::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	Vec3f s = p - m_sphere->Center();
	float slength = s.length();
	GfxTL::Vector3Df l;
	m_frame.ToLocal(*((GfxTL::Vector3Df *)&s), &l);
	if(slength > 0) l[2] /= slength;
	l[2] = GfxTL::Math< float >::Clamp(l[2], -1, 1);
	float radius = std::sqrt(1.f - l[2] * l[2]) * m_sphere->Radius();
	param->first = std::acos(l[2]) * m_sphere->Radius();
	param->second = std::atan2(l[1], l[0]) * radius;
}

bool LowStretchSphereParametrization::InSpace(float u, float v, Vec3f *p) const
{
	float uangle = u / m_sphere->Radius();
	float cosu = std::cos(uangle);
	float sinu = std::sin(uangle);
	float radius = sinu * m_sphere->Radius();
	float vangle = v / radius;
	float sinv = std::sin(vangle);
	float cosv = std::cos(vangle);
	GfxTL::Vector3Df g;
	m_frame.ToGlobal(GfxTL::Vector3Df(sinu * cosv, sinu * sinv, cosu), &g);
	*p = m_sphere->Radius() * Vec3f(g) + m_sphere->Center();
	return true;
}

bool LowStretchSphereParametrization::InSpace(float u, float v, Vec3f *p,
	Vec3f *n) const
{
	float uangle = u / m_sphere->Radius();
	float cosu = std::cos(uangle);
	float sinu = std::sin(uangle);
	float radius = sinu * m_sphere->Radius();
	float vangle = v / radius;
	float sinv = std::sin(vangle);
	float cosv = std::cos(vangle);
	GfxTL::Vector3Df g;
	m_frame.ToGlobal(GfxTL::Vector3Df(sinu * cosv, sinu * sinv, cosu), &g);
	*n = Vec3f(g);
	*p = m_sphere->Radius() * (*n) + m_sphere->Center();
	return true;
}

template< class IteratorT >
void LowStretchSphereParametrization::Optimize(IteratorT begin, IteratorT end,
	float epsilon)
{
	// tries to find a rotation of the parametrization that does not cut
	// the points

	// pull the center of the points onto the equator
	float minUangle = float(M_PI), maxUangle = 0;
	if(end - begin <= 1)
		return;
	for(IteratorT i = begin; i != end; ++i)
	{
		Vec3f s = *i - m_sphere->Center();
		float slength = s.length();
		float h = m_frame.Height(*((GfxTL::Vector3Df *)&s));
		if(slength > 0)	h /= slength;
		h = GfxTL::Math< float >::Clamp(h, -1, 1);
		float uangle = std::acos(h);
		if(minUangle > uangle)
			minUangle = uangle;
		if(maxUangle < uangle)
			maxUangle = uangle;
	}
	float centerUangle = (minUangle + maxUangle) / 2;
	// centerUangle should be located at 90 degrees
	GfxTL::Quaternion< float > q;
	q.RotationRad(-2 * (centerUangle - float(M_PI / 2)), m_frame[0][0], m_frame[0][1], m_frame[0][2]);
	GfxTL::Vector3Df newNormal;
	q.Rotate(m_frame[2], &newNormal);
	m_frame.FromNormal(newNormal);

	// a rotation around the normal is searched
	// for this find all the angles and then extract the largest gap
	MiscLib::Vector< float > vangles;
	for(IteratorT i = begin; i != end; ++i)
	{
		Vec3f s = *i - m_sphere->Center();
		float slength = s.length();
		GfxTL::Vector3Df l;
		m_frame.ToLocal(*((GfxTL::Vector3Df *)&s), &l);
		if(slength > 0)	l[2] /= slength;
		l[2] = GfxTL::Math< float >::Clamp(l[2], -1, 1);
		float uangle = std::acos(l[2]);
		if(uangle * m_sphere->Radius() < float(M_PI) * m_sphere->Radius() - 2 * epsilon
			&& uangle * m_sphere->Radius() > float(-M_PI) * m_sphere->Radius() + 2 * epsilon)
			vangles.push_back(std::atan2(l[1], l[0]));
	}
	if (vangles.size() < 2)
	{
		return;
	}
	std::sort(vangles.begin(), vangles.end());
	// try to find a large gap
	float maxGap = vangles.front() + 2 * float(M_PI) - vangles.back();
	float lower = vangles.back(), upper = vangles.front() + 2 * float(M_PI);
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
	// reparameterize by rotating the frame on the normal such that x direction
	// coincides with the gap
	float rotationAngle = (lower + upper) / 2;
	m_frame.RotateOnNormal(rotationAngle + float(M_PI));
}

#endif
