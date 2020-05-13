#ifndef TORUS_HEADER
#define TORUS_HEADER
#include "basic.h"
#include <MiscLib/Vector.h>
#include "PointCloud.h"
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/MathHelper.h>
#include <ostream>
#include <istream>
#include <stdio.h>
#include <MiscLib/NoShrinkVector.h>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE Torus
{
public:
	enum { RequiredSamples = 4 };
	bool Init(const MiscLib::Vector< Vec3f > &samples);
	bool InitAverage(const MiscLib::Vector< Vec3f > &samples);
	bool Init(bool binary, std::istream *i);
	void Init(FILE *i);
	void Init(float *array);
	inline float Distance(const Vec3f &p) const;
	inline void Normal(const Vec3f &p, Vec3f *n) const;
	inline float DistanceAndNormal(const Vec3f &p, Vec3f *n) const;
	inline float SignedDistance(const Vec3f &p) const;
	inline float SignedDistanceAndNormal(const Vec3f &p, Vec3f *n) const;
	inline void Project(const Vec3f &p, Vec3f *pp) const;
	void Transform(float scale,	const Vec3f &translate);
	const Vec3f &Center() const { return m_center; }
	const Vec3f &AxisDirection() const { return m_normal; }
	const float MinorRadius() const { return m_rminor; }
	const float MajorRadius() const { return m_rmajor; }
	bool LeastSquaresFit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end);
	bool Fit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end)
	{ return LeastSquaresFit(pc, begin, end); }
	bool IsAppleShaped() const { return m_appleShaped; }
	float AppleCutOffAngle() const { return m_cutOffAngle; }
	void Serialize(bool binary, std::ostream *o) const;
	static size_t SerializedSize();
	void Serialize(FILE *o) const;
	void Serialize(float* array) const;
	static size_t SerializedFloatSize();

private:
	void ComputeAppleParams();

private:
	Vec3f m_normal;
	Vec3f m_center;
	float m_rminor;
	float m_rmajor;
	bool m_appleShaped; // an apple shaped torus has rminor > rmajor
	float m_cutOffAngle; // for an apple shaped torus
		// the minor circle is cut off
	float m_appleHeight; // height of the "apple" point
};

float Torus::Distance(const Vec3f &p) const
{
	Vec3f s = p - m_center;
	float spin1 = m_normal.dot(s);
	float spin0 = (s - spin1 * m_normal).length();
	spin0 -= m_rmajor;
	if(!m_appleShaped)
		return fabs(std::sqrt(spin0 * spin0 + spin1 * spin1) - m_rminor);
	// apple shaped torus distance
	float minorAngle = std::atan2(spin1, spin0); // minor angle
	if(fabs(minorAngle) < m_cutOffAngle)
		return fabs(std::sqrt(spin0 * spin0 + spin1 * spin1) - m_rminor);
	spin0 += 2 * m_rmajor - m_rminor;
	if(minorAngle < 0)
		spin1 += m_appleHeight;
	else
		spin1 -= m_appleHeight;
	return std::sqrt(spin0 * spin0 + spin1 * spin1);
}

void Torus::Normal(const Vec3f &p, Vec3f *n) const
{
	Vec3f s = p - m_center, tmp;
	float spin1 = m_normal.dot(s);
	float spin0 = (s - (tmp = spin1 * m_normal)).length();
	spin0 -= m_rmajor;
	if(m_appleShaped)
	{
		float minorAngle = std::atan2(spin1, spin0); // minor angle
		if(fabs(minorAngle) > m_cutOffAngle)
		{
			*n = m_normal;
			if(minorAngle < 0)
				*n *= -1;
			return;
		}
	}
	Vec3f pln = s.cross(m_normal);
	Vec3f plx = m_normal.cross(pln);
	plx.normalize();
	*n = spin0 * plx + tmp;
	*n /= std::sqrt(spin0 * spin0 + spin1 * spin1);
}

float Torus::DistanceAndNormal(const Vec3f &p, Vec3f *n) const
{
	Vec3f s = p - m_center, tmp;
	float spin1 = m_normal.dot(s);
	float spin0 = (s - (tmp = spin1 * m_normal)).length();
	spin0 -= m_rmajor;
	if(m_appleShaped)
	{
		float minorAngle = std::atan2(spin1, spin0); // minor angle
		if(fabs(minorAngle) > m_cutOffAngle)
		{
			*n = m_normal;
			if(minorAngle < 0)
				*n *= -1;
			spin0 += 2 * m_rmajor - m_rminor;
			if(minorAngle < 0)
				spin1 += m_appleHeight;
			else
				spin1 -= m_appleHeight;
			return std::sqrt(spin0 * spin0 + spin1 * spin1);
		}
	}
	Vec3f pln = s.cross(m_normal);
	Vec3f plx = m_normal.cross(pln);
	plx.normalize();
	*n = spin0 * plx + tmp;
	float d = std::sqrt(spin0 * spin0 + spin1 * spin1);
	*n /= d;
	return fabs(d - m_rminor);
}

float Torus::SignedDistance(const Vec3f &p) const
{
	Vec3f s = p - m_center;
	float spin1 = m_normal.dot(s);
	float spin0 = (s - spin1 * m_normal).length();
	spin0 -= m_rmajor;
	if(!m_appleShaped)
		return std::sqrt(spin0 * spin0 + spin1 * spin1) - m_rminor;
	// apple shaped torus distance
	float minorAngle = std::atan2(spin1, spin0); // minor angle
	if(fabs(minorAngle) < m_cutOffAngle)
		return std::sqrt(spin0 * spin0 + spin1 * spin1) - m_rminor;
	spin0 += 2 * m_rmajor - m_rminor;
	if(minorAngle < 0)
		spin1 += m_appleHeight;
	else
		spin1 -= m_appleHeight;
	return -std::sqrt(spin0 * spin0 + spin1 * spin1);
}

float Torus::SignedDistanceAndNormal(const Vec3f &p, Vec3f *n) const
{
	Vec3f s = p - m_center, tmp;
	float spin1 = m_normal.dot(s);
	float spin0 = (s - (tmp = spin1 * m_normal)).length();
	spin0 -= m_rmajor;
	if(m_appleShaped)
	{
		float minorAngle = std::atan2(spin1, spin0); // minor angle
		if(fabs(minorAngle) > m_cutOffAngle)
		{
			*n = m_normal;
			if(minorAngle < 0)
				*n *= -1;
			spin0 += 2 * m_rmajor - m_rminor;
			if(minorAngle < 0)
				spin1 += m_appleHeight;
			else
				spin1 -= m_appleHeight;
			return -std::sqrt(spin0 * spin0 + spin1 * spin1);
		}
	}
	Vec3f pln = s.cross(m_normal);
	Vec3f plx = m_normal.cross(pln);
	plx.normalize();
	*n = spin0 * plx + tmp;
	float d = std::sqrt(spin0 * spin0 + spin1 * spin1);
	*n /= d;
	return d - m_rminor;
}

void Torus::Project(const Vec3f &p, Vec3f *pp) const
{
	Vec3f s = p - m_center, tmp;
	float spin1 = m_normal.dot(s);
	float spin0 = (s - (tmp = spin1 * m_normal)).length();
	spin0 -= m_rmajor;
	if(m_appleShaped)
	{
		float minorAngle = std::atan2(spin1, spin0); // minor angle
		if(fabs(minorAngle) > m_cutOffAngle)
		{
			*pp = m_center + GfxTL::Math< float >::Sign(minorAngle) * m_normal;
			return;
		}
	}
	Vec3f pln = s.cross(m_normal);
	Vec3f plx = m_normal.cross(pln);
	plx.normalize();
	float d = std::sqrt(spin0 * spin0 + spin1 * spin1);
	*pp = m_center + (m_rminor / d) * (spin0 * plx + tmp)
		+ m_rmajor * plx;
}

#endif
