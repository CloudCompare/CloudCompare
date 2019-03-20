#ifndef _PLANE_H_
#define _PLANE_H_
#include "basic.h"
#include <MiscLib/Vector.h>
#include "PointCloud.h"
#include <ostream>
#include <istream>
#include <stdio.h>
#include <MiscLib/NoShrinkVector.h>
#include <GfxTL/Plane.h>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE Plane
{
public:
	enum { RequiredSamples = 1 };
	Plane() {}
	Plane(Vec3f p1, Vec3f p2, Vec3f p3);
	Plane(Vec3f p1, Vec3f normal);
	virtual ~Plane();
	bool Init(Vec3f p1, Vec3f p2, Vec3f p3);
	bool Init(const MiscLib::Vector< Vec3f > &samples);
	bool InitAverage(const MiscLib::Vector< Vec3f > &samples);
	bool Init(bool binary, std::istream *i);
	void Init(FILE *i);
	void Init(float* array);
	float getDistance (const Vec3f &pos) const {return fabs(m_dist - m_normal.dot(pos));}
	float Distance(const Vec3f &pos) const { return fabs(m_dist - m_normal.dot(pos)); }
	float SignedDistance(const Vec3f &pos) const { return m_normal.dot(pos) - m_dist; }
	void Normal(const Vec3f &, Vec3f *n) const { *n = m_normal; }
	float DistanceAndNormal(const Vec3f &pos, Vec3f *n) const { *n = m_normal; return Distance(pos); }
	const Vec3f &getNormal () const {return m_normal;}
	const Vec3f &getPosition () const {return m_pos;}
	bool equals (Plane plane);
	float SignedDistToOrigin() const { return m_dist; }
	bool Fit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end)
	{ return LeastSquaresFit(pc, begin, end); }
	bool LeastSquaresFit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end);
	template< class IteratorT >
	bool LeastSquaresFit(IteratorT begin, IteratorT end);
	static bool Interpolate(const MiscLib::Vector< Plane > &planes,
		const MiscLib::Vector< float > &weights, Plane *ip);
	void Serialize(bool binary, std::ostream *o) const;
	static size_t SerializedSize();
	void Serialize(FILE *o) const;
	void Serialize(float* array) const;
	static size_t SerializedFloatSize();
	
	void Transform(float scale, const Vec3f &translate);
	float Intersect(const Vec3f &p, const Vec3f &r) const;
   
protected:
	Vec3f m_normal;
	Vec3f m_pos;
	float m_dist;
};

template< class IteratorT >
bool Plane::LeastSquaresFit(IteratorT begin, IteratorT end)
{
	GfxTL::Plane< GfxTL::Vector3Df > pl;
	GfxTL::Vector3Df mean;
	GfxTL::Mean(begin, end, &mean);
	pl.Fit(mean, begin, end);
	*this = Plane(Vec3f(mean.Data()), Vec3f(pl.Normal().Data()));
	return true;
}

#endif //_PLANE_H_
