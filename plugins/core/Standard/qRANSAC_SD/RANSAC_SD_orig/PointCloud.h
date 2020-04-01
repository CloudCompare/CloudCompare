#ifndef _POINTCLOUD_H
#define _POINTCLOUD_H
#include <cstdio>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <MiscLib/Vector.h>
#include <limits>
#include <GfxTL/VectorXD.h>
#include "basic.h"
#include <MiscLib/Vector.h>
#include <MiscLib/AlignedAllocator.h>
#undef min
#undef max

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

struct DLL_LINKAGE Point {
	enum { Dim = 3 };
	typedef float ScalarType;
	typedef float value_type;
	Vec3f pos;
	Vec3f normal;
#ifdef POINTSWITHINDEX
	unsigned index;
#endif
	//unsigned int meshFaceIndex;
	Point() {}
	Point(const Vec3f &Pos) {/*index = -1;*/ pos = Pos; normal = Vec3f(0,0,0); }
	Point(const Vec3f &Pos, const Vec3f &Normal) { pos = Pos; normal = Normal; }
	const float operator[](unsigned int i) const
	{
		return pos[i];
	}
	float &operator[](unsigned int i)
	{
		return pos[i];
	}
	operator const Vec3f &() const
	{
		return pos;
	}
	operator Vec3f &()
	{
		return pos;
	}
	operator GfxTL::VectorXD< 3, float > &()
	{
		return *reinterpret_cast< GfxTL::Vector3Df * >(&pos);
	}
	operator const GfxTL::VectorXD< 3, float > &() const
	{
		return *reinterpret_cast< const GfxTL::Vector3Df * >(&pos);
	}
};

class DLL_LINKAGE PointCloud
: public MiscLib::Vector< Point >
{
public:
	PointCloud();
	PointCloud(Point *points, unsigned int size);
	PointCloud &operator+=(const PointCloud &other)
	{
		size_t oldSize = size();
		resize(oldSize + other.size());
		std::copy(other.begin(), other.end(), begin() + oldSize);
		m_min = Vec3f(std::min(m_min[0], other.m_min[0]),
			std::min(m_min[1], other.m_min[1]),
			std::min(m_min[2], other.m_min[2]));
		m_max = Vec3f(std::max(m_max[0], other.m_max[0]),
			std::max(m_max[1], other.m_max[1]),
			std::max(m_max[2], other.m_max[2]));
		return *this;
	}
	void swapPoints (unsigned int i, unsigned int j) 
	{
		std::swap(at(i), at(j));
	}
	void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
	void reset(size_t s = 0);
	void setBBox (Vec3f bbl, float size) { m_min = bbl; m_max = m_min + Vec3f(size,size,size); }
	void setBBox (Vec3f min, Vec3f max) { m_min = min; m_max = max; }
	void widenBBox (float delta) { Vec3f d(delta,delta,delta); m_min -=d; m_max += d; }
	float getScale() const
	{
	   Vec3f diff = m_max - m_min;
	   return std::max(std::max(diff[0], diff[1]), diff[2]);
	}
	const Vec3f &getOffset() const { return m_min; }
	float *getBbox () const;
	// returns a transfromed bbox if m_transformed is true
	void GetCurrentBBox(Vec3f *min, Vec3f *max) const;
	const Vec3f &GetBBoxMin() const { return m_min; }
	const Vec3f &GetBBoxMax() const { return m_max; }
	Vec3f &GetBBoxMin() { return m_min; }
	Vec3f &GetBBoxMax() { return m_max; }
	void Translate(const Vec3f &trans);

private:
   Vec3f m_min, m_max;
};

#endif
