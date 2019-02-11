#ifndef SPHERE_HEADER
#define SPHERE_HEADER
#include "basic.h"
#include <MiscLib/Vector.h>
#include <stdexcept>
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <utility>
#include "PointCloud.h"
#include <ostream>
#include <istream>
#include <stdio.h>
#include <utility>
#include <MiscLib/NoShrinkVector.h>
#include "LevMarLSWeight.h"
#include "LevMarFitting.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

struct DLL_LINKAGE InvalidTetrahedonError
: public std::runtime_error
{
	InvalidTetrahedonError();
};

class DLL_LINKAGE Sphere
{
public:
	enum { RequiredSamples = 2 };
	Sphere();
	Sphere(const Vec3f &center, float radius);
	Sphere(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
		const Vec3f &p4);
	bool Init(const MiscLib::Vector< Vec3f > &samples);
	bool Init(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
		const Vec3f &p4);
	bool Init2(const Vec3f &p1, const Vec3f &p2, const Vec3f &n1,
		const Vec3f &n2);
	bool Init(bool binary, std::istream *i);
	void Init(FILE *i);
	void Init(float *array);
	inline float Distance(const Vec3f &p) const;
	inline void Normal(const Vec3f &p, Vec3f *normal) const;
	inline float DistanceAndNormal(const Vec3f &p, Vec3f *normal) const;
	inline float SignedDistance(const Vec3f &p) const;
	void Project(const Vec3f &p, Vec3f *pp) const;
	const Vec3f &Center() const;
	void Center(const Vec3f &center) { m_center = center; }
	float Radius() const;
	void Radius(float radius) { m_radius = radius; }
	bool LeastSquaresFit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end);
	template< class IteratorT >
	bool LeastSquaresFit(IteratorT begin, IteratorT end);
	bool Fit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end)
	{ return LeastSquaresFit(pc, begin, end); }
	static bool Interpolate(const MiscLib::Vector< Sphere > &spheres,
		const MiscLib::Vector< float > &weights, Sphere *is);
	void Serialize(bool binary, std::ostream *o) const;
	static size_t SerializedSize();
	void Serialize(FILE *o) const;
	void Serialize(float* array) const;
	static size_t SerializedFloatSize();
		
	void Transform(float scale, const Vec3f &translate);
	inline unsigned int Intersect(const Vec3f &p, const Vec3f &r,
		float *first, float *second) const;

private:
	template< class WeightT >
	class LevMarSimpleSphere
	: public WeightT
	{
	public:
		enum { NumParams = 4 };
		typedef float ScalarType;

		template< class IteratorT >
		ScalarType Chi(const ScalarType *params, IteratorT begin, IteratorT end,
			ScalarType *values, ScalarType *temp) const
		{
			ScalarType chi = 0;
			size_t size = end - begin;
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static) reduction(+:chi)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				float s = begin[idx][0] - params[0];
				s *= s;
				for(unsigned int j = 1; j < 3; ++j)
				{
					float ss = begin[idx][j] - params[j];
					s += ss * ss;
				}
				values[idx] = WeightT::Weigh(std::sqrt(s) - params[3]);
				chi += values[idx] * values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType *params, IteratorT begin, IteratorT end,
			const ScalarType *values, const ScalarType *temp, ScalarType *matrix) const
		{
			int size = static_cast<int>(end - begin);
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				float s[3];
				s[0] = begin[idx][0] - params[0];
				float sl = s[0] * s[0];
				for(unsigned int i = 1; i < 3; ++i)
				{
					s[i] = begin[idx][i] - params[i];
					sl += s[i] * s[i];
				}
				sl = std::sqrt(sl);
				matrix[idx * NumParams + 0] = -s[0] / sl;
				matrix[idx * NumParams + 1] = -s[1] / sl;
				matrix[idx * NumParams + 2] = -s[2] / sl;
				matrix[idx * NumParams + 3] = -1;
				WeightT::template DerivWeigh< NumParams >(sl - params[3],
					matrix + idx * NumParams);
			}
		}

		void Normalize(ScalarType *) const
		{}
	};

	template< class WeightT >
	class LevMarSphere
	: public WeightT
	{
	public:
		enum { NumParams = 7 };
		typedef float ScalarType;

		// parametrization:	params[0] - params[2] = normal
		//					params[3] - params[5] = point
		//					params[6] = 1 / radius

		template< class IteratorT >
		ScalarType Chi(const ScalarType *params, IteratorT begin, IteratorT end,
			ScalarType *values, ScalarType *temp) const
		{
			ScalarType chi = 0;
			ScalarType radius = 1 / params[6];
			Vec3f center = -radius * Vec3f(params[0], params[1], params[2])
				+ Vec3f(params[3], params[4], params[5]);
			int size = end - begin;
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static) reduction(+:chi)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				temp[idx] = (begin[idx] - center).length();
				chi += (values[idx] = WeightT::Weigh(temp[idx] - radius))
					* values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType *params, IteratorT begin, IteratorT end,
			const ScalarType *values, const ScalarType *temp, ScalarType *matrix) const
		{
			Vec3f normal(params[0], params[1], params[2]);
			Vec3f point(params[3], params[4], params[5]);
			int size = end - begin;
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				ScalarType denominator = -1.f / temp[idx] * params[6];
				matrix[idx * NumParams + 0] =
					(matrix[idx * NumParams + 3] = (point[0] - normal[0] * params[6] - begin[idx][0]))
						* denominator;
				matrix[idx * NumParams + 1] =
					(matrix[idx * NumParams + 4] = (point[1] - normal[1] * params[6] - begin[idx][1]))
						* denominator;
				matrix[idx * NumParams + 2] =
					(matrix[idx * NumParams + 5] = (point[2] - normal[2] * params[6] - begin[idx][2]))
						* denominator;
				matrix[idx * NumParams + 3] /= temp[idx];
				matrix[idx * NumParams + 4] /= temp[idx];
				matrix[idx * NumParams + 5] /= temp[idx];
				matrix[idx * NumParams + 6] = (normal[0] * matrix[idx * NumParams + 3]
					+ normal[1] * matrix[idx * NumParams + 4]
					+ normal[2] * matrix[idx * NumParams + 5] + 1) * params[6] * params[6];
				WeightT::template DerivWeigh< NumParams >(temp[idx] - 1.f / params[6],
					matrix + idx * NumParams);
			}
		}

		void Normalize(ScalarType *params) const
		{
			ScalarType len = std::sqrt(params[0] * params[0]
				+ params[1] * params[1] + params[2] * params[2]);
			params[0] /= len;
			params[1] /= len;
			params[2] /= len;
		}
	};

private:
	Vec3f m_center;
	float m_radius;
};

inline float Sphere::Distance(const Vec3f &p) const
{
	return fabs((m_center - p).length() - m_radius);
}

inline void Sphere::Normal(const Vec3f &p, Vec3f *normal) const
{
	*normal = p - m_center;
	normal->normalize();
}

inline float Sphere::DistanceAndNormal(const Vec3f &p, Vec3f *normal) const
{
	*normal = p - m_center;
	float l = normal->length();
	if(l > 0)
		*normal /= l;
	return fabs(l - m_radius);
}

inline float Sphere::SignedDistance(const Vec3f &p) const
{
	return (m_center - p).length() - m_radius;
}

template< class IteratorT >
bool Sphere::LeastSquaresFit(IteratorT begin, IteratorT end)
{
	LevMarSimpleSphere< LevMarLSWeight > levMarSphere;
	float param[4];
	for(size_t i = 0; i < 3; ++i)
		param[i] = m_center[i];
	param[3] = m_radius;
	if(!LevMar(begin, end, levMarSphere, param))
		return false;
	for(size_t i = 0; i < 3; ++i)
		m_center[i] = param[i];
    m_radius = param[3];
	return true;
}

inline unsigned int Sphere::Intersect(const Vec3f &p, const Vec3f &r,
	float *first, float *second) const
{
	using namespace std;
	Vec3f kDiff = p - m_center;
    float fA0 = kDiff.dot(kDiff) - m_radius*m_radius;
    float fA1, fDiscr, fRoot;
    if (fA0 <= 0)
    {
        // P is inside the sphere
        fA1 = r.dot(kDiff);
        fDiscr = fA1*fA1 - fA0;
        fRoot = sqrt(fDiscr);
        *first = -fA1 + fRoot;
        return 1;
    }
    // else: P is outside the sphere
    fA1 = r.dot(kDiff);
    if (fA1 >= 0)
		return 0;
    fDiscr = fA1*fA1 - fA0;
    if(fDiscr < 0)
		return 0;
    else if(fDiscr >= /* zero tolerance eps */ 1e-7f)
    {
        fRoot = sqrt(fDiscr);
        *first = -fA1 - fRoot;
        *second = -fA1 + fRoot;
		return 2;
    }
	*first = -fA1;
    return 1;
}

class DLL_LINKAGE SphereAsSquaresParametrization
{
	public:
		SphereAsSquaresParametrization() {}
		SphereAsSquaresParametrization(const Sphere &sphere,
			const Vec3f &planeNormal);
		void Init(const Sphere &sphere, const Vec3f &planeNormal);

		// returns < 0 if point is on lower hemisphere
		float Parameters(const Vec3f &p,
			std::pair< float, float > *param) const;
		bool InSpace(const std::pair< float, float > &param, bool lower,
			Vec3f *p) const;
		bool InSpace(const std::pair< float, float > &param, bool lower,
			Vec3f *p, Vec3f *n) const;
		void Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
			const GfxTL::Vector3Df &trans);
		void HyperplaneCoordinateSystem( Vec3f* hcs0, Vec3f* hcs1, Vec3f* hcs2 ) const;

	private:
		void Hemisphere2Disk(const Vec3f &p,
			std::pair< float, float > *inDisk) const;
		void Disk2Square(const std::pair< float, float > &inDisk,
			std::pair< float, float > *inSquare) const;
		void Square2Disk(const std::pair< float, float > &inSquare,
			std::pair< float, float > *inDisk) const;
		void Disk2Hemisphere(const std::pair< float, float > &inDisk,
			Vec3f *p) const;

	private:
		Sphere m_sphere;
		Vec3f m_planeNormal;
		GfxTL::HyperplaneCoordinateSystem< float, 3 > m_hcs;
};

class DLL_LINKAGE UpperSphereAsSquaresParametrization
: public SphereAsSquaresParametrization
{
public:
	UpperSphereAsSquaresParametrization() {}
	UpperSphereAsSquaresParametrization(const SphereAsSquaresParametrization &p)
	: SphereAsSquaresParametrization(p) {}
	bool InSpace(const std::pair< float, float > &param, Vec3f *p) const
	{ return SphereAsSquaresParametrization::InSpace(param, false, p); }
	bool InSpace(const std::pair< float, float > &param, Vec3f *p, Vec3f *n) const
	{ return SphereAsSquaresParametrization::InSpace(param, false, p, n); }
	bool InSpace(float u, float v, Vec3f *p) const
	{ return SphereAsSquaresParametrization::InSpace(std::make_pair(u, v), false, p); }
	bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const
	{ return SphereAsSquaresParametrization::InSpace(std::make_pair(u, v), false, p, n); }
};

class DLL_LINKAGE LowerSphereAsSquaresParametrization
: public SphereAsSquaresParametrization
{
public:
	LowerSphereAsSquaresParametrization() {}
	LowerSphereAsSquaresParametrization(const SphereAsSquaresParametrization &p)
	: SphereAsSquaresParametrization(p) {}
	bool InSpace(const std::pair< float, float > &param, Vec3f *p) const
	{ return SphereAsSquaresParametrization::InSpace(param, true, p); }
	bool InSpace(const std::pair< float, float > &param, Vec3f *p, Vec3f *n) const
	{ return SphereAsSquaresParametrization::InSpace(param, true, p, n); }
	bool InSpace(float u, float v, Vec3f *p) const
	{ return SphereAsSquaresParametrization::InSpace(std::make_pair(u, v), true, p); }
	bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const
	{ return SphereAsSquaresParametrization::InSpace(std::make_pair(u, v), true, p, n); }
};

#endif
