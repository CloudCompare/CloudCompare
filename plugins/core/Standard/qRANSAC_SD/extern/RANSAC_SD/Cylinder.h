#ifndef CYLINDER_HEADER
#define CYLINDER_HEADER
#include "basic.h"
#include <stdexcept>
#include <utility>
#include <MiscLib/Vector.h>
#include "PointCloud.h"
#include <ostream>
#include <istream>
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <stdio.h>
#include <MiscLib/NoShrinkVector.h>
#include "LevMarLSWeight.h"
#include "LevMarFitting.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE Cylinder
{
public:
	struct ParallelNormalsError
	: public std::runtime_error
	{
		ParallelNormalsError();
	};
	enum { RequiredSamples = 2 };
	Cylinder();
	Cylinder(const Vec3f &axisDir, const Vec3f &axisPos, float radius);
	Cylinder(const Vec3f &pointA, const Vec3f &pointB,
		const Vec3f &normalA, const Vec3f &normalB);
	bool Init(const MiscLib::Vector< Vec3f > &samples);
	bool InitAverage(const MiscLib::Vector< Vec3f > &samples);
	bool Init(const Vec3f &axisDir, const Vec3f &axisPos, float radius);
	bool Init(const Vec3f &pointA, const Vec3f &pointB,
		const Vec3f &normalA, const Vec3f &normalB);
	bool Init(bool binary, std::istream *i);
	void Init(FILE *i);
	void Init(float* array);
	inline float Distance(const Vec3f &p) const;
	inline void Normal(const Vec3f &p, Vec3f *normal) const;
	inline float DistanceAndNormal(const Vec3f &p, Vec3f *normal) const;
	inline float SignedDistance(const Vec3f &p) const;
	void Project(const Vec3f &p, Vec3f *pp) const;
	// parameters are (height, angle)
	void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const;
	float Radius() const;
	float &Radius();
	const Vec3f &AxisDirection() const;
	Vec3f &AxisDirection();
	const Vec3f &AxisPosition() const;
	Vec3f &AxisPosition();
	const Vec3f AngularDirection() const;
	void RotateAngularDirection(float radians);
	bool LeastSquaresFit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end);
	template< class IteratorT >
	bool LeastSquaresFit(IteratorT begin, IteratorT end);
	bool Fit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end)
	{ return LeastSquaresFit(pc, begin, end); }
	static bool Interpolate(const MiscLib::Vector< Cylinder > &cylinders,
		const MiscLib::Vector< float > &weights, Cylinder *ic);
	void Serialize(bool binary, std::ostream *o) const;
	static size_t SerializedSize();
	void Serialize(FILE *o) const;
	void Serialize(float* array) const;
	static size_t SerializedFloatSize();
		
	void Transform(float scale, const Vec3f &translate);
	void Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
		const GfxTL::Vector3Df &trans);
	inline unsigned int Intersect(const Vec3f &p, const Vec3f &r,
		float *first, float *second) const;

private:
	template< class WeightT >
	class LevMarCylinder
	: public WeightT
	{
	public:
		enum { NumParams = 7 };
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
				Vec3f s;
				for(unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType u = params[5] * s[1] - params[4] * s[2];
				u *= u;
				ScalarType v = params[3] * s[2] - params[5] * s[0];
				u += v * v;
				v = params[4] * s[0] - params[3] * s[1];
				u += v * v;
				temp[idx] = std::sqrt(u);
				chi += (values[idx] = WeightT::Weigh(temp[idx] - params[6]))
					* values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType *params, IteratorT begin, IteratorT end,
			const ScalarType *values, const ScalarType *temp, ScalarType *matrix) const
		{
			size_t size = end - begin;
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				Vec3f s;
				for(unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType g = s[0] * begin[idx][0] + s[1] * begin[idx][1]
					+ s[2] * begin[idx][2];
				if(temp[idx] < 1.0e-6)
				{
					matrix[idx * NumParams + 0] = std::sqrt(1 - params[3] * params[3]);
					matrix[idx * NumParams + 1] = std::sqrt(1 - params[4] * params[4]);
					matrix[idx * NumParams + 2] = std::sqrt(1 - params[5] * params[5]);
				}
				else
				{
					matrix[idx * NumParams + 0] = (params[3] * g - s[0]) / temp[idx];
					matrix[idx * NumParams + 1] = (params[4] * g - s[1]) / temp[idx];
					matrix[idx * NumParams + 2] = (params[5] * g - s[2]) / temp[idx];
				}
				matrix[idx * NumParams + 3] = g * matrix[idx * NumParams + 0];
				matrix[idx * NumParams + 4] = g * matrix[idx * NumParams + 1];
				matrix[idx * NumParams + 5] = g * matrix[idx * NumParams + 2];
				matrix[idx * NumParams + 6] = -1;
				WeightT::template DerivWeigh< NumParams >(temp[idx] - params[6],
					matrix + idx * NumParams);
			}
		}

		void Normalize(ScalarType *params) const
		{
			ScalarType l = std::sqrt(params[3] * params[3] + params[4] * params[4]
				+ params[5] * params[5]);
			for(unsigned int i = 3; i < 6; ++i)
				params[i] /= l;
			// find point on axis closest to origin
			float lambda = -(params[0] * params[3] + params[1] * params[4] +
				params[2] * params[5]);
			for(unsigned int i = 0; i < 3; ++i)
				params[i] = params[i] + lambda * params[i + 3];
		}
	};

private:
	Vec3f m_axisDir;
	Vec3f m_axisPos;
	float m_radius;
	GfxTL::HyperplaneCoordinateSystem< float, 3 > m_hcs;
	float m_angularRotatedRadians;
};

inline float Cylinder::Distance(const Vec3f &p) const
{
	Vec3f diff = p - m_axisPos;
	float lambda = m_axisDir.dot(diff);
	float axisDist = (diff - lambda * m_axisDir).length();
	return fabs(axisDist - m_radius);
}

inline void Cylinder::Normal(const Vec3f &p, Vec3f *normal) const
{
	Vec3f diff = p - m_axisPos;
	float lambda = m_axisDir.dot(diff);
	*normal = diff - lambda * m_axisDir;
	normal->normalize();
}

inline float Cylinder::DistanceAndNormal(const Vec3f &p, Vec3f *normal) const
{
	Vec3f diff = p - m_axisPos;
	float lambda = m_axisDir.dot(diff);
	*normal = diff - lambda * m_axisDir;
	float axisDist = normal->length();
	if(axisDist > 0) 
		*normal /= axisDist;
	return fabs(axisDist - m_radius);
}

inline float Cylinder::SignedDistance(const Vec3f &p) const
{
	Vec3f diff = p - m_axisPos;
	float lambda = m_axisDir.dot(diff);
	float axisDist = (diff - lambda * m_axisDir).length();
	return axisDist - m_radius;
}

template< class IteratorT >
bool Cylinder::LeastSquaresFit(IteratorT begin, IteratorT end)
{
	float param[7];
	for(size_t i = 0; i < 3; ++i)
		param[i] = m_axisPos[i];
	for(size_t i = 0; i < 3; ++i)
		param[i + 3] = m_axisDir[i];
	param[6] = m_radius;
	LevMarCylinder< LevMarLSWeight > levMarCylinder;
	if(!LevMar(begin, end, levMarCylinder, param))
		return false;
	for(size_t i = 0; i < 3; ++i)
		m_axisPos[i] = param[i];
	for(size_t i = 0; i < 3; ++i)
		m_axisDir[i] = param[i + 3];
	m_radius = param[6];
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

inline unsigned int Cylinder::Intersect(const Vec3f &p, const Vec3f &r,
	float *first, float *second) const
{
	using namespace std;
    // Create a coordinate system for the cylinder.  In this system, the
    // cylinder segment center C is the origin and the cylinder axis direction
    // W is the z-axis.  U and V are the other coordinate axis directions.
    // If P = x*U+y*V+z*W, the cylinder is x^2 + y^2 = r^2, where r is the
    // cylinder radius.  The end caps are |z| = h/2, where h is the cylinder
    // height.
    float fRSqr = m_radius * m_radius;

    // convert incoming line origin to cylinder coordinates
    Vec3f kDiff = p - m_axisPos;
    Vec3f kP(kDiff.dot(m_hcs[0]), kDiff.dot(m_hcs[1]), m_axisDir.dot(kDiff));

    // Get the z-value, in cylinder coordinates, of the incoming line's
    // unit-length direction.
    float fDz = m_axisDir.dot(r);

    if(fabs(fDz) >= 1.f - 1e-7f)
        // The line is parallel to the cylinder axis.
		return 0;

    // convert incoming line unit-length direction to cylinder coordinates
	Vec3f kD(r.dot(m_hcs[0]), r.dot(m_hcs[1]), r.dot(m_axisDir));

    float fA0, fA1, fA2, fDiscr, fRoot, fInv;

    // Test intersection of line P+t*D with infinite cylinder
    // x^2+y^2 = r^2.  This reduces to computing the roots of a
    // quadratic equation.  If P = (px,py,pz) and D = (dx,dy,dz),
    // then the quadratic equation is
    //   (dx^2+dy^2)*t^2 + 2*(px*dx+py*dy)*t + (px^2+py^2-r^2) = 0
    fA0 = kP[0]*kP[0] + kP[1]*kP[1] - fRSqr;
    fA1 = kP[0]*kD[0] + kP[1]*kD[1];
    fA2 = kD[0]*kD[0] + kD[1]*kD[1];
    fDiscr = fA1*fA1 - fA0*fA2;
    if (fDiscr < 0)
        // line does not intersect cylinder
        return 0;
    else if (fDiscr > 1e-7f)
    {
        // line intersects cylinder in two places
        fRoot = sqrt(fDiscr);
        fInv = (1.f)/fA2;
        *first = (-fA1 - fRoot)*fInv;
        *second = (-fA1 + fRoot)*fInv;
        return 2;
    }
    // line is tangent to the cylinder
    *first = -fA1/fA2;
    return 1;
}

#endif
