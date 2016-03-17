#ifndef CONE_HEADER
#define CONE_HEADER
#ifdef DOPARALLEL
#include <omp.h>
#endif
#include "basic.h"
#include "PointCloud.h"
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <stdexcept>
#include <ostream>
#include <istream>
#include <stdio.h>
#if !defined(_WIN32) && !defined(WIN32)
#include <unistd.h>
#endif
#include <MiscLib/NoShrinkVector.h>
#include "LevMarLSWeight.h"
#include "LevMarFitting.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

// This implements a one sided cone!
class DLL_LINKAGE Cone
{
public:
	struct ParallelPlanesError
	: public std::runtime_error
	{
		ParallelPlanesError()
		: std::runtime_error("Parallel planes in cone construction")
		{}
	};
	enum { RequiredSamples = 3 };
	Cone();
	Cone(const Vec3f &center, const Vec3f &axisDir, float angle);
	Cone(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
		const Vec3f &n1, const Vec3f &n2, const Vec3f &n3);
	bool Init(const MiscLib::Vector< Vec3f > &samples);
	bool InitAverage(const MiscLib::Vector< Vec3f > &samples);
	bool Init(const Vec3f &center, const Vec3f &axisDir, float angle);
	bool Init(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
		const Vec3f &n1, const Vec3f &n2, const Vec3f &n3);
	bool Init(bool binary, std::istream *i);
	void Init(FILE *i);
	void Init(float* array);
	inline float Distance(const Vec3f &p) const;
	inline void Normal(const Vec3f &p, Vec3f *n) const;
	inline float DistanceAndNormal(const Vec3f &p, Vec3f *n) const;
	inline float SignedDistance(const Vec3f &p) const;
	inline float SignedDistanceAndNormal(const Vec3f &p, Vec3f *n) const;
	void Project(const Vec3f &p, Vec3f *pp) const;
	// Paramterizes into (length, angle)
	void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const;
	inline float Height(const Vec3f &p) const;
	inline float Angle() const;
	inline const Vec3f &Center() const;
	inline const Vec3f &AxisDirection() const;
	Vec3f &AxisDirection() { return m_axisDir; }
	inline const Vec3f AngularDirection() const;
	//void AngularDirection(const Vec3f &angular);
	void RotateAngularDirection(float radians);
	inline float RadiusAtLength(float length) const;
	bool LeastSquaresFit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end);
	template< class IteratorT >
	bool LeastSquaresFit(IteratorT begin, IteratorT end);
	bool Fit(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end)
	{ return LeastSquaresFit(pc, begin, end); }
	static bool	Interpolate(const MiscLib::Vector< Cone > &cones,
		const MiscLib::Vector< float > &weights, Cone *ic);
	void Serialize(bool binary, std::ostream *o) const;
	static size_t SerializedSize();
	void Serialize(FILE *o) const;
	void Serialize(float* array) const;
	static size_t SerializedFloatSize();
	void Transform(float scale, const Vec3f &translate);
	void Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
		const GfxTL::Vector3Df &trans);
	inline unsigned int Intersect(const Vec3f &p, const Vec3f &r,
		float lambda[2], Vec3f interPts[2]) const;

private:
	template< class WeightT >
	class LevMarCone
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
			ScalarType cosPhi = std::cos(params[6]);
			ScalarType sinPhi = std::sin(params[6]);
			int size = end - begin;
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static) reduction(+:chi)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				Vec3f s;
				for(unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType g = fabs(s[0] * params[3] + s[1] * params[4] + s[2] * params[5]);
				ScalarType f = s.sqrLength() - (g * g);
				if(f <= 0)
					f = 0;
				else
					f = std::sqrt(f);
				temp[idx] = f;
				chi += (values[idx] = WeightT::Weigh(cosPhi * f - sinPhi * g))
					* values[idx];
			}
			return chi;
		}

		template< class IteratorT >
		void Derivatives(const ScalarType *params, IteratorT begin, IteratorT end,
			const ScalarType *values, const ScalarType *temp, ScalarType *matrix) const
		{
			ScalarType sinPhi = -std::sin(params[6]);
			ScalarType cosPhi = std::cos(params[6]);
			int size = end - begin;
#ifdef DOPARALLEL
			#pragma omp parallel for schedule(static)
#endif
			for(int idx = 0; idx < size; ++idx)
			{
				Vec3f s;
				for(unsigned int j = 0; j < 3; ++j)
					s[j] = begin[idx][j] - params[j];
				ScalarType g = fabs(s[0] * params[3] + s[1] * params[4] + s[2] * params[5]);
				ScalarType ggradient[6];
				for(unsigned int j = 0; j < 3; ++j)
					ggradient[j] = -params[j + 3];
				for(unsigned int j = 0; j < 3; ++j)
					ggradient[j + 3] = s[j] - params[j + 3] * g;
				ScalarType fgradient[6];
				if(temp[idx] < 1.0e-6)
				{
					fgradient[0] = std::sqrt(1 - params[3] * params[3]);
					fgradient[1] = std::sqrt(1 - params[4] * params[4]);
					fgradient[2] = std::sqrt(1 - params[5] * params[5]);
				}
				else
				{
					fgradient[0] = (params[3] * g - s[0]) / temp[idx];
					fgradient[1] = (params[4] * g - s[1]) / temp[idx];
					fgradient[2] = (params[5] * g - s[2]) / temp[idx];
				}
				fgradient[3] = g * fgradient[0];
				fgradient[4] = g * fgradient[1];
				fgradient[5] = g * fgradient[2];
				for(unsigned int j = 0; j < 6; ++j)
					matrix[idx * NumParams + j] =
						cosPhi * fgradient[j] + sinPhi * ggradient[j];
				matrix[idx * NumParams + 6] = temp[idx] * sinPhi - g * cosPhi;
				WeightT::template DerivWeigh< NumParams >(cosPhi * temp[idx] + sinPhi * g,
					matrix + idx * NumParams);
			}
		}

		void Normalize(float *params) const
		{
			// normalize direction
			ScalarType l = std::sqrt(params[3] * params[3] + params[4] * params[4] +
				params[5] * params[5]);
			for(unsigned int i = 3; i < 6; ++i)
				params[i] /= l;
			// normalize angle
			params[6] -= std::floor(params[6] / (2 * ScalarType(M_PI))) * (2 * ScalarType(M_PI)); // params[6] %= 2*M_PI
			if(params[6] > M_PI)
			{
				params[6] -= std::floor(params[6] / ScalarType(M_PI)) * ScalarType(M_PI); // params[6] %= M_PI
				for(unsigned int i = 3; i < 6; ++i)
					params[i] *= -1;
			}
			if(params[6] > ScalarType(M_PI) / 2)
				params[6] = ScalarType(M_PI) - params[6];
		}
	};

private:
	Vec3f m_center; // this is the apex of the cone
	Vec3f m_axisDir; // the axis points into the interior of the cone
	float m_angle; // the opening angle
	Vec3f m_normal;
	Vec3f m_normalY; // precomputed normal part
	float m_n2d[2];
	GfxTL::HyperplaneCoordinateSystem< float, 3 > m_hcs;
	float m_angularRotatedRadians;
};

inline float Cone::Distance(const Vec3f &p) const
{
	// this is for one sided cone!
	Vec3f s = p - m_center;
	float g = s.dot(m_axisDir); // distance to plane orhogonal to
		// axisdir through center
	// distance to axis
	float sqrS = s.sqrLength();
	float f = sqrS - (g * g);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	float da = m_n2d[0] * f;
	float db = m_n2d[1] * g;
	if(g < 0 && da - db < 0) // is inside other side of cone -> disallow
		return std::sqrt(sqrS);
	return fabs(da + db);
}

inline void Cone::Normal(const Vec3f &p, Vec3f *n) const
{
	Vec3f s = p - m_center;
	Vec3f pln = s.cross(m_axisDir);
	Vec3f plx = m_axisDir.cross(pln);
	plx.normalize();
	// we are not dealing with two-sided cone
	*n = m_normal[0] * plx + m_normalY;
}

inline float Cone::DistanceAndNormal(const Vec3f &p, Vec3f *n) const
{
	// this is for one-sided cone !!!
	Vec3f s = p - m_center;
	float g = s.dot(m_axisDir); // distance to plane orhogonal to
		// axisdir through center
	// distance to axis
	float sqrS = s.sqrLength();
	float f = sqrS - (g * g);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	float da = m_n2d[0] * f;
	float db = m_n2d[1] * g;
	float dist;
	if(g < 0 && da - db < 0) // is inside other side of cone -> disallow
		dist = std::sqrt(sqrS);
	else
		dist = fabs(da + db);
	// need normal
	Vec3f plx = s - g * m_axisDir;
	plx.normalize();
	*n = m_normal[0] * plx + m_normalY;
	return dist;
}

inline float Cone::SignedDistance(const Vec3f &p) const
{
	// this is for one sided cone!
	Vec3f s = p - m_center;
	float g = s.dot(m_axisDir); // distance to plane orhogonal to
		// axisdir through center
	// distance to axis
	float sqrS = s.sqrLength();
	float f = sqrS - (g * g);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	float da = m_n2d[0] * f;
	float db = m_n2d[1] * g;
	if(g < 0 && da - db < 0) // is inside other side of cone -> disallow
		return std::sqrt(sqrS);
	return da + db;
}

inline float Cone::SignedDistanceAndNormal(const Vec3f &p, Vec3f *n) const
{
	// this is for one-sided cone !!!
	Vec3f s = p - m_center;
	float g = s.dot(m_axisDir); // distance to plane orhogonal to
		// axisdir through center
	// distance to axis
	float sqrS = s.sqrLength();
	float f = sqrS - (g * g);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	float da = m_n2d[0] * f;
	float db = m_n2d[1] * g;
	float dist;
	if(g < 0 && da - db < 0) // is inside other side of cone -> disallow
		dist = std::sqrt(sqrS);
	else
		dist = da + db;
	// need normal
	Vec3f plx = s - g * m_axisDir;
	plx.normalize();
	*n = m_normal[0] * plx + m_normalY;
	return dist;
}

float Cone::Angle() const
{
	return m_angle;
}

const Vec3f &Cone::Center() const
{
	return m_center;
}

const Vec3f &Cone::AxisDirection() const
{
	return m_axisDir;
}

const Vec3f Cone::AngularDirection() const
{
	return Vec3f(m_hcs[0].Data());
}

float Cone::RadiusAtLength(float length) const
{
	return std::sin(m_angle) * fabs(length);
}

float Cone::Height(const Vec3f &p) const
{
	Vec3f s = p - m_center;
	return m_axisDir.dot(s);
}

template< class IteratorT >
bool Cone::LeastSquaresFit(IteratorT begin, IteratorT end)
{
	float param[7];
	for(unsigned int i = 0; i < 3; ++i)
		param[i] = m_center[i];
	for(unsigned int i = 0; i < 3; ++i)
		param[i + 3] = m_axisDir[i];
	param[6] = m_angle;
	LevMarCone< LevMarLSWeight > levMarCone;
	if(!LevMar(begin, end, levMarCone, param))
		return false;
	if(param[6] < 1.0e-6 || param[6] > float(M_PI) / 2 - 1.0e-6)
		return false;
	for(unsigned int i = 0; i < 3; ++i)
		m_center[i] = param[i];
	for(unsigned int i = 0; i < 3; ++i)
		m_axisDir[i] = param[i + 3];
	m_angle = param[6];
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	// it could be that the axis has flipped during fitting
	// we need to detect such a case
	// for this we run over all points and compute the sum
	// of their respective heights. If that sum is negative
	// the axis needs to be flipped.
	float heightSum = 0;
	intptr_t size = end - begin;
//#ifndef _WIN64 // for some reason the Microsoft x64 compiler crashes at the next line
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static) reduction(+:heightSum)
#endif
//#endif
	for(intptr_t i = 0; i < size; ++i)
		heightSum += Height(begin[i]);
	if(heightSum < 0)
	{
		m_axisDir *= -1;
		m_hcs.FromNormal(m_axisDir);
	}
	return true;
}

inline unsigned int Cone::Intersect(const Vec3f &p, const Vec3f &r,
	float lambda[2], Vec3f interPts[2]) const
{
    // Set up the quadratic Q(t) = c2*t^2 + 2*c1*t + c0 that corresponds to
    // the cone.  Let the vertex be V, the unit-length direction vector be A,
    // and the angle measured from the cone axis to the cone wall be Theta,
    // and define g = cos(Theta).  A point X is on the cone wall whenever
    // Dot(A,(X-V)/|X-V|) = g.  Square this equation and factor to obtain
    //   (X-V)^T * (A*A^T - g^2*I) * (X-V) = 0
    // where the superscript T denotes the transpose operator.  This defines
    // a double-sided cone.  The line is L(t) = P + t*D, where P is the line
    // origin and D is a unit-length direction vector.  Substituting
    // X = L(t) into the cone equation above leads to Q(t) = 0.  Since we
    // want only intersection points on the single-sided cone that lives in
    // the half-space pointed to by A, any point L(t) generated by a root of
    // Q(t) = 0 must be tested for Dot(A,L(t)-V) >= 0.
	using namespace std;
    float fAdD = m_axisDir.dot(r);
    float tmp, fCosSqr = (tmp = cos(m_angle)) * tmp;
    Vec3f kE = p - m_center;
    float fAdE = m_axisDir.dot(kE);
    float fDdE = r.dot(kE);
    float fEdE = kE.dot(kE);
    float fC2 = fAdD*fAdD - fCosSqr;
    float fC1 = fAdD*fAdE - fCosSqr*fDdE;
    float fC0 = fAdE*fAdE - fCosSqr*fEdE;
    float fdot;

    // Solve the quadratic.  Keep only those X for which Dot(A,X-V) >= 0.
	unsigned int interCount = 0;
    if (fabs(fC2) >= 1e-7)
    {
        // c2 != 0
        float fDiscr = fC1*fC1 - fC0*fC2;
        if (fDiscr < 0)
        {
            // Q(t) = 0 has no real-valued roots.  The line does not
            // intersect the double-sided cone.
			return 0;
        }
        else if (fDiscr > 1e-7)
        {
            // Q(t) = 0 has two distinct real-valued roots.  However, one or
            // both of them might intersect the portion of the double-sided
            // cone "behind" the vertex.  We are interested only in those
            // intersections "in front" of the vertex.
            float fRoot = sqrt(fDiscr);
            float fInvC2 = 1.0f/fC2;
            interCount = 0;

            float fT = (-fC1 - fRoot)*fInvC2;
			if(fT > 0) // intersect only in positive direction of ray
			{
				interPts[interCount] = p + fT*r;
				kE = interPts[interCount] - m_center;
				fdot = kE.dot(m_axisDir);
				if (fdot > 0)
				{
					lambda[interCount] = fT;
					interCount++;
				}
			}

            fT = (-fC1 + fRoot)*fInvC2;
			if(fT > 0)
			{
				interPts[interCount] = p + fT*r;
				kE = interPts[interCount] - m_center;
				fdot = kE.dot(m_axisDir);
				if (fdot > 0)
				{
					lambda[interCount] = fT;
					interCount++;
				}
			}
        }
        else if(fC1 / fC2 < 0)
        {
            // one repeated real root (line is tangent to the cone)
            interPts[0] = p - (fC1/fC2)*r;
			lambda[0] = -(fC1 / fC2);
            kE = interPts[0] - m_center;
            if (kE.dot(m_axisDir) > 0)
                interCount = 1;
            else
                interCount = 0;
        }
    }
    else if (fabs(fC1) >= 1.0e-7f)
    {
        // c2 = 0, c1 != 0 (D is a direction vector on the cone boundary)
		lambda[0] = -(0.5f * fC0/fC1);
		if(lambda[0] < 0)
			return 0;
        interPts[0] = p + lambda[0] *r;
        kE = interPts[0] - m_center;
        fdot = kE.dot(m_axisDir);
        if (fdot > 0)
            interCount = 1;
        else
            interCount = 0;
    }
    else if (abs(fC0) >= 1e-7)
    {
        // c2 = c1 = 0, c0 != 0
        interCount = 0;
    }
    else
    {
        // c2 = c1 = c0 = 0, cone contains ray V+t*D where V is cone vertex
        // and D is the line direction.
        interCount = 1;
		lambda[0] = (m_center - p).dot(r);
        interPts[0] = m_center;
    }

    return interCount;
}

#endif
