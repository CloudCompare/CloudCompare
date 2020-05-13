#include "Cone.h"
#include "Plane.h"
#include "LevMarFitting.h"
#include <MiscLib/Performance.h>
#include <GfxTL/IndexedIterator.h>
#include "LevMarLSWeight.h"
#include <GfxTL/Mean.h>
#ifdef DOPARALLEL
#include <omp.h>
#endif

extern int dmat_solve ( int n, int rhs_num, double a[] );

Cone::Cone()
: m_angularRotatedRadians(0)
{}

Cone::Cone(const Vec3f &center, const Vec3f &axisDir, float angle)
: m_angularRotatedRadians(0)
{
	m_center = center;
	m_axisDir = axisDir;
	m_angle = angle;
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
}

Cone::Cone(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
	const Vec3f &n1, const Vec3f &n2, const Vec3f &n3)
: m_angularRotatedRadians(0)
{
	if(!Init(p1, p2, p3, n1, n2, n3))
		throw ParallelPlanesError();
}

bool Cone::Init(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 6)
		return false;
	size_t c = samples.size() >> 1;
	return Init(samples[0], samples[1], samples[2],
		samples[c], samples[c + 1], samples[c + 2]);
}

class LevMarPlaneDistance
{
public:
	enum { NumParams = 3 };
	typedef float ScalarType;

	template< class IteratorT >
	ScalarType Chi(const ScalarType *params, IteratorT begin, IteratorT end,
		ScalarType *values, ScalarType *temp) const
	{
		ScalarType chi = 0;
		intptr_t size = end - begin;
#ifdef DOPARALLEL
		#pragma omp parallel for schedule(static) reduction(+:chi)
#endif
		for(intptr_t i = 0; i < size; ++i)
		{
			values[i] = 0;
			for(unsigned int j = 0; j < 3; ++j)
				values[i] += begin[i][j] * params[j];
			values[i] -= begin[i][3];
			chi += values[i] * values[i];
		}
		return chi;
	}

	template< class IteratorT >
	void Derivatives(const ScalarType *params, IteratorT begin, IteratorT end,
		ScalarType *values, ScalarType *temp, ScalarType *matrix) const
	{
		intptr_t size = end - begin;
#ifdef DOPARALLEL
		#pragma omp parallel for schedule(static)
#endif
		for(intptr_t i = 0; i < size; ++i)
		{
			for(unsigned int j = 0; j < 3; ++j)
				matrix[i * NumParams + j] = begin[i][j];
		}
	}

	void Normalize(ScalarType *) const {}
};

bool Cone::InitAverage(const MiscLib::Vector< Vec3f > &samples)
{
	// setup all the planes
	size_t c = samples.size() / 2;
	MiscLib::Vector< GfxTL::Vector4Df > planes(c);
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static)
#endif
	for (int i = 0; i < static_cast<int>(c); ++i)
	{
		for(unsigned int j = 0; j < 3; ++j)
			planes[i][j] = samples[i][j];
		planes[i][3] = samples[i].dot(samples[i + c]);
	}
	// compute center by intersecting the three planes given by (p1, n1)
	// (p2, n2) and (p3, n3)
	// set up linear system
	double a[4 * 3];
	double d1 = samples[0].dot(samples[c + 0]);
	double d2 = samples[1].dot(samples[c + 1]);
	double d3 = samples[2].dot(samples[c + 2]);
	// column major
	a[0 + 0 * 3] = samples[c + 0][0];
	a[1 + 0 * 3] = samples[c + 1][0];
	a[2 + 0 * 3] = samples[c + 2][0];
	a[0 + 1 * 3] = samples[c + 0][1];
	a[1 + 1 * 3] = samples[c + 1][1];
	a[2 + 1 * 3] = samples[c + 2][1];
	a[0 + 2 * 3] = samples[c + 0][2];
	a[1 + 2 * 3] = samples[c + 1][2];
	a[2 + 2 * 3] = samples[c + 2][2];
	a[0 + 3 * 3] = d1;
	a[1 + 3 * 3] = d2;
	a[2 + 3 * 3] = d3;
	if(dmat_solve(3, 1, a))
		return false;
	m_center[0] = static_cast<float>(a[0 + 3 * 3]);
	m_center[1] = static_cast<float>(a[1 + 3 * 3]);
	m_center[2] = static_cast<float>(a[2 + 3 * 3]);

	LevMarPlaneDistance planeDistance;
	LevMar(planes.begin(), planes.end(), planeDistance,
		(float *)m_center);

	MiscLib::Vector< GfxTL::Vector3Df > spoints(c);
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static)
#endif
	for (int i = 0; i < static_cast<int>(c); ++i)
	{
		spoints[i] = GfxTL::Vector3Df(samples[i] - m_center);
		spoints[i].Normalize();
	}
	GfxTL::Vector3Df axisDir;
	GfxTL::MeanOfNormals(spoints.begin(), spoints.end(), &axisDir);
	m_axisDir = GfxTL::Vector3Df(axisDir);

	// make sure axis points in good direction
	// the axis is defined to point into the interior of the cone
	float heightSum = 0;
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static) reduction(+:heightSum)
#endif
	for(int i = 0; i < static_cast<int>(c); ++i)
		heightSum += Height(samples[i]);
	if(heightSum < 0)
		m_axisDir *= -1;

	float angleReduction = 0;
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static) reduction(+:angleReduction)
#endif
	for(int i = 0; i < static_cast<int>(c); ++i)
	{
		float angle = m_axisDir.dot(samples[i + c]);
		if(angle < -1) // clamp angle to [-1, 1]
			angle = -1;
		else if(angle > 1)
			angle = 1;
		if(angle < 0)
			// m_angle = omega + 90
			angle = std::acos(angle) - float(M_PI) / 2;
		else
			// m_angle = 90 - omega
			angle = float(M_PI) / 2 - std::acos(angle);
		angleReduction += angle;
	}
	angleReduction /= c;
	m_angle = angleReduction;
	if(m_angle < 1.0e-6 || m_angle > float(M_PI) / 2 - 1.0e-6)
		return false;
	//if(m_angle > 1.3962634015954636615389526147909) // 80 degrees
	if(m_angle > 1.4835298641951801403851371532153f) // 85 degrees
		return false;
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cone::Init(const Vec3f &center, const Vec3f &axisDir, float angle)
{
	if(angle > 1.4835298641951801403851371532153) // do not allow flat cones
		return false;
	m_center = center;
	m_axisDir = axisDir;
	m_angle = angle;
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cone::Init(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
	const Vec3f &n1, const Vec3f &n2, const Vec3f &n3)
{
	//float ncheck = std::max(n2.dot(n3), std::max(n1.dot(n2), n1.dot(n3)));
	//if(ncheck > 0.999)
	//	return false;
	// compute center by intersecting the three planes given by (p1, n1)
	// (p2, n2) and (p3, n3)
	// set up linear system
	double a[4 * 3];
	double d1 = p1.dot(n1);
	double d2 = p2.dot(n2);
	double d3 = p3.dot(n3);
	// column major
	a[0 + 0 * 3] = n1[0];
	a[1 + 0 * 3] = n2[0];
	a[2 + 0 * 3] = n3[0];
	a[0 + 1 * 3] = n1[1];
	a[1 + 1 * 3] = n2[1];
	a[2 + 1 * 3] = n3[1];
	a[0 + 2 * 3] = n1[2];
	a[1 + 2 * 3] = n2[2];
	a[2 + 2 * 3] = n3[2];
	a[0 + 3 * 3] = d1;
	a[1 + 3 * 3] = d2;
	a[2 + 3 * 3] = d3;
	if(dmat_solve(3, 1, a))
		return false;
	m_center[0] = static_cast<float>(a[0 + 3 * 3]);
	m_center[1] = static_cast<float>(a[1 + 3 * 3]);
	m_center[2] = static_cast<float>(a[2 + 3 * 3]);

	// compute axisDir
	Vec3f s1 = p1 - m_center;
	Vec3f s2 = p2 - m_center;
	Vec3f s3 = p3 - m_center;
	s1.normalize();
	s2.normalize();
	s3.normalize();
	Plane pl(s1 + m_center, s2 + m_center, s3 + m_center);
	m_axisDir = pl.getNormal();
	// make sure axis points in direction of s1
	// this defines the side of the cone!!!
	if(m_axisDir.dot(s1) < 0)
		m_axisDir *= -1;
	m_angle = 0;
	float angle = m_axisDir.dot(n1);
	if(angle < -1) // clamp angle to [-1, 1]
		angle = -1;
	else if(angle > 1)
		angle = 1;
	if(angle < 0)
		// m_angle = omega + 90
		angle = std::acos(angle) - float(M_PI) / 2;
	else
		// m_angle = 90 - omega
		angle = float(M_PI) / 2 - std::acos(angle);
	m_angle += angle;
	angle = m_axisDir.dot(n2);
	if(angle < -1) // clamp angle to [-1, 1]
		angle = -1;
	else if(angle > 1)
		angle = 1;
	if(angle < 0)
		// m_angle = omega + 90
		angle = std::acos(angle) - float(M_PI) / 2;
	else
		// m_angle = 90 - omega
		angle = float(M_PI) / 2 - std::acos(angle);
	m_angle += angle;
	angle = m_axisDir.dot(n3);
	if(angle < -1) // clamp angle to [-1, 1]
		angle = -1;
	else if(angle > 1)
		angle = 1;
	if(angle < 0)
		// m_angle = omega + 90
		angle = std::acos(angle) - float(M_PI) / 2;
	else
		// m_angle = 90 - omega
		angle = float(M_PI) / 2 - std::acos(angle);
	m_angle += angle;
	m_angle /= 3;
	if(m_angle < 1.0e-6 || m_angle > float(M_PI) / 2 - 1.0e-6)
		return false;
	//if(m_angle > 1.3962634015954636615389526147909) // 80 degrees
	if(m_angle > 1.4835298641951801403851371532153f) // 85 degrees
		return false;
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cone::Init(bool binary, std::istream *i)
{
	float rotate = 0;
	if(binary)
	{
		i->read((char *)&m_center, sizeof(m_center));
		i->read((char *)&m_axisDir, sizeof(m_axisDir));
		i->read((char *)&m_angle, sizeof(m_angle));
		i->read((char *)&rotate,
			sizeof(rotate));
	}
	else
	{
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_center[j];
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_axisDir[j];
		(*i) >> m_angle;
		(*i) >> rotate;
	}
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	RotateAngularDirection(rotate);
	return true;
}

void Cone::Init(float *array)
{
	float rotate = 0;
	for(int i=0;i<3;i++){
		m_center[i] = array[i];
		m_axisDir[i] = array[3+i];
	}
	m_angle = array[6];
	rotate = array[7];
	
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	RotateAngularDirection(rotate);
}


void Cone::Init(FILE *i)
{
	float rotate = 0;
	size_t readrtn; //unused return warning suppresion
	readrtn = fread(&m_center, sizeof(m_center), 1, i);
	readrtn = fread(&m_axisDir, sizeof(m_axisDir), 1, i);
	readrtn = fread(&m_angle, sizeof(m_angle), 1, i);
	readrtn = fread(&rotate, sizeof(rotate), 1, i);
	m_normal = Vec3f(std::cos(-m_angle), std::sin(-m_angle), 0);
	m_normalY = m_normal[1] * m_axisDir;
	m_n2d[0] = std::cos(m_angle);
	m_n2d[1] = -std::sin(m_angle);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	RotateAngularDirection(rotate);
}

void Cone::Project(const Vec3f &p, Vec3f *pp) const
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
	if(g < 0 && da - db < 0) // is inside other side of cone -> disallow
	{
		*pp = m_center;
		return;
	}
	float dist = -(da + db);
	// need normal
	Vec3f plx = s - g * m_axisDir;
	plx.normalize();
	Vec3f n = m_normal[0] * plx + m_normalY;
	*pp = p + dist * n;
}

void Cone::Parameters(const Vec3f &p, std::pair< float, float > *param) const
{
	// parametrize
	Vec3f s = p - m_center;
	float height = m_axisDir.dot(s);
	float planex = s.dot(m_hcs[0].Data());
	float planey = s.dot(m_hcs[1].Data());
	float l = planex * planex + planey * planey;
	if(l > 0)
	{
		planex /= l;
		planey /= l;
	}
	float angle = std::atan2(planey, planex);
	if(angle < 0)
		angle += float(2 * M_PI);
	/*Vec3f axisDiff = s - height * m_axisDir;
	axisDiff.normalize();
	float angle = m_angular.dot(axisDiff);
	if(angle < -1) // clamp angle to [-1, 1]
		angle = -1;
	else if(angle > 1)
		angle = 1;
	if(m_angular.cross(axisDiff).dot(m_axisDir) < 0)
		angle = std::acos(-angle) + M_PI;
	else
		angle = std::acos(angle);
	// angle ok*/
	// get length from height
	//float length = height / std::cos(m_angle);
	//param->first = length;
	// this should be more precise than a division with std::cos:
	// this is for two sided cone!
	// distance to axis
	float sqrS = s.sqrLength();
	float f = sqrS - (height * height);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	float sdist = fabs(m_n2d[0] * f + ((height < 0)? -1 : 1) * m_n2d[1] * height);
	float length = std::sqrt(sqrS + sdist * sdist);
	param->first = /*(height < 0)? -length :*/ length;
	param->second = angle;
	/*// get normal for p
	Vec3f pln = s.cross(m_axisDir);
	Vec3f plx = m_axisDir.cross(pln);
	Vec3f n;
	if(plx.normalize() < 1.0e-6)
	{
		*param = std::make_pair(0.0f, angle);
		return height;
	}
	if(height < 0)
		n = m_normal[0] * plx - m_normalY;
	else
	n = m_normal[0] * plx + m_normalY;
	Vec3f l = n.cross(pln);
	l.normalize();
	// make sure l points in direction of axis
	if(m_axisDir.dot(l) < 0)
		l *= -1;
	// project p on line m_center + lambda * l
	// get lambda
	float lambda = s.dot(l);
	// make sure l points in direction of axis
	if(m_axisDir.dot(l) < 0)
	{
		if(lambda > 0)
		{
			*param = std::make_pair(s.length(), angle);
			return height;
		}
	}
	else if(lambda < 0)
	{
		*param = std::make_pair(s.length(), angle);
		return height;
	}
	*param = std::make_pair(*fabs(lambda), angle);*/
}

void Cone::RotateAngularDirection(float radians)
{
	GfxTL::Quaternion< float > q;
	q.RotationRad(radians, m_axisDir[0], m_axisDir[1], m_axisDir[2]);
	Vec3f vvec;
	q.Rotate(AngularDirection(), &vvec);
	m_hcs[0] = GfxTL::Vector3Df(vvec);
	m_hcs[1] = GfxTL::Vector3Df(m_axisDir.cross(Vec3f(m_hcs[0].Data())));
	m_angularRotatedRadians += radians;
}

//void Cone::AngularDirection(const Vec3f &angular)
//{
//	m_hcs[0] = GfxTL::Vector3Df(angular);
//	m_hcs[1] = GfxTL::Vector3Df(m_axisDir.cross(Vec3f(m_hcs[0].Data())));
//}

float ConeDistance(const float *param, const float *x)
{
	Vec3f s;
	for(unsigned int i = 0; i < 3; ++i)
		s[i] = x[i] - param[i];
	float g = fabs(s[0] * param[3] + s[1] * param[4] + s[2] * param[5]);
	float f = s.sqrLength() - (g * g);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	return std::cos(param[6]) * f - std::sin(param[6]) * g;
		//- param[7];
}

void ConeDistanceDerivatives(const float *param, const float *x,
	float *gradient)
{
	Vec3f s;
	for(unsigned int i = 0; i < 3; ++i)
		s[i] = x[i] - param[i];
	float g = fabs(s[0] * param[3] + s[1] * param[4] + s[2] * param[5]);
	float f = s.sqrLength() - (g * g);
	if(f <= 0)
		f = 0;
	else
		f = std::sqrt(f);
	float ggradient[6];
	for(unsigned int i = 0; i < 3; ++i)
		ggradient[i] = -param[i + 3];
	for(unsigned int i = 0; i < 3; ++i)
		ggradient[i + 3] = s[i] - param[i + 3] * g;
	float fgradient[6];
	if(f < 1.0e-6)
	{
		fgradient[0] = std::sqrt(1 - param[3] * param[3]);
		fgradient[1] = std::sqrt(1 - param[4] * param[4]);
		fgradient[2] = std::sqrt(1 - param[5] * param[5]);
	}
	else
	{
		fgradient[0] = (param[3] * g - s[0]) / f;
		fgradient[1] = (param[4] * g - s[1]) / f;
		fgradient[2] = (param[5] * g - s[2]) / f;
	}
	fgradient[3] = g * fgradient[0];
	fgradient[4] = g * fgradient[1];
	fgradient[5] = g * fgradient[2];
	float sinPhi = -std::sin(param[6]);
	float cosPhi = std::cos(param[6]);
	for(unsigned int i = 0; i < 6; ++i)
		gradient[i] = cosPhi * fgradient[i] + sinPhi * ggradient[i];
	gradient[6] = f * sinPhi - g * cosPhi;
	//gradient[7] = -1;
}

void NormalizeConeParams(float *param)
{
	// normalize direction
	float l = std::sqrt(param[3] * param[3] + param[4] * param[4] +
		param[5] * param[5]);
	for(unsigned int i = 3; i < 6; ++i)
		param[i] /= l;
	// normalize angle
	param[6] -= std::floor(param[6] / (2 * float(M_PI))) * (2 * float(M_PI)); // param[6] %= 2*M_PI
	if(param[6] > M_PI)
	{
		param[6] -= std::floor(param[6] / float(M_PI)) * float(M_PI); // param[6] %= M_PI
		for(unsigned int i = 3; i < 6; ++i)
			param[i] *= -1;
	}
	if(param[6] > float(M_PI) / 2)
		param[6] = float(M_PI) - param[6];
}

bool Cone::LeastSquaresFit(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	bool retVal = LeastSquaresFit(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()));
	return retVal;
}

bool Cone::Interpolate(const MiscLib::Vector< Cone > &cones,
	const MiscLib::Vector< float > &weights, Cone *ic)
{
	Vec3f center(0, 0, 0);
	Vec3f axisDir(0, 0, 0);
	float omega = 0;
	for(size_t i = 0; i < cones.size(); ++i)
	{
		center += weights[i] * cones[i].Center();
		axisDir += weights[i] * cones[i].AxisDirection();
		omega += weights[i] * cones[i].Angle();
	}
	axisDir.normalize();
	return ic->Init(center, axisDir, omega);
}

void Cone::Serialize(bool binary, std::ostream *o) const
{
	if(binary)
	{
		o->write((const char *)&m_center, sizeof(m_center));
		o->write((const char *)&m_axisDir, sizeof(m_axisDir));
		o->write((const char *)&m_angle, sizeof(m_angle));
		o->write((const char *)&m_angularRotatedRadians,
			sizeof(m_angularRotatedRadians));
	}
	else
	{
		(*o) << m_center[0] << " " << m_center[1] << " " << m_center[2] << " "
			<< m_axisDir[0] << " " << m_axisDir[1] << " " << m_axisDir[2] << " "
			<< m_angle << " " << m_angularRotatedRadians << " ";
	}
}

size_t Cone::SerializedSize()
{
	return sizeof(Vec3f)
		+ sizeof(Vec3f)
		+ sizeof(float)
		+ sizeof(float);
}

void Cone::Serialize(FILE *o) const
{
	fwrite(&m_center, sizeof(m_center), 1, o);
	fwrite(&m_axisDir, sizeof(m_axisDir), 1, o);
	fwrite(&m_angle, sizeof(m_angle), 1, o);
	fwrite(&m_angularRotatedRadians, sizeof(m_angularRotatedRadians), 1, o);
}

size_t Cone::SerializedFloatSize()
{
	return 8;
}

void Cone::Serialize(float* array) const
{
	for(int i=0;i<3;i++){
		array[i] = m_center[i];
		array[i+3] = m_axisDir[i];
	}
	array[6] = m_angle;
	array[7] = m_angularRotatedRadians;
}
	

void Cone::Transform(float scale, const Vec3f &translate)
{
	m_center *= scale;
	m_center += translate;
}

void Cone::Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
	const GfxTL::Vector3Df &trans)
{
	m_center = Vec3f((rot * GfxTL::Vector3Df(m_center) + trans).Data());
	m_axisDir = Vec3f((rot * GfxTL::Vector3Df(m_axisDir)).Data());
	m_hcs[0] = rot * m_hcs[0];
	m_hcs[1] = rot * m_hcs[1];
	m_normalY = m_normal[1] * m_axisDir;
}
