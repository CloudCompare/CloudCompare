#include "Cylinder.h"
#include "LevMarFitting.h"
#include <MiscLib/Performance.h>
#include <GfxTL/VectorXD.h>
#include "LevMarFitting.h"
#include "LevMarLSWeight.h"
#include <GfxTL/IndexedIterator.h>
#include <GfxTL/Mean.h>
#ifdef DOPARALLEL
#include <omp.h>
#endif

Cylinder::ParallelNormalsError::ParallelNormalsError()
: std::runtime_error("Parallel normals")
{}

Cylinder::Cylinder()
: m_angularRotatedRadians(0)
{}

Cylinder::Cylinder(const Vec3f &axisDir, const Vec3f &axisPos, float radius)
: m_angularRotatedRadians(0)
{
	Init(axisDir, axisPos, radius);
}

Cylinder::Cylinder(const Vec3f &pointA, const Vec3f &pointB,
	const Vec3f &normalA, const Vec3f &normalB)
: m_angularRotatedRadians(0)
{
	if(!Init(pointA, pointB, normalA, normalB))
		throw ParallelNormalsError();
}

bool Cylinder::Init(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 4)
		return false;
	// estimate axis from all pairs
	m_axisDir = Vec3f(0, 0, 0);
	size_t c = samples.size() / 2;
	m_axisDir = samples[0 + c].cross(samples[1 + c]);
	if(m_axisDir.normalize() < 1e-3)
		return false;
	m_axisPos = Vec3f(0, 0, 0);
	m_radius = 0;
	// project first normal into plane
	float l = m_axisDir.dot(samples[0 + c]);
	Vec3f xdir = samples[0 + c] - l * m_axisDir;
	xdir.normalize();
	Vec3f ydir = m_axisDir.cross(xdir);
	ydir.normalize();
	// xdir is the x axis in the plane (y = 0) samples[0] is the origin
	float lineBnx = ydir.dot(samples[1 + c]);
	if(fabs(lineBnx) < 1.0e-6)
		return false;
	float lineBny = -xdir.dot(samples[1 + c]);
	// origin of lineB
	Vec3f originB = samples[1] - samples[0];
	float lineBOx = xdir.dot(originB);
	float lineBOy = ydir.dot(originB);
	float lineBd = lineBnx * lineBOx + lineBny * lineBOy;
	// lineB in the plane complete
	// point of intersection is y = 0 and x = lineBd / lineBnx
	float radius = lineBd / lineBnx;
	m_axisPos += samples[0] + radius * xdir;
	m_radius += fabs(radius);
	m_radius += std::sqrt((radius - lineBOx) * (radius - lineBOx) + lineBOy * lineBOy);
	m_radius /= 2;
	if(m_radius > 1e6)
		return false;

	// find point on axis closest to origin
	float lambda = m_axisDir.dot(-m_axisPos);
	m_axisPos = m_axisPos + lambda * m_axisDir;

	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cylinder::InitAverage(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 4)
		return false;
	// estimate axis from covariance of normal vectors
	MiscLib::Vector< GfxTL::Vector3Df > normals;
	size_t c = samples.size() / 2;
	for(size_t i = c; i < samples.size(); ++i)
	{
		normals.push_back(GfxTL::Vector3Df(samples[i]));
		normals.push_back(GfxTL::Vector3Df(-samples[i]));
	}
	GfxTL::MatrixXX< 3, 3, float > cov, eigenVectors;
	GfxTL::Vector3Df eigenValues;
	GfxTL::CovarianceMatrix(GfxTL::Vector3Df(0, 0, 0),
		normals.begin(), normals.end(), &cov);
	GfxTL::Jacobi(cov, &eigenValues, &eigenVectors);
	// find the minimal eigenvalue and corresponding vector
	float minEigVal = eigenValues[0];
	unsigned int minEigIdx = 0;
	for(unsigned int i = 1; i < 3; ++i)
		if(eigenValues[i] < minEigVal)
		{
			minEigVal = eigenValues[i];
			minEigIdx = i;
		}
	m_axisDir = Vec3f(eigenVectors[minEigIdx]);
	// get a point on the axis from all pairs
	m_axisPos = Vec3f(0, 0, 0);
	m_radius = 0;
	size_t pointCount = 0;
	for(size_t i = 0; i < c - 1; ++i)
		for(size_t j = i + 1; j < c; ++j)
		{
			// project first normal into plane
			float l = m_axisDir.dot(samples[i + c]);
			Vec3f xdir = samples[i + c] - l * m_axisDir;
			xdir.normalize();
			Vec3f ydir = m_axisDir.cross(xdir);
			ydir.normalize();
			// xdir is the x axis in the plane (y = 0) samples[i] is the origin
			float lineBnx = ydir.dot(samples[j + c]);
			if(fabs(lineBnx) < .05f)
				continue;
			float lineBny = -xdir.dot(samples[j + c]);
			// origin of lineB
			Vec3f originB = samples[j] - samples[i];
			float lineBOx = xdir.dot(originB);
			float lineBOy = ydir.dot(originB);
			float lineBd = lineBnx * lineBOx + lineBny * lineBOy;
			// lineB in the plane complete
			// point of intersection is y = 0 and x = lineBd / lineBnx
			float radius = lineBd / lineBnx;
			m_axisPos += samples[i] + radius * xdir;
			m_radius += fabs(radius);
			m_radius += std::sqrt((radius - lineBOx) * (radius - lineBOx) + lineBOy * lineBOy);
			++pointCount;
		}
	if(!pointCount)
		return false;
	m_axisPos /= pointCount;
	m_radius /= pointCount * 2;
	if(m_radius > 1e6)
		return false;

	// find point on axis closest to origin
	float lambda = m_axisDir.dot(-m_axisPos);
	m_axisPos = m_axisPos + lambda * m_axisDir;

	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cylinder::Init(const Vec3f &axisDir, const Vec3f &axisPos, float radius)
{
	m_axisDir = axisDir;
	m_axisPos = axisPos;
	m_radius = radius;
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cylinder::Init(const Vec3f &pointA, const Vec3f &pointB,
	const Vec3f &normalA, const Vec3f &normalB)
{
	if(normalA.dot(normalB) > 0.9998477)
		return false;
	m_axisDir = normalA.cross(normalB);
	if(m_axisDir.normalize() < 1.0e-6)
		return false;
	// normalA is the x axis in the plane (y = 0) pointA is the origin
	Vec3f planeY = normalA.cross(m_axisDir); // planeX = normalA
	planeY.normalize();
	float lineBnx = planeY.dot(normalB);
	float lineBny = -normalA.dot(normalB);
	// origin of lineB
	Vec3f originB = pointB - pointA;
	float lineBOx = normalA.dot(originB);
	float lineBOy = planeY.dot(originB);
	float lineBd = lineBnx * lineBOx + lineBny * lineBOy;
	// lineB in the plane complete
	// point of intersection is y = 0 and x = lineBd / lineBnx
	m_radius = lineBd / lineBnx;
	m_axisPos = pointA + m_radius * normalA;
	m_radius = fabs(m_radius);
	if(m_radius > 1e6)
		return false;
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	return true;
}

bool Cylinder::Init(bool binary, std::istream *i)
{
	float rotate = 0;
	if(binary)
	{
		i->read((char *)&m_axisDir, sizeof(m_axisDir));
		i->read((char *)&m_axisPos, sizeof(m_axisPos));
		i->read((char *)&m_radius, sizeof(m_radius));
		i->read((char *)&rotate, sizeof(rotate));
	}
	else
	{
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_axisDir[j];
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_axisPos[j];
		(*i) >> m_radius;
		(*i) >> rotate;
	}
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	RotateAngularDirection(rotate);
	return true;
}

void Cylinder::Init(FILE *i)
{
	float rotate = 0;
	fread(&m_axisDir, sizeof(m_axisDir), 1, i);
	fread(&m_axisPos, sizeof(m_axisPos), 1, i);
	fread(&m_radius, sizeof(m_radius), 1, i);
	fread(&rotate, sizeof(rotate), 1, i);
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	RotateAngularDirection(rotate);
}

void Cylinder::Init(float *array)
{
	float rotate = 0;
	for(int i=0;i<3;i++){
		m_axisDir[i] = array[i];
		m_axisPos[i] = array[i+3];
	}
	m_radius=array[6];
	rotate=array[7];	
	
	m_hcs.FromNormal(m_axisDir);
	m_angularRotatedRadians = 0;
	RotateAngularDirection(rotate);
}

void Cylinder::Project(const Vec3f &p, Vec3f *pp) const
{
	Vec3f diff = m_axisPos - p;
	float lambda = m_axisDir.dot(diff);
	*pp = (diff - lambda * m_axisDir);
	float l = pp->length();
	*pp *= (l - m_radius) / l;
	*pp += p;
}

void Cylinder::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	Vec3f diff = p - m_axisPos;
	param->first = m_axisDir.dot(diff);
	float planex = diff.dot(m_hcs[0].Data());
	float planey = diff.dot(m_hcs[1].Data());
	float l = planex * planex + planey * planey;
	if(l > 0)
	{
		planex /= l;
		planey /= l;
	}
	param->second = std::atan2(planey, planex);
	if(param->second < 0)
		param->second += float(2 * M_PI);
}

float Cylinder::Radius() const
{
	return m_radius;
}

float &Cylinder::Radius()
{
	return m_radius;
}

const Vec3f &Cylinder::AxisDirection() const
{
	return m_axisDir;
}

Vec3f &Cylinder::AxisDirection()
{
	return m_axisDir;
}

const Vec3f &Cylinder::AxisPosition() const
{
	return m_axisPos;
}

Vec3f &Cylinder::AxisPosition()
{
	return m_axisPos;
}

const Vec3f Cylinder::AngularDirection() const
{
	return Vec3f(m_hcs[0].Data());
}

void Cylinder::RotateAngularDirection(float radians)
{
	GfxTL::Quaternion< float > q;
	q.RotationRad(radians, m_axisDir[0], m_axisDir[1], m_axisDir[2]);
	Vec3f vvec;
	q.Rotate(AngularDirection(), &vvec);
	m_hcs[0] = GfxTL::Vector3Df(vvec);
	m_hcs[1] = GfxTL::Vector3Df(m_axisDir.cross(Vec3f(m_hcs[0].Data())));
	m_angularRotatedRadians += radians;
}

float CylinderDistance(const float *param, const float *x)
{
	Vec3f s;
	for(size_t i = 0; i < 3; ++i)
		s[i] = x[i] - param[i];
	float u = param[5] * s[1] - param[4] * s[2];
	u *= u;
	float v = param[3] * s[2] - param[5] * s[0];
	u += v * v;
	v = param[4] * s[0] - param[3] * s[1];
	u += v * v;
	return std::sqrt(u) - param[6];
}

void CylinderDistanceDerivatives(const float *param, const float *x,
	float *gradient)
{
	Vec3f s;
	for(size_t i = 0; i < 3; ++i)
		s[i] = x[i] - param[i];
	float g = s[0] * x[0] + s[1] * x[1] + s[2] * x[2];
	float f = param[5] * s[1] - param[4] * s[2];
	f *= f;
	float v = param[3] * s[2] - param[5] * s[0];
	f += v * v;
	v = param[4] * s[0] - param[3] * s[1];
	f += v * v;
	f = std::sqrt(f);
	if(f < 1.0e-6)
	{
		gradient[0] = std::sqrt(1 - param[3] * param[3]);
		gradient[1] = std::sqrt(1 - param[4] * param[4]);
		gradient[2] = std::sqrt(1 - param[5] * param[5]);
	}
	else
	{
		gradient[0] = (param[3] * g - s[0]) / f;
		gradient[1] = (param[4] * g - s[1]) / f;
		gradient[2] = (param[5] * g - s[2]) / f;
	}
	gradient[3] = g * gradient[0];
	gradient[4] = g * gradient[1];
	gradient[5] = g * gradient[2];
	gradient[6] = -1;
}

void NormalizeCylinderParams(float *param)
{
	float l = std::sqrt(param[3] * param[3] + param[4] * param[4]
		+ param[5] * param[5]);
	for(unsigned int i = 3; i < 6; ++i)
		param[i] /= l;
	// find point on axis closest to origin
    float lambda = -(param[0] * param[3] + param[1] * param[4] +
		param[2] * param[5]);
	for(unsigned int i = 0; i < 3; ++i)
		param[i] = param[i] + lambda * param[i + 3];
}

bool Cylinder::LeastSquaresFit(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	bool retVal = LeastSquaresFit(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()));
	return retVal;
}

bool Cylinder::Interpolate(const MiscLib::Vector< Cylinder > &cylinders,
	const MiscLib::Vector< float > &weights, Cylinder *ic)
{
	Vec3f axisPos(0, 0, 0);
	Vec3f axisDir(0, 0, 0);
	float r = 0;
	for(size_t i = 0; i < cylinders.size(); ++i)
	{
		axisPos += weights[i] * cylinders[i].AxisPosition();
		axisDir += weights[i] * cylinders[i].AxisDirection();
		r += weights[i] * cylinders[i].Radius();
	}
	axisDir.normalize();
	return ic->Init(axisDir, axisPos, r);
}

void Cylinder::Serialize(bool binary, std::ostream *o) const
{
	if(binary)
	{
		o->write((const char *)&m_axisDir, sizeof(m_axisDir));
		o->write((const char *)&m_axisPos, sizeof(m_axisPos));
		o->write((const char *)&m_radius, sizeof(m_radius));
		o->write((const char *)&m_angularRotatedRadians,
			sizeof(m_angularRotatedRadians));
	}
	else
	{
		(*o) << m_axisDir[0] << " " << m_axisDir[1] << " " << m_axisDir[2] << " "
			<< m_axisPos[0] << " " << m_axisPos[1] << " " << m_axisPos[2] << " "
			<< m_radius << " " << m_angularRotatedRadians << " ";
	}
}

size_t Cylinder::SerializedSize()
{
	return sizeof(Vec3f)
		+ sizeof(Vec3f)
		+ sizeof(float)
		+ sizeof(float);
}

void Cylinder::Serialize(float *array) const
{

	for(int i=0;i<3;i++){
		array[i] = m_axisDir[i];
		array[i+3] = m_axisPos[i];
	}
	array[6] = m_radius;
	array[7] = m_angularRotatedRadians;
}

size_t Cylinder::SerializedFloatSize()
{
	return 8;
}

void Cylinder::Serialize(FILE *o) const
{
	fwrite(&m_axisDir, sizeof(m_axisDir), 1, o);
	fwrite(&m_axisPos, sizeof(m_axisPos), 1, o);
	fwrite(&m_radius, sizeof(m_radius), 1, o);
	fwrite(&m_angularRotatedRadians,
		sizeof(m_angularRotatedRadians), 1, o);
}

void Cylinder::Transform(float scale, const Vec3f &translate)
{
	m_axisPos *= scale;
	m_axisPos += translate;
	m_radius *= scale;
}

void Cylinder::Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
	const GfxTL::Vector3Df &trans)
{
	m_axisDir = Vec3f((rot * GfxTL::Vector3Df(m_axisDir)).Data());
	m_axisPos = Vec3f((rot * GfxTL::Vector3Df(m_axisPos) + trans).Data());
	m_hcs[0] = rot * m_hcs[0];
	m_hcs[1] = rot * m_hcs[1];
}
