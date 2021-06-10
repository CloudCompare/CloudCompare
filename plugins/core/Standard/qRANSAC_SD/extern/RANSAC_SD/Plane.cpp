#include "Plane.h"
//#include "pca.h"
#include <MiscLib/Performance.h>
#include "LevMarFitting.h"
#include <GfxTL/VectorXD.h>
#include <GfxTL/IndexedIterator.h>
#include <GfxTL/Plane.h>
#include <GfxTL/Mean.h>

Plane::Plane(Vec3f p1, Vec3f p2, Vec3f p3)
{
   m_normal= (p2-p1).cross(p3-p2);
   m_normal.normalize();		
   m_pos = p1;
   m_dist = m_pos.dot(m_normal);
}

Plane::Plane(Vec3f p1, Vec3f normal)
{
   m_normal = normal;
   m_pos = p1;
   m_dist = m_pos.dot(m_normal);
}

Plane::~Plane()
{
}

bool Plane::Init(Vec3f p1, Vec3f p2, Vec3f p3)
{
	m_normal= (p2-p1).cross(p3-p2);
	if(m_normal.sqrLength() < 1E-6f)
		return false;
	m_normal.normalize();		
	m_pos = p1;
	m_dist = m_pos.dot(m_normal);
	return true;
}

bool Plane::Init(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 6)
		return false;
	return Init(samples[0], samples[1], samples[2]);
}

bool Plane::InitAverage(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 1)
		return false;
	m_normal = Vec3f(0, 0, 0);
	m_pos = Vec3f(0, 0, 0);
	size_t c = samples.size() / 2;
	MiscLib::Vector< GfxTL::Vector3Df > normals(c);
	for(size_t i = 0; i < c; ++i)
		normals[i] = GfxTL::Vector3Df(samples[i + c]);
	GfxTL::Vector3Df meanNormal;
	GfxTL::MeanOfNormals(normals.begin(), normals.end(), &meanNormal);
	m_normal = Vec3f(meanNormal.Data());
	GfxTL::Vector3Df mean;
	GfxTL::Mean(samples.begin(), samples.begin() + c, &mean);
	m_pos = Vec3f(mean.Data());
	m_dist = m_pos.dot(m_normal);
	return true;
}

bool Plane::Init(bool binary, std::istream *i)
{
	if(binary)
	{
		i->read((char *)&m_normal, sizeof(m_normal));
		i->read((char *)&m_dist, sizeof(m_dist));
		i->read((char *)&m_pos, sizeof(m_pos));
	}
	else
	{
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_normal[j];
		(*i) >> m_dist;
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_pos[j];
	}
	return true;
}

void Plane::Init(float* array){
	for(int i=0;i<3;i++){
		m_normal[i] = array[i];
		m_pos[i] = array[i+4];
	}
	m_dist=array[3];
}

void Plane::Init(FILE *i)
{
	size_t readrtn; //unused return warning suppresion
	readrtn = fread(&m_normal, sizeof(m_normal), 1, i);
	readrtn = fread(&m_dist, sizeof(m_dist), 1, i);
	readrtn = fread(&m_pos, sizeof(m_pos), 1, i);
}

bool Plane::equals (Plane other)
{
   return ( (other.getNormal().dot (getNormal() ) > 0.90) &&
        (getDistance (other.getPosition()) < 0.2) );   
}

template< class WeightT >
class LevMarPlane
: public WeightT
{
public:
	enum { NumParams = 4 };
	typedef float ScalarType;

	LevMarPlane()
	{}
	// parametrization:
	// params[0] - params[2] = normal vector
	// params[3] = dist to origin

	template< class IteratorT >
	ScalarType Chi(const ScalarType *params, IteratorT begin, IteratorT end,
		ScalarType *values, ScalarType *temp) const
	{
		ScalarType chi = 0;
		int size = end - begin;
#ifdef DOPARALLEL
		#pragma omp parallel for schedule(static) reduction(+:chi)
#endif
		for(int idx = 0; idx < size; ++idx)
		{
			temp[idx] = params[0] * begin[idx][0] + params[1] * begin[idx][1]
				+ params[2] * begin[idx][2] - params[3];
			chi += (values[idx] = WeightT::Weigh(temp[idx]))
				* values[idx];
		}
		return chi;
	}

	template< class IteratorT >
	void Derivatives(const ScalarType *params, IteratorT begin, IteratorT end,
		const ScalarType *values, const ScalarType *temp, ScalarType *matrix) const
	{
		int size = end - begin;
#ifdef DOPARALLEL
		#pragma omp parallel for schedule(static)
#endif
		for(int idx = 0; idx < size; ++idx)
		{
			matrix[idx * NumParams + 0] = begin[idx][0];
			matrix[idx * NumParams + 1] = begin[idx][1];
			matrix[idx * NumParams + 2] = begin[idx][2];
			matrix[idx * NumParams + 3] = -1;
			WeightT::template DerivWeigh< NumParams >(temp[idx],
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

bool Plane::LeastSquaresFit(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	LeastSquaresFit(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()));
	return true;
}

bool Plane::Interpolate(const MiscLib::Vector< Plane > &planes,
	const MiscLib::Vector< float > &weights, Plane *ip)
{
	Vec3f normal(0, 0, 0);
	Vec3f position(0, 0, 0);
	for(size_t i = 0; i < planes.size(); ++i)
	{
		normal += weights[i] * planes[i].getNormal();
		position += weights[i] * planes[i].getPosition();
	}
	normal.normalize();
	*ip = Plane(position, normal);
	return true;
}

void Plane::Serialize(bool binary, std::ostream *o) const
{
	if(binary)
	{
		o->write((const char *)&m_normal, sizeof(m_normal));
		o->write((const char *)&m_dist, sizeof(m_dist));
		o->write((const char *)&m_pos, sizeof(m_pos));
	}
	else
#ifdef DIRK_FORMAT
	{
		(*o) << std::endl 
			<< "n " << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << std::endl 
			<< "d " << m_dist << std::endl
			<< "p ";
		for(size_t i = 0; i < 3; ++i)
			(*o) << m_pos[i] << " ";
		(*o) << std::endl;
	}
#else
	{
		(*o) << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << " "
			<< m_dist << " ";
		for(size_t i = 0; i < 3; ++i)
			(*o) << m_pos[i] << " ";
	}
#endif
}

size_t Plane::SerializedSize()
{
	return sizeof(Vec3f)
		+ sizeof(float)
		+ sizeof(Vec3f);
}

size_t Plane::SerializedFloatSize()
{
	return 7;
}

void Plane::Serialize(float* array) const
{
	for(int i=0;i<3;i++){
		array[i] = m_normal[i];
		array[i+4] = m_pos[i];
	}
	array[3] = m_dist;
}


void Plane::Serialize(FILE *o) const
{
	fwrite(&m_normal, sizeof(m_normal), 1, o);
	fwrite(&m_dist, sizeof(m_dist), 1, o);
	fwrite(&m_pos, sizeof(m_pos), 1, o);
}

void Plane::Transform(float scale, const Vec3f &translate)
{
	m_pos *= scale;
	m_pos += translate;
	m_dist = m_pos.dot(m_normal);
}

float Plane::Intersect(const Vec3f &p, const Vec3f &r) const
{
	return -SignedDistance(p) / (m_normal.dot(r));
}
