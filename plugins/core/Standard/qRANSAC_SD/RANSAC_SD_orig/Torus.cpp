#include "Torus.h"
#include <GfxTL/VectorXD.h>
#include <limits>
#include <iterator>
#include "LevMarFitting.h"
#include <MiscLib/Performance.h>
#include <GfxTL/IndexedIterator.h>
#include "LevMarLSWeight.h"
#ifdef DOPARALLEL
#include <omp.h>
#endif

template< class InIteratorT, class OutIteratorT >
static void SpinImage(const Vec3f &axisPos, const Vec3f &axisDir,
	InIteratorT begin, InIteratorT end, OutIteratorT out)
{
	for(; begin != end; ++begin)
	{
		Vec3f s = *begin - axisPos;
		GfxTL::Vector2Df spin;
		spin[1] = s.dot(axisDir);
		spin[0] = (s - spin[1] * axisDir).length();
		*out = spin;
	}
}

template< class InIteratorT >
static bool CircleFrom3Points(InIteratorT i, float *r,
	GfxTL::Vector2Df *center)
{
	float a;
	float b;
	float bot;
	float c;
	float top1;
	float top2;
	a = std::sqrt(std::pow(i[1][0] - i[0][0], 2)
			+ std::pow(i[1][1] - i[0][1], 2));
	b = std::sqrt(std::pow(i[2][0] - i[1][0], 2)
			+ std::pow(i[2][1] - i[1][1], 2));
	c = std::sqrt(std::pow(i[0][0] - i[2][0], 2)
			+ std::pow(i[0][1] - i[2][1], 2));

	bot = (a + b + c) * (-a + b + c) * (a - b + c) * (a + b - c);

	if(bot <= 0.f)
		return false;

	*r = a * b * c / std::sqrt(bot);
	//  center.
	top1 = (i[1][1] - i[0][1]) * c * c - (i[2][1] - i[0][1]) * a * a;
	top2 = (i[1][0] - i[0][0]) * c * c - (i[2][0] - i[0][0]) * a * a;
	bot = (i[1][1] - i[0][1]) * (i[2][0] - i[0][0])  
		- (i[2][1] - i[0][1]) * (i[1][0] - i[0][0]);

	(*center)[0] = i[0][0] + 0.5f * top1 / bot;
	(*center)[1] = i[0][1] - 0.5f * top2 / bot;
	return true;
}

// this function estimates the torus from four samples
bool Torus::Init(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 8)
		return false;

	// the algorithm used is as follows:
	// 1. The axis of rotation is estimated
	// 2. A spin image of the samples around the axis is constructed
	// 3. A circle is fitted to the samples in the spin image

	// 1. Axis of rotation
	// The formula used can be found in "Geometric least-squares fitting of
	// spheres, cylinders, cones and tori" by Lukacs, Marshall and Martin 1997
	// solve quadratic equation
	size_t k = samples.size() >> 1;
	MiscLib::Vector< GfxTL::Vector3Dd > dsamples;
	dsamples.reserve(samples.size());
	for(size_t i = 0; i < samples.size(); ++i)
		dsamples.push_back(GfxTL::Vector3Dd(samples[i][0], samples[i][1],
			samples[i][2]));
	GfxTL::Vector3Dd n0xn1 = dsamples[k] % dsamples[k + 1];
	double a01 = n0xn1 * dsamples[k + 2];
	double b01 = n0xn1 * dsamples[k + 3];
	double a0 = ((dsamples[2] - dsamples[1])
		% dsamples[k]) * dsamples[k + 2];
	double a1 = ((dsamples[0] - dsamples[2])
		% dsamples[k + 1]) * dsamples[k + 2];
	double a = ((dsamples[0] - dsamples[2])
		% (dsamples[1] - dsamples[0])) * dsamples[k + 2];
	double b0 = ((dsamples[3] - dsamples[1])
		% dsamples[k]) * dsamples[k + 3];
	double b1 = ((dsamples[0] - dsamples[3])
		% dsamples[k + 1]) * dsamples[k + 3];
	double b = ((dsamples[0] - dsamples[3])
		% (dsamples[1] - dsamples[0])) * dsamples[k + 3];
	double cc = b01 / a01;
	double ccc = b0 - a0 * cc;
	double c = -(b1 - a1 * cc) / ccc;
	double d = (-b + a * cc) / ccc;
	double p = (a0 * c + a1 + a01 * d) / (2 * a01 * c);
	double q = (a + a0 * d) / (a01 * c);
	double rt = p * p - q;
	if(rt < -1e-8)
		return false;
	if(rt < 0)
		rt = 0;
	float t1 = static_cast<float>(-p + std::sqrt(rt));
	float t2 = static_cast<float>(-p - std::sqrt(rt));
	float s1 = static_cast<float>(c * t1 + d);
	float s2 = static_cast<float>(c * t2 + d);

	Vec3f pos1 = samples[0] + s1 * samples[k];
	Vec3f normal1 = pos1 - (samples[1] + t1 * samples[k + 1]);
	normal1.normalize();
	Vec3f pos2 = samples[0] + s2 * samples[k];
	Vec3f normal2 = pos2 - (samples[1] + t2 * samples[k + 1]);
	normal2.normalize();

	// at this point there are two possible solutions for the axis
	MiscLib::Vector< GfxTL::Vector2Df > spin1, spin2;
	SpinImage(pos1, normal1, samples.begin(), samples.begin() + k,
		std::back_inserter(spin1));
	SpinImage(pos2, normal2, samples.begin(), samples.begin() + k,
		std::back_inserter(spin2));
	float minorRadius1, majorRadius1, minorRadius2, majorRadius2,
		distSum1 = std::numeric_limits< float >::infinity(),
		distSum2 = std::numeric_limits< float >::infinity(),
		tmp;
	GfxTL::Vector2Df minorCenter1, minorCenter2;
	if(CircleFrom3Points(spin1.begin(), &minorRadius1, &minorCenter1))
	{
		majorRadius1 = minorCenter1[0];
		// compute the distance of the points to the torus
		distSum1 = 0;
		for (size_t i = 3; i < spin1.size(); ++i)
		{
			tmp = ((spin1[i] - minorCenter1).Length() - minorRadius1);
			distSum1 += tmp * tmp;
		}
	}
	if(CircleFrom3Points(spin2.begin(), &minorRadius2, &minorCenter2))
	{
		majorRadius2 = minorCenter2[0];
		// compute the distance of the points to the torus
		distSum2 = 0;
		for (size_t i = 3; i < spin2.size(); ++i)
		{
			tmp = ((spin2[i] - minorCenter2).Length() - minorRadius2);
			distSum2 += tmp * tmp;
		}
	}
	if(distSum1 != std::numeric_limits< float >::infinity()
		&& distSum1 < distSum2)
	{
		m_normal = normal1;
		m_rminor = minorRadius1;
		m_rmajor = majorRadius1;
		m_center = pos1 + minorCenter1[1] * m_normal;
	}
	else if(distSum2 != std::numeric_limits< float >::infinity())
	{
		m_normal = normal2;
		m_rminor = minorRadius2;
		m_rmajor = majorRadius2;
		m_center = pos2 + minorCenter2[1] * m_normal;
	}
	else
		return false;
	m_appleShaped = m_rmajor < m_rminor;
	ComputeAppleParams();
	return true;
}

bool Torus::InitAverage(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 8)
		return false;

	// the algorithm used is as follows:
	// 1. The axis of rotation is estimated
	// 2. A spin image of the samples around the axis is constructed
	// 3. A circle is fitted to the samples in the spin image

	// 1. Axis of rotation
	// The formula used can be found in "Geometric least-squares fitting of
	// spheres, cylinders, cones and tori" by Lukacs, Marshall and Martin 1997
	// solve quadratic equation
	size_t k = samples.size() >> 1;
	MiscLib::Vector< GfxTL::Vector3Dd > dsamples;
	dsamples.reserve(samples.size());
	for(size_t i = 0; i < samples.size(); ++i)
		dsamples.push_back(GfxTL::Vector3Dd(samples[i][0], samples[i][1],
			samples[i][2]));
	// run over all four tuples until two axes are succesfully
	// established
	Vec3f pos1, normal1, pos2, normal2;
	for(size_t w = 0; w < k/* - 3*/; ++w)
	{
		for(size_t x = 0/*w + 1*/; x < k/* - 2*/; ++x)
		{
			GfxTL::Vector3Dd n0xn1 = dsamples[k + w] % dsamples[k + x];
			for(size_t y = 0/*x + 1*/; y < k/* - 1*/; ++y)
			{
				for(size_t z = 0/*y + 1*/; z < k; ++z)
				{
					double a01 = n0xn1 * dsamples[k + y];
					double b01 = n0xn1 * dsamples[k + z];
					if(GfxTL::Math< double >::Abs(a01) < 1.0e-6
						|| GfxTL::Math< double >::Abs(b01) < 1.0e-6)
							continue;
					double a0 = ((dsamples[y] - dsamples[x])
						% dsamples[k + w]) * dsamples[k + y];
					double a1 = ((dsamples[w] - dsamples[y])
						% dsamples[k + x]) * dsamples[k + y];
					double a = ((dsamples[w] - dsamples[y])
						% (dsamples[x] - dsamples[w])) * dsamples[k + y];
					double b0 = ((dsamples[z] - dsamples[x])
						% dsamples[k + w]) * dsamples[k + z];
					double b1 = ((dsamples[w] - dsamples[z])
						% dsamples[k + x]) * dsamples[k + z];
					double b = ((dsamples[w] - dsamples[z])
						% (dsamples[x] - dsamples[w])) * dsamples[k + z];
					double cc = b01 / a01;
					double ccc = b0 - a0 * cc;
					double c = -(b1 - a1 * cc) / ccc;
					double d = (-b + a * cc) / ccc;
					double p = (a0 * c + a1 + a01 * d) / (2 * a01 * c);
					double q = (a + a0 * d) / (a01 * c);
					double rt = p * p - q;
					if(rt < -1e-8)
						continue;
					if(rt < 0)
						rt = 0;
					float t1 = static_cast<float>(-p + std::sqrt(rt));
					float t2 = static_cast<float>(-p - std::sqrt(rt));
					float s1 = static_cast<float>(c * t1 + d);
					float s2 = static_cast<float>(c * t2 + d);
					pos1 = samples[w] + s1 * samples[k + w];
					normal1 = pos1 - (samples[x] + t1 * samples[k + x]);
					normal1.normalize();
					pos2 = samples[w] + s2 * samples[k + w];
					normal2 = pos2 - (samples[x] + t2 * samples[k + x]);
					normal2.normalize();
					goto foundAxis;
				}
			}
		}
	}
	return false;

foundAxis:
	// at this point there are two possible solutions for the axis
	MiscLib::Vector< GfxTL::Vector2Df > spin1, spin2;
	SpinImage(pos1, normal1, samples.begin(), samples.begin() + k,
		std::back_inserter(spin1));
	SpinImage(pos2, normal2, samples.begin(), samples.begin() + k,
		std::back_inserter(spin2));
	float minorRadius1, majorRadius1, minorRadius2, majorRadius2,
		distSum1 = std::numeric_limits< float >::infinity(),
		distSum2 = std::numeric_limits< float >::infinity(),
		tmp;
	GfxTL::Vector2Df minorCenter1, minorCenter2;
	if(CircleFrom3Points(spin1.begin(), &minorRadius1, &minorCenter1))
	{
		majorRadius1 = minorCenter1[0];
		// compute the distance of the points to the torus
		distSum1 = 0;
		for (size_t i = 3; i < spin1.size(); ++i)
		{
			tmp = ((spin1[i] - minorCenter1).Length() - minorRadius1);
			distSum1 += tmp * tmp;
		}
	}
	if(CircleFrom3Points(spin2.begin(), &minorRadius2, &minorCenter2))
	{
		majorRadius2 = minorCenter2[0];
		// compute the distance of the points to the torus
		distSum2 = 0;
		for (size_t i = 3; i < spin2.size(); ++i)
		{
			tmp = ((spin2[i] - minorCenter2).Length() - minorRadius2);
			distSum2 += tmp * tmp;
		}
	}
	if(distSum1 != std::numeric_limits< float >::infinity()
		&& distSum1 < distSum2)
	{
		m_normal = normal1;
		m_rminor = minorRadius1;
		m_rmajor = majorRadius1;
		m_center = pos1 + minorCenter1[1] * m_normal;
	}
	else if(distSum2 != std::numeric_limits< float >::infinity())
	{
		m_normal = normal2;
		m_rminor = minorRadius2;
		m_rmajor = majorRadius2;
		m_center = pos2 + minorCenter2[1] * m_normal;
	}
	else
		return false;
	m_appleShaped = m_rmajor < m_rminor;
	ComputeAppleParams();
	return true;
}

bool Torus::Init(bool binary, std::istream *i)
{
	if(binary)
	{
		i->read((char *)&m_normal, sizeof(m_center));
		i->read((char *)&m_center, sizeof(m_center));
		i->read((char *)&m_rminor, sizeof(m_rminor));
		i->read((char *)&m_rmajor, sizeof(m_rmajor));
	}
	else
	{
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_normal[j];
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_center[j];
		(*i) >> m_rminor;
		(*i) >> m_rmajor;
	}
	m_appleShaped = m_rmajor < m_rminor;
	ComputeAppleParams();
	return true;
}

void Torus::Init(FILE *i)
{
	float rot; // dummy rotation placeholder
	size_t readrtn; //unused return warning suppresion
	readrtn = fread(&m_normal, sizeof(m_normal), 1, i);
	readrtn = fread(&m_center, sizeof(m_center), 1, i);
	readrtn = fread(&m_rminor, sizeof(m_rminor), 1, i);
	readrtn = fread(&m_rmajor, sizeof(m_rmajor), 1, i);
	readrtn = fread(&rot, sizeof(rot), 1, i);
	m_appleShaped = m_rmajor < m_rminor;
	ComputeAppleParams();
}

void Torus::Init(float* array)
{
	for(int i=0;i<3;i++){
		m_normal[i] = array[i];
		m_center[i] = array[i+3];
	}
	m_rminor=array[6];
	m_rmajor=array[7];
	m_appleShaped = m_rmajor < m_rminor;
	ComputeAppleParams();
}

void Torus::Transform(float scale,	const Vec3f &translate)
{
	m_rmajor *= scale;
	m_rminor *= scale;
	m_center += translate;
}

float TorusDistance(const float *param,	const float *x)
{
	Vec3f s;
	s[0] = x[0] - param[0];
	s[1] = x[1] - param[1];
	s[2] = x[2] - param[2];
	float g = s[0] * param[3] + s[1] * param[4] + s[2] * param[5];
	float f = param[5] * s[1] - param[4] * s[2];
	f *= f;
	float v = param[3] * s[2] - param[5] * s[0];
	f += v * v;
	v = param[4] * s[0] - param[3] * s[1];
	f += v * v;
	f = std::sqrt(f);
	float tmp;
	tmp = (f - param[6]);
	return std::sqrt(g * g + (tmp * tmp)) - param[7];
}

void TorusDistanceDerivatives(const float *param, const float *x,
	float *gradient)
{
	Vec3f s;
	s[0] = x[0] - param[0];
	s[1] = x[1] - param[1];
	s[2] = x[2] - param[2];
	float g = s[0] * param[3] + s[1] * param[4] + s[2] * param[5];
	float f = param[5] * s[1] - param[4] * s[2];
	f *= f;
	float v = param[3] * s[2] - param[5] * s[0];
	f += v * v;
	v = param[4] * s[0] - param[3] * s[1];
	f += v * v;
	f = std::sqrt(f);
	float dg[6];
	dg[0] = -param[3];
	dg[1] = -param[4];
	dg[2] = -param[5];
	dg[3] = s[0] - param[3] * g;
	dg[4] = s[1] - param[4] * g;
	dg[5] = s[2] - param[5] * g;
	float df[6];
	df[0] = (param[3] * g - s[0]) / f;
	df[1] = (param[4] * g - s[1]) / f;
	df[2] = (param[5] * g - s[2]) / f;
	df[3] = g * df[0];
	df[4] = g * df[1];
	df[5] = g * df[2];
	float tmp;
	tmp = (f - param[6]);
	float d = std::sqrt(g * g + (tmp * tmp)) - param[7];
	float dr = d + param[7];
	float fr = f - param[6];
	for(unsigned int i = 0; i < 6; ++i)
		gradient[i] = (g * dg[i] + fr * df[i]) / dr;
	gradient[6] = -fr * dr;
	gradient[7] = -1;
}

void NormalizeTorusParams(float *param)
{
	float l = std::sqrt(param[3] * param[3] + param[4] * param[4]
		+ param[5] * param[5]);
	for(unsigned int i = 3; i < 6; ++i)
		param[i] /= l;
}

template< class WeightT >
class LevMarTorus
: public WeightT
{
public:
	enum { NumParams = 8 };
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
			s[0] = begin[idx][0] - params[0];
			s[1] = begin[idx][1] - params[1];
			s[2] = begin[idx][2] - params[2];
			ScalarType g = s[0] * params[3] + s[1] * params[4] + s[2] * params[5];
			ScalarType f = params[5] * s[1] - params[4] * s[2];
			f *= f;
			ScalarType v = params[3] * s[2] - params[5] * s[0];
			f += v * v;
			v = params[4] * s[0] - params[3] * s[1];
			f += v * v;
			f = std::sqrt(f);
			temp[idx] = f;
			ScalarType tmp;
			tmp = (f - params[6]);
			chi += (values[idx] = WeightT::Weigh(
				std::sqrt(g * g + (tmp * tmp)) - params[7]))
				* values[idx];;
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
			s[0] = begin[idx][0] - params[0];
			s[1] = begin[idx][1] - params[1];
			s[2] = begin[idx][2] - params[2];
			ScalarType g = s[0] * params[3] + s[1] * params[4]
				+ s[2] * params[5];
			ScalarType dg[6];
			dg[0] = -params[3];
			dg[1] = -params[4];
			dg[2] = -params[5];
			dg[3] = s[0] - params[3] * g;
			dg[4] = s[1] - params[4] * g;
			dg[5] = s[2] - params[5] * g;
			ScalarType df[6];
			df[0] = (params[3] * g - s[0]) / temp[idx];
			df[1] = (params[4] * g - s[1]) / temp[idx];
			df[2] = (params[5] * g - s[2]) / temp[idx];
			df[3] = g * df[0];
			df[4] = g * df[1];
			df[5] = g * df[2];
			ScalarType tmp;
			tmp = (temp[idx] - params[6]);
			ScalarType d = std::sqrt(g * g + (tmp * tmp)) - params[7];
			ScalarType dr = d + params[7];
			ScalarType fr = temp[idx] - params[6];
			for(unsigned int j = 0; j < 6; ++j)
				matrix[idx * NumParams + j] = (g * dg[j] + fr * df[j]) / dr;
			matrix[idx * NumParams + 6] = -fr * dr;
			matrix[idx * NumParams + 7] = -1;
			WeightT::template DerivWeigh< NumParams >(d, matrix + idx * NumParams);
		}
	}

	void Normalize(float *params) const
	{
		ScalarType l = std::sqrt(params[3] * params[3] + params[4] * params[4]
			+ params[5] * params[5]);
		for(unsigned int i = 3; i < 6; ++i)
			params[i] /= l;
	}
};

bool Torus::LeastSquaresFit(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	float param[8];
	for(size_t i = 0; i < 3; ++i)
		param[i] = m_center[i];
	for(size_t i = 0; i < 3; ++i)
		param[i + 3] = m_normal[i];
	param[6] = m_rmajor;
	param[7] = m_rminor;
	LevMarTorus< LevMarLSWeight > levMarTorus;
	if(!LevMar(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()), levMarTorus, param))
	{
		return false;
	}
	for(size_t i = 0; i < 3; ++i)
		m_center[i] = param[i];
	for(size_t i = 0; i < 3; ++i)
		m_normal[i] = param[i + 3];
	m_rmajor = param[6];
	m_rminor = param[7];
	m_appleShaped = m_rmajor < m_rminor;
	ComputeAppleParams();
	return true;
}

void Torus::Serialize(bool binary, std::ostream *o) const
{
	if(binary)
	{
		o->write((const char *)&m_normal, sizeof(m_normal));
		o->write((const char *)&m_center, sizeof(m_center));
		o->write((const char *)&m_rminor, sizeof(m_rminor));
		o->write((const char *)&m_rmajor, sizeof(m_rmajor));
	}
	else
	{
		(*o) << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << " "
			<< m_center[0] << " " << m_center[1] << " " << m_center[2] << " "
			<< m_rminor << " " << m_rmajor << " ";
	}
}

size_t Torus::SerializedSize()
{
	return sizeof(Vec3f)
		+ sizeof(Vec3f)
		+ sizeof(float)
		+ sizeof(float);
}

void Torus::Serialize(FILE *o) const
{
	fwrite(&m_normal, sizeof(m_normal), 1, o);
	fwrite(&m_center, sizeof(m_center), 1, o);
	fwrite(&m_rminor, sizeof(m_rminor), 1, o);
	fwrite(&m_rmajor, sizeof(m_rmajor), 1, o);
}

size_t Torus::SerializedFloatSize()
{
	return 8;
}

void Torus::Serialize(float *array) const
{
	for(int i=0;i<3;i++){
		array[i] = m_normal[i];
		array[i+3] = m_center[i];
	}
	array[6] = m_rminor;
	array[7] = m_rmajor;
}


void Torus::ComputeAppleParams()
{
	if(!m_appleShaped)
	{
		m_cutOffAngle = static_cast<float>(M_PI);
		m_appleHeight = 0;
		return;
	}
	m_cutOffAngle = static_cast<float>(std::acos((2 * m_rmajor - m_rminor) / m_rminor) + M_PI / 2);
	m_appleHeight = std::sin(m_cutOffAngle) * m_rminor;
}
