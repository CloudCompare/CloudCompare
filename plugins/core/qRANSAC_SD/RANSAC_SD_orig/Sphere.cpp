#include "Sphere.h"
#include <MiscLib/Performance.h>
#include <GfxTL/IndexedIterator.h>
#include <GfxTL/MathHelper.h>
#ifdef DOPARALLEL
#include <omp.h>
#endif

extern int dmat_solve ( int n, int rhs_num, double a[] );

void tetrahedron_circumsphere_3d ( double tetra[3*4], double *r, double pc[3] )

//******************************************************************************
//
//  Purpose:
//
//    TETRAHEDRON_CIRCUMSPHERE_3D computes the circumsphere of a tetrahedron in 3D.
//
//  Discussion:
//
//    The circumsphere, or circumscribed sphere, of a tetrahedron is the sphere that
//    passes through the four vertices.  The circumsphere is not necessarily
//    the smallest sphere that contains the tetrahedron.
//
//    Surprisingly, the diameter of the sphere can be found by solving
//    a 3 by 3 linear system.  This is because the vectors P2 - P1,
//    P3 - P1 and P4 - P1 are secants of the sphere, and each forms a
//    right triangle with the diameter through P1.  Hence, the dot product of
//    P2 - P1 with that diameter is equal to the square of the length
//    of P2 - P1, and similarly for P3 - P1 and P4 - P1.  This determines
//    the diameter vector originating at P1, and hence the radius and
//    center.
//
//  Modified:
//
//    10 August 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Adrian Bowyer and John Woodwark,
//    A Programmer's Geometry,
//    Butterworths, 1983.
//
//  Parameters:
//
//    Input, double TETRA[3*4], the vertices of the tetrahedron.
//
//    Output, double *R, PC[3], the coordinates of the center of the
//    circumscribed sphere, and its radius.  If the linear system is
//    singular, then R = -1, PC[] = 0.
//
{
# define DIM_NUM 3
# define RHS_NUM 1

  double a[DIM_NUM*(DIM_NUM+RHS_NUM)];
  int info;
//
//  Set up the linear system.
//
  a[0+0*3] = tetra[0+1*3] - tetra[0+0*3];
  a[0+1*3] = tetra[1+1*3] - tetra[1+0*3];
  a[0+2*3] = tetra[2+1*3] - tetra[2+0*3];
  a[0+3*3] = std::pow ( tetra[0+1*3] - tetra[0+0*3], 2 ) 
           + std::pow ( tetra[1+1*3] - tetra[1+0*3], 2 ) 
           + std::pow ( tetra[2+1*3] - tetra[2+0*3], 2 );

  a[1+0*3] = tetra[0+2*3] - tetra[0+0*3];
  a[1+1*3] = tetra[1+2*3] - tetra[1+0*3];
  a[1+2*3] = tetra[2+2*3] - tetra[2+0*3];
  a[1+3*3] = std::pow ( tetra[0+2*3] - tetra[0+0*3], 2 ) 
           + std::pow ( tetra[1+2*3] - tetra[1+0*3], 2 ) 
           + std::pow ( tetra[2+2*3] - tetra[2+0*3], 2 );

  a[2+0*3] = tetra[0+3*3] - tetra[0+0*3];
  a[2+1*3] = tetra[1+3*3] - tetra[1+0*3];
  a[2+2*3] = tetra[2+3*3] - tetra[2+0*3];
  a[2+3*3] = std::pow ( tetra[0+3*3] - tetra[0+0*3], 2 ) 
           + std::pow ( tetra[1+3*3] - tetra[1+0*3], 2 ) 
           + std::pow ( tetra[2+3*3] - tetra[2+0*3], 2 );
//
//  Solve the linear system.
//
  info = dmat_solve ( DIM_NUM, RHS_NUM, a );
//
//  If the system was singular, return a consolation prize.
//
  if ( info != 0 )
  {
    *r = -1.0;
	for(size_t i = 0; i < DIM_NUM; ++i)
		pc[i] = 0;
    //dvec_zero ( DIM_NUM, pc );
    return;
  }
//
//  Compute the radius and center.
//
  *r = 0.5 * std::sqrt 
    ( a[0+3*3] * a[0+3*3] 
    + a[1+3*3] * a[1+3*3] 
    + a[2+3*3] * a[2+3*3] );

  pc[0] = tetra[0+0*3] + 0.5 * a[0+3*3];
  pc[1] = tetra[1+0*3] + 0.5 * a[1+3*3];
  pc[2] = tetra[2+0*3] + 0.5 * a[2+3*3];

  return;
# undef DIM_NUM
# undef RHS_NUM
}

bool Midpoint(const Vec3f &p1, const Vec3f &n1, const Vec3f &p2, const Vec3f &n2,
	Vec3f *mid)
{
	float d1343,d4321,d1321,d4343,d2121;
	float numer,denom, mua, mub;

	Vec3f p13 = p1 - p2;
	// p43 = n2
	// p21 = n1
	d1343 = p13[0] * n2[0] + p13[1] * n2[1] + p13[2] * n2[2];
	d4321 = n2[0] * n1[0] + n2[1] * n1[1] + n2[2] * n1[2];
	d1321 = p13[0] * n1[0] + p13[1] * n1[1] + p13[2] * n1[2];
	d4343 = n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2];
	d2121 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

	denom = d2121 * d4343 - d4321 * d4321;
	if(fabs(denom) < 1.0e-6)
		return false;
	numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	mub = (d1343 + d4321 * (mua)) / d4343;

	Vec3f pa, pb;
	pa = p1 + mua * n1;
	pb = p2 + mub * n2;
	*mid = 0.5f * (pa + pb);
	return true;
}

InvalidTetrahedonError::InvalidTetrahedonError()
: std::runtime_error("Invalid tetrahedon")
{}

Sphere::Sphere()
{}

Sphere::Sphere(const Vec3f &center, float radius)
: m_center(center)
, m_radius(radius)
{}

Sphere::Sphere(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
	const Vec3f &p4)
{
	if(!Init(p1, p2, p3, p4))
		throw InvalidTetrahedonError();
}

bool Sphere::Init(const MiscLib::Vector< Vec3f > &samples)
{
	if(samples.size() < 4)
		return false;
	// get center
	size_t c = samples.size() / 2;
	m_center = Vec3f(0, 0, 0);
	size_t midCount = 0;
	for(size_t i = 0; i < c - 1; ++i)
		for(size_t j = i + 1; j < c; ++j)
		{
			Vec3f mid;
			if(!Midpoint(samples[i], samples[i + c], samples[j], samples[j + c], &mid))
				continue;
			m_center += mid;
			++midCount;
		}
	if(!midCount)
		return false;
	m_center /= midCount;
	m_radius = 0;
	for(size_t i = 0; i < c; ++i)
	{
		float d = (samples[i] - m_center).length();
		m_radius += d;
	}
	m_radius /= c;
	return true;
}

bool Sphere::Init(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3,
	const Vec3f &p4)
{
	// convert to double array
	double tetra[4 * 3];
	for(size_t i = 0; i < 3; ++i)
		tetra[0 * 3 + i] = p1[i];
	for(size_t i = 0; i < 3; ++i)
		tetra[1 * 3 + i] = p2[i];
	for(size_t i = 0; i < 3; ++i)
		tetra[2 * 3 + i] = p3[i];
	for(size_t i = 0; i < 3; ++i)
		tetra[3 * 3 + i] = p4[i];
	double r, pc[3];
	tetrahedron_circumsphere_3d(tetra, &r, pc);
	if(r < 0)
		return false;
	m_radius = r;
	m_center[0] = pc[0];
	m_center[1] = pc[1];
	m_center[2] = pc[2];
	return true;
}

bool Sphere::Init2(const Vec3f &p1, const Vec3f &p2, const Vec3f &n1,
	const Vec3f &n2)
{
/*
   Calculate the line segment PaPb that is the shortest route between
   two lines P1P2 and P3P4. Calculate also the values of mua and mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Return FALSE if no solution exists.
*/
	float d1343,d4321,d1321,d4343,d2121;
	float numer,denom, mua, mub;

	Vec3f p13 = p1 - p2;
	// p43 = n2
	// p21 = n1
	d1343 = p13[0] * n2[0] + p13[1] * n2[1] + p13[2] * n2[2];
	d4321 = n2[0] * n1[0] + n2[1] * n1[1] + n2[2] * n1[2];
	d1321 = p13[0] * n1[0] + p13[1] * n1[1] + p13[2] * n1[2];
	d4343 = n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2];
	d2121 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

	denom = d2121 * d4343 - d4321 * d4321;
	if(fabs(denom) < 1.0e-6)
		return false;
	numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	mub = (d1343 + d4321 * (mua)) / d4343;

	Vec3f pa, pb;
	pa = p1 + mua * n1;
	pb = p2 + mub * n2;

	// get the midpoint between pa and pb and make it the center
	m_center = 0.5f * (pa + pb);
	// make the radius the average of the two distances
	float da = (p1 - m_center).length();
	float db = (p2 - m_center).length();
	m_radius = 0.5f * (da + db);
	// do some plausability checks
	// lets say the actual distance should not deviate by more than 10%
	float dev = da / m_radius;
	if(dev < 0.9f || dev > 1.1f)
		return false;
	dev = db / m_radius;
	if(dev < 0.9f || dev > 1.1f)
		return false;
	// distance between pa and pb should not be greater than 10% of the radius
	dev = (pa - pb).length() / m_radius;
	if(dev > 0.1f)
		return false;
	return true;
}

bool Sphere::Init(bool binary, std::istream *i)
{
	if(binary)
	{
		i->read((char *)&m_center, sizeof(m_center));
		i->read((char *)&m_radius, sizeof(m_radius));
	}
	else
	{
		for(size_t j = 0; j < 3; ++j)
			(*i) >> m_center[j];
		(*i) >> m_radius;
	}
	return true;
}

void Sphere::Init(FILE *i)
{
	fread(&m_center, sizeof(m_center), 1, i);
	fread(&m_radius, sizeof(m_radius), 1, i);
}

void Sphere::Init(float *array)
{
	for(int i=0;i<3;i++){
		m_center[i] = array[i];
	}
	m_radius=array[3];
}

void Sphere::Project(const Vec3f &p, Vec3f *pp) const
{
	*pp = p - m_center;
	float l = pp->length();
	*pp *= m_radius / l;
	*pp += m_center;
}

const Vec3f &Sphere::Center() const
{
	return m_center;
}

float Sphere::Radius() const
{
	return m_radius;
}

float SphereDistance(const float *param, const float *x)
{
	float s = x[0] - param[0];
	s *= s;
	for(unsigned int i = 1; i < 3; ++i)
	{
		float ss = x[i] - param[i];
		s += ss * ss;
	}
	return std::sqrt(s) - param[3];
}

void SphereDistanceDerivatives(const float *param, const float *x,
	float *gradient)
{
	float s[3];
	s[0] = x[0] - param[0];
	float sl = s[0] * s[0];
	for(unsigned int i = 1; i < 3; ++i)
	{
		s[i] = x[i] - param[i];
		sl += s[i] * s[i];
	}
	sl = std::sqrt(sl);
	gradient[0] = -s[0] / sl;
	gradient[1] = -s[1] / sl;
	gradient[2] = -s[2] / sl;
	gradient[3] = -1;
}

void NormalizeSphereParams(float *param)
{}

bool Sphere::LeastSquaresFit(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	bool retVal = LeastSquaresFit(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()));
	return retVal;
}

bool Sphere::Interpolate(const MiscLib::Vector< Sphere > &spheres,
	const MiscLib::Vector< float > &weights, Sphere *is)
{
	Vec3f center(0, 0, 0);
	float radius = 0;
	for(size_t i = 0; i < spheres.size(); ++i)
	{
		center += weights[i] * spheres[i].Center();
		radius += weights[i] * spheres[i].Radius();
	}
	is->Center(center);
	is->Radius(radius);
	return true;
}

void Sphere::Serialize(bool binary, std::ostream *o) const
{
	if(binary)
	{
		o->write((const char *)&m_center, sizeof(m_center));
		o->write((const char *)&m_radius, sizeof(m_radius));
	}
	else
	{
		(*o) << m_center[0] << " " << m_center[1] << " " << m_center[2] << " "
			<< m_radius << " ";
	}
}

size_t Sphere::SerializedSize()
{
	return sizeof(Vec3f)
		+ sizeof(float);
}

size_t Sphere::SerializedFloatSize()
{
	return 4;
}

void Sphere::Serialize(FILE *o) const
{
	fwrite(&m_center, sizeof(m_center), 1, o);
	fwrite(&m_radius, sizeof(m_radius), 1, o);
}

void Sphere::Serialize(float *array) const
{
	for(int i=0;i<3;i++){
		array[i] = m_center[i];
	}
	array[3] = m_radius;
}


void Sphere::Transform(float scale, const Vec3f &translate)
{
	m_center *= scale;
	m_center += translate;
	m_radius *= scale;
}

SphereAsSquaresParametrization::SphereAsSquaresParametrization(const Sphere &sphere,
	const Vec3f &planeNormal)
: m_sphere(sphere)
, m_planeNormal(planeNormal)
, m_hcs(GfxTL::Vector3Df(planeNormal))
{}

void SphereAsSquaresParametrization::Init(const Sphere &sphere,
	const Vec3f &planeNormal)
{
	m_sphere = sphere;
	m_planeNormal = planeNormal;
	m_hcs.FromNormal(planeNormal[0], planeNormal[1], planeNormal[2]);
}

float SphereAsSquaresParametrization::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	// convert to hemisphere coordinates
	Vec3f s = p - m_sphere.Center();
	s.normalize();
	Vec3f hs;
	hs[0] = s.dot(m_hcs[0].Data());
	hs[1] = s.dot(m_hcs[1].Data());
	hs[2] = s.dot(m_planeNormal);
	float ret = hs[2];
	hs[2] = fabs(hs[2]);
	std::pair< float, float > inDisk;
	Hemisphere2Disk(hs, &inDisk);
	Disk2Square(inDisk, param);
	return ret;
}

bool SphereAsSquaresParametrization::InSpace(
	const std::pair< float, float > &param, bool lower, Vec3f *p) const
{
	if(param.first < -0.1 || param.first > 1.1
		|| param.second < -0.1 || param.second > 1.1)
		return false;
	std::pair< float, float > clampedParam;
	clampedParam.first = GfxTL::Math< float >::Clamp(param.first, 0, 1);
	clampedParam.second = GfxTL::Math< float >::Clamp(param.second, 0, 1);
	std::pair< float, float > inDisk;
	Square2Disk(clampedParam, &inDisk);
	Vec3f s;
	Disk2Hemisphere(inDisk, &s);
	*p = Vec3f((s[0] * m_hcs[0] + s[1] * m_hcs[1] +
		GfxTL::Vector3Df((lower? -1 : 1) * s[2] * m_planeNormal)).Data());
	*p *= m_sphere.Radius();
	*p += m_sphere.Center();
	return true;
}

bool SphereAsSquaresParametrization::InSpace(
	const std::pair< float, float > &param, bool lower, Vec3f *p,
	Vec3f *n) const
{
	if(param.first < -0.1 || param.first > 1.1
		|| param.second < -0.1 || param.second > 1.1)
		return false;
	std::pair< float, float > clampedParam;
	clampedParam.first = GfxTL::Math< float >::Clamp(param.first, 0, 1);
	clampedParam.second = GfxTL::Math< float >::Clamp(param.second, 0, 1);
	std::pair< float, float > inDisk;
	Square2Disk(clampedParam, &inDisk);
	Vec3f s;
	Disk2Hemisphere(inDisk, &s);
	if(lower)
		s[2] *= -1;
	*n = Vec3f((s[0] * m_hcs[0] + s[1] * m_hcs[1] +
		GfxTL::Vector3Df(s[2] * m_planeNormal)).Data());
	*p = m_sphere.Radius() * (*n);
	*p += m_sphere.Center();
	return true;
}

void SphereAsSquaresParametrization::Transform(
	const GfxTL::MatrixXX< 3, 3, float > &rot, const GfxTL::Vector3Df &trans)
{
	m_sphere = Sphere(Vec3f((rot * GfxTL::Vector3Df(m_sphere.Center())
		+ trans).Data()), m_sphere.Radius());
	m_planeNormal = Vec3f((rot * GfxTL::Vector3Df(m_planeNormal)).Data());
	m_hcs[0] = rot * m_hcs[0];
	m_hcs[1] = rot * m_hcs[1];
}

void SphereAsSquaresParametrization::HyperplaneCoordinateSystem( Vec3f* hcs0, Vec3f* hcs1, Vec3f* hcs2 ) const
{
	hcs0->setValue( m_hcs[0] );
	hcs1->setValue( m_hcs[1] );
	hcs2->setValue( m_hcs[2] );
}


void SphereAsSquaresParametrization::Hemisphere2Disk(const Vec3f &p,
	std::pair< float, float > *inDisk) const
{
    inDisk->first = std::sqrt(1 - p[2]); 
    inDisk->second = std::atan2(p[1], p[0]); 
}

void SphereAsSquaresParametrization::Disk2Square(
	const std::pair< float, float > &inDisk,
	std::pair< float, float > *inSquare) const
{
    float r = inDisk.first; 
    float phi = inDisk.second; 
    float a, b; 

    if(phi < float(-M_PI / 4.0)) 
        phi += float(2 * M_PI); 

    if(phi < float(M_PI / 4.0)) 
    { 
        a = r; 
        b = phi * a / float(M_PI / 4.0); 
    } 
    else if(phi < float( 3 * M_PI / 4.0)) 
    { 
        b = r; 
        a = -(phi - float(M_PI / 2.0))* b / float(M_PI / 4.0); 
    } 
    else if(phi < float(5 * M_PI / 4.0)) 
    { 
        a = -r; 
        b = (phi - float(M_PI))*a / float(M_PI / 4.0); 
    } 
    else 
    { 
        b = -r; 
        a = -(phi - float(3 * M_PI / 2.0))* b /float(M_PI / 4.0); 
    } 

    inSquare->first = (a + float(1.0)) / float(2.0); 
    inSquare->second = (b + float(1.0)) / float(2.0); 
}

void SphereAsSquaresParametrization::Square2Disk(
	const std::pair< float, float > &inSquare,
	std::pair< float, float > *inDisk) const
{
    float phi, r; 
    float a = 2 * inSquare.first - 1; 
    float b = 2 * inSquare.second - 1; 

    if(a > -b)
	{
        if(a > b) 
        { 
            r = a; 
            phi = float(M_PI / 4.0) * (b / a); 
        } 
        else 
        { 
            r = b; 
            phi = float(M_PI / 4.0) * (2 - (a / b)); 
        }
	}
    else 
    { 
        if(a < b) 
        { 
            r = -a; 
            phi = float(M_PI / 4.0) * (4 + (b / a)); 
        } 
        else 
        { 
            r = -b; 
            if(b != 0) 
				phi = float(M_PI / 4.0) * (6 - (a / b)); 
            else  
				phi = 0; 
        } 
    } 

    inDisk->first = r; 
    inDisk->second = phi; 
}

void SphereAsSquaresParametrization::Disk2Hemisphere(
	const std::pair< float, float > &inDisk, Vec3f *p) const
{
    (*p)[0] = inDisk.first * std::sqrt(2 - inDisk.first * inDisk.first)
		* std::cos(inDisk.second); 
	(*p)[1] = inDisk.first * std::sqrt(2 - inDisk.first * inDisk.first)
		* std::sin(inDisk.second); 
    (*p)[2] = 1 - inDisk.first * inDisk.first;
}
