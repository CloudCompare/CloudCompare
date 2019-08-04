#include "TorusPrimitiveShape.h"
#include "PrimitiveShapeVisitor.h"
#include <MiscLib/Performance.h>
#include <sstream>
#include "ConePrimitiveShape.h"
#include "CylinderPrimitiveShape.h"
#include "SpherePrimitiveShape.h"
#include "PlanePrimitiveShape.h"
extern MiscLib::performance_t totalTime_torusConnected;

TorusPrimitiveShape::TorusPrimitiveShape(const Torus &torus)
: m_torus(torus)
, m_parametrization(m_torus)
{}

TorusPrimitiveShape::TorusPrimitiveShape(const TorusPrimitiveShape &tps)
: BitmapPrimitiveShape(tps)
, m_torus(tps.m_torus)
, m_parametrization(tps.m_parametrization)
{
	m_parametrization.Shape(m_torus);
}

size_t TorusPrimitiveShape::Identifier() const
{
	return 4;
}

PrimitiveShape *TorusPrimitiveShape::Clone() const
{
	return new TorusPrimitiveShape(*this);
}

float TorusPrimitiveShape::Distance(const Vec3f &p) const
{
	return m_torus.Distance(p);
}

float TorusPrimitiveShape::SignedDistance(const Vec3f &p) const
{
	return m_torus.SignedDistance(p);
}

float TorusPrimitiveShape::NormalDeviation(const Vec3f &p,
	const Vec3f &n) const
{
	Vec3f normal;
	m_torus.Normal(p, &normal);
	return n.dot(normal);
}

void TorusPrimitiveShape::DistanceAndNormalDeviation(const Vec3f &p,
	const Vec3f &n, std::pair< float, float > *dn) const
{
	Vec3f normal;
	dn->first = m_torus.DistanceAndNormal(p, &normal);
	dn->second = n.dot(normal);
}

void TorusPrimitiveShape::Project(const Vec3f &p, Vec3f *pp) const
{
	m_torus.Project(p, pp);
}

void TorusPrimitiveShape::Normal(const Vec3f &p, Vec3f *n) const
{
	m_torus.Normal(p, n);
}

unsigned int TorusPrimitiveShape::ConfidenceTests(unsigned int numTests,
	float epsilon, float normalThresh, float rms, const PointCloud &pc,
	const MiscLib::Vector< size_t > &indices) const
{
	return BasePrimitiveShape::ConfidenceTests< Torus >(numTests, epsilon,
		normalThresh, rms, pc, indices);
}

void TorusPrimitiveShape::Description(std::string *s) const
{
	std::ostringstream ostr;
	ostr << "Torus (minor=" << m_torus.MinorRadius()
		<< " major=" << m_torus.MajorRadius() << ")";
	*s = ostr.str();
//	*s = "Torus";
}

bool TorusPrimitiveShape::Fit(const PointCloud &pc, float epsilon,
	float normalThresh,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	Torus fit = m_torus;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		m_torus = fit;
		m_parametrization.Shape(m_torus);
		return true;
	}
	return false;
}

PrimitiveShape *TorusPrimitiveShape::LSFit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end,
	std::pair< size_t, float > *score) const
{
	Torus fit = m_torus;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		score->first = -1;
		return new TorusPrimitiveShape(fit);
	}
	score->first = 0;
	return NULL;
}

LevMarFunc< float > *TorusPrimitiveShape::SignedDistanceFunc() const
{
	return new TorusLevMarFunc(m_torus);
}

void TorusPrimitiveShape::Serialize(std::ostream *o, bool binary) const
{
	if(binary)
	{
		const char id = 4;
		(*o) << id;
	}
	else
		(*o) << "4" << " ";
	m_torus.Serialize(binary, o);
	m_parametrization.Serialize(o, binary);
	if(!binary)
		*o << std::endl;
}

void TorusPrimitiveShape::Deserialize(std::istream *i, bool binary)
{
	m_torus.Init(binary, i);
	m_parametrization.Shape(m_torus);
	m_parametrization.Deserialize(i, binary);
}

size_t TorusPrimitiveShape::SerializedSize() const
{
	return m_torus.SerializedSize() + m_parametrization.SerializedSize() + 1;
}

void TorusPrimitiveShape::Transform(float scale, const Vec3f &translate)
{
	m_torus.Transform(scale, translate);
}

void TorusPrimitiveShape::Visit(PrimitiveShapeVisitor *visitor) const
{
	visitor->Visit(*this);
}

void TorusPrimitiveShape::SuggestSimplifications(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end, float distThresh,
	MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const
{
	// sample the bounding box in parameter space at 25 locations
	// these points are used to estimate the other shapes
	// if the shapes succeed the suggestion is returned
	MiscLib::Vector< Vec3f > samples(2 * 25);
	float uStep = (m_extBbox.Max()[0] - m_extBbox.Min()[0]) / 4;
	float vStep = (m_extBbox.Max()[1] - m_extBbox.Min()[1]) / 4;
	float u = m_extBbox.Min()[0];
	for(unsigned int i = 0; i < 5; ++i, u += uStep)
	{
		float v = m_extBbox.Min()[1];
		for(unsigned int j = 0; j < 5; ++j, v += vStep)
		{
			float bmpu;
			if(m_torus.MajorRadius() < m_torus.MinorRadius() * 2)
				bmpu = u * (m_torus.MajorRadius() + m_torus.MinorRadius());
			else
				bmpu = u * m_torus.MajorRadius();
			InSpace(bmpu, v * m_torus.MinorRadius(), &samples[i * 5 + j],
				&samples[i * 5 + j + 25]);
		}
	}
	size_t c = samples.size() / 2;
	// now check all the shape types
	Cone cone;
	if(cone.InitAverage(samples))
	{
		cone.LeastSquaresFit(samples.begin(), samples.begin() + c);
		bool failed = false;
		for(size_t i = 0; i < c; ++i)
			if(cone.Distance(samples[i]) > distThresh)
			{
				failed = true;
				break;
			}
		if(!failed)
		{
			suggestions->push_back(new ConePrimitiveShape(cone));
			suggestions->back()->Release();
		}
	}
	Cylinder cylinder;
	if(cylinder.InitAverage(samples))
	{
		cylinder.LeastSquaresFit(samples.begin(), samples.begin() + c);
		bool failed = false;
		for(size_t i = 0; i < c; ++i)
			if(cylinder.Distance(samples[i]) > distThresh)
			{
				failed = true;
				break;
			}
		if(!failed)
		{
			suggestions->push_back(new CylinderPrimitiveShape(cylinder));
			suggestions->back()->Release();
		}
	}
	Sphere sphere;
	if(sphere.Init(samples))
	{
		sphere.LeastSquaresFit(samples.begin(), samples.begin() + c);
		bool failed = false;
		for(size_t i = 0; i < c; ++i)
			if(sphere.Distance(samples[i]) > distThresh)
			{
				failed = true;
				break;
			}
		if(!failed)
		{
			suggestions->push_back(new SpherePrimitiveShape(sphere));
			suggestions->back()->Release();
		}
	}
	Plane plane;
	if(plane.LeastSquaresFit(samples.begin(), samples.begin() + c))
	{
		bool failed = false;
		for(size_t i = 0; i < c; ++i)
			if(plane.Distance(samples[i]) > distThresh)
			{
				failed = true;
				break;
			}
		if(!failed)
		{
			suggestions->push_back(new PlanePrimitiveShape(plane));
			suggestions->back()->Release();
		}
	}
	/*// although theoretically possible, we never suggest a cone since a misclassification
	// of a cone as a torus is extremley seldom
	// The parametrization is given as major arclength and minor arclength
	float radialMajor = m_extBbox.Max()[0] - m_extBbox.Min()[0];
	float radialMinor = m_extBbox.Max()[1] - m_extBbox.Min()[1];
	float lengthMajor = radialMajor * m_torus.MajorRadius();
	float lengthMinor = radialMinor * m_torus.MinorRadius();
	float meanRadius = (m_torus.MajorRadius() + m_torus.MinorRadius()) / 2;
	// suggest a cylinder if either of the two radii can be replaced by
	// a non curved direction
	// we suggest a cylinder if the major radius causes an error less than distThresh
	// this tests if the major radius can be replaced
	float radiusDiffMajor = (m_torus.MajorRadius() - (std::cos(radialMajor / 2)
		* m_torus.MajorRadius())) / 2;
	if(radiusDiffMajor < distThresh)
	{
		// construct the cylinder
		// the axis of the cylinder is given
		float majorCenter = (m_extBbox.Max()[0] - m_extBbox.Min()[0]) / 2
			* m_torus.MajorRadius();
		Vec3f pos, normal, cyAxisDir;
		InSpace(majorCenter, M_PI * m_torus.MinorRadius(), &pos, &normal);
		cyAxisDir = normal.cross(m_torus.AxisDirection());
		cyAxisDir.normalize();
		pos -= (m_torus.MinorRadius() - radiusDiffMajor) * normal;
		Cylinder cylinder(cyAxisDir, pos, m_torus.MinorRadius());
		suggestions->push_back(new CylinderPrimitiveShape(cylinder));
		suggestions->back()->Release();
	}
	// now test if the minor radius can be replaced
	float radiusDiffMinor = (m_torus.MinorRadius() - (std::cos(radialMinor / 2)
		* m_torus.MinorRadius())) / 2;
	if(radiusDiffMinor < distThresh)
	{
		// if the minor radius is replaced 
	}
	// we suggest a sphere if the torus is apple shaped
	if(m_torus.IsAppleShaped())
	{
		Sphere sphere(m_torus.Center(),
			(m_torus.MajorRadius() + m_torus.MinorRadius()) / 2);
		suggestions->push_back(new SpherePrimitiveShape(sphere));
		suggestions->back()->Release();
	}
	// we can also suggest a sphere if the error introduced by a common radius
	// for minor and major does not introduce an error
	//else if()
	//{
	//}
	// we suggest a plane if both major and minor radius cause ony small error
	float radiusDiffMinor = (m_torus.MinorRadius() - (std::cos(radialMinor / 2)
		* m_torus.MinorRadius())) / 2;
	if(radiusDiffMajor < distThresh && radiusDiffMinor < distThresh)
	{
		GfxTL::Vector2Df paramCenter;
		m_extBbox.Center(&paramCenter);
		Vec3f pos, normal;
		InSpace(paramCenter[0] * m_torus.MajorRadius(),
			paramCenter[1] * m_torus.MinorRadius(), &pos, &normal);
		Plane plane(pos, normal);
		suggestions->push_back(new PlanePrimitiveShape(plane));
		suggestions->back()->Release();
	}*/
}

void TorusPrimitiveShape::OptimizeParametrization(const PointCloud &pc,
	size_t begin, size_t end, float epsilon)
{
	m_parametrization.Optimize(GfxTL::IndexIterate(IndexIterator(begin), pc.begin()),
		GfxTL::IndexIterate(IndexIterator(end), pc.begin()), epsilon);
}

bool TorusPrimitiveShape::Similar(float tolerance,
	const TorusPrimitiveShape &shape) const
{
	return m_torus.MajorRadius() <= (1.f + tolerance) * shape.m_torus.MajorRadius()
		&& (1.f + tolerance) * m_torus.MajorRadius() >= shape.m_torus.MajorRadius()
		&& m_torus.MinorRadius() <= (1.f + tolerance) * shape.m_torus.MinorRadius()
		&& (1.f + tolerance) * m_torus.MinorRadius() >= shape.m_torus.MinorRadius();
}

void TorusPrimitiveShape::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	m_parametrization.Parameters(p, param);
}

void TorusPrimitiveShape::Parameters(
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void TorusPrimitiveShape::Parameters(
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

bool TorusPrimitiveShape::InSpace(float u, float v, Vec3f *p, Vec3f *n) const
{
	return m_parametrization.InSpace(u, v, p, n);
}

void TorusPrimitiveShape::BitmapExtent(float epsilon,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox,
	MiscLib::Vector< std::pair< float, float > > *params,
	size_t *uextent, size_t *vextent)
{
	*uextent = static_cast<size_t>(std::ceil((bbox->Max()[0] - bbox->Min()[0]) / epsilon));
	*vextent = static_cast<size_t>(std::ceil((bbox->Max()[1] - bbox->Min()[1]) / epsilon));
}

void TorusPrimitiveShape::InBitmap(const std::pair< float, float > &param,
	float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	size_t uextent, size_t vextent,
	std::pair< int, int > *inBmp) const
{
	inBmp->first  = static_cast<int>(std::floor((param.first  - bbox.Min()[0]) / epsilon));
	inBmp->second = static_cast<int>(std::floor((param.second - bbox.Min()[1]) / epsilon));
}

void TorusPrimitiveShape::WrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, bool *uwrap, bool *vwrap) const
{
	m_parametrization.WrapBitmap(bbox, epsilon, uwrap, vwrap);
}

void TorusPrimitiveShape::WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, size_t uextent, size_t vextent,
	MiscLib::Vector< int > *componentImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels) const
{
	m_parametrization.WrapComponents(bbox, epsilon, uextent, vextent,
		componentImg, labels);
}

void TorusPrimitiveShape::SetExtent(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	const MiscLib::Vector< int > &componentsImg, size_t uextent,
	size_t vextent, float epsilon, int label)
{}

bool TorusPrimitiveShape::InSpace(size_t u, size_t v, float epsilon,
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
	size_t vextent, Vec3f *p, Vec3f *n) const
{
	return m_parametrization.InSpace((u + .5f) * epsilon + bbox.Min()[0],
		(v + .5f) * epsilon + bbox.Min()[1], p, n);
}
