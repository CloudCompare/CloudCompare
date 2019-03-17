#include "CylinderPrimitiveShape.h"
#include "ScoreComputer.h"
#include "PrimitiveShapeVisitor.h"
#include <limits>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <GfxTL/NullClass.h>
#include <iostream>
#include <algorithm>
#include <MiscLib/Performance.h>
#include "TorusPrimitiveShape.h"
#include "ConePrimitiveShape.h"
#include "SpherePrimitiveShape.h"
#include "PlanePrimitiveShape.h"
extern MiscLib::performance_t totalTime_cylinderConnected;

CylinderPrimitiveShape::CylinderPrimitiveShape()
{}

CylinderPrimitiveShape::CylinderPrimitiveShape(const Cylinder &cylinder)
: m_cylinder(cylinder)
{}

size_t CylinderPrimitiveShape::Identifier() const
{
	return 2;
}

PrimitiveShape *CylinderPrimitiveShape::Clone() const
{
	return new CylinderPrimitiveShape(*this);
}

bool CylinderPrimitiveShape::Init(const Vec3f &pointA, const Vec3f &pointB,
	const Vec3f &normalA, const Vec3f &normalB)
{
	return m_cylinder.Init(pointA, pointB, normalA, normalB);
}

float CylinderPrimitiveShape::Distance(const Vec3f &p) const
{
	return m_cylinder.Distance(p);
}

float CylinderPrimitiveShape::SignedDistance(const Vec3f &p) const
{
	return m_cylinder.SignedDistance(p);
}

float CylinderPrimitiveShape::NormalDeviation(const Vec3f &p,
	const Vec3f &n) const
{
	Vec3f normal;
	m_cylinder.Normal(p, &normal);
	return n.dot(normal);
}

void CylinderPrimitiveShape::DistanceAndNormalDeviation(
	const Vec3f &p, const Vec3f &n, std::pair< float, float > *dn) const
{
	Vec3f normal;
	dn->first = m_cylinder.DistanceAndNormal(p, &normal);
	dn->second = n.dot(normal);
}

void CylinderPrimitiveShape::Project(const Vec3f &p, Vec3f *pp) const
{
	m_cylinder.Project(p, pp);
}

void CylinderPrimitiveShape::Normal(const Vec3f &p, Vec3f *n) const
{
	m_cylinder.Normal(p, n);
}

unsigned int CylinderPrimitiveShape::ConfidenceTests(unsigned int numTests,
	float epsilon, float normalThresh, float rms, const PointCloud &pc,
	const MiscLib::Vector< size_t > &indices) const
{
	return BasePrimitiveShape::ConfidenceTests< Cylinder >(numTests, epsilon,
		normalThresh, rms, pc, indices);
}

void CylinderPrimitiveShape::Description(std::string *s) const
{
	*s = "Cylinder";
}

bool CylinderPrimitiveShape::Fit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)

{
	Cylinder fit = m_cylinder;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		m_cylinder = fit;
		return true;
	}
	return false;
}

PrimitiveShape *CylinderPrimitiveShape::LSFit(const PointCloud &pc,
	float epsilon, float normalThresh,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end,
	std::pair< size_t, float > *score) const
{
	Cylinder fit = m_cylinder;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		score->first = -1;
		return new CylinderPrimitiveShape(fit);
	}
	score->first = 0;
	return NULL;
}

LevMarFunc< float > *CylinderPrimitiveShape::SignedDistanceFunc() const
{
	return new CylinderLevMarFunc(m_cylinder);
}

void CylinderPrimitiveShape::Serialize(std::ostream *o, bool binary) const
{
	if(binary)
	{
		const char id = 2;
		(*o) << id;
	}
	else
		(*o) << "2" << " ";
	m_cylinder.Serialize(binary, o);
	if(!binary)
		*o << std::endl;
}

size_t CylinderPrimitiveShape::SerializedSize() const
{
	return m_cylinder.SerializedSize() + 1;
}

void CylinderPrimitiveShape::Transform(float scale, const Vec3f &translate)
{
	m_cylinder.Transform(scale, translate);
}

void CylinderPrimitiveShape::Transform(
	const GfxTL::MatrixXX< 3, 3, float > &rot, const GfxTL::Vector3Df &trans)
{
	m_cylinder.Transform(rot, trans);
}

void CylinderPrimitiveShape::Visit(PrimitiveShapeVisitor *visitor) const
{
	visitor->Visit(*this);
}

void CylinderPrimitiveShape::SuggestSimplifications(const PointCloud &pc,
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
			InSpace(u, v * m_cylinder.Radius(), &samples[i * 5 + j],
				&samples[i * 5 + j + 25]);
	}
	size_t c = samples.size() / 2;
	// now check all the shape types
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
	/*// We suggest a sphere if a curvature of radius along the height
	// does not introduce too large an error
	float length = m_extBbox.Max()[0] - m_extBbox.Min()[0];
	float meanLength = (m_extBbox.Max()[0] + m_extBbox.Min()[0]) / 2;
	float radiusDiff = (std::sqrt(m_cylinder.Radius() * m_cylinder.Radius()
		+ length * length / 4) - m_cylinder.Radius()) / 2;
	float radialExtent = m_extBbox.Max()[1] - m_extBbox.Min()[1];
	if(radiusDiff < distThresh)
	{
		// the center of the sphere is given as the point on the axis
		// with the height of the mean length
		Vec3f center = meanLength * m_cylinder.AxisDirection()
			+ m_cylinder.AxisPosition();
		Sphere sphere(center, m_cylinder.Radius() + radiusDiff);
		suggestions->push_back(new SpherePrimitiveShape(sphere));
		suggestions->back()->Release();
	}

	// We suggest a plane if the mean radius causes only a small error
	// for this we need the angular extent in the curved direction of the cone
	radiusDiff = (m_cylinder.Radius() - std::cos(radialExtent / 2)
		* m_cylinder.Radius()) / 2;
	if(radiusDiff < distThresh)
	{
		GfxTL::Vector2Df bboxCenter;
		m_extBbox.Center(&bboxCenter);
		Vec3f pos, normal;
		InSpace(bboxCenter[0], bboxCenter[1] * m_cylinder.Radius(),
			&pos, &normal);
		// offset position
		pos -= radiusDiff * normal;
		Plane plane(pos, normal);
		suggestions->push_back(new PlanePrimitiveShape(plane));
		suggestions->back()->Release();
	}*/
}

bool CylinderPrimitiveShape::Similar(float tolerance,
	const CylinderPrimitiveShape &shape) const
{
	return m_cylinder.Radius() <= (1.f + tolerance) * shape.m_cylinder.Radius()
		&& (1.f + tolerance) * m_cylinder.Radius() >= shape.m_cylinder.Radius();
}

float CylinderPrimitiveShape::Height() const
{
	return m_extBbox.Max()[0] - m_extBbox.Min()[0];
}

float CylinderPrimitiveShape::MinHeight() const
{
	return m_extBbox.Min()[0];
}

float CylinderPrimitiveShape::MaxHeight() const
{
	return m_extBbox.Max()[0];
}

void CylinderPrimitiveShape::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	m_cylinder.Parameters(p, param);
	// convert angle to arc length
	param->second *= m_cylinder.Radius();
}

void CylinderPrimitiveShape::Parameters(
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void CylinderPrimitiveShape::Parameters(
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void CylinderPrimitiveShape::BitmapExtent(float epsilon,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox,
	MiscLib::Vector< std::pair< float, float > > *params,
	size_t *uextent, size_t *vextent)
{
	*uextent = size_t(std::ceil(
		(bbox->Max()[0] - bbox->Min()[0]) / epsilon));
	*vextent = size_t(std::ceil((bbox->Max()[1] - bbox->Min()[1]) / epsilon));
	if((*vextent) * (*uextent) > 1e6)
	{
		// try to reparameterize
		if(bbox->Min()[1] > epsilon && bbox->Max()[1] < 2 * M_PI * m_cylinder.Radius() - epsilon)
			return; // there is no wrapping -> we can't do anything
		MiscLib::Vector< float > angularParams(params->size());
		for(size_t i = 0; i < params->size(); ++i)
			angularParams[i] = (*params)[i].second;
		std::sort(angularParams.begin(), angularParams.end());
		// try to find a large gap
		float maxGap = 0;
		float lower, upper;
		for(size_t i = 1; i < angularParams.size(); ++i)
		{
			float gap = angularParams[i] - angularParams[i - 1];
			if(gap > maxGap)
			{
				maxGap = gap;
				lower = angularParams[i - 1];
				upper = angularParams[i];
			}
		}
		if(maxGap > epsilon)
		{
			// reparameterize with new angular cut
			float newCut = (lower + upper) / 2.f;
			m_cylinder.RotateAngularDirection(newCut / m_cylinder.Radius());
			bbox->Min()[1] = std::numeric_limits< float >::infinity();
			bbox->Max()[1] = -std::numeric_limits< float >::infinity();
			for(size_t i = 0; i < params->size(); ++i)
			{
				(*params)[i].second -= newCut;
				if((*params)[i].second < 0)
					(*params)[i].second = static_cast<float>(2 * M_PI * m_cylinder.Radius() + (*params)[i].second);
				if((*params)[i].second < bbox->Min()[1])
					bbox->Min()[1] = (*params)[i].second;
				if((*params)[i].second > bbox->Max()[1])
					bbox->Max()[1] = (*params)[i].second;
			}
			*vextent = size_t(std::ceil((bbox->Max()[1] - bbox->Min()[1]) / epsilon));
		}
	}
}

void CylinderPrimitiveShape::InBitmap(const std::pair< float, float > &param,
	float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	size_t uextent, size_t vextent, std::pair< int, int > *inBmp) const
{
	// convert the parameters to bitmap coordinates
	inBmp->first  = static_cast<int>(std::floor((param.first  - bbox.Min()[0]) / epsilon));
	inBmp->second = static_cast<int>(std::floor((param.second - bbox.Min()[1]) / epsilon));
}
void CylinderPrimitiveShape::WrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, float epsilon, bool *uwrap,
	bool *vwrap) const
{
	*uwrap = false;
	if(bbox.Max()[1] - bbox.Min()[1]
		>= 2 * M_PI * m_cylinder.Radius() - 2 * epsilon)
		*vwrap = true; // wrap around angular component
	else
		*vwrap = false;
}

void CylinderPrimitiveShape::PreWrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, size_t uextent, size_t vextent, MiscLib::Vector< char > *bmp) const
{	
	// wraps the bitmpap around the v-axis
	// note: we do not check, if the cylinder is really wrapped around !
	//		 Use WrapBitmap for this check
	for (int i=0; i < uextent; i++)
	{
		char t = (*bmp)[i];
		bmp->push_back(t);
	}
}

void CylinderPrimitiveShape::SetExtent(
	const GfxTL::AABox< GfxTL::Vector2Df > &extBbox,
	const MiscLib::Vector< int > &componentsImg, size_t uextent,
	size_t vextent, float epsilon, int label)
{
	if(extBbox.Min()[1] * m_cylinder.Radius() <= epsilon
		&& extBbox.Max()[1] * m_cylinder.Radius()
			>= 2 * M_PI * m_cylinder.Radius() - epsilon)
	{
		// component has been cut along angular direction
		// run from both sides to find both ends
		size_t row = 0, j = 0;
		for(; j < vextent; ++j)
		{
			bool found = false;
			for(size_t i = 0; i < uextent; ++i)
			{
				if(componentsImg[row + i] == label)
				{
					found = true;
					break;
				}
			}
			if(!found)
				break;
			row += uextent;
		}
		size_t maxj = j;
		if(maxj == vextent) // cylinder is complete
		{
			m_clip = false;
			return;
		}
		row = (vextent - 1) * uextent, j = 0;
		for(; j < vextent; ++j)
		{
			bool found = false;
			for(size_t i = 0; i < uextent; ++i)
			{
				if(componentsImg[row + i] == label)
				{
					found = true;
					break;
				}
			}
			if(!found)
				break;
			row -= uextent;
		}
		size_t minj = j;
		// convert min and max to angular parameters
		m_minPhi = minj * epsilon / m_cylinder.Radius() + extBbox.Min()[1];
		m_maxPhi = maxj * epsilon / m_cylinder.Radius() + extBbox.Min()[1];
	}
	else
	{
		m_minPhi = extBbox.Min()[1];
		m_maxPhi = extBbox.Max()[1];
	}
	m_clip = true;
}

bool CylinderPrimitiveShape::InSpace(float u, float v, Vec3f *p, Vec3f *n) const
{
	GfxTL::Quaternion< float > q;
	q.RotationRad(v / m_cylinder.Radius(), m_cylinder.AxisDirection()[0],
		m_cylinder.AxisDirection()[1], m_cylinder.AxisDirection()[2]);
	Vec3f vvec;
	q.Rotate(m_cylinder.AngularDirection(), &vvec);
	*p = u * m_cylinder.AxisDirection() + m_cylinder.Radius() * vvec
		+ m_cylinder.AxisPosition();
	*n = vvec;
	return true;
}

bool CylinderPrimitiveShape::InSpace(size_t u, size_t v, float epsilon,
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
	size_t vextent, Vec3f *p, Vec3f *n) const
{
	GfxTL::Quaternion< float > q;
	q.RotationRad((bbox.Min()[1] + epsilon * (v + .5f)) / m_cylinder.Radius(),
		m_cylinder.AxisDirection()[0],
		m_cylinder.AxisDirection()[1],
		m_cylinder.AxisDirection()[2]);
	Vec3f vvec;
	q.Rotate(m_cylinder.AngularDirection(), &vvec);
	*p = (bbox.Min()[0] + epsilon * (u + .5f)) * m_cylinder.AxisDirection() +
		m_cylinder.Radius() * vvec + m_cylinder.AxisPosition();
	*n = vvec;
	return true;
}
