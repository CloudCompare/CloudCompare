#include "ConePrimitiveShape.h"
#include "PrimitiveShapeVisitor.h"
#include "ScoreComputer.h"
#include "Bitmap.h"
#include <GfxTL/NullClass.h>
#include <GfxTL/MathHelper.h>
#include <limits>
#include <algorithm>
#include <iostream>
#include <MiscLib/Performance.h>
#include "TorusPrimitiveShape.h"
#include "CylinderPrimitiveShape.h"
#include "SpherePrimitiveShape.h"
#include "PlanePrimitiveShape.h"

extern MiscLib::performance_t totalTime_coneConnected;
#undef max
#undef min

ConePrimitiveShape::ConePrimitiveShape(const Cone &cone)
: m_cone(cone)
{}

size_t ConePrimitiveShape::Identifier() const
{
	return 3;
}

PrimitiveShape *ConePrimitiveShape::Clone() const
{
	return new ConePrimitiveShape(*this);
}

float ConePrimitiveShape::Distance(const Vec3f &p) const
{
	return m_cone.Distance(p);
}

float ConePrimitiveShape::SignedDistance(const Vec3f &p) const
{
	return m_cone.SignedDistance(p);
}

float ConePrimitiveShape::NormalDeviation(const Vec3f &p, const Vec3f &n) const
{
	Vec3f normal;
	m_cone.Normal(p, &normal);
	return n.dot(normal);
}

void ConePrimitiveShape::DistanceAndNormalDeviation(
	const Vec3f &p, const Vec3f &n, std::pair< float, float > *dn) const
{
	Vec3f normal;
	dn->first = m_cone.DistanceAndNormal(p, &normal);
	dn->second = n.dot(normal);
}

void ConePrimitiveShape::Project(const Vec3f &p, Vec3f *pp) const
{
	m_cone.Project(p, pp);
}

void ConePrimitiveShape::Normal(const Vec3f &p, Vec3f *n) const
{
	m_cone.Normal(p, n);
}

unsigned int ConePrimitiveShape::ConfidenceTests(unsigned int numTests,
	float epsilon, float normalThresh, float rms, const PointCloud &pc,
	const MiscLib::Vector< size_t > &indices) const
{
	return BasePrimitiveShape::ConfidenceTests< Cone >(numTests, epsilon,
		normalThresh, rms, pc, indices);
}

void ConePrimitiveShape::Description(std::string *s) const
{
	*s = "Cone";
}

bool ConePrimitiveShape::Fit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	Cone fit = m_cone;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		m_cone = fit;
		return true;
	}
	return false;

}

PrimitiveShape *ConePrimitiveShape::LSFit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end,
	std::pair< size_t, float > *score) const
{
	Cone fit = m_cone;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		score->first = -1;
		return new ConePrimitiveShape(fit);
	}
	score->first = 0;
	return NULL;
}

LevMarFunc< float > *ConePrimitiveShape::SignedDistanceFunc() const
{
	return new ConeLevMarFunc(m_cone);
}

void ConePrimitiveShape::Serialize(std::ostream *o, bool binary) const
{
	if(binary)
	{
		const char id = 3;
		(*o) << id;
	}
	else
		(*o) << "3" << " ";
	m_cone.Serialize(binary, o);
	if(!binary)
		(*o) << std::endl;
}

size_t ConePrimitiveShape::SerializedSize() const
{
	return m_cone.SerializedSize() + 1;
}

void ConePrimitiveShape::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	m_cone.Parameters(p, param); // this gives us length and angle
	// select parametrization with smaller stretch
	if(m_cone.Angle() < float(M_PI / 4)) // angle of cone less than 45 degrees
	{
		// parameterize in the following way:
		// u = length
		// v = arc length
		float r = m_cone.RadiusAtLength(param->first);
		param->second = (param->second - float(M_PI)) * r; // convert to arc length and centralize
	}
	else
	{
		float l = param->first;
		param->first = std::sin(param->second) * l;
		param->second = std::cos(param->second) * l;
	}
}

void ConePrimitiveShape::Parameters(GfxTL::IndexedIterator<
		MiscLib::Vector< size_t >::iterator, PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void ConePrimitiveShape::Parameters(GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void ConePrimitiveShape::Transform(float scale, const Vec3f &translate)
{
	m_cone.Transform(scale, translate);
}

void ConePrimitiveShape::Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
	const GfxTL::Vector3Df &trans)
{
	m_cone.Transform(rot, trans);
}

void ConePrimitiveShape::Visit(PrimitiveShapeVisitor *visitor) const
{
	visitor->Visit(*this);
}

void ConePrimitiveShape::SuggestSimplifications(const PointCloud &pc,
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
			float bmpu, bmpv;
			if(m_cone.Angle() >= M_PI / 4)
			{
				bmpu = std::sin(v) * u;
				bmpv = std::cos(v) * u;
			}
			else
			{
				bmpu = u;
				float r = m_cone.RadiusAtLength(u);
				bmpv = (v - float(M_PI)) * r;
			}
			InSpace(bmpu, bmpv, &samples[i * 5 + j],
				&samples[i * 5 + j + 25]);
		}
	}
	size_t c = samples.size() / 2;
	// now check all the shape types
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

	/*// simpler shapes are suggested if the maximal curvature of the cone
	// is small compared to the extend in relation to the distThresh

	float meanRadius, length, meanLength, radialExtent;
	// the cone is parameterized as length and angle
	// in this case the cone is parametrized as length and arclength
	meanRadius = (m_cone.RadiusAtLength(m_extBbox.Min()[0])
		+ m_cone.RadiusAtLength(m_extBbox.Max()[0])) / 2;
	length = m_extBbox.Max()[0] - m_extBbox.Min()[0];
	meanLength = (m_extBbox.Max()[0] + m_extBbox.Min()[0]) / 2;
	// the radial extent
	radialExtent = m_extBbox.Max()[1] - m_extBbox.Min()[1];
	// We suggest a cylinder if the opening angle of the cone is so small
	// that over the whole height the difference is less than distThresh
	if(std::sin(m_cone.Angle()) * length / 2 < distThresh)
	{
		// construct the cylinder
		// it has the same axis as the cone
		// and we use the average radius of the cone
		Cylinder cylinder(m_cone.AxisDirection(), m_cone.Center(), 
			meanRadius);
		suggestions->push_back(new CylinderPrimitiveShape(cylinder));
		suggestions->back()->Release();
	}

	// We suggest a sphere if a curvature of mean radius along the height
	// does not introduce too large an error
	float sphereRadius = std::tan(m_cone.Angle()) * meanLength;
	float radiusDiff = (std::sqrt(sphereRadius * sphereRadius + length * length / 4)
		- sphereRadius) / 2;
	if(radiusDiff < distThresh)
	{
		// the center of the sphere is given as the point on the axis
		// with the height of the mean length
		Vec3f center = (meanLength / std::cos(m_cone.Angle()))
			* m_cone.AxisDirection() + m_cone.Center();
		Sphere sphere(center, sphereRadius + radiusDiff);
		suggestions->push_back(new SpherePrimitiveShape(sphere));
		suggestions->back()->Release();
	}

	// We suggest a plane if the mean radius causes only a small error
	// for this we need the angular extent in the curved direction of the cone
	radiusDiff = meanRadius - std::sin(radialExtent) * meanRadius;
	if(radiusDiff < distThresh)
	{
		GfxTL::Vector2Df bboxCenter;
		m_extBbox.Center(&bboxCenter);
		Vec3f pos, normal;
		InSpace(bboxCenter[0], bboxCenter[1] * m_cone.RadiusAtLength(bboxCenter[0]),
			&pos, &normal);
		Plane plane(pos, normal);
		suggestions->push_back(new PlanePrimitiveShape(plane));
		suggestions->back()->Release();
	}*/
}

bool ConePrimitiveShape::Similar(float tolerance,
	const ConePrimitiveShape &shape) const
{
	return m_cone.Angle() <= (1.f + tolerance) * shape.m_cone.Angle()
		&& (1.f + tolerance) * m_cone.Angle() >= shape.m_cone.Angle();
}

void ConePrimitiveShape::BitmapExtent(float epsilon,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox,
	MiscLib::Vector< std::pair< float, float > > *params,
	size_t *uextent, size_t *vextent)
{
	*uextent = std::ceil((bbox->Max()[0] - bbox->Min()[0]) / epsilon); // no wrappig along u direction
	*vextent = std::ceil((bbox->Max()[1] - bbox->Min()[1]) / epsilon) + 1; // add one for wrapping
	if((*vextent) * (*uextent) > 1e6 && m_cone.Angle() < float(M_PI / 4))
	{
		// try to reparameterize
		// try to find cut in the outer regions
		MiscLib::Vector< float > angularParams;//(params->size());
		angularParams.reserve(params->size());
		float outer = 3.f * std::max(fabs(bbox->Min()[0]), fabs(bbox->Max()[0])) / 4.f;
		for(size_t i = 0; i < params->size(); ++i)
			if((*params)[i].first > outer)
				angularParams.push_back(((*params)[i].second
					/ m_cone.RadiusAtLength((*params)[i].first)) + float(M_PI));
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
		// reparameterize with new angular cut
		float newCut = (lower + upper) / 2.f;
		m_cone.RotateAngularDirection(newCut);
		bbox->Min()[1] = std::numeric_limits< float >::infinity();
		bbox->Max()[1] = -std::numeric_limits< float >::infinity();
		for(size_t i = 0; i < params->size(); ++i)
		{
			float r = m_cone.RadiusAtLength((*params)[i].first);
			(*params)[i].second = ((*params)[i].second / r) + float(M_PI) - newCut;
			if((*params)[i].second < 0)
				(*params)[i].second = 2 * float(M_PI) + (*params)[i].second;
			(*params)[i].second = ((*params)[i].second - float(M_PI)) * r;
			if((*params)[i].second < bbox->Min()[1])
				bbox->Min()[1] = (*params)[i].second;
			if((*params)[i].second > bbox->Max()[1])
				bbox->Max()[1] = (*params)[i].second;
		}
		*vextent = std::floor((bbox->Max()[1] - bbox->Min()[1]) / epsilon) + 1;
	}
}

void ConePrimitiveShape::InBitmap(const std::pair< float, float > &param,
	float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	size_t uextent, size_t vextent, std::pair< int, int > *inBmp) const
{
	// convert u = length and v = arc length into bitmap coordinates
	inBmp->first = std::floor((param.first - bbox.Min()[0]) / epsilon);
	inBmp->second = std::floor((param.second - bbox.Min()[1]) / epsilon);
}

void ConePrimitiveShape::PreWrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, float epsilon,
	size_t uextent, size_t vextent, MiscLib::Vector< char > *bmp) const
{
	if(m_cone.Angle() >= float(M_PI / 4))
		return;
	// for wrapping we copy the first pixel to the last one in each v column
	for(size_t u = 0; u < uextent; ++u)
	{
		// determine the coordinates of the last pixel in the column
		// get the radius of the column
		float r = m_cone.RadiusAtLength(u * epsilon + bbox.Min()[0]);
		size_t v = std::floor((2 * float(M_PI) * r - bbox.Min()[1]) / epsilon) + 1;
		if(v >= vextent)
			continue;
		if((*bmp)[u])
			(*bmp)[v * uextent + u] = (*bmp)[u]; // do the wrap
//		if(!(*bmp)[u * vextent + v])
//			(*bmp)[u * vextent + v] = (*bmp)[u * vextent]; // do the wrap
	}
}

void ConePrimitiveShape::WrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, float epsilon, bool *uwrap,
	bool *vwrap) const
{
	*uwrap = *vwrap = false;
}

void ConePrimitiveShape::WrapComponents(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, size_t uextent, size_t vextent,
	MiscLib::Vector< int > *componentImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels) const
{
	if(m_cone.Angle() >= float(M_PI / 4))
		return;
	// for wrapping we copy the first pixel to the last one in each v column
	for(size_t u = 0; u < uextent; ++u)
	{
		// determine the coordinates of the last pixel in the column
		// get the radius of the column
		float r = m_cone.RadiusAtLength(u * epsilon + bbox.Min()[0]);
		size_t v = std::floor((2 * float(M_PI) * r  - bbox.Min()[1]) / epsilon) + 1;
		if(v >= vextent)
			continue;
		if((*componentImg)[u])
			(*componentImg)[v * uextent + u] = (*componentImg)[u]; // do the wrap
	}
	// relabel the components
	MiscLib::Vector< std::pair< int, size_t > > tempLabels(*labels);
	for(size_t u = 0; u < uextent; ++u)
	{
		float r = m_cone.RadiusAtLength(u * epsilon + bbox.Min()[0]);
		size_t v = std::floor((2 * float(M_PI) * r  - bbox.Min()[1]) / epsilon) + 1;
		if(v >= vextent)
			continue;
		if(!(*componentImg)[v * uextent + u])
			continue;
		// get the neighbors
		int n[8];
		size_t i = 0;
		if(v >= 1)
		{
			size_t prevRow = (v - 1) * uextent;
			if(u >= 1)
				n[i++] = (*componentImg)[prevRow + u - 1];
			n[i++] = (*componentImg)[prevRow + u];
			if(u < uextent - 1)
				n[i++] = (*componentImg)[prevRow + u + 1];
		}
		size_t row = v * uextent;
		if(u >= 1)
			n[i++] = (*componentImg)[row + u - 1];
		if(u < uextent - 1)
			n[i++] = (*componentImg)[row + u + 1];
		if(v < vextent - 1)
		{
			size_t nextRow = (v + 1) * uextent;
			if(u >= 1)
				n[i++] = (*componentImg)[nextRow + u - 1];
			n[i++] = (*componentImg)[nextRow + u];
			if(u < uextent - 1)
				n[i++] = (*componentImg)[nextRow + u + 1];
		}
		// associate labels
		int l = (*componentImg)[v * uextent + u];
		for(size_t j = 0; j < i; ++j)
			if(n[j])
				AssociateLabel(l, n[j], &tempLabels);
	}
	// condense labels
	for(size_t i = tempLabels.size() - 1; i > 0; --i)
		tempLabels[i].first = ReduceLabel(i, tempLabels);
	MiscLib::Vector< int > condensed(tempLabels.size());
	labels->clear();
	labels->reserve(condensed.size());
	int count = 0;
    for(size_t i = 0; i < tempLabels.size(); ++i)
		if(i == tempLabels[i].first)
		{
			labels->push_back(std::make_pair(count, tempLabels[i].second));
			condensed[i] = count;
			++count;
		}
		else
			(*labels)[condensed[tempLabels[i].first]].second
				+= tempLabels[i].second;
	// set new component ids
	for(size_t i = 0; i < componentImg->size(); ++i)
		(*componentImg)[i] =
			condensed[tempLabels[(*componentImg)[i]].first];
}

void ConePrimitiveShape::SetExtent(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	const MiscLib::Vector< int > &componentsImg, size_t uextent,
	size_t vextent, float epsilon, int label)
{}

bool ConePrimitiveShape::InSpace(float length, float arcLength, Vec3f *p,
	Vec3f *n) const
{
	float angle;
	if(m_cone.Angle() >= float(M_PI / 4))
	{
		//angle = std::atan2(arcLength, length);
		angle = std::atan2(length, arcLength);
		length = std::sqrt(length * length + arcLength * arcLength);
//		angle = std::acos(GfxTL::Math< float >::Clamp(arcLength / length, -1.f, 1.f));
	}
	else
		angle = (arcLength / m_cone.RadiusAtLength(length)) + float(M_PI);
	GfxTL::Quaternion< float > q;
	q.RotationRad(angle, m_cone.AxisDirection()[0],
		m_cone.AxisDirection()[1], m_cone.AxisDirection()[2]);
	Vec3f vvec;
	q.Rotate(m_cone.AngularDirection(), &vvec);
	*p = std::sin(m_cone.Angle()) * fabs(length) * vvec +
		std::cos(m_cone.Angle()) * length * m_cone.AxisDirection() +
		m_cone.Center();
	m_cone.Normal(*p, n);
	return true;
}

bool ConePrimitiveShape::InSpace(size_t u, size_t v, float epsilon,
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
	size_t vextent, Vec3f *p, Vec3f *n) const
{
	float length, angle;
	if(m_cone.Angle() >= float(M_PI / 4))
	{
		float uf = ((float(u) + .5f) * epsilon) + bbox.Min()[0];
		float vf = ((float(v) + .5f) * epsilon) + bbox.Min()[1];
		length = std::sqrt(uf * uf + vf * vf);
		//angle = std::atan2(vf, uf);
		angle = std::atan2(uf, vf);
		//angle = std::acos(GfxTL::Math< float >::Clamp(vf / length, -1.f, 1.f));
	}
	else
	{
		length = ((float(u) + .5f) * epsilon) + bbox.Min()[0];
		float arcLength = ((float(v) + .5f) * epsilon) + bbox.Min()[1];
		angle = (arcLength / m_cone.RadiusAtLength(length)) + float(M_PI);
	}
	if(angle > 2 * float(M_PI))
		return false;
	//float angle = ((v * epsilon) / m_cone.RadiusAtLength(std::max(fabs(bbox.Min()[0]), fabs(bbox.Max()[0])))
	//	+ bbox.Min()[1]);
	GfxTL::Quaternion< float > q;
	q.RotationRad(angle, m_cone.AxisDirection()[0],
		m_cone.AxisDirection()[1], m_cone.AxisDirection()[2]);
	Vec3f vvec;
	q.Rotate(m_cone.AngularDirection(), &vvec);
	*p = std::sin(m_cone.Angle()) * fabs(length) * vvec +
		std::cos(m_cone.Angle()) * length * m_cone.AxisDirection() +
		m_cone.Center();
	// TODO: this is very lazy and should be optimized!
	m_cone.Normal(*p, n);
	return true;
}
