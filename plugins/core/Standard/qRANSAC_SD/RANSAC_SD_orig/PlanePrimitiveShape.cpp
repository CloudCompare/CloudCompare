#include "PlanePrimitiveShape.h"
#include "ScoreComputer.h"
#include "PrimitiveShapeVisitor.h"
#include <GfxTL/NullClass.h>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <MiscLib/Performance.h>
extern MiscLib::performance_t totalTime_planeConnected;

PlanePrimitiveShape::PlanePrimitiveShape(const Vec3f &a, const Vec3f &b,
	const Vec3f &c)
: m_plane(a, b, c)
{
	m_hcs.FromNormal(m_plane.getNormal());
}

PlanePrimitiveShape::PlanePrimitiveShape(const Plane &plane)
: m_plane(plane)
{
	m_hcs.FromNormal(m_plane.getNormal());
}

size_t PlanePrimitiveShape::Identifier() const
{
	return 0;
}

PrimitiveShape *PlanePrimitiveShape::Clone() const
{
	return new PlanePrimitiveShape(*this);
}

float PlanePrimitiveShape::Distance(const Vec3f &p) const
{
	return m_plane.getDistance(p);
}

float PlanePrimitiveShape::SignedDistance(const Vec3f &p) const
{
	return m_plane.SignedDistance(p);
}

float PlanePrimitiveShape::NormalDeviation(const Vec3f &,
	const Vec3f &n) const
{
	return m_plane.getNormal().dot(n);
}

void PlanePrimitiveShape::DistanceAndNormalDeviation(
	const Vec3f &p, const Vec3f &n, std::pair< float, float > *dn) const
{
	dn->first = m_plane.getDistance(p);
	dn->second = m_plane.getNormal().dot(n);
}

void PlanePrimitiveShape::Project(const Vec3f &p, Vec3f *pp) const
{
	*pp = p - ((m_plane.getNormal().dot(p - m_plane.getPosition()))
		* m_plane.getNormal());
}

void PlanePrimitiveShape::Normal(const Vec3f &p, Vec3f *n) const
{
	*n = m_plane.getNormal();
}

unsigned int PlanePrimitiveShape::ConfidenceTests(unsigned int numTests,
	float epsilon, float normalThresh, float rms, const PointCloud &pc,
	const MiscLib::Vector< size_t > &indices) const
{
	return BasePrimitiveShape::ConfidenceTests< Plane >(numTests, epsilon,
		normalThresh, rms, pc, indices);
}

void PlanePrimitiveShape::Description(std::string *s) const
{
	*s = "Plane";
}

bool PlanePrimitiveShape::Fit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	Plane fit = m_plane;
	//if(fit.LeastSquaresFit(pc, begin, end))
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		m_plane = fit;
		m_hcs.FromNormal(m_plane.getNormal());
		return true;
	}
	return false;
}

PrimitiveShape *PlanePrimitiveShape::LSFit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end,
	std::pair< size_t, float > *score) const
{
	Plane fit = m_plane;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		score->first = -1;
		return new PlanePrimitiveShape(fit);
	}
	score->first = 0;
	return NULL;
}

void PlanePrimitiveShape::Serialize(std::ostream *o, bool binary) const
{
	if(binary)
	{
		const char id = 0;
		(*o) << id;
	}
	else
		(*o) << "0" << " ";
	m_plane.Serialize(binary, o);
	if(!binary)
		*o << std::endl;
}

size_t PlanePrimitiveShape::SerializedSize() const
{
	return m_plane.SerializedSize() + 1;
}

void PlanePrimitiveShape::Transform(float scale, const Vec3f &translate)
{
	m_plane.Transform(scale, translate);
}

void PlanePrimitiveShape::Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
	const GfxTL::Vector3Df &trans)
{
	GfxTL::Vector3Df transNormal = rot * GfxTL::Vector3Df(
		m_plane.getNormal());
	GfxTL::Vector3Df transOrigin = rot * GfxTL::Vector3Df(
		m_plane.getPosition()) + trans;
	m_plane = Plane(Vec3f(transOrigin.Data()), Vec3f(transNormal.Data()));
	m_hcs[0] = rot * m_hcs[0];
	m_hcs[1] = rot * m_hcs[1];
}

void PlanePrimitiveShape::Visit(PrimitiveShapeVisitor *visitor) const
{
	visitor->Visit(*this);
}

bool PlanePrimitiveShape::Similar(float, const PlanePrimitiveShape &) const
{
	return true; // planes are always similar!
}

LevMarFunc< float > *PlanePrimitiveShape::SignedDistanceFunc() const
{
	return new PlaneLevMarFunc(m_plane);
}

void PlanePrimitiveShape::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	Vec3f pp = p - m_plane.getPosition();
	param->first = pp.dot(m_hcs[0].Data());
	param->second = pp.dot(m_hcs[1].Data());
}

void PlanePrimitiveShape::Parameters(
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void PlanePrimitiveShape::Parameters(
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void PlanePrimitiveShape::BitmapExtent(float epsilon,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox,
	MiscLib::Vector< std::pair< float, float > > *params,
	size_t *uextent, size_t *vextent)
{
	*uextent = size_t(std::ceil((bbox->Max()[0] - bbox->Min()[0]) / epsilon)) + 1;
	*vextent = size_t(std::ceil((bbox->Max()[1] - bbox->Min()[1]) / epsilon)) + 1;
}

void PlanePrimitiveShape::InBitmap(const std::pair< float, float > &param,
	float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t,
	size_t, std::pair< int, int > *inBmp) const
{
	inBmp->first = (int)std::floor((param.first - bbox.Min()[0]) / epsilon);
	inBmp->second = (int)std::floor((param.second - bbox.Min()[1]) / epsilon);
}
void PlanePrimitiveShape::WrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, float epsilon, bool *uwrap,
	bool *vwrap) const
{
	*uwrap = false;
	*vwrap = false;
}

void PlanePrimitiveShape::SetExtent(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, const MiscLib::Vector< int > &,
	size_t, size_t, float, int)
{}

bool PlanePrimitiveShape::InSpace(float u, float v, Vec3f *p, Vec3f *n) const
{
	*p = m_plane.getPosition() + Vec3f((u * m_hcs[0] + v * m_hcs[1]).Data());
	*n = m_plane.getNormal();
	return true;
}

bool PlanePrimitiveShape::InSpace(size_t u, size_t v, float epsilon,
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
	size_t vextent, Vec3f *p, Vec3f *n) const
{
	*p = Vec3f(((bbox.Min()[0] + epsilon * (float(u) + .5f)) * m_hcs[0] +
		(bbox.Min()[1] + epsilon * (float(v) + .5f)) * m_hcs[1]).Data()) +
		m_plane.getPosition();
	*n = m_plane.getNormal();
	return true;
}
