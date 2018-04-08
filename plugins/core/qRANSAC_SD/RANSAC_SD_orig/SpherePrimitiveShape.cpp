#include "SpherePrimitiveShape.h"
#include "PrimitiveShapeVisitor.h"
#include "Bitmap.h"
#include <GfxTL/AABox.h>
#include <GfxTL/NullClass.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "IndexIterator.h"
#include <GfxTL/KdTree.h>
#include <GfxTL/BBoxDistanceKdTreeStrategy.h>
#include <GfxTL/CellRangeDataTreeStrategy.h>
#include <GfxTL/ImmediateTreeDataKernels.h>
#include <GfxTL/VectorKernel.h>
#include <GfxTL/NullTreeStrategy.h>
#include <GfxTL/IncrementalDistanceKdTreeStrategy.h>
#include <GfxTL/MaxIntervalSplittingKdTreeStrategy.h>
#include <GfxTL/CellBBoxBuildInformationKdTreeStrategy.h>
#include <GfxTL/BBoxBuildInformationTreeStrategy.h>
#include <GfxTL/BucketSizeSubdivisionTreeStrategy.h>
#include <GfxTL/L2Norm.h>
#include <GfxTL/NearestNeighbors.h>
#include <GfxTL/Mean.h>
#include <GfxTL/Covariance.h>
#include <GfxTL/IndexedIterator.h>
#include <GfxTL/Jacobi.h>
#include <MiscLib/Performance.h>
#include "PlanePrimitiveShape.h"
extern MiscLib::performance_t totalTime_sphereConnected;

SpherePrimitiveShape::SpherePrimitiveShape(const Sphere &s)
: m_sphere(s)
, m_parametrization(m_sphere)
{}

SpherePrimitiveShape::SpherePrimitiveShape(const SpherePrimitiveShape &sps)
: BitmapPrimitiveShape(sps)
, m_sphere(sps.m_sphere)
, m_parametrization(sps.m_parametrization)
{
	m_parametrization.Shape(m_sphere);
}

size_t SpherePrimitiveShape::Identifier() const
{
	return 1;
}

bool SpherePrimitiveShape::Init(bool binary, std::istream *i)
{
	// read the polygons but ignore them
	GfxTL::AABox< GfxTL::Vector2Df > bboxUpper, bboxLower;
	size_t upperuextent, uppervextent, loweruextent, lowervextent;
	if(binary)
	{
		// read number of components
		size_t size;
		i->read((char *)&size, sizeof(size));
		if(size)
		{
			// read upper bbox
			i->read((char *)&bboxUpper, sizeof(bboxUpper));
			// read upperuextent and uppervextent
			i->read((char *)&upperuextent, sizeof(upperuextent));
			i->read((char *)&uppervextent, sizeof(uppervextent));
			for(size_t j = 0; j < size; ++j)
			{
				// read number of polys of component
				size_t numPolys;
				i->read((char *)&numPolys, sizeof(numPolys));
				for(size_t k = 0; k < numPolys; ++k)
				{
					// read number of points in poly
					size_t numPoints;
					i->read((char *)&numPoints, sizeof(numPoints));
					GfxTL::VectorXD< 2, size_t > pp;
					for(size_t l = 0; l < numPoints; ++l)
						i->read((char *)&pp, sizeof(pp));
				}
			}
		}
		// do the same for lower bitmap
		// read number of components
		i->read((char *)&size, sizeof(size));
		if(size)
		{
			// read lower bbox
			i->read((char *)&bboxLower, sizeof(bboxLower));
			// read loweruextent and lowervextent
			i->read((char *)&loweruextent, sizeof(loweruextent));
			i->read((char *)&lowervextent, sizeof(lowervextent));
			for(size_t j = 0; j < size; ++j)
			{
				// read number of polys of component
				size_t numPolys;
				i->read((char *)&numPolys, sizeof(numPolys));
				for(size_t k = 0; k < numPolys; ++k)
				{
					// read number of points in poly
					size_t numPoints;
					i->read((char *)&numPoints, sizeof(numPoints));
					GfxTL::VectorXD< 2, size_t > pp;
					for(size_t l = 0; l < numPoints; ++l)
						i->read((char *)&pp, sizeof(pp));
				}
			}
		}
	}
	else
	{
		// read number of components
		size_t size;
		(*i) >> size;
		if(size)
		{
			// read upper bbox
			(*i) >> bboxUpper.Min()[0] >> bboxUpper.Max()[0]
				>> bboxUpper.Min()[1] >> bboxUpper.Max()[1];
			// read upperuextent and uppervextent
			(*i) >> upperuextent >> uppervextent;
			for(size_t j = 0; j < size; ++j)
			{
				// read number of polys of component
				size_t numPolys;
				(*i) >> numPolys;
				for(size_t k = 0; k < numPolys; ++k)
				{
					// read number of points in poly
					size_t numPoints;
					(*i) >> numPoints;
					GfxTL::VectorXD< 2, size_t > pp;
					for(size_t l = 0; l < numPoints; ++l)
						(*i) >> pp[0] >> pp[1];
				}
			}
		}
		// read number of components
		(*i) >> size;
		if(size)
		{
			// read lower bbox
			(*i) >> bboxLower.Min()[0] >> bboxLower.Max()[0]
				>> bboxLower.Min()[1] >> bboxLower.Max()[1];
			// read loweruextent and lowervextent
			(*i) >> loweruextent >> lowervextent;
			for(size_t j = 0; j < size; ++j)
			{
				// read number of polys of component
				size_t numPolys;
				(*i) >> numPolys;
				for(size_t k = 0; k < numPolys; ++k)
				{
					// read number of points in poly
					size_t numPoints;
					(*i) >> numPoints;
					GfxTL::VectorXD< 2, size_t > pp;
					for(size_t l = 0; l < numPoints; ++l)
						(*i) >> pp[0] >> pp[1];
				}
			}
		}
	}
	return true;
}

PrimitiveShape *SpherePrimitiveShape::Clone() const
{
	return new SpherePrimitiveShape(*this);
}

float SpherePrimitiveShape::Distance(const Vec3f &p) const
{
	return m_sphere.Distance(p);
}

float SpherePrimitiveShape::SignedDistance(const Vec3f &p) const
{
	return m_sphere.SignedDistance(p);
}

float SpherePrimitiveShape::NormalDeviation(const Vec3f &p,
	const Vec3f &n) const
{
	Vec3f normal;
	m_sphere.Normal(p, &normal);
	return n.dot(normal);
}

void SpherePrimitiveShape::DistanceAndNormalDeviation(
	const Vec3f &p, const Vec3f &n, std::pair< float, float > *dn) const
{
	Vec3f normal;
	dn->first = m_sphere.DistanceAndNormal(p, &normal);
	dn->second = n.dot(normal);
}

void SpherePrimitiveShape::Project(const Vec3f &p, Vec3f *pp) const
{
	m_sphere.Project(p, pp);
}

void SpherePrimitiveShape::Normal(const Vec3f &p, Vec3f *n) const
{
	m_sphere.Normal(p, n);
}

unsigned int SpherePrimitiveShape::ConfidenceTests(unsigned int numTests,
	float epsilon, float normalThresh, float rms, const PointCloud &pc,
	const MiscLib::Vector< size_t > &indices) const
{
	return BasePrimitiveShape::ConfidenceTests< Sphere >(numTests, epsilon,
		normalThresh, rms, pc, indices);
}

void SpherePrimitiveShape::Description(std::string *s) const
{
	*s = "Sphere";
}

bool SpherePrimitiveShape::Fit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end)
{
	// do LS-fitting
	Sphere fit = m_sphere;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		m_sphere = fit;
		m_parametrization.Shape(m_sphere);
		return true;
	}
	return false;
}

PrimitiveShape *SpherePrimitiveShape::LSFit(const PointCloud &pc, float epsilon,
	float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end,
	std::pair< size_t, float > *score) const
{
	// do LS-fitting
	Sphere fit = m_sphere;
	if(fit.LeastSquaresFit(pc, begin, end))
	{
		score->first = -1;
		return new SpherePrimitiveShape(fit);
	}
	score->first = 0;
	return NULL;
}

void SpherePrimitiveShape::Serialize(std::ostream *o, bool binary) const
{
	if(binary)
	{
		const char id = 1;
		(*o) << id;
	}
	else
		(*o) << "1" << " ";
	m_sphere.Serialize(binary, o);
	m_parametrization.Serialize(o, binary);
	if(!binary)
		*o << std::endl;
}

void SpherePrimitiveShape::Deserialize(std::istream *i, bool binary)
{
	m_sphere.Init(binary, i);
	m_parametrization.Shape(m_sphere);
	m_parametrization.Deserialize(i, binary);
}

size_t SpherePrimitiveShape::SerializedSize() const
{
	return m_sphere.SerializedSize()
		+ m_parametrization.SerializedSize() + 1;
}

LevMarFunc< float > *SpherePrimitiveShape::SignedDistanceFunc() const
{
	return new SphereLevMarFunc(m_sphere);
}

void SpherePrimitiveShape::Transform(float scale, const Vec3f &translate)
{
	m_sphere.Transform(scale, translate);
}

void SpherePrimitiveShape::Visit(PrimitiveShapeVisitor *visitor) const
{
	visitor->Visit(*this);
}

void SpherePrimitiveShape::SuggestSimplifications(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end, float distThresh,
	MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const
{
	// sample the bounding box in parameter space at 25 locations
	// these points are used to estimate the other shapes
	// if the shapes succeed the suggestion is returned
	MiscLib::Vector< Vec3f > samples;
	samples.resize(2 * 25);
	size_t c = samples.size() / 2;
	float uStep = (m_extBbox.Max()[0] - m_extBbox.Min()[0]) / 4;
	float vStep = (m_extBbox.Max()[1] - m_extBbox.Min()[1]) / 4;
	float u = m_extBbox.Min()[0];
	for(unsigned int i = 0; i < 5; ++i, u += uStep)
	{
		float v = m_extBbox.Min()[1];
		for(unsigned int j = 0; j < 5; ++j, v += vStep)
			m_parametrization.InSpace(u, v,
				&samples[i * 5 + j], &samples[i * 5 + j + c]);
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

	/*// we suggest a plane if the radius is large enough so that the error
	// along the two directions is less than distThresh
	float ulength = 0, vlength = 0;
	if(m_hasBitmap.first)
	{
		// has points on the upper side
		ulength = m_extBboxUpper.Max()[0] - m_extBboxUpper.Min()[0];
		vlength = m_extBboxUpper.Max()[1] - m_extBboxUpper.Min()[1];
	}
	if(m_hasBitmap.second)
	{
		ulength += m_extBboxLower.Max()[0] - m_extBboxLower.Min()[0];
		vlength += m_extBboxLower.Max()[1] - m_extBboxLower.Min()[1];
	}
	float arcLength = M_PI * std::max(ulength, vlength);
	float radiusDiff = (m_sphere.Radius() - std::sin(arcLength / 2)
		* m_sphere.Radius()) / 2;
	if(radiusDiff < distThresh)
	{
		Vec3f pos, normal;
		SphereAsSquaresParametrization ssp(m_sphere, m_parametrizationNormal);
		if(m_hasBitmap.first && m_hasBitmap.second)
		{
			GfxTL::Vector3Df center, eigenValues;
			GfxTL::Mean(GfxTL::IndexIterate(begin, pc.begin()),
				GfxTL::IndexIterate(end, pc.begin()), &center);
			GfxTL::MatrixXX< 3, 3, float > cov, eigenVectors;
			GfxTL::CovarianceMatrix(center, GfxTL::IndexIterate(begin, pc.begin()),
				GfxTL::IndexIterate(end, pc.begin()), &cov);
			GfxTL::Jacobi(cov, &eigenValues, &eigenVectors);
			GfxTL::EigSortDesc(&eigenValues, &eigenVectors);
			GfxTL::Vector3Df n = GfxTL::Vector3Df(eigenVectors[2]);
			Plane plane(Vec3f(center.Data()), Vec3f(n.Data()));
			suggestions->push_back(new PlanePrimitiveShape(plane));
			suggestions->back()->Release();
		}
		else if(m_hasBitmap.first)
		{
			GfxTL::Vector2Df center;
			m_extBboxUpper.Center(&center);
			ssp.InSpace(std::make_pair(center[0], center[1]), false, &pos, &normal);
			// offset position
			pos -= radiusDiff * normal;
			Plane plane(pos, normal);
			suggestions->push_back(new PlanePrimitiveShape(plane));
			suggestions->back()->Release();
		}
		else if(m_hasBitmap.second)
		{
			GfxTL::Vector2Df center;
			m_extBboxUpper.Center(&center);
			ssp.InSpace(std::make_pair(center[0], center[1]), true, &pos, &normal);
			// offset position
			pos -= radiusDiff * normal;
			Plane plane(pos, normal);
			suggestions->push_back(new PlanePrimitiveShape(plane));
			suggestions->back()->Release();
		}
	}*/
}

void SpherePrimitiveShape::OptimizeParametrization(const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end, float epsilon)
{
	m_parametrization.Optimize(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()), epsilon);
}

void SpherePrimitiveShape::OptimizeParametrization(const PointCloud &pc,
	size_t begin, size_t end, float epsilon)
{
	m_parametrization.Optimize(GfxTL::IndexIterate(IndexIterator(begin), pc.begin()),
		GfxTL::IndexIterate(IndexIterator(end), pc.begin()), epsilon);
}

bool SpherePrimitiveShape::Similar(float tolerance,
	const SpherePrimitiveShape &shape) const
{
	return m_sphere.Radius() <= (1.f + tolerance) * shape.m_sphere.Radius()
		&& (1.f + tolerance) * m_sphere.Radius() >= shape.m_sphere.Radius();
}

void SpherePrimitiveShape::Parameters(const Vec3f &p,
	std::pair< float, float > *param) const
{
	m_parametrization.Parameters(p, param);
}

void SpherePrimitiveShape::Parameters(
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

void SpherePrimitiveShape::Parameters(
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > begin,
	GfxTL::IndexedIterator< IndexIterator,
		PointCloud::const_iterator > end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	ParametersImpl(begin, end, bmpParams);
}

bool SpherePrimitiveShape::InSpace(float u, float v, Vec3f *p, Vec3f *n) const
{
	return m_parametrization.InSpace(u, v, p, n);
}

void SpherePrimitiveShape::BitmapExtent(float epsilon,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox,
	MiscLib::Vector< std::pair< float, float > > *params,
	size_t *uextent, size_t *vextent)
{
	*uextent = std::ceil((bbox->Max()[0] - bbox->Min()[0]) / epsilon);
	*vextent = std::ceil((bbox->Max()[1] - bbox->Min()[1]) / epsilon);
}

void SpherePrimitiveShape::InBitmap(const std::pair< float, float > &param,
	float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	size_t uextent, size_t vextent,
	std::pair< int, int > *inBmp) const
{
	inBmp->first = std::floor((param.first - bbox.Min()[0]) / epsilon);
	inBmp->second = std::floor((param.second - bbox.Min()[1]) / epsilon);
}

void SpherePrimitiveShape::WrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, bool *uwrap, bool *vwrap) const
{
	m_parametrization.WrapBitmap(bbox, epsilon, uwrap, vwrap);
}

void SpherePrimitiveShape::WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, size_t uextent, size_t vextent,
	MiscLib::Vector< int > *componentImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels) const
{
	m_parametrization.WrapComponents(bbox, epsilon, uextent, vextent,
		componentImg, labels);
}

bool SpherePrimitiveShape::InSpace(size_t u, size_t v, float epsilon,
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
	size_t vextent, Vec3f *p, Vec3f *n) const
{
	return m_parametrization.InSpace((u + .5f) * epsilon + bbox.Min()[0],
		(v + .5f) * epsilon + bbox.Min()[1], p, n);
}
