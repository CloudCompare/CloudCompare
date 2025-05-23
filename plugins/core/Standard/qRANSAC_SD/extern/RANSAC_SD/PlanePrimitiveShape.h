#ifndef PLANEPRIMITIVESHAPE_HEADER
#define PLANEPRIMITIVESHAPE_HEADER
#include "BitmapPrimitiveShape.h"
#include "Plane.h"
#include <GfxTL/HyperplaneCoordinateSystem.h>
#include <GfxTL/AABox.h>
#include <GfxTL/MatrixXX.h>
#include <GfxTL/VectorXD.h>
#include "LevMarFunc.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE PlanePrimitiveShape
: public BitmapPrimitiveShape
{
public:
	PlanePrimitiveShape(const Vec3f &a, const Vec3f &b, const Vec3f &c);
	PlanePrimitiveShape(const Plane &plane);
	size_t Identifier() const override;
	unsigned int RequiredSamples() const  override { return Plane::RequiredSamples; }
	PrimitiveShape *Clone() const override;
	float Distance(const Vec3f &p) const override;
	float SignedDistance(const Vec3f &p) const override;
	float NormalDeviation(const Vec3f &p, const Vec3f &n) const override;
	void DistanceAndNormalDeviation(const Vec3f &p, const Vec3f &n,
		std::pair< float, float > *dn) const override;
	void Project(const Vec3f &p, Vec3f *pp) const override;
	void Normal(const Vec3f &p, Vec3f *n) const override;
	unsigned int ConfidenceTests(unsigned int numTests, float epsilon,
		float normalThresh, float rms, const PointCloud &pc,
		const MiscLib::Vector< size_t > &indices) const override;
	void Description(std::string *s) const override;
	bool Fit(const PointCloud &pc, float epsilon, float normalThresh,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end) override;
	PrimitiveShape *LSFit(const PointCloud &pc, float epsilon,
		float normalThresh,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end,
		std::pair< size_t, float > *score) const override;
	LevMarFunc< float > *SignedDistanceFunc() const override;
	void Serialize(std::ostream *o, bool binary = true) const override;
	size_t SerializedSize() const override;
	virtual void Serialize(float* array) const  override { m_plane.Serialize(array); }
	virtual size_t SerializedFloatSize() const  override { return m_plane.SerializedFloatSize(); }
	void Transform(float scale, const Vec3f &translate) override;
	void Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
		const GfxTL::Vector3Df &trans);
	void Visit(PrimitiveShapeVisitor *visitor) const override;
	bool Similar(float tolerance, const PlanePrimitiveShape &) const;
	const Plane &Internal() const { return m_plane; }

	void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const override;
	void Parameters(GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
			PointCloud::const_iterator > begin,
		GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
			PointCloud::const_iterator > end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const override;
	void Parameters(GfxTL::IndexedIterator< IndexIterator,
			PointCloud::const_iterator > begin,
		GfxTL::IndexedIterator< IndexIterator,
			PointCloud::const_iterator > end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const override;
	bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const override;
	void BitmapExtent(float epsilon,
		GfxTL::AABox< GfxTL::Vector2Df > *bbox,
		MiscLib::Vector< std::pair< float, float > > *params,
		size_t *uextent, size_t *vextent) override;
	void InBitmap(const std::pair< float, float > &param, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, std::pair< int, int > *inBmp) const override;
	void WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, bool *uwrap, bool *vwrap) const override;
	void SetExtent(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		const MiscLib::Vector< int > &componentsImg, size_t uextent,
		size_t vextent, float epsilon, int label);
	bool InSpace(size_t u, size_t v, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, Vec3f *p, Vec3f *n) const override;

	Vec3f getXDim() const { return Vec3f(m_hcs[0].Data()); }
	Vec3f getYDim() const { return Vec3f(m_hcs[1].Data()); }

private:
	template< class IteratorT >
	void ParametersImpl(IteratorT begin, IteratorT end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const;

private:
	Plane m_plane;
	GfxTL::HyperplaneCoordinateSystem< float, 3 > m_hcs;
};

template< class IteratorT >
void PlanePrimitiveShape::ParametersImpl(IteratorT begin, IteratorT end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	bmpParams->resize(end - begin);
	size_t j = 0;
	for(IteratorT i = begin; i != end; ++i, ++j)
	{
		Vec3f pp = *i - m_plane.getPosition();
		(*bmpParams)[j].first = pp.dot(m_hcs[0].Data());
		(*bmpParams)[j].second = pp.dot(m_hcs[1].Data());
	}
}

class DLL_LINKAGE PlaneLevMarFunc
: public LevMarFunc< float >
{
public:
	PlaneLevMarFunc(const Plane &plane)
	{
		for(unsigned int i = 0; i < 3; ++i)
			m_plane[i] = plane.getNormal()[i];
		m_plane[3] = plane.SignedDistToOrigin();
	}

	float operator()(const float *x) const
	{
		return m_plane[0] * x[0] + m_plane[1] * x[1] +
			m_plane[2] * x[2] - m_plane[3];
	}

	void operator()(const float *x, float *gradient) const
	{
		for(unsigned int i = 0; i < 3; ++i)
			gradient[i] = m_plane[i];
	}

private:
	GfxTL::Vector4Df m_plane;
};

#endif
