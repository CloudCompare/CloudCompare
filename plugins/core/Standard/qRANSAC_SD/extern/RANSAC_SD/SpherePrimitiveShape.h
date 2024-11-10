#ifndef SPHEREPRIMITIVESHAPE_HEADER
#define SPHEREPRIMITIVESHAPE_HEADER
#include "BitmapPrimitiveShape.h"
#include "Sphere.h"
#include <GfxTL/AABox.h>
#include "LevMarFunc.h"
#include <istream>
#include <GfxTL/MathHelper.h>
#include <GfxTL/IndexedIterator.h>
#include <GfxTL/Mean.h>
#include <GfxTL/Covariance.h>
#include <GfxTL/Jacobi.h>
#include "LowStretchSphereParametrization.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE SpherePrimitiveShape
: public BitmapPrimitiveShape
{
public:
	typedef LowStretchSphereParametrization ParametrizationType;
	SpherePrimitiveShape() : m_parametrization(m_sphere), m_minRadius(-std::numeric_limits<float>::infinity()), m_maxRadius(std::numeric_limits<float>::infinity()) {}
	SpherePrimitiveShape(const Sphere &s, float minRadius = -std::numeric_limits<float>::infinity(), float maxRadius = std::numeric_limits<float>::infinity());
	SpherePrimitiveShape(const SpherePrimitiveShape &sps);
	size_t Identifier() const override;
	unsigned int RequiredSamples() const override { return Sphere::RequiredSamples; }
	bool Init(bool binary, std::istream *i);
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
	void Deserialize(std::istream *i, bool binary);
	size_t SerializedSize() const override;
	virtual void Serialize(float* array) const  override { m_sphere.Serialize(array); }
	virtual size_t SerializedFloatSize() const  override { return m_sphere.SerializedFloatSize(); }
		
	void Transform(float scale, const Vec3f &translate) override;
	void Visit(PrimitiveShapeVisitor *visitor) const override;
	void SuggestSimplifications(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end, float distThresh,
		MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const override;
	void OptimizeParametrization(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end, float epsilon);
	void OptimizeParametrization(const PointCloud &pc,
		size_t begin, size_t end, float epsilon) override;
	bool Similar(float tolerance, const SpherePrimitiveShape &shape) const;
	const Sphere &Internal() const { return m_sphere; }

	// implementation of bitmap primitive shape
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
	void InBitmap(const std::pair< float, float > &param,
		float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		size_t uextent, size_t vextent,
		std::pair< int, int > *inBmp) const override;
	void WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, bool *uwrap, bool *vwrap) const override;
	void WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< int > *componentImg,
		MiscLib::Vector< std::pair< int, size_t > > *labels) const override;
	bool InSpace(size_t u, size_t v, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, Vec3f *p, Vec3f *n) const override;
	bool CheckGeneratedShapeWithinLimits(const PointCloud& pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end) override
	{
		if ((m_sphere.Radius() >= m_minRadius) && (m_sphere.Radius() <= m_maxRadius))
		{
			return true;
		}
		return false;
	}
private:
	template< class IteratorT >
	void ParametersImpl(IteratorT begin, IteratorT end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const;

private:
	Sphere m_sphere;
	float m_minRadius;
	float m_maxRadius;
	ParametrizationType m_parametrization;
};

template< class IteratorT >
void SpherePrimitiveShape::ParametersImpl(IteratorT begin, IteratorT end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	bmpParams->resize(end - begin);
	size_t j = 0;
	for(IteratorT i = begin; i != end; ++i, ++j)
		m_parametrization.Parameters(*i, &(*bmpParams)[j]);
}

class DLL_LINKAGE SphereLevMarFunc
: public LevMarFunc< float >
{
public:
	SphereLevMarFunc(const Sphere &s)
	: m_sphere(s)
	{}

	float operator()(const float *x) const
	{
		return m_sphere.SignedDistance(*((const Vec3f *)x));
	}

	void operator()(const float *x, float *gradient) const
	{
		m_sphere.Normal(*((const Vec3f *)x), (Vec3f *)gradient);
	}

private:
	Sphere m_sphere;
};

#endif
