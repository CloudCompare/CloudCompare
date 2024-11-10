#ifndef TORUSPRIMITIVESHAPE_HEADER
#define TORUSPRIMITIVESHAPE_HEADER
#include "BitmapPrimitiveShape.h"
#include "Torus.h"
#include "SimpleTorusParametrization.h"
#include "LowStretchTorusParametrization.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE TorusPrimitiveShape
: public BitmapPrimitiveShape
{
public:
	typedef LowStretchTorusParametrization ParametrizationType;
	TorusPrimitiveShape() 
		: m_parametrization(m_torus)
		, m_allowAppleShaped(false)
		, m_minMinorRadius(-std::numeric_limits<float>::infinity())
		, m_minMajorRadius(-std::numeric_limits<float>::infinity())
		, m_maxMinorRadius(std::numeric_limits<float>::infinity())
		, m_maxMajorRadius(std::numeric_limits<float>::infinity())
	{}
	TorusPrimitiveShape(const Torus &torus, bool allowAppleShaped = false
		, float minMinorRadius = -std::numeric_limits<float>::infinity()
		, float minMajorRadius = -std::numeric_limits<float>::infinity()
		, float maxMinorRadius = std::numeric_limits<float>::infinity()
		, float maxMajorRadius = std::numeric_limits<float>::infinity());
	TorusPrimitiveShape(const TorusPrimitiveShape &tps);
	size_t Identifier() const override;
	unsigned int RequiredSamples() const  override { return Torus::RequiredSamples; }
	PrimitiveShape *Clone() const override;
	float Distance(const Vec3f &p) const override;
	float SignedDistance(const Vec3f &p) const override;
	float NormalDeviation(const Vec3f &p, const Vec3f &n) const override;
	void DistanceAndNormalDeviation(const Vec3f &p,
		const Vec3f &n, std::pair< float, float > *dn) const override;
	void Project(const Vec3f &p, Vec3f *pp) const override;
	void Normal(const Vec3f &p, Vec3f *n) const override;
	unsigned int ConfidenceTests(unsigned int numTests, float epsilon,
		float normalThresh, float rms, const PointCloud &pc,
		const MiscLib::Vector< size_t > &indices) const override;
	void Description(std::string *s) const override;
	// refitting
	bool Fit(const PointCloud &pc, float epsilon,
		float normalThresh,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end) override;
	PrimitiveShape *LSFit(const PointCloud &pc, float epsilon,
		float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end,
		std::pair< size_t, float > *score) const override;
	LevMarFunc< float > *SignedDistanceFunc() const override;
	void Serialize(std::ostream *o, bool binary = true) const override;
	void Deserialize(std::istream *i, bool binary);
	size_t SerializedSize() const override;
	virtual void Serialize(float* array) const override { m_torus.Serialize(array); }
	virtual size_t SerializedFloatSize() const override { return m_torus.SerializedFloatSize(); }
	void Transform(float scale, const Vec3f &translate) override;
	void Visit(PrimitiveShapeVisitor *visitor) const override;
	void SuggestSimplifications(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end, float distThresh,
		MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const override;
	void OptimizeParametrization(const PointCloud &pc,
		size_t begin, size_t end, float epsilon) override;
	bool Similar(float tolerance, const TorusPrimitiveShape &shape) const;

	// implementation of BitmapPrimitiveShape
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
	void SetExtent(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		const MiscLib::Vector< int > &componentsImg, size_t uextent,
		size_t vextent, float epsilon, int label);
	bool InSpace(size_t u, size_t v, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, Vec3f *p, Vec3f *n) const override;
	const Torus &Internal() const { return m_torus; }
	bool CheckGeneratedShapeWithinLimits(const PointCloud& pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end) override
	{
		if ((!m_allowAppleShaped && m_torus.IsAppleShaped()) || 
			m_torus.MinorRadius() < m_minMinorRadius ||
			m_torus.MajorRadius() < m_minMajorRadius ||
			m_torus.MinorRadius() > m_maxMinorRadius || 
			m_torus.MajorRadius() > m_maxMajorRadius)
		{
			return false;
		}
		return true;
	}

private:
	template< class IteratorT >
	void ParametersImpl(IteratorT begin, IteratorT end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const;

private:
	Torus m_torus;
	ParametrizationType m_parametrization;
	bool m_allowAppleShaped;
	float m_minMinorRadius;
	float m_minMajorRadius;
	float m_maxMinorRadius;
	float m_maxMajorRadius;
};

template< class IteratorT >
void TorusPrimitiveShape::ParametersImpl(IteratorT begin, IteratorT end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	bmpParams->resize(end - begin);
	size_t j = 0;
	for(IteratorT i = begin; i != end; ++i, ++j)
		m_parametrization.Parameters(*i, &(*bmpParams)[j]);
}

class DLL_LINKAGE TorusLevMarFunc
: public LevMarFunc< float >
{
public:
	TorusLevMarFunc(const Torus &t)
	: m_torus(t)
	{}

	float operator()(const float *x) const
	{
		return m_torus.SignedDistance(*((const Vec3f *)x));
	}

	void operator()(const float *x, float *gradient) const
	{
		m_torus.Normal(*((const Vec3f *)x), (Vec3f *)gradient);
	}

private:
	Torus m_torus;
};

#endif
