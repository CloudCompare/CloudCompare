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
	TorusPrimitiveShape() : m_parametrization(m_torus) {}
	TorusPrimitiveShape(const Torus &torus);
	TorusPrimitiveShape(const TorusPrimitiveShape &tps);
	size_t Identifier() const;
	unsigned int RequiredSamples() const { return Torus::RequiredSamples; }
	PrimitiveShape *Clone() const;
	float Distance(const Vec3f &p) const;
	float SignedDistance(const Vec3f &p) const;
	float NormalDeviation(const Vec3f &p, const Vec3f &n) const;
	void DistanceAndNormalDeviation(const Vec3f &p,
		const Vec3f &n, std::pair< float, float > *dn) const;
	void Project(const Vec3f &p, Vec3f *pp) const;
	void Normal(const Vec3f &p, Vec3f *n) const;
	unsigned int ConfidenceTests(unsigned int numTests, float epsilon,
		float normalThresh, float rms, const PointCloud &pc,
		const MiscLib::Vector< size_t > &indices) const;
	void Description(std::string *s) const;
	// refitting
	bool Fit(const PointCloud &pc, float epsilon,
		float normalThresh,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end);
	PrimitiveShape *LSFit(const PointCloud &pc, float epsilon,
		float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end,
		std::pair< size_t, float > *score) const;
	LevMarFunc< float > *SignedDistanceFunc() const;
	void Serialize(std::ostream *o, bool binary = true) const;
	void Deserialize(std::istream *i, bool binary);
	size_t SerializedSize() const;
	virtual void Serialize(float* array) const {m_torus.Serialize(array);}
	virtual size_t SerializedFloatSize() const {return m_torus.SerializedFloatSize();}		
	void Transform(float scale, const Vec3f &translate);
	void Visit(PrimitiveShapeVisitor *visitor) const;
	void SuggestSimplifications(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end, float distThresh,
		MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const;
	void OptimizeParametrization(const PointCloud &pc,
		size_t begin, size_t end, float epsilon);
	bool Similar(float tolerance, const TorusPrimitiveShape &shape) const;

	// implementation of BitmapPrimitiveShape
	void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const;
	void Parameters(GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
			PointCloud::const_iterator > begin,
		GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
			PointCloud::const_iterator > end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const;
	void Parameters(GfxTL::IndexedIterator< IndexIterator,
			PointCloud::const_iterator > begin,
		GfxTL::IndexedIterator< IndexIterator,
			PointCloud::const_iterator > end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const;
	bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const;
	void BitmapExtent(float epsilon,
		GfxTL::AABox< GfxTL::Vector2Df > *bbox,
		MiscLib::Vector< std::pair< float, float > > *params,
		size_t *uextent, size_t *vextent);
	void InBitmap(const std::pair< float, float > &param,
		float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		size_t uextent, size_t vextent,
		std::pair< int, int > *inBmp) const;
	void WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, bool *uwrap, bool *vwrap) const;
	void WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< int > *componentImg,
		MiscLib::Vector< std::pair< int, size_t > > *labels) const;
	void SetExtent(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		const MiscLib::Vector< int > &componentsImg, size_t uextent,
		size_t vextent, float epsilon, int label);
	bool InSpace(size_t u, size_t v, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, Vec3f *p, Vec3f *n) const;
	const Torus &Internal() const { return m_torus; }

private:
	template< class IteratorT >
	void ParametersImpl(IteratorT begin, IteratorT end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const;

private:
	Torus m_torus;
	ParametrizationType m_parametrization;
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
