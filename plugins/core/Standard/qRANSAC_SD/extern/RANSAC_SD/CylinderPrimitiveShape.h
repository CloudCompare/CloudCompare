#ifndef CYLINDERPRIMITIVESHAPE_HEADER
#define CYLINDERPRIMITIVESHAPE_HEADER
#include "BitmapPrimitiveShape.h"
#include "Cylinder.h"
#include "LevMarFunc.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE CylinderPrimitiveShape
: public BitmapPrimitiveShape
{
public:
	CylinderPrimitiveShape();
	size_t Identifier() const override;
	unsigned int RequiredSamples() const  override { return Cylinder::RequiredSamples; }
	CylinderPrimitiveShape(const Cylinder &cylinder, float minRadius = -std::numeric_limits<float>::infinity(), float maxRadius = std::numeric_limits<float>::infinity(), float maxLength = std::numeric_limits<float>::infinity());
	PrimitiveShape *Clone() const override;
	bool Init(const Vec3f &pointA, const Vec3f &pointB,
		const Vec3f &normalA, const Vec3f &normalB);
	bool Init(bool binary, std::istream *i) { return BitmapPrimitiveShape::Init(binary, i); }
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
	virtual void Serialize(float* array) const  override { m_cylinder.Serialize(array); }
	virtual size_t SerializedFloatSize() const  override { return m_cylinder.SerializedFloatSize(); }
	void Transform(float scale, const Vec3f &translate) override;
	void Transform(const GfxTL::MatrixXX< 3, 3, float > &rot,
		const GfxTL::Vector3Df &trans);
	void Visit(PrimitiveShapeVisitor *visitor) const override;
	void SuggestSimplifications(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end, float distThresh,
		MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const override;
	bool Similar(float tolerance, const CylinderPrimitiveShape &shape) const;
	const Cylinder &Internal() const { return m_cylinder; }
	float Height() const;
	float MinHeight() const;
	float MaxHeight() const;

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
	void PreWrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< char > *bmp) const override;
	void SetExtent(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		const MiscLib::Vector< int > &componentsImg, size_t uextent,
		size_t vextent, float epsilon, int label);
	bool InSpace(size_t u, size_t v, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, Vec3f *p, Vec3f *n) const override;
	bool CheckGeneratedShapeWithinLimits(const PointCloud& pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end) override
	{
		if (m_cylinder.Radius() <= m_maxRadius && Height() <= m_maxLength)
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
	Cylinder m_cylinder;
	bool m_clip;
	float m_minPhi;
	float m_maxPhi;
	float m_minRadius;
	float m_maxRadius;
	float m_maxLength;
};

template< class IteratorT >
void CylinderPrimitiveShape::ParametersImpl(IteratorT begin, IteratorT end,
	MiscLib::Vector< std::pair< float, float > > *bmpParams) const
{
	bmpParams->resize(end - begin);
	size_t j = 0;
	for(IteratorT i = begin; i != end; ++i, ++j)
	{
		m_cylinder.Parameters(*i, &(*bmpParams)[j]);
		(*bmpParams)[j].second = (*bmpParams)[j].second * m_cylinder.Radius();
	}
}

class DLL_LINKAGE CylinderLevMarFunc
: public LevMarFunc< float >
{
public:
	CylinderLevMarFunc(const Cylinder &cy)
	: m_cylinder(cy)
	{}

	float operator()(const float *x) const
	{
		return m_cylinder.SignedDistance(*((const Vec3f *)x));
	}

	void operator()(const float *x, float *gradient) const
	{
		m_cylinder.Normal(*((const Vec3f *)x), (Vec3f *)gradient);
	}

private:
	Cylinder m_cylinder;
};

#endif
