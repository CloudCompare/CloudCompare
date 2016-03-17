#ifndef BITMAPPRIMITIVESHAPE_HEADER
#define BITMAPPRIMITIVESHAPE_HEADER
#include "BasePrimitiveShape.h"
#include <GfxTL/AABox.h>
#include <MiscLib/Vector.h>
#include <algorithm>
#include <istream>
#include <MiscLib/Performance.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/IndexedIterator.h>
#include "IndexIterator.h"
#include <MiscLib/Pair.h>
#ifdef DOPARALLEL
#include <omp.h>
#endif

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

struct BitmapInfo
{
	MiscLib::Vector< std::pair< float, float > > params;
	MiscLib::Vector< char > bitmap;
	GfxTL::AABox< GfxTL::Vector2Df > bbox;
	MiscLib::Vector< size_t > bmpIdx;
	size_t uextent, vextent;
};

class DLL_LINKAGE BitmapPrimitiveShape
: public BasePrimitiveShape
{
public:
	bool Init(bool binary, std::istream *i);
	size_t ConnectedComponent(const PointCloud &pc, float epsilon,
		MiscLib::Vector< size_t > *indices, bool doFiltering = true, float* borderRatio = 0 );
	size_t AllConnectedComponents(const PointCloud &pc, float epsilon, BitmapInfo& bitmapInfo,
		MiscLib::Vector< size_t > *indices, MiscLib::Vector< int >& componentsImg, 
		MiscLib::Vector< std::pair< int, size_t > >& labels, bool doFiltering = true );
	void TrimmingPolygons(const PointCloud &pc, float epsilon,
		size_t begin, size_t end,
		std::deque< ComponentPolygons > *polys) const;
	void GenerateBitmapPoints(const PointCloud &pc, float epsilon,
		size_t begin, size_t end, PointCloud *bmpPc) const;

public:
	virtual void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const = 0;
	virtual bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const = 0;
	virtual void Parameters(GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
			PointCloud::const_iterator > begin,
		GfxTL::IndexedIterator< MiscLib::Vector< size_t >::iterator,
			PointCloud::const_iterator > end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const = 0;
	virtual void Parameters(GfxTL::IndexedIterator< IndexIterator,
			PointCloud::const_iterator > begin,
		GfxTL::IndexedIterator< IndexIterator,
			PointCloud::const_iterator > end,
		MiscLib::Vector< std::pair< float, float > > *bmpParams) const = 0;
	virtual void BitmapExtent(float epsilon,
		GfxTL::AABox< GfxTL::Vector2Df > *bbox,
		MiscLib::Vector< std::pair< float, float > > *params,
		size_t *uextent, size_t *vextent) = 0;
	virtual void InBitmap(const std::pair< float, float > &param,
		float epsilon, const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		size_t uextent, size_t vextent,
		std::pair< int, int > *inBmp) const = 0;
	virtual void PreWrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< char > *bmp) const;
	virtual void WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, bool *uwrap, bool *vwrap) const = 0;
	virtual void WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
		float epsilon, size_t uextent, size_t vextent,
		MiscLib::Vector< int > *componentImg,
		MiscLib::Vector< std::pair< int, size_t > > *labels) const;
	virtual bool InSpace(size_t u, size_t v, float epsilon,
		const GfxTL::AABox< GfxTL::Vector2Df > &bbox, size_t uextent,
		size_t vextent, Vec3f *p, Vec3f *n) const = 0;
	template< class IteratorT >
	void BuildBitmap(const PointCloud &pc, float *epsilon, IteratorT begin,
		IteratorT end, MiscLib::Vector< std::pair< float, float > > *params,
		GfxTL::AABox< GfxTL::Vector2Df > *bbox,
		MiscLib::Vector< char > *bitmap, size_t *uextent, size_t *vextent,
		MiscLib::Vector< size_t > *bmpIdx) const;
	template< class IteratorT >
	void BuildBitmap(const PointCloud &pc, float *epsilon, IteratorT begin,
		IteratorT end, MiscLib::Vector< std::pair< float, float > > *params,
		GfxTL::AABox< GfxTL::Vector2Df > *bbox,
		MiscLib::Vector< char > *bitmap, size_t *uextent, size_t *vextent,
		MiscLib::Vector< size_t > *bmpIdx, size_t border) const;
	void BuildPolygons(const PointCloud &pc, float epsilon, size_t begin,
		size_t end, GfxTL::AABox< GfxTL::Vector2Df > *bbox,
		size_t *uextent, size_t *vextent,
		std::deque< ComponentPolygons > *polys) const;

protected:
	mutable GfxTL::AABox< GfxTL::Vector2Df > m_extBbox;
};

template< class IteratorT >
void BitmapPrimitiveShape::BuildBitmap(const PointCloud &pc, float *epsilon,
	IteratorT begin, IteratorT end, MiscLib::Vector< std::pair< float, float > > *params,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox, MiscLib::Vector< char > *bitmap,
	size_t *uextent, size_t *vextent, MiscLib::Vector< size_t > *bmpIdx) const
{
	int size = end - begin;
	params->resize(size);
	// compute parameters and extent
	Parameters(GfxTL::IndexIterate(begin, pc.begin()),
		GfxTL::IndexIterate(end, pc.begin()), params);
	bbox->Min() = GfxTL::Vector2Df(std::numeric_limits< float >::infinity(),
		std::numeric_limits< float >::infinity());
	bbox->Max() = -bbox->Min();
	for(size_t i = 0; i < (size_t)size; ++i)
	{
		if((*params)[i].first < bbox->Min()[0])
			bbox->Min()[0] = (*params)[i].first;
		if((*params)[i].first > bbox->Max()[0])
			bbox->Max()[0] = (*params)[i].first;
		if((*params)[i].second < bbox->Min()[1])
			bbox->Min()[1] = (*params)[i].second;
		if((*params)[i].second > bbox->Max()[1])
			bbox->Max()[1] = (*params)[i].second;
	}
	// bbox gives the bounding box in parameter space
	// we can now set up the bitmap
	const_cast< BitmapPrimitiveShape * >(this)->BitmapExtent(*epsilon, bbox, params,
		uextent, vextent);
	if(*uextent < 2)
		*uextent = 2;
	if(*vextent < 2)
		*vextent = 2;
	bitmap->resize((*uextent) * (*vextent));
	std::fill(bitmap->begin(), bitmap->end(), false);
	// set all true bits in bitmap
	bmpIdx->resize(params->size());
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static)
#endif
	for(int i = 0; i < size; ++i)
	{
		std::pair< int, int > bmpParam;
		InBitmap((*params)[i], *epsilon, *bbox, *uextent, *vextent, &bmpParam);
		// clamp bitmap coords
		bmpParam.first = GfxTL::Math< int >::Clamp(bmpParam.first, 0, *uextent - 1);
		bmpParam.second = GfxTL::Math< int >::Clamp(bmpParam.second, 0, *vextent - 1);
		(*bitmap)[(*bmpIdx)[i] = bmpParam.first	+ bmpParam.second * (*uextent)] = true;
	}
}

template< class IteratorT >
void BitmapPrimitiveShape::BuildBitmap(const PointCloud &pc, float *epsilon,
	IteratorT begin, IteratorT end, MiscLib::Vector< std::pair< float, float > > *params,
	GfxTL::AABox< GfxTL::Vector2Df > *bbox, MiscLib::Vector< char > *bitmap,
	size_t *uextent, size_t *vextent, MiscLib::Vector< size_t > *bmpIdx,
	size_t border) const
{
	params->resize(end - begin);
	// compute parameters and extent
	Parameters(pc[*begin].pos, &(*params)[0]);
	bbox->Min() = bbox->Max() = GfxTL::Vector2Df((*params)[0].first,
		(*params)[0].second);
	size_t j = 1;
	IteratorT i = begin;
	for(++i; i != end; ++i, ++j)
	{
		Parameters(pc[*i].pos, &(*params)[j]);
		if(bbox->Min()[0] > (*params)[j].first)
			bbox->Min()[0] = (*params)[j].first;
		else if(bbox->Max()[0] < (*params)[j].first)
			bbox->Max()[0] = (*params)[j].first;
		if(bbox->Min()[1] > (*params)[j].second)
			bbox->Min()[1] = (*params)[j].second;
		else if(bbox->Max()[1] < (*params)[j].second)
			bbox->Max()[1] = (*params)[j].second;
	}
	// bbox gives the bounding box in parameter space
	// we can now set up the bitmap
	const_cast< BitmapPrimitiveShape * >(this)->BitmapExtent(*epsilon, bbox, params,
		uextent, vextent);
	if(*uextent < 2)
		*uextent = 2;
	if(*vextent < 2)
		*vextent = 2;
	bitmap->resize(((*uextent) + 2 * border) * ((*vextent) + 2 * border));
	std::fill(bitmap->begin(), bitmap->end(), false);
	// set all true bits in bitmap
	bmpIdx->resize(params->size());
	size_t lineWidth = (*uextent) + 2 * border;
	for(size_t i = 0; i < params->size(); ++i)
	{
		std::pair< int, int > bmpParam;
		InBitmap((*params)[i], *epsilon, *bbox, *uextent, *vextent, &bmpParam);
		// clamp bitmap coords
		bmpParam.first = GfxTL::Math< int >::Clamp(bmpParam.first, 0, *uextent - 1);
		bmpParam.second = GfxTL::Math< int >::Clamp(bmpParam.second, 0, *vextent - 1);
		(*bitmap)[(*bmpIdx)[i] = bmpParam.first + border
			+ (bmpParam.second + border) * lineWidth] = true;
	}
}

#endif
