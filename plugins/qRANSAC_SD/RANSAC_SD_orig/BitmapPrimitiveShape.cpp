#include "BitmapPrimitiveShape.h"
#include "Bitmap.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "IndexIterator.h"
#include <MiscLib/Performance.h>
#include <float.h>
using namespace MiscLib;

void BitmapPrimitiveShape::PreWrapBitmap(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, float epsilon,
	size_t uextent, size_t vextent, MiscLib::Vector< char > *bmp) const
{
	// the default case is: do nothing
}

void BitmapPrimitiveShape::WrapComponents(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, size_t uextent, size_t vextent,
	MiscLib::Vector< int > *componentImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels) const
{
	// default: do nothing
}

bool BitmapPrimitiveShape::Init(bool binary, std::istream *i)
{
	if(binary)
	{
		GfxTL::AABox< GfxTL::Vector2Df > bbox;
		size_t uextent, vextent;
		// read number of components
		size_t size;
		i->read((char *)&size, sizeof(size));
		if(size)
		{
			// read bbox
			i->read((char *)&bbox, sizeof(bbox));
			// read uextent and vextent
			i->read((char *)&uextent, sizeof(uextent));
			i->read((char *)&vextent, sizeof(vextent));
			// read every component
			for(size_t j = 0; j < size; ++j)
			{
				// read number of polys in component
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
		GfxTL::AABox< GfxTL::Vector2Df > bbox;
		size_t uextent, vextent;
		// read number of components
		size_t size;
		(*i) >> size;
		if(size)
		{
			// read bbox
			(*i) >> bbox.Min()[0] >> bbox.Max()[0]
				>> bbox.Min()[1] >> bbox.Max()[1];
			// read uextent and vextent
			(*i) >> uextent >> vextent;
			// read every component
			for(size_t j = 0; j < size; ++j)
			{
				// read number of polys in component
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

size_t BitmapPrimitiveShape::AllConnectedComponents(const PointCloud &pc, float epsilon,
	BitmapInfo& bitmapInfo, MiscLib::Vector< size_t > *indices, MiscLib::Vector< int >& componentsImg, 
		MiscLib::Vector< std::pair< int, size_t > >& labels, bool doFiltering )
{
	// first find the extent in the parametrization
	// but remember the parametrized points for projection into a bitmap
	size_t size = indices->size();
	if(!size)
		return 0;

	// set up bitmap
	MiscLib::Vector< std::pair< float, float > > extParams;

	BuildBitmap(pc, &epsilon, indices->begin(), indices->end(), &bitmapInfo.params,
		&bitmapInfo.bbox, &bitmapInfo.bitmap, &bitmapInfo.uextent, &bitmapInfo.vextent, &bitmapInfo.bmpIdx);

	/*static int fname_int = 0;
	std::ostringstream fn;
	fn << "bitmapImg" << fname_int++ << ".txt";
	std::ofstream file;
	file.open(fn.str().c_str(), std::ios::out);
	for(size_t j = 0; j < vextent; ++j)
	{
		for(size_t i = 0; i < uextent; ++i)
			file << bitmap[j * uextent + i];
		file << std::endl;
	}
	file.close();*/

	// do a wrapping by copying pixels
	PreWrapBitmap(bitmapInfo.bbox, epsilon, bitmapInfo.uextent, bitmapInfo.vextent, &bitmapInfo.bitmap);

	MiscLib::Vector< char > tempBmp(bitmapInfo.bitmap.size()); // temporary bitmap object
	bool uwrap, vwrap;
	WrapBitmap(bitmapInfo.bbox, epsilon, &uwrap, &vwrap);

	if (doFiltering)
	{
		// closing
		DilateCross(bitmapInfo.bitmap, bitmapInfo.uextent, bitmapInfo.vextent, uwrap, vwrap, &tempBmp);
		ErodeCross(tempBmp, bitmapInfo.uextent, bitmapInfo.vextent, uwrap, vwrap, &bitmapInfo.bitmap);
		// opening
		//ErodeCross(bitmap, uextent, vextent, uwrap, vwrap, &tempBmp);
		//DilateCross(tempBmp, uextent, vextent, uwrap, vwrap, &bitmap);
	}

	Components(bitmapInfo.bitmap, bitmapInfo.uextent, bitmapInfo.vextent, uwrap, vwrap, &componentsImg,
		&labels);
	if(labels.size() <= 1) // found no connected component!
	{
		return 0; // associate no points with this shape
	}

	WrapComponents(bitmapInfo.bbox, epsilon, bitmapInfo.uextent, bitmapInfo.vextent, &componentsImg, &labels);

	return labels.size();
}

size_t BitmapPrimitiveShape::ConnectedComponent(
	const PointCloud &pc, float epsilon,
	MiscLib::Vector< size_t > *indices, bool doFiltering, float* borderRatio )
{
	MiscLib::Vector< int > componentsImg;
	MiscLib::Vector< std::pair< int, size_t > > labels;

	BitmapInfo bitmapInfo;
	if( AllConnectedComponents( pc, epsilon, bitmapInfo, indices, componentsImg, labels, doFiltering ) <= 1 )
		return 0;

	size_t size = indices->size();
	MiscLib::Vector< size_t >::iterator begin = indices->begin();

	// find the largest component
	size_t maxComp = 1;
	for(size_t i = 2; i < labels.size(); ++i)
		if(labels[maxComp].second < labels[i].second)
			maxComp = i;

	GfxTL::AABox< GfxTL::Vector2Df > bbox;
	bbox.Min() = GfxTL::Vector2Df(
			std::numeric_limits< float >::infinity(),
			std::numeric_limits< float >::infinity());
	bbox.Max() = -bbox.Min();
	// compute bbox and update indices
	size_t offset = 0;
	for(size_t i = 0; i < size; ++i)
	{
		if(componentsImg[bitmapInfo.bmpIdx[i]] == labels[maxComp].first)
		{
			std::swap(begin[offset], begin[i]);
			offset++;
			// update bounding box
			if(bbox.Min()[0] > bitmapInfo.params[i].first)
				bbox.Min()[0] = bitmapInfo.params[i].first;
			if(bbox.Max()[0] < bitmapInfo.params[i].first)
				bbox.Max()[0] = bitmapInfo.params[i].first;
			if(bbox.Min()[1] > bitmapInfo.params[i].second)
				bbox.Min()[1] = bitmapInfo.params[i].second;
			if(bbox.Max()[1] < bitmapInfo.params[i].second)
				bbox.Max()[1] = bitmapInfo.params[i].second;
		}
	}

	// ratio between border and connected-comp size should be calculated if borderRatio is a valid pointer
	if( borderRatio )
	{
		int borderPixels = 0;
		int maxLabel = labels[maxComp].first;
		int row = bitmapInfo.uextent;
		int pos = 0;
		char numNeighbours = 0;
		int ccSize = 0;

		// test neightbourhood for all bitmappixels that are not marginal
		for( size_t v = 1; v < bitmapInfo.vextent-1; ++v )
		{
			for( size_t u = 1; u < bitmapInfo.uextent-1; ++u )
			{
				pos = row + u;

				if( componentsImg[pos] == maxLabel )
				{
					ccSize++;
					numNeighbours = bitmapInfo.bitmap[pos-1] + bitmapInfo.bitmap[pos+1] + 
								bitmapInfo.bitmap[pos-bitmapInfo.uextent-1] + bitmapInfo.bitmap[pos-bitmapInfo.uextent+1] +
								bitmapInfo.bitmap[pos+bitmapInfo.uextent-1] + bitmapInfo.bitmap[pos+bitmapInfo.uextent+1] +
								bitmapInfo.bitmap[pos-bitmapInfo.uextent] + bitmapInfo.bitmap[pos+bitmapInfo.uextent];

					if( (int)numNeighbours != 8 )
						++borderPixels;
				}
			}
			row += bitmapInfo.uextent;
		}

		// check left/right margins
		row = bitmapInfo.uextent;
		for( size_t v = 1; v < bitmapInfo.vextent-1; ++v )
		{
			ccSize++;
			if( componentsImg[row] == maxLabel )
				++borderPixels;

			ccSize++;
			if( componentsImg[row+bitmapInfo.uextent-1] == maxLabel )
				++borderPixels;

			row += bitmapInfo.uextent;
		}

		// check top/bottom margins
		row = ( bitmapInfo.vextent-1 ) * bitmapInfo.uextent;
		for( size_t u = 0; u < bitmapInfo.uextent; ++u )
		{
			ccSize++;
			if( componentsImg[u] == maxLabel )
				++borderPixels;

			ccSize++;
			if( componentsImg[row + u] == maxLabel )
				++borderPixels;
		}
		
		*borderRatio = static_cast<float>( borderPixels ) / static_cast<float>( ccSize );
	}

	m_extBbox = bbox;
	return offset;
}

void BitmapPrimitiveShape::TrimmingPolygons(const PointCloud &pc,
	float epsilon, size_t begin, size_t end,
	std::deque< ComponentPolygons > *polys) const
{
	GfxTL::AABox< GfxTL::Vector2Df > bbox;
	size_t uextent, vextent;
	BuildPolygons(pc, epsilon, begin, end, &bbox, &uextent, &vextent, polys);
}

void BitmapPrimitiveShape::GenerateBitmapPoints(const PointCloud &pc,
	float epsilon, size_t begin, size_t end, PointCloud *bmpPc) const
{
	// constructing the bitmap is similar to ConnectedComponent
	MiscLib::Vector< std::pair< float, float > > params, extParams;
	MiscLib::Vector< char > bitmap;
	MiscLib::Vector< size_t > bmpIdx;
	GfxTL::AABox< GfxTL::Vector2Df > bbox;
	size_t uextent, vextent;
	BuildBitmap(pc, &epsilon, IndexIterator(begin), IndexIterator(end), &params,
		&bbox, &bitmap, &uextent, &vextent, &bmpIdx);
	m_extBbox = bbox;

	//// do closing
	//MiscLib::Vector< char > tempBmp(bitmap.size());
	//DilateSquare(bitmap, uextent, vextent, false, false, &tempBmp);
	//ErodeSquare(tempBmp, uextent, vextent, false, false, &bitmap);
	//// do opening
	//ErodeSquare(bitmap, uextent, vextent, false, false, &tempBmp);
	//DilateSquare(tempBmp, uextent, vextent, false, false, &bitmap);

	if(bmpPc)
	{
		size_t count = 0;
		for(size_t i = 0; i < vextent * uextent; ++i)
		{
			// create point on surface
			if(!bitmap[i])
				continue;
			if(count >= bmpPc->size())
				bmpPc->resize(2 * count + 1);
			if(InSpace(i % uextent, i / uextent, epsilon, bbox, uextent,
				vextent, &(*bmpPc)[count].pos, &(*bmpPc)[count].normal))
				++count;
		}
		bmpPc->resize(count);
	}
}

void BitmapPrimitiveShape::BuildPolygons(const PointCloud &pc, float epsilon,
	size_t begin, size_t end, GfxTL::AABox< GfxTL::Vector2Df > *bbox,
	size_t *uextent, size_t *vextent,
	std::deque< ComponentPolygons > *polys) const
{
	// curves are extracted in the following way:
	// first the bitmap is constructed
	// then connected components are found
	// for each component the curves are found

	// constructing the bitmap is similar to ConnectedComponent
	// -> use the same code
	MiscLib::Vector< std::pair< float, float > > params;
	MiscLib::Vector< char > bitmap;
	MiscLib::Vector< size_t > bmpIdx;
	BuildBitmap(pc, &epsilon, IndexIterator(begin), IndexIterator(end), &params,
		bbox, &bitmap, uextent, vextent, &bmpIdx);

	// do closing
	MiscLib::Vector< char > tempBmp(bitmap.size());
	DilateCross(bitmap, *uextent, *vextent, false, false, &tempBmp);
	ErodeCross(tempBmp, *uextent, *vextent, false, false, &bitmap);

	// find connected components
	MiscLib::Vector< int > componentsImg;
	MiscLib::Vector< std::pair< int, size_t > > labels;
	Components(bitmap, *uextent, *vextent, false, false, &componentsImg,
		&labels);
	if(labels.size() <= 1) // found no connected component!
		return;

	// for each component find all polygons
	for(size_t i = 1; i < labels.size(); ++i)
	{
		polys->resize(polys->size() + 1);
		ComponentLoops(componentsImg, *uextent, *vextent, labels[i].first,
			false, false, &(*polys)[polys->size() - 1]);
	}
}
