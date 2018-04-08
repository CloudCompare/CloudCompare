#ifndef SCOREPRIMITIVESHAPEVISITOR_HEADER
#define SCOREPRIMITIVESHAPEVISITOR_HEADER
#include "PrimitiveShapeVisitor.h"
#include <MiscLib/RefCounted.h>
#include <MiscLib/RefCountPtr.h>
#include <MiscLib/NoShrinkVector.h>
#include "PlanePrimitiveShape.h"
#include "SpherePrimitiveShape.h"
#include "CylinderPrimitiveShape.h"
#include "ConePrimitiveShape.h"
#include "TorusPrimitiveShape.h"

template< class PointCompT, class OctreeT >
class ScorePrimitiveShapeVisitorImpl
: public PrimitiveShapeVisitor
{
public:
	typedef PointCompT PointCompatibilityFunc;
	typedef OctreeT OctreeType;
	typedef MiscLib::RefCounted< MiscLib::Vector< size_t > > IndicesType;
	ScorePrimitiveShapeVisitorImpl(float distThresh,
		float normalThresh) : m_pointComp(distThresh, normalThresh) {}
	void SetOctree(const OctreeT &oct) { m_oct = &oct; }
	const OctreeT &GetOctree() const { return *m_oct; }
	IndicesType *GetIndices() { return m_indices; }
	void SetIndices(IndicesType *indices) { m_indices = indices; }
	void SetShapeIndex(const MiscLib::Vector< int > &shapeIndex)
	{ m_shapeIndex = &shapeIndex; }
	const MiscLib::Vector< int > &GetShapeIndex() { return *m_shapeIndex; }
	template< class ShapeT >
	void Visit(const ShapeT &primShape)
	{
		//if(!m_indices)
		//	m_indices.New< IndicesType >();
		//m_upperBound = 0;
		//m_sampled = 0;
		m_oct->Score(primShape.Internal(), /*1,*/ this);
	}
	template< class ShapeT, class OctT >
	void operator()(const ShapeT &shape, const OctT &oct, size_t i)
	{
		if((*m_shapeIndex)[i] != -1)
			return;
		if(m_pointComp(shape, oct, i))
			m_indices->push_back(i);
	}
	float Epsilon() const { return m_pointComp.DistanceThresh(); }
	//size_t &UpperBound() { return m_upperBound; }
	//size_t &SampledPoints() { return m_sampled; }
	const PointCompatibilityFunc &PointCompFunc() const { return m_pointComp; }

private:
	PointCompatibilityFunc m_pointComp;
	const OctreeT *m_oct;
	/*MiscLib::RefCountPtr<*/ IndicesType /*>*/ *m_indices;
	const MiscLib::Vector< int > *m_shapeIndex;
	//size_t m_upperBound;
	//size_t m_sampled;
};

template< class PointCompT, class OctreeT >
class ScorePrimitiveShapeVisitor
: public PrimitiveShapeVisitorShell< ScorePrimitiveShapeVisitorImpl< PointCompT, OctreeT > >
{
public:
	ScorePrimitiveShapeVisitor(float distThresh,
		float normalThresh) : PrimitiveShapeVisitorShell<
			ScorePrimitiveShapeVisitorImpl< PointCompT, OctreeT > >(
				distThresh, normalThresh) {}
};

#endif
