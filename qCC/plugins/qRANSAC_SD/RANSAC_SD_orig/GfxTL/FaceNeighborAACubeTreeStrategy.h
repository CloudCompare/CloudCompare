#ifndef GfxTL__FACENEIGHBORAACUBETREESTRATEGY_HEADER__
#define GfxTL__FACENEIGHBORAACUBETREESTRATEGY_HEADER__

namespace GfxTL
{

// requires that InheritedStrategyT supports StrategyBase::CellParent() and CellType::Center()
template< class InheritedStrategyT >
struct FaceNeighborAACubeTreeStrategy
{
	typedef typename InheritedStrategyT::value_type value_type;

	class CellData
	: public InheritedStrategyT::CellData
	{};

	template< class BaseT >
	class StrategyBase
	: public InheritedStrategyT::template StrategyBase< BaseT >
	{
	public:
		typedef typename InheritedStrategyT::template StrategyBase< BaseT > BaseType;
		typedef typename BaseType::CellType CellType;
		enum { Dim = CellType::Dim };

		// returns the face neighbor along axis in direction (0 = left, 1 = right)
		// returns in level the level of the neighbor relative to the input cell (upwards)
		// returns NULL if the cell is on the boundary
		// This function does not return all face neighbors in the given direction
		// in the case that the neighbors live on a deeper level than the input cell.
		// In such a case the face neighbor on the same level as the input cell is
		// return. All face neighbors on that side are children of the returned cell.
		const CellType *CellFaceNeighbor(const CellType &cell,
			unsigned int axis, unsigned int dir, size_t *level) const
		{
			const CellType *parent = BaseType::CellParent(cell);
			if(!parent) return NULL; // root cell does not have any face neighbors
			unsigned int childRelation = CellChildRelation(cell, *parent);
			// check if face neighbor is another child of parent
			// this is the case if dir is opposite to the childRelation
			if(((childRelation >> (Dim - 1 - axis)) & 1) ^ (dir & 1)) // cell on opposite side?
			{
				unsigned int faceNeighborRelation
					= childRelation ^ (1 << (Dim - 1 - axis)); // flip the respective bit
				if(BaseType::ExistChild(*parent, faceNeighborRelation))
				{
					*level = 0;
					return &(*parent)[faceNeighborRelation];
				}
				*level = 1;
				return parent; // degenerate case -> cell does not have an actual face neighbor
			}
			// otherwise the face neighbor is a neighbor of the parent
			size_t l;
			const CellType *n = CellFaceNeighbor(*parent, axis, dir, &l);
			if(!n) return NULL;
			if(l > 0) // if the face neighbor of the parent does not live on the same level as the parent
				// we are unable to find any deeper face neighbor
			{
				*level = l + 1;
				return n;
			}
			// otherwise try to find the child of n that is our face neighbor
			// our face neighbor is the child that has the opposite side than us on axis
			unsigned int faceNeighborRelation
				= childRelation ^ (1 << (Dim - 1 - axis)); // flip the respective bit
			if(BaseType::ExistChild(*n, faceNeighborRelation))
			{
				*level = 0;
				return &(*n)[faceNeighborRelation];
			}
			*level = 1;
			return n;
		}

		// non-const version
		CellType *CellFaceNeighbor(CellType &cell,
			unsigned int axis, unsigned int dir, size_t *level)
		{
			return const_cast< CellType * >(CellFaceNeighbor(cell, axis, dir, level));
		}

		unsigned int CellChildRelation(const CellType &cell, const CellType &parent) const
		{
			unsigned int childRelation = 0;
			for(unsigned int i = 0; i < Dim; ++i)
			{
				if(cell.Center()[i] > parent.Center()[i])
					childRelation |= 1 << (Dim - 1 - i);
			}
			return childRelation;
		}
	};
};

};

#endif
