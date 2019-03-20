#ifndef GfxTL__SLIDINGMIDPOINTSPLITTINGKDTREESTRATEGY_HEADER__
#define GfxTL__SLIDINGMIDPOINTSPLITTINGKDTREESTRATEGY_HEADER__
#include <GfxTL/ScalarTypeDeferer.h>
#include <GfxTL/ScalarTypeConversion.h>

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct SlidingMidpointSplittingKdTreeStrategy
	{
		typedef typename InheritedStrategyT::value_type value_type;

		struct CellData
		: public InheritedStrategyT::CellData
		{};

		template< class BaseT >
		class StrategyBase
		: public InheritedStrategyT::template StrategyBase< BaseT >
		{
		public:
			typedef typename InheritedStrategyT::template StrategyBase< BaseT >
				BaseType;
			typedef typename BaseType::CellType CellType;
			typedef typename ScalarTypeDeferer< value_type >::ScalarType
				ScalarType;
			typedef typename ScalarTypeConversion< ScalarType, ScalarType >::
				DifferenceType DiffScalarType;

		protected:
			template< class BuildInformationT >
			void ComputeSplit(const BuildInformationT &bi, CellType *cell)
			{
				DiffScalarType *cellDiff = new DiffScalarType[BaseType::m_dim];
				Sub(bi.CellBBox()[1], bi.CellBBox()[0], &cellDiff);
				DiffScalarType *diff = new DiffScalarType[BaseType::m_dim];
				Sub(bi.BBox()[1], bi.BBox()[0], &diff);
				unsigned int axis = 0;
				DiffScalarType length = cellDiff[0];
				DiffScalarType spread = diff[0];
				for(unsigned int j = 1; j < BaseType::m_dim; ++j)
				{
					if((cellDiff[j] > length && spread < 2 * diff[j])
						|| (cellDiff[j] == length && diff[j] > spread)
						|| diff[j] > 2 * spread)
					{
						axis = j;
						length = cellDiff[j];
						spread = diff[j];
					}
				}
				cell->SplitAxis() = axis;
				cell->SplitValue() =
					(bi.CellBBox()[0][axis] + bi.CellBBox()[1][axis]) / 2;
				if(cell->SplitValue() < bi.BBox()[0][axis]
					|| cell->SplitValue() > bi.BBox()[1][axis])
					cell->SplitValue() = (bi.BBox()[1][axis] + bi.BBox()[0][axis]) / 2;
				delete[] cellDiff;
				delete[] diff;
			}

			template< class BuildInformationT >
			void ComputeSplit(unsigned int axis, const BuildInformationT &bi,
				CellType *cell)
			{
				DiffScalarType *cellDiff = new DiffScalarType[BaseType::m_dim];
				Sub(bi.CellBBox()[1], bi.CellBBox()[0], &cellDiff);
				DiffScalarType *diff = new DiffScalarType[BaseType::m_dim];
				Sub(bi.BBox()[1], bi.BBox()[0], &diff);
				cell->SplitAxis() = axis;
				cell->SplitValue() =
					(bi.CellBBox()[0][axis] + bi.CellBBox()[1][axis]) / 2;
				if(cell->SplitValue() < bi.BBox()[0][axis])
					cell->SplitValue() = bi.BBox()[0][axis];
				else if(cell->SplitValue() > bi.BBox()[1][axis])
					cell->SplitValue() = bi.BBox()[1][axis];
				delete[] cellDiff;
				delete[] diff;
			}

			template< class BuildInformationT >
			bool AlternateSplit(const BuildInformationT &bi, CellType *cell)
			{
				unsigned int axis = (cell->SplitAxis() + 1) % BaseType::m_dim;
				cell->SplitAxis() = axis;
				cell->SplitValue() =
					(bi.CellBBox()[1][axis] + bi.CellBBox()[0][axis]) / 2;
				if(cell->SplitValue() < bi.BBox()[0][axis])
					cell->SplitValue() = bi.BBox()[0][axis];
				else if(cell->SplitValue() > bi.BBox()[1][axis])
					cell->SplitValue() = bi.BBox()[1][axis];
				return true;
			}
		};
	};
};

#endif
