#ifndef GfxTL__MAXINTERVALSPLITTINGKDTREESTRATEGY_HEADER__
#define GfxTL__MAXINTERVALSPLITTINGKDTREESTRATEGY_HEADER__
#include <GfxTL/ScalarTypeDeferer.h>
#include <GfxTL/ScalarTypeConversion.h>

namespace GfxTL
{
	template< class InheritedStrategyT >
	struct MaxIntervalSplittingKdTreeStrategy
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
				DiffScalarType *diff = new DiffScalarType[BaseType::m_dim];
				this->Sub(bi.BBox()[1], bi.BBox()[0], &diff);
				unsigned int axis = 0;
				DiffScalarType length = diff[0];
				for(unsigned int j = 1; j < BaseType::m_dim; ++j)
				{
					if(diff[j] > length)
					{
						axis = j;
						length = diff[j];
					}
				}
				cell->SplitAxis() = axis;
				cell->SplitValue()  =
					(bi.BBox()[0][axis] + bi.BBox()[1][axis]) / 2;
				delete[] diff;
			}

			template< class BuildInformationT >
			void ComputeSplit(unsigned int axis, const BuildInformationT &bi,
				CellType *cell)
			{ 
				DiffScalarType *diff = new DiffScalarType[BaseType::m_dim];
				Sub(bi.BBox()[1], bi.BBox()[0], &diff);
				cell->SplitAxis() = axis;
				cell->SplitValue()  =
					(bi.BBox()[0][axis] + bi.BBox()[1][axis]) / 2;
				delete[] diff;
			}

			template< class BuildInformationT >
			bool AlternateSplit(const BuildInformationT &bi, CellType *cell)
			{
				unsigned int axis = (cell->SplitAxis() + 1) % BaseType::m_dim;
				cell->SplitAxis() = axis;
				cell->SplitValue() =
					(bi.BBox()[1][axis] + bi.BBox()[0][axis]) / 2;
				return true;
			}
		};
	};
};

#endif
